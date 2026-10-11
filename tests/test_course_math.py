import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from course_sim.cloud_format import OUTPUT, decode, encode, xt32_points
from course_sim.imu_attitude import Attitude
from course_slam.evaluate import evaluate, load_trajectory
from course_slam.transforms import assign_pose, pose_matrix, covariance_at_base
from course_navigation.map_projection import Grid
from course_navigation.velocity_adapter import VelocityGate
from course_navigation.metrics import RunMetrics


def beam_cloud():
    pts=np.zeros(32,dtype=OUTPUT)
    angle=np.radians(np.arange(-16,16))
    pts['x'],pts['z']=10*np.cos(angle),10*np.sin(angle)
    return encode(pts,Header(frame_id='lidar_link'))


def test_xt32_rings_survive_noise_dropout_and_snapshot_time():
    cloud=beam_cloud()
    full=xt32_points(cloud)
    np.testing.assert_array_equal(full['ring'],np.arange(32))
    seed=42
    noisy=xt32_points(cloud,.1,.4,np.random.default_rng(seed))
    repeat=xt32_points(cloud,.1,.4,np.random.default_rng(seed))
    np.testing.assert_array_equal(noisy,repeat)
    assert 0<len(noisy)<32
    elevations=np.degrees(np.arctan2(noisy['z'],np.hypot(noisy['x'],noisy['y'])))
    np.testing.assert_allclose(elevations,noisy['ring'].astype(float)-16,atol=1e-5)
    assert (noisy['time']==0).all()


def test_decoder_respects_big_endian_and_row_padding():
    dtype=np.dtype({'names':['x','y','z'],'formats':['>f4']*3,'offsets':[4,8,12],'itemsize':20})
    data=bytearray(96)
    array=np.ndarray((2,2),dtype=dtype,buffer=data,strides=(48,20))
    array['x']=[[1,2],[3,4]]
    msg=PointCloud2(height=2,width=2,is_bigendian=True,point_step=20,row_step=48,data=bytes(data))
    msg.fields=[PointField(name=n,offset=o,datatype=7,count=1) for n,o in zip('xyz',[4,8,12])]
    np.testing.assert_array_equal(decode(msg)['x'],[1,2,3,4])
    msg.data=bytes(data[:-1])
    with pytest.raises(ValueError,match='Truncated'):
        decode(msg)


def test_empty_dropouts_and_invalid_beam_geometry():
    assert len(xt32_points(beam_cloud(),dropout=1))==0
    pts=np.zeros(1,dtype=OUTPUT)
    pts['x']=10
    pts['z']=1.123
    with pytest.raises(ValueError,match='beam angles'):
        xt32_points(encode(pts,Header()))


def test_attitude_from_gravity_then_gyro_and_clock_reset():
    attitude=Attitude()
    attitude.update(0,[0,0,0],[0,0,9.81])
    for i in range(1,201):
        q=attitude.update(i*.005,[0,0,1],[0,0,9.81])
    assert Rotation.from_quat(q).as_euler('xyz')[2]==pytest.approx(1,abs=.001)
    q=attitude.update(0,[0,0,0],[0,0,9.81])
    np.testing.assert_allclose(q,[0,0,0,1],atol=1e-8)
    with pytest.raises(ValueError):
        attitude.update(1,[np.nan,0,0],[0,0,9.81])


def test_lever_arm_transform_rotates_and_covariance_couples_axes():
    matrix=np.eye(4)
    matrix[:3,:3]=Rotation.from_euler('z',math.pi/2).as_matrix()
    matrix[:3,3]=[1,2,3]
    pose=Pose()
    assign_pose(pose,matrix)
    np.testing.assert_allclose(pose_matrix(pose),matrix,atol=1e-10)
    base=np.eye(4)
    base[0,3]=-1
    np.testing.assert_allclose((matrix@base)[:3,3],[1,1,3],atol=1e-10)
    cov=covariance_at_base(np.eye(6),matrix[:3,:3],[-1,0,0])
    assert cov[0,0]==pytest.approx(2)
    assert np.linalg.eigvalsh(cov).min()>0


def trajectory(xyz,q=None):
    xyz=np.asarray(xyz)
    if q is None:
        q=np.tile([0,0,0,1],(len(xyz),1))
    return np.column_stack([np.arange(len(xyz),dtype=float),xyz,q])


def test_ate_rigid_alignment_and_rpe_do_not_hide_scale():
    truth=trajectory([[0,0,0],[1,0,0],[1,1,0],[0,1,1]])
    r=Rotation.from_euler('z',.7)
    estimate=trajectory(r.apply(truth[:,1:4])+[4,5,2],np.tile(r.as_quat(),(4,1)))
    result=evaluate(estimate,truth)
    assert result['ATE_RMSE_m']<1e-10
    assert result['RPE_trans_m']<1e-10
    assert result['RPE_rot_deg']<1e-10
    assert result['ATE_unaligned_RMSE_m']>1
    scaled=truth.copy()
    scaled[:,1:4]*=2
    assert evaluate(scaled,truth)['ATE_RMSE_m']>.1


def test_missing_rpe_and_invalid_trajectory(tmp_path):
    e=trajectory([[0,0,0],[1,0,0]])
    result=evaluate(e,e,rpe_interval=2)
    assert result['RPE_trans_m'] is None
    e[1,0]=e[0,0]
    p=tmp_path/'bad.tum'
    np.savetxt(p,e)
    with pytest.raises(ValueError,match='Timestamps'):
        load_trajectory(p)


def test_unknown_space_remains_unknown_and_obstacles_override_free():
    grid=Grid(resolution=1,origin=(0.,0.),size=(6.,6.))
    assert (grid.data()==-1).all()
    grid.observe([1.5,1.5,0],[[4.5,1.5,.5]])
    grid.obstacles([[3.5,1.5,.5],[4.5,4.5,-1]])
    data=grid.data()
    assert data[1,1]==0 and data[1,3]==100 and data[5,5]==-1
    assert data[1,4]==-1  # Surface endpoint is not inferred free.
    np.testing.assert_array_equal(grid.cells([[-.1,0]]),[[-1,0]])


def test_velocity_limit_nan_stale_and_clock_backwards():
    gate=VelocityGate()
    gate.state_time=10
    gate.set_command([3,4,2],10)
    x,y,yaw=gate.output(10.1)
    assert math.hypot(x,y)==pytest.approx(.5) and yaw==.6
    assert gate.output(11)==(0,0,0)
    assert gate.output(9)==(0,0,0)
    gate.set_command([math.nan,0,0],10)
    assert gate.output(10)==(0,0,0)


def test_collision_events_and_missing_evidence():
    run=RunMetrics(0,(2,0))
    assert run.finish(1,False,'timeout')['collision_count'] is None
    for t,f in [(0,100),(1,100),(2,100),(3,0),(4,100)]:
        run.collision(t,f,50)
    run.pose(0,0)
    run.pose(1,0)
    run.pose(2,0)
    result=run.finish(5,True,'succeeded')
    assert result['collision_count']==2 and result['path_length_m']==2 and result['success']


def test_invalid_sensor_and_gate_parameters_are_rejected():
    with pytest.raises(ValueError):
        xt32_points(beam_cloud(),sigma=math.nan)
    with pytest.raises(ValueError):
        VelocityGate(max_linear=math.inf)


def test_late_pose_does_not_count_a_backwards_path():
    run=RunMetrics(0,(2,0))
    run.pose(0,0,stamp=1)
    run.pose(1,0,stamp=2)
    run.pose(-5,0,stamp=1.5)
    run.pose(2,0,stamp=3)
    assert run.finish(4,True,'succeeded')['path_length_m']==2
