"""Source-time localization snapshots; unrelated TF cannot renew either edge."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))


def fixture():
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link'
    odom.header.stamp.sec=1;odom.pose.pose.orientation.w=1.
    transforms={}
    for parent,child in [('map','base_link'),('map','odom'),('odom','base_link')]:
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
        t.header.stamp.sec=1;t.transform.rotation.w=1.;transforms[(parent,child)]=t
    class Buffer:
        def lookup_transform(self,parent,child,time):
            assert time.nanoseconds==1000000000,'latest query mixes source times'
            return transforms[(parent,child)]
    seen={k:dict(stamp=1.,wall=10.) for k in [('map','odom'),('odom','base_link')]}
    return Buffer(),odom,seen,transforms


def test_all_tf_queries_use_advancing_odom_time_and_agree_with_pose():
    from stage4_route.ros_transport import coherent_localization
    b,o,s,t=fixture()
    assert len(coherent_localization(b,o,s,sim=1.1,wall=10.1))==3
    t[('odom','base_link')].transform.translation.x=.01
    with pytest.raises(ValueError,match='disagreement'):coherent_localization(b,o,s,sim=1.1,wall=10.1)


def test_stale_edge_cannot_hide_under_fresh_other_tf():
    from stage4_route.ros_transport import coherent_localization
    b,o,s,t=fixture();s[('map','odom')]['wall']=9.
    with pytest.raises(ValueError,match='Stale'):coherent_localization(b,o,s,sim=1.1,wall=10.1)
    s.pop(('map','odom'))
    with pytest.raises(ValueError):coherent_localization(b,o,s,sim=1.1,wall=10.1)


def test_mixed_transform_stamp_and_nonfinite_pose_fail_closed():
    from stage4_route.ros_transport import coherent_localization
    b,o,s,t=fixture();t[('map','odom')].header.stamp.nanosec=100000000
    with pytest.raises(ValueError,match='source time'):coherent_localization(b,o,s,sim=1.1,wall=10.1)
    b,o,s,t=fixture();o.pose.pose.position.x=float('nan')
    with pytest.raises(ValueError):coherent_localization(b,o,s,sim=1.1,wall=10.1)


def test_odom_arriving_before_tf_uses_latest_complete_fresh_snapshot():
    from stage4_route.ros_transport import select_coherent_localization
    import copy
    b,o,s,t=fixture();new=copy.deepcopy(o);new.header.stamp.nanosec=100000000
    # Source-time1.1 is not yet present on both links;1.0 remains fresh.
    selected,transforms,receipt=select_coherent_localization(b,[(o,10.),(new,10.1)],s,sim=1.15,wall=10.15)
    assert selected is o and receipt==10.
    with pytest.raises(ValueError):select_coherent_localization(b,[(o,10.)],s,sim=1.6,wall=10.6)


def test_nearly_unit_norm_cannot_hide_pose_rotation_disagreement():
    from stage4_route.ros_transport import coherent_localization
    import math
    b,o,s,t=fixture()
    o.pose.pose.orientation.w=1/math.cos(.05)
    t[('odom','base_link')].transform.rotation.z=math.sin(.05)
    t[('odom','base_link')].transform.rotation.w=math.cos(.05)
    with pytest.raises(ValueError,match='disagreement'):
        coherent_localization(b,o,s,sim=1.1,wall=10.1)


def test_accepted_small_quaternion_norm_error_does_not_bias_body_heading():
    from stage4_route.ros_transport import yaw
    from geometry_msgs.msg import Quaternion
    import math
    q=Quaternion(z=math.sin(.3)*math.sqrt(1.005),w=math.cos(.3)*math.sqrt(1.005))
    assert yaw(q)==pytest.approx(.6,abs=1e-12)
