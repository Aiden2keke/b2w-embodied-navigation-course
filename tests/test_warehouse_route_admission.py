"""Offline geometry checks; synthetic fault injection is distinct from recorded replay."""
import math
from pathlib import Path
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import warehouse_route_admission as admission


def grid():
    return dict(width=120, height=120, resolution=.05, origin=(-3., -3.), data=[0]*14400)


def put(g, xy, value):
    x, y = (math.floor((v-o)/g['resolution']) for v,o in zip(xy,g['origin']))
    g['data'][y*g['width']+x] = value


def check(g, **overrides):
    args = dict(pose=(0.,0.,0.), path=[(0.,0.,0.),(.5,0.,0.)], radius=.4,
                padding=.05, reaction_stop_m=.2, ready=True)
    args.update(overrides)
    return admission.admit(g, **args)


def test_side_clutter_outside_sweep_does_not_require_open_disk():
    g=grid(); put(g,(0.,1.4),100)
    assert check(g)['passed']


@pytest.mark.parametrize('xy,kind', [((0.,0.),'current'),((.8,0.),'sweep'),((1.1,0.),'stop')])
def test_current_path_and_stop_obstacles_reject(xy,kind):
    g=grid();put(g,xy,100)
    if kind=='sweep':
        result=check(g,path=[(0.,0.,0.),(1.,0.,0.)])
    else: result=check(g)
    assert not result['passed']
    assert result['conflict']['value']==100


def test_unknown_in_stop_envelope_rejects():
    g=grid(); put(g,(1.1,0.),-1)
    assert not check(g)['passed']


def test_initial_rotation_requires_entire_articulated_radius():
    g=grid();put(g,(0.,.6),100)
    assert not check(g,path=[(0.,0.,0.),(0.,0.,math.pi/2),(.5,0.,math.pi/2)])['passed']


@pytest.mark.parametrize('overrides', [dict(path=None),dict(path=[]),dict(path=[(1.,0.,0.),(2.,0.,0.)]),
    dict(radius=math.nan),dict(radius=0),dict(pose=(math.nan,0.,0.)),dict(ready=False),
    dict(reaction_stop_m=None),dict(reaction_stop_m=-1),dict(padding=-1)])
def test_missing_or_invalid_inputs_fail_closed(overrides):
    assert not check(grid(),**overrides)['passed']


def test_no_sampling_gap_between_path_vertices():
    g=grid();put(g,(.95,.05),100)
    assert not check(g,path=[(0.,0.,0.),(2.,0.,0.)],radius=.01,padding=.01,reaction_stop_m=.05)['passed']


def test_actual_urdf_full_joint_containment_exceeds_nav2_static_rectangle():
    radius=admission.collision_radius(Path(__file__).resolve().parents[1]/'artifacts/recovery/robot.urdf')
    assert 1.30 < radius < 1.32  # hand sum of hip offset, serial link lengths and wheel sphere
    assert radius > math.hypot(.75,.43)


def test_plan_validation_requires_success_fresh_map_frame_and_goal_endpoint():
    packet=dict(status=4, frame='map',stamp=10.,poses=[(0.,0.,1.57),(0.,1.,1.57)])
    prefix=admission.initial_prefix(packet,(0.,0.,0.),(0.,1.),10.1)
    assert prefix[0]==(0.,0.,0.) and prefix[-1][:2]==(0.,.5)
    for replacement in (dict(status=6),dict(frame='odom'),dict(stamp=9.),dict(poses=[]),
                        dict(poses=[(0.,0.,0.),(2.,2.,0.)])):
        with pytest.raises(ValueError):
            admission.initial_prefix(dict(packet,**replacement),(0.,0.,0.),(0.,1.),10.1)


def test_path_collision_is_diagnosed_even_when_stopping_bound_is_missing():
    g=grid();put(g,(.8,0.),100)
    result=check(g,path=[(0.,0.,0.),(1.,0.,0.)],reaction_stop_m=None)
    assert result['reason']=='swept_footprint'


@pytest.mark.parametrize('bad', [dict(frame='odom'),dict(status=6),dict(stamp=1.),dict(poses=[]),None])
def test_route_boundary_missing_or_unreliable_plan_never_admits(tmp_path,monkeypatch,bad):
    run_boundary(tmp_path,monkeypatch,bad)


def run_boundary(tmp_path,monkeypatch,replacement='valid',expect_success=False,**options):
    from types import SimpleNamespace as NS
    import rclpy.action
    import warehouse_ab_trial as trial
    import run_warehouse_route as route
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped
    goal=route.critical_goals(route.load_route())[0]['pose']['pose']['position']
    packet=dict(frame='map',status=4,stamp=10.,poses=[(0.,0.,0.),(0.,.5,1.57),(goal['x'],goal['y'],1.57)])
    if isinstance(replacement,dict): packet.update(replacement)
    path=RosPath();path.header.frame_id=packet['frame'];path.header.stamp.sec=int(packet['stamp'])
    for x,y,yaw in packet['poses']:
        p=PoseStamped();p.pose.position.x=x;p.pose.position.y=y
        p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);path.poses.append(p)
    events=[]
    def future(result): return NS(done=lambda:True,result=lambda:result)
    class Client:
        def __init__(self,node,kind,name): assert name=='/compute_path_to_pose'
        def server_is_ready(self): return True
        def send_goal_async(self,request):
            events.append('compute_path')
            assert request.planner_id=='GridBased' and not request.use_start
            return future(NS(accepted=replacement is not None,get_result_async=lambda:future(NS(status=packet['status'],result=NS(path=path)))))
        def destroy(self): events.append('destroy_client')
    monkeypatch.setattr(rclpy.action,'ActionClient',Client)
    node=NS(get_publishers_info_by_topic=lambda name: [] if name=='/goal_pose' else [1],
            get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:path.header.stamp)))
    monitor=NS(grid=grid(),pose=lambda:(0.,0.,0.),ready=lambda:True,physical_veto=lambda:None,final=[0.]*6,clock=10.1)
    def wait(predicate,timeout): assert predicate()
    if options.get('planning_only'):options.setdefault('deferred_capture',{})
    if expect_success:
        trial.route_start_admission(node,monitor,tmp_path,wait,lambda:events.append('navigation_stage'),**options)
    else:
        with pytest.raises((RuntimeError,ValueError)):
            trial.route_start_admission(node,monitor,tmp_path,wait,lambda:events.append('navigation_stage'),**options)
    assert events==['navigation_stage','compute_path','destroy_client']
    return events


def test_route_reaches_planning_stage_but_empirical_coast_cannot_waive_bounds(tmp_path,monkeypatch):
    import json
    run_boundary(tmp_path,monkeypatch)
    result=json.loads((tmp_path/'route-start-admission.json').read_text())
    assert result['reason']=='missing_validated_reaction_stopping_bound'
    assert not result['passed'] and not result['observed_coast_is_operating_bound']


def test_actual_canonical_recording_clears_footprint_but_does_not_admit_without_path():
    import json,hashlib
    root=Path(__file__).resolve().parents[1]
    file=root/'artifacts/trials/motion-continuous-20261025-1/precheck-09dd3f9808114acab09462b38820c567/pocket-admission-initial.json'
    assert hashlib.sha256(file.read_bytes()).hexdigest()=='47880732fffadf53114fb8580169922b20b9b62e2eb3098619bd015a3edc6f36'
    saved=json.loads(file.read_text())
    from warehouse_ab_trial import open_pocket
    assert not open_pocket(saved['grid'],saved['sensor_pose'][:2])
    result=admission.admit(saved['grid'],pose=saved['sensor_pose'],path=None,
        radius=admission.collision_radius(root/'artifacts/recovery/robot.urdf'),padding=.05,
        reaction_stop_m=None,ready=True)
    assert result['current_footprint_passed']
    assert not result['passed'] and result['reason']=='missing_validated_path'


def test_prismatic_or_unsupported_collision_cannot_claim_fixed_containment(tmp_path):
    file=tmp_path/'unsafe.urdf'
    file.write_text('<robot><link name="base_link"/><link name="leg"><collision><geometry><sphere radius="0.1"/></geometry></collision></link><joint name="slide" type="prismatic"><parent link="base_link"/><child link="leg"/><origin xyz="0 0 0"/></joint></robot>')
    with pytest.raises(ValueError): admission.collision_radius(file)


def directional(g,**kwargs):
    args=dict(pose=(0.,0.,0.),path=[(0.,0.,0.),(.5,0.,0.)],half_extents=(.7,.38),
        padding=.05,stop=dict(forward_m=.3,reverse_m=.02,lateral_m=.02,yaw_rad=.02,rotation_drift_m=.02),ready=True)
    args.update(kwargs)
    return admission.admit_directional(g,**args)


def test_directional_braking_does_not_expand_into_unrelated_side_clutter():
    g=grid();put(g,(0.,1.1),100)
    assert directional(g)['passed']
    put(g,(1.5,0.),100)
    assert not directional(g)['passed']


def test_rotation_sweeps_rectangle_between_legal_endpoints():
    g=grid();put(g,(.55,.55),100)
    assert directional(g,path=[(0.,0.,0.),(.1,0.,0.)],stop=None)['footprint_passed']
    result=directional(g,path=[(0.,0.,0.),(0.,0.,math.pi/2)],stop=None)
    assert not result['passed'] and result['reason']=='initial_sweep_collision'


def test_directional_unknown_stopping_cells_and_absent_bounds_fail_closed():
    g=grid();put(g,(1.5,0.),-1)
    assert not directional(g)['passed']
    assert directional(grid(),stop=None)['reason']=='missing_qualified_stopping_bounds'


def test_reviewed_envelope_requires_expected_bytes_deployment_and_raw_evidence(tmp_path):
    import json,hashlib
    raw=tmp_path/'raw.json';raw.write_text('recorded input')
    packet=dict(schema_version=1,urdf_sha256='a'*64,profile_sha256='b'*64,command_caps=[.4,.6],
        half_extents=[.7,.38],padding_m=.05,localization_error_m=.02,tracking_error_m=.03,
        body_tilt_limit_rad=.1,physical_speed_cap_mps=.5,physical_yaw_cap_rad_s=.8,stop=dict(forward_m=.3,reverse_m=.02,lateral_m=.02,yaw_rad=.05,rotation_drift_m=.02),
        evidence=[dict(path=str(raw),sha256=hashlib.sha256(raw.read_bytes()).hexdigest())])
    file=tmp_path/'envelope.json';file.write_text(json.dumps(packet));sha=hashlib.sha256(file.read_bytes()).hexdigest()
    assert admission.load_envelope(file,sha,'a'*64,'b'*64,[.4,.6])['half_extents']==[.7,.38]
    for digest in ('c'*64,None):
        with pytest.raises(ValueError):admission.load_envelope(file,digest,'a'*64,'b'*64,[.4,.6])
    raw.write_text('changed')
    with pytest.raises(ValueError):admission.load_envelope(file,sha,'a'*64,'b'*64,[.4,.6])


def test_runtime_can_admit_with_supplied_directional_bounds_not_hardcoded_none(tmp_path,monkeypatch):
    import json
    envelope=dict(half_extents=[.7,.38],padding_m=.05,localization_error_m=.02,tracking_error_m=.03,
        stop=dict(forward_m=.3,reverse_m=.02,lateral_m=.02,yaw_rad=.05,rotation_drift_m=.02))
    run_boundary(tmp_path,monkeypatch,expect_success=True,envelope=envelope)
    result=json.loads((tmp_path/'route-start-admission.json').read_text())
    assert result['passed'] and result['unavailable_bounds']==[]


def test_planning_result_is_successful_capture_without_motion_admission(tmp_path,monkeypatch):
    import warehouse_ab_trial as trial
    capture={}
    run_boundary(tmp_path,monkeypatch,expect_success=True,planning_only=True,deferred_capture=capture)
    assert (tmp_path/'initial-smac-plan.json').exists() and not (tmp_path/'route-start-map.json').exists()
    trial.finish_planning_capture(tmp_path,capture)
    assert (tmp_path/'route-start-map.json').exists()


def test_arrival_yaw_uses_validated_last_nonzero_path_tangent_not_goal_quaternion():
    packet=dict(status=4,frame='map',stamp=10.,request_pose=(0.,0.,0.),
        poses=[(0.,0.,0.),(.5,0.,0.),(.5,.5,2.4),(.5,.5,2.4)])
    assert admission.arrival_yaw(packet,(.5,.5),grid(),(.7,.38),.05,10.1)==pytest.approx(math.pi/2)
    g=grid();put(g,(.5,.5),-1)
    with pytest.raises(ValueError):admission.arrival_yaw(packet,(.5,.5),g,(.7,.38),.05,10.1)


def test_planning_capture_defers_bulk_grid_and_geometry_until_owned_teardown(tmp_path,monkeypatch):
    import warehouse_ab_trial as trial
    capture={};calls=[]
    real=admission.admit
    def forbidden_while_active(*args,**kwargs):
        calls.append('geometry');return real(*args,**kwargs)
    monkeypatch.setattr(admission,'admit',forbidden_while_active)
    run_boundary(tmp_path,monkeypatch,expect_success=True,planning_only=True,deferred_capture=capture)
    assert not calls and not (tmp_path/'route-start-map.json').exists()
    assert capture['snapshot']['clock']==10.1
    result=trial.finish_planning_capture(tmp_path,capture)
    assert calls and not result['passed']
    assert (tmp_path/'initial-smac-plan.json').exists() and (tmp_path/'route-start-map.json').exists()
