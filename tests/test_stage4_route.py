"""Stage 4 contract: no ROS initialization or motion in these tests."""
import copy
import json
import math
from pathlib import Path
import sys
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from stage4_route.contract import select_candidate, load_selection, digest, registration, check_registration
from stage4_route.evidence import RouteEvidence

BACKUP = Path('/home/zyd/b2w-course-evidence/waypoints/20261019-v1')

@pytest.fixture
def selection(tmp_path):
    if not BACKUP.exists(): pytest.skip('preserved V1 backup unavailable')
    return select_candidate(BACKUP/'candidate-16nlistv/candidate.yaml', BACKUP/'warehouse.yaml', tmp_path)


def test_exact_selection_and_v2_migration(selection):
    s = load_selection(selection)
    original = yaml.safe_load((BACKUP/'candidate-16nlistv/candidate.yaml').read_text())
    assert s['goals'] == original['goals']
    assert [g['name'] for g in s['goals']] == ['G1','G2','G3','G4','G5']
    c=yaml.safe_load(Path(s['candidate']).read_text())
    assert c['candidate_schema_version']==2 and c['deployable'] is False
    assert s['radius_m']==.25 and s['source_v1_sha256']==digest(BACKUP/'candidate-16nlistv/candidate.yaml')
    assert s['candidate_sha256']==digest(s['candidate'])


@pytest.mark.parametrize('field', ['goals','radius_m','source_map','runtime_config_sha256','candidate_sha256'])
def test_selection_tampering(selection,field):
    s=json.loads(selection.read_text());s[field]=None;selection.write_text(json.dumps(s))
    with pytest.raises((ValueError,TypeError)):load_selection(selection)


def test_map_source_mismatch(selection,tmp_path):
    s=load_selection(selection); p=Path(s['candidate']);c=yaml.safe_load(p.read_text());c['source_map']['image_sha256']='0'*64
    p.write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError):load_selection(selection)


def capture():
    return dict(session_id='test-session', graph={'/clock':['a'],'/slam/odom':['b'],'/slam/map_points':['c']},
                wall=100.,sim=20., selection_sha256='1'*64, live_map_sha256='2'*64)


def pairs(dx=0.):
    return dict(saved=[[0,0],[8,0],[0,8],[8,8]],live=[[dx,0],[8+dx,0],[dx,8],[8+dx,8]])


def test_registration_same_frame_is_insufficient():
    c=capture();r=registration(c,pairs(),alignment_p95=.02)
    check_registration(r,c,wall=110,sim=25)
    for change in ({'session_id':'new'}, {'graph':{}}, {'selection_sha256':'3'*64}):
        with pytest.raises(ValueError):check_registration(r,{**c,**change},wall=110,sim=25)
    with pytest.raises(ValueError):registration(c,pairs(.2),alignment_p95=.02)
    with pytest.raises(ValueError):registration(c,pairs(),alignment_p95=.3)
    with pytest.raises(ValueError):check_registration(r,c,wall=300,sim=25)
    with pytest.raises(ValueError):check_registration(r,c,wall=110,sim=19)


def goals():
    return [dict(name=f'G{i+1}',position=[2.*i,0.],yaw_rad=0.) for i in range(5)]


def sample(t,x,**kw):
    return dict(sim=t,wall=t,x=x,y=0.,yaw=0.,tf_stamp=t,odom_stamp=t,map_odom_stamp=t,
                slam_map_age=.1,clock_age=.01,pose_age=.01,tf_receipt_age=.01,
                speed=0.,wz=0.,command_speed=0.,command_wz=0.,safety='clear',**kw)


def visit(e,t,i):
    e.observe(sample(t,2.*i));e.observe(sample(t+.1,2.*i))


def test_inclusive_boundary_and_no_momentary_advancement():
    e=RouteEvidence(goals(),start=[0,0],count=5)
    e.observe(sample(1,.25));assert e.events==[]
    e.observe(sample(1.1,.25));assert [x['goal'] for x in e.events]==['G1']
    f=RouteEvidence(goals(),start=[0,0],count=5)
    f.observe(sample(1,.250001));f.observe(sample(1.1,.250001));assert f.events==[]


@pytest.mark.parametrize('field,value', [('tf_stamp',0),('odom_stamp',0),('map_odom_stamp',0),('pose_age',2),('tf_receipt_age',2),('slam_map_age',10),('clock_age',2),('x',float('nan'))])
def test_stale_or_missing_pose_fails(field,value):
    e=RouteEvidence(goals(),start=[0,0]);s=sample(1,0);s[field]=value;e.observe(s)
    assert e.failure and not e.events


def test_missing_tf_rejected():
    e=RouteEvidence(goals(),start=[0,0]);s=sample(1,0);del s['tf_stamp'];e.observe(s);assert e.failure


def test_missed_out_of_order_no_skip_no_retry():
    e=RouteEvidence(goals(),start=[0,0]);visit(e,1,1)
    assert e.failure=='out_of_order_G2' and not e.events
    visit(e,2,0);assert not e.events


def test_detour_and_gap_invalidates():
    e=RouteEvidence(goals(),start=[0,0]);visit(e,1,0)
    
    for i in range(1,5):
        s=sample(1.1+i*.1,i*.1);s['y']=i*.1;e.observe(s)
    assert e.failure=='disallowed_detour'
    e=RouteEvidence(goals(),start=[0,0]);e.observe(sample(1,0));e.observe(sample(2,0));assert e.failure=='pose_gap'


def complete():
    e=RouteEvidence(goals(),start=[0,0]);t=1.
    # Continuous samples through all goals; terminal rest lasts >1 s.
    for x in [i*.1 for i in range(81)]:
        e.observe(sample(t,x));t+=.1
    for _ in range(5):e.observe(sample(t,8.));t+=.1
    return e


@pytest.mark.parametrize('status',[0,2,5,6,None])
def test_action_false_success_aborted_canceled(status):
    e=complete();e.terminal(status,sim=10.);r=e.finish(cleanup=True,safety_complete=True)
    assert not r['practical_route_pass']


def test_terminal_success_alone_not_pass():
    e=RouteEvidence(goals(),start=[0,0]);e.observe(sample(1,8.));e.terminal(4,sim=1.)
    assert not e.finish(cleanup=True,safety_complete=True)['full_route_pass']


def test_stop_after_action_and_cleanup_required():
    e=complete();e.terminal(4,sim=e.last['sim']+.1);assert not e.finish(cleanup=True,safety_complete=True)['full_route_pass']
    for t in [e.terminal_sim+i*.1 for i in range(14)]:e.observe(sample(t,8.))
    r=e.finish(cleanup=True,safety_complete=True);assert r['full_route_pass'] and len(r['region_events'])==5
    assert r['narrow_passage_confirmed']
    assert not e.finish(cleanup=False,safety_complete=True)['full_route_pass']
    assert not e.finish(cleanup=True,safety_complete=False)['full_route_pass']


def test_terminal_drift_movement_or_collision_no_pass():
    for field,val in [('speed',.1),('command_speed',.1),('safety','unknown'),('safety','collision')]:
        e=complete();e.terminal(4,sim=e.last['sim']+.1)
        for i in range(14):
            s=sample(e.terminal_sim+i*.1,8.);s[field]=val;e.observe(s)
        assert not e.finish(cleanup=True,safety_complete=True)['full_route_pass']


def test_foreign_goal_and_interrupt_fail_closed():
    for why in ['competing_goal','manual_replacement','interrupted','cleanup_failed']:
        e=complete();e.fail(why);e.terminal(4,sim=e.last['sim']+.1)
        assert not e.finish(cleanup=True,safety_complete=True)['full_route_pass']


def test_launch_default_manager_and_opt_in_exclusion():
    from test_dwb_stage1_launch import load,context_for
    from launch_ros.actions import Node
    m=load(ROOT/'course_navigation/launch/navigation.launch.py')
    for disabled in (False,True):
        c=context_for(m,goal_manager_enabled='false' if disabled else 'true')
        nodes=m.setup(c)
        executables=[n._Node__node_executable for n in nodes if isinstance(n,Node)]
        assert ('goal_manager' in executables) is not disabled
        assert 'speed_governor' not in executables


def test_no_goal_pruning_recovery_or_loop(selection):
    s=load_selection(selection)
    assert s['execution']==dict(planner_action='/compute_path_through_poses',controller_action='/follow_path',planner_id='GridBased',controller_id='FollowPath',goal_checker_id='general_goal_checker',replan=False,recovery=False)


def test_safety_grid_filled_footprint_unknown_blocks():
    from stage4_route.spatial import footprint_clear
    grid=dict(width=50,height=50,resolution=.1,origin=[-2.5,-2.5,0],data=[0]*2500)
    assert footprint_clear(grid,[0,0,0])
    # Interior cell, not merely perimeter or robot center.
    grid['data'][26*50+28]=255
    assert not footprint_clear(grid,[0,0,0])
    grid['data'][26*50+28]=254
    assert not footprint_clear(grid,[0,0,0])
    assert not footprint_clear(grid,[20,20,0])


def test_single_action_and_cleanup_on_abort_interrupt():
    from stage4_route.runner import run_route
    class Fake:
        audit={}
        def __init__(self,why):self.why=why;self.sent=[];self.closed=0
        def ready(self,*a):return sample(1,0)
        def send(self,*a):self.sent.append(a)
        def tick(self):
            if self.why=='interrupt':raise KeyboardInterrupt()
            return dict(status=6)
        def sim(self):return 1.
        def close(self):self.closed+=1;return True
    for why in ('abort','interrupt'):
        f=Fake(why);r=run_route(dict(goals=goals(),execution={}),dict(session_id='one'),f,count=5)
        assert len(f.sent)==1 and f.closed==1 and not r['full_route_pass']


def test_cleanup_on_readiness_exception():
    from stage4_route.runner import run_route
    class Fake:
        audit={}
        def ready(self,*a):raise ValueError('registration')
        def close(self):self.closed=True;return False
    f=Fake();r=run_route({},dict(session_id='one'),f,count=2)
    assert f.closed and not r['cleanup_passed'] and 'registration' in r['error']


def test_planned_path_cannot_substitute_for_measured_passage():
    from stage4_route.spatial import assess_path
    grid=dict(width=250,height=100,resolution=.1,origin=[-2,-5,0],data=[0]*25000)
    path=[[i*.1,0,0] for i in range(81)]
    r=assess_path(path,goals(),[0,0],[grid]);assert r['passed'] and not r['physical_passage_proven']
    grid['data'][50*250+30]=254
    r=assess_path(path,goals(),[0,0],[grid]);assert not r['passed'] and r['sampled_footprint_blocked']>0


def test_installed_runtime_imports_without_node_init():
    from rclpy.action import get_action_client_names_and_types_by_node
    from rclpy.parameter import parameter_value_to_python
    from nav2_msgs.action import ComputePathThroughPoses,NavigateThroughPoses
    from nav2_msgs.msg import BehaviorTreeLog
    from stage4_route.ros_transport import RosTransport
    assert callable(RosTransport.preflight)


def test_repeated_tf_cannot_confirm_region():
    e=RouteEvidence(goals(),start=[0,0]);e.observe(sample(1,0));s=sample(1.1,0);s['tf_stamp']=1.;e.observe(s)
    assert e.failure and not e.events


def test_fault_during_cleanup_invalidates_report():
    from stage4_route.runner import run_route
    class Fake:
        audit={};failure=None
        def ready(self,*a):return sample(1,0)
        def send(self,*a):pass
        def tick(self):return dict(status=6)
        def sim(self):return 1.
        def close(self):self.failure='late_collision';return True
    r=run_route(dict(goals=goals(),execution={}),dict(session_id='one'),Fake(),count=2)
    assert not r['full_route_pass']


def test_preflight_rejects_early_future_region():
    from stage4_route.spatial import assess_path
    g=goals();g[3]['position']=[-.1,0.]
    grid=dict(width=250,height=100,resolution=.1,origin=[-2,-5,0],data=[0]*25000)
    r=assess_path([[i*.1,0,0] for i in range(81)],g,[0,0],[grid])
    assert not r['passed'] and r['out_of_order_planned_samples']>0


def test_rehashed_changed_pose_is_not_selected_candidate(selection,tmp_path):
    p=Path(load_selection(selection)['candidate']);c=yaml.safe_load(p.read_text())
    c['goals'][1]['yaw_rad']+=.1;a=c['goals'][1]['yaw_rad'];c['goals'][1]['quaternion'].update(z=math.sin(a/2),w=math.cos(a/2))
    p.write_text(yaml.safe_dump(c,sort_keys=False));m=yaml.safe_load(p.with_name('manifest.yaml').read_text())
    m['outputs']['candidate.yaml']=digest(p);p.with_name('manifest.yaml').write_text(yaml.safe_dump(m,sort_keys=False))
    with pytest.raises(ValueError,match='identity mismatch'):select_candidate(p,BACKUP/'warehouse.yaml',tmp_path/'changed')


def test_source_map_bytes_mismatch_rejects_before_selection(selection,tmp_path):
    import shutil
    s=load_selection(selection);folder=tmp_path/'map';folder.mkdir()
    for name in ('warehouse.yaml','warehouse.pgm'):shutil.copyfile(BACKUP/name,folder/name)
    with (folder/'warehouse.pgm').open('ab') as f:f.write(b'\n')
    with pytest.raises(ValueError,match='map hash'):select_candidate(s['candidate'],folder/'warehouse.yaml',tmp_path/'badmap')
