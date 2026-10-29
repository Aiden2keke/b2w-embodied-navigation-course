"""Targeted startup diagnostics; real callbacks without a ROS context or motion."""
import time
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from unittest.mock import patch
import pytest
from test_navigation_watchdog import fixture,state
from test_stage4_zero_boundary import zero_config
from course_navigation.stage4_guard import FinalCommandGuard
from course_navigation.stage4_control import read_control


@pytest.mark.parametrize('zero_only',[True,False])
@pytest.mark.parametrize('kind',['sleep','cpu'])
def test_publication_overrun_retains_timing_and_only_zero_mode_recovers(tmp_path,kind,zero_only):
    a,c,links,out=fixture()
    if zero_only:a.stage4_guard=FinalCommandGuard(zero_config(tmp_path))
    else:
        from test_stage4_safety_guard import configured
        a.stage4_guard,_=configured(tmp_path)
        a.stage4_guard.config['adapter_heartbeat_path']=str(tmp_path/'health')
        Path(a.stage4_guard.config['lease_path']).unlink()  # unarmed is NOT ZeroOnly
    original=a.stage4_context
    def measured_context():
        if kind=='sleep':time.sleep(.065)
        else:
            until=time.thread_time()+.060
            while time.thread_time()<until:pass
        return original()
    a.stage4_context=measured_context
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());a.publish()
    assert a.stage4_guard.failure==(None if zero_only else 'adapter_publication_deadline')
    assert Path(a.stage4_guard.config['inhibit_path']).exists() is (not zero_only)
    assert out[-1]==(0.,0.,0.) and not a.stage4_guard.armed
    health=read_control(a.stage4_guard.config['adapter_heartbeat_path'])
    timing=health['publication_timing']['sections']['prepublish']
    assert timing['wall_s']>=.05
    if kind=='sleep':assert timing['thread_cpu_s']<.03
    else:assert timing['thread_cpu_s']>=.05
    assert 'context' in health['publication_timing']['sections']


def test_stage4_bridge_is_explicit_one_way_evaluation_only():
    from stage4_session import evaluation_bridge_argv
    argv=evaluation_bridge_argv()
    assert '/b2w/evaluation/full_contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts' in argv
    assert '/b2w/evaluation/collision_inventory@std_msgs/msg/String[ignition.msgs.StringMsg' in argv
    assert not any('cmd_vel' in arg or 'ground_truth' in arg or ']' in arg for arg in argv)


def test_owned_evaluation_bridge_failure_stops_before_watchdog_or_probe(tmp_path,monkeypatch):
    import json
    import stage4_session as session
    launched=[]
    class Child:
        def __init__(self,code):self.returncode=code
        def poll(self):return self.returncode
        def wait(self,timeout=None):return self.returncode or 0
    def popen(argv,**kwargs):
        launched.append(argv)
        return Child(None if len(launched)==1 else 2)
    monkeypatch.setattr(session.subprocess,'Popen',popen)
    monkeypatch.setattr(session.OwnedProcesses,'cleanup',lambda self,child:dict(passed=True,remaining={}))
    value=dict(guard=zero_config(tmp_path),world='/unused-test-world',pretrained_directory='/unused-test-policy')
    assert session.execute(value,tmp_path/'session')==1
    result=json.loads((tmp_path/'session/session-result.json').read_text())
    assert 'evaluation bridge exited during bootstrap' in result['error']
    assert not result['zero_motion_smoke_pass'] and len(launched)==2
    assert not (tmp_path/'session/watchdog.json').exists()
    assert not (tmp_path/'session/lease.json').exists()


def test_first_zero_overrun_timing_survives_following_fast_callbacks(tmp_path):
    a,c,links,out=fixture();a.stage4_guard=FinalCommandGuard(zero_config(tmp_path));original=a.stage4_context
    def delayed():time.sleep(.065);return original()
    a.stage4_context=delayed
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());a.publish()
        first=read_control(a.stage4_guard.config['adapter_heartbeat_path'])['first_deadline_timing']
        a.stage4_context=original;a.publish()
    second=read_control(a.stage4_guard.config['adapter_heartbeat_path'])['first_deadline_timing']
    assert first['sections']['prepublish']['wall_s']>=.05
    assert second['sections']['prepublish']==first['sections']['prepublish']
    assert first['deadline_stage']==second['deadline_stage']=='prepublish'
    assert a.stage4_guard.failure is None and all(v==(0.,0.,0.) for v in out)
    assert not Path(a.stage4_guard.config['lease_path']).exists()


@pytest.mark.parametrize('fault',['owner','lease','nonzero'])
def test_zero_overrun_does_not_hide_other_sticky_faults(tmp_path,fault):
    from geometry_msgs.msg import Twist
    a,c,links,out=fixture();a.stage4_guard=FinalCommandGuard(zero_config(tmp_path));g=a.stage4_guard
    original=a.stage4_context
    def delayed():time.sleep(.065);return original()
    a.stage4_context=delayed
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());a.publish()
        assert g.failure is None
        if fault=='owner':g.config['owner']['start_ticks']='lost-owner'
        elif fault=='lease':Path(g.config['lease_path']).write_text('{}')
        else:
            msg=Twist();msg.linear.x=.001;a.command(msg)
        a.publish()
    assert g.failure and Path(g.config['inhibit_path']).exists()
    assert not g.armed and all(v==(0.,0.,0.) for v in out)
