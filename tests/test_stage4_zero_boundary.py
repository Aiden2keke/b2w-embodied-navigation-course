"""Zero observation cannot inherit motion qualification or arm through a lease."""
import os
from pathlib import Path
from unittest.mock import patch
import pytest
from course_navigation.stage4_control import process_identity,write_control,read_control
from course_navigation.stage4_guard import FinalCommandGuard
from test_stage4_safety_guard import decide,configured
from test_navigation_watchdog import fixture,state


def zero_config(tmp_path):
    return dict(session_id='observe',zero_only=True,owner=process_identity(os.getpid()),
        lease_path=str(tmp_path/'lease'),inhibit_path=str(tmp_path/'inhibit'),
        adapter_heartbeat_path=str(tmp_path/'health'))


def test_zero_guard_needs_no_physical_bounds_and_never_arms(tmp_path):
    g=FinalCommandGuard(zero_config(tmp_path))
    assert decide(g)==(0.,0.,0.) and not g.armed and not g.failure
    write_control(g.config['lease_path'],dict(session_id='observe',sequence=1,wall=20.,owner=g.config['owner'],adapter=g.adapter_identity))
    assert decide(g)==(0.,0.,0.) and not g.armed
    assert g.failure=='unexpected_lease_in_zero_only_session'


def test_motion_guard_still_requires_measured_bounds(tmp_path):
    cfg=zero_config(tmp_path);cfg.pop('zero_only')
    with pytest.raises((KeyError,ValueError)):FinalCommandGuard(cfg)


def test_zero_context_wait_emits_health_and_recovers(tmp_path):
    a,c,links,out=fixture();a.stage4_guard=FinalCommandGuard(zero_config(tmp_path))
    # Direct edge is ready; composite at odom time has not reached the TF listener.
    from tf2_ros import TransformException
    composite=links.pop(('map','base_link'))
    def lookup(p,ch,t):
        if (p,ch) not in links:raise TransformException('not yet received')
        return links[p,ch]
    a.tf.lookup_transform=lookup
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());a.publish()
        health=read_control(a.stage4_guard.config['adapter_heartbeat_path'])
        assert not a.stage4_guard.failure and health['context_state']=='waiting_tf'
        links['map','base_link']=composite;a.publish()
    assert read_control(a.stage4_guard.config['adapter_heartbeat_path'])['context_state']=='ready'
    assert out==[(0.,0.,0.),(0.,0.,0.)]


@pytest.mark.parametrize('armed',[False,True])
def test_context_corruption_latches_even_before_arming(tmp_path,armed):
    a,c,links,out=fixture();a.stage4_guard,_=configured(tmp_path);a.stage4_guard.armed=armed
    links['map','base_link'].transform.rotation.w=2.
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());a.publish()
    assert out[-1]==(0.,0.,0.) and a.stage4_guard.failure


def test_zero_localization_wait_always_writes_health(tmp_path):
    a,c,links,out=fixture();a.stage4_guard=FinalCommandGuard(zero_config(tmp_path))
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):a.publish()
    assert read_control(a.stage4_guard.config['adapter_heartbeat_path'])['context_state']=='waiting_localization'
    assert out==[(0.,0.,0.)] and not a.stage4_guard.failure


def test_zero_stream_late_join_binds_only_observed_interval():
    from course_navigation.stage4_contact_stream import ContactStream
    from types import SimpleNamespace as NS
    stream=ContactStream(['body::collision'],first_sequence=None,observation_only=True)
    stream.contact_message(NS(header=NS(frame_id='b2w_evaluation_seq_42',stamp=NS(sec=10,nanosec=0)),contacts=[]),wall=20.)
    stream.inventory(dict(sequence=42,interval_start=9.9,stamp=10.,sampled_steps=20,
        collisions=['body::collision'],ever_forbidden_contact=False,evaluation_only=True),wall=20.)
    assert stream.ready(sim=10.,wall=20.)
    assert stream.intervals.closed(start=9.9,end=10.,sim=10.,wall=20.)
    assert not stream.intervals.closed(start=0.,end=10.,sim=10.,wall=20.)


def test_zero_inputs_pin_source_inventory_without_motion_evidence(tmp_path):
    import json,hashlib,sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
    from stage4_session import observation_guard
    source=tmp_path/'robot.sdf';source.write_text('<sdf><model><link name="body"><collision name="collision"/></link></model></sdf>')
    inv=tmp_path/'inventory.json';inv.write_text(json.dumps(dict(converted_sdf=str(source),
        converted_sdf_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),collisions=['body::collision'])))
    cfg=dict(session_id='observe',collision_inventory_path=str(inv),collision_inventory_sha256=hashlib.sha256(inv.read_bytes()).hexdigest())
    value=observation_guard(cfg)
    assert value['zero_only'] is True and 'stop_translation_m' not in value and 'saved_grid_path' not in value
    source.write_text('<sdf/>')
    with pytest.raises(ValueError):observation_guard(cfg)


@pytest.mark.parametrize('axis',['linear.x','linear.z','angular.x','angular.z'])
def test_zero_adapter_rejects_any_nonzero_input_axis(tmp_path,axis):
    from geometry_msgs.msg import Twist
    a,c,links,out=fixture();a.stage4_guard=FinalCommandGuard(zero_config(tmp_path))
    msg=Twist();group,name=axis.split('.');setattr(getattr(msg,group),name,.001)
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):a.command(msg);a.publish()
    assert a.stage4_guard.failure and out[-1]==(0.,0.,0.) and not a.stage4_guard.armed


def test_armed_tf_absence_still_latches(tmp_path):
    from tf2_ros import TransformException
    a,c,links,out=fixture();a.stage4_guard,_=configured(tmp_path);a.stage4_guard.armed=True
    a.tf.lookup_transform=lambda *args:(_ for _ in ()).throw(TransformException('missing'))
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):a.state(state());a.publish()
    assert a.stage4_guard.failure=='localization_transform_failure' and out[-1]==(0.,0.,0.)


def test_zero_invalid_state_is_permanent_not_a_tf_wait(tmp_path):
    a,c,links,out=fixture();a.stage4_guard=FinalCommandGuard(zero_config(tmp_path))
    msg=state();msg.pose.pose.orientation.w=0.
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):a.state(msg);a.publish()
    assert a.stage4_guard.failure and out[-1]==(0.,0.,0.)


def test_zero_bootstrap_accepts_only_ready_three_grid_adapter(tmp_path):
    import subprocess,sys,time
    from stage4_route.session import SessionOwner
    cfg=zero_config(tmp_path);owner=SessionOwner(tmp_path/'session',cfg)
    child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(5)'],
        env=dict(os.environ,B2W_NARROW_OWNER=owner.token),start_new_session=True)
    try:
        now=time.monotonic()
        health=dict(session_id='observe',sequence=1,wall=now,adapter=process_identity(child.pid),context_state='waiting_tf',
            prepared=dict(failure=None,grids={k:{} for k in ('live','local','global')},structural_wall_budget=7.5,structural_source_wall_budget=7.5))
        with pytest.raises(ValueError):owner.accept_adapter(health,wall=now)
        health['context_state']='ready'
        assert owner.accept_adapter(health,wall=now)['zero_only'] is True
        assert not Path(owner.config['lease_path']).exists()
    finally:owner.stop('test_done');child.terminate();child.wait(timeout=2.)


def test_probe_capacity_waits_for_five_recorded_seconds_then_checks_real_capacity(tmp_path):
    from types import SimpleNamespace as NS
    from stage4_route.ros_transport import RosTransport
    from stage4_zero_probe import check_capacity
    t=object.__new__(RosTransport);t.recording_first_wall=10.;t.recording_window_wall=10.
    t.recorded_input_bytes=100;t.recording_window_bytes=100;t.recording_peak_rate=20.
    t.raw_path=tmp_path/'journal';t.raw_path.write_bytes(b'raw');t.audit={}
    t.journal=NS(max_bytes=1000000,reserve=0)
    assert check_capacity(t,wall=14.9,last_capacity=0.)==0.
    assert 'recording_capacity' not in t.audit
    with patch('stage4_route.ros_transport.time.monotonic',lambda:15.):
        assert check_capacity(t,wall=15.,last_capacity=0.)==15.
    assert t.audit['recording_capacity']['observed_wall_s']==5.
    t.journal.max_bytes=4
    with patch('stage4_route.ros_transport.time.monotonic',lambda:16.):
        with pytest.raises(ValueError,match='Insufficient recording capacity'):check_capacity(t,wall=16.,last_capacity=15.)


def test_motion_contact_stream_cannot_implicitly_late_join():
    from course_navigation.stage4_contact_stream import ContactStream
    with pytest.raises(ValueError):ContactStream(['body::collision'],first_sequence=None)
