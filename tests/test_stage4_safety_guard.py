"""Effective final guard must refuse reachable collisions and independent faults."""
import json
import os
from pathlib import Path
import time
import pytest


def grid():
    return dict(width=160,height=160,resolution=.025,origin=[-2.,-2.,0.],data=[0]*25600)


def configured(tmp_path):
    from course_navigation.stage4_guard import FinalCommandGuard
    from course_navigation.stage4_control import process_identity, write_control
    cfg=dict(session_id='trial', owner=process_identity(os.getpid()), lease_path=str(tmp_path/'lease.json'),
             inhibit_path=str(tmp_path/'inhibit.json'), stop_translation_m=.2, stop_yaw_rad=.05,
             uncertainty_m=.05, max_speed=.3, max_wz=.3, max_command=.1, max_command_wz=.2,
             abort_latency_s=.2, max_sim_rate=1.5, bounds_evidence_sha256='a'*64)
    cfg['abort_latency_s']=.6
    write_control(cfg['lease_path'],dict(session_id='trial',sequence=1,wall=20.,
        owner=cfg['owner'],adapter=process_identity(os.getpid())))
    guard=FinalCommandGuard(cfg)
    from course_navigation.stage4_contact_stream import ContactStream
    from types import SimpleNamespace as NS
    guard.contacts=ContactStream(['body::collision'],first_sequence=1)
    guard.contacts.inventory(dict(sequence=1,interval_start=9.,stamp=10.,sampled_steps=200,
        collisions=['body::collision'],ever_forbidden_contact=False,evaluation_only=True),wall=20.)
    guard.contacts.contact_message(NS(header=NS(frame_id='b2w_evaluation_seq_1',stamp=NS(sec=10,nanosec=0)),contacts=[]),wall=20.)
    for source,receipt in ((0.,10.),(5.,15.),(10.,20.)):
        guard.observe_structure(source,sim=source,wall=receipt)
    for source,receipt in ((0.,10.),(5.,15.)):
        guard.install('live',grid(),frame='map',source=source,receipt=receipt,encoding='costmap',epoch='trial')
    for name in ('saved','live','local','global'):
        guard.install(name,grid(),frame='odom' if name=='local' else 'map',source=10.,receipt=20.,
                      encoding='costmap',epoch='trial')
    return guard,cfg


def decide(g,**changes):
    kw=dict(command=(.05,0.,0.),poses={'map':[0.,0.,0.],'odom':[0.,0.,0.]},
            pose_stamp=10.,pose_receipt=20.,sim=10.,wall=20.,speed=.1,wz=0.)
    kw.update(changes);return g.output(**kw)


def test_guard_uses_all_grids_whole_stop_reach_and_latches_veto(tmp_path):
    g,cfg=configured(tmp_path)
    assert decide(g)==(.05,0.,0.)
    blocked=grid();blocked['data'][80*160+116]=254  # x=.9, outside body, inside stop reach
    g.install('global',blocked,frame='map',source=10.,receipt=20.,encoding='costmap',epoch='trial')
    assert decide(g)==(0.,0.,0.) and g.failure
    g.install('global',grid(),frame='map',source=10.,receipt=20.,encoding='costmap',epoch='trial')
    assert decide(g)==(0.,0.,0.)  # fresh data cannot clear a latched fault


@pytest.mark.parametrize('fault',['inhibit','lease','owner','pose','clock','overspeed','contact','grid'])
def test_independent_faults_force_final_zero(tmp_path,fault):
    from course_navigation.stage4_control import write_control
    g,cfg=configured(tmp_path);assert decide(g)==(.05,0.,0.)
    kw={}
    if fault=='inhibit':write_control(cfg['inhibit_path'],dict(reason='rejected_cancel'))
    if fault=='lease':kw['wall']=20.3
    if fault=='owner':g.config['owner']['start_ticks']='invalid'
    if fault=='pose':kw['pose_stamp']=9.
    if fault=='clock':kw['sim']=9.9
    if fault=='overspeed':kw['speed']=.31
    if fault=='contact':g.inhibit('forbidden_contact')
    if fault=='grid':g.invalidate('local')
    assert decide(g,**kw)==(0.,0.,0.)


def test_unknown_occupancy_is_blocked_and_pending_preparation_denies(tmp_path):
    g,cfg=configured(tmp_path)
    g.invalidate('live');assert decide(g)==(0.,0.,0.)
    blocked=grid();blocked['data'][80*160+80]=-1
    g.install('live',blocked,frame='map',source=10.,receipt=20.,encoding='occupancy',epoch='trial')
    assert decide(g)==(0.,0.,0.)


def test_watchdog_is_action_independent_and_latches_owner_loss(tmp_path):
    from course_navigation.stage4_control import Watchdog,process_identity
    owner=process_identity(os.getpid())
    cfg=dict(session_id='trial',owner=owner,adapter=owner,
             heartbeat_path=str(tmp_path/'heartbeat.json'),adapter_heartbeat_path=str(tmp_path/'adapter.json'),lease_path=str(tmp_path/'lease.json'),
             inhibit_path=str(tmp_path/'inhibit.json'))
    from course_navigation.stage4_control import write_control
    write_control(cfg['heartbeat_path'],dict(session_id='trial',sequence=1,wall=20.))
    write_control(cfg['adapter_heartbeat_path'],dict(session_id='trial',sequence=1,wall=20.,adapter=owner))
    watchdog=Watchdog(cfg)
    assert watchdog.step(wall=20.)
    assert not watchdog.step(wall=20.6)
    assert Path(cfg['inhibit_path']).exists() and not Path(cfg['lease_path']).exists()


def test_real_adapter_publication_applies_guard_after_all_command_transforms(tmp_path):
    from test_navigation_watchdog import fixture,state,command
    from unittest.mock import patch
    a,c,links,out=fixture();g,cfg=configured(tmp_path)
    a.stage4_guard=g;a.stage4_context=lambda:dict(poses={'map':[0.,0.,0.],'odom':[0.,0.,0.]},
        pose_stamp=10.,pose_receipt=20.,sim=10.,wall=20.,speed=.1,wz=0.)
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall), patch('course_navigation.velocity_adapter.require_motion_qualification',lambda:None):
        a.state(state());command(a);a.publish()
        assert out[-1]==(.1,0.,0.)
        g.inhibit('action_server_stalled');a.publish()
        assert out[-1]==(0.,0.,0.)


@pytest.mark.parametrize('fault',['localization','command'])
def test_adapter_health_loss_cannot_resume_after_guard_was_armed(tmp_path,fault):
    from test_navigation_watchdog import fixture,state,command,stamped
    from unittest.mock import patch
    a,c,links,out=fixture();g,cfg=configured(tmp_path);a.stage4_guard=g
    a.stage4_context=lambda:dict(poses={'map':[0.,0.,0.],'odom':[0.,0.,0.]},
        pose_stamp=10.,pose_receipt=20.,sim=10.,wall=20.,speed=.1,wz=0.)
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall), patch('course_navigation.velocity_adapter.require_motion_qualification',lambda:None):
        a.state(state());command(a);a.publish();assert out[-1]==(.1,0.,0.)
        if fault=='localization':stamped(links['map','odom'],9.)
        else:a.gate.command_time=19.
        a.publish();assert out[-1]==(0.,0.,0.)
        stamped(links['map','odom'],10.);command(a);a.publish()
        assert out[-1]==(0.,0.,0.) and g.failure


def test_control_file_fifo_is_rejected_without_blocking(tmp_path):
    import subprocess,sys
    p=tmp_path/'fifo';os.mkfifo(p)
    code='from course_navigation.stage4_control import read_control; read_control('+repr(str(p))+')'
    env=dict(os.environ,PYTHONPATH=str(Path(__file__).resolve().parents[1]/'course_navigation'))
    r=subprocess.run([sys.executable,'-c',code],env=env,capture_output=True,timeout=1.)
    assert r.returncode!=0


def test_watchdog_cannot_consume_its_own_lease_as_owner_heartbeat(tmp_path):
    from course_navigation.stage4_control import Watchdog,process_identity
    owner=process_identity(os.getpid());path=str(tmp_path/'same.json')
    with pytest.raises(ValueError):Watchdog(dict(session_id='trial',owner=owner,adapter=owner,
        heartbeat_path=path,adapter_heartbeat_path=str(tmp_path/'adapter.json'),lease_path=path,inhibit_path=str(tmp_path/'stop.json')))


def test_clock_rate_above_declared_bound_and_restamped_lease_fail(tmp_path):
    from course_navigation.stage4_control import write_control,process_identity
    g,cfg=configured(tmp_path);assert decide(g)==(.05,0.,0.)
    assert decide(g,sim=10.1,wall=20.01)==(0.,0.,0.) and g.failure
    other=tmp_path/'other';other.mkdir();g,cfg=configured(other);assert decide(g)==(.05,0.,0.)
    write_control(cfg['lease_path'],dict(session_id='trial',sequence=1,wall=20.01,
        owner=cfg['owner'],adapter=process_identity(os.getpid())))
    assert decide(g,wall=20.01)==(0.,0.,0.) and g.failure


def test_grid_clock_regression_is_not_a_fresh_revision(tmp_path):
    g,cfg=configured(tmp_path);assert decide(g)==(.05,0.,0.)
    g.install('local',grid(),frame='odom',source=9.9,receipt=20.1,encoding='costmap',epoch='trial')
    assert decide(g)==(0.,0.,0.) and g.failure


def test_watchdog_requires_advancing_publication_health(tmp_path):
    from course_navigation.stage4_control import Watchdog,process_identity,write_control
    ident=process_identity(os.getpid())
    cfg=dict(session_id='trial',owner=ident,adapter=ident,**{k:str(tmp_path/k) for k in
        ('heartbeat_path','lease_path','inhibit_path','adapter_heartbeat_path')})
    write_control(cfg['heartbeat_path'],dict(session_id='trial',sequence=1,wall=20.))
    write_control(cfg['adapter_heartbeat_path'],dict(session_id='trial',sequence=1,wall=20.,adapter=ident))
    w=Watchdog(cfg);assert w.step(wall=20.)
    # A new owner heartbeat cannot revive a frozen adapter publication.
    write_control(cfg['heartbeat_path'],dict(session_id='trial',sequence=2,wall=20.2))
    assert not w.step(wall=20.2) and Path(cfg['inhibit_path']).exists()


def test_real_adapter_context_detects_quaternion_disagreement(tmp_path):
    from test_navigation_watchdog import fixture,state,command
    from unittest.mock import patch
    a,c,links,out=fixture();g,cfg=configured(tmp_path);a.stage4_guard=g
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall), patch('course_navigation.velocity_adapter.require_motion_qualification',lambda:None):
        a.state(state());command(a);a.publish();assert out[-1]==(.1,0.,0.)
        links['odom','base_link'].transform.rotation.z=.1
        links['odom','base_link'].transform.rotation.w=(1-.01)**.5
        a.publish();assert out[-1]==(0.,0.,0.) and g.failure


def test_real_grid_worker_prepares_then_joins_on_shutdown(tmp_path):
    from course_navigation.stage4_guard import GridPreparation
    g,cfg=configured(tmp_path);worker=GridPreparation(g)
    try:
        worker.submit('local',grid(),frame='odom',source=10.1,receipt=20.1,encoding='costmap',epoch='trial')
        deadline=time.monotonic()+2.
        while time.monotonic()<deadline:
            with g.lock:entry=g.grids.get('local')
            if entry and entry[1]==10.1:break
            time.sleep(.005)
        assert entry and entry[1]==10.1
    finally:assert worker.close()
    assert not worker.worker.is_alive() and Path(cfg['inhibit_path']).exists()


def test_final_adapter_has_its_own_unconditional_motion_interlock(tmp_path):
    from test_navigation_watchdog import fixture,state,command
    from unittest.mock import patch
    a,c,links,out=fixture();g,cfg=configured(tmp_path);a.stage4_guard=g
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());command(a);a.publish()
    assert out[-1]==(0.,0.,0.) and 'UNQUALIFIED_FOR_MOTION' in g.failure


def test_stage4_launch_forwards_manifest_and_excludes_other_command_owners():
    from test_dwb_stage1_launch import load,context_for
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    root=Path(__file__).resolve().parents[1]
    m=load(root/'course_navigation/launch/navigation.launch.py')
    for enabled in ('true','false'):
        context=context_for(m,stage4_guard_manifest='/tmp/private-session/guard.json',goal_manager_enabled=enabled)
        if enabled=='true':
            with pytest.raises(ValueError,match='Stage4'):m.setup(context)
        else:
            nodes=m.setup(context);adapter=next(n for n in nodes if isinstance(n,Node) and n._Node__node_executable=='velocity_adapter')
            params=evaluate_parameters(context,adapter._Node__parameters)
            assert any(v.get('stage4_guard_manifest')=='/tmp/private-session/guard.json' for v in params if isinstance(v,dict))
