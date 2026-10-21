"""Owned zero-only bootstrap IPC, with real child identities and watchdog process."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))


def test_owner_bootstrap_requires_complete_prepared_health_and_never_mints_lease(tmp_path):
    from stage4_route.session import SessionOwner
    from test_stage4_safety_guard import configured
    from course_navigation.stage4_control import process_identity,read_control
    _,cfg=configured(tmp_path)
    owner=SessionOwner(tmp_path/'owned',cfg)
    child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(10)'],env=dict(os.environ,B2W_NARROW_OWNER=owner.token),start_new_session=True)
    try:
        now=time.monotonic();owner.heartbeat(now)
        health=dict(session_id='trial',wall=now,sequence=1,adapter=process_identity(child.pid),
                    prepared=dict(failure=None,structural_wall_budget=7.5,structural_source_wall_budget=7.5,grids={}))
        with pytest.raises(ValueError):owner.accept_adapter(health,wall=now)
        assert not Path(owner.config['lease_path']).exists()
        health['prepared']['grids']={name:dict(source=10.,receipt=now,sha256='a'*64) for name in ('saved','live','local','global')}
        assert owner.accept_adapter(health,wall=now)['adapter']==process_identity(child.pid)
        assert read_control(owner.config['heartbeat_path'])['sequence']==1
        assert not Path(owner.config['lease_path']).exists()  # only watchdog can issue it
        health['adapter']=process_identity(os.getpid())
        with pytest.raises(ValueError):owner.accept_adapter(health,wall=now)
    finally:
        owner.stop('test_end');child.terminate();child.wait(timeout=2.)
    assert Path(owner.config['inhibit_path']).exists()


def test_real_watchdog_expires_stalled_health_and_is_reaped(tmp_path):
    from stage4_route.session import SessionOwner
    from test_stage4_safety_guard import configured
    from course_navigation.stage4_control import process_identity,write_control,read_control
    _,cfg=configured(tmp_path);owner=SessionOwner(tmp_path/'owned',cfg)
    child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(10)'],env=dict(os.environ,B2W_NARROW_OWNER=owner.token),start_new_session=True)
    watchdog=None
    try:
        now=time.monotonic();owner.heartbeat(now)
        health=dict(session_id='trial',wall=now,sequence=1,adapter=process_identity(child.pid),
                    prepared=dict(failure=None,structural_wall_budget=7.5,structural_source_wall_budget=7.5,grids={k:{} for k in ('saved','live','local','global')}))
        write_control(owner.config['adapter_heartbeat_path'],health)
        manifest=owner.accept_adapter(health,wall=now)
        write_control(owner.directory/'watchdog.json',manifest)
        watchdog=subprocess.Popen([sys.executable,str(Path(__file__).resolve().parents[1]/'scripts/stage4_safety_watchdog.py'),
            '--manifest',str(owner.directory/'watchdog.json'),'--wall-limit','2'],
            env=dict(os.environ,PYTHONPATH=str(Path(__file__).resolve().parents[1]/'course_navigation')),start_new_session=True)
        assert watchdog.wait(timeout=3.)==2
        assert read_control(owner.config['inhibit_path'])['inhibited'] is True
        assert not Path(owner.config['lease_path']).exists()
    finally:
        owner.stop('test_end')
        if watchdog and watchdog.poll() is None:watchdog.terminate();watchdog.wait(timeout=2.)
        child.terminate();child.wait(timeout=2.)


def test_transport_structural_map_age_has_independent_three_publication_budget():
    from stage4_route.ros_transport import RosTransport
    from course_navigation.structural_freshness import StructuralMapCadence
    from types import SimpleNamespace as NS
    from unittest.mock import patch
    t=object.__new__(RosTransport);t.live_structural=StructuralMapCadence();t.sim=lambda:14.
    for source,wall in ((0.,10.),(5.,15.),(10.,20.)):t.live_structural.observe(source,source,wall)
    t.messages={'/map':NS(header=NS(frame_id='map',stamp=NS(sec=10,nanosec=0)))}
    with patch('stage4_route.ros_transport.time.monotonic',lambda:24.):
        assert t.check_live_map()['slam_map_age']==4.
    with patch('stage4_route.ros_transport.time.monotonic',lambda:28.):
        with pytest.raises(ValueError):t.check_live_map()


def test_zero_only_probe_rejects_nonzero_on_any_command_axis():
    from test_stage4_transport import transport
    from geometry_msgs.msg import Twist
    t=transport();t.stationary_only=True;m=Twist();m.linear.z=.001
    t.receive('/path_manager/path_manager_ros/nav_vel',m)
    assert t.failure


def test_zero_only_watchdog_monitors_health_without_issuing_motion_lease(tmp_path):
    from course_navigation.stage4_control import Watchdog,write_control,process_identity
    identity=process_identity(os.getpid())
    config=dict(session_id='zero',owner=identity,adapter=identity,zero_only=True,
        **{k:str(tmp_path/k) for k in ('heartbeat_path','lease_path','inhibit_path','adapter_heartbeat_path')})
    write_control(config['heartbeat_path'],dict(session_id='zero',sequence=1,wall=10.))
    write_control(config['adapter_heartbeat_path'],dict(session_id='zero',sequence=1,wall=10.,adapter=identity))
    assert Watchdog(config).step(wall=10.)
    assert not Path(config['lease_path']).exists()


def test_idle_adapter_zero_only_bootstrap_needs_no_command_or_motion_lease(tmp_path):
    from test_navigation_watchdog import fixture,state
    from test_stage4_safety_guard import configured
    from course_navigation.stage4_control import read_control
    from unittest.mock import patch
    a,c,links,out=fixture();guard,config=configured(tmp_path)
    Path(config['lease_path']).unlink();guard.config['adapter_heartbeat_path']=str(tmp_path/'adapter-health.json')
    a.stage4_guard=guard
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());a.publish();a.publish()
    assert out[-1]==(0.,0.,0.) and not guard.armed and not guard.failure
    health=read_control(guard.config['adapter_heartbeat_path'])
    assert health['sequence']==2 and health['prepared']['failure'] is None
    assert not Path(config['lease_path']).exists()
