"""Transient adapter health stays atomic and fails closed; no ROS context/motion."""
import os
from pathlib import Path
import time
from unittest.mock import patch
import pytest
from course_navigation import stage4_control as control
from course_navigation.stage4_guard import FinalCommandGuard, GridPreparation
from test_navigation_watchdog import fixture, state
from test_stage4_zero_boundary import zero_config


def adapter(tmp_path):
    a,c,links,out=fixture()
    a.stage4_guard=FinalCommandGuard(zero_config(tmp_path))
    return a,c,out


def test_adapter_health_never_requests_disk_durability(tmp_path,monkeypatch):
    a,c,out=adapter(tmp_path)
    def forbidden(fd):raise OSError('durability must not block transient heartbeat')
    monkeypatch.setattr(control.os,'fsync',forbidden)
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());a.publish()
    assert a.stage4_guard.failure is None
    health=control.read_control(a.stage4_guard.config['adapter_heartbeat_path'])
    assert health['sequence']==1 and health['wall']==c.wall
    assert out==[(0.,0.,0.)] and not Path(a.stage4_guard.config['lease_path']).exists()


def test_general_control_still_requires_fsync(tmp_path,monkeypatch):
    calls=[];original=control.os.fsync
    def sync(fd):calls.append(fd);return original(fd)
    monkeypatch.setattr(control.os,'fsync',sync)
    control.write_control(tmp_path/'owner',dict(sequence=1))
    assert len(calls)==1


def test_transient_health_atomic_secure_and_failed_replace_keeps_old_record(tmp_path,monkeypatch):
    path=tmp_path/'health';old=dict(sequence=1,wall=10.);new=dict(sequence=2,wall=10.02)
    control.write_adapter_health(path,old);replace=control.os.replace
    def observe_before_replace(source,target):
        assert control.read_control(target)==old
        assert Path(source).stat().st_mode & 0o777==0o600
        assert Path(source).stat().st_uid==os.getuid()
        return replace(source,target)
    monkeypatch.setattr(control.os,'replace',observe_before_replace)
    control.write_adapter_health(path,new);assert control.read_control(path)==new
    def fail(source,target):raise OSError('injected rename failure')
    monkeypatch.setattr(control.os,'replace',fail)
    with pytest.raises(OSError):control.write_adapter_health(path,dict(sequence=3))
    assert control.read_control(path)==new and list(tmp_path.iterdir())==[path]
    with pytest.raises(ValueError):control.write_adapter_health(path,dict(payload='x'*8193))
    with pytest.raises(ValueError):control.write_adapter_health(path,dict(wall=float('nan')))
    assert control.read_control(path)==new


@pytest.mark.parametrize('failure',['delay','write','rename'])
def test_delayed_or_failed_adapter_write_latches_zero_and_never_leases(tmp_path,monkeypatch,failure):
    a,c,out=adapter(tmp_path);replace=control.os.replace
    if failure=='delay':
        def delayed(source,target):time.sleep(.065);return replace(source,target)
        monkeypatch.setattr(control.os,'replace',delayed)
    elif failure=='rename':
        def failed(source,target):raise OSError('injected rename failure')
        monkeypatch.setattr(control.os,'replace',failed)
    else:
        def failed(path,value):raise OSError('injected write failure')
        monkeypatch.setattr(control,'write_adapter_health',failed)
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());a.publish()
    g=a.stage4_guard
    assert ('adapter_health_publication_deadline' if failure=='delay' else 'adapter_health_write:') in g.failure
    assert out and all(v==(0.,0.,0.) for v in out) and not g.armed
    assert Path(g.config['inhibit_path']).exists() and not Path(g.config['lease_path']).exists()
    assert not list(tmp_path.glob('.control-*'))
    if failure=='delay':
        monkeypatch.setattr(control.os,'replace',replace)
        with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):a.publish()
        first=control.read_control(g.config['adapter_heartbeat_path'])['first_deadline_timing']
        assert first['deadline_stage']=='health' and first['health_io']['wall_s']>=.05


@pytest.mark.parametrize('fault',['missing','stale','conflicting_duplicate','regression','owner'])
def test_watchdog_bad_transient_health_latches_zero_and_no_lease(tmp_path,fault):
    a,c,out=adapter(tmp_path);g=a.stage4_guard
    cfg=dict(g.config,adapter=g.adapter_identity,heartbeat_path=str(tmp_path/'owner'))
    owner=dict(session_id=cfg['session_id'],sequence=1,wall=20.)
    health=dict(owner,adapter=g.adapter_identity)
    control.write_control(cfg['heartbeat_path'],owner)
    control.write_adapter_health(cfg['adapter_heartbeat_path'],health)
    w=control.Watchdog(cfg);assert w.step(wall=20.)
    assert w.step(wall=20.01)  # identical duplicate may be read until its ORIGINAL time expires
    if fault=='missing':Path(cfg['adapter_heartbeat_path']).unlink()
    elif fault=='stale':
        control.write_control(cfg['heartbeat_path'],dict(owner,sequence=2,wall=20.16))
    elif fault=='conflicting_duplicate':control.write_adapter_health(cfg['adapter_heartbeat_path'],dict(health,wall=20.01))
    elif fault=='regression':control.write_adapter_health(cfg['adapter_heartbeat_path'],dict(health,sequence=0))
    else:cfg['owner']['start_ticks']='not-this-process';w.config['owner']=cfg['owner']
    assert not w.step(wall=20.16 if fault=='stale' else 20.02)
    assert not Path(cfg['lease_path']).exists() and Path(cfg['inhibit_path']).exists()
    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
        a.state(state());a.publish()
    assert out and all(v==(0.,0.,0.) for v in out) and not g.armed
    assert not w.step(wall=20.02)  # sticky after a later healthy callback


def test_owned_adapter_shutdown_joins_worker_and_inhibits(tmp_path):
    a,c,out=adapter(tmp_path);g=a.stage4_guard
    a.stage4_worker=GridPreparation(g)
    try:
        assert a.stop_stage4()
        assert not a.stage4_worker.worker.is_alive()
        assert g.failure=='adapter_shutdown'
        assert Path(g.config['inhibit_path']).exists() and not Path(g.config['lease_path']).exists()
        assert out==[(0.,0.,0.)]
    finally:a.stage4_worker.close()


@pytest.mark.parametrize('operation',['write','flush'])
def test_transient_partial_write_failure_removes_temporary_and_preserves_old(tmp_path,monkeypatch,operation):
    path=tmp_path/'health';old=dict(sequence=1)
    control.write_adapter_health(path,old);factory=control.tempfile.NamedTemporaryFile
    def broken_file(**kwargs):
        f=factory(**kwargs)
        def fail(*args):raise OSError('injected '+operation+' failure')
        setattr(f,operation,fail)
        return f
    monkeypatch.setattr(control.tempfile,'NamedTemporaryFile',broken_file)
    with pytest.raises(OSError):control.write_adapter_health(path,dict(sequence=2))
    assert control.read_control(path)==old and list(tmp_path.iterdir())==[path]


def test_owned_shutdown_joins_even_if_final_publisher_context_is_invalid(tmp_path):
    a,c,out=adapter(tmp_path);g=a.stage4_guard;a.stage4_worker=GridPreparation(g)
    def invalid(msg):raise RuntimeError('publisher context invalid')
    a.pub.publish=invalid
    try:
        with pytest.raises(RuntimeError):a.stop_stage4()
        assert not a.stage4_worker.worker.is_alive() and g.failure=='adapter_shutdown'
        assert Path(g.config['inhibit_path']).exists() and not Path(g.config['lease_path']).exists()
    finally:a.stage4_worker.close()
