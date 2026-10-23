"""Real atomic control reads and bounded clock interleavings; no ROS or Gazebo."""
import importlib.util
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace
import pytest
from course_navigation import stage4_control as c


def setup_records(tmp_path):
    identity=c.process_identity(os.getpid())
    cfg=dict(session_id='zero',owner=identity,adapter=identity,zero_only=True,
        **{k:str(tmp_path/k) for k in ('heartbeat_path','lease_path','inhibit_path','adapter_heartbeat_path')})
    health=dict(session_id='zero',sequence=1,wall=20.,adapter=identity)
    c.write_control(cfg['heartbeat_path'],dict(session_id='zero',sequence=1,wall=20.))
    c.write_adapter_health(cfg['adapter_heartbeat_path'],health)
    return cfg,health


def script_module():
    path=Path(__file__).resolve().parents[1]/'scripts/stage4_safety_watchdog.py'
    spec=importlib.util.spec_from_file_location('watchdog_cli_under_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('updated_record',['adapter','owner'])
def test_cli_reads_concurrent_publication_before_sampling_clock(tmp_path,monkeypatch,updated_record):
    cfg,health=setup_records(tmp_path);manifest=tmp_path/'manifest';c.write_control(manifest,cfg)
    clock=SimpleNamespace(now=20.);real_read=c.read_control;changed=[]
    def read(path):
        target=cfg['adapter_heartbeat_path'] if updated_record=='adapter' else cfg['heartbeat_path']
        if str(path)==target and not changed:
            changed.append(True);clock.now=20.01
            value=dict(health,sequence=2,wall=clock.now)
            if updated_record=='owner':value.pop('adapter')
            c.write_control(path,value)
        return real_read(path)
    monkeypatch.setattr(c,'read_control',read)
    # Same clock is observed by caller and control module; the race is deterministic.
    import time
    monkeypatch.setattr(time,'monotonic',lambda:clock.now)
    monkeypatch.setattr(time,'sleep',lambda _:setattr(clock,'now',20.2))
    module=script_module()
    monkeypatch.setattr('sys.argv',['watchdog','--manifest',str(manifest),'--wall-limit','.1'])
    assert module.main()==0  # Old caller samples20.0, then rejects the new20.01 record.
    assert changed and not Path(cfg['lease_path']).exists()
    assert c.read_control(cfg['inhibit_path'])['reason']=='watchdog_terminated'


@pytest.mark.parametrize(('mutation','wall','predicate'),[
    ('session',20.01,'health.session_id'),
    ('adapter',20.01,'health.adapter'),
    ('sequence',20.01,'health.sequence'),
    ('nonfinite',20.01,'health.wall_nonfinite'),
    ('future',20.01,'health.age_negative'),
    ('stale',20.151,'health.age_gt_150ms'),
    ('missing',20.01,'health.read'),
])
def test_first_fault_records_exact_rejected_health_and_latches(tmp_path,mutation,wall,predicate):
    cfg,health=setup_records(tmp_path)
    if mutation=='session':health['session_id']='wrong'
    if mutation=='adapter':health['adapter']=dict(cfg['adapter'],start_ticks='wrong')
    if mutation=='sequence':health['sequence']=0
    if mutation=='nonfinite':health['wall']=float('nan')
    if mutation=='future':health['wall']=20.02
    if mutation=='missing':Path(cfg['adapter_heartbeat_path']).unlink()
    else:Path(cfg['adapter_heartbeat_path']).write_text(json.dumps(health))
    watchdog=c.Watchdog(cfg)
    assert not watchdog.step(wall=wall)
    fault=watchdog.first_fault
    assert fault['failed_predicate']==predicate
    assert fault['sampled_wall']==wall
    if mutation=='future':assert fault['health_age_s']==pytest.approx(-.01)
    if mutation=='stale':assert fault['health_age_s']==pytest.approx(.151)
    if mutation!='missing':assert fault['health']['sequence']==health['sequence']
    # Error cannot be healed or its first snapshot overwritten by another record.
    c.write_adapter_health(cfg['adapter_heartbeat_path'],dict(health,session_id='zero',sequence=2,wall=wall,adapter=cfg['adapter']))
    assert not watchdog.step(wall=wall)
    assert watchdog.first_fault is fault
    assert Path(cfg['inhibit_path']).exists() and not Path(cfg['lease_path']).exists()


def test_post_read_sample_does_not_renew_stale_health(tmp_path,monkeypatch):
    cfg,health=setup_records(tmp_path)
    import time
    monkeypatch.setattr(time,'monotonic',lambda:20.151)
    watchdog=c.Watchdog(cfg)
    assert not watchdog.step()
    assert watchdog.first_fault['failed_predicate']=='health.age_gt_150ms'
    assert c.read_control(cfg['adapter_heartbeat_path'])['wall']==20.


def test_duplicates_expire_and_conflicting_duplicate_is_rejected(tmp_path):
    cfg,health=setup_records(tmp_path);watchdog=c.Watchdog(cfg)
    assert watchdog.step(wall=20.) and watchdog.step(wall=20.1)
    health['wall']=20.01;c.write_adapter_health(cfg['adapter_heartbeat_path'],health)
    assert not watchdog.step(wall=20.1)
    assert watchdog.failure=='adapter_publication_regression_or_conflict'
    assert not Path(cfg['lease_path']).exists()


def test_cli_logs_one_snapshot_only_after_inhibit(tmp_path,monkeypatch,capsys):
    cfg,health=setup_records(tmp_path);manifest=tmp_path/'manifest';c.write_control(manifest,cfg)
    import time
    monkeypatch.setattr(time,'monotonic',lambda:20.2)
    module=script_module()
    monkeypatch.setattr('sys.argv',['watchdog','--manifest',str(manifest),'--wall-limit','.1'])
    assert module.main()==2
    lines=capsys.readouterr().out.splitlines()
    assert len(lines)==1
    logged=json.loads(lines[0]);assert logged['event']=='stage4_watchdog_first_fault'
    assert logged['failed_predicate']=='health.age_gt_150ms'
    assert c.read_control(cfg['inhibit_path'])['reason']=='adapter_publication_stalled'
    assert not Path(cfg['lease_path']).exists()
