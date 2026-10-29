"""Sustained-only admission and stop guards; never start ROS/Gazebo."""
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import warehouse_ab_trial as trial
from test_warehouse_precheck_only import session

BASE=Path(__file__).resolve().parents[1]/'artifacts/recovery/root-installed-manifest.json'
BASE_SHA='48d09d89b3528367d8a6ecd2a793f7e9562265cbb28be25272b55886447b57b5'

def cli():
    return ['--profile','B','--check','--precheck-only','--precheck-axis','sustained-yaw',
            '--precheck-pulse-s','1','--precheck-spawn-x','0','--precheck-spawn-y','-1',
            '--controller-revision-sha256','a'*64]

def test_one_second_is_admitted_offline_without_execute(monkeypatch):
    checked=[]
    monkeypatch.setattr(trial,'check_inputs',lambda a:checked.append(a) or {})
    monkeypatch.setattr(trial,'execute',lambda *a:pytest.fail('motion started'))
    assert trial.main(cli())==0
    assert checked[0].precheck_pulse_s==1 and checked[0].wall_limit==420

@pytest.mark.parametrize('flag,value',[('--precheck-pulse-s','1.000001'),('--precheck-pulse-s','nan'),
 ('--precheck-pulse-s','inf'),('--precheck-pulse-s','0'),('--precheck-pulse-s','-.1'),
 ('--profile','A'),('--precheck-spawn-x','1'),('--precheck-spawn-y','0')])
def test_wrong_range_or_profile_rejects_before_deployment(monkeypatch,flag,value):
    args=cli();args[args.index(flag)+1]=value
    monkeypatch.setattr(trial,'check_inputs',lambda a:pytest.fail('deployment reached'))
    with pytest.raises(SystemExit):trial.main(args)

@pytest.mark.parametrize('flags',[['--gui'],['--recording-end-hold-s','1'],['--stationary-view-s','1'],
 ['--recording-parent-pid','1','--recording-parent-start-ticks','1'],['--video-ready-file','unused'],
 ['--route-mode','full','--continuation-yaws','0','0','0']])
def test_sustained_rejects_recording_and_route_options(monkeypatch,flags):
    monkeypatch.setattr(trial,'check_inputs',lambda a:pytest.fail('deployment reached'))
    with pytest.raises(SystemExit):trial.main(cli()+flags)

def test_sustained_requires_revision_before_input_check(monkeypatch):
    args=cli();i=args.index('--controller-revision-sha256');del args[i:i+2]
    monkeypatch.setattr(trial,'check_inputs',lambda a:pytest.fail('deployment reached'))
    with pytest.raises(SystemExit):trial.main(args)

def candidate(tmp_path):
    if not BASE.is_file() or trial.digest(BASE)!=BASE_SHA:
        pytest.skip('the source checkout does not contain the pinned sustained-turn baseline manifest')
    m=json.loads(BASE.read_text());m['controller_revision']['source_pins'][str(trial.ROOT/'scripts/warehouse_ab_trial.py')]=trial.digest(trial.ROOT/'scripts/warehouse_ab_trial.py')
    m['controller_revision']['sustained_turn']=dict(baseline_manifest_path=str(BASE),baseline_manifest_sha256=BASE_SHA,max_duration_s=1.,command_rad_s=.25,profile='B')
    p=tmp_path/'candidate.json';p.write_text(json.dumps(m))
    args=NS(deployment_manifest=p,controller_revision_sha256=trial.digest(p),precheck_axis='sustained-yaw',precheck_only=True)
    old=json.loads((trial.ROOT/'artifacts/recovery/root-installed-manifest.json').read_text())
    return m,args,old

@pytest.mark.parametrize('tamper',['missing_scope','wrong_baseline','native','source_install','other_source','stationary','wrong_digest'])
def test_sustained_manifest_cannot_launder_other_identity_changes(tmp_path,tamper):
    m,a,old=candidate(tmp_path)
    if tamper=='missing_scope':del m['controller_revision']['sustained_turn']
    elif tamper=='wrong_baseline':m['controller_revision']['sustained_turn']['baseline_manifest_sha256']='0'*64
    elif tamper=='native':m['native'][next(iter(m['native']))]['sha256']='0'*64
    elif tamper=='source_install':m['source_install'][0]['source_sha256']='0'*64
    elif tamper=='other_source':m['controller_revision']['source_pins'][str(trial.ROOT/'scripts/observe_warehouse_narrow_failure.py')]='0'*64
    elif tamper=='stationary':m['controller_revision']['stationary_evidence']['sha256']='0'*64
    a.deployment_manifest.write_text(json.dumps(m));a.controller_revision_sha256=trial.digest(a.deployment_manifest)
    if tamper=='wrong_digest':a.controller_revision_sha256='0'*64
    with pytest.raises(ValueError):trial.validate_controller_revision(a,m,old)

def test_stale_imported_manifest_cannot_authorize_sustained_turn(tmp_path):
    m,a,old=candidate(tmp_path)
    with pytest.raises(ValueError,match='repository-relative source pin'):
        trial.validate_controller_revision(a,m,old)

def test_sustained_is_one_positive_pulse_then_measured_zero_rest(session):
    session.args.precheck_axis='sustained-yaw';session.args.precheck_pulse_s=1.
    assert session.run()==0,session.report().get('error')
    r=session.report();assert [p['kind'] for p in r['precheck']]==['sustained-yaw']
    p=r['precheck'][0];assert 1<=p['end_pulse_sim']-p['start_sim']<1.11
    assert {tuple(v) for _,v in session.commands}=={(0.,)*6,(0.,0.,0.,0.,0.,.25)}
    assert session.commands[-1][1]==[0.]*6 and r['normal_rest']['six_axis_final_zero']
    assert r['normal_cleanup']['passed'] and session.goals==0

@pytest.mark.parametrize('fault',['clearance','duplicate_owner','goal_source','readiness','turn',
 'pulse_clock_freeze','physical_stop','watchdog_completion','no_new_map','stale_map'])
def test_sustained_fault_zeroes_and_does_not_try_a_second_pulse(session,fault):
    session.args.precheck_axis='sustained-yaw';session.args.precheck_pulse_s=1.;session.fault=fault
    assert session.run()==1
    r=session.report();assert r['error'] and len(r['precheck'])<=1 and session.goals==0
    assert session.commands[-1][1]==[0.]*6 and r['cleanup']['passed']

@pytest.mark.parametrize('yaw,vx,wz,expected',[(.351,0,0,True),(.1,.251,0,True),(.1,0,.401,True),
 (.1,float('nan'),0,True),(.35,.25,.4,False)])
def test_whole_run_guard_applies_with_zero_command_during_stop(yaw,vx,wz,expected):
    m=trial.Monitor.__new__(trial.Monitor)
    m.precheck_axis='sustained-yaw';m.precheck_motion_origin=(0.,0.,0.)
    m.truth_pose=lambda:(0.,0.,yaw);m.select_truth=lambda wall:True
    pose=NS(position=NS(x=0.,y=0.,z=.65),orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))
    m.truth=(0.,0.,NS(pose=NS(pose=pose),twist=NS(twist=NS(linear=NS(x=vx,y=0.),angular=NS(z=wz)))))
    m.phase='precheck';m.final=[0.]*6
    assert bool(m.physical_veto()) is expected


def test_sustained_failure_consumes_one_attempt_before_execute(tmp_path,monkeypatch):
    calls=[]
    monkeypatch.setattr(trial,'check_inputs',lambda a:{})
    monkeypatch.setattr(trial,'execute',lambda a,p:calls.append(p) or 1)
    args=cli();args.remove('--check');args+=['--execute','--authorize-motion','precheck-only','--output-root',str(tmp_path)]
    assert trial.main(args)==1
    assert (tmp_path/'precheck.used').is_file()
    assert trial.main(args)==2 and len(calls)==1

@pytest.mark.parametrize('conflict',['gpu','simulator'])
def test_host_admission_rejects_competing_gpu_or_simulator(tmp_path,monkeypatch,conflict):
    import run_g1_g5_record as record
    monkeypatch.setattr(record.media,'run',lambda args:NS(stdout='1234, foreign CUDA, 200' if conflict=='gpu' else ''))
    original=Path.glob
    cmdline=tmp_path/'123'/'cmdline';cmdline.parent.mkdir();cmdline.write_bytes(b'/usr/bin/ign\0gazebo\0-s\0')
    monkeypatch.setattr(Path,'glob',lambda self,pattern:iter([cmdline]) if str(self)=='/proc' else original(self,pattern))
    with pytest.raises(RuntimeError):record.host_admission()


def test_rolling_demo_is_one_combined_motion_then_zero_rest(session):
    session.args.precheck_axis='rolling-demo';session.args.precheck_pulse_s=1.
    assert session.run()==0,session.report().get('error')
    r=session.report();assert [p['kind'] for p in r['precheck']]==['rolling-demo']
    assert {tuple(v) for _,v in session.commands}=={(0.,)*6,(.05,0.,0.,0.,0.,.05)}
    assert session.commands[-1][1]==[0.]*6 and r['normal_rest']['six_axis_final_zero']
    assert session.goals==0 and r['normal_cleanup']['passed']


def test_rolling_revision_preserves_original_controller_and_scope(tmp_path):
    m,a,old=candidate(tmp_path);del m['controller_revision']['sustained_turn']
    m['controller_revision']['rolling_demo']=dict(baseline_manifest_path=str(BASE),baseline_manifest_sha256=BASE_SHA,
        max_duration_s=1.,command_vx_m_s=.05,command_wz_rad_s=.05,profile='B')
    a.precheck_axis='rolling-demo';a.deployment_manifest.write_text(json.dumps(m));a.controller_revision_sha256=trial.digest(a.deployment_manifest)
    with pytest.raises(ValueError,match='repository-relative source pin'):
        trial.validate_controller_revision(a,m,old)
    m['controller_revision']['rolling_demo']['command_vx_m_s']=.1
    a.deployment_manifest.write_text(json.dumps(m));a.controller_revision_sha256=trial.digest(a.deployment_manifest)
    with pytest.raises(ValueError):trial.validate_controller_revision(a,m,old)
