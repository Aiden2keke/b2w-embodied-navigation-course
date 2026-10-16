"""Offline calibration contracts; no ROS/Gazebo session is needed."""
import builtins
import csv
import hashlib
import json
import math
from pathlib import Path

import pytest

from scripts import run_velocity_steps as runner
from scripts.velocity_step_core import make_sequence, requested_at


@pytest.mark.parametrize('speeds,end', [((.2,),12.),((.04,.07,.10,.13,.20),60.)])
def test_custom_sequence_and_half_open_boundaries(speeds,end):
    seq=make_sequence(speeds)
    assert seq[-1]['end_s']==end
    for i,u in enumerate(speeds):
        start=i*12.
        assert requested_at(seq,start+3.-1e-8)==(0.,0.,0.)
        assert requested_at(seq,start+3.)==(u,0.,0.)
        assert requested_at(seq,start+8.)==(0.,0.,0.)
    assert requested_at(seq,end)==(0.,0.,0.)
    assert all(p['vy']==p['wz']==0. for p in seq)


@pytest.mark.parametrize('value',['','0','-0.1','nan','inf','0.2,0.20','.03','.40001','a','0.2,','0.1,,0.2'])
def test_invalid_cli_is_explicit_and_precedes_any_live_work(value,capsys):
    with pytest.raises(SystemExit) as exc:
        runner.make_parser().parse_args(['--speeds='+value])
    assert exc.value.code==2
    assert 'error:' in capsys.readouterr().err


def test_custom_dry_run_and_default_contract(capsys):
    assert runner.main(['--dry-run','--speeds','.2'])==0
    data=json.loads(capsys.readouterr().out)
    assert data['duration_sim_s']==12.
    assert [p['vx'] for p in data['sequence'] if p['phase']=='target']==[.2]
    assert runner.main(['--dry-run'])==0
    data=json.loads(capsys.readouterr().out)
    assert data['sequence']==make_sequence((.1,.2,.3,.4))
    assert data['duration_sim_s']==48. and data['request_hz']==20.


def test_custom_export_duration_and_precise_speed_filenames(tmp_path):
    seq=make_sequence((.071,.072))
    runner._write_outputs(tmp_path,{'sequence':seq,'experiment_start_sim_s':0.,
        'sequence_duration_sim_s':24.},[],{'per_speed':[], 'quality':{}})
    with (tmp_path/'timeseries.csv').open() as f:
        assert len(list(csv.DictReader(f)))==480
    assert runner.speed_filename(.1)=='speed-0.10.json'
    assert runner.speed_filename(.071)!=runner.speed_filename(.072)


def core():
    from scripts import velocity_calibration_core
    return velocity_calibration_core


def observations(fn, repeats=1):
    return [{'run_id':str(r),'u':u,'v':fn(u)} for r in range(repeats) for u in [.1,.2,.3,.4]]


def test_proportional_and_affine_and_zero_state():
    c=core()
    prop=c.fit_model('M1',observations(lambda u:1.5*u))
    affine=c.fit_model('M2',observations(lambda u:u+.07)+[{'run_id':'0','u':0.,'v':0.}])
    assert prop['parameters']['k']==pytest.approx(1.5)
    assert affine['parameters']==pytest.approx({'a':1.,'b':.07})
    assert c.predict(affine,0.)==0.
    assert c.predict(affine,.05) is None
    assert c.inverse(affine,0.)==0.
    assert c.inverse(affine,.1) is None
    assert affine['input_domain']=={'zero_state':[0.,0.], 'moving':[.1,.4]}


def test_monotonic_piecewise_inverse_no_extrapolation_or_zero_bridge():
    c=core(); model=c.fit_model('M3',observations(lambda u:u*u+u))
    assert c.inverse(model,.3)==pytest.approx(.2 + (.3-.24)/(.39-.24)*.1)
    assert c.predict(model,.05) is None
    assert c.inverse(model,.05) is None
    assert c.inverse(model,1.) is None
    assert c.predict(model,.5) is None
    bad=c.fit_model('M3',observations(lambda u:.4-u))
    assert bad['valid'] is False and c.inverse(bad,.2) is None
    plateau=c.fit_model('M3',observations(lambda u:.2))
    assert plateau['monotonic'] is True
    assert c.inverse(plateau,.2) is None # inverse is not uniquely identifiable


def test_repeat_aggregation_run_count_and_loso():
    c=core();rows=observations(lambda u:u+.07,3)
    rows[-1]['v']+=.03
    result=c.compare_models(rows)
    assert result['independent_runs']==3 and result['distinct_speeds']==4
    repeat=result['repeatability'][-1]
    assert repeat['n_runs']==3 and repeat['mean']==pytest.approx(.48)
    assert repeat['std']>0 and repeat['range']==pytest.approx(.03)
    model=result['models']['M2']
    assert len(model['residuals_by_speed'])==4
    assert model['loso']['predicted_count']==6 # interior speeds only: no extrapolation
    assert model['loso']['unsupported_count']==6
    assert model['metrics']['mae']>=0.
    assert result['production_selection'] is None


def test_wheel_order_and_sign_conversion_from_synthetic_axes():
    c=core(); names=['RL_foot_joint','FR_foot_joint','RR_foot_joint','FL_foot_joint']
    signs={'FR_foot_joint':1.,'FL_foot_joint':-1.,'RR_foot_joint':1.,'RL_foot_joint':-1.}
    actual=c.forward_equivalent(names,[-4.,1.,3.,-2.],signs)
    assert actual==[1.,2.,3.,4.]
    assert c.forward_equivalent(names,[-4.,None,3.,-2.],signs) is None
    assert c.forward_equivalent(names[:-1],[-4.,1.,3.],signs) is None
    assert c.axis_forward_sign([0.,1.,0.])==1.
    assert c.axis_forward_sign([0.,-1.,0.])==-1.
    with pytest.raises(ValueError):c.axis_forward_sign([1.,0.,0.])


def test_offline_cli_never_imports_ros_or_starts_processes(tmp_path,monkeypatch):
    from scripts import fit_velocity_calibration as cli
    real_import=builtins.__import__
    def guarded(name,*a,**k):
        if name.split('.')[0] in {'rclpy','launch','launch_ros','rosbag2_py'}:
            pytest.fail('offline fitter imported ROS')
        return real_import(name,*a,**k)
    monkeypatch.setattr(builtins,'__import__',guarded)
    import subprocess
    monkeypatch.setattr(subprocess,'Popen',lambda *a,**k:pytest.fail('offline fitter started a process'))
    # Native synthetic capture with genuinely missing policy/wheel/truth streams.
    run=tmp_path/'run';run.mkdir()
    (run/'manifest.json').write_text(json.dumps({'sequence':make_sequence((.2,)),
        'experiment_start_sim_s':0.,'termination':{'status':'completed'},'cleanup':{'passed':True}}))
    (run/'events.jsonl').write_text('')
    before={p.name:p.read_bytes() for p in run.iterdir()}
    assert cli.main([str(run),'--output-dir',str(tmp_path/'fit')])==0
    assert before=={p.name:p.read_bytes() for p in run.iterdir()}
    result=json.loads((tmp_path/'fit/calibration.json').read_text())
    row=result['observations'][0]
    assert row['u'] is None and row['v'] is None
    assert row['wheel']['actual_mean_radps'] is None
    assert row['invalid_reasons']
    assert result['truth']['models']['M2']['valid'] is False
    assert cli.main([str(run),'--output-dir',str(run/'bad')])!=0
    assert not (run/'bad').exists()


def test_input_copies_do_not_count_as_independent_runs(tmp_path):
    from scripts import fit_velocity_calibration as cli
    for name in ('a','b'):
        d=tmp_path/name;d.mkdir()
        (d/'manifest.json').write_text(json.dumps({'termination':{'status':'completed'},'cleanup':{'passed':True}}))
        (d/'events.jsonl').write_text('')
    assert cli.main([str(tmp_path/'a'),str(tmp_path/'b'),'--output-dir',str(tmp_path/'fit')])!=0


def native_fixture(directory, run_token='one', missing=()):
    from scripts import fit_velocity_calibration as cli
    import yaml
    names=yaml.safe_load((runner.ROOT/'course_sim/config/rl_sar.yaml').read_text())['b2w_rl_sar_controller']['ros__parameters']['joint_names']
    directory.mkdir()
    manifest={'sequence':make_sequence((.2,)),'experiment_start_sim_s':0.,
        'termination':{'status':'completed'},'cleanup':{'passed':True},'owner_token_sha256':run_token,
        'source_hashes':{p:cli.digest(runner.ROOT/p) for p in cli.AUTH_FILES}}
    events=[]
    def add(topic,t,data,frame=None):
        if topic not in missing:events.append({'topic':topic,'sim_s':t,'source_s':t,'wall_s':t+len(run_token),
            'data':data,'valid':True,'epoch':0,'child_frame_id':frame})
    for i in range(241):
        t=i*.05; moving=3<=t<8;u=.2 if moving else 0.;v=.27 if moving else 0.
        for topic in [runner.RAW_TOPIC,runner.SHARED_TOPIC,runner.FINAL_TOPIC,runner.REQUEST_TOPIC]:
            add(topic,t,{'linear_x':u,'linear_y':0.,'angular_z':0.})
        obs=[0.]*57;obs[6]=u;add('/network_input_debug',t,{'data':obs})
        add('/joint_commands',t,{'data':[0.]*12+[1.,2.,3.,4.]})
        add('/sim/raw/joint_states',t,{'name':names,'velocity':[0.]*12+[2.4]*4,'position':[0.]*16})
        for topic in (runner.TRUTH_TOPIC,runner.SLAM_TOPIC):
            add(topic,t,{'linear':[v,0.,0.],'angular':[0.,0.,0.],
                'position':[min(5.,max(0.,t-3))*.27,0.,.6],'quaternion_xyzw':[0.,0.,0.,1.]},'base_link')
    (directory/'manifest.json').write_text(json.dumps(manifest))
    (directory/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    return manifest,events


def test_native_three_runs_and_missing_torque_stays_null(tmp_path):
    from scripts import fit_velocity_calibration as cli
    dirs=[tmp_path/str(i) for i in range(3)]
    for i,d in enumerate(dirs):native_fixture(d,run_token=str(i)* (i+1))
    result=cli.analyze_directories(dirs)
    assert result['truth']['independent_runs']==3
    assert result['truth']['repeatability'][0]['std']==pytest.approx(0.)
    assert result['truth']['models']['M1']['parameters']['k']==pytest.approx(1.35)
    assert result['truth']['models']['M2']['valid'] is False
    assert result['observations'][0]['torque_peak_limit_ratio'] is None
    w=result['observations'][0]['wheel']
    assert w['target_radps']==pytest.approx([1.,2.,3.,4.])
    assert w['actual_radps']==pytest.approx([2.4]*4)
    assert w['tracking_target_minus_actual_radps']==pytest.approx([-1.4,-.4,.6,1.6])
    assert w['tracking_mean_radps']==pytest.approx(.1)
    assert w['tracking_aggregate_mae_radps']==pytest.approx(1.)


def test_missing_streams_never_become_zero_and_auth_failure_is_explicit(tmp_path):
    from scripts import fit_velocity_calibration as cli
    m,events=native_fixture(tmp_path/'run',missing=('/sim/raw/joint_states',runner.RAW_TOPIC))
    row=cli.extract_observations(events,m,'test')[0]
    assert row['wheel']['actual_mean_radps'] is None
    assert row['pipeline']['raw']['mean'] is None
    assert row['valid_for_fit'] is False
    m['source_hashes'][cli.CPP]='wrong'
    row=cli.extract_observations(events,m,'test')[0]
    assert row['u'] is None and row['authentication']['reasons']


def test_actual_recorder_twist_only_sets_linear_x():
    """Exercise the unmodified real method with a plain zero-initialized Twist."""
    import ast
    from types import SimpleNamespace
    path=runner.ROOT/'scripts/velocity_step_recorder.py'
    tree=ast.parse(path.read_text())
    method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='record_request')
    def twist():return SimpleNamespace(linear=SimpleNamespace(x=0.,y=0.,z=0.),angular=SimpleNamespace(x=0.,y=0.,z=0.))
    namespace={'Twist':twist};exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),namespace)
    sent=[];recorded=[];rec=SimpleNamespace(publisher=SimpleNamespace(publish=sent.append),_write=lambda *a:recorded.append(a))
    namespace['record_request'](rec,.07,{},3.)
    assert vars(sent[0].linear)=={'x':.07,'y':0.,'z':0.}
    assert vars(sent[0].angular)=={'x':0.,'y':0.,'z':0.}


def test_real_tick_uses_custom_completion_boundary_without_ros():
    import ast
    from types import SimpleNamespace
    tree=ast.parse((runner.ROOT/'scripts/run_velocity_steps.py').read_text())
    method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='tick')
    namespace=vars(runner).copy();exec(compile(ast.Module(body=[method],type_ignores=[]),'runner tick','exec'),namespace)
    now=runner.time.monotonic();sent=[]
    node=SimpleNamespace(clock_s=11.999,experiment_start=0.,run_start_wall=now,last_clock_wall=now,
        last_audit_wall=now,sequence=make_sequence((.2,)),zero_only=False,complete=False,abort_reason=None,
        lifecycle_poll=lambda:None,recorder=SimpleNamespace(record_request=lambda *a:sent.append(a)))
    namespace['tick'](node);assert node.complete is False
    node.clock_s=12.;namespace['tick'](node);assert node.complete is True
    assert all(call[0]==0. for call in sent)


def test_native_float32_plateau_with_jitter_remains_one_speed(tmp_path):
    """Integral rounding must not fabricate distinct commands or an affine slope."""
    from scripts import fit_velocity_calibration as cli
    import random
    import struct
    actual=struct.unpack('f',struct.pack('f',.2))[0]
    all_rows=[]
    for i in range(3):
        m,events=native_fixture(tmp_path/str(i),run_token=str(i))
        rng=random.Random(i)
        debug=[e for e in events if e['topic']=='/network_input_debug']
        for e in debug:e['data']['data'][6]=actual if 3<=e['sim_s']<8 else 0.
        # Nonuniform policy receipts, retaining a fully constant native plateau.
        t=5.95
        events=[e for e in events if e['topic']!='/network_input_debug']
        while t<8.1:
            events.append({'topic':'/network_input_debug','sim_s':t,'source_s':None,
                'epoch':0,'valid':True,'data':{'data':[0.]*6+[actual]+[0.]*50}})
            t+=rng.uniform(.016,.026)
        rows=cli.extract_observations(events,m,str(i));all_rows.extend(rows)
    assert all(r['valid_for_fit'] for r in all_rows)
    result=core().compare_models(all_rows)
    assert result['distinct_speeds']==1
    assert result['repeatability'][0]['n_runs']==3
    assert result['models']['M2']['valid'] is False
    assert all(r['u']==actual for r in all_rows)


@pytest.mark.parametrize('changed_path',['course_slam/config/lio_sam.yaml',
    'third_party/b2w_rl_sar/robot_assets/b2w_description/urdf/b2w_description.urdf',
    'course_sim/launch/simulation.launch.py'])
def test_changed_estimator_plant_or_launch_contract_cannot_be_pooled(tmp_path,changed_path):
    from scripts import fit_velocity_calibration as cli
    dirs=[tmp_path/'a',tmp_path/'b']
    for i,d in enumerate(dirs):
        m,_=native_fixture(d,run_token=str(i)*(i+1))
        m['source_hashes'][changed_path]=str(i)
        (d/'manifest.json').write_text(json.dumps(m))
    with pytest.raises(ValueError,match='incompatible'):
        cli.analyze_directories(dirs)


def test_truth_fit_does_not_depend_on_valid_slam_and_varying_policy_is_rejected(tmp_path):
    from scripts import fit_velocity_calibration as cli
    m,events=native_fixture(tmp_path/'run',missing=(runner.SLAM_TOPIC,))
    row=cli.extract_observations(events,m,'run')[0]
    assert row['v']==pytest.approx(.27) and row['valid_for_fit'] is True
    assert row['slam_v'] is None and row['slam_invalid_reasons']
    for e in events:
        if e['topic']=='/network_input_debug' and e['sim_s']==7.:
            e['data']['data'][6]+=.0000001
    row=cli.extract_observations(events,m,'run')[0]
    assert row['valid_for_fit'] is False
    assert any('constant' in s for s in row['invalid_reasons'])
