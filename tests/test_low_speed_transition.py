"""Prospective transition protocol: no fitting and no implied production bridge."""
import struct
from pathlib import Path

import pytest


def api():
    from scripts import low_speed_transition
    return low_speed_transition


def test_protocol_exists():
    assert Path('scripts/low_speed_transition.py').is_file()


def test_preregistered_points_zero_reference_repeat_anchors_and_timing():
    p=api().plan()
    assert p['policy_inputs']==[0.,.005,.01,.015,.02,.03,.04,.01,.04]
    assert p['sequence'][-1]['end_s']==108.
    assert len(p['sequence'])==27
    assert all(row['vy']==row['wz']==0. for row in p['sequence'])
    assert p['m2_inverse_enabled'] is False
    assert p['validated_condition']=='pure_forward_vy0_wz0'
    assert p['refit_performed'] is False and p['production_selection'] is None
    assert p['interpretations']==['A_CONTINUOUS_BRANCH_COMPATIBLE_ON_TESTED_GRID','B_DEAD_ZONE_MINIMUM_MOVING_SPEED_EVIDENCE','C_UNSUITABLE_OR_INSUFFICIENT']
    assert p['legacy_guard']['kind']=='experiment_range_not_safety_threshold'
    assert p['idle_hold_audit']['strict_less_than']==.01
    assert p['idle_hold_audit']['enabled'] is True
    assert p['idle_hold_audit']['command_space_before_float32'] is True
    assert p['idle_hold_audit']['point_005_idle'] is True
    assert p['idle_hold_audit']['point_010_idle'] is False
    assert p['idle_hold_audit']['point_005_float32'] == struct.unpack('f',struct.pack('f',.005))[0]
    assert p['idle_hold_audit']['point_005_float32'] != 0.


def test_existing_range_guard_preserved_with_explicit_diagnostic_extension():
    from scripts.velocity_step_core import make_sequence
    with pytest.raises(ValueError):make_sequence([.005])
    with pytest.raises(ValueError):make_sequence([0.])
    assert make_sequence([.04,.07])[-1]['end_s']==24.
    assert make_sequence([0.,.005,.01,.01],transition_diagnostic=True)[-1]['end_s']==48.
    for invalid in [-.001,.041,float('nan')]:
        with pytest.raises(ValueError):make_sequence([invalid],transition_diagnostic=True)


def rows(values):
    points=api().SPEEDS
    return [{'requested_u':u,'truth_steady_vx':v,'valid':True,'stable':True} for u,v in zip(points,values)]


def test_continuous_branch_interpretation_is_limited_to_tested_grid():
    m=api();r=m.interpret(rows([0.,0.,.015,.030,.045,.075,.1068,.015,.1068]),True)
    assert r['classification']=='A_CONTINUOUS_BRANCH_COMPATIBLE_ON_TESTED_GRID'
    assert r['production_authorized'] is False
    assert r['continuity_proven'] is False


def test_dead_zone_and_minimum_observed_speed_are_reported_without_bridge():
    m=api();r=m.interpret(rows([0.,0.,.075,.08,.09,.10,.107,.075,.107]),True)
    assert r['classification']=='B_DEAD_ZONE_MINIMUM_MOVING_SPEED_EVIDENCE'
    assert r['minimum_observed_moving_vx']==.075
    assert r['production_authorized'] is False and r['bridge_selected'] is False


@pytest.mark.parametrize('case',['invalid','unstable','nonmonotonic','repeat_mismatch','missing'])
def test_unsuitable_or_missing_cannot_select_branch(case):
    m=api();data=rows([0.,0.,.015,.030,.045,.075,.1068,.015,.1068]);valid=True
    if case=='invalid':valid=False
    if case=='unstable':data[3]['stable']=False
    if case=='nonmonotonic':data[4]['truth_steady_vx']=.1
    if case=='repeat_mismatch':data[-1]['truth_steady_vx']=.07
    if case=='missing':data.pop()
    assert m.interpret(data,valid)['classification']=='C_UNSUITABLE_OR_INSUFFICIENT'


def native_evidence():
    from scripts.fit_velocity_calibration import AUTH_FILES, authenticate, digest
    m=api();protocol=m.plan();hashes={f:digest(m.ROOT/f) for f in AUTH_FILES};hashes[str(m.base.NAV_CONFIG.relative_to(m.ROOT))]=digest(m.base.NAV_CONFIG);auth=authenticate({'source_hashes':hashes})
    events=[];x=0.;physical={0.:0.,.005:0.,.01:.075,.015:.08,.02:.09,.03:.10,.04:.1068}
    for i in range(2161):
        t=i*.05;u=next((r['vx'] for r in protocol['sequence'] if r['start_s']<=t<r['end_s']),0.)
        v=physical[u];x+=v*.05
        def event(topic,data,source=None,child=None):
            return {'topic':topic,'sim_s':t,'source_s':source,'epoch':0,'valid':True,'child_frame_id':child,'data':data}
        for topic in [m.base.REQUEST_TOPIC,m.base.RAW_TOPIC,m.base.SHARED_TOPIC,m.base.FINAL_TOPIC]:
            events.append(event(topic,{'linear_x':u,'linear_y':0.,'angular_z':0.}))
        obs=[0.]*57;obs[6]=struct.unpack('f',struct.pack('f',u))[0]
        events.append(event('/network_input_debug',{'data':obs}))
        for topic in [m.base.TRUTH_TOPIC,m.base.SLAM_TOPIC]:
            events.append(event(topic,{'linear':[v,0.,0.],'angular':[0.,0.,0.],'position':[x,0.,.6],'quaternion_xyzw':[0.,0.,0.,1.]},t,'base_link'))
        targets=[0.]*16
        if u>=.01:
            targets[12:]=[1.]*4
        events.append(event('/joint_commands',{'data':targets}))
        events.append(event('/joint_torques',{'data':[0.]*16}))
        events.append(event('/sim/raw/joint_states',{'name':auth['joint_names'],'position':[0.]*16,'velocity':targets},t))
        events.append(event('/clock',{'clock_s':t},t))
    manifest={'sequence':protocol['sequence'],'experiment_start_sim_s':0.,'transition_protocol':protocol,
        'transition_source_hashes':m.source_hashes(),'m2_inverse_enabled':False,'source_hashes':hashes,
        'truth_stop_speed_mps':.005,'policy_sha256':m.base.sha256(m.base.POLICY),
        'termination':{'status':'completed','final_status':'completed'},'cleanup':{'passed':True},
        'readiness':{'completed':True,'checks':{k:True for k in m.base.READINESS_KEYS}},
        'max_gap_s':{'truth':.15,'slam':.35,'command':.075}}
    return events,manifest


def test_native_report_authenticates_all_nine_points_and_preserves_repeats(monkeypatch):
    from scripts import velocity_calibration_core as core
    from scripts import fit_velocity_calibration as fitter
    def forbidden(*a,**k):raise AssertionError('transition diagnostic must not fit models')
    monkeypatch.setattr(core,'fit_model',forbidden);monkeypatch.setattr(core,'compare_models',forbidden)
    monkeypatch.setattr(fitter,'compare_models',forbidden)
    m=api();e,manifest=native_evidence();r=m.report(e,manifest)
    assert r['data_valid'],r['data_invalid_reasons']+[(p['trial'],p['invalid_reasons']) for p in r['points']]
    assert len(r['points'])==9
    assert r['interpretation']['classification']=='B_DEAD_ZONE_MINIMUM_MOVING_SPEED_EVIDENCE',r
    assert r['points'][1]['actual_policy_u']!=0.
    assert r['points'][1]['truth_steady_vx']==0.
    assert r['validated_condition']=='pure_forward_vy0_wz0'


@pytest.mark.parametrize('failure',['missing_torque','zero_quaternion','missing_source_stamp','unsupported_command','active_inverse','failed_cleanup','wrong_protocol'])
def test_invalid_native_evidence_cannot_select_transition(failure):
    m=api();e,manifest=native_evidence()
    if failure=='missing_torque':e=[r for r in e if r['topic']!='/joint_torques']
    if failure=='zero_quaternion':
        for r in e:
            if r['topic']==m.base.TRUTH_TOPIC:r['data']['quaternion_xyzw']=[0.]*4
    if failure=='missing_source_stamp':
        for r in e:
            if r['topic']==m.base.TRUTH_TOPIC:r['source_s']=None
    if failure=='unsupported_command':
        for r in e:
            if r['topic']==m.base.FINAL_TOPIC and r['sim_s']==10.:r['data']['linear_x']=.041
    if failure=='active_inverse':manifest['m2_inverse_enabled']=True
    if failure=='failed_cleanup':manifest['cleanup']['passed']=False
    if failure=='wrong_protocol':manifest['transition_protocol']['policy_inputs']=[.04]
    result=m.report(e,manifest)
    assert not result['data_valid']
    assert result['interpretation']['classification']=='C_UNSUITABLE_OR_INSUFFICIENT'


def test_zero_flush_error_is_recorded_without_escaping():
    from types import SimpleNamespace as NS
    def bad():raise RuntimeError('injected publish failure')
    error=api().flush_zero(NS(publish_zero=bad),NS(spin_once=lambda **k:None),NS(ok=lambda:True))
    assert 'injected publish failure' in error


def test_default_cli_is_offline_and_does_not_start_processes(monkeypatch,capsys):
    import builtins
    import subprocess
    m=api();real=builtins.__import__
    def guarded(name,*args,**kwargs):
        if name.split('.')[0] in {'rclpy','launch','launch_ros','tf2_ros'}:raise AssertionError('ROS import forbidden')
        return real(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',guarded)
    monkeypatch.setattr(subprocess,'Popen',lambda *a,**k:pytest.fail('process forbidden'))
    assert m.main([])==0
    assert 'route-low-speed-transition-v1' in capsys.readouterr().out


def test_transition_stop_resolution_does_not_call_01_mps_stopped():
    from scripts.velocity_step_core import analyze_run
    m=api();e,manifest=native_evidence()
    for event in e:
        if event['topic']==m.base.TRUTH_TOPIC and 8.<=event['sim_s']<=12.:
            event['data']['linear'][0]=.01
            event['data']['position'][0]=.01*(event['sim_s']-8.)
    r=analyze_run(e,{**manifest,'sequence':manifest['sequence'][:3],'truth_stop_speed_mps':.005})
    assert r['per_speed'][0]['stop']['stop_censored'] is True


def test_transition_repeat_exports_do_not_overwrite_trials(tmp_path):
    m=api();e,manifest=native_evidence();manifest['mode']='route_low_speed_transition'
    analysis=m.analyze_run(e,manifest)
    m.base._write_outputs(tmp_path,manifest,e,analysis)
    files=list((tmp_path/'per-speed').glob('*.json'))
    assert len(files)==9
    assert len([p for p in files if p.name.endswith('speed-0.01.json')])==2


@pytest.mark.parametrize('topic',['/harness/requested','/course_navigation/cmd_vel_raw','/course_navigation/cmd_vel','/path_manager/path_manager_ros/nav_vel'])
def test_contradictory_in_range_pipeline_plateau_is_rejected(topic):
    m=api();e,manifest=native_evidence()
    for r in e:
        if r['topic']==topic and 27.<=r['sim_s']<32.:r['data']['linear_x']=.02
    assert not m.report(e,manifest)['data_valid']


def test_torque_stream_gap_is_not_authenticated_by_one_remaining_sample():
    m=api();e,manifest=native_evidence()
    e=[r for r in e if not (r['topic']=='/joint_torques' and 5.<=r['sim_s']<100.)]
    assert not m.report(e,manifest)['data_valid']


@pytest.mark.parametrize('field',['nav_config','policy'])
def test_report_requires_actual_downstream_config_and_policy_identity(field):
    m=api();e,manifest=native_evidence()
    if field=='nav_config':manifest['source_hashes'][str(m.base.NAV_CONFIG.relative_to(m.ROOT))]='changed'
    else:manifest['policy_sha256']='changed'
    assert not m.report(e,manifest)['data_valid']


def test_execute_exit_fails_when_final_data_report_is_invalid(tmp_path,monkeypatch):
    import json
    m=api();root=tmp_path/'capture'
    def fake_run(args):
        d=root/'owned';d.mkdir(parents=True)
        (d/'manifest.json').write_text('{}');(d/'events.jsonl').write_text('')
        return 0
    monkeypatch.setattr(m.base,'run_experiment',fake_run)
    monkeypatch.setattr(m,'analyze_directory',lambda d:{'data_valid':False,'interpretation':{'classification':'C_UNSUITABLE_OR_INSUFFICIENT'}})
    assert m.main(['--execute','--output-root',str(root)])==1


def test_target_pose_gap_outside_steady_window_rejects_data():
    m=api();e,manifest=native_evidence()
    e=[r for r in e if not (r['topic']==m.base.TRUTH_TOPIC and 27.<=r['sim_s']<28.)]
    assert not m.report(e,manifest)['data_valid']


def test_partial_steady_torque_coverage_is_not_a_valid_point():
    m=api();e,manifest=native_evidence()
    e=[r for r in e if not (r['topic']=='/joint_torques' and 30.<=r['sim_s']<31.)]
    assert not m.report(e,manifest)['data_valid']


def test_minimum_observed_speed_is_actual_truth_not_silently_baseline_subtracted():
    m=api();points=rows([.004,.004,.079,.084,.094,.104,.111,.079,.111])
    result=m.interpret(points,True)
    assert result['classification']=='B_DEAD_ZONE_MINIMUM_MOVING_SPEED_EVIDENCE'
    assert result['minimum_observed_moving_vx']==.079
    assert result['minimum_observed_moving_increment']==pytest.approx(.075)
