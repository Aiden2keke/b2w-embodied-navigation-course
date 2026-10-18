"""Synthetic evidence exercises the prospective gates, never simulates a live PASS."""
import copy
import struct
from pathlib import Path

import pytest


def evidence():
    from scripts import m2_open_space as m
    from scripts.fit_velocity_calibration import AUTH_FILES, digest
    p=m.plan(); events=[]; x=0.
    for i in range(4801):
        t=i/100; v=next((s['vx'] for s in p['sequence'] if s['start_s']<=t<s['end_s']),0.)
        u=m.m2.inverse(v); x+=v*.01
        def e(topic,data,source=None,child=None):
            return {'topic':topic,'sim_s':t,'source_s':source,'epoch':0,'valid':True,'data':data,'child_frame_id':child}
        events += [e('/clock',{'clock_s':t},t), e(m.base.TRUTH_TOPIC,{'linear':[v,0.,0.], 'angular':[0.,0.,0.],
                    'position':[x,0.,.6],'quaternion_xyzw':[0.,0.,0.,1.]},t,'base_link')]
        if i%2==0:
            obs=[0.]*57;obs[6]=struct.unpack('f',struct.pack('f',u))[0]
            events.append(e('/network_input_debug',{'data':obs}))
        if i%5==0:
            for topic,c in [(m.base.REQUEST_TOPIC,v),(m.base.SHARED_TOPIC,v),(m.base.FINAL_TOPIC,u)]:
                events.append(e(topic,{'linear_x':c,'linear_y':0.,'angular_z':0.}))
            events.append(e(m.m2.DIAGNOSTIC_TOPIC,{'candidate':m.m2.identity(),'status':'accepted','desired':[v,0.,0.],'policy':[u,0.,0.]}))
    manifest={'sequence':p['sequence'],'experiment_start_sim_s':0.,'candidate':m.m2.identity(),
        'source_hashes':{f:digest(m.ROOT/f) for f in AUTH_FILES},'termination':{'status':'completed'},
        'readiness':{'completed':True,'checks':{k:True for k in m.READINESS_CHECKS}},'prospective_plan':m.plan(),'cleanup':{'passed':True},
        'candidate_source_hashes':m.source_hashes(), 'max_gap_s':{'truth':.15,'slam':.35,'command':.075}}
    return events,manifest


def test_report_exists():
    assert Path('scripts/m2_open_space.py').is_file()


def test_all_supported_points_are_authenticated_and_reported():
    from scripts.m2_open_space import report
    e,m=evidence();r=report(e,m)
    assert r['passed'],r
    assert r['data_pass'] and r['tracking_pass'] and r['stability_pass']
    assert [p['desired_physical_vx'] for p in r['points']]==[.12,.15,.2,.25]
    assert all(abs(p['truth_minus_desired'])<1e-10 for p in r['points'])
    assert all(p['final_policy_command']['mean'] is not None for p in r['points'])
    assert all(p['observed_onnx_command']['mean'] is not None for p in r['points'])


@pytest.mark.parametrize('failure',['inactive','unsupported','corruption','missing','cleanup','tracking','drift','identity'])
def test_report_rejects_invalid_or_failed_capture(failure):
    from scripts import m2_open_space as tool
    e,m=evidence()
    if failure=='inactive':e=[r for r in e if r['topic']!=tool.m2.DIAGNOSTIC_TOPIC]
    if failure=='unsupported':e.append({'topic':tool.m2.DIAGNOSTIC_TOPIC,'data':{'status':'unsupported'}})
    if failure=='corruption':
        for r in e:
            if r['topic']=='/network_input_debug':r['data']['data'][7]=.1
    if failure=='missing':e=[r for r in e if r['topic']!=tool.base.FINAL_TOPIC]
    if failure=='cleanup':m['cleanup']['passed']=False
    if failure=='tracking':
        for r in e:
            if r['topic']==tool.base.TRUTH_TOPIC:r['data']['linear'][0]+=.02
    if failure=='drift':
        for r in e:
            if r['topic']==tool.base.TRUTH_TOPIC:r['data']['position'][1]=r['sim_s']*.02
    if failure=='identity':m['candidate']['sha256']='wrong'
    assert not tool.report(e,m)['passed']


def test_start_requires_observed_zero_and_stand_time():
    from scripts.m2_open_space import start_ready
    assert start_ready({'fresh':True},(0.,0.,0.),2.2)
    assert not start_ready({'fresh':True},(.12,0.,0.),3.)
    assert not start_ready({'fresh':True},None,3.)
    assert not start_ready({'fresh':False},(0.,0.,0.),3.)
    assert not start_ready({'fresh':True},(0.,0.,0.),2.)


@pytest.mark.parametrize('topic',['/course_navigation/cmd_vel','/path_manager/path_manager_ros/nav_vel','/network_input_debug'])
def test_stop_phase_axis_or_sign_corruption_rejected(topic):
    from scripts import m2_open_space as tool
    e,m=evidence()
    for row in e:
        if row['topic']==topic and row['sim_s']==10.:
            if topic=='/network_input_debug':row['data']['data'][6]=-.1
            else:row['data']['linear_x']=-.1
    assert not tool.report(e,m)['passed']


@pytest.mark.parametrize('failure',['executor','node','recorder','runtime'])
def test_cleanup_always_reaches_owned_process_cleanup(failure):
    from types import SimpleNamespace as NS
    from scripts.m2_open_space import teardown
    calls=[]
    def call(name):
        calls.append(name)
        if name==failure:raise RuntimeError('injected '+name)
    ex=NS(shutdown=lambda:call('executor'),remove_node=lambda n:calls.append('remove'))
    node=NS(destroy_node=lambda:call('node'))
    recorder=NS(destroy_node=lambda:call('recorder'),close=lambda:calls.append('close'))
    runtime=NS(ok=lambda:True,shutdown=lambda:call('runtime'))
    launches=NS(cleanup=lambda:(calls.append('owned_cleanup') or {'passed':True}))
    result=teardown(ex,node,recorder,runtime,launches,{})
    assert 'owned_cleanup' in calls
    assert result['errors']
    assert result['cleanup']['passed']


@pytest.mark.parametrize('failure',['clock','source_stamps','quaternion','readiness_checks','sequence'])
def test_data_gate_requires_complete_authenticated_capture(failure):
    from scripts import m2_open_space as tool
    e,m=evidence()
    if failure=='clock':e=[r for r in e if r['topic']!='/clock']
    if failure=='source_stamps':
        for r in e:r['source_s']=None
    if failure=='quaternion':
        for r in e:
            if r['topic']==tool.base.TRUTH_TOPIC:r['data']['quaternion_xyzw']=[0.,0.,0.,0.]
    if failure=='readiness_checks':m['readiness']['checks']={'no_navigation_or_smoother':False}
    if failure=='sequence':m['sequence'][1]['end_s']=7.
    assert not tool.report(e,m)['passed']


def test_stopping_clock_gap_cannot_earn_data_pass():
    from scripts import m2_open_space as tool
    e,m=evidence();e=[r for r in e if not (r['topic']=='/clock' and 8.<=r['sim_s']<12.)]
    assert not tool.report(e,m)['data_pass']


def test_missing_early_target_commands_cannot_earn_data_pass():
    from scripts import m2_open_space as tool
    e,m=evidence();e=[r for r in e if not (r['topic']==tool.base.FINAL_TOPIC and 3.<=r['sim_s']<6.)]
    assert not tool.report(e,m)['data_pass']


def test_obvious_truth_sign_reversal_during_target_fails_stability():
    from scripts import m2_open_space as tool
    e,m=evidence()
    for r in e:
        if r['topic']==tool.base.TRUTH_TOPIC and 4.<=r['sim_s']<4.2:r['data']['linear'][0]=-.1
    assert not tool.report(e,m)['stability_pass']
