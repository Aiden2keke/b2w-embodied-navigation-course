"""Offline E1 request checks never create motion authority or launch a session."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import pytest
from course_navigation import stage4_qualification as qualification

ROOT=Path(__file__).resolve().parents[1]


def reference(path,value):
    path.write_text(json.dumps(value))
    return dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def request(tmp_path):
    zero=reference(tmp_path/'zero.json',dict(kind='stage4_zero_session',zero_motion_smoke_pass=True,
        cleanup=dict(passed=True),motion_authorized=False))
    refs={name:reference(tmp_path/(name+'.json'),dict(review_pending=True))
          for name in ('bootstrap_envelope','independent_abort','local_clearance')}
    return dict(kind='stage4_e1_pulse_request',version=1,mode='e1_forward_pulse',session_id='new-e1',attempt_id='one',
        command=dict(linear_x=.05,linear_y=0.,linear_z=0.,angular_x=0.,angular_y=0.,angular_z=0.),
        pulse_source_s=2.,rest_source_s=5.,rest_wall_s=3.,attempt_source_s=90.,attempt_wall_s=180.,
        scope=dict(valid_from_sim=10.,valid_until_sim=100.,region=[-3.,-3.,3.,3.]),
        zero_only_result=zero,evidence=refs)


def check(value,**kwargs):
    # An explicit assertion gives a feature-missing failure before implementation.
    assert hasattr(qualification,'validate_e1_pulse_request'), 'Missing offline E1 request validation'
    return qualification.validate_e1_pulse_request(value,**kwargs)


def test_structural_success_never_qualifies_motion_or_creates_authority(tmp_path):
    value=request(tmp_path);before=copy.deepcopy(value)
    result=check(value,session_id='new-e1')
    assert result['request_valid'] is True
    assert result['motion_qualified'] is False and result['motion_authorized'] is False
    assert value==before
    assert sorted(p.name for p in tmp_path.iterdir())==['bootstrap_envelope.json','independent_abort.json','local_clearance.json','zero.json']
    with pytest.raises(ValueError,match='UNQUALIFIED_FOR_MOTION'):qualification.require_motion_qualification()


@pytest.mark.parametrize('fault',['mode','session','attempt','extra','command','axis','bool_axis','nan','pulse','rest_source','rest_wall','sim_ceiling','wall_ceiling','scope_time','scope_region','evidence_missing','relative','hash','replaced','zero_failed','cleanup_failed','zero_authorized','huge_number','symlink','bool_version','missing_axis'])
def test_invalid_or_unbound_requests_rejected(tmp_path,fault):
    value=request(tmp_path)
    if fault=='huge_number':value['pulse_source_s']=10**400
    elif fault=='symlink':
        link=tmp_path/'link.json';link.symlink_to(value['evidence']['local_clearance']['path'])
        value['evidence']['local_clearance']['path']=str(link)
    elif fault=='bool_version':value['version']=True
    elif fault=='missing_axis':del value['command']['linear_z']
    elif fault=='mode':value['mode']='segment'
    elif fault=='session':value['session_id']='other'
    elif fault=='attempt':value['attempt_id']=''
    elif fault=='extra':value['motion_authorized']=True
    elif fault=='command':value['command']['linear_x']=.051
    elif fault=='axis':value['command']['angular_y']=.001
    elif fault=='bool_axis':value['command']['linear_y']=False
    elif fault=='nan':value['pulse_source_s']=float('nan')
    elif fault=='pulse':value['pulse_source_s']=2.001
    elif fault=='rest_source':value['rest_source_s']=4.99
    elif fault=='rest_wall':value['rest_wall_s']=2.99
    elif fault=='sim_ceiling':value['attempt_source_s']=90.01
    elif fault=='wall_ceiling':value['attempt_wall_s']=180.01
    elif fault=='scope_time':value['scope']['valid_until_sim']=10.
    elif fault=='scope_region':value['scope']['region']=[0.,0.,0.,1.]
    elif fault=='evidence_missing':del value['evidence']['independent_abort']
    elif fault=='relative':value['evidence']['local_clearance']['path']='relative.json'
    elif fault=='hash':value['evidence']['local_clearance']['sha256']='A'*64
    elif fault=='replaced':Path(value['evidence']['local_clearance']['path']).write_text('{}')
    else:
        path=Path(value['zero_only_result']['path']);zero=json.loads(path.read_text())
        if fault=='zero_failed':zero['zero_motion_smoke_pass']=False
        elif fault=='cleanup_failed':zero['cleanup']['passed']=False
        elif fault=='zero_authorized':zero['motion_authorized']=True
        value['zero_only_result']=reference(path,zero)
    with pytest.raises(ValueError):check(value,session_id='new-e1')


def test_request_scope_cannot_omit_stop_time_or_allow_unbounded_wait(tmp_path):
    value=request(tmp_path);value['scope']['valid_until_sim']=16.
    with pytest.raises(ValueError):check(value,session_id='new-e1')
    value=request(tmp_path);value['attempt_wall_s']=2.
    with pytest.raises(ValueError):check(value,session_id='new-e1')


def test_offline_cli_checks_request_without_environment_or_output(tmp_path):
    import os
    from course_navigation.stage4_control import write_control
    value=request(tmp_path);path=tmp_path/'request.json';write_control(path,value)
    env={key:value for key,value in os.environ.items() if key not in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY')}
    result=subprocess.run([sys.executable,str(ROOT/'scripts/stage4_session.py'),'--inputs',str(path),
        '--check-e1-request'],capture_output=True,text=True,env=env)
    assert result.returncode==0, result.stderr
    assert 'NONZERO remains NO-GO' in result.stdout
    assert not (tmp_path/'lease.json').exists()


def test_reserved_e1_execution_interface_unconditionally_rejects_before_io(tmp_path):
    from course_navigation.stage4_control import write_control
    value=request(tmp_path);path=tmp_path/'request.json';write_control(path,value)
    approval=tmp_path/'qualification.json';write_control(approval,dict(motion_authorized=True,approved=True))
    for source in (path,tmp_path/'missing.json'):
        output=tmp_path/'session'
        result=subprocess.run([sys.executable,str(ROOT/'scripts/stage4_session.py'),'--inputs',str(source),
            '--qualification',str(approval),'--output',str(output),'--execute-e1'],capture_output=True,text=True)
        assert result.returncode==2
        assert 'UNQUALIFIED_FOR_MOTION' in result.stderr
        assert not output.exists()
