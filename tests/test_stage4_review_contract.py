"""Review regressions: formal diagnostics and pre-attempt rejection side effects."""
import importlib.util
import json
from pathlib import Path
import sys
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
spec=importlib.util.spec_from_file_location('route_cli',ROOT/'scripts/run_candidate_route.py')
cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)


@pytest.mark.parametrize('mode',['segment','critical','full'])
def test_diagnostic_practical_pass_is_not_a_formal_exit_success(mode):
    report=dict(result='PRACTICAL_ROUTE_PASS_SAFETY_UNCERTAIN',practical_route_pass=True,
                full_route_pass=False,segment_pass=False,cleanup_passed=True)
    assert cli.execution_exit_code(report,mode)==3


@pytest.mark.parametrize('mode,key',[('segment','segment_pass'),('critical','segment_pass'),('full','full_route_pass')])
def test_exit_requires_scope_matching_formal_pass_and_clean_result(mode,key):
    report=dict(result='FULL_ROUTE_PASS' if key=='full_route_pass' else 'SEGMENT_PASS',
                practical_route_pass=True,full_route_pass=False,segment_pass=False,cleanup_passed=True,error=None)
    report[key]=True
    assert cli.execution_exit_code(report,mode)==0
    report['error']='cleanup failed'
    assert cli.execution_exit_code(report,mode)!=0


@pytest.mark.parametrize('fault',['missing','malformed','wrong_session','wrong_selection','invalid_rate'])
def test_invalid_deadline_never_consumes_execution_lock(tmp_path,monkeypatch,fault):
    from stage4_route import contract,motion_qualification,ros_transport
    selection=tmp_path/'selection.json';selection.write_text('{}')
    session=tmp_path/'session';session.mkdir()
    (session/'capture.json').write_text(json.dumps(dict(session_id='one',selection_sha256=contract.digest(selection))))
    metrics=dict(session_id='one',selection_sha256=contract.digest(selection),length_m=1.,
                 rotation_rad=0.,achieved_speed_lower_bound_m_s=.1,achieved_yaw_rate_lower_bound_rad_s=.1)
    path=tmp_path/'deadline.json'
    if fault=='malformed':path.write_text('{')
    elif fault!='missing':
        if fault=='wrong_session':metrics['session_id']='other'
        if fault=='wrong_selection':metrics['selection_sha256']='b'*64
        if fault=='invalid_rate':metrics['achieved_speed_lower_bound_m_s']=0.
        path.write_text(json.dumps(metrics))
    # Bypass only the independent motion interlock to reach pre-ROS validation.
    # Real validation and exclusive filesystem writes remain under test.
    monkeypatch.setattr(motion_qualification,'require_motion_qualification',lambda:None)
    monkeypatch.setattr(cli,'load_selection',lambda p:{})
    monkeypatch.setattr(ros_transport,'RosTransport',lambda *a,**k:pytest.fail('invalid inputs constructed ROS transport'))
    args=['run','--selection',str(selection),'--session',str(session),'--mode','segment','--segment','start-g1','--execute-user-run']
    if fault!='missing':args+=['--deadline-evidence',str(path)]
    assert cli.main(args)==2
    assert not (session/'execution-lock.json').exists()


def test_successful_action_and_g1_hold_without_physics_stays_diagnostic():
    from test_segment_evidence import goals,sample
    from stage4_route.runner import run_route
    class Observed:
        audit={};failure=None
        def __init__(self):self.i=0
        def ready(self,*a):return sample(1.,-1.)
        def send(self,g,e):assert [v['name'] for v in g]==['G1']
        def tick(self):
            i=self.i;self.i+=1
            s=sample(1.+i*.1,min(0.,-1.+i*.1))
            return dict(sample=s,**({'status':4} if i==12 else {}))
        def sim(self):return 1.+(self.i-1)*.1
        def close(self):return True
    report=run_route(dict(goals=goals(),execution={}),dict(session_id='one'),Observed(),count=1)
    assert report['practical_route_pass'] and report['terminal_stop_confirmed']
    assert not report['full_route_pass'] and not report['segment_pass'] and not report['safety_complete']
    assert cli.execution_exit_code(report,'segment')==3


@pytest.mark.parametrize('fault',['missing_registration','bad_registration','missing_deployment','bad_deployment','wrong_deployment_session'])
def test_invalid_registration_or_deployment_never_consumes_attempt(tmp_path,monkeypatch,fault):
    import time
    from stage4_route import contract,motion_qualification,ros_transport
    selection=tmp_path/'selection.json';selection.write_text('{}');session=tmp_path/'session';session.mkdir()
    capture=dict(session_id='one',selection_sha256=contract.digest(selection),sim=10.,wall=time.time(),graph={},live_map_sha256='a'*64)
    (session/'capture.json').write_text(json.dumps(capture))
    metrics=dict(session_id='one',selection_sha256=contract.digest(selection),length_m=1.,rotation_rad=0.,
        achieved_speed_lower_bound_m_s=.1,achieved_yaw_rate_lower_bound_rad_s=.1)
    deadline=tmp_path/'deadline.json';deadline.write_text(json.dumps(metrics))
    pairs=dict(saved=[[0,0],[5,0],[0,5],[5,5]],live=[[0,0],[5,0],[0,5],[5,5]])
    if fault!='missing_registration':
        (session/'registration.json').write_text('{' if fault=='bad_registration' else json.dumps(contract.registration(capture,pairs,alignment_p95=0.)))
    guard=session/'guard.json';guard.write_text(json.dumps(dict(session_id='other')))
    if fault in ('bad_deployment','wrong_deployment_session'):
        (session/'deployment.json').write_text('{' if fault=='bad_deployment' else json.dumps(dict(
            kind='stage4_session_deployment',version=1,session_id='other',guard_manifest=str(guard),guard_manifest_sha256=contract.digest(guard))))
    monkeypatch.setattr(motion_qualification,'require_motion_qualification',lambda:None)
    monkeypatch.setattr(cli,'load_selection',lambda p:{})
    monkeypatch.setattr(ros_transport,'RosTransport',lambda *a,**k:pytest.fail('invalid inputs constructed ROS transport'))
    assert cli.main(['run','--selection',str(selection),'--session',str(session),'--mode','segment','--segment','start-g1',
        '--execute-user-run','--deadline-evidence',str(deadline)])==2
    assert not (session/'execution-lock.json').exists()
