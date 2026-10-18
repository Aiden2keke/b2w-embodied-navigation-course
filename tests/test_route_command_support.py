import pytest


def test_audit_module_exists():
    from pathlib import Path
    assert Path('scripts/audit_route_command_support.py').is_file()


def test_count_time_domains_no_gap_extrapolation_and_quantiles():
    from scripts.audit_route_command_support import statistics
    rows=[(0.,0.),(.1,.05),(.2,.12),(.3,.2),(.4,.3),(2.,-.1)]
    r=statistics(rows,0.,3.)
    assert r['count']==6
    assert r['counts']=={'zero':1,'low':1,'supported':2,'high':1,'negative':1}
    assert r['time_s']['zero']==pytest.approx(.1)
    assert r['time_s']['low']==pytest.approx(.1)
    assert r['time_s']['supported']==pytest.approx(.2)
    assert r['covered_time_s']==pytest.approx(.4)
    assert r['positive']['min']==.05
    assert r['positive']['max']==.3
    assert r['count_fraction']['low']==pytest.approx(1/6)
    assert r['time_fraction']['low']==pytest.approx(.1/3)


def test_missing_stream_is_missing_not_zero():
    from scripts.audit_route_command_support import statistics
    r=statistics([],0.,5.)
    assert r['count']==0 and r['count_fraction'] is None
    assert r['time_fraction'] is None and r['positive'] is None


def test_old_goal_cancel_does_not_end_new_request_and_missing_first_goal_is_explicit(tmp_path):
    import json
    from scripts.audit_route_command_support import load
    events=[{'topic':'/rosout','sim_receive_s':1.,'value':{'msg':'Begin navigating from current location (0, 0) to (0, 6)'}},
            {'topic':'/rosout','sim_receive_s':5.,'value':{'msg':'Goal succeeded'}},
            {'topic':'/goal_pose','sim_receive_s':6.,'value':{'pose':{'position':{'x':-4.,'y':6.}}}},
            {'topic':'/rosout','sim_receive_s':6.01,'value':{'msg':'Goal canceled'}},
            {'topic':'/rosout','sim_receive_s':6.02,'value':{'msg':'Begin navigating from current location (0, 6) to (-4, 6)'}},
            {'topic':'/rosout','sim_receive_s':10.,'value':{'msg':'Goal failed'}}]
    f=tmp_path/'events.jsonl';f.write_text(''.join(json.dumps(e)+'\n' for e in events))
    r=load(f)
    windows=[w for w in r['windows'] if w['window']!='whole_capture']
    assert len(windows)==2
    assert windows[0]['association']['goal']['xy'] is None
    assert windows[0]['association']['goal']['provenance']=='navigation log only; rounded coordinates not reconstructed'
    assert windows[1]['association']['terminal']['sim_receive_s']==10.
    assert windows[1]['association']['begin']['sim_receive_s']==6.02


def test_equal_stamp_last_receipt_wins_not_maximum_velocity():
    from scripts.audit_route_command_support import statistics
    r=statistics([(0.,.2),(0.,0.),(.1,0.)],0.,.2)
    assert r['time_s']['zero']==pytest.approx(.1)
    assert r['time_s']['supported']==0.
    assert r['counts']['supported']==1 and r['counts']['zero']==2


def test_recovery_boundary_is_preserved_without_claiming_terminal_braking(tmp_path):
    import json
    from scripts.audit_route_command_support import load
    rows=[{'topic':'/rosout','sim_receive_s':1.,'value':{'msg':'Begin navigating from current location (0, 0) to (0, 6)'}},
          {'topic':'/behavior_tree_log','sim_receive_s':2.,'value':{'event_log':[{'node_name':'Spin','current_status':'RUNNING','previous_status':'IDLE'}]}},
          {'topic':'/rosout','sim_receive_s':3.,'value':{'msg':'Goal failed'}}]
    p=tmp_path/'events.jsonl';p.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    r=load(p);names={w['window']:w for w in r['windows']}
    assert names['observed_goal_1_before_first_recovery']['streams']['raw']['window_sim_s']==[1.,2.]
    assert names['observed_goal_1_from_first_recovery']['streams']['raw']['window_sim_s']==[2.,3.]
    assert names['observed_goal_1']['association']['terminal_braking']=='not established by these command/log markers'
