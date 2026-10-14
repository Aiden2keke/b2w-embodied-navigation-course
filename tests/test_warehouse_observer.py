"""Synthetic tests only: no ROS node, simulator, action or command publication."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))


def helpers():
    import warehouse_observer_helpers
    return warehouse_observer_helpers


def event(t, value, epoch=0):
    return dict(t=t, epoch=epoch, value=value)


def row(t, x=0., yaw=0., raw=(.2, 0., 0.), final=(.2, 0., 0.), active=True):
    return dict(t=t, epoch=0, active=active, goal_ids=['g'] if active else [],
                truth=[x, 0., yaw], slam=[x, 0., yaw], raw=raw,
                smooth=raw, final=final, force_n=0.)


def test_alignment_is_causal_staleness_bounded_and_epoch_specific():
    h = helpers()
    events = [event(1., 'old'), event(1.2, 'future'), event(1.05, 'other', 1)]
    assert h.align(events, 1.1, 0, .2)['value'] == 'old'
    assert h.align(events, 1.1, 1, .2)['value'] == 'other'
    assert h.align(events, 1.4, 0, .1) is None
    assert h.align([], 1., 0, .2) is None


def test_alignment_does_not_turn_missing_command_into_zero():
    h = helpers()
    assert h.command_state(None) == 'missing'
    assert h.command_state([0, 0, 0]) == 'zero'
    assert h.command_state([0, .1, 0]) == 'motion'
    assert h.command_state([0, 0, .2]) == 'motion'
    assert h.command_state([float('nan'), 0, 0]) == 'missing'


def test_clock_epoch_changes_only_on_backward_clock():
    h = helpers()
    c = h.ClockEpoch()
    assert [c.update(t) for t in [0, 1, 1, .1, .2]] == [0, 0, 0, 1, 1]


def test_stall_requires_active_goal_and_dense_stationary_interval():
    h = helpers()
    assert len(h.stationary_intervals([row(t/10) for t in range(61)])) == 1
    assert not h.stationary_intervals([row(t/10, active=False) for t in range(61)])
    assert not h.stationary_intervals([row(t/10, x=t*.01) for t in range(61)])
    assert not h.stationary_intervals([row(t/10, yaw=t*.02) for t in range(61)])
    assert not h.stationary_intervals([row(t) for t in [0, 1, 5, 6]])


def test_stall_not_created_across_reset_missing_motion_or_goal_replacement():
    h = helpers()
    rows = [row(t/10) for t in range(51)]
    for r in rows[25:]: r['epoch'] = 1
    assert not h.stationary_intervals(rows)
    for r in rows: r['truth'] = None; r['slam'] = None
    assert not h.stationary_intervals(rows)
    rows = [row(t/10) for t in range(51)]
    for r in rows[25:]: r['goal_ids'] = ['new']
    assert not h.stationary_intervals(rows)


def test_classifier_nonzero_final_with_stationary_truth_is_B_candidate():
    h = helpers()
    rows = [row(t/10) for t in range(51)]
    r = h.classify_interval(rows)
    assert 'B' in r['cases']
    assert r['physical_collision_proven'] is False
    assert r['first_causal_layer'] is None


def test_zero_command_chronology_is_A_only_with_prior_motion_evidence():
    h = helpers()
    stopped = [row(t/10+1, raw=(0,0,0), final=(0,0,0)) for t in range(51)]
    assert 'A' not in h.classify_interval(stopped)['cases']
    before = [row(t/10, x=t*.02, raw=(0,0,0), final=(0,0,0)) for t in range(10)]
    assert 'A' in h.classify_interval(stopped, before=before)['cases']


def test_adapter_boundary_mixed_missing_and_localization_disagreement_are_explicit():
    h = helpers()
    rows = [row(t/10, final=(0,0,0)) for t in range(51)]
    assert h.classify_interval(rows)['command_boundary'] == 'raw_motion_final_zero'
    for r in rows: r['final'] = None
    assert 'B' not in h.classify_interval(rows)['cases']
    for i, r in enumerate(rows): r['final']=(.2,0,0); r['slam']=[i*.1,0,0]
    result = h.classify_interval(rows)
    assert result['localization_disagreement'] is True
    assert result['first_causal_layer'] is None


def test_contact_rise_and_logs_are_temporal_support_not_automatic_causality():
    h = helpers()
    rows = [row(t/10+1) for t in range(51)]
    for r in rows[:5]: r['force_n'] = 75
    before = [row(t/10) for t in range(10)]
    logs = [dict(t=1.1, epoch=0, text='No valid trajectories out of 400!'),
            dict(t=5.5, epoch=0, text='Failed to make progress')]
    result = h.classify_interval(rows, before, logs)
    assert set(result['cases']) >= {'B', 'C', 'D', 'E'}
    assert result['physical_collision_proven'] is False
    for r in before: r['force_n'] = None
    assert 'C' not in h.classify_interval(rows, before)['cases']


def grid(w=10, h=10, res=1.):
    return dict(width=w, height=h, resolution=res, origin=[0., 0., 0.], data=[0]*(w*h))


def test_footprint_padding_and_oriented_corner_center_safe_overlap():
    h = helpers()
    poly = h.pad_footprint([[.7,.38],[.7,-.38],[-.7,-.38],[-.7,.38]], .05)
    assert poly[0] == pytest.approx([.75, .43])
    g = grid(); g['data'][3*10+4] = 254
    r = h.footprint_geometry(g, [3.5,3.5,math.pi/4], [[1,.3],[1,-.3],[-1,-.3],[-1,.3]])
    assert r['center_cost'] == 0
    assert r['lethal_overlap'] is True
    assert r['center_safe_footprint_colliding'] is True
    assert r['clearance_to_lethal_m'] == 0


def test_filled_footprint_detects_interior_obstacle_and_separates_inscribed():
    h = helpers(); g = grid(); poly=[[-2,-2],[2,-2],[2,2],[-2,2]]
    g['data'][5*10+5]=254
    assert h.footprint_geometry(g,[4,4,0],poly)['lethal_overlap'] is True
    g['data'][5*10+5]=253
    r=h.footprint_geometry(g,[4,4,0],poly)
    assert r['lethal_overlap'] is False
    assert r['inscribed_overlap'] is True
    assert r['center_safe_footprint_colliding'] is False


def test_geometry_clearance_unknown_out_of_bounds_and_rotated_origin():
    h=helpers(); g=grid(); poly=[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]]
    g['data'][3*10+5]=254
    r=h.footprint_geometry(g,[3.5,3.5,0],poly)
    assert r['clearance_to_lethal_m'] == pytest.approx(1.3)
    g['data'][3*10+3]=255
    assert h.footprint_geometry(g,[3.5,3.5,0],poly)['unknown_overlap'] is True
    assert h.footprint_geometry(g,[0,0,0],poly)['complete'] is False
    g['origin']=[1,2,math.pi/2]
    assert h.footprint_geometry(g,[-2.5,5.5,math.pi/2],poly)['center_cost']==255


def test_geometry_rejects_corrupt_grid_instead_of_claiming_clear():
    h=helpers(); g=grid(); g['data'].pop()
    with pytest.raises(ValueError): h.footprint_geometry(g,[3,3,0],[[-1,-1],[1,-1],[1,1],[-1,1]])


def test_missing_topic_report_distinguishes_no_messages_type_mismatch_and_absence():
    h=helpers(); expected={'/a':'Twist','/b':'Odom','/c':'Clock','/d':'Path'}
    r=h.topic_health(expected, {'/a':['Twist'],'/b':['Wrong'],'/c':['Clock']}, {'/a':5})
    assert r['/a']['state']=='recorded'
    assert r['/b']['state']=='type_mismatch'
    assert r['/c']['state']=='no_messages'
    assert r['/d']['state']=='absent'


def test_A_is_not_reported_when_command_zero_follows_last_displacement():
    h=helpers()
    before=[row(t,x=x,raw=(.2,0,0) if i<3 else (0,0,0))
            for i,(t,x) in enumerate(zip([.1,.3,.5,.7,.9],[0,.1,.2,.2,.2]))]
    stopped=[row(t/10+1,x=.2,raw=(0,0,0),final=(0,0,0)) for t in range(51)]
    assert 'A' not in h.classify_interval(stopped,before)['cases']


def test_closed_footprint_touch_includes_cell_on_lower_grid_edge():
    h=helpers(); g=grid(w=3,h=3);g['data'][3]=254
    r=h.footprint_geometry(g,[1.2,1.5,0],[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]])
    assert r['clearance_to_lethal_m']==pytest.approx(0)
    assert r['lethal_overlap'] is True


def test_action_status_is_unknown_without_live_publisher_not_implicitly_active():
    h=helpers(); status={'status_list':[{'status':2,'goal_info':{'goal_id':{'uuid':[1]*16}}}]}
    assert h.navigation_state(status,False)['active'] is None
    assert h.navigation_state(None,True)['active'] is None
    assert h.navigation_state(status,True)['active'] is True
    assert h.navigation_state({'status_list':[]},True)['active'] is False


def test_published_padded_footprint_is_recovered_without_padding_twice():
    h=helpers(); padded=[[-.75,-.43],[.75,-.43],[.75,.43],[-.75,.43]]
    pose=[3.,4.,math.pi/3]
    world=h.transform_polygon(padded,pose)
    recovered=h.relative_footprint(world,pose)
    for a,b in zip(recovered,padded): assert a==pytest.approx(b)


def test_unknown_cell_touching_lower_edge_prevents_clear_verdict():
    h=helpers();g=grid(w=3,h=3);g['data'][3]=255
    r=h.footprint_geometry(g,[1.2,1.5,0],[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]])
    assert r['unknown_overlap'] is True
    assert r['clear_of_represented_lethal'] is None


def test_A_rejects_an_earlier_zero_interrupted_by_renewed_commands():
    h=helpers();before=[]
    for t,x,v in zip([.1,.3,.5,.7,.9],[0,.1,.2,.2,.2],[0,.2,.2,0,0]):
        before.append(row(t,x=x,raw=(v,0,0)))
    stopped=[row(t/10+1,x=.2,raw=(0,0,0),final=(0,0,0)) for t in range(51)]
    assert 'A' not in h.classify_interval(stopped,before)['cases']


def test_receive_only_progress_log_does_not_prove_E_occurrence_order():
    h=helpers();rows=[row(t/10+1) for t in range(51)]
    logs=[dict(t=5.5,epoch=0,text='Failed to make progress',chronology_valid=False,timestamp_basis='receive_only')]
    result=h.classify_interval(rows,logs=logs)
    assert 'E' not in result['cases']
    assert result['progress_logs']


HEAVY_DWB = {'/evaluation', '/received_global_plan', '/transformed_global_plan', '/local_plan'}
CORE_TOPICS = {
    '/clock', '/goal_pose', '/plan', '/course_navigation/cmd_vel_raw',
    '/course_navigation/cmd_vel', '/path_manager/path_manager_ros/nav_vel',
    '/slam/odom', '/sim/ground_truth/odom', '/collision_plate/force_torque',
    '/navigate_to_pose/_action/status', '/follow_path/_action/status',
    '/behavior_tree_log', '/rosout', '/tf', '/tf_static',
    '/local_costmap/costmap_raw', '/global_costmap/costmap_raw',
    '/local_costmap/published_footprint', '/global_costmap/published_footprint',
    '/network_input_debug',
}


def test_default_light_cli_excludes_heavy_subscriptions():
    import observe_warehouse_narrow_failure as observer
    args = observer.build_parser().parse_args([])
    assert args.dwb_debug is False
    assert set(observer.expected_topics(args.dwb_debug)) == CORE_TOPICS
    assert set(observer.TOPICS) == CORE_TOPICS


def test_opt_in_cli_restores_heavy_subscriptions_and_core_in_both_modes():
    import observe_warehouse_narrow_failure as observer
    args = observer.build_parser().parse_args(['--dwb-debug'])
    assert args.dwb_debug is True
    assert set(observer.expected_topics(args.dwb_debug)) == CORE_TOPICS | HEAVY_DWB
    for mode in [False, True]:
        assert CORE_TOPICS <= set(observer.expected_topics(mode))


def test_health_rates_percentiles_and_fresh_coverage_include_stale_ages():
    h = helpers(); rows = []
    for i, age in enumerate([0., 1., 2., 3., 4.]):
        r = row(i)
        r.update(map_base_tf={'age_s': age}, odom_base_tf={'age_s': age*2},
                 ages={k: .1 for k in ['raw', 'smooth', 'final', 'slam', 'truth', 'force_n']},
                 observed_ages={k: age for k in ['raw', 'smooth', 'final']})
        rows.append(r)
    rows[-1]['raw'] = None; rows[-1]['ages']['raw'] = None
    rows[-1]['force_n'] = None; rows[-1]['ages']['force_n'] = None
    result = h.measurement_health(rows, {'/a': 20}, {'/a':'Twist','/absent':'Odom'}, 10., 5., 'light')
    assert result['wall_duration_s'] == 10.
    assert result['simulation_duration_s'] == 5.
    assert result['sim_wall_ratio'] == .5
    assert result['topics']['/a'] == {'received_messages':20, 'observed_receive_hz_wall':2.}
    assert result['topics']['/absent']['observed_receive_hz_wall'] is None
    assert result['aligned_sample_observed_hz_wall'] == .5
    assert result['aligned_sample_observed_hz_sim'] == 1.
    assert result['tf_age_s']['map_base'] == {'count':5, 'p50':2., 'p95':3.8, 'max':4.}
    assert result['tf_age_s']['odom_base']['p95'] == pytest.approx(7.6)
    assert result['command_age_s']['raw']['max'] == 4.
    assert result['fresh_samples']['raw'] == {'count':4, 'fraction':.8}
    assert result['fresh_samples']['wrench'] == {'count':4, 'fraction':.8}
    assert result['fresh_samples']['truth'] == {'count':5, 'fraction':1.}
    assert result['mode'] == 'light'


def test_health_missing_invalid_and_stale_are_unknown_never_zero_motion():
    h = helpers(); r = row(1.)
    for key in ['raw','smooth','final','slam','truth','force_n']: r[key] = None
    r['observed_ages'] = {'raw': None, 'smooth': float('nan'), 'final': -1.}
    r['map_base_tf'] = None; r['odom_base_tf'] = {'age_s': float('inf')}
    result = h.measurement_health([r], {}, {'/missing':'Twist'}, 1., None, 'light')
    assert result['simulation_duration_s'] is None
    assert result['sim_wall_ratio'] is None
    assert result['tf_age_s']['map_base'] == {'count':0,'p50':None,'p95':None,'max':None}
    assert result['tf_age_s']['odom_base']['max'] is None
    assert all(v['max'] is None for v in result['command_age_s'].values())
    assert all(v == {'count':0, 'fraction':0.} for v in result['fresh_samples'].values())
    assert h.command_state(r['raw']) == 'missing'
    empty = h.measurement_health([], {}, {}, 0., None, 'dwb-debug')
    assert empty['aligned_sample_observed_hz_wall'] is None
    assert empty['fresh_samples']['raw']['fraction'] is None
    assert empty['mode'] == 'dwb-debug'


def test_fresh_health_requires_valid_value_and_bounded_age():
    h=helpers(); r=row(1.)
    r['raw']=[float('nan'),0,0]; r['slam']=[float('nan'),0,0]
    r['ages']={'raw':0.,'smooth':.31,'final':-.1,'slam':.1,'truth':.51,'force_n':.5}
    r['force_n']=0.
    result=h.measurement_health([r],{}, {},1.,1.,'light')
    assert all(result['fresh_samples'][k]['count']==0 for k in ['raw','smooth','final','slam','truth'])
    assert result['fresh_samples']['wrench']['count']==1


def test_observed_age_keeps_stale_history_unknown_in_causal_alignment():
    h=helpers(); events=[event(1.,[.2,0,0]),event(6.,[0,0,0]),event(4.,[0,0,0],1)]
    assert h.observed_age(events,5.,0)==4.
    assert h.align(events,5.,0,.3) is None
    assert h.observed_age(events,5.,2) is None


def test_measurement_clock_sums_within_epochs_and_preserves_absent_clock():
    h=helpers(); c=h.MeasurementClock()
    assert c.duration_s is None
    c.update(10.)
    assert c.duration_s is None
    for t in [12.,12.,1.,4.]: c.update(t)
    assert c.duration_s == 5.
    paused=h.MeasurementClock(); paused.update(3.); paused.update(3.)
    assert paused.duration_s == 0.


def test_final_reports_include_exact_directory_bytes_and_mode(tmp_path):
    import json
    import observe_warehouse_narrow_failure as observer
    (tmp_path/'events.jsonl').write_text('abc')
    health=helpers().measurement_health([],{}, {},1.,None,'light')
    observer.write_final_reports(tmp_path, {'mode':'light'}, {'mode':'light'}, health)
    expected=sum(p.stat().st_size for p in tmp_path.rglob('*') if p.is_file())
    for name in ['manifest.json','summary.json']:
        report=json.loads((tmp_path/name).read_text())
        assert report['measurement_health']['output_directory_bytes']==expected
        assert report['measurement_health']['mode']=='light'


def test_health_collection_does_not_change_stall_classification():
    import copy
    h=helpers(); rows=[row(t/10) for t in range(51)]
    baseline=h.classify_interval(rows)
    intervals=h.stationary_intervals(rows)
    saved=copy.deepcopy(rows)
    h.measurement_health(rows,{}, {},6.,5.,'light')
    assert rows==saved
    assert h.classify_interval(rows)==baseline
    assert h.stationary_intervals(rows)==intervals
    assert baseline['cases']==['B']


def recorded_event(seq, t, topic, value, epoch=0, stamp=None):
    return dict(seq=seq, sim_receive_s=t, epoch=epoch, topic=topic,
                stamp_s=stamp, value=value)


def test_precursor_retains_feasibility_before_stationarity_without_relabeling_D():
    h=helpers(); rows=[row(99.47+i*.1, raw=None, final=(0,0,0)) for i in range(40)]
    events=[recorded_event(i,t,'/rosout',{'msg':text},stamp=1791960649.+i)
            for i,t,text in [(1,95.035,'No valid trajectories out of 419!'),
                             (2,95.34,'Controller patience exceeded'),
                             (3,97.5575,'Controller patience exceeded'),
                             (4,98.0175,'Starting point in lethal space')]]
    twist={'linear':{'x':0.,'y':0.},'angular':{'z':0.}}
    events.append(recorded_event(5,97.555,'/course_navigation/cmd_vel_raw',twist))
    result=h.classify_interval(rows,logs=[dict(t=95.035,epoch=0,text='No valid trajectories')])
    assert result['cases']==[]
    assert hasattr(h,'precursor_chronology'), 'missing explicit precursor chronology'
    precursor=h.precursor_chronology(rows,events)
    assert [e['seq'] for e in precursor['events']]==[1,2,5,3,4]
    assert precursor['events'][0]['offset_from_stationary_s']==pytest.approx(-4.435)
    assert precursor['events'][0]['timestamp_basis']=='simulation_receive_only'
    assert precursor['last_raw_command']['t']==97.555
    assert precursor['last_raw_command']['state']=='zero'
    assert precursor['last_raw_command']['offset_from_stationary_s']==pytest.approx(-1.915)
    assert result['first_causal_layer'] is None
    assert all(r['raw'] is None for r in rows)


def test_precursor_is_bounded_by_epoch_window_and_current_goal_activation():
    h=helpers(); rows=[row(10+i*.1,raw=None,final=(0,0,0)) for i in range(40)]
    rows[0]['goal_ids']=['01'*16]
    status={'status_list':[{'status':2,'goal_info':{'goal_id':{'uuid':[1]*16}}}]}
    events=[recorded_event(1,3.9,'/rosout',{'msg':'No valid trajectories'}),
            recorded_event(2,8.,'/rosout',{'msg':'No valid trajectories'},epoch=1),
            recorded_event(3,8.5,'/rosout',{'msg':'No valid trajectories'}),
            recorded_event(4,9.,'/navigate_to_pose/_action/status',status),
            recorded_event(5,9.5,'/rosout',{'msg':'Controller patience exceeded'}),
            recorded_event(6,10.1,'/rosout',{'msg':'Starting point in lethal space'})]
    assert hasattr(h,'precursor_chronology'), 'missing explicit precursor chronology'
    result=h.precursor_chronology(rows,events)
    assert result['window_start']==9.
    assert [e['seq'] for e in result['events']]==[4,5]
    assert result['last_raw_command'] is None


def test_precursor_preserves_integer_header_precision_and_action_acceptance_stamp():
    h=helpers(); rows=[row(10+i*.1,raw=None,final=(0,0,0)) for i in range(40)]
    rows[0]['goal_ids']=['01'*16]
    status={'status_list':[{'status':2,'goal_info':{'goal_id':{'uuid':[1]*16},
                                                'stamp':{'sec':9,'nanosec':123456789}}}]}
    events=[recorded_event(1,9.,'/navigate_to_pose/_action/status',status),
            recorded_event(2,9.5,'/rosout',{'msg':'No valid trajectories',
                                          'stamp':{'sec':1791960649,'nanosec':186666489}},
                           stamp=1791960649.1866665)]
    result=h.precursor_chronology(rows,events)
    assert result['events'][0].get('goal_stamp')=={'sec':9,'nanosec':123456789}
    assert result['events'][1].get('source_header_stamp')=={'sec':1791960649,'nanosec':186666489}
    assert result['events'][1]['timestamp_basis']=='simulation_receive_only'
