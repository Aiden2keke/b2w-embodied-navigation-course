import json
import math
from pathlib import Path

import pytest

from scripts.velocity_step_core import analyze_run, make_sequence, requested_at, unwrap_yaw
from scripts import run_velocity_steps as runner


def event(topic, t, data, *, source=None, epoch=0, frame='odom'):
    return {'topic': topic, 'kind': 'odometry' if 'odom' in topic else 'twist',
            'sim_s': t, 'source_s': source, 'wall_s': t, 'epoch': epoch,
            'frame_id': frame, 'data': data, 'valid': True}


def odom(t, x, y, yaw, vx, vy=0., wz=0., *, topic='/sim/ground_truth/odom'):
    return event(topic, t, {'position': [x, y, 0.6], 'quaternion_xyzw': [0., 0., math.sin(yaw/2), math.cos(yaw/2)],
                            'linear': [vx, vy, 0.], 'angular': [0., 0., wz]}, source=t)


def twists(topic, values):
    return [event(topic, t, {'linear_x': v, 'linear_y': 0., 'angular_z': 0.}) for t, v in values]


def synthetic_run(request=.2, truth=.24, final=.24, peak=None):
    seq = make_sequence()
    events = []
    for i in range(0, 1501):
        t = i / 100.
        events.append({'topic': '/clock', 'kind': 'clock', 'sim_s': t, 'source_s': t, 'wall_s': t, 'epoch': 0, 'data': {'clock_s': t}, 'valid': True})
    command_times = [i / 20. for i in range(301)]
    events += [event('/harness/requested', t, {'linear_x': request if 3 <= t < 8 else 0.,
                'linear_y': 0., 'angular_z': 0.}) for t in command_times]
    for topic, amplitude in [('/course_navigation/cmd_vel_raw', request),
                             ('/course_navigation/cmd_vel', request),
                             ('/path_manager/path_manager_ros/nav_vel', final)]:
        events += twists(topic, [(t, amplitude if 3 <= t < 8 else 0.) for t in command_times])
    truth_events=[]
    x=0.
    times=[i/10. for i in range(151)]
    for t in times:
        if t < 3: vx=0.
        elif t < 4: vx=truth*(t-3.)
        elif t < 8: vx=truth
        elif t < 9: vx=truth*(9.-t)
        else: vx=0.
        if peak is not None and abs(t-4.5)<1e-8: vx=peak
        if truth_events:
            dt=t-truth_events[-1]['source_s']; x += max(0.,vx)*dt
        truth_events.append(odom(t,x,0.,0.,vx))
    events += truth_events
    events += [odom(t, max(0., min(t-3.,5.))*request, 0., 0., request if 3 <= t < 8 else 0., topic='/slam/odom')
               for t in times]
    return events, {'sequence': seq, 'max_gap_s': {'truth': .15, 'slam': .15, 'command': .075}, 'truth_topic':'/sim/ground_truth/odom','slam_topic':'/slam/odom'}


def test_sequence_has_four_exact_levels_and_48_sim_seconds():
    seq = make_sequence()
    assert [p['vx'] for p in seq if p['phase'] == 'target'] == [.1, .2, .3, .4]
    assert len(seq) == 12 and seq[-1]['end_s'] == 48.
    assert all(p['vy'] == p['wz'] == 0. for p in seq)
    assert [(p['phase'], p['start_s'], p['end_s']) for p in seq[:3]] == [('settle',0.,3.),('target',3.,8.),('stop',8.,12.)]


def test_sequence_rejects_nonfinite_or_nonpositive_values():
    for kwargs in [{'speeds': (.1, .2, .3, float('nan'))}, {'hold_s': 0}, {'settle_s': -1}]:
        with pytest.raises(ValueError): make_sequence(**kwargs)


def test_requested_at_uses_half_open_phases_and_zeros_outside():
    seq = make_sequence()
    assert requested_at(seq, -1.) == (0.,0.,0.)
    assert requested_at(seq, 2.999) == (0.,0.,0.)
    assert requested_at(seq, 3.) == (.1,0.,0.)
    assert requested_at(seq, 8.) == (0.,0.,0.)
    assert requested_at(seq, 15.) == (.2,0.,0.)
    assert requested_at(seq, 48.) == (0.,0.,0.)


def test_yaw_unwrap_uses_shortest_angle():
    result = unwrap_yaw([math.radians(179), math.radians(-179)])
    assert result[1] - result[0] == pytest.approx(math.radians(2))


def test_steady_state_is_time_weighted_and_uses_fixed_final_two_seconds():
    events, manifest = synthetic_run()
    result = analyze_run(events, manifest)['per_speed'][0]
    assert result['truth']['steady_state_mean_vx_mps'] == pytest.approx(.24)
    assert result['truth']['gain_actual_over_requested'] == pytest.approx(2.4)
    assert result['truth']['bias_signed_mps'] == pytest.approx(.14)
    assert result['truth']['bias_absolute_mps'] == pytest.approx(.14)
    assert result['steady_state_window_s'] == [6.,8.]


def test_steady_state_missing_slope_validity_does_not_hide_tail_mean():
    events, manifest = synthetic_run()
    for e in events:
        if e['topic'] == '/sim/ground_truth/odom' and 6. <= e['sim_s'] <= 8.:
            t=e['sim_s']; e['data']['linear'][0] = .2 + (t-6.)*.1
    result=analyze_run(events,manifest)['per_speed'][0]['truth']
    assert result['steady_state_valid'] is False
    assert result['steady_state_mean_vx_mps'] is None
    assert result['tail_mean_vx_mps'] is not None


def test_rise_and_requested_and_tail_overshoot_are_distinct():
    events,manifest=synthetic_run(request=.2,truth=.2,peak=.24)
    truth=analyze_run(events,manifest)['per_speed'][0]['truth']
    assert truth['rise_time_10_90_s'] == pytest.approx(.4)
    assert truth['peak_vx_mps'] == pytest.approx(.24)
    assert truth['overshoot_requested_pct'] == pytest.approx(140.)
    assert truth['overshoot_tail_pct'] == pytest.approx(20.)


def test_stop_detection_requires_continuous_planar_and_yaw_quiet():
    events,manifest=synthetic_run(request=.1,truth=.1,final=.1)
    # Twist stays yawing after translation stops; truth stop must not be declared early.
    for e in events:
        if e['topic']=='/sim/ground_truth/odom' and e['sim_s'] >= 9.:
            yaw=(e['sim_s']-9.)*.1
            e['data']['quaternion_xyzw']=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)]
    result=analyze_run(events,manifest)['per_speed'][0]['stop']
    assert result['truth_effective_stop_s'] is None
    assert result['stop_censored'] is True


def test_stopping_path_distance_and_net_displacement_are_separate():
    events,manifest=synthetic_run(request=.2,truth=.2,final=.2)
    # Add a supported curved out-and-back tail; derived path exceeds net translation.
    events=[e for e in events if not (e['topic']=='/sim/ground_truth/odom' and e['sim_s'] in (8.5,9.))]
    events += [odom(8.5,.9,.1,.1,.1,.2,.1),odom(9.,1.,0.,0.,0.)]
    truth=analyze_run(events,manifest)['per_speed'][0]['truth']
    stop=analyze_run(events,manifest)['per_speed'][0]['stop']
    assert stop['stop_path_distance_m'] >= stop['stop_net_displacement_m']


def test_missing_truth_never_falls_back_to_slam():
    events,manifest=synthetic_run()
    events=[e for e in events if e['topic'] != '/sim/ground_truth/odom']
    result=analyze_run(events,manifest)['per_speed'][0]
    assert result['truth']['steady_state_mean_vx_mps'] is None
    assert result['truth']['missing_reason']
    assert result['slam']['steady_state_mean_vx_mps'] is not None


def test_clock_epoch_crossing_invalidates_analysis():
    events,manifest=synthetic_run()
    events[-1]['epoch']=1
    result=analyze_run(events,manifest)
    assert result['quality']['clock_epoch_crossing'] is True
    assert result['per_speed'][0]['truth']['steady_state_mean_vx_mps'] is None


def test_nonfinite_values_and_missing_streams_are_reported_as_null():
    events,manifest=synthetic_run()
    events += [odom(6.5,float('nan'),0.,0.,float('nan'))]
    events=[e for e in events if e['topic'] != '/course_navigation/cmd_vel_raw']
    result=analyze_run(events,manifest)
    assert result['quality']['nonfinite_event_count'] >= 1
    assert result['per_speed'][0]['pipeline']['raw_coverage_fraction'] == 0.
    assert result['per_speed'][0]['truth']['steady_state_mean_vx_mps'] is not None
    json.dumps(result,allow_nan=False)


def test_requested_and_final_zero_times_are_independent():
    events,manifest=synthetic_run()
    result=analyze_run(events,manifest)['per_speed'][0]
    assert result['stop']['requested_zero_s'] == 8.
    assert result['stop']['final_nav_zero_s'] == pytest.approx(8., abs=.06)
    assert result['pipeline']['final_zero_receipt_s'] == pytest.approx(8., abs=.06)


def test_physical_stop_before_final_zero_retains_signed_time_and_null_distance():
    events,manifest=synthetic_run(request=.1,truth=.1,final=.1)
    for e in events:
        if e['topic']=='/path_manager/path_manager_ros/nav_vel' and e['sim_s'] >= 8.:
            e['data']['linear_x']=.1 if e['sim_s'] < 9.5 else 0.
    stop=analyze_run(events,manifest)['per_speed'][0]['stop']
    assert stop['truth_effective_stop_s'] < stop['final_nav_zero_s']
    assert stop['final_zero_to_truth_stop_s'] < 0.
    assert stop['final_zero_to_stop_path_distance_m'] is None
    assert stop['final_zero_stop_reason']=='stop_precedes_reference'


def test_command_integrals_apply_freshness_and_integrate_actual_final_command():
    events,manifest=synthetic_run(request=.2,truth=.2,final=.15)
    result=analyze_run(events,manifest)['per_speed'][0]['pipeline']
    assert result['final_command_integral_m'] <= .15 * 5 + 1e-9
    assert result['final_command_integral_m'] >= 0.
    assert result['final_coverage_fraction'] > .9


def test_requested_publisher_receipt_rate_is_reported():
    events,manifest=synthetic_run()
    events += [{'topic':'/harness/requested','kind':'twist','sim_s':t,'wall_s':t,'epoch':0,
                'data':{'linear_x':.1,'linear_y':0.,'angular_z':0.},'valid':True} for t in [3.,3.05,3.10]]
    result=analyze_run(events,manifest)['per_speed'][0]['pipeline']
    assert result['requested_receive_hz'] == pytest.approx(20.)


def test_runner_derives_20_hz_from_unchanged_controller_and_smoother_baseline():
    assert runner.request_rate_hz() == 20.


def test_readiness_gate_rejects_missing_truth_instrumentation_and_any_failed_gate():
    checks={key:True for key in runner.READINESS_KEYS}
    assert runner.readiness_complete(checks)
    checks['fresh_ground_truth_instrumentation']=False
    assert not runner.readiness_complete(checks)
    checks['fresh_ground_truth_instrumentation']=True
    checks['clock_epoch_stable']=False
    assert not runner.readiness_complete(checks)


def test_publisher_ownership_and_clock_rollback_contracts():
    assert runner.publisher_contract_ok({'raw':{'matches':True},'final':{'matches':True}})
    assert not runner.publisher_contract_ok({'raw':{'matches':True},'final':{'matches':False}})
    assert not runner.clock_rolled_back(None,0.)
    assert not runner.clock_rolled_back(1.,1.)
    assert runner.clock_rolled_back(1.,.9)


def test_fixture_is_deterministic_open_and_asymmetric(tmp_path):
    first=runner.generate_fixture_world(runner.SOURCE_WORLD,tmp_path/'first.sdf')
    second=runner.generate_fixture_world(runner.SOURCE_WORLD,tmp_path/'second.sdf')
    assert first['source_sha256'] == runner.sha256(runner.SOURCE_WORLD)
    assert first['generated_sha256'] == second['generated_sha256']
    assert first['generated_sha256'] != first['source_sha256']
    assert first['floor']['diffuse_rgba'] == '0.6 0.65 0.7 1'
    assert len([g for g in first['geometry'] if 'landmark' in g['name']]) == 6
    assert first['spawn']['yaw_rad'] == 0.
    assert first['landmarks_are_control_inputs'] is False


def test_dry_run_has_no_ros_process_or_motion_side_effects(capsys, monkeypatch):
    import builtins
    original_import=builtins.__import__
    def import_guard(name,*args,**kwargs):
        if name=='rclpy' or name.startswith('rclpy.'):
            pytest.fail('dry-run imported the ROS runtime')
        return original_import(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',import_guard)
    monkeypatch.setattr(runner.subprocess,'Popen',lambda *a,**k: pytest.fail('dry-run launched a child'))
    assert runner._dry_run(gui=True) == 0
    output=json.loads(capsys.readouterr().out)
    assert output['ros_imported'] is False
    assert output['children_launched'] is False
    assert output['motion_published'] is False
    assert output['request_hz'] == 20.


def test_offline_analyze_preserves_raw_capture_bytes(tmp_path, monkeypatch):
    events,manifest=synthetic_run()
    manifest['experiment_start_sim_s']=0.
    run=tmp_path/'run'; run.mkdir()
    raw=run/'events.jsonl'
    raw.write_text(''.join(json.dumps(e,allow_nan=False)+'\n' for e in events))
    (run/'manifest.json').write_text(json.dumps(manifest,allow_nan=False))
    before=runner.sha256(raw)
    monkeypatch.setattr(runner.subprocess,'Popen',lambda *a,**k: pytest.fail('offline analysis launched a child'))
    assert runner._analyze_dir(run) == 0
    assert runner.sha256(raw) == before
    assert all((run/name).is_file() for name in ['timeseries.csv','summary.json','summary.csv','summary.md'])
    assert len(list((run/'per-speed').glob('*.json'))) == 4


def test_runner_cleanup_interrupts_only_owned_processes(tmp_path):
    import os
    import signal
    import subprocess
    import sys
    token='step-test-'+str(tmp_path.name)
    launches=runner._OwnedLaunches(tmp_path,token,os.environ.copy())
    owned=launches.start('owned',[sys.executable,'-c','import time; time.sleep(60)'])
    unrelated=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'])
    try:
        report=launches.cleanup()
        assert report['passed'] is True
        assert report['signal_order'][0]=='INT'
        assert owned.poll() is not None
        assert unrelated.poll() is None
    finally:
        if unrelated.poll() is None:
            unrelated.send_signal(signal.SIGTERM)
            unrelated.wait(timeout=5)


def test_analysis_uses_actual_nonzero_experiment_clock_origin():
    events, manifest = synthetic_run(truth=.1, final=.1)
    for e in events:
        for key in ('sim_s', 'source_s'):
            if e.get(key) is not None:
                e[key] += 40.
    manifest['experiment_start_sim_s'] = 40.
    row = analyze_run(events, manifest)['per_speed'][0]
    assert row['target_window_s'] == [43., 48.]
    assert row['truth']['steady_state_mean_vx_mps'] == pytest.approx(.1)
    assert row['stop']['requested_zero_s'] == 48.


def test_missing_requested_zero_does_not_crash_or_fabricate_stop_delay():
    events, manifest = synthetic_run(truth=.1, final=.1)
    events = [e for e in events if e['topic'] != '/harness/requested']
    stop = analyze_run(events, manifest)['per_speed'][0]['stop']
    assert stop['truth_effective_stop_s'] is not None
    assert stop['requested_zero_to_truth_stop_s'] is None


def test_ideal_supported_response_can_settle_without_next_phase_sample():
    events, manifest = synthetic_run(truth=.1, final=.1)
    truth = analyze_run(events, manifest)['per_speed'][0]['truth']
    assert truth['settling_censored'] is False
    assert truth['settling_time_s'] == pytest.approx(.8)


def test_readiness_diagnostics_never_replace_required_gate_semantics():
    checks = {key: True for key in runner.READINESS_KEYS}
    checks['publisher_audit'] = {}
    assert runner.readiness_complete(checks)
    del checks[runner.READINESS_KEYS[0]]
    assert not runner.readiness_complete(checks)


def test_smoke_mode_is_exposed_without_enabling_sequence():
    args = runner.make_parser().parse_args(['--readiness-smoke', '--no-gui'])
    assert args.readiness_smoke is True


def test_real_ros_executor_services_runner_and_recorder_zero_only(tmp_path, monkeypatch):
    """Removing recorder from the owned executor loses clock/raw callbacks."""
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, JointState
    from scripts.velocity_step_recorder import create_recorder
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=[], domain_id=195)
    recorder = node = driver = executor = None
    try:
        recorder = create_recorder(tmp_path)
        node = runner._make_ros_node(tmp_path, recorder)()
        executor = runner._make_executor(node, recorder)
        driver = Node('velocity_step_test_driver')
        clock = driver.create_publisher(Clock, '/clock', qos_profile_sensor_data)
        imu_pub = driver.create_publisher(Imu, '/sim/raw/imu', qos_profile_sensor_data)
        joint_pub = driver.create_publisher(JointState, '/sim/raw/joint_states', qos_profile_sensor_data)
        deadline = time.monotonic() + 4.
        i = 0
        while time.monotonic() < deadline:
            i += 1
            msg = Clock(); msg.clock.sec = 10; msg.clock.nanosec = i * 1000000
            clock.publish(msg)
            imu = Imu(); imu.header.stamp = msg.clock; imu.orientation.w = 1.; imu_pub.publish(imu)
            joint = JointState(); joint.header.stamp = msg.clock; joint.name = runner.JOINT_NAMES
            joint.position = [0.] * 16; joint.velocity = [0.] * 16; joint_pub.publish(joint)
            for _ in range(12): executor.spin_once(timeout_sec=.005)
            if recorder.counts.get('/clock', 0) >= 3 and recorder.counts.get(runner.RAW_TOPIC, 0) >= 3 and node.first_joint_sim is not None:
                break
        assert node.clock_s is not None
        assert node.latest_imu is not None
        assert node.first_joint_sim is not None
        assert recorder.counts.get('/clock', 0) >= 3
        assert recorder.counts.get(runner.RAW_TOPIC, 0) >= 3
        assert recorder.counts.get('/sim/raw/imu', 0) > 0
        assert recorder.counts.get('/sim/raw/joint_states', 0) > 0
        assert node.experiment_start is None
        recorder.stream.flush()
        events = runner._read_events(tmp_path / 'events.jsonl')
        requests = [e for e in events if e['topic'] == '/harness/requested']
        assert len(requests) >= 3
        assert all(e['sim_s'] is not None and e['data']['linear_x'] == 0. for e in requests)
        assert set(executor.get_nodes()) == {node, recorder}
    finally:
        if executor is not None: executor.shutdown()
        for obj in (driver, node, recorder):
            if obj is not None: obj.destroy_node()
        if recorder is not None: recorder.close()
        if rclpy.ok(): rclpy.shutdown()


def test_zero_confirmation_accepts_irregular_supported_receipts():
    from scripts.velocity_step_core import _first_zero
    assert _first_zero([(8.,0.),(8.06,0.),(8.12,0.),(8.18,0.),(8.24,0.)],8.,9.,max_gap=.075) == 8.


def test_body_lateral_twist_is_read_from_linear_vector():
    events, manifest = synthetic_run(truth=.1, final=.1)
    for e in events:
        if e['topic'] == '/sim/ground_truth/odom': e['data']['linear'][1] = .03
    row = analyze_run(events,manifest)['per_speed'][0]
    assert row['truth']['peak_abs_vy_mps'] == pytest.approx(.03)


def test_indexed_timeseries_lookup_keeps_native_time_bases_and_latest_tie():
    rows = [event('odom', 12., {'vx':1}, source=10.),
            event('odom', 13., {'vx':2}, source=11.),
            event('odom', 13., {'vx':3}, source=11.)]
    index = runner._index_events(rows)
    assert runner._latest(index,'odom',11.) is None
    assert runner._latest(index,'odom',11.,source_time=True)['data']['vx'] == 3
    assert runner._latest(index,'odom',12.)['data']['vx'] == 1
    assert runner._latest(index,'odom',13.)['data']['vx'] == 3
    assert runner._latest(index,'missing',100.) is None


def test_interrupt_keeps_ros_context_alive_for_zero_flush(tmp_path):
    import os
    import subprocess
    import sys
    env = dict(os.environ, ROS_DOMAIN_ID='196', ROS_LOG_DIR=str(tmp_path/'ros-logs'))
    code = '''
import os, signal, json, rclpy
from scripts.run_velocity_steps import _initialize_ros
previous = _initialize_ros()
try:
    os.kill(os.getpid(), signal.SIGINT)
except KeyboardInterrupt:
    print(json.dumps({'context_alive_for_flush': rclpy.ok()}))
finally:
    if rclpy.ok(): rclpy.shutdown()
    for sig, handler in previous.items(): signal.signal(sig, handler)
'''
    result = subprocess.run([sys.executable,'-c',code],cwd=runner.ROOT,env=env,capture_output=True,text=True,timeout=10)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)['context_alive_for_flush'] is True
