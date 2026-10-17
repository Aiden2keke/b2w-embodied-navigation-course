"""Full route contract exercised offline; no ROS initialization or navigation."""
import builtins
import copy
import json
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import run_warehouse_route as route
from test_warehouse_route import Transport, XY, snapshot

ORDER = ['G1', 'G2', 'G3', 'G4', 'G5']
LEGS = ['Start-G1', 'G1-G2', 'G2-G3', 'G3-G4', 'G4-G5']


class FullTransport(Transport):
    def __init__(self, statuses=(4,) * 5, offsets=(0.,) * 5, change=None, fail=None):
        super().__init__(statuses, offsets, fail)
        self.change = change or (lambda i, m: None)

    def send_and_wait(self, pose, on_sent):
        result = super().send_and_wait(pose, on_sent)
        m = result['metrics']
        sec = m['goal_pose']['header']['stamp']['sec']
        m['start_map_base_link']['header']['stamp']['sec'] = sec
        m['end_map_base_link']['header']['stamp']['sec'] = sec + 1
        self.change(len(self.sent) - 1, m)
        return result


def run(live, checkpoint=lambda r: None):
    return route.run_full(route.load_route(), execute=True,
                          transport_factory=lambda: live, checkpoint=checkpoint)


def test_full_goals_frozen_order_position_heading_and_honest_provenance():
    config = route.load_route()
    original = copy.deepcopy(config)
    goals = route.full_goals(config)
    assert config == original
    assert [g['name'] for g in goals] == ORDER
    assert [tuple(g['pose']['pose']['position'][k] for k in 'xy') for g in goals] == XY
    assert [g['orientation_provenance'] for g in goals] == [
        'canonical_clicked', 'canonical_clicked', 'runner_derived_transit',
        'runner_derived_transit', 'runner_derived_transit']
    for goal in goals[:2]:
        assert goal['yaw_rad'] == -2.8966601449198057
        assert goal['pose']['pose']['orientation'] == {
            'x': 0., 'y': 0., 'z': -0.9925103760697063, 'w': 0.12216035933955931}
    # Independently hand-checked bearings, no expectation computed by runner.
    assert [g['yaw_rad'] for g in goals[2:]] == pytest.approx(
        [0.6661992265321878, -0.7645785058120054, -0.7645785058120054], abs=1e-9)
    for g in goals:
        q = g['pose']['pose']['orientation']
        assert q['z'] == math.sin(g['yaw_rad'] / 2)
        assert q['w'] == math.cos(g['yaw_rad'] / 2)


def test_full_success_only_after_fifth_goal_and_cleanup():
    live = FullTransport()
    checkpoints = []
    report = run(live, lambda r: checkpoints.append(copy.deepcopy(r)))
    assert [tuple(s['pose']['position'][k] for k in 'xy') for s in live.sent] == XY
    assert report['route_mode'] == 'full' and report['route_order'] == ORDER
    assert report['completed_waypoints'] == ORDER
    assert report['failed_leg'] is None and report['no_retry'] is True
    assert report['full_route_pass'] is report['success'] is True
    assert report['narrow_leg_pass'] is True and live.closed
    for state in checkpoints:
        if state['full_route_pass']:
            assert state['completed_waypoints'] == ORDER and state['cleanup']['passed']
        if len(state['sent_poses']) < 5:
            assert state['full_route_pass'] is False


@pytest.mark.parametrize('failed', range(5))
@pytest.mark.parametrize('status', [5, 6, None])
def test_failure_each_leg_stops_without_retry_or_skip(failed, status):
    live = FullTransport(statuses=(4,) * failed + (status,) + (4,) * (4 - failed))
    report = run(live)
    assert len(live.sent) == failed + 1
    assert report['completed_waypoints'] == ORDER[:failed]
    assert report['failed_leg'] == LEGS[failed]
    assert report['full_route_pass'] is report['success'] is False
    assert report['narrow_leg_pass'] is (failed >= 2)
    assert report['segments'][-1]['failure_reason']
    assert live.closed and report['cleanup']['passed']


@pytest.mark.parametrize('failed', range(5))
@pytest.mark.parametrize('defect', ['manager', 'missing_tf', 'stale_tf', 'future_tf', 'evaluation'])
def test_every_advancement_gate_applies_to_every_leg(failed, defect):
    def change(i, m):
        if i != failed:
            return
        if defect == 'manager': m['success'] = False
        if defect == 'missing_tf': m['start_map_base_link'] = None
        if defect == 'stale_tf': m['end_map_base_link']['age_s'] = .500001
        if defect == 'future_tf': m['end_map_base_link']['age_s'] = -.001
        if defect == 'evaluation': m['end_map_base_link']['transform']['translation']['x'] += .050001
    live = FullTransport(change=change)
    report = run(live)
    assert len(live.sent) == failed + 1
    assert report['completed_waypoints'] == ORDER[:failed]
    assert report['failed_leg'] == LEGS[failed] and not report['full_route_pass']
    assert report['narrow_leg_pass'] is (failed >= 2)


@pytest.mark.parametrize('offset, passed', [(.049999, True), (.05, True), (.050001, False)])
def test_position_staging_boundary_inclusive_and_no_yaw_gate(offset, passed):
    goal = copy.deepcopy(route.full_goals(route.load_route())[2])
    goal['pose']['pose']['position'].update(x=0., y=0.)
    metrics = {'nav2_status': 4, 'success': True, 'end_reason': 'succeeded',
               'start_map_base_link': snapshot(), 'end_map_base_link': snapshot(offset, 0.)}
    # Endpoint identity quaternion differs from transit yaw. Position alone gates.
    result = route.evaluate_full_goal(goal, metrics, .05)
    assert result['success'] is passed
    assert result['endpoint_error_m'] == offset


@pytest.mark.parametrize('mutation', ['missing', 'duplicate', 'order', 'position', 'nan',
                                      'orientation', 'coincident', 'missing_goals'])
def test_malformed_route_rejected_before_transport(mutation):
    config = route.load_route()
    if mutation == 'missing': config['goals'].pop()
    if mutation == 'duplicate': config['goals'][3] = copy.deepcopy(config['goals'][2])
    if mutation == 'order': config['goals'][2:4] = reversed(config['goals'][2:4])
    if mutation == 'position': del config['goals'][2]['position']
    if mutation == 'nan': config['goals'][2]['position'][0] = float('nan')
    if mutation == 'orientation': config['goals'][2]['orientation'] = 'canonical_clicked'
    if mutation == 'coincident': config['goals'][3]['position'] = config['goals'][2]['position'][:]
    if mutation == 'missing_goals': del config['goals']
    with pytest.raises(ValueError):
        route.run_full(config, execute=True, transport_factory=lambda: pytest.fail('Malformed route sent'))


@pytest.mark.parametrize('failure', [RuntimeError('missing metrics'), KeyboardInterrupt()])
def test_partial_failure_preserves_sent_pose_and_schema(failure):
    live = FullTransport(fail=failure)
    report = run(live)
    assert len(report['sent_poses']) == len(report['segments']) == 1
    assert report['segments'][0]['sent_pose'] == live.sent[0]
    assert report['segments'][0]['failure_reason']
    assert report['failed_leg'] == 'Start-G1' and not report['full_route_pass']
    assert live.closed


def test_cleanup_failure_never_passes_full_route():
    class BrokenCleanup(FullTransport):
        def close(self): return {'passed': False}
    report = run(BrokenCleanup())
    assert report['completed_waypoints'] == ORDER
    assert not report['full_route_pass'] and not report['success']


def test_exact_pose_association_still_required():
    def change(i, m):
        m['goal_pose'] = copy.deepcopy(m['goal_pose'])
        m['goal_pose']['header']['stamp']['nanosec'] += 1
    report = run(FullTransport(change=change))
    assert len(report['sent_poses']) == 1 and not report['full_route_pass']
    assert 'match' in report['error']


def test_clock_epoch_change_between_legs_cannot_pass():
    def change(i, m):
        if i == 1:
            m['start_map_base_link']['header']['stamp']['sec'] = 1
            m['end_map_base_link']['header']['stamp']['sec'] = 2
    report = run(FullTransport(change=change))
    assert len(report['sent_poses']) == 2 and not report['narrow_leg_pass']
    assert not report['full_route_pass']


def test_full_artifact_schema_and_immutable_canonical_source(tmp_path, monkeypatch):
    original = route.ROUTE.read_bytes()
    live = FullTransport()
    monkeypatch.setattr('plot_warehouse_route.render_run', lambda p: {'primary': {'status': 'unavailable'},
                        'images': [], 'composite_image_path': None})
    assert route.main(['--mode', 'full', '--execute', '--output-root', str(tmp_path)],
                      transport_factory=lambda: live) == 0
    report = json.loads(next(tmp_path.glob('*/report.json')).read_text())
    assert route.ROUTE.read_bytes() == original
    assert report['route_mode'] == 'full' and report['route_order'] == ORDER
    assert report['total_route_duration_wall_s'] >= 0
    assert report['total_route_duration_sim_s'] is not None
    assert report['manifest']['production_calibration']['state'] == 'no_production_inverse'
    assert report['manifest']['production_calibration']['sha256']
    assert report['manifest']['config_sha256']['course_navigation/config/nav2.yaml']
    for i, leg in enumerate(report['segments']):
        assert leg['leg_id'] == LEGS[i] and leg['to'] == ORDER[i]
        assert leg['from'] == ('Start' if i == 0 else ORDER[i-1])
        assert leg['sent_pose'] == live.sent[i]
        assert leg['action_terminal_status'] == 4
        assert leg['goal_manager_terminal_status'] == 'succeeded'
        assert leg['goal_start_sim_time_s'] is not None and leg['goal_end_sim_time_s'] is not None
        assert leg['goal_start_wall_time_s'] <= leg['goal_end_wall_time_s']
        assert leg['boundary_localization']['start']['fresh']
        assert leg['boundary_localization']['end']['fresh']
        assert leg['end_map_base_link'] is not None
        assert leg['endpoint_error_m'] == 0.
        assert leg['success'] and leg['failure_reason'] is None


def test_full_explicit_dry_run_no_ros_import_or_transport(tmp_path, monkeypatch):
    original = builtins.__import__
    def guarded(name, *args, **kwargs):
        if name.split('.')[0] in {'rclpy', 'geometry_msgs', 'nav2_msgs', 'tf2_ros'}:
            pytest.fail(f'Dry run imported {name}')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', guarded)
    assert route.main(['--mode', 'full', '--dry-run', '--output-root', str(tmp_path)],
                      transport_factory=lambda: pytest.fail('Dry run constructed transport')) == 0
    report = json.loads(next(tmp_path.glob('*/report.json')).read_text())
    assert report['sent_poses'] == report['segments'] == report['completed_waypoints'] == []
    assert not report['full_route_pass'] and not report['narrow_leg_pass']
    assert [g['name'] for g in report['planned_goals']] == ORDER
    with pytest.raises(SystemExit):
        route.main(['--mode', 'full', '--execute', '--dry-run'])


def test_unpublished_preparation_keeps_previous_segments_renderable(tmp_path, monkeypatch):
    import plot_warehouse_route
    class PreparationFailure(FullTransport):
        def send_and_wait(self, pose, on_sent):
            if len(self.sent) == 2:
                prepared = copy.deepcopy(pose)
                prepared['header']['stamp']['sec'] = 13
                on_sent(prepared, 'prepared')
                raise RuntimeError('Publish failed')
            return super().send_and_wait(pose, on_sent)
    # Exercise real offline renderer. No map capture means unavailable panels,
    # while previously sent segments must still be enumerated and retained.
    live = PreparationFailure()
    assert route.main(['--mode', 'full', '--execute', '--output-root', str(tmp_path)],
                      transport_factory=lambda: live) == 1
    report = json.loads(next(tmp_path.glob('*/report.json')).read_text())
    assert report['visualization']['status'] != 'error'
    overlay = json.loads(next(tmp_path.glob('*/overlays/trajectory-overlays.json')).read_text())
    assert overlay['executed_waypoints'] == ['G1', 'G2']
    assert report['failed_leg'] == 'G2-G3' and report['narrow_leg_pass']
    assert report['segments'][-1]['publish_state'] == 'prepared'
    assert len(report['sent_poses']) == 2


def test_session_invalidated_during_cleanup_cannot_pass_full():
    from types import SimpleNamespace
    class ResetOnClose(FullTransport):
        def __init__(self):
            super().__init__()
            self.evidence = SimpleNamespace(epoch_invalid=False)
        def close(self):
            self.evidence.epoch_invalid = True
            return super().close()
    report = run(ResetOnClose())
    assert report['full_route_pass'] is False
    assert report['stop_reason'] == 'session_interference_or_clock_reset'


@pytest.mark.parametrize('failure', [RuntimeError('not ready'), KeyboardInterrupt()])
def test_start_failure_closes_transport_without_attempts(failure):
    class StartFailure(FullTransport):
        def start(self): raise failure
    live = StartFailure()
    report = run(live)
    assert live.closed and live.sent == []
    assert report['completed_waypoints'] == [] and report['failed_leg'] is None
    assert not report['full_route_pass'] and not report['narrow_leg_pass']


@pytest.mark.parametrize('key,value', [('age_s', None), ('age_s', float('nan')),
                                      ('age_s', float('inf')), ('age_s', True),
                                      ('child_frame_id', 'odom')])
def test_invalid_boundary_freshness_is_not_success(key, value):
    def change(i, m): m['end_map_base_link'][key] = value
    report = run(FullTransport(change=change))
    assert len(report['sent_poses']) == 1 and not report['full_route_pass']


def test_independent_runner_and_manager_clock_receipts_are_not_a_reset():
    # goal_manager's clock can be .1s ahead of this node's last /clock callback.
    # Both streams are monotonic; start/end boundary evidence is fresh.
    def change(i, m):
        m['start_map_base_link']['age_s'] = .1
        m['end_map_base_link']['age_s'] = .1
    report = run(FullTransport(change=change))
    assert report['full_route_pass'] is True
    assert report['completed_waypoints'] == ORDER


def test_manager_time_cannot_run_backward_within_a_goal():
    def change(i, m):
        m['start_map_base_link']['header']['stamp']['sec'] = 100
    report = run(FullTransport(change=change))
    assert len(report['sent_poses']) == 1
    assert not report['full_route_pass']
    assert report['segments'][0]['failure_reason'] == 'session_interference_or_clock_reset'


def test_fresh_process_dry_run_rejects_even_eager_ros_imports(tmp_path):
    import subprocess
    script = '''
import builtins, runpy, sys
from pathlib import Path
original = builtins.__import__
def guarded(name, *args, **kwargs):
    if name.split('.')[0] in {'rclpy', 'geometry_msgs', 'nav2_msgs', 'tf2_ros'}:
        raise AssertionError('dry-run attempted ROS import: ' + name)
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
sys.path.insert(0, 'scripts')
sys.argv = ['scripts/run_warehouse_route.py', '--mode', 'full', '--dry-run', '--output-root', sys.argv[1]]
runpy.run_path(str(Path('scripts/run_warehouse_route.py').resolve()), run_name='__main__')
'''
    result = subprocess.run([sys.executable, '-c', script, str(tmp_path)],
                            cwd=route.ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    report = json.loads(next(tmp_path.glob('*/report.json')).read_text())
    assert report['cleanup']['ros_initialized'] is False
    assert report['sent_poses'] == []
