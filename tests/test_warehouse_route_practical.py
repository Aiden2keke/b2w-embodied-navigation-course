"""Practical advancement contracts, entirely offline (no ROS initialization)."""
import copy
import json
from types import SimpleNamespace

import pytest

from test_warehouse_route import XY
from test_warehouse_route_full import FullTransport, ORDER
import run_warehouse_route as route


def run(live, checkpoint=lambda r: None, advancement='practical'):
    return route.run_full(route.load_route(), execute=True, advancement=advancement,
                          transport_factory=lambda: live, checkpoint=checkpoint)


@pytest.mark.parametrize('index', [0, 1])
def test_success_with_015m_error_sends_next_frozen_waypoint(index):
    live = FullTransport(offsets=(.15,) * 5, statuses=(4,) * (index + 1) + (6,) * (4 - index))
    report = run(live)
    assert report['completed_waypoints'] == ORDER[:index + 1]
    assert [tuple(s['pose']['position'][k] for k in 'xy') for s in live.sent] == XY[:index + 2]
    segment = report['segments'][index]
    assert segment['endpoint_error_m'] == pytest.approx(.15)
    assert segment['success'] and not segment['evaluation_passed']
    assert not report['strict_full_route_pass'] and not report['full_route_pass']


def test_all_five_successes_complete_practical_route_only_after_cleanup():
    live = FullTransport(offsets=(.15, .3, .4, .35, .15))
    states = []
    report = run(live, lambda r: states.append((copy.deepcopy(r), live.closed)))
    assert report['completed_waypoints'] == ORDER
    assert report['practical_route_completed'] is report['success'] is True
    assert report['stop_reason'] == 'practical_route_completed'
    assert report['strict_full_route_pass'] is report['full_route_pass'] is False
    assert not report['narrow_leg_pass']
    assert live.closed and report['cleanup']['passed']
    for state, closed in states:
        if state['practical_route_completed']:
            assert state['completed_waypoints'] == ORDER and closed


@pytest.mark.parametrize('defect', ['missing_start', 'missing_end', 'stale_end'])
def test_historical_tf_warning_never_fabricates_endpoint_evidence(defect):
    def change(i, m):
        if defect == 'missing_start': m['start_map_base_link'] = None
        if defect == 'missing_end': m['end_map_base_link'] = None
        if defect == 'stale_end': m['end_map_base_link']['age_s'] = .8
    report = run(FullTransport(offsets=(.15,) * 5, change=change))
    assert report['practical_route_completed']
    for segment in report['segments']:
        assert 'boundary_localization_unavailable_or_stale' in segment['warnings']
        assert segment['endpoint_error_m'] == (None if defect == 'missing_end' else pytest.approx(.15))
        if defect == 'missing_end':
            assert segment['end_map_base_link'] is None
            assert not segment['evidence_complete']


@pytest.mark.parametrize('status,reason,manager_ok', [
    (6, 'aborted', False), (5, 'canceled', False), (None, 'rejected', False),
    (None, 'timeout', False), (4, 'succeeded', False)])
def test_real_failure_or_manager_failure_stops_without_retry(status, reason, manager_ok):
    def change(i, m):
        if i == 1: m.update(nav2_status=status, end_reason=reason, success=manager_ok)
    live = FullTransport(change=change)
    report = run(live)
    assert report['completed_waypoints'] == ['G1'] and len(live.sent) == 2
    assert report['stop_reason'] == 'navigation_failed'
    assert not report['success'] and live.closed


def test_strict_default_still_stops_at_005m_gate():
    live = FullTransport(offsets=(.15,) * 5)
    report = route.run_full(route.load_route(), execute=True, transport_factory=lambda: live)
    assert len(live.sent) == 1 and not report['success']
    assert report['stop_reason'] == 'endpoint_evaluation_failed'
    assert report['advancement'] == 'strict'
    assert not report['strict_full_route_pass'] and not report['practical_route_completed']


@pytest.mark.parametrize('advancement', ['strict', 'practical'])
def test_even_perfect_practical_arrivals_never_claim_strict_benchmark_pass(advancement):
    report = run(FullTransport(), advancement=advancement)
    assert report['success']
    assert report['strict_full_route_pass'] is report['full_route_pass'] is (advancement == 'strict')
    assert report['practical_route_completed'] is (advancement == 'practical')


@pytest.mark.parametrize('defect', ['concurrent', 'clock', 'cleanup'])
def test_practical_safety_failures_never_complete(defect):
    class Unsafe(FullTransport):
        def close(self):
            result = super().close()
            if defect == 'cleanup': result['passed'] = False
            return result
    live = Unsafe()
    def change(i, m):
        if i == 0 and defect == 'concurrent': live.foreign_goal = True
        if i == 0 and defect == 'clock':
            m['start_map_base_link']['header']['stamp']['sec'] = 100
    live.change = change
    report = run(live)
    assert not report['success'] and not report['practical_route_completed']
    assert report['stop_reason'] == ('cleanup_failed' if defect == 'cleanup'
                                     else 'session_interference_or_clock_reset')
    assert len(live.sent) == (5 if defect == 'cleanup' else 1)


@pytest.mark.parametrize('advancement', ['strict', 'practical'])
def test_cli_full_mode_records_policy_in_offline_dry_run(tmp_path, advancement):
    assert route.main(['--mode', 'full', '--advancement', advancement, '--dry-run',
                       '--output-root', str(tmp_path)],
                      transport_factory=lambda: pytest.fail('Dry run constructed transport')) == 0
    report = json.loads(next(tmp_path.glob('*/report.json')).read_text())
    assert report['advancement'] == advancement and report['sent_poses'] == []
    assert not report['practical_route_completed'] and not report['strict_full_route_pass']


def test_practical_flag_cannot_change_critical_mode(tmp_path):
    with pytest.raises(SystemExit):
        route.main(['--advancement', 'practical', '--output-root', str(tmp_path)])
    assert list(tmp_path.iterdir()) == []


class ReadinessTransport(route.RosGoalTransport):
    """Keep real readiness and send logic; replace ROS IO without creating a node."""
    def __init__(self, directory, recovers, historical='missing'):
        super().__init__(ready_timeout=.25)
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from builtin_interfaces.msg import Time
        from rosidl_runtime_py.convert import message_to_ordereddict
        self.PoseStamped, self.convert = PoseStamped, message_to_ordereddict
        self.directory, self.timeout = directory, 120.
        self.recovers, self.sent, self.waits = recovers, [], 0
        self.historical = historical
        self.current_fresh = True
        self.node = SimpleNamespace(
            get_node_names=lambda: ['goal_manager'],
            get_publishers_info_by_topic=lambda topic: [object()],
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                nanoseconds=(11 + len(self.sent)) * 10**9,
                to_msg=lambda: Time(sec=11 + len(self.sent)))))
        def lookup(*args):
            tf = TransformStamped()
            tf.header.stamp.sec = 11 + len(self.sent) - (0 if self.current_fresh else 2)
            return tf
        self.tf = SimpleNamespace(lookup_transform=lookup)
        self.publisher = SimpleNamespace(publish=self.publish, get_subscription_count=lambda: 2)
        self.cancel = SimpleNamespace(service_is_ready=lambda: True)
        self.readiness = SimpleNamespace(active=lambda now: True)

    def start(self):
        return {'results_directory': str(self.directory)}

    def spin(self, deadline):
        # During readiness after G1, only a recovered current TF permits G2.
        if len(self.sent) == 1 and not self.current_fresh and not self.pending:
            self.waits += 1
            assert 0 < deadline - route.time.monotonic() <= self.ready_timeout
            if not self.recovers:
                raise RuntimeError('ROS shutdown or bounded wait expired')
            self.current_fresh = True

    def publish(self, msg):
        sent = self.convert(msg)
        self.sent.append(sent)
        if len(self.sent) == 2:
            assert self.current_fresh and self.waits > 0
        # Real manager-shaped metrics, including a missing/stale historical endpoint.
        fixture = FullTransport(offsets=(.15,) * 5)
        result = fixture.send_and_wait(sent, lambda *a: None)
        metrics = result['metrics']
        metrics['goal_pose'] = sent
        metrics['start_map_base_link']['header']['stamp']['sec'] = sent['header']['stamp']['sec']
        if self.historical == 'missing':
            metrics['end_map_base_link'] = None
        else:
            metrics['end_map_base_link']['age_s'] = .8
        (self.directory / f'20261018T000000.{len(self.sent):06d}Z.json').write_text(json.dumps(metrics))
        self.current_fresh = len(self.sent) != 1

    def close(self):
        return {'passed': True}


@pytest.mark.parametrize('recovers', [True, False])
@pytest.mark.parametrize('historical', ['missing', 'stale'])
def test_historical_tf_advances_only_after_real_current_readiness(tmp_path, recovers, historical):
    live = ReadinessTransport(tmp_path, recovers, historical)
    report = run(live)
    assert live.waits == 1
    assert len(live.sent) == (5 if recovers else 1)
    assert report['practical_route_completed'] is recovers
    assert report['completed_waypoints'] == (ORDER if recovers else ['G1'])
    assert report['segments'][0]['endpoint_error_m'] == (None if historical == 'missing'
                                                        else pytest.approx(.15))
    assert 'boundary_localization_unavailable_or_stale' in report['segments'][0]['warnings']
    if not recovers:
        assert report['stop_reason'] == 'transport_error'
        assert report['failed_leg'] == 'G1-G2'
        assert report['segments'][-1].get('sent_pose') is None


@pytest.mark.parametrize('defect', ['clock_epoch', 'backwards_stamp', 'concurrent'])
def test_reset_or_interference_during_readiness_stops_before_next_publish(tmp_path, defect):
    from builtin_interfaces.msg import Time
    class ResetDuringReadiness(ReadinessTransport):
        def spin(self, deadline):
            waiting = len(self.sent) == 1 and not self.pending
            super().spin(deadline)
            if waiting:
                if defect == 'clock_epoch':
                    from warehouse_route_evidence import RouteEvidence
                    self.evidence = RouteEvidence(tmp_path, route.load_route())
                    self.evidence.epoch_invalid = True
                elif defect == 'concurrent':
                    self.foreign_goal = True
                else:
                    self.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                        nanoseconds=10_000_000_000,
                        to_msg=lambda: Time(sec=10)))
                    self.tf.lookup_transform = lambda *a: SimpleNamespace(header=SimpleNamespace(
                        stamp=SimpleNamespace(sec=10, nanosec=0)))
    live = ResetDuringReadiness(tmp_path, True)
    report = run(live)
    assert len(live.sent) == 1
    assert report['completed_waypoints'] == ['G1']
    assert not report['success'] and not report['practical_route_completed']
    assert report['failed_leg'] == 'G1-G2'
    assert report['segments'][-1].get('sent_pose') is None
