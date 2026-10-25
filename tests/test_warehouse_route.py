"""Offline contract tests. ROS messages are used without rclpy.init or any node."""
import copy
from concurrent.futures import Future
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import run_warehouse_route as route


XY = [(-0.18571805953979492, 7.189985752105713),
      (-4.47874116897583, 6.116940498352051),
      (-3.928757667541504, 3.436979293823242),
      (0.5741653442382812, 6.976665496826172),
      (5.592322826385498, 2.1632280349731445)]


def snapshot(x=0., y=0.):
    return {'header': {'frame_id': 'map', 'stamp': {'sec': 10, 'nanosec': 0}},
            'child_frame_id': 'base_link', 'age_s': 0.,
            'transform': {'translation': {'x': x, 'y': y, 'z': .6},
                          'rotation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}}


class Transport:
    """Replaces only ROS/network IO; exercises the real sequencing/evaluation."""
    def __init__(self, statuses=(4, 4), offsets=(.01, .01), fail=None):
        self.statuses, self.offsets, self.fail = statuses, offsets, fail
        self.sent = []
        self.closed = False

    def start(self):
        return {'results_directory': '/mock/metrics', 'goal_tolerance_m': .3}

    def send_and_wait(self, pose, on_sent):
        sent = copy.deepcopy(pose)
        sent['header']['stamp'] = {'sec': 11 + len(self.sent), 'nanosec': 123}
        self.sent.append(sent)
        on_sent(sent)
        if self.fail:
            raise self.fail
        i = len(self.sent) - 1
        p = sent['pose']['position']
        status = self.statuses[i]
        metrics = {'goal_id': f'mock-{i}', 'goal_pose': sent,
                   'nav2_status': status, 'success': status == 4,
                   'end_reason': {4: 'succeeded', 5: 'canceled', 6: 'aborted'}.get(status, 'timeout'),
                   'start_map_base_link': snapshot(),
                   'end_map_base_link': snapshot(p['x'], p['y'] + self.offsets[i]),
                   'final_goal_distance_m': self.offsets[i],
                   'goal_tolerance_m': .3, 'collision_count': 0,
                   'path_length_m': 2., 'time_to_goal_s': 10.,
                   'planning_latency_ms': 20.}
        return {'metrics': metrics, 'metrics_file': f'/mock/{i}.json',
                'metrics_sha256': 'mock-sha'}

    def close(self):
        self.closed = True
        return {'passed': True, 'child_processes': [], 'session_processes_touched': False}


def test_frozen_positions_and_shared_corridor_orientation():
    config = route.load_route()
    assert [tuple(g['position']) for g in config['goals']] == XY
    assert [g['orientation'] for g in config['goals'][2:]] == [None] * 3
    goals = route.critical_goals(config)
    assert [g['name'] for g in goals] == ['G1', 'G2']
    for i, goal in enumerate(goals):
        p = goal['pose']['pose']
        assert (p['position']['x'], p['position']['y'], p['position']['z']) == (*XY[i], 0.)
        assert p['orientation'] == pytest.approx(
            {'x': 0., 'y': 0., 'z': -0.9925103760697063, 'w': 0.12216035933955931})
        assert goal['yaw_rad'] == pytest.approx(-2.8966601449198057)
    assert config['evaluation_tolerance_m'] == .05


def test_default_dry_run_never_constructs_transport_or_sends():
    def forbidden():
        pytest.fail('Dry-run constructed live transport')
    report = route.run_critical(route.load_route(), transport_factory=forbidden)
    assert report['dry_run'] is True
    assert report['sent_poses'] == report['segments'] == []
    assert report['stop_reason'] == 'dry_run'


def test_full_dry_run_preserves_unresolved_canonical_orientations():
    report = route.run_critical(route.load_route(), mode='full')
    assert [g['name'] for g in report['planned_goals']] == ['G1', 'G2', 'G3', 'G4', 'G5']
    assert report['unresolved_orientations'] == ['G3', 'G4', 'G5']
    assert [g['pose']['pose']['orientation'] for g in report['planned_goals'][2:]] == [None] * 3
    assert report['sent_poses'] == []


def test_full_cli_dry_run_is_offline_and_invalid_execution_creates_no_output(tmp_path, monkeypatch):
    import builtins
    original = builtins.__import__
    def guarded(name, *args, **kwargs):
        if name.split('.')[0] in {'rclpy', 'geometry_msgs', 'nav2_msgs', 'tf2_ros'}:
            pytest.fail(f'Full dry-run imported {name}')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', guarded)
    assert route.main(['--mode', 'full', '--output-root', str(tmp_path)]) == 0
    report = json.loads(next(tmp_path.glob('*/report.json')).read_text())
    assert report['mode'] == 'full' and report['unresolved_orientations'] == ['G3', 'G4', 'G5']
    before = set(tmp_path.iterdir())
    with pytest.raises(SystemExit):
        route.main(['--mode', 'full', '--execute', '--output-root', str(tmp_path)])
    assert set(tmp_path.iterdir()) == before


@pytest.mark.parametrize('yaws', [None, (0.,), (0., 0., float('nan'))])
def test_full_execution_rejects_unresolved_or_invalid_yaws_before_transport(yaws):
    def forbidden():
        pytest.fail('Invalid full route constructed transport')
    with pytest.raises(ValueError, match='yaw|orientation'):
        route.run_critical(route.load_route(), mode='full', continuation_yaws=yaws,
                           execute=True, transport_factory=forbidden)


def test_full_sequence_uses_explicit_yaws_and_all_evaluation_gates():
    live = Transport(statuses=(4,) * 5, offsets=(.01,) * 5)
    report = route.run_critical(route.load_route(), mode='full', continuation_yaws=(.1, -.2, .3),
                               execute=True, transport_factory=lambda: live)
    assert report['success'] is True and report['stop_reason'] == 'full_succeeded'
    assert [s['segment'] for s in report['segments']] == ['Start→G1', 'G1→G2', 'G2→G3', 'G3→G4', 'G4→G5']
    assert [tuple(s['pose']['position'][a] for a in 'xy') for s in live.sent] == XY
    assert [g['yaw_rad'] for g in report['planned_goals'][2:]] == [.1, -.2, .3]
    for sent, yaw in zip(live.sent[2:], (.1, -.2, .3)):
        assert sent['pose']['orientation'] == pytest.approx(
            dict(x=0., y=0., z=math.sin(yaw / 2), w=math.cos(yaw / 2)))
    assert live.closed


@pytest.mark.parametrize('statuses,offsets,count,reason', [
    ((4,) * 5, (.06484485334757964,) * 5, 1, 'staging_evaluation_failed'),
    ((4,) * 5, (.01, .06, .01, .01, .01), 2, 'critical_evaluation_failed'),
    ((4, 4, 6, 4, 4), (.01,) * 5, 3, 'continuation_failed'),
    ((4,) * 5, (.01, .01, .01, .06, .01), 4, 'continuation_evaluation_failed'),
])
def test_full_sequence_stops_at_first_failed_gate(statuses, offsets, count, reason):
    live = Transport(statuses=statuses, offsets=offsets)
    report = route.run_critical(route.load_route(), mode='full', continuation_yaws=(0., 0., 0.),
                               execute=True, transport_factory=lambda: live)
    assert len(live.sent) == count and live.closed
    assert report['success'] is False and report['stop_reason'] == reason


def test_cli_dry_run_never_imports_ros_and_records_hashes(tmp_path, monkeypatch):
    import builtins
    original = builtins.__import__
    def guarded(name, *args, **kwargs):
        if name.split('.')[0] in {'rclpy', 'geometry_msgs', 'nav2_msgs', 'tf2_ros'}:
            pytest.fail(f'Dry-run imported {name}')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', guarded)
    assert route.main(['--output-root', str(tmp_path)]) == 0
    report = json.loads(next(tmp_path.glob('*/report.json')).read_text())
    assert report['sent_poses'] == []
    assert report['manifest']['route_sha256'] == hashlib.sha256(route.ROUTE.read_bytes()).hexdigest()
    assert len(report['manifest']['head']) == 40
    assert 'course_navigation/config/nav2.yaml' in report['manifest']['config_sha256']


def test_execution_requires_literal_flag(tmp_path):
    live = Transport()
    args = ['--output-root', str(tmp_path)]
    assert route.main(args, transport_factory=lambda: live) == 0
    assert not live.sent and not live.closed
    assert route.main(args + ['--execute'], transport_factory=lambda: live) == 0
    assert len(live.sent) == 2 and live.closed
    with pytest.raises(SystemExit):
        route.main(args + ['--execute=false'], transport_factory=lambda: live)


@pytest.mark.parametrize('statuses, count, reason', [
    ((6,), 1, 'staging_failed'), ((5,), 1, 'staging_failed'),
    ((None,), 1, 'staging_failed'), ((4, 6), 2, 'critical_failed'),
    ((4, 4), 2, 'critical_succeeded')])
def test_stop_on_first_failure_or_critical_completion(statuses, count, reason):
    live = Transport(statuses=statuses)
    report = route.run_critical(route.load_route(), execute=True, transport_factory=lambda: live)
    assert len(live.sent) == count
    assert report['stop_reason'] == reason
    assert live.closed and report['cleanup']['passed']
    assert [s['name'] for s in report['segments']] == ['G1', 'G2'][:count]
    assert all((s['pose']['position']['x'], s['pose']['position']['y']) not in XY[2:]
               for s in report['sent_poses'])
    assert report['segments'][0]['goal_manager_metrics']['path_length_m'] == 2.
    assert report['segments'][0]['sent_pose'] == live.sent[0]


def test_navigation_success_outside_staging_tolerance_stops_before_G2():
    live = Transport(offsets=(.06484485334757964, .06))
    report = route.run_critical(route.load_route(), execute=True, transport_factory=lambda: live)
    assert len(live.sent) == len(report['sent_poses']) == len(report['segments']) == 1
    assert report['segments'][0]['navigation_success'] is True
    assert report['segments'][0]['evidence_complete'] is True
    assert report['segments'][0]['evaluation_passed'] is False
    assert report['segments'][0]['endpoint_error_m'] == pytest.approx(.06484485334757964)
    assert report['success'] is False and report['stop_reason'] == 'staging_evaluation_failed'
    assert [tuple(s['pose']['position'][a] for a in 'xy') for s in live.sent] == XY[:1]
    assert report['segments'][0]['goal_manager_metrics']['success'] is True
    assert report['segments'][0]['goal_manager_metrics']['goal_tolerance_m'] == .3
    assert live.closed and report['cleanup']['passed']


@pytest.mark.parametrize('staging_offset', [.01, .05])
def test_staging_within_tolerance_proceeds_and_G2_evaluation_is_unchanged(staging_offset):
    live = Transport(offsets=(staging_offset, .06))
    report = route.run_critical(route.load_route(), execute=True, transport_factory=lambda: live)
    assert len(live.sent) == 2
    assert report['segments'][0]['navigation_success'] is True
    assert report['segments'][0]['evaluation_passed'] is True
    assert report['segments'][1]['navigation_success'] is True
    assert report['segments'][1]['evaluation_passed'] is False
    assert report['segments'][1]['endpoint_error_m'] == pytest.approx(.06)
    assert report['success'] is False and report['stop_reason'] == 'critical_evaluation_failed'
    assert [tuple(s['pose']['position'][a] for a in 'xy') for s in live.sent] == XY[:2]
    assert report['segments'][0]['goal_manager_metrics']['goal_tolerance_m'] == .3


@pytest.mark.parametrize('failure', [RuntimeError('missing metrics'), KeyboardInterrupt()])
def test_transport_error_or_interrupt_preserves_attempt_and_cleans(failure):
    live = Transport(fail=failure)
    saved = []
    report = route.run_critical(route.load_route(), execute=True,
                                transport_factory=lambda: live,
                                checkpoint=lambda r: saved.append(copy.deepcopy(r)))
    assert len(live.sent) == len(report['sent_poses']) == 1
    assert len(report['segments']) == 1 and saved[-1]['cleanup']['passed']
    assert live.closed and report['success'] is False


def test_metric_association_rejects_same_xy_with_different_stamp(tmp_path):
    sent = route.critical_goals(route.load_route())[0]['pose']
    other = copy.deepcopy(sent)
    other['header']['stamp']['sec'] += 1
    (tmp_path / '20260101T000000.000001Z.json').write_text(json.dumps({'goal_pose': other}))
    assert route.find_metrics(tmp_path, sent, set()) is None
    path = tmp_path / '20260101T000000.000002Z.json'
    path.write_text(json.dumps({'goal_pose': sent, 'nav2_status': 4}))
    assert route.find_metrics(tmp_path, sent, set())['metrics_file'] == str(path)
    assert route.find_metrics(tmp_path, sent, {path}) is None


def test_missing_end_tf_is_not_a_zero_error_or_success():
    goal = route.critical_goals(route.load_route())[0]
    metrics = {'nav2_status': 4, 'success': True, 'end_reason': 'succeeded',
               'start_map_base_link': snapshot(), 'end_map_base_link': None,
               'final_goal_distance_m': 0.}
    result = route.evaluate_goal(goal, metrics, .05)
    assert result['endpoint_error_m'] is None
    assert result['evaluation_passed'] is False
    assert result['evidence_complete'] is False


def test_start_failure_closes_partially_initialized_transport():
    class Broken(Transport):
        def start(self):
            raise RuntimeError('No ready stack')
    live = Broken()
    report = route.run_critical(route.load_route(), execute=True, transport_factory=lambda: live)
    assert live.closed and not live.sent
    assert report['success'] is False and report['stop_reason'] == 'transport_error'


def test_attached_transport_reuses_context_without_owning_shutdown():
    context = SimpleNamespace(ok=lambda: True, init=lambda: pytest.fail('Must not initialize shared context'))
    live = route.RosGoalTransport(reuse_existing_context=True)
    live.start_context(context)
    assert live.initialized is False
    live.rclpy = SimpleNamespace(ok=lambda: True, shutdown=lambda: pytest.fail('Must not shut shared context'))
    assert live.close()['passed']


def test_unused_action_server_does_not_require_a_prior_goal_to_publish_status():
    from geometry_msgs.msg import TransformStamped
    live = route.RosGoalTransport()
    live.statuses = None  # An unused action server need not publish an empty status.
    tf = TransformStamped()
    tf.header.stamp.sec = 10
    live.tf = SimpleNamespace(lookup_transform=lambda *a: tf)
    live.node = SimpleNamespace(
        get_node_names=lambda: ['goal_manager'],
        get_publishers_info_by_topic=lambda topic: [SimpleNamespace(node_name='bt_navigator')],
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)))
    live.publisher = SimpleNamespace(get_subscription_count=lambda: 2)
    live.cancel = SimpleNamespace(service_is_ready=lambda: True)
    live.readiness = SimpleNamespace(active=lambda now: True)
    live.spin = lambda deadline: pytest.fail('Idle fresh action server should be usable without a previous goal')
    live.require_ready(100.)
    live.foreign_goal = True
    with pytest.raises(RuntimeError, match='Concurrent'):
        live.require_ready(100.)


@pytest.mark.parametrize('status, reason, pending', [
    (4, 'succeeded', False), (None, 'timeout', True),
    (None, 'clock_reset', True), (None, 'rejected', False)])
def test_real_transport_records_preparation_separately_from_published_pose(tmp_path, status, reason, pending):
    from geometry_msgs.msg import PoseStamped
    from builtin_interfaces.msg import Time
    from rosidl_runtime_py.convert import message_to_ordereddict
    live = route.RosGoalTransport()
    live.require_ready = lambda deadline: None
    live.PoseStamped, live.convert = PoseStamped, message_to_ordereddict
    live.directory, live.timeout = tmp_path, 120.
    live.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=10, nanosec=456))))
    live.spin = lambda deadline: None
    events = []
    def publish(msg):
        events.append('publish')
        metrics = {'goal_pose': message_to_ordereddict(msg), 'nav2_status': status, 'end_reason': reason}
        (tmp_path / '20260101T000000.000001Z.json').write_text(json.dumps(metrics))
    live.publisher = SimpleNamespace(publish=publish)
    def sent(pose, state='published'):
        events.append(state)
        assert pose['header']['stamp'] == {'sec': 10, 'nanosec': 456}
    result = live.send_and_wait(route.critical_goals(route.load_route())[0]['pose'], sent)
    assert events == ['prepared', 'publish', 'published']
    assert result['metrics']['nav2_status'] == status and live.pending is pending


def test_cleanup_failure_cannot_report_benchmark_success():
    class Leaked(Transport):
        def close(self):
            return {'passed': False}
    report = route.run_critical(route.load_route(), execute=True, transport_factory=Leaked)
    assert report['success'] is False and report['cleanup']['passed'] is False


def test_goal_monitor_matches_manager_best_effort_and_detects_replacement():
    from geometry_msgs.msg import PoseStamped
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rclpy.qos import ReliabilityPolicy
    live = route.RosGoalTransport()
    subscriptions = {}
    live.node = SimpleNamespace(create_subscription=lambda cls, topic, cb, qos:
                                subscriptions.update({topic: (cb, qos)}))
    live.convert = message_to_ordereddict
    live.attach_monitors()
    callback, qos = subscriptions['/goal_pose']
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    msg = PoseStamped()
    live.owned_poses.append(message_to_ordereddict(msg))
    callback(msg)
    assert live.foreign_goal is False
    msg.header.stamp.sec = 2
    callback(msg)
    assert live.foreign_goal is True


@pytest.mark.parametrize('terminal, passed', [(False, False), (True, True)])
def test_cancel_acknowledgment_requires_observed_owned_action_terminal(terminal, passed):
    live = route.RosGoalTransport()
    live.initialized = live.pending = True
    live.owned_actions = {'owned-uuid': 2}
    live.cancel_confirmation_timeout = .001
    future = Future()
    future.set_result(SimpleNamespace(success=True, message='Cancel requested'))
    live.Trigger = SimpleNamespace(Request=lambda: None)
    live.cancel = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda request: future)
    destroyed = []
    live.node = SimpleNamespace(destroy_node=lambda: destroyed.append(True))
    def spin(node, timeout_sec):
        if terminal:
            live.owned_actions['owned-uuid'] = 5
    live.rclpy = SimpleNamespace(ok=lambda: True, spin_once=spin,
                                spin_until_future_complete=lambda *a, **kw: None,
                                shutdown=lambda: None)
    cleanup = live.close()
    assert cleanup['passed'] is passed and destroyed == [True]
    assert cleanup['cancel_outcome'] == ('owned_action_terminal' if passed else 'manager_cancel_unconfirmed')


def test_foreign_goal_cleanup_never_invokes_global_cancel():
    live = route.RosGoalTransport()
    live.pending = live.foreign_goal = True
    live.node = SimpleNamespace(destroy_node=lambda: None)
    live.cancel = SimpleNamespace(call_async=lambda request: pytest.fail('Canceled unowned goal'))
    cleanup = live.close()
    assert cleanup['passed'] is False
    assert cleanup['cancel_outcome'] == 'foreign_goal_not_canceled'


def test_goal_manager_exports_original_pose_status_and_fresh_tfs(tmp_path):
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from rosidl_runtime_py.convert import message_to_ordereddict
    from course_navigation.goal_manager import GoalManager
    from course_navigation.metrics import RunMetrics
    from collections import deque
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'map', 'base_link'
    tf.header.stamp.sec = 10
    tf.transform.rotation.w = 1.
    tf.transform.translation.x = 1.01
    fake = SimpleNamespace(tf=SimpleNamespace(lookup_transform=lambda *a: tf),
                           sim_time=lambda: 10., active=None, last_odom=None, tolerance=.3,
                           pending_poses=deque(), finish=lambda *a: None,
                           get_logger=lambda: SimpleNamespace(info=lambda *a: None))
    fake.map_base_snapshot = lambda: GoalManager.map_base_snapshot(fake)
    msg = PoseStamped()
    msg.header.frame_id, msg.header.stamp.sec = 'map', 10
    msg.pose.position.x, msg.pose.orientation.w = 1., 1.
    GoalManager.goal(fake, msg)
    assert fake.goal_message == message_to_ordereddict(msg)
    assert fake.start_map_base_link['child_frame_id'] == 'base_link'
    token = fake.active
    future = Future()
    future.set_result(SimpleNamespace(status=4))
    finish_calls = []
    fake.finish = lambda *a: finish_calls.append(a)
    GoalManager.result(fake, future, token)
    assert fake.nav2_status == 4 and finish_calls == [(True, 'succeeded')]
    fake.active = RunMetrics(0., (1., 0.))
    fake.directory, fake.timeout, fake.force_threshold = tmp_path, 120., 50.
    fake.nav_handle = fake.planning_handle = None
    GoalManager.finish(fake, True, 'succeeded')
    record = json.loads(next(tmp_path.glob('*.json')).read_text())
    assert record['goal_pose'] == message_to_ordereddict(msg)
    assert record['nav2_status'] == 4
    assert record['start_map_base_link']['transform']['translation']['x'] == 1.01
    assert record['end_map_base_link']['transform']['translation']['x'] == 1.01
    assert record['final_goal_distance_m'] == pytest.approx(.01)
    fake.sim_time = lambda: 11.
    assert fake.map_base_snapshot() is None  # Stale TF is missing, not current.
