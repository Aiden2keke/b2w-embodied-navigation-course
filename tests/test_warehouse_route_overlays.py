"""Mock-map/trajectory evidence only; no ROS initialization or motion."""
import copy
import json
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import run_warehouse_route as benchmark
import warehouse_route_evidence as evidence
import plot_warehouse_route as plotting


def pose(x, y, sec, nanosec=0, frame='map'):
    return {'header': {'frame_id': frame, 'stamp': {'sec': sec, 'nanosec': nanosec}},
            'pose': {'position': {'x': x, 'y': y, 'z': 0.},
                     'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}}


def grid(yaw=0.):
    """Deliberately tiny MOCK obstacle map; never warehouse evidence."""
    return {'header': {'frame_id': 'map', 'stamp': {'sec': 12, 'nanosec': 0}},
            'info': {'resolution': .5, 'width': 4, 'height': 3,
                     'origin': {'position': {'x': -1., 'y': 2., 'z': 0.},
                                'orientation': {'x': 0., 'y': 0.,
                                                'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)}}},
            'data': [-1, 0, 100, 0, 0, 0, 100, 0, 0, 0, 100, 0]}


def mock_run(directory, missing_map=False, full=False):
    directory.mkdir()
    config = benchmark.load_route()
    capture = evidence.RouteEvidence(directory, config)
    segments = []
    goals = benchmark.critical_goals(config)
    if full:
        # Supplied offline artifacts only; no execution mode or live G3–G5 goals.
        goals += [{'name': g['name'], 'pose': pose(*g['position'], 1)} for g in config['goals'][2:]]
    for i, goal in enumerate(goals):
        target = copy.deepcopy(goal['pose'])
        target['header']['stamp']['sec'] = 10 + i * 2
        capture.begin(target)
        x, y = config['goals'][i]['position']
        capture.sample(pose(x + .2, y + .1, 10 + i * 2))
        capture.sample(pose(x, y, 11 + i * 2))
        capture.finish(None if missing_map else grid())
        segments.append({'name': goal['name'], 'sent_pose': target,
                         'segment': 'Start→G1' if i == 0 else f'G{i}→G{i+1}',
                         'endpoint_error_m': .01,
                         'goal_manager_metrics': {'path_length_m': 2.}})
    (directory / 'route-snapshot.json').write_text(json.dumps(config))
    (directory / 'report.json').write_text(json.dumps({
        'manifest': {'head': '3' * 40, 'route_sha256': benchmark.digest(benchmark.ROUTE)},
        'segments': segments, 'dry_run': False}))
    return capture


def test_map_geometry_preserves_ros_row_order_and_rotated_origin():
    g = evidence.map_geometry(grid(math.pi / 2))
    assert g['data'].tolist() == [[-1, 0, 100, 0], [0, 0, 100, 0], [0, 0, 100, 0]]
    assert g['corners'][0] == pytest.approx([-1., 2.])
    assert g['corners'][1] == pytest.approx([-1., 4.])
    assert g['corners'][2] == pytest.approx([-2.5, 4.])


@pytest.mark.parametrize('change', ['frame', 'size', 'values', 'quaternion'])
def test_invalid_map_rejected_without_a_guessed_transform(change):
    m = grid()
    if change == 'frame': m['header']['frame_id'] = 'world'
    if change == 'size': m['data'].pop()
    if change == 'values': m['data'][0] = 255
    if change == 'quaternion': m['info']['origin']['orientation']['x'] = .5
    with pytest.raises(ValueError):
        evidence.map_geometry(m)


def test_capture_standard_path_throttling_and_exact_map_snapshot(tmp_path):
    capture = evidence.RouteEvidence(tmp_path, benchmark.load_route())
    target = benchmark.critical_goals(benchmark.load_route())[0]['pose']
    capture.begin(target)
    capture.sample(pose(0., 0., 10))
    capture.sample(pose(.01, 0., 10, 50_000_000))  # Below evidence sampling interval.
    capture.sample(pose(.1, 0., 10, 150_000_000))
    capture.finish(grid())
    index = json.loads((tmp_path / 'evidence/index.json').read_text())
    segment = index['segments'][0]
    path = json.loads((tmp_path / segment['path_file']).read_text())
    saved_map = json.loads((tmp_path / segment['map_source']['file']).read_text())
    assert path['header']['frame_id'] == 'map'
    assert [p['pose']['position']['x'] for p in path['poses']] == [0., .1]
    assert path['poses'][1]['header']['stamp']['nanosec'] == 150_000_000
    assert saved_map == grid()
    assert segment['map_source']['topic'] == '/map'
    assert segment['map_source']['sha256'] == benchmark.digest(tmp_path / segment['map_source']['file'])


def test_path_gaps_are_not_drawn_as_continuous_traversal():
    p = {'header': {'frame_id': 'map'}, 'poses': [pose(0, 0, 1), pose(.1, 0, 1, 200_000_000),
                                               pose(5, 5, 3)]}
    chunks = evidence.path_chunks(p)
    assert [len(c) for c in chunks] == [2, 1]
    p['poses'][2]['header']['frame_id'] = 'odom'
    with pytest.raises(ValueError, match='frame'):
        evidence.path_chunks(p)


def test_generate_full_and_each_segment_png_and_metadata(tmp_path):
    run = tmp_path / 'mock-run'
    mock_run(run)
    report = plotting.render_run(run, label='MOCK INPUT — not physical evidence')
    assert report['detour_judgment'] is None
    assert report['executed_waypoints'] == ['G1', 'G2']
    assert report['unexecuted_waypoints'] == ['G3', 'G4', 'G5']
    assert [r['segment_id'] for r in report['images']] == ['full-route', 'Start-G1', 'G1-G2']
    for row in report['images']:
        p = Path(row['image_path'])
        assert p.read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
        assert p.stat().st_size > 1000
        assert row['route_id'] == benchmark.load_route()['route_id']
        assert row['head'] == '3' * 40
        assert row['waypoints'] == {g['name']: g['position'] for g in benchmark.load_route()['goals']}
        assert row['map_source']['topic'] == '/map'
    assert report['images'][2]['path_length_m'] == 2.
    assert report['images'][2]['endpoint_error_m'] == .01
    assert (run / 'overlays/trajectory-overlays.json').is_file()


def test_missing_map_never_invents_floorplan(tmp_path):
    run = tmp_path / 'missing-map'
    mock_run(run, missing_map=True)
    report = plotting.render_run(run)
    # Diagnostic geometry is never invented; the new composite retains an
    # explicit unavailable panel so the user can still inspect the run.
    assert not (run / 'overlays/full-route.png').exists()
    assert Path(report['composite_image_path']).is_file()
    assert all(i['image_path'] is None and i['status'] == 'unavailable' for i in report['images'])


def test_offline_full_route_artifacts_render_all_five_without_an_execution_mode(tmp_path):
    run = tmp_path / 'mock-full-route'
    mock_run(run, full=True)
    report = plotting.render_run(run, label='MOCK full-route artifacts only')
    assert report['executed_waypoints'] == ['G1', 'G2', 'G3', 'G4', 'G5']
    assert report['unexecuted_waypoints'] == []
    assert [row['segment_id'] for row in report['images']] == [
        'full-route', 'Start-G1', 'G1-G2', 'G2-G3', 'G3-G4', 'G4-G5']
    assert all(Path(row['image_path']).is_file() for row in report['images'])


def test_renderer_rejects_other_run_and_corrupted_map(tmp_path):
    run = tmp_path / 'this-run'
    mock_run(run)
    index_file = run / 'evidence/index.json'
    index = json.loads(index_file.read_text())
    original_id = index['run_id']
    index['run_id'] = 'another-startup'
    index_file.write_text(json.dumps(index))
    with pytest.raises(ValueError, match='run'):
        plotting.render_run(run)
    index['run_id'] = original_id
    index_file.write_text(json.dumps(index))
    source = run / index['segments'][0]['map_source']['file']
    source.write_text('{}')
    result = plotting.render_run(run)
    assert all(i['status'] == 'unavailable' for i in result['images'])
    assert any('SHA' in str(error) for error in result['capture_errors'])


def test_offline_cli_never_imports_ros(tmp_path, monkeypatch):
    run = tmp_path / 'offline'
    mock_run(run)
    import builtins
    original = builtins.__import__
    def guarded(name, *args, **kwargs):
        if name.split('.')[0] in {'rclpy', 'nav2_msgs', 'geometry_msgs', 'tf2_ros'}:
            pytest.fail('Offline renderer imported ROS')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', guarded)
    assert plotting.main([str(run), '--label', 'MOCK']) == 0


@pytest.mark.parametrize('status', [4, None])
@pytest.mark.parametrize('capture_error', [False, True])
def test_transport_reuses_tf_and_map_without_starting_a_node(tmp_path, status, capture_error):
    from types import SimpleNamespace
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from builtin_interfaces.msg import Time
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.set_message import set_message_fields
    live = benchmark.RosGoalTransport(evidence_directory=tmp_path / 'run')
    if capture_error:
        def broken_sample(pose):
            raise OSError('Evidence sample unavailable')
        live.evidence.sample = broken_sample
    live.require_ready = lambda deadline: None
    live.PoseStamped, live.convert = PoseStamped, message_to_ordereddict
    live.directory, live.timeout = tmp_path, 120.
    tr = TransformStamped()
    tr.header.frame_id, tr.child_frame_id = 'map', 'base_link'
    tr.header.stamp.sec = 10
    tr.transform.rotation.w = 1.
    live.tf = SimpleNamespace(lookup_transform=lambda *a: tr)
    subscriptions = {}
    live.node = SimpleNamespace(create_subscription=lambda cls, topic, callback, qos:
                                subscriptions.update({topic: (callback, qos)}),
        destroy_node=lambda: None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=10_200_000_000, to_msg=lambda: Time(sec=10))))
    live.attach_map_evidence()
    from rclpy.qos import DurabilityPolicy
    callback, qos = subscriptions['/map']
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    msg = OccupancyGrid()
    set_message_fields(msg, grid())
    callback(msg)
    def publish(msg):
        (tmp_path / '20260101T000000.000001Z.json').write_text(json.dumps(
            {'goal_pose': message_to_ordereddict(msg), 'nav2_status': status,
             'end_reason': 'succeeded' if status == 4 else 'timeout'}))
    live.publisher = SimpleNamespace(publish=publish)
    def spin(deadline):
        tr.header.stamp.nanosec = 200_000_000
        tr.transform.translation.x = .1
        live.capture_pose()
    live.spin = spin
    result = live.send_and_wait(benchmark.critical_goals(benchmark.load_route())[0]['pose'], lambda *a: None)
    assert result['metrics']['nav2_status'] == status
    assert live.pending is (status is None)
    if status is None:
        # Timeout evidence serialization must not run before cancellation.
        assert not (tmp_path / 'run/evidence/index.json').exists()
        from concurrent.futures import Future
        future = Future()
        future.set_result(SimpleNamespace(success=True))
        events = []
        live.Trigger = SimpleNamespace(Request=lambda: None)
        def cancel(request):
            events.append('cancel')
            return future
        live.cancel = SimpleNamespace(service_is_ready=lambda: True, call_async=cancel)
        live.rclpy = SimpleNamespace(ok=lambda: True, spin_until_future_complete=lambda *a, **kw: None,
            spin_once=lambda *a, **kw: live.owned_actions.update({'owned': 5}))
        finish = live.finish_evidence
        def persist():
            events.append('persist')
            finish()
        live.finish_evidence = persist
        assert live.close()['passed']
        assert events == ['cancel', 'persist']
    index = json.loads((tmp_path / 'run/evidence/index.json').read_text())
    path = json.loads((tmp_path / 'run' / index['segments'][0]['path_file']).read_text())
    assert [p['pose']['position']['x'] for p in path['poses']] == ([] if capture_error else [0., .1])
    if capture_error:
        assert index['errors']


def test_render_failure_does_not_change_navigation_outcome(tmp_path, monkeypatch):
    from test_warehouse_route import Transport
    def broken(*args, **kwargs):
        raise OSError('Plot storage unavailable')
    monkeypatch.setattr(plotting, 'render_run', broken)
    assert benchmark.main(['--execute', '--output-root', str(tmp_path)], transport_factory=Transport) == 0
    report = json.loads(next(tmp_path.glob('*/report.json')).read_text())
    assert report['success'] is True
    assert report['visualization']['status'] == 'error'
    assert report['stop_reason'] == 'critical_succeeded'


def test_clock_reset_between_segments_invalidates_full_route_composition(tmp_path):
    run = tmp_path / 'clock-reset'
    capture = mock_run(run)
    capture.observe_clock(20_000_000_000)
    capture.observe_clock(1_000_000_000)
    capture.save()
    report = plotting.render_run(run)
    assert all(i['image_path'] is None for i in report['images'])
    assert any('clock' in e.lower() for e in report['capture_errors'])


def test_storage_failure_marks_segment_unavailable_without_poisoning_next(tmp_path, monkeypatch):
    capture = evidence.RouteEvidence(tmp_path, benchmark.load_route())
    goals = benchmark.critical_goals(benchmark.load_route())
    capture.begin(goals[0]['pose'])
    capture.sample(pose(0, 0, 10))
    write = evidence.write
    def fail_once(path, value):
        monkeypatch.setattr(evidence, 'write', write)
        raise OSError('One failed write')
    monkeypatch.setattr(evidence, 'write', fail_once)
    capture.finish(grid())
    assert capture.current is None
    capture.begin(goals[1]['pose'])
    capture.sample(pose(1, 0, 11))
    capture.finish(grid())
    index = json.loads((tmp_path / 'evidence/index.json').read_text())
    assert index['segments'][0]['invalid_reason']
    assert index['segments'][1]['segment_id'] == 'G1-G2'
    assert index['segments'][1]['path_file']


@pytest.mark.parametrize('changed', ['snapshot', 'sent_goal'])
def test_nominal_waypoints_bound_to_captured_route_content(tmp_path, changed):
    run = tmp_path / changed
    mock_run(run)
    if changed == 'snapshot':
        path = run / 'route-snapshot.json'
        value = json.loads(path.read_text())
        value['goals'][0]['position'][0] += 1.
    else:
        path = run / 'report.json'
        value = json.loads(path.read_text())
        value['segments'][0]['sent_pose']['pose']['position']['x'] += 1.
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match='route|waypoint'):
        plotting.render_run(run)


def test_incomplete_full_route_capture_is_visible_in_image_title(tmp_path, monkeypatch):
    run = tmp_path / 'partial'
    mock_run(run)
    index_file = run / 'evidence/index.json'
    index = json.loads(index_file.read_text())
    index['segments'][0]['invalid_reason'] = 'Missing localization'
    index_file.write_text(json.dumps(index))
    titles = []
    draw = plotting.draw_overlay
    def observe(*args):
        titles.append(args[-2])
        return draw(*args)
    monkeypatch.setattr(plotting, 'draw_overlay', observe)
    plotting.render_run(run)
    assert 'INCOMPLETE' in titles[0] and 'Start-G1' in titles[0]


def test_default_benchmark_never_constructs_capture_or_imports_plotting(tmp_path, monkeypatch):
    import builtins
    original = builtins.__import__
    def guarded(name, *args, **kwargs):
        if name.split('.')[0] in {'rclpy', 'nav_msgs', 'matplotlib', 'plot_warehouse_route'}:
            pytest.fail('Default dry-run imported live capture/plotting')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', guarded)
    def forbidden():
        pytest.fail('Default dry-run constructed transport')
    assert benchmark.main(['--output-root', str(tmp_path)], transport_factory=forbidden) == 0
    run = next(tmp_path.iterdir())
    report = json.loads((run / 'report.json').read_text())
    assert report['sent_poses'] == []
    assert report['visualization'] == {'status': 'dry_run', 'images': []}
    assert not (run / 'evidence').exists() and not (run / 'overlays').exists()


def test_long_method_labels_keep_complete_title_inside_image(tmp_path, monkeypatch):
    from matplotlib.figure import Figure
    run = tmp_path / 'long-label'
    mock_run(run)
    save = Figure.savefig
    def checked_save(fig, *args, **kwargs):
        fig.canvas.draw()
        bounds = fig._suptitle.get_window_extent(fig.canvas.get_renderer())
        assert bounds.x0 >= 0 and bounds.x1 <= fig.bbox.width
        return save(fig, *args, **kwargs)
    monkeypatch.setattr(Figure, 'savefig', checked_save)
    plotting.render_run(run, label='MOCK MAP + MOCK TRAJECTORY — NOT WAREHOUSE EVIDENCE')
