"""Offline exact-source floorplan/registration/composite tests; no ROS nodes."""
import copy
import json
from pathlib import Path
import sys

import numpy as np
import pytest
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import warehouse_truth_floorplan as floor
import warehouse_truth_overlay as primary


def odom(x, y, sec, ns=0, frame='sim_world'):
    return {'header': {'frame_id': frame, 'stamp': {'sec': sec, 'nanosec': ns}},
            'child_frame_id': 'base_link', 'pose': {'pose': {
                'position': {'x': x, 'y': y, 'z': .6},
                'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}}}


def truth_run(directory):
    import run_warehouse_route as bench
    from warehouse_route_evidence import RouteEvidence
    from test_warehouse_route_overlays import grid, pose
    directory.mkdir()
    config = bench.load_route()
    capture = RouteEvidence(directory, config)
    segments = []
    start = [0., 0.]
    for i, goal in enumerate(bench.critical_goals(config)):
        sent = copy.deepcopy(goal['pose'])
        sent['header']['stamp']['sec'] = 10 + 5*i
        capture.begin(sent)
        end = config['goals'][i]['position']
        for j in range(40):
            xy = np.array(start) + (np.array(end)-start)*j/39
            stamp = 10+5*i+j//10
            ns = j%10*100_000_000
            p = pose(*xy, stamp, ns)
            capture.sample(p)
            world = capture.sample_truth(odom(xy[0]+.3, xy[1]-.1, stamp, ns))
            capture.add_registration_pair(world, p)
        capture.finish(grid())
        segments.append({'name': goal['name'], 'segment': 'Start→G1' if i == 0 else 'G1→G2',
                         'sent_pose': sent, 'endpoint_error_m': 0., 'goal_manager_metrics': {'path_length_m': 2.}})
        start = end
    (directory/'route-snapshot.json').write_text(json.dumps(config))
    (directory/'report.json').write_text(json.dumps({'manifest': {
        'head': '3'*40, 'warehouse_world_geometry_sha256': floor.world_geometry_sha256(ROOT/'third_party/aws_warehouse/warehouse.world'),
        'warehouse_collision_mesh_sha256': bench.manifest()['warehouse_collision_mesh_sha256']},
        'segments': segments, 'dry_run': False}))
    return capture


def test_canonical_provenance_and_exact_pixel_world_contract():
    metadata, image = floor.load_floorplan()
    assert metadata['frame_id'] == 'world'
    assert metadata['upstream_revision'] == 'ee0af733315e78432408c3cd98d378ecee5f767c'
    assert metadata['model_count'] == 25
    assert metadata['projection'] == 'all-height orthographic authored static collision triangles; GroundB is support'
    assert floor.sha256(floor.IMAGE) == metadata['image_sha256']
    assert image.shape[:2] == (metadata['height'], metadata['width'])
    assert metadata['world_geometry_sha256'] == floor.world_geometry_sha256(ROOT / 'third_party/aws_warehouse/warehouse.world')
    for source, sha in metadata['source_sha256'].items():
        assert floor.sha256(ROOT / source) == sha
    points = np.array([[0., 0.], [metadata['xmin'], metadata['ymax']], [1.234, -3.456]])
    pixels = floor.world_to_pixel(points, metadata)
    np.testing.assert_allclose(floor.pixel_to_world(pixels, metadata), points, atol=1e-12)
    np.testing.assert_allclose(pixels[1], [-.5, -.5], atol=1e-12)
    center = floor.pixel_to_world([[0, 0]], metadata)[0]
    assert center[0] == pytest.approx(metadata['xmin'] + metadata['resolution'] / 2)
    assert center[1] == pytest.approx(metadata['ymax'] - metadata['resolution'] / 2)


def test_missing_or_corrupt_canonical_floorplan_fails_explicitly(tmp_path):
    with pytest.raises((ValueError, FileNotFoundError)):
        floor.load_floorplan(tmp_path / 'absent.json')
    metadata = json.loads(floor.METADATA.read_text())
    metadata['image'] = 'bad.png'
    (tmp_path / 'bad.png').write_bytes(b'wrong')
    (tmp_path / 'floor.json').write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match='SHA'):
        floor.load_floorplan(tmp_path / 'floor.json')


def test_mesh_projection_raster_is_from_triangles_not_bounding_boxes():
    # Exact projected right triangle: upper-right half of its AABB is not filled.
    geometry = [{'support': False, 'triangles_xy': [[[0., 0.], [1., 0.], [0., 1.]]]}]
    meta = dict(xmin=0., xmax=1., ymin=0., ymax=1., resolution=.25, width=4, height=4)
    image = floor.rasterize(geometry, meta)
    assert tuple(image[3, 0]) == (20, 20, 20)
    assert tuple(image[0, 3]) != (20, 20, 20)


def test_world_geometry_digest_is_portable_and_pose_sensitive(tmp_path):
    world = ROOT / 'third_party/aws_warehouse/warehouse.world'
    text = world.read_text()
    moved = tmp_path / 'moved.world'
    moved.write_text(text.replace(str(ROOT), '/another/checkout'))
    assert floor.world_geometry_sha256(moved) == floor.world_geometry_sha256(world)
    moved.write_text(text.replace('-5.795143', '-5.795144', 1))
    assert floor.world_geometry_sha256(moved) != floor.world_geometry_sha256(world)


def test_legacy_empty_pose_frame_is_default_but_named_frames_are_rejected():
    import xml.etree.ElementTree as ET
    empty = ET.fromstring('<link><pose frame="">1 2 3 0 0 0</pose></link>')
    np.testing.assert_allclose(floor.pose_matrix(empty)[:3, 3], [1, 2, 3])
    empty.find('pose').set('frame', 'another_frame')
    with pytest.raises(ValueError, match='pose'):
        floor.pose_matrix(empty)


def test_truth_capture_keeps_world_coordinates_and_never_changes_map_goal(tmp_path):
    import run_warehouse_route as bench
    from warehouse_route_evidence import RouteEvidence
    capture = RouteEvidence(tmp_path, bench.load_route())
    target = bench.critical_goals(bench.load_route())[0]['pose']
    capture.begin(target)
    capture.sample_truth(odom(123.25, -45.5, 10))
    capture.finish(None)
    index = json.loads((tmp_path/'evidence/index.json').read_text())
    saved = index['segments'][0]
    truth = json.loads((tmp_path/saved['truth_path_file']).read_text())
    assert truth['header']['frame_id'] == 'world'
    assert truth['poses'][0]['pose']['position']['x'] == 123.25
    assert saved['truth_source']['source_frame_ids'] == ['sim_world']
    assert saved['sent_pose'] == target


def test_registration_acceptance_and_rejection():
    points = np.column_stack((np.linspace(0, 4, 50), np.sin(np.linspace(0, 4, 50))))
    angle = .2
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    world = points @ rotation.T + [.3, -.1]
    pairs = [{'stamp_ns': i*100_000_000, 'map_xy': a.tolist(), 'world_xy': b.tolist()}
             for i, (a, b) in enumerate(zip(points, world))]
    fit = primary.fit_registration(pairs)
    assert fit['accepted'] and fit['rms_residual_m'] < 1e-12
    np.testing.assert_allclose(fit['rotation'], rotation, atol=1e-12)
    np.testing.assert_allclose(fit['translation'], [.3, -.1], atol=1e-12)
    assert not primary.fit_registration(pairs[:29])['accepted']
    bad = copy.deepcopy(pairs)
    bad[-1]['world_xy'][0] += 1.
    assert not primary.fit_registration(bad)['accepted']
    stationary = [{'stamp_ns': i, 'map_xy': [0., 0.], 'world_xy': [1., 1.]} for i in range(50)]
    assert not primary.fit_registration(stationary)['accepted']
    conflict = copy.deepcopy(pairs)
    conflict[-1]['stamp_ns'] = pairs[0]['stamp_ns']
    assert not primary.fit_registration(conflict)['accepted']


def test_composite_horizontal_truth_primary_and_diagnostic_preserved(tmp_path):
    import plot_warehouse_route as plotting
    run = tmp_path/'truth-run'
    truth_run(run)
    result = plotting.render_run(run, label='MOCK truth/path; canonical sourced floorplan')
    assert result['primary']['status'] == 'generated'
    assert result['primary']['registration']['accepted']
    composite = Image.open(result['composite_image_path'])
    left = Image.open(result['primary']['images'][0]['image_path'])
    right = Image.open(result['images'][0]['image_path'])
    assert composite.height == max(left.height, right.height)
    assert composite.width == left.width + right.width
    assert result['composite_layout'] == dict(left='PRIMARY', right='DIAGNOSTIC',
        primary_rectangle=[0, 0, left.width, left.height],
        diagnostic_rectangle=[left.width, 0, left.width+right.width, right.height])
    np.testing.assert_array_equal(np.asarray(composite)[:left.height, :left.width], np.asarray(left).astype(np.uint8)[..., :3])
    np.testing.assert_array_equal(np.asarray(composite)[:right.height, left.width:], np.asarray(right).astype(np.uint8)[..., :3])
    assert (run/'overlays/G1-G2.png').is_file()  # Existing diagnostic remains.
    assert result['images'][0]['role'] == 'DIAGNOSTIC'
    assert result['primary']['images'][0]['role'] == 'PRIMARY'
    assert [r['segment_id'] for r in result['primary']['images']] == ['full-route', 'Start-G1', 'G1-G2']
    assert result['primary']['images'][2]['path_length_m'] > 4.


def test_horizontal_composite_pads_unequal_heights_without_resizing(tmp_path, monkeypatch):
    import plot_warehouse_route as plotting
    draw = primary.draw_primary
    def taller_panel(destination, *args, **kwargs):
        draw(destination, *args, **kwargs)
        original = Image.open(destination).convert('RGB')
        padded = Image.new('RGB', (original.width, original.height+40), 'white')
        padded.paste(original, (0, 0))
        padded.save(destination)
    monkeypatch.setattr(primary, 'draw_primary', taller_panel)
    run = tmp_path/'unequal-heights'
    truth_run(run)
    result = plotting.render_run(run, label='MOCK unequal panel heights')
    left = Image.open(result['primary']['images'][0]['image_path']).convert('RGB')
    right = Image.open(result['images'][0]['image_path']).convert('RGB')
    composite = np.asarray(Image.open(result['composite_image_path']))
    assert composite.shape == (left.height, left.width+right.width, 3)
    assert result['composite_layout']['primary_rectangle'] == [0, 0, left.width, left.height]
    assert result['composite_layout']['diagnostic_rectangle'] == [left.width, 20, left.width+right.width, 20+right.height]
    np.testing.assert_array_equal(composite[:, :left.width], np.asarray(left))
    np.testing.assert_array_equal(composite[20:20+right.height, left.width:], np.asarray(right))
    assert np.all(composite[:20, left.width:] == 255)
    assert np.all(composite[20+right.height:, left.width:] == 255)


@pytest.mark.parametrize('missing', ['truth', 'floorplan', 'world_mismatch', 'mesh_mismatch'])
def test_unavailable_primary_keeps_composite_and_diagnostic(tmp_path, monkeypatch, missing):
    import plot_warehouse_route as plotting
    run = tmp_path/missing
    truth_run(run)
    if missing == 'truth':
        path = run/'evidence/index.json'; index = json.loads(path.read_text())
        for segment in index['segments']: segment['truth_path_file'] = None
        path.write_text(json.dumps(index))
    elif missing == 'floorplan':
        monkeypatch.setattr(primary, 'FLOORPLAN', tmp_path/'missing.json')
    else:
        path = run/'report.json'; report = json.loads(path.read_text())
        if missing == 'world_mismatch':
            report['manifest']['warehouse_world_geometry_sha256'] = 'wrong'
        else:
            mesh = next(iter(report['manifest']['warehouse_collision_mesh_sha256']))
            report['manifest']['warehouse_collision_mesh_sha256'][mesh] = 'wrong'
        path.write_text(json.dumps(report))
    result = plotting.render_run(run)
    assert result['primary']['status'] == 'unavailable'
    assert Path(result['composite_image_path']).is_file()
    assert (run/'overlays/full-route.png').is_file()


def test_truth_available_without_diagnostic_map(tmp_path):
    import plot_warehouse_route as plotting
    run = tmp_path/'no-slam-map'
    truth_run(run)
    index_file = run/'evidence/index.json'
    index = json.loads(index_file.read_text())
    for segment in index['segments']: segment['map_source'] = None
    index_file.write_text(json.dumps(index))
    result = plotting.render_run(run)
    assert result['primary']['status'] == 'generated'
    assert all(r['status'] == 'unavailable' for r in result['images'])
    assert Path(result['composite_image_path']).is_file()


@pytest.mark.parametrize('source', ['map_source', 'path_file'])
def test_corrupt_diagnostic_does_not_suppress_valid_truth(tmp_path, source):
    import plot_warehouse_route as plotting
    run = tmp_path/source
    truth_run(run)
    index = json.loads((run/'evidence/index.json').read_text())
    for segment in index['segments']:
        filename = segment[source]['file'] if source == 'map_source' else segment[source]
        (run/filename).write_text('{}')
    result = plotting.render_run(run)
    assert result['primary']['status'] == 'generated'
    assert all(r['status'] == 'unavailable' for r in result['images'])
    assert any('SHA' in str(error) for error in result['capture_errors'])
    assert Path(result['composite_image_path']).is_file()


def test_rejected_registration_never_guesses_primary_waypoints(tmp_path):
    import plot_warehouse_route as plotting
    run = tmp_path/'no-registration'
    truth_run(run)
    index_file = run/'evidence/index.json'
    index = json.loads(index_file.read_text())
    for segment in index['segments']: segment['registration_file'] = None
    index_file.write_text(json.dumps(index))
    result = plotting.render_run(run)
    assert result['primary']['status'] == 'generated'
    assert not result['primary']['registration']['accepted']
    assert result['primary']['images'][0]['waypoints'] == {}
    assert len(result['images'][0]['waypoints']) == 5


def test_passive_truth_transport_pairs_exact_stamp_and_contains_bad_data(tmp_path):
    from types import SimpleNamespace
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.set_message import set_message_fields
    import run_warehouse_route as bench
    live = bench.RosGoalTransport(evidence_directory=tmp_path)
    callbacks = {}
    live.node = SimpleNamespace(create_subscription=lambda cls, topic, cb, qos: callbacks.update({topic: cb}))
    live.convert = message_to_ordereddict
    live.attach_truth_evidence()
    target = bench.critical_goals(bench.load_route())[0]['pose']
    live.evidence.begin(target)
    received = Odometry()
    set_message_fields(received, odom(2., 3., 10, 200_000_000))
    callbacks['/sim/ground_truth/odom'](received)
    requested = []
    def lookup(parent, child, stamp):
        requested.append((parent, child, stamp.nanoseconds))
        tr = TransformStamped()
        tr.header.frame_id = 'map'; tr.child_frame_id = 'base_link'
        tr.header.stamp = received.header.stamp
        tr.transform.rotation.w = 1.
        return tr
    live.tf = SimpleNamespace(lookup_transform=lookup)
    live.pair_truth_evidence()
    assert requested == [('map', 'base_link', 10_200_000_000)]
    assert len(live.evidence.current['registration_pairs']) == 1
    assert live.evidence.current['sent_pose'] == target
    # Unsupported frame invalidates only truth evidence, never the map goal.
    received.header.frame_id = 'invented-frame'
    callbacks['/sim/ground_truth/odom'](received)
    assert live.evidence.current['truth_invalid_reason']
    assert live.evidence.current['invalid_reason'] is None
    assert live.evidence.current['sent_pose'] == target


def test_truth_and_registration_clock_contract(tmp_path):
    import run_warehouse_route as bench
    from warehouse_route_evidence import RouteEvidence
    from test_warehouse_route_overlays import pose
    capture = RouteEvidence(tmp_path, bench.load_route())
    capture.begin(bench.critical_goals(bench.load_route())[0]['pose'])
    truth = capture.sample_truth(odom(1., 2., 10))
    with pytest.raises(ValueError, match='synchronized'):
        capture.add_registration_pair(truth, pose(1., 2., 11))
    capture.observe_clock(20_000_000_000)
    capture.observe_clock(1_000_000_000)
    assert capture.current['truth_invalid_reason']
    assert capture.sample_truth(odom(1., 2., 12)) is None


def test_run_report_recognizes_generated_primary_without_diagnostic(tmp_path, monkeypatch):
    import run_warehouse_route as bench
    import plot_warehouse_route as plotting
    from test_warehouse_route import Transport
    monkeypatch.setattr(plotting, 'render_run', lambda *a: dict(
        primary={'status': 'generated'}, composite_image_path='/mock/route-comparison.png',
        images=[{'status': 'unavailable'}]))
    assert bench.main(['--execute', '--output-root', str(tmp_path)], transport_factory=Transport) == 0
    report = json.loads(next(tmp_path.glob('*/report.json')).read_text())
    assert report['visualization']['status'] == 'generated'
    assert report['visualization']['primary_status'] == 'generated'
    assert report['stop_reason'] == 'critical_succeeded'


def test_unavailable_world_provenance_does_not_change_navigation(tmp_path, monkeypatch):
    import run_warehouse_route as bench
    from test_warehouse_route import Transport
    def unreadable(*args):
        raise OSError('Evaluation world provenance unavailable')
    monkeypatch.setattr(floor, 'world_geometry_sha256', unreadable)
    assert bench.main(['--execute', '--output-root', str(tmp_path)], transport_factory=Transport) == 0
    report = json.loads(next(tmp_path.glob('*/report.json')).read_text())
    assert report['stop_reason'] == 'critical_succeeded'
    assert report['success'] is True
    assert report['manifest']['warehouse_geometry_error']
    assert report['visualization']['primary_status'] == 'unavailable'
