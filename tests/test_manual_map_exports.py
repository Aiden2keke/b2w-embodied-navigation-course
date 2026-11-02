import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

from PIL import Image
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import warehouse_manual_logger as logger
import warehouse_truth_floorplan as floor


SESSION = ROOT / 'course_results/tracks/20261102T031146Z'


@pytest.mark.skipif(not SESSION.is_dir(), reason='recorded 20261102T031146Z session is not present')
def test_recorded_session_exports_both_registered_maps_without_rewriting_source(tmp_path):
    source_csv = SESSION / 'trajectory.csv'
    original_csv = source_csv.read_bytes()
    result = logger.export_recorded_session(SESSION, tmp_path)

    assert source_csv.read_bytes() == original_csv
    assert (tmp_path / 'trajectory_ground_truth.png').is_file()
    assert (tmp_path / 'trajectory_slam.png').is_file()
    metadata = json.loads((tmp_path / 'result.json').read_text())
    assert metadata['trajectory_source'] == '/sim/ground_truth/odom'
    assert metadata['motion_classification'] == 'stationary'
    assert metadata['images']['slam']['available'] is True
    assert metadata['images']['slam']['trajectory_drawn'] is False
    assert metadata['images']['slam']['registration_sha256'] == logger.sha256(logger.SAVED_REGISTRATION)
    assert result['trajectory_samples'] == 5540


@pytest.mark.skipif(not SESSION.is_dir(), reason='recorded 20261102T031146Z session is not present')
def test_saved_map_start_and_end_use_accepted_world_to_slam_transform(tmp_path):
    summary = logger.export_recorded_session(SESSION, tmp_path)
    transform = json.loads(logger.SAVED_REGISTRATION.read_text())['transform']
    c, s = math.cos(transform['yaw_rad']), math.sin(transform['yaw_rad'])
    for source, expected in ((summary['start_world_xy'], summary['start_saved_map_xy']),
                             (summary['end_world_xy'], summary['end_saved_map_xy'])):
        actual = [c * source[0] - s * source[1] + transform['x_m'],
                  s * source[0] + c * source[1] + transform['y_m']]
        assert np.allclose(actual, expected, atol=1e-12)
    world_meta, _ = floor.load_floorplan()
    assert np.allclose(summary['start_world_pixel_xy'],
                       floor.world_to_pixel([summary['start_world_xy']], world_meta)[0])
    assert np.allclose(summary['end_world_pixel_xy'],
                       floor.world_to_pixel([summary['end_world_xy']], world_meta)[0])

    report = json.loads((tmp_path / 'result.json').read_text())['images']['slam']
    ox, oy, _ = report['origin']; res = report['resolution_m_per_pixel']
    width, height = report['pgm_size_px']
    sx, sy = summary['start_saved_map_xy']
    expected_saved_pixel = [(sx-ox)/res-.5, (oy+height*res-sy)/res-.5]
    assert np.allclose(report['start_image_pixel_xy'], expected_saved_pixel, atol=1e-12)
    crop = report['display_crop_pixels']
    assert np.allclose(report['start_display_pixel_xy'],
                       [expected_saved_pixel[0]-crop['col0'], expected_saved_pixel[1]-crop['row0']])
    assert crop['col1']-crop['col0'] < width and crop['row1']-crop['row0'] < height


@pytest.mark.skipif(not SESSION.is_dir(), reason='recorded 20261102T031146Z session is not present')
def test_stationary_session_has_no_blue_travel_line_and_uses_real_map_assets(tmp_path):
    logger.export_recorded_session(SESSION, tmp_path)
    gt = np.asarray(Image.open(tmp_path / 'trajectory_ground_truth.png').convert('RGB'))
    slam = np.asarray(Image.open(tmp_path / 'trajectory_slam.png').convert('RGB'))
    assert gt.shape[0] >= 1600 and slam.shape[0] >= 1600
    # A stationary capture must not render its measured balance drift as a route.
    blue = (slam[:, :, 2] > 200) & (slam[:, :, 1] > 70) & (slam[:, :, 0] < 80)
    assert int(blue.sum()) == 0
    assert int(np.all(slam < 30, axis=2).sum()) > 1000  # Original SLAM occupied/noise pixels remain visible.


@pytest.mark.skipif(not SESSION.is_dir(), reason='recorded 20261102T031146Z session is not present')
def test_ctrl_c_runs_dual_map_export_and_ros_cleanup(tmp_path):
    rows = logger._read_truth_csv(SESSION / 'trajectory.csv')
    metadata, background = logger.load_fixed_map(ROOT / 'third_party/aws_warehouse/warehouse.world')
    registration, error = logger.load_saved_registration()
    assert registration is not None, error

    class FakeRclpy:
        class executors:
            class ExternalShutdownException(Exception):
                pass

        @staticmethod
        def spin(_node):
            raise KeyboardInterrupt

        @staticmethod
        def ok():
            return True

        def __init__(self):
            self.shutdown_called = False

        def shutdown(self):
            self.shutdown_called = True

    class FakeNode:
        def __init__(self):
            self.destroyed = False

        @staticmethod
        def get_clock():
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=rows[-1]['timestamp_ns']+1))

        def destroy_node(self):
            self.destroyed = True

    capture = logger.ManualLogger.__new__(logger.ManualLogger)
    capture.rclpy = FakeRclpy()
    capture.node = FakeNode()
    capture.output = tmp_path
    capture.truth_rows = rows
    capture.fixed_metadata, capture.fixed_image = metadata, background
    capture.saved_registration = registration
    capture.started = 0
    capture.stop_reason = 'shutdown'
    capture.motion_enabled = False
    capture.run()

    report = json.loads((tmp_path / 'result.json').read_text())
    assert report['stop_reason'] == 'interrupted'
    assert report['images']['slam']['available'] is True
    assert all((tmp_path / name).is_file() for name in
               ('trajectory.csv', 'trajectory_ground_truth.png', 'trajectory_slam.png', 'result.json'))
    assert capture.node.destroyed and capture.rclpy.shutdown_called
