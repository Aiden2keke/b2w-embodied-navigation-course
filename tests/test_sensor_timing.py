"""Guard XT32 encoding semantics and Python work at the 10 Hz scan boundary."""
import cProfile
import importlib.util
import pstats
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from course_sim.cloud_format import OUTPUT, decode, encode


def test_full_xt32_encoding_preserves_payload_without_per_byte_python_work():
    points = np.zeros(64000, dtype=OUTPUT)
    points['x'] = np.linspace(1., 10., len(points))
    points['ring'] = np.arange(len(points)) % 32
    header = Header(frame_id='lidar_link')
    header.stamp.sec, header.stamp.nanosec = 12, 3456
    profile = cProfile.Profile()
    message = profile.runcall(encode, points, header)
    assert message.header == header
    assert message.width == 64000 and message.point_step == 24
    assert bytes(message.data) == points.tobytes()
    np.testing.assert_array_equal(decode(message), points)
    assert deserialize_message(serialize_message(message), PointCloud2) == message
    # The original bytes setter performs millions of Python calls for a scan.
    # Count interpreter work rather than asserting a machine-dependent duration.
    assert pstats.Stats(profile).total_calls < 10000


def test_encoding_empty_and_strided_arrays_preserves_wire_layout():
    points = np.zeros(12, dtype=OUTPUT)
    points['x'] = np.arange(12)
    for selected in [points[::2], points[:0]]:
        message = encode(selected, Header(frame_id='lidar_link'))
        assert bytes(message.data) == selected.tobytes()
        assert message.row_step == len(selected) * OUTPUT.itemsize
        np.testing.assert_array_equal(decode(message), selected)


def test_timing_summary_retains_signed_age_gaps_and_missing_data():
    path = Path(__file__).resolve().parents[1] / 'scripts/check_sensor_timing.py'
    spec = importlib.util.spec_from_file_location('sensor_timing_probe', path)
    probe = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(probe)
    result = probe.summarize({'/imu': [
        {'wall': 1., 'stamp': 10., 'clock': 9.99},
        {'wall': 1.5, 'stamp': 10.5, 'clock': 10.7},
        {'wall': 2., 'stamp': 11., 'clock': None}], '/missing': [],
        '/clock': [{'wall': 1., 'stamp': 10., 'clock': 10.}]}, 2.)
    assert result['/imu']['received_count'] == 3
    assert result['/imu']['effective_sim_hz'] == 2.
    assert result['/imu']['clock_minus_header_s']['samples'] == 2
    np.testing.assert_allclose(result['/imu']['clock_minus_header_s']['max'], .2, atol=1e-12)
    np.testing.assert_allclose(result['/imu']['clock_minus_header_s']['p50'], .095, atol=1e-12)
    assert result['/imu']['inter_message_wall_gap_s']['max'] == .5
    assert result['/missing']['effective_sim_hz'] is None
    assert result['/missing']['clock_minus_header_s']['p50'] is None
    assert result['/clock']['first_header_stamp_s'] is None
    assert result['/clock']['stamp_source'] == 'clock'
