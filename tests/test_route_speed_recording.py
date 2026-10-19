import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from route_speed_recording import SpeedEvidence


def test_physical_ground_speed_uses_planar_norm_and_reports_modes(tmp_path):
    evidence = SpeedEvidence(tmp_path)
    evidence.status({'stamp_s': 1., 'mode': 'NARROW', 'command_limit': .11})
    evidence.odometry('truth', 1.1, .108, .144)  # Exactly .18 planar physical speed.
    evidence.status({'stamp_s': 2., 'mode': 'OPEN', 'command_limit': .43})
    evidence.odometry('truth', 2.1, .3, .4)
    evidence.odometry('slam', 2.1, .28, .4)
    evidence.odometry('truth', 4., .4, 0.)  # No fresh mode; do not label OPEN.
    evidence.odometry('truth', 4.1, float('nan'), 0.)
    evidence.close()
    summary = json.loads((tmp_path/'speed-summary.json').read_text())
    assert summary['truth']['NARROW']['median_mps'] == pytest.approx(.18)
    assert summary['truth']['OPEN']['max_mps'] == .5
    assert summary['truth']['UNKNOWN']['count'] == 1
    assert summary['invalid_samples'] == 1
    assert (tmp_path/'speed-trace.jsonl').is_file()


def test_clock_reset_drops_mode_and_missing_evidence_is_null(tmp_path):
    evidence = SpeedEvidence(tmp_path)
    evidence.status({'stamp_s': 10., 'mode': 'OPEN', 'command_limit': .43})
    evidence.odometry('truth', 2., .2, 0.)
    evidence.close()
    summary = json.loads((tmp_path/'speed-summary.json').read_text())
    assert summary['truth']['OPEN']['max_mps'] is None
    assert summary['truth']['UNKNOWN']['count'] == 1


def test_new_status_epoch_cannot_reuse_old_open_when_clock_catches_up(tmp_path):
    evidence = SpeedEvidence(tmp_path)
    evidence.status({'stamp_s': 10., 'mode': 'OPEN', 'command_limit': .43})
    evidence.status({'stamp_s': 2., 'mode': 'NARROW', 'command_limit': .11})
    evidence.odometry('truth', 10.1, .4, 0.)
    evidence.close()
    summary = json.loads((tmp_path/'speed-summary.json').read_text())
    assert summary['truth']['OPEN']['count'] == 0
    assert summary['truth']['UNKNOWN']['count'] == 1


def test_recorder_writes_command_chain_and_loaded_planner_snapshot(tmp_path):
    from concurrent.futures import Future
    from types import SimpleNamespace as NS
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from route_speed_recording import RouteSpeedRecorder
    callbacks = {}
    requests = {}
    def client(kind, service):
        def call(request):
            requests[service] = list(request.names)
            future = Future()
            future.set_result(NS(values=[Parameter(name, value=(
                'nav2_smac_planner/SmacPlanner2D' if name == 'GridBased.plugin' else .43
            )).get_parameter_value() for name in request.names]))
            return future
        return NS(call_async=call)
    node = NS(create_subscription=lambda kind, topic, cb, qos: callbacks.setdefault(topic, cb),
              create_client=client, get_clock=lambda: NS(now=lambda: NS(nanoseconds=2_000_000_000)))
    recorder = RouteSpeedRecorder(node, tmp_path)
    for topic in ('/course_navigation/cmd_vel_raw', '/course_navigation/cmd_vel',
                  '/path_manager/path_manager_ros/nav_vel'):
        msg = Twist()
        msg.linear.x, msg.angular.z = .11, .6
        callbacks[topic](msg)
    recorder.close()
    snapshot = json.loads((tmp_path/'speed-runtime-parameters.json').read_text())
    assert snapshot['planner_server']['GridBased.plugin'] == 'nav2_smac_planner/SmacPlanner2D'
    assert 'm2_inverse_enabled' in requests['/velocity_adapter/get_parameters']
    events = [json.loads(line) for line in (tmp_path/'speed-trace.jsonl').read_text().splitlines()]
    assert len(events) == 3
    assert all(e['command'] == [.11, 0., .6] for e in events)
    assert {e['topic'] for e in events} == {
        '/course_navigation/cmd_vel_raw', '/course_navigation/cmd_vel',
        '/path_manager/path_manager_ros/nav_vel'}
