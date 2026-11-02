import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

from warehouse_track_artifacts import evaluate_physical_waypoints, write_track_artifacts


def test_interrupted_run_writes_actual_tf_track_artifacts(tmp_path):
    route = {'route_id': 'warehouse-g1-g5', 'frame_id': 'map', 'goals': [
        {'name': f'G{i}', 'position': [float(i), 0.]} for i in range(1, 6)]}
    stamp = lambda n: {'sec': n, 'nanosec': 0}
    def pose(n, x):
        return {'header': {'frame_id': 'map', 'stamp': stamp(n)}, 'pose': {
            'position': {'x': x, 'y': 0., 'z': 0.},
            'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}}
    grid = {'header': {'frame_id': 'map', 'stamp': stamp(3)}, 'info': {
        'width': 4, 'height': 3, 'resolution': 1., 'origin': {
            'position': {'x': -1., 'y': -1., 'z': 0.},
            'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}},
        'data': [0, 0, 0, 0, 0, 100, 100, 0, 0, 0, 0, 0]}
    paths = [pose(1, 0.), pose(2, .5), pose(3, .8)]
    report = {'started_wall_s': 10., 'ended_wall_s': 12., 'stop_reason': 'interrupted',
              'segments': [{'name': 'G1', 'navigation_success': True}],
              'waypoint_observations': [{'name': 'G1'}]}
    index = {'segments': [{'waypoint': 'G1', 'path': {
        'header': {'frame_id': 'map', 'stamp': stamp(3)}, 'poses': paths}, 'map': grid}]}

    result = write_track_artifacts(tmp_path, route, report, index)

    assert (tmp_path / 'trajectory.png').is_file()
    rows = list(csv.DictReader((tmp_path / 'trajectory.csv').open()))
    assert [float(row['x']) for row in rows] == [0., .5, .8]
    saved = json.loads((tmp_path / 'result.json').read_text())
    assert saved['outcome'] == 'interrupted'
    assert saved['reached_goals'] == ['G1']
    assert saved['failure_reason'] == 'interrupted'
    assert result['trajectory_samples'] == 3


def test_full_success_requires_public_tf_truth_evaluation_without_video(tmp_path):
    route = {'goals': [{'name': f'G{i}', 'position': [float(i), 0.]} for i in range(1, 6)]}
    planned = [{'name': f'G{i}', 'yaw_rad': 0., 'pose': {'pose': {'position': {'x': float(i), 'y': 0.}}}}
               for i in range(1, 6)]
    segments = []
    for i, g in enumerate(route['goals'], 1):
        sent = {'pose': {'position': {'x': g['position'][0], 'y': 0.}}}
        metrics = {'success': True, 'nav2_status': 4, 'goal_pose': sent}
        metric_file = tmp_path/f'{i}.json';metric_file.write_text(json.dumps(metrics))
        import hashlib
        segments.append({'name': g['name'], 'navigation_success': True, 'evaluation_passed': True,
            'evidence_complete': True, 'nav2_result': {'status': 4}, 'sent_pose': sent,
            'metrics_file': str(metric_file), 'metrics_sha256': hashlib.sha256(metric_file.read_bytes()).hexdigest(),
            'goal_manager_metrics': metrics})
    observations = [{'name': f'G{i}', 'sim': float(i), 'truth_stamp': float(i),
                     'truth_world_se2': [float(i)+10., 2., 0.],
                     'localized_se2': [float(i), 0., 0.], 'contact_samples': 2,
                     'contact_failure': None, 'readiness': {'passed': True},
                     'sent_pose': segments[i-1]['sent_pose']}
                    for i in range(1, 6)]
    pairs = [{'stamp_ns': i*1_000_000_000, 'map_xy': [float(i), 0.],
              'world_xy': [float(i)+10., 2.]} for i in range(1, 6)]
    admissions = [dict(passed=True, complete_leg_path_checked=True, admission_label=f'G{i}',
                       admission_goal_xy=[float(i), 0.],
                       admitted_path=[[float(i)-.1, 0., 0.], [float(i), 0., 0.]])
                  for i in range(1, 6)]
    stops = [dict(waypoint=f'G{i}', measured_lio_rest=True, measured_ground_truth_rest=True,
                  six_axis_final_zero=True, wall_s=3., start_sim=float(i), end_sim=float(i+5))
             for i in range(1, 5)]

    result = evaluate_physical_waypoints(route, dict(success=True, planned_goals=planned,
        segments=segments), observations, pairs, metrics_root=tmp_path,
        leg_admissions=admissions, settled_stops=stops)

    assert result['status'] == 'passed'
    assert result['verified_waypoints'] == ['G1', 'G2', 'G3', 'G4', 'G5']
    metric = Path(segments[0]['metrics_file'])
    metric.write_text(metric.read_text()+' ')
    rejected = evaluate_physical_waypoints(route, dict(success=True, planned_goals=planned,
        segments=segments), observations, pairs, metrics_root=tmp_path,
        leg_admissions=admissions, settled_stops=stops)
    assert rejected['status'] == 'incomplete'
    assert rejected['reason'] == 'goal_manager_metrics_path_or_digest_failed'
