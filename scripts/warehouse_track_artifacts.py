"""Write compact, video-free per-run route artifacts from public map-frame TF evidence."""
import csv
import json
import math
from pathlib import Path

from run_narrow_baseline import digest, write
from warehouse_route_evidence import map_geometry, path_chunks, route_content_sha256, stamp_ns


def _load_ref(root, ref, key, sha_key):
    path = (root / ref[key]).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError('Evidence path escapes run or is missing')
    if digest(path) != ref[sha_key]:
        raise ValueError('Evidence digest mismatch')
    return json.loads(path.read_text())


def _pose_rows(poses, segment):
    rows = []
    for p in poses:
        if p['header']['frame_id'] != 'map':
            raise ValueError('Trajectory must use the public map frame')
        q, pos = p['pose']['orientation'], p['pose']['position']
        yaw = math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']),
                         1 - 2 * (q['y']**2 + q['z']**2))
        rows.append(dict(timestamp_ns=stamp_ns(p['header']), x=pos['x'], y=pos['y'],
                         yaw_rad=yaw, segment=segment, source='map->base_link TF'))
    return rows


def evaluate_physical_waypoints(route, route_report, observations, registration_pairs, *, metrics_root=None,
                               leg_admissions=None, settled_stops=None):
    """Evaluation-only truth/map check; never called by planning or motion admission."""
    def reject(reason, **details):
        return dict(status='incomplete', reason=reason, verified_waypoints=[], **details)
    expected = route['goals']
    segments = route_report.get('segments', [])
    if (route_report.get('success') is not True or len(segments) != 5 or len(observations) != 5 or
            [s.get('name') for s in segments] != [g['name'] for g in expected] or
            [o.get('name') for o in observations] != [g['name'] for g in expected]):
        return reject('full_route_waypoint_records_missing_or_incomplete')
    if len(registration_pairs) < 5:
        return reject('insufficient_synchronized_tf_truth_registration_samples', samples=len(registration_pairs))
    leg_admissions = leg_admissions or []
    if (len(leg_admissions) != 5 or any(a.get('passed') is not True or
            a.get('complete_leg_path_checked') is not True or a.get('admission_label') != f'G{i}'
            for i, a in enumerate(leg_admissions, 1))):
        return reject('per_leg_full_path_or_stopping_admission_missing')
    settled_stops = settled_stops or []
    if (len(settled_stops) != 4 or [s.get('waypoint') for s in settled_stops] !=
            ['G1', 'G2', 'G3', 'G4'] or any(not all(s.get(k) is True for k in
            ('measured_lio_rest', 'measured_ground_truth_rest', 'six_axis_final_zero')) or
            s.get('wall_s', 0) < 3 or s.get('end_sim', 0)-s.get('start_sim', 0) < 5
            for s in settled_stops)):
        return reject('settled_stop_checkpoint_missing_or_invalid')
    if metrics_root is None:
        return reject('goal_manager_metrics_root_unavailable')
    metrics_root = Path(metrics_root).resolve()
    map_xy = [[p['map_xy'][0], p['map_xy'][1]] for p in registration_pairs]
    world_xy = [[p['world_xy'][0], p['world_xy'][1]] for p in registration_pairs]
    if any(not all(math.isfinite(v) for v in point) for point in map_xy+world_xy):
        return reject('nonfinite_registration_pair')
    mx = sum(p[0] for p in map_xy)/len(map_xy); my = sum(p[1] for p in map_xy)/len(map_xy)
    wx = sum(p[0] for p in world_xy)/len(world_xy); wy = sum(p[1] for p in world_xy)/len(world_xy)
    spread = sum((p[0]-mx)**2+(p[1]-my)**2 for p in map_xy)
    if spread < .25:
        return reject('registration_track_has_insufficient_spatial_baseline', spread_m2=spread)
    a = sum((p[0]-mx)*(q[0]-wx)+(p[1]-my)*(q[1]-wy) for p, q in zip(map_xy, world_xy))
    b = sum((p[0]-mx)*(q[1]-wy)-(p[1]-my)*(q[0]-wx) for p, q in zip(map_xy, world_xy))
    if math.hypot(a, b) < 1e-8:
        return reject('registration_rotation_unobservable')
    yaw = math.atan2(b, a); c, s = math.cos(yaw), math.sin(yaw)
    tx, ty = wx-(c*mx-s*my), wy-(s*mx+c*my)
    transform = [tx, ty, yaw]
    residuals = [math.hypot(tx+c*p[0]-s*p[1]-q[0], ty+s*p[0]+c*p[1]-q[1])
                 for p, q in zip(map_xy, world_xy)]
    if max(residuals) > .05:
        return reject('tf_truth_registration_residual_exceeded', max_residual_m=max(residuals),
                      transform_world_from_map=transform)
    angle_error = lambda value: abs(math.atan2(math.sin(value), math.cos(value)))
    verified, results, previous_sim = [], [], -math.inf
    for index, (goal, segment, obs) in enumerate(zip(expected, segments, observations)):
        admission = leg_admissions[index]
        admitted_path = admission.get('admitted_path', [])
        if (admission.get('admission_goal_xy') != goal['position'] or len(admitted_path) < 2 or
                math.dist(admitted_path[-1][:2], goal['position']) > .05):
            return reject('per_leg_admission_target_or_path_mismatch', waypoint=goal['name'])
        if (segment.get('navigation_success') is not True or segment.get('evaluation_passed') is not True or
                segment.get('evidence_complete') is not True or segment.get('nav2_result', {}).get('status') != 4):
            return reject('goal_manager_or_strict_localization_evaluation_failed', waypoint=goal['name'])
        metrics = segment.get('goal_manager_metrics', {})
        try:
            metrics_path = Path(segment['metrics_file']).resolve()
            if (not metrics_path.is_relative_to(metrics_root) or not metrics_path.is_file() or
                    digest(metrics_path) != segment['metrics_sha256']):
                return reject('goal_manager_metrics_path_or_digest_failed', waypoint=goal['name'])
            stored_metrics = json.loads(metrics_path.read_text())
        except (KeyError, OSError, ValueError, TypeError) as error:
            return reject('goal_manager_metrics_artifact_unavailable', waypoint=goal['name'], detail=repr(error))
        if (metrics.get('success') is not True or metrics.get('nav2_status') != 4 or
                metrics.get('goal_pose') != segment.get('sent_pose') or
                stored_metrics.get('success') is not True or stored_metrics.get('nav2_status') != 4 or
                stored_metrics.get('goal_pose') != segment.get('sent_pose')):
            return reject('goal_manager_metrics_identity_or_status_failed', waypoint=goal['name'])
        if segment.get('sent_pose', {}).get('pose', {}).get('position', {}).get('x') != goal['position'][0] or \
                segment.get('sent_pose', {}).get('pose', {}).get('position', {}).get('y') != goal['position'][1]:
            return reject('sent_waypoint_differs_from_canonical_route', waypoint=goal['name'])
        try:
            sim, truth_stamp = obs['sim'], obs['truth_stamp']
            truth, estimate = obs['truth_world_se2'], obs['localized_se2']
            if (not all(math.isfinite(v) for v in [sim, truth_stamp]+truth+estimate) or
                    sim <= previous_sim or not 0 <= sim-truth_stamp <= .5 or
                    obs['contact_failure'] or obs['contact_samples'] <= 0 or
                    obs['readiness'].get('passed') is not True or obs['sent_pose'] != segment['sent_pose']):
                return reject('waypoint_observation_freshness_contact_or_identity_failed', waypoint=goal['name'])
            nearest = min(abs(p['stamp_ns']*1e-9-truth_stamp) for p in registration_pairs)
            if nearest > .5:
                return reject('waypoint_lacks_nearby_synchronized_tf_truth_pair', waypoint=goal['name'],
                              nearest_registration_age_s=nearest)
            gx, gy = goal['position']
            target_x, target_y = tx+c*gx-s*gy, ty+s*gx+c*gy
            position_error = math.hypot(truth[0]-target_x, truth[1]-target_y)
            estimate_error = math.hypot(estimate[0]-gx, estimate[1]-gy)
            yaw_goal = route_report['planned_goals'][len(verified)]['yaw_rad']
            truth_yaw_error = angle_error(truth[2]-yaw_goal-yaw)
            estimate_yaw_error = angle_error(estimate[2]-yaw_goal)
            if max(position_error, estimate_error) > .05 or max(truth_yaw_error, estimate_yaw_error) > .3:
                return reject('physical_waypoint_tolerance_failed', waypoint=goal['name'],
                    truth_position_error_m=position_error, estimated_position_error_m=estimate_error,
                    truth_heading_error_rad=truth_yaw_error, estimated_heading_error_rad=estimate_yaw_error)
            verified.append(goal['name']); previous_sim=sim
            results.append(dict(name=goal['name'], truth_position_error_m=position_error,
                estimated_position_error_m=estimate_error, truth_heading_error_rad=truth_yaw_error,
                estimated_heading_error_rad=estimate_yaw_error, arrival_sim=sim,
                registration_pair_age_s=nearest))
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            return reject('malformed_waypoint_evaluation_evidence', waypoint=goal['name'], detail=repr(error))
    return dict(status='passed', verified_waypoints=verified, transform_world_from_map_se2=transform,
                registration_samples=len(registration_pairs), registration_max_residual_m=max(residuals),
                waypoints=results, source='timestamp-matched public map->base_link TF and evaluation-only Gazebo truth')


def _render(path, grid, rows, goals, final):
    import numpy as np
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.colors import ListedColormap, BoundaryNorm
    from matplotlib.transforms import Affine2D

    fig = Figure(figsize=(10, 8)); FigureCanvasAgg(fig); ax = fig.subplots()
    if grid is not None:
        geom = map_geometry(grid); values = geom['data']
        colors = [(v, v, v, 1.) for v in np.linspace(1., .05, 101)] + [(.68, .72, .76, 1.)]
        cmap = ListedColormap(colors)
        raster = np.where(values == -1, 101, values)
        h, w = raster.shape; ox, oy, yaw = geom['origin']
        ax.imshow(raster, origin='lower', interpolation='nearest', cmap=cmap,
                  norm=BoundaryNorm(np.arange(-.5, 102.5), cmap.N),
                  extent=[0, w*geom['resolution'], 0, h*geom['resolution']],
                  transform=Affine2D().rotate(yaw).translate(ox, oy) + ax.transData,
                  zorder=0)
    else:
        ax.text(.5, .5, 'Occupancy map unavailable in this run', ha='center', va='center',
                transform=ax.transAxes)
    groups = {}
    for row in rows:
        groups.setdefault(row['segment'], []).append(row)
    for i, (name, points) in enumerate(groups.items()):
        points.sort(key=lambda row: row['timestamp_ns'])
        xy = np.asarray([[p['x'], p['y']] for p in points])
        ax.plot(xy[:, 0], xy[:, 1], '.-', color=f'C{i % 10}', linewidth=1.5, markersize=2,
                label=f'{name} actual path')
    start_xy = (rows[0]['x'], rows[0]['y']) if rows else (0., 0.)
    ax.scatter(*start_xy, marker='*', s=160, color='#239b56', edgecolor='black', label='Start', zorder=5)
    for name, position in goals.items():
        ax.scatter(*position, marker='o', s=70, facecolors='none', edgecolors='#bd1550', zorder=4)
        ax.annotate(name, position, xytext=(5, 5), textcoords='offset points', color='#9c003b', weight='bold')
    if final is not None:
        ax.scatter(final['x'], final['y'], marker='X', s=90, color='#ff8c00', edgecolor='black',
                   label='Final position', zorder=6)
        direction = None
        if len(rows) > 1:
            previous = rows[-2]
            direction = (final['x'] - previous['x'], final['y'] - previous['y'])
        if direction is None or math.hypot(*direction) < 1e-8:
            direction = (math.cos(final['yaw_rad']), math.sin(final['yaw_rad']))
        ax.quiver(final['x'], final['y'], *direction,
                  angles='xy', scale_units='xy', scale=2, color='#ff8c00', width=.006, zorder=6)
    ax.set(xlabel='map x (m)', ylabel='map y (m)', title='Warehouse occupancy map and actual trajectory')
    ax.set_aspect('equal'); ax.grid(alpha=.15); ax.legend(loc='best', fontsize=8)
    fig.tight_layout(); fig.savefig(path, dpi=150); fig.clear()


def write_track_artifacts(output_directory, route, report, evidence_index, evidence_root=None,
                          metrics_root=None):
    """Create requested artifacts even for incomplete/failed/interrupted captures."""
    output = Path(output_directory).resolve(); output.mkdir(parents=True, exist_ok=True)
    root = Path(evidence_root).resolve() if evidence_root else output
    if ('route_id' in evidence_index and evidence_index['route_id'] != route['route_id']) or (
            'run_id' in evidence_index and evidence_index['run_id'] != root.name) or (
            evidence_index.get('route_content_sha256') not in (None, route_content_sha256(route))):
        raise ValueError('Route evidence belongs to another run or canonical route')
    rows, grids, reached, registrations = [], [], [], []
    route_report = report.get('route', report)
    successful_goals = {segment.get('name') for segment in route_report.get('segments', [])
                        if segment.get('navigation_success') is True and
                        segment.get('evaluation_passed', True) is True}
    for i, segment in enumerate(evidence_index.get('segments', [])):
        path_doc = segment.get('path')
        if path_doc is None and segment.get('path_file'):
            path_doc = _load_ref(root, segment, 'path_file', 'path_sha256')
        if path_doc:
            poses = path_doc.get('poses', [])
            path_chunks(path_doc)
            rows.extend(_pose_rows(poses, segment.get('waypoint', segment.get('segment_id', f'segment-{i+1}'))))
        map_doc = segment.get('map')
        if map_doc is None and segment.get('map_source'):
            map_doc = _load_ref(root, segment['map_source'], 'file', 'sha256')
        if map_doc is not None:
            map_geometry(map_doc); grids.append(map_doc)
        pairs = segment.get('registration_pairs')
        if pairs is None and segment.get('registration_file'):
            pairs = _load_ref(root, segment, 'registration_file', 'registration_sha256')
        if pairs:
            registrations.extend(pairs)
        name = segment.get('waypoint')
        if name in successful_goals and name not in reached and (poses if path_doc else []):
            reached.append(name)
    if not grids and evidence_index.get('map_source'):
        grids.append(_load_ref(root, evidence_index['map_source'], 'file', 'sha256'))
    if not rows and evidence_index.get('start_pose'):
        rows.extend(_pose_rows([evidence_index['start_pose']], 'Start'))
    rows.sort(key=lambda row: row['timestamp_ns'])
    with (output / 'trajectory.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['timestamp_ns', 'x', 'y', 'yaw_rad', 'segment', 'source'])
        writer.writeheader(); writer.writerows(rows)
    goals = {goal['name']: goal['position'] for goal in route['goals']}
    final = rows[-1] if rows else None
    _render(output / 'trajectory.png', grids[-1] if grids else None, rows, goals, final)
    success = (route_report.get('success') is True and report.get('success', True) is True)
    reason = None if success else (report.get('error') or report.get('stop_reason') or
        (route_report.get('stop_reason') if route_report.get('success') is not True else
         'owner_safety_or_cleanup_failed') or 'incomplete')
    stamps = [row['timestamp_ns'] for row in rows]
    duration = route_report.get('total_route_duration_wall_s') or report.get('elapsed_wall_s')
    if duration is None and len(stamps) > 1:
        duration = (stamps[-1] - stamps[0]) * 1e-9
    physical = (evaluate_physical_waypoints(route, route_report,
        report.get('waypoint_observations', []), registrations, metrics_root=metrics_root,
        leg_admissions=report.get('route_leg_admissions'),
        settled_stops=report.get('settled_waypoint_stops'))
        if route_report.get('success') is True else
        dict(status='not_applicable', reason='route_did_not_complete'))
    navigation_success = route_report.get('success') is True and report.get('success', True) is True
    verified_success = navigation_success and physical.get('status') == 'passed'
    if navigation_success and not verified_success:
        reason = 'physical_waypoint_evaluation_incomplete'
    result = dict(route='Start -> G1 -> G2 -> G3 -> G4 -> G5', reached_goals=reached,
                  outcome='success' if verified_success else ('interrupted' if reason == 'interrupted' else 'failure'),
                  success=verified_success, navigation_success=navigation_success,
                  physical_evaluation=physical, duration_s=duration, failure_reason=reason, trajectory_samples=len(rows),
                  trajectory_source='synchronized public map->base_link TF; Gazebo truth not used',
                  occupancy_map_source='/map nav_msgs/msg/OccupancyGrid' if grids else None,
                  final_position=({k: final[k] for k in ('x', 'y', 'yaw_rad')} if final else None),
                  artifact_errors=evidence_index.get('errors', []))
    write(output / 'result.json', result)
    return result
