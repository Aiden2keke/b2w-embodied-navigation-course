"""One online dual-planner pair; raw costs are descriptive, never planner objectives."""
import copy
import hashlib
import json
import math
import time

from narrow_fixture import chosen_route

PLUGIN_TYPES = {'NavFn': 'nav2_navfn_planner/NavfnPlanner',
                'Smac2D': 'nav2_smac_planner/SmacPlanner2D'}


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def path_metrics(points, cm):
    """Midpoints spaced <= resolution/2, floor cells; exposure = sum(raw_cost * ds)."""
    meta = cm['metadata']; origin = meta['origin']['position']; resolution = meta['resolution']
    samples = []; length = 0.
    for a, b in zip(points, points[1:]):
        distance = math.dist(a, b); length += distance
        if distance == 0: continue
        count = max(1, math.ceil(distance / (resolution / 2)))
        for i in range(count):
            t = (i + .5) / count
            x, y = [aa + (bb - aa) * t for aa, bb in zip(a, b)]
            ix = math.floor((x - origin['x']) / resolution)
            iy = math.floor((y - origin['y']) / resolution)
            cost = int(cm['data'][iy * meta['size_x'] + ix]) if (
                0 <= ix < meta['size_x'] and 0 <= iy < meta['size_y']) else None
            samples.append(dict(xy=[x, y], cell=[ix, iy], raw_cost=cost, ds_m=distance/count))
    costs = [s['raw_cost'] for s in samples if s['raw_cost'] is not None]
    ordered = sorted(costs)
    outside = sum(s['raw_cost'] is None for s in samples)
    return dict(geometric_path_length_m=length if len(points) > 1 else None,
        raw_cost_stats=dict(min=min(costs), mean=sum(costs)/len(costs),
            p95=ordered[math.ceil(.95*len(costs))-1], max=max(costs)) if costs else None,
        integrated_raw_cost_exposure_cost_m=sum(s['raw_cost']*s['ds_m'] for s in samples)
            if samples and not outside else None,
        unknown_samples=costs.count(255), cost_253_samples=costs.count(253),
        cost_254_samples=costs.count(254), out_of_bounds_samples=outside,
        sampling_definition='Segment midpoints <= resolution/2; Costmap2D floor; p95 nearest rank',
        exposure_definition='Sum(raw master cost * segment length), cost*m; includes 255/253/254; descriptive only, not either planner internal objective',
        samples=samples)


def compare_planners(case, start, goal, snapshot, plan, origin_xy_yaw, width):
    """Callbacks use one live server/session. Only planner_id differs between requests."""
    from nav2_msgs.action import ComputePathToPose
    from rosidl_runtime_py.convert import message_to_ordereddict
    result = dict(comparison_eligible=False, planners={}, costmap_hashes={},
        costmap_snapshot_source='/global_costmap/get_costmap live raw master',
        snapshot_limit='Three service snapshots bracket calls; not an atomic plugin-memory trace',
        metrics_costmap_reference='costmap-before-planner-1.json',
        request_order=['NavFn', 'Smac2D'])
    maps = []
    def capture(label):
        cm = snapshot(); maps.append(cm)
        raw = bytes(cm['data']); digest = hashlib.sha256(raw).hexdigest()
        save(case/f'costmap-{label}.json', cm)
        (case/f'costmap-{label}.raw.u8').write_bytes(raw)
        result['costmap_hashes'][label] = digest
        save(case/'comparison.json', result)
    capture('before-planner-1')
    template = ComputePathToPose.Goal()
    template.start = copy.deepcopy(start); template.goal = copy.deepcopy(goal); template.use_start = True
    ox, oy, yaw = origin_xy_yaw
    for planner_id, next_snapshot in [('NavFn', 'between-planners'), ('Smac2D', 'after-planner-2')]:
        request = copy.deepcopy(template); request.planner_id = planner_id
        exact = message_to_ordereddict(request)
        save(case/f'{planner_id}-request.json', exact)
        record = dict(actual_plugin_id=planner_id, configured_plugin_type=PLUGIN_TYPES[planner_id],
            explicit_start=exact['start'], explicit_goal=exact['goal'], use_start=True,
            compute_path_status=None, chosen_route='unclassified', returned_path=None)
        wall = time.monotonic()
        try:
            response = plan(request)
            record['wall_action_roundtrip_ms'] = (time.monotonic()-wall)*1000
            record['compute_path_status'] = response.status
            record['compute_path_status_name'] = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(response.status, 'UNKNOWN')
            returned = message_to_ordereddict(response.result)
            save(case/f'{planner_id}-compute-path-result.json', returned)
            record['returned_path'] = returned['path']
        except RuntimeError as error:
            record['wall_action_roundtrip_ms'] = (time.monotonic()-wall)*1000
            record['request_error'] = str(error)
        result['planners'][planner_id] = record
        capture(next_snapshot)
    for record in result['planners'].values():
        path = record['returned_path']
        points = [[p['pose']['position']['x'], p['pose']['position']['y']] for p in path['poses']] if path else []
        record.update(path_metrics(points, maps[0]))
        local = [[math.cos(yaw)*(x-ox)+math.sin(yaw)*(y-oy),
                  -math.sin(yaw)*(x-ox)+math.cos(yaw)*(y-oy)] for x, y in points]
        record['chosen_route'] = chosen_route(local, width) if record['compute_path_status'] == 4 else 'unclassified'
    result['raw_master_hashes_equal'] = len(set(result['costmap_hashes'].values())) == 1
    # Timestamp/layer name are not grid geometry. Equal bytes with moved origin is not equal input.
    grids = [{k: cm['metadata'][k] for k in ('size_x', 'size_y', 'resolution', 'origin')} for cm in maps]
    result['costmap_grid_geometry_equal'] = all(g == grids[0] for g in grids)
    result['comparison_eligible'] = (result['raw_master_hashes_equal'] and result['costmap_grid_geometry_equal']
        and all('request_error' not in r for r in result['planners'].values()))
    result['confounds'] = []
    if not result['raw_master_hashes_equal']: result['confounds'].append('raw master costmap changed during planner pair')
    if not result['costmap_grid_geometry_equal']: result['confounds'].append('costmap grid geometry changed')
    if any('request_error' in r for r in result['planners'].values()): result['confounds'].append('incomplete action result')
    save(case/'comparison.json', result)
    return result
