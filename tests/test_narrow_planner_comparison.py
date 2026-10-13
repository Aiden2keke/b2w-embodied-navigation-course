"""Catch changed explicit inputs, omitted middle hash and incorrect cost integration."""
import copy
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))


def costmap(data=(0, 10, 20, 30)):
    return dict(metadata=dict(size_x=4, size_y=1, resolution=1.,
        origin=dict(position=dict(x=0., y=0., z=0.),
                    orientation=dict(x=0., y=0., z=0., w=1.))), data=list(data))


def test_exposure_integrates_distance_and_counts_reserved_costs():
    from narrow_planner_comparison import path_metrics
    result = path_metrics([[.5, .5], [3.5, .5]], costmap())
    assert result['geometric_path_length_m'] == 3.
    assert result['integrated_raw_cost_exposure_cost_m'] == 45.
    assert result['raw_cost_stats'] == dict(min=0, mean=15., p95=30., max=30)
    reserved = path_metrics([[.5, .5], [3.5, .5]], costmap((255, 253, 254, 0)))
    assert reserved['unknown_samples'] == 1
    assert reserved['cost_253_samples'] == 2
    assert reserved['cost_254_samples'] == 2
    outside = path_metrics([[-1., .5], [1., .5]], costmap())
    assert outside['out_of_bounds_samples'] > 0
    assert outside['integrated_raw_cost_exposure_cost_m'] is None
    assert path_metrics([], costmap())['geometric_path_length_m'] is None


@pytest.mark.parametrize('change_index', [None, 1, 2])
def test_comparison_uses_identical_explicit_inputs_and_all_three_hashes(tmp_path, change_index):
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose
    from narrow_planner_comparison import compare_planners
    start = PoseStamped(); start.header.frame_id = 'map'; start.header.stamp.sec = 23
    start.pose.position.x = .5; start.pose.position.y = .5; start.pose.orientation.w = 1.
    goal = copy.deepcopy(start); goal.pose.position.x = 3.5
    events = []; requests = []
    def snapshot():
        index = events.count('snapshot'); events.append('snapshot')
        return costmap((1, 10, 20, 30)) if index == change_index else costmap()
    def plan(request):
        events.append(request.planner_id); requests.append(copy.deepcopy(request))
        response = ComputePathToPose.Result()
        response.path.header.frame_id = 'map'; response.path.poses = [start, goal]
        return SimpleNamespace(status=4, result=response)
    result = compare_planners(tmp_path, start, goal, snapshot, plan, [.5, .5, 0.], 1.7)
    assert events == ['snapshot', 'NavFn', 'snapshot', 'Smac2D', 'snapshot']
    assert all(r.use_start for r in requests)
    assert requests[0].start == requests[1].start == start
    assert requests[0].goal == requests[1].goal == goal
    assert result['comparison_eligible'] is (change_index is None)
    assert result['planners']['NavFn']['chosen_route'] == 'narrow'
    assert len(list(tmp_path.glob('costmap-*.raw.u8'))) == 3
    assert json.loads((tmp_path/'NavFn-request.json').read_text())['use_start'] is True


def test_grid_geometry_change_is_a_confound_even_with_equal_raw_bytes(tmp_path):
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose
    from narrow_planner_comparison import compare_planners
    count = 0
    def snapshot():
        nonlocal count
        cm = costmap(); count += 1
        if count == 3: cm['metadata']['origin']['position']['x'] = 1.
        return cm
    response = SimpleNamespace(status=6, result=ComputePathToPose.Result())
    result = compare_planners(tmp_path, PoseStamped(), PoseStamped(), snapshot,
                              lambda request: response, [0., 0., 0.], 1.7)
    assert result['raw_master_hashes_equal']
    assert not result['comparison_eligible']


def test_comparison_cli_forces_single_a_case_and_dual_config(tmp_path):
    run = subprocess.run([sys.executable, str(ROOT/'scripts/run_narrow_baseline.py'),
        '--same-server-planner-comparison', '--prepare-only', '--scenarios', 'B',
        '--widths', '.8', '1.0', '--seeds', '1', '2', '--goal-distance', '1',
        '--output-dir', str(tmp_path)], capture_output=True, text=True)
    assert run.returncode == 0, run.stderr
    design = json.loads(next(tmp_path.glob('*/A*/fixture.json')).read_text())
    assert (design['scenario_id'], design['corridor_width_m'], design['seed']) == ('A', 1.7, 0)
    assert design['goal_relative_start_xy'] == [6., 0.]
    assert design['measurement_scope'] == 'same_server_planner_only_comparison'
    assert len(list(tmp_path.glob('*/**/fixture.json'))) == 1
    manifest = json.loads(next(tmp_path.glob('*/manifest.json')).read_text())
    assert manifest['nav_params_file'] == str(ROOT/'course_navigation/config/nav2_dual_planner_diag.yaml')
    assert not manifest['gui'] and not manifest['rviz']


def test_dual_config_retains_baseline_and_navfn_parameter_contract():
    import yaml
    baseline = yaml.safe_load((ROOT/'course_navigation/config/nav2.yaml').read_text())
    dual = yaml.safe_load((ROOT/'course_navigation/config/nav2_dual_planner_diag.yaml').read_text())
    original_planner = baseline.pop('planner_server')['ros__parameters']
    dual_planner = dual.pop('planner_server')['ros__parameters']
    assert baseline == dual
    assert dual_planner['NavFn'] == original_planner['GridBased']
    assert dual_planner['Smac2D']['allow_unknown'] == original_planner['GridBased']['allow_unknown']
    assert 'cost_travel_multiplier' not in dual_planner['Smac2D']


def test_incomplete_action_preserves_three_snapshots_and_rejects_comparison(tmp_path):
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose
    from narrow_planner_comparison import compare_planners
    def plan(request):
        if request.planner_id == 'NavFn': raise RuntimeError('timeout')
        return SimpleNamespace(status=6, result=ComputePathToPose.Result())
    result = compare_planners(tmp_path, PoseStamped(), PoseStamped(), costmap,
                              plan, [0., 0., 0.], 1.7)
    assert result['raw_master_hashes_equal']
    assert not result['comparison_eligible']
    assert result['planners']['NavFn']['request_error'] == 'timeout'
    assert result['planners']['Smac2D']['compute_path_status'] == 6
    assert len(list(tmp_path.glob('costmap-*.raw.u8'))) == 3
