"""Launch expansion and command watchdog offline; no nodes or Gazebo run."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch_ros.actions import Node

from course_navigation.adaptive_speed import CommandLimit

ROOT = Path(__file__).resolve().parents[1]


def expand(**overrides):
    path = ROOT/'course_navigation/launch/navigation.launch.py'
    spec = importlib.util.spec_from_file_location('adaptive_nav_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    defaults = dict(nav_params_file=str(ROOT/'course_navigation/config/nav2.yaml'),
                    map_mode='online', map_file='', map_width='40.', map_height='40.',
                    adaptive_speed='false', w_ref='',
                    adaptive_params_file=str(ROOT/'course_navigation/config/adaptive_speed.yaml'),
                    results_directory='/tmp/adaptive-launch-tests',
                    timeout_s='120.', goal_tolerance_m='.3')
    context.launch_configurations.update({**defaults, **overrides})
    return module.setup(context)


def params(node):
    merged = {}
    for source in node._Node__parameters:
        if isinstance(source, dict):
            for key, value in source.items():
                merged[''.join(v.text for v in key)] = value
    return merged


def test_required_width_and_custom_dwb_config_rejected_when_adaptive():
    for kwargs in [dict(w_ref=''), dict(w_ref='0'), dict(w_ref='nan'),
                   dict(w_ref='1.5', nav_params_file=str(ROOT/'course_navigation/config/nav2_smac2d_diag.yaml'))]:
        with pytest.raises(ValueError):
            expand(adaptive_speed='true', **kwargs)


def test_profile_preserves_costmaps_and_navfn_dwb_rollback(tmp_path):
    from course_navigation.adaptive_profile import build_profile
    baseline = yaml.safe_load((ROOT/'course_navigation/config/nav2.yaml').read_text())
    original = deepcopy(baseline)
    adaptive = build_profile(baseline)
    assert original == baseline
    for key in ('local_costmap', 'global_costmap', 'behavior_server'):
        assert adaptive[key] == baseline[key]
    assert adaptive['planner_server']['ros__parameters']['GridBased']['plugin'] == 'nav2_smac_planner/SmacPlanner2D'
    assert baseline['planner_server']['ros__parameters']['GridBased']['plugin'] == 'nav2_navfn_planner/NavfnPlanner'
    assert baseline['controller_server']['ros__parameters']['FollowPath']['plugin'] == 'dwb_core::DWBLocalPlanner'
    follow = adaptive['controller_server']['ros__parameters']['FollowPath']
    assert follow['plugin'] == 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    assert follow['use_collision_detection'] and follow['use_regulated_linear_velocity_scaling']
    assert follow['rotate_to_heading_angular_vel'] == .6
    assert follow['desired_linear_vel'] == adaptive['velocity_smoother']['ros__parameters']['max_velocity'][0] == .43


def expanded_parameters(adaptive):
    from launch_ros.utilities import evaluate_parameters
    context = LaunchContext()
    effective = {}
    for n in expand(adaptive_speed=str(adaptive).lower(), w_ref='1.5'):
        if not isinstance(n, Node):
            continue
        executable = n._Node__node_executable
        if executable not in ('planner_server', 'controller_server', 'speed_governor', 'velocity_smoother', 'velocity_adapter'):
            continue
        for source in evaluate_parameters(context, n._Node__parameters):
            if isinstance(source, Path):
                effective[executable] = yaml.safe_load(source.read_text())[executable]['ros__parameters']
            elif isinstance(source, dict):
                effective.setdefault(executable, {}).update(source)
    return effective


def test_actual_launch_expanded_planner_and_entire_speed_chain():
    for adaptive in (False, True):
        effective = expanded_parameters(adaptive)
        planner = effective['planner_server']['GridBased']
        controller = effective['controller_server']['FollowPath']
        if adaptive:
            assert planner['plugin'] == 'nav2_smac_planner/SmacPlanner2D'
            assert 'use_astar' not in planner
            assert controller['plugin'] == 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
            assert controller['desired_linear_vel'] == effective['velocity_smoother']['max_velocity'][0] == effective['velocity_adapter']['max_linear'] == .43
            governor = effective['speed_governor']
            assert governor['narrow_command'] == .11 and governor['open_command'] == .43
            assert governor['open_physical'] == .5 and governor['narrow_physical'] == .18
            assert governor['speed_limit_topic'] == effective['controller_server']['speed_limit_topic'] == effective['velocity_adapter']['speed_limit_topic']
        else:
            assert planner['plugin'] == 'nav2_navfn_planner/NavfnPlanner'
            assert controller['plugin'] == 'dwb_core::DWBLocalPlanner'
            assert effective['velocity_smoother']['max_velocity'][0] == .4
            assert 'speed_governor' not in effective


def test_launch_settings_release_safe_start_without_changing_goal_or_collision_contract():
    from dataclasses import fields
    import numpy as np
    from course_navigation.adaptive_speed import Config, CostGrid, Governor
    effective = expanded_parameters(True)
    settings = effective['speed_governor']
    g = Governor(Config(**{f.name: settings[f.name] for f in fields(Config)}))
    # Padded footprint has .245m on the near side, matching the Oct 18 start
    # within cell resolution. The former .35m release rule cannot clear this.
    cells = np.zeros((200, 200), dtype=np.uint8)
    centers = -5+(np.arange(200)+.5)*.05
    cells[(centers >= .685) | (centers <= -2.339), :] = 254
    grid = CostGrid(cells, .05, (-5., -5., 0.))
    import json
    fp = json.loads(settings['footprint'])
    g.update(0., grid, [(0., 0.), (2., 0.)], (0., 0., 0.), fp, .18)
    assert g.update(.51, grid, [(0., 0.), (2., 0.)], (0., 0., 0.), fp, .18).mode == 'OPEN'
    baseline = yaml.safe_load((ROOT/'course_navigation/config/nav2.yaml').read_text())
    original = baseline['controller_server']['ros__parameters']
    for key in original.keys()-{'FollowPath'}:
        assert effective['controller_server'][key] == original[key]
    follow = effective['controller_server']['FollowPath']
    assert follow['use_collision_detection'] and follow['use_rotate_to_heading']
    assert follow['use_regulated_linear_velocity_scaling'] and follow['use_cost_regulated_linear_velocity_scaling']


def test_expanded_launch_enables_governor_and_adapter_watchdog_only_when_opted_in():
    default = expand()
    adaptive = expand(adaptive_speed='true', w_ref='1.5')
    default_nodes = [n for n in default if isinstance(n, Node)]
    adaptive_nodes = [n for n in adaptive if isinstance(n, Node)]
    assert len(adaptive_nodes) == len(default_nodes)+1
    def executable(node):
        return node._Node__node_executable
    assert not any(executable(n) == 'speed_governor' for n in default_nodes)
    assert sum(executable(n) == 'speed_governor' for n in adaptive_nodes) == 1
    adapter = next(n for n in adaptive_nodes if executable(n) == 'velocity_adapter')
    assert 'speed_limit_topic' in params(adapter)
    assert params(adapter)['max_linear'] == .43
    rollback = next(n for n in default_nodes if executable(n) == 'velocity_adapter')
    assert 'speed_limit_topic' not in params(rollback)


def test_governor_parameters_evaluate_to_ros_scalars_not_nested_array():
    from launch_ros.utilities import evaluate_parameters
    nodes = expand(adaptive_speed='true', w_ref='1.5')
    governor = next(n for n in nodes if isinstance(n, Node) and n._Node__node_executable == 'speed_governor')
    evaluated = evaluate_parameters(LaunchContext(), governor._Node__parameters)
    import json
    assert isinstance(evaluated[0]['footprint'], str)
    assert json.loads(evaluated[0]['footprint']) == [[.75, .43], [.75, -.43], [-.75, -.43], [-.75, .43]]


def test_limit_watchdog_is_fail_closed_and_zero_is_invalid():
    gate = CommandLimit(.43, .5)
    assert gate.apply((.4, 0., .6), 10., 20.) == (0., 0., 0.)
    gate.receive(.11, False, 10., 20.)
    assert gate.apply((.4, 0., .6), 10.1, 20.1) == pytest.approx((.11, 0., .165))
    assert gate.apply((-.4, 0., -.6), 10.1, 20.1) == pytest.approx((-.11, 0., -.165))
    assert gate.apply((.4, 0., .6), 10.6, 20.6) == (0., 0., 0.)
    for limit, percentage in [(0., False), (.1, True), (float('nan'), False), (-1., False)]:
        gate.receive(limit, percentage, 11., 21.)
        assert gate.apply((.4, 0., .6), 11., 21.) == (0., 0., 0.)


@pytest.mark.parametrize('command,expected', [
    ((.4, 0., .6), (.11, 0., .165)),
    ((.24, .32, .6), (.066, .088, .165)),
    ((-.4, 0., -.6), (-.11, 0., -.165)),
    ((0., -.4, .6), (0., -.11, .165)),
    ((.08, .06, .3), (.08, .06, .3)),
    ((0., 0., .6), (0., 0., .6)),
    ((1e-8, -1e-8, -.6), (1e-8, -1e-8, -.6)),
])
def test_moving_limit_preserves_twist_ratios_but_in_place_yaw_is_intentional(command, expected):
    gate = CommandLimit(.43, .5)
    gate.receive(.11, False, 10., 20.)
    assert gate.apply(command, 10.1, 20.1) == pytest.approx(expected)


def test_open_to_narrow_clip_retains_curvature_and_existing_angular_bound():
    from course_navigation.velocity_adapter import VelocityGate
    safety = VelocityGate(max_linear=.43, max_angular=.6)
    safety.set_command((.4, 0., 5.), 20.)
    safety.state_time = 20.
    command = safety.output(20.1)
    gate = CommandLimit(.43, .5)
    gate.receive(.43, False, 10., 20.)
    assert gate.apply(command, 10., 20.) == (.4, 0., .6)
    gate.receive(.11, False, 10.1, 20.1)
    clipped = gate.apply(command, 10.1, 20.1)
    assert clipped == pytest.approx((.11, 0., .165))
    assert clipped[2]/clipped[0] == pytest.approx(1.5)
    assert abs(clipped[2]) <= .6
    # Stale heartbeat still stops a pure rotation, not just translation.
    assert gate.apply((0., 0., .6), 11., 21.) == (0., 0., 0.)


def test_numerical_translation_residue_does_not_suppress_in_place_yaw():
    gate = CommandLimit(.43, .5)
    gate.receive(1e-8, False, 10., 20.)
    assert gate.apply((1e-7, 0., .6), 10., 20.) == pytest.approx((1e-8, 0., .6))


def test_adapter_publication_preserves_rollback_and_applies_curvature_limit(monkeypatch):
    from types import SimpleNamespace as NS
    from course_navigation.velocity_adapter import VelocityAdapter, VelocityGate
    import course_navigation.velocity_adapter as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 20.)
    adapter = VelocityAdapter.__new__(VelocityAdapter)
    adapter.m2_enabled = adapter.require_tf = False
    adapter.gate = VelocityGate(max_linear=.43, max_angular=.6)
    adapter.gate.set_command((.4, 0., 5.), 20.)
    adapter.gate.state_time = 20.
    published = []
    adapter.pub = NS(publish=lambda msg: published.append((msg.linear.x, msg.linear.y, msg.angular.z)))
    adapter.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
    adapter.speed_limit = None  # Opt-out uses the original safety gate only.
    adapter.publish()
    assert published[-1] == (.4, 0., .6)
    adapter.speed_limit = CommandLimit(.43, .5)
    adapter.speed_limit.receive(.11, False, 10., 20.)
    adapter.publish()
    assert published[-1] == pytest.approx((.11, 0., .165))
    adapter.gate.set_command((0., 0., 5.), 20.)
    adapter.publish()
    assert published[-1] == (0., 0., .6)


def test_installed_interfaces_and_rpp_parameters():
    from nav2_msgs.msg import SpeedLimit, Costmap
    assert SpeedLimit.get_fields_and_field_types()['speed_limit'] == 'double'
    assert 'metadata' in Costmap.get_fields_and_field_types()
    from course_navigation.adaptive_profile import RPP
    import xml.etree.ElementTree as ET
    xml = Path('/opt/ros/humble/share/nav2_smac_planner/smac_plugin_2d.xml')
    assert any(c.attrib.get('name') == 'nav2_smac_planner/SmacPlanner2D'
               for c in ET.parse(xml).findall('.//class'))
    binary = Path('/opt/ros/humble/lib/libnav2_regulated_pure_pursuit_controller.so').read_bytes()
    for name in RPP:
        if name != 'plugin':
            assert ('.'+name).encode()+b'\0' in binary, name


def test_bringup_forwards_opt_in_and_measured_width_without_launching():
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.utilities import perform_substitutions
    path = ROOT/'course_bringup/launch/course.launch.py'
    spec = importlib.util.spec_from_file_location('adaptive_bringup_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    context.launch_configurations.update(mode='navigation', adaptive_speed='true', w_ref='1.5')
    includes = [a for a in module.setup(context) if isinstance(a, IncludeLaunchDescription)]
    nav_args = next(dict(a.launch_arguments) for a in includes if 'w_ref' in dict(a.launch_arguments))
    assert nav_args['adaptive_speed'] == 'true' and nav_args['w_ref'] == '1.5'


def test_byte_identical_baseline_copy_is_supported_without_relaxing_profile(tmp_path):
    p=tmp_path/'baseline-copy.yaml';p.write_bytes((ROOT/'course_navigation/config/nav2.yaml').read_bytes())
    assert expand(adaptive_speed='true',w_ref='1.5',nav_params_file=str(p))
    changed=yaml.safe_load(p.read_text());changed['local_costmap']['local_costmap']['ros__parameters']['footprint_padding']=0.
    p.write_text(yaml.safe_dump(changed))
    with pytest.raises(ValueError):expand(adaptive_speed='true',w_ref='1.5',nav_params_file=str(p))
