from pathlib import Path
import os
import json
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    config = LaunchConfiguration('nav_params_file').perform(context) or str(Path(get_package_share_directory('course_navigation')) / 'config/nav2.yaml')
    adaptive = LaunchConfiguration('adaptive_speed').perform(context).lower()
    if adaptive not in ('true', 'false'):
        raise ValueError('adaptive_speed must be true or false')
    adaptive = adaptive == 'true'
    observe = LaunchConfiguration('dwb_observability', default='false').perform(context).lower()
    if observe not in ('true', 'false'):
        raise ValueError('dwb_observability must be true or false')
    observe = observe == 'true'
    observation_parameters = []
    if observe:
        profile = yaml.safe_load(Path(config).read_text())
        if adaptive or profile['controller_server']['ros__parameters']['FollowPath']['plugin'] != 'dwb_core::DWBLocalPlanner':
            raise ValueError('dwb_observability requires DWB and adaptive_speed:=false')
        observation_parameters = [str(Path(get_package_share_directory('course_navigation')) /
                                      'config/nav2_dwb_observability.yaml')]
    governor_params = None
    if adaptive:
        from course_navigation.adaptive_profile import build_profile, SPEED_TOPIC
        from course_navigation.adaptive_speed import Config
        from dataclasses import fields
        # Explicit measurement required; no waypoint-specific speed logic or guessed width.
        settings_file = (LaunchConfiguration('adaptive_params_file').perform(context)
                         or str(Path(get_package_share_directory('course_navigation'))/'config/adaptive_speed.yaml'))
        settings = yaml.safe_load(Path(settings_file).read_text())['speed_governor']['ros__parameters']
        try:
            width = float(LaunchConfiguration('w_ref').perform(context))
        except ValueError as error:
            raise ValueError('adaptive_speed requires measured w_ref:=<obstacle-to-obstacle metres>') from error
        cfg = Config(**{**{f.name: settings[f.name] for f in fields(Config) if f.name in settings}, 'w_ref': width})
        if cfg.open_command > .44:
            raise ValueError('Initial opt-in ceiling must be <=0.44; measure before any further increase')
        default_config = Path(get_package_share_directory('course_navigation'))/'config/nav2.yaml'
        if Path(config).resolve() != default_config.resolve():
            raise ValueError('adaptive_speed requires the default nav2.yaml baseline; omit nav_params_file')
        baseline = yaml.safe_load(Path(config).read_text())
        profile = build_profile(baseline, cfg)
        # Parameters come from the existing costmap footprint, including Nav2 signed padding.
        costmap = baseline['local_costmap']['local_costmap']['ros__parameters']
        footprint = json.loads(costmap['footprint'])
        padding = costmap['footprint_padding']
        padded = [[v + (padding if v > 0 else -padding if v < 0 else 0) for v in p] for p in footprint]
        governor_params = {'use_sim_time': True, **settings, **vars(cfg),
                           'footprint': json.dumps(padded), 'speed_limit_topic': SPEED_TOPIC}
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='b2w-rpp-', delete=False) as stream:
            yaml.safe_dump(profile, stream)
            config = stream.name
    manager_enabled = LaunchConfiguration('goal_manager_enabled', default='true').perform(context).lower()
    if manager_enabled not in ('true', 'false'):
        raise ValueError('goal_manager_enabled must be true or false')
    guard_manifest = LaunchConfiguration('stage4_guard_manifest', default='').perform(context)
    if guard_manifest and (adaptive or manager_enabled != 'false' or not Path(guard_manifest).is_absolute()):
        raise ValueError('Stage4 requires absolute guard manifest, adaptive_speed:=false and goal_manager_enabled:=false')
    mode = LaunchConfiguration('map_mode').perform(context)
    map_file = LaunchConfiguration('map_file').perform(context)
    if mode not in ['online','fixed'] or (mode == 'fixed' and not Path(map_file).is_file()):
        raise ValueError('map_mode must be online or fixed; fixed requires an existing map_file')
    servers = [
        ('nav2_controller', 'controller_server'), ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'), ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'), ('nav2_waypoint_follower', 'waypoint_follower'),
        ('nav2_velocity_smoother', 'velocity_smoother')]
    nodes = []
    if governor_params is not None:
        nodes.append(Node(package='course_navigation', executable='speed_governor',
                          parameters=[governor_params], output='screen'))
    for package, executable in servers:
        remaps = [('cmd_vel', '/course_navigation/cmd_vel')]
        if executable == 'controller_server':
            remaps = [('cmd_vel', '/course_navigation/cmd_vel_raw')]
        elif executable == 'velocity_smoother':
            remaps = [('cmd_vel', '/course_navigation/cmd_vel_raw'),
                      ('cmd_vel_smoothed', '/course_navigation/cmd_vel')]
        elif executable == 'bt_navigator':
            remaps.append(('goal_pose', '/course_navigation/unused_nav2_goal_pose'))
        nodes.append(Node(package=package, executable=executable, name=executable,
                          parameters=[config] + (observation_parameters if executable == 'controller_server' else []),
                          remappings=remaps, output='log',
                          arguments=['--ros-args', '--log-level', 'rclcpp:=debug',
                                     '--log-level', 'planner_server:=debug']
                          if executable == 'planner_server' and
                          os.environ.get('B2W_NARROW_NAVFN_DEBUG') == '1' else []))
    if mode == 'online':
        width = float(LaunchConfiguration('map_width').perform(context))
        height = float(LaunchConfiguration('map_height').perform(context))
        nodes.append(Node(package='course_navigation', executable='map_projector', parameters=[{
            'use_sim_time': True, 'size': [width, height], 'origin': [-width / 2, -height / 2]}], output='screen'))
    else:
        nodes.append(Node(package='nav2_map_server', executable='map_server', name='map_server',
            parameters=[{'use_sim_time': True, 'yaml_filename': map_file}], output='screen'))
        servers.append(('nav2_map_server','map_server'))
    nodes.extend([
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', parameters=[{'use_sim_time': True,
             'autostart': True, 'node_names': [s[1] for s in servers]}], output='screen'),
        Node(package='course_navigation', executable='velocity_adapter', parameters=[{'use_sim_time': True,
             'stage4_guard_manifest': guard_manifest,
             **({'max_linear': cfg.open_command, 'speed_limit_topic': SPEED_TOPIC} if adaptive else {})}], output='screen'),
        Node(package='course_navigation', executable='goal_manager', parameters=[{'use_sim_time': True,
             'results_directory': LaunchConfiguration('results_directory'),
             'timeout_s': ParameterValue(LaunchConfiguration('timeout_s'), value_type=float),
             'goal_tolerance_m': ParameterValue(LaunchConfiguration('goal_tolerance_m'), value_type=float)}], output='screen'),
    ])
    if manager_enabled == 'false':
        nodes = [n for n in nodes if n._Node__node_executable != 'goal_manager']
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stage4_guard_manifest', default_value=''),
        DeclareLaunchArgument('goal_manager_enabled', default_value='true'),
        DeclareLaunchArgument('map_mode', default_value='online'),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('nav_params_file', default_value=''),
        DeclareLaunchArgument('dwb_observability', default_value='false'),
        DeclareLaunchArgument('adaptive_speed', default_value='false'),
        DeclareLaunchArgument('w_ref', default_value=''),
        DeclareLaunchArgument('adaptive_params_file', default_value=''),
        DeclareLaunchArgument('map_width', default_value='20.0'),
        DeclareLaunchArgument('map_height', default_value='16.0'),
        DeclareLaunchArgument('results_directory', default_value='course_results/navigation'),
        DeclareLaunchArgument('timeout_s', default_value='120.0'),
        DeclareLaunchArgument('goal_tolerance_m', default_value='0.3'), OpaqueFunction(function=setup)])
