from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    config = LaunchConfiguration('nav_params_file').perform(context) or str(Path(get_package_share_directory('course_navigation')) / 'config/nav2.yaml')
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
                          parameters=[config], remappings=remaps, output='log',
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
        Node(package='course_navigation', executable='velocity_adapter', parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='course_navigation', executable='goal_manager', parameters=[{'use_sim_time': True,
             'results_directory': LaunchConfiguration('results_directory'),
             'timeout_s': ParameterValue(LaunchConfiguration('timeout_s'), value_type=float),
             'goal_tolerance_m': ParameterValue(LaunchConfiguration('goal_tolerance_m'), value_type=float)}], output='screen'),
    ])
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_mode', default_value='online'),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('nav_params_file', default_value=''),
        DeclareLaunchArgument('map_width', default_value='20.0'),
        DeclareLaunchArgument('map_height', default_value='16.0'),
        DeclareLaunchArgument('results_directory', default_value='course_results/navigation'),
        DeclareLaunchArgument('timeout_s', default_value='120.0'),
        DeclareLaunchArgument('goal_tolerance_m', default_value='0.3'), OpaqueFunction(function=setup)])
