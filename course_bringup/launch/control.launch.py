"""Select a single existing manual/SRU source behind the course command gate."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    source = LaunchConfiguration('source').perform(context)
    if source not in ['manual', 'sru']:
        raise ValueError('control.launch.py source must be manual or sru')
    localized = LaunchConfiguration('localized').perform(context).lower() == 'true'
    state = '/slam/odom' if localized else '/sim/body_state'
    gate = Node(package='course_navigation', executable='velocity_adapter', parameters=[{
        'use_sim_time': True, 'input_topic': '/course_navigation/source_cmd_vel',
        'state_topic': state, 'require_localization_tf': localized}], output='screen')
    remaps = [('/path_manager/path_manager_ros/nav_vel', '/course_navigation/source_cmd_vel')]
    if source == 'manual':
        config = str(Path(get_package_share_directory('b2w_joystick_control'))/'config/ps5_config.yaml')
        return [gate, Node(package='joy', executable='joy_node', parameters=[{'use_sim_time': True,
                'dev': LaunchConfiguration('joy_dev'), 'autorepeat_rate': 20.0}], output='log'),
                Node(package='b2w_joystick_control', executable='joystick_teleop_node',
                     parameters=[config, {'use_sim_time': True}], remappings=remaps, output='screen')]
    if not localized:
        raise ValueError('SRU course mode requires SLAM localization; choose mode:=slam')
    return [gate, Node(package='joy', executable='game_controller_node',
            parameters=[{'use_sim_time': True, 'autorepeat_rate': 20.0}],
            remappings=[('/joy', '/rsl_joy')], output='log'),
            Node(package='rl_nav_controller', executable='rl_nav_controller', arguments=['--sim'],
            parameters=[{'use_sim_time': True}], remappings=remaps+[('/dlio/odom_node/odom','/slam/odom')], output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('source',default_value='manual'),
        DeclareLaunchArgument('localized',default_value='false'),
        DeclareLaunchArgument('joy_dev',default_value='/dev/input/js0'),OpaqueFunction(function=setup)])
