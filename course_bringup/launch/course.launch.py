from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode = value('mode')
    if mode not in ['sim', 'record', 'slam', 'navigation']:
        raise ValueError('mode must be sim, record, slam, or navigation')
    source = value('control_source')
    if source == 'auto':
        source = 'nav2' if mode == 'navigation' else 'none'
    if source not in ['none', 'nav2', 'manual', 'sru']:
        raise ValueError('control_source must be auto, none, nav2, manual, or sru')
    if mode == 'navigation' and source != 'nav2':
        raise ValueError('navigation mode owns Nav2; choose mode:=slam for manual/SRU')
    if source == 'nav2' and mode != 'navigation':
        raise ValueError('control_source:=nav2 requires mode:=navigation')
    actions = []
    def include(package, filename, args=None):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / 'launch' / filename)),
            launch_arguments=(args or {}).items()))
    include('course_sim', 'simulation.launch.py', {key: value(key) for key in
            ['gui', 'headless_rendering', 'world', 'scene', 'warehouse_assets',
             'spawn_x', 'spawn_y', 'spawn_z', 'fault_profile', 'seed', 'horizontal_samples',
             'locomotion_backend', 'pretrained_directory']})
    if mode in ['slam', 'navigation']:
        include('course_slam', 'slam.launch.py', {'params_file': value('slam_params_file'),
                'adapter_params_file': value('adapter_params_file'),
                'auto_ground_alignment': 'false' if value('map_mode') == 'fixed' else 'auto'})
        actions[-1] = TimerAction(period=20.0, actions=[actions[-1]])
    if mode == 'navigation':
        nav_args = {key: value(key) for key in
                    ['results_directory', 'timeout_s', 'goal_tolerance_m', 'map_mode', 'map_file', 'nav_params_file']}
        if value('scene') == 'warehouse':
            nav_args.update(map_width='40.0', map_height='40.0')
        include('course_navigation', 'navigation.launch.py', nav_args)
    if source in ['manual', 'sru']:
        include('course_bringup', 'control.launch.py', {'source': source,
                'localized': str(mode == 'slam').lower(), 'joy_dev': value('joy_dev')})
    if mode == 'record':
        record_world = value('world') or str(Path(value('warehouse_assets')) / 'warehouse.world'
            if value('scene') == 'warehouse' else Path(get_package_share_directory('course_sim')) / 'worlds/confined.world')
        actions.append(Node(package='course_sim', executable='record_course', arguments=[
            '--output-dir', value('bag_directory'), '--fault-profile', value('fault_profile'),
            '--seed', value('seed'), '--horizontal-samples', value('horizontal_samples'),
            '--scene', value('scene'), '--world', record_world,
            '--locomotion-backend', value('locomotion_backend'),
            '--pretrained-directory', value('pretrained_directory')], output='screen'))
    if value('rviz').lower() == 'true':
        config = yaml.safe_load((Path(get_package_share_directory('course_bringup')) /
                                 'rviz/course.rviz').read_text())
        if mode in ['sim', 'record']:
            config['Visualization Manager']['Global Options']['Fixed Frame'] = 'base_link'
            # Without SLAM no measured world datum exists; a body-fixed grid
            # would misleadingly cut through the robot rather than mark ground.
            config['Visualization Manager']['Displays'][0]['Enabled'] = False
        with tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', prefix='course-', delete=False) as stream:
            yaml.safe_dump(config, stream)
            rviz_config = stream.name
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', str(
            rviz_config)],
            parameters=[{'use_sim_time': True}], output='log'))
    return actions


def generate_launch_description():
    defaults = {'mode': 'sim', 'gui': 'false', 'headless_rendering': 'auto', 'rviz': 'false', 'world': '',
                'scene': 'confined', 'warehouse_assets': str(Path.cwd() / 'third_party/aws_warehouse'),
                'locomotion_backend': 'rl_sar', 'pretrained_directory': str(Path.cwd() / 'third_party/b2w_rl_sar'),
                'spawn_x': 'auto', 'spawn_y': 'auto', 'spawn_z': 'auto',
                'fault_profile': 'normal', 'seed': '0', 'horizontal_samples': '2000',
                'control_source': 'auto', 'joy_dev': '/dev/input/js0',
                'slam_params_file': '', 'adapter_params_file': '', 'nav_params_file': '',
                'map_mode': 'online', 'map_file': '',
                'results_directory': 'course_results/navigation', 'bag_directory': 'bags',
                'timeout_s': '120.0', 'goal_tolerance_m': '0.3'}
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
                              OpaqueFunction(function=setup)])
