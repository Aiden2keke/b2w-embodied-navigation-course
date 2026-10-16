import os
import yaml
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, ExecuteProcess,
                            SetEnvironmentVariable, RegisterEventHandler, EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from course_sim.description import generate_description
from course_sim.session import SessionLease

_session_lease = None  # Retain the lock until the launch process exits.


def setup(context):
    global _session_lease
    _session_lease = SessionLease(os.environ.get('ROS_DOMAIN_ID', '0'))
    share = Path(get_package_share_directory('course_sim'))
    model = Path(get_package_share_directory('b2w_description_ros2'))
    sensors = yaml.safe_load((share / 'config/sensors.yaml').read_text())
    backend = LaunchConfiguration('locomotion_backend').perform(context)
    if backend not in ['legacy', 'rl_sar']:
        raise ValueError('locomotion_backend must be legacy or rl_sar')
    pretrained = Path(LaunchConfiguration('pretrained_directory').perform(context)).expanduser().resolve()
    policy = pretrained / 'policy/b2w/robot_lab/policy.onnx'
    training_urdf = pretrained / 'robot_assets/b2w_description/urdf/b2w_description.urdf'
    if backend == 'rl_sar' and (not policy.is_file() or not training_urdf.is_file()):
        raise ValueError(f'Missing pretrained B2-W bundle at {pretrained}')
    samples = int(LaunchConfiguration('horizontal_samples').perform(context))
    sensors['lidar']['horizontal_samples'] = samples
    profile = LaunchConfiguration('fault_profile').perform(context)
    profiles = yaml.safe_load((share / 'config/fault_profiles.yaml').read_text())
    if profile not in profiles:
        raise ValueError(f'Unknown fault_profile: {profile}')
    fault = profiles[profile]
    description = generate_description((model / 'urdf/b2w_description_rsl.urdf').read_text(),
                                       sensors, fault['slip_ratio'], model / 'meshes',
                                       'torque' if backend == 'rl_sar' else 'legacy',
                                       training_urdf.read_text() if backend == 'rl_sar' else None)
    params = str(Path(get_package_share_directory('b2w_controllers')) / 'config/b2w_controllers.yaml')
    scene = LaunchConfiguration('scene').perform(context)
    if scene not in ['confined', 'warehouse']:
        raise ValueError('scene must be confined or warehouse')
    warehouse = Path(LaunchConfiguration('warehouse_assets').perform(context)).expanduser().resolve()
    world = LaunchConfiguration('world').perform(context) or str(
        warehouse / 'warehouse.world' if scene == 'warehouse' else share / 'worlds/confined.world')
    if not Path(world).is_file():
        raise ValueError(f'World not found: {world}; run scripts/prepare_warehouse.py for warehouse assets')
    world_name = ET.parse(world).getroot().find('world').get('name')
    spawn = []
    for axis, default in [('x', 0. if scene == 'warehouse' else -6.), ('y', 0.), ('z', 1.)]:
        value = LaunchConfiguration('spawn_' + axis).perform(context)
        spawn.extend(['-' + axis, str(default) if value == 'auto' else value])
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    command = ['ign', 'gazebo', '-r', '-v', '3']
    if not gui:
        command.append('-s')
    headless = LaunchConfiguration('headless_rendering').perform(context).lower()
    if headless == 'true' or (headless == 'auto' and not os.environ.get('DISPLAY')):
        command.append('--headless-rendering')
    command.append(world)
    seed = int(LaunchConfiguration('seed').perform(context))
    simulator = ExecuteProcess(cmd=command, output='log')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share / 'config' / (
                 'bridge_rl_sar.yaml' if backend == 'rl_sar' else 'bridge.yaml')), 'use_sim_time': True}], output='log')
    controller = Node(package='b2w_controllers', executable=(
            'b2w_rl_sar_controller' if backend == 'rl_sar' else 'b2w_controllers'),
             parameters=([str(share / 'config/rl_sar.yaml'), {'policy_path': str(policy), 'use_sim_time': True}]
                 if backend == 'rl_sar' else [params, {'topics.odometry': '/sim/body_state', 'use_sim_time': True}]), output='log')
    stop_on_exit = [RegisterEventHandler(OnProcessExit(target_action=process,
                    on_exit=[EmitEvent(event=Shutdown(reason=f'{label} exited; stopping the course session'))]))
                    for process, label in [(simulator, 'Gazebo'), (bridge, 'sensor bridge'), (controller, 'locomotion controller')]]
    return [
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', str(share / 'config' / (
            'fastdds_local.xml' if os.environ.get('ROS_LOCALHOST_ONLY') == '1' else 'fastdds.xml'))),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', str(model.parent) + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')),
        *stop_on_exit,
        simulator,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='b2w_robot_state_publisher', remappings=[('robot_description', '/b2w/robot_description')],
             parameters=[{'robot_description': description, 'use_sim_time': True}], output='log'),
        Node(package='ros_gz_sim', executable='create', arguments=[
            '-world', world_name, '-name', 'b2w', '-topic', '/b2w/robot_description', *spawn], output='screen'),
        bridge,
        controller,
        Node(package='course_sim', executable='sensor_disturbance', parameters=[{
            k: v for k, v in fault.items() if k != 'slip_ratio'}, {'seed': seed, 'use_sim_time': True}], output='screen'),
        Node(package='course_sim', executable='sensor_cloud_disturbance', parameters=[{
            k: v for k, v in fault.items() if k != 'slip_ratio'}, {'seed': seed, 'use_sim_time': True}], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('headless_rendering', default_value='auto'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('scene', default_value='confined'),
        DeclareLaunchArgument('locomotion_backend', default_value='rl_sar'),
        DeclareLaunchArgument('pretrained_directory', default_value=str(Path.cwd() / 'third_party/b2w_rl_sar')),
        DeclareLaunchArgument('warehouse_assets', default_value=str(Path.cwd() / 'third_party/aws_warehouse')),
        *(DeclareLaunchArgument('spawn_' + axis, default_value='auto') for axis in 'xyz'),
        DeclareLaunchArgument('fault_profile', default_value='normal'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('horizontal_samples', default_value='2000',
                              description='Reduced density must be explicitly recorded'),
        OpaqueFunction(function=setup),
    ])
