from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    get_package_share_directory('lio_sam')  # Fail clearly when the real backend is missing.
    share = Path(get_package_share_directory('course_slam'))
    params = LaunchConfiguration('params_file').perform(context) or str(share / 'config/lio_sam.yaml')
    adapter_params = LaunchConfiguration('adapter_params_file').perform(context) or str(share / 'config/adapter.yaml')
    nodes = []
    for name in ['imuPreintegration', 'imageProjection', 'featureExtraction', 'mapOptimization']:
        nodes.append(Node(package='lio_sam', executable='lio_sam_' + name,
                          parameters=[params], remappings=[('/tf', '/lio_internal/tf')], output='log'))
    overrides = {'use_sim_time': True}
    ground = LaunchConfiguration('auto_ground_alignment').perform(context).lower()
    if ground not in ['auto', 'true', 'false']:
        raise ValueError('auto_ground_alignment must be auto, true or false')
    if ground != 'auto':
        overrides['auto_ground_alignment'] = ground == 'true'
    nodes.append(Node(package='course_slam', executable='pose_adapter',
                      parameters=[adapter_params, overrides], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('adapter_params_file', default_value=''),
        DeclareLaunchArgument('auto_ground_alignment', default_value='auto'), OpaqueFunction(function=setup)])
