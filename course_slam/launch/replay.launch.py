from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch = str(Path(get_package_share_directory('course_slam')) / 'launch/slam.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('bag'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch)),
        # Recorded /clock owns the time source; do not generate a second playback clock.
        ExecuteProcess(cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'), '--topics',
                            '/pointcloud', '/imu', '/tf_static', '/clock', '/sim/ground_truth/odom'], output='screen'),
    ])
