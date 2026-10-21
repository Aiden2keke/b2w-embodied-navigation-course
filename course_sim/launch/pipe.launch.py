"""Standalone stationary culvert; inherits the existing Fortress robot/sensor stack."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from course_sim.pipe_environment import PipeConfig, make_pipe_world, clearance


def setup(context):
    value=lambda name: LaunchConfiguration(name).perform(context)
    share=Path(get_package_share_directory('course_sim'))
    config=PipeConfig(**yaml.safe_load(Path(value('config')).read_text()))
    world=Path(value('world')).expanduser().resolve()
    robot=Path(get_package_share_directory('b2w_description_ros2'))/'urdf/b2w_description_rsl.urdf'
    if not clearance(config,robot)['passed']:
        raise ValueError('pipe spawn intersects floor, wall or open endpoint bounds')
    if world.read_text()!=make_pipe_world(config):
        raise ValueError('world/config mismatch; regenerate using scripts/prepare_pipe.py')
    args={name:value(name) for name in ['gui','headless_rendering','locomotion_backend',
          'pretrained_directory','fault_profile','seed','horizontal_samples']}
    args.update(world=str(world),scene='confined',spawn_x=str(config.spawn_x_m),
                spawn_y=str(config.spawn_y_m),spawn_z=str(config.spawn_z_m))
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(share/'launch/simulation.launch.py')),launch_arguments=args.items())]


def generate_launch_description():
    share=Path(get_package_share_directory('course_sim'))
    defaults=dict(config=str(share/'config/pipe_v0.yaml'),world=str(share/'worlds/pipe_v0.world'),
        gui='false',headless_rendering='auto',locomotion_backend='rl_sar',
        pretrained_directory=str(Path.cwd()/'third_party/b2w_rl_sar'),fault_profile='normal',
        seed='0',horizontal_samples='2000')
    return LaunchDescription([*(DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()),
                              OpaqueFunction(function=setup)])
