from glob import glob
from setuptools import find_packages, setup

package_name = 'course_navigation'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='B2-W course team', maintainer_email='course@example.invalid',
    description='B2-W course simulation, localization and navigation integration',
    license='MIT', entry_points={'console_scripts': ['map_projector = course_navigation.map_node:main', 'velocity_adapter = course_navigation.velocity_adapter:main', 'goal_manager = course_navigation.goal_manager:main']},
)
