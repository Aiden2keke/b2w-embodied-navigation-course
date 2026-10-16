from glob import glob
from setuptools import find_packages, setup

package_name = 'course_sim'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.xml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='B2-W course team', maintainer_email='course@example.invalid',
    description='B2-W course simulation, localization and navigation integration',
    license='MIT', entry_points={'console_scripts': [
        'sensor_disturbance = course_sim.sensor_disturbance:main',
        'sensor_cloud_disturbance = course_sim.sensor_disturbance:cloud_main',
        'record_course = course_sim.record:main']},
)
