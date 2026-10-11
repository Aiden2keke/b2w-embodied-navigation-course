"""Exercise the native URI loader used by RViz against generated B2-W meshes."""
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET
import pytest
import yaml
from course_sim.description import generate_description


def test_torque_model_has_one_force_plugin_per_joint_and_maps_training_trunk():
    root = Path(__file__).resolve().parents[1]
    model = root / 'b2w_sim/b2w_description_ros2'
    sensors = yaml.safe_load((root / 'course_sim/config/sensors.yaml').read_text())
    source = '<robot><link name="trunk"><inertial><mass value="36.5"/></inertial></link></robot>'
    description = ET.fromstring(generate_description(
        (model / 'urdf/b2w_description_rsl.urdf').read_text(), sensors,
        mesh_directory=model / 'meshes', joint_control='torque', dynamics_urdf=source))
    plugins = description.findall('gazebo/plugin')
    assert not any(p.get('name', '').endswith(('JointController', 'JointPositionController')) for p in plugins)
    forces = [p.findtext('joint_name') for p in plugins if p.get('name', '').endswith('ApplyJointForce')]
    joints = [j.get('name') for j in description.findall('joint') if j.get('type') != 'fixed']
    assert len(forces) == len(set(forces)) == 16
    assert set(forces) == set(joints)
    assert description.find("link[@name='base_link']/inertial/mass").get('value') == '36.5'
    assert description.find("gazebo[@reference='imu_link']/sensor[@type='imu']") is not None
    bridge = yaml.safe_load((root / 'course_sim/config/bridge_rl_sar.yaml').read_text())
    commands = {row['gz_topic_name'] for row in bridge if row['direction'] == 'ROS_TO_GZ'}
    assert commands == {f'/model/b2w/joint/{name}/cmd_force' for name in joints}


def test_generated_meshes_load_with_rviz_resource_retriever(tmp_path):
    prefix = Path('/opt/ros/humble')
    if not shutil.which('g++') or not (prefix / 'include/resource_retriever').exists():
        pytest.skip('Native RViz resource_retriever development files unavailable')
    source = tmp_path / 'probe.cpp'
    source.write_text('''#include <resource_retriever/retriever.hpp>
#include <iostream>
int main(int argc,char** argv) {
  resource_retriever::Retriever loader;
  try { for(int i=1;i<argc;++i) if(!loader.get(argv[i]).size) return 1; }
  catch(const std::exception& e) { std::cerr<<e.what(); return 1; }
}
''')
    binary = tmp_path / 'probe'
    subprocess.run(['g++', '-std=c++17', str(source),
                    '-I' + str(prefix / 'include/resource_retriever'),
                    '-I' + str(prefix / 'include/rcutils'), '-L' + str(prefix / 'lib'),
                    '-Wl,-rpath,' + str(prefix / 'lib'), '-lresource_retriever',
                    '-o', str(binary)], check=True, capture_output=True)
    root = Path(__file__).resolve().parents[1]
    model = root / 'b2w_sim/b2w_description_ros2'
    sensors = yaml.safe_load((root / 'course_sim/config/sensors.yaml').read_text())
    urdf = generate_description((model / 'urdf/b2w_description_rsl.urdf').read_text(),
                                sensors, mesh_directory=model / 'meshes')
    meshes = sorted({m.get('filename') for m in ET.fromstring(urdf).findall('.//mesh')})
    assert meshes
    result = subprocess.run([str(binary), *meshes], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_fortress_lidar_ignores_robot_visuals_after_urdf_conversion(tmp_path):
    if not shutil.which('ign'):
        pytest.skip('Fortress URDF converter unavailable')
    root = Path(__file__).resolve().parents[1]
    model = root / 'b2w_sim/b2w_description_ros2'
    sensors = yaml.safe_load((root / 'course_sim/config/sensors.yaml').read_text())
    urdf = generate_description((model / 'urdf/b2w_description_rsl.urdf').read_text(),
                                sensors, mesh_directory=model / 'meshes', joint_control='torque')
    path = tmp_path / 'robot.urdf'
    path.write_text(urdf)
    converted = subprocess.run(['ign', 'sdf', '-p', str(path)], check=True, capture_output=True, text=True)
    sdf = ET.fromstring(converted.stdout)
    mask = int(sdf.findtext('.//sensor[@type="gpu_lidar"]/lidar/visibility_mask', '4294967295'))
    visuals = sdf.findall('.//visual')
    assert visuals
    assert all(int(v.findtext('visibility_flags', '4294967295')) & mask == 0 for v in visuals)
    # Default camera visibility still includes all of these robot visuals.
    assert all(int(v.findtext('visibility_flags')) != 0 for v in visuals)
