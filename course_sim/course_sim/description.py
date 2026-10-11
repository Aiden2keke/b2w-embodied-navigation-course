"""Generate a Fortress course model while preserving upstream policy and joints."""
import math
import copy
import xml.etree.ElementTree as ET
from pathlib import Path


def sub(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if text is not None:
        node.text = str(text)
    return node


def generate_description(urdf, sensors, slip_ratio=0.0, mesh_directory=None,
                         joint_control='legacy', dynamics_urdf=None):
    if not 0 <= slip_ratio <= 1:
        raise ValueError('slip_ratio must be in [0,1]')
    root = ET.fromstring(urdf)
    root.set('name', 'b2w')
    if joint_control not in ['legacy', 'torque']:
        raise ValueError('joint_control must be legacy or torque')
    if dynamics_urdf:
        dynamics = ET.fromstring(dynamics_urdf)
        for link in root.findall('link'):
            source_name = 'trunk' if link.get('name') == 'base_link' else link.get('name')
            source = dynamics.find(f"link[@name='{source_name}']/inertial")
            if source is not None:
                for old in list(link.findall('inertial')):
                    link.remove(old)
                link.insert(0, copy.deepcopy(source))
        for joint in root.findall('joint'):
            if joint.get('type') == 'fixed':
                continue
            for old in list(joint.findall('dynamics')):
                joint.remove(old)
            source = dynamics.find(f"joint[@name='{joint.get('name')}']/dynamics")
            if source is not None:
                joint.append(copy.deepcopy(source))
    for sensor in root.findall('.//sensor'):
        for plugin in list(sensor.findall('plugin')):
            sensor.remove(plugin)
    for gazebo in root.findall('gazebo'):
        for plugin in list(gazebo.findall('plugin')):
            if joint_control == 'torque' and any(name in plugin.get('name', '') for name in
                                                ['JointPositionController', 'JointController']):
                gazebo.remove(plugin)
                continue
            if any(name in plugin.get('name', '') for name in ['Sensors', 'ForceTorque']):
                gazebo.remove(plugin)  # These systems belong on the world.
                continue
            plugin.set('filename', plugin.get('filename', '').replace('gz-sim-', 'ignition-gazebo-'))
            plugin.set('name', plugin.get('name', '').replace('gz::sim::', 'ignition::gazebo::'))
            if 'JointStatePublisher' in plugin.get('name', ''):
                sub(plugin, 'update_rate', 100)
                for joint in root.findall('joint'):
                    if joint.get('type') != 'fixed':
                        sub(plugin, 'joint_name', joint.get('name'))
            if 'OdometryPublisher' in plugin.get('name', ''):
                plugin.find('odom_frame').text = 'sim_world'
                plugin.find('tf_topic').text = '/sim/raw/ground_truth_tf'
                sub(plugin, 'odom_topic', '/sim/raw/ground_truth/odom')
    if joint_control == 'torque':
        for joint in root.findall('joint'):
            if joint.get('type') == 'fixed':
                continue
            plugin = sub(sub(root, 'gazebo'), 'plugin',
                         filename='ignition-gazebo-apply-joint-force-system',
                         name='ignition::gazebo::systems::ApplyJointForce')
            sub(plugin, 'joint_name', joint.get('name'))
    if mesh_directory:
        for mesh in root.findall('.//mesh'):
            prefix = 'package://b2w_description_ros2/meshes/'
            if mesh.get('filename', '').startswith(prefix):
                # RViz resource_retriever requires a URI, while Fortress accepts file:// too.
                mesh.set('filename', (Path(mesh_directory) / mesh.get('filename')[len(prefix):]).absolute().as_uri())

    # Reserve bit 1 for this robot's render geometry. GUI cameras still see it;
    # the course LiDAR uses bit 0 to model a self-filtered environmental scan.
    # These extensions survive URDF fixed-link reduction in Fortress.
    for link in root.findall('link'):
        if link.find('visual') is not None:
            visual = sub(sub(root, 'gazebo', reference=link.get('name')), 'visual')
            sub(visual, 'visibility_flags', 2)

    lidar = sensors['lidar']
    imu = sensors['imu']
    for name, config, kind, topic in [
            ('course_xt32', lidar, 'gpu_lidar', '/sim/raw/scan'),
            ('course_imu', imu, 'imu', '/sim/raw/imu')]:
        gazebo = sub(root, 'gazebo', reference=config['frame'])
        sensor = sub(gazebo, 'sensor', name=name, type=kind)
        sub(sensor, 'always_on', 'true')
        sub(sensor, 'update_rate', config['rate_hz'])
        sub(sensor, 'topic', topic)
        sub(sensor, 'ignition_frame_id', config['frame'])
        if kind == 'gpu_lidar':
            ray = sub(sensor, 'lidar')
            sub(ray, 'visibility_mask', 1)
            scan = sub(ray, 'scan')
            horizontal = sub(scan, 'horizontal')
            count = int(lidar['horizontal_samples'])
            if count < 2 or int(lidar['channels']) != 32:
                raise ValueError('XT32 requires 32 channels and at least two horizontal samples')
            for key, value in [('samples', count), ('resolution', 1), ('min_angle', -math.pi),
                               ('max_angle', math.pi - 2 * math.pi / count)]:
                sub(horizontal, key, value)
            vertical = sub(scan, 'vertical')
            for key, value in [('samples', 32), ('resolution', 1),
                               ('min_angle', math.radians(lidar['min_vertical_deg'])),
                               ('max_angle', math.radians(lidar['max_vertical_deg']))]:
                sub(vertical, key, value)
            distance = sub(ray, 'range')
            for key, value in [('min', lidar['min_range_m']), ('max', lidar['max_range_m']), ('resolution', 0.001)]:
                sub(distance, key, value)
        else:
            sub(sensor, 'imu')

    plugin = sub(sub(root, 'gazebo'), 'plugin',
                 filename='ignition-gazebo-wheel-slip-system',
                 name='ignition::gazebo::systems::WheelSlip')
    total_mass = sum(float(m.get('value')) for m in root.findall('link/inertial/mass'))
    for joint in root.findall('joint'):
        if not joint.get('name', '').endswith('_foot_joint'):
            continue
        link = joint.find('child').get('link')
        wheel = sub(plugin, 'wheel', link_name=link)
        sub(wheel, 'slip_compliance_lateral', slip_ratio)
        sub(wheel, 'slip_compliance_longitudinal', slip_ratio)
        sub(wheel, 'wheel_normal_force', total_mass * 9.81 / 4)
        collision = root.find(f"link[@name='{link}']/collision/geometry/cylinder")
        radius = float(collision.get('radius')) if collision is not None else 0.1125
        sub(wheel, 'wheel_radius', radius)
    ET.indent(root)
    return ET.tostring(root, encoding='unicode')


def make_world():
    sdf = ET.Element('sdf', version='1.8')
    world = sub(sdf, 'world', name='course_world')
    physics = sub(world, 'physics', name='physics', type='ignored')
    sub(physics, 'max_step_size', 0.0025)
    sub(physics, 'real_time_factor', 1.0)
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors'),
                           ('imu', 'Imu'), ('forcetorque', 'ForceTorque')]:
        p = sub(world, 'plugin', filename=f'ignition-gazebo-{filename}-system',
                name=f'ignition::gazebo::systems::{name}')
        if name == 'Sensors':
            sub(p, 'render_engine', 'ogre2')
    light = sub(world, 'light', name='sun', type='directional')
    sub(light, 'pose', '0 0 10 0 0 0')
    sub(light, 'direction', '-0.5 0.1 -0.9')
    sub(light, 'diffuse', '0.8 0.8 0.8 1')
    boxes = [('floor', (0, 0, -0.1), (20, 16, 0.2)),
             ('north', (0, 1.6, 1), (16, 0.2, 2)),
             ('south', (-2, -1.6, 1), (12, 0.2, 2)),
             ('west', (-8, 0, 1), (0.2, 3.4, 2)),
             ('east', (8, -2.4, 1), (0.2, 8, 2)),
             ('branch', (3.8, -4.0, 1), (0.2, 5, 2)),
             ('end', (5.9, -6.5, 1), (4.2, 0.2, 2)),
             ('obstacle', (0, 0.9, 0.5), (0.6, 0.6, 1))]
    for name, position, size in boxes:
        model = sub(world, 'model', name=name)
        sub(model, 'static', 'true')
        sub(model, 'pose', ' '.join(map(str, position)) + ' 0 0 0')
        link = sub(model, 'link', name='link')
        for tag in ['collision', 'visual']:
            obj = sub(link, tag, name=tag)
            sub(sub(sub(obj, 'geometry'), 'box'), 'size', ' '.join(map(str, size)))
            if tag == 'visual':
                sub(sub(obj, 'material'), 'diffuse', '0.6 0.65 0.7 1')
    ET.indent(sdf)
    return ET.tostring(sdf, encoding='unicode')
