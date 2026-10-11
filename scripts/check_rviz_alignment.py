#!/usr/bin/python3
"""Check the live RViz description route and measured ground/robot alignment."""
import argparse
import json
import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
from course_sim.cloud_format import decode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=12.)
    parser.add_argument('--output', default='/tmp/b2w-rviz-alignment.json')
    parser.add_argument('--inject-foreign-description', action='store_true',
                        help='Regression check: publish an unrelated robot on /robot_description')
    args = parser.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '91')
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    dds = 'fastdds_local.xml' if os.environ['ROS_LOCALHOST_ONLY'] == '1' else 'fastdds.xml'
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(Path(__file__).resolve().parents[1] / 'course_sim/config' / dds)
    rclpy.init()
    node = rclpy.create_node('b2w_rviz_alignment_check')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    if args.inject_foreign_description:
        foreign = node.create_publisher(String, '/robot_description', qos)
        foreign.publish(String(data='<robot name="marvin_robot"><link name="base_link"><visual>'
                                    '<geometry><mesh filename="package://marvin_description/meshes/missing.stl"/>'
                                    '</geometry></visual></link></robot>'))
    state = {}
    def description(msg): state['description'] = msg.data
    def cloud(msg):
        p = decode(msg)
        state['xyz'] = np.column_stack([p[k] for k in 'xyz'])
    def grid(msg): state['map'] = msg
    node.create_subscription(String, '/b2w/robot_description', description, qos)
    node.create_subscription(PointCloud2, '/slam/map_points', cloud, qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, '/map', grid, qos)
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    result = {'ros_domain_id': os.environ['ROS_DOMAIN_ID'],
              'localhost_only': os.environ['ROS_LOCALHOST_ONLY'],
              'foreign_description_injected': args.inject_foreign_description}
    checks = {}
    checks['description_received'] = 'description' in state
    if 'description' in state:
        robot = ET.fromstring(state['description'])
        result['robot_name'] = robot.get('name')
        checks['b2w_model'] = robot.get('name') == 'b2w'
        meshes = {m.get('filename') for m in robot.findall('.//mesh')}
        result['mesh_count'] = len(meshes)
        checks['meshes_exist'] = bool(meshes) and all(
            uri.startswith('file://') and Path(uri[7:]).is_file() for uri in meshes)
        bottoms = {}
        for link in robot.findall('link'):
            if not link.get('name').endswith('_foot'):
                continue
            cylinder = link.find('collision/geometry/cylinder')
            origin = link.find('collision/origin')
            if cylinder is None:
                continue
            try:
                tf = buffer.lookup_transform('map', link.get('name'), rclpy.time.Time()).transform
                rotation = Rotation.from_quat([getattr(tf.rotation, k) for k in 'xyzw'])
                center = np.array([getattr(tf.translation, k) for k in 'xyz'])
                offset = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                collision_rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
                center += rotation.apply(offset)
                axis = (rotation * Rotation.from_euler('xyz', collision_rpy)).apply([0., 0., 1.])
                extent = (float(cylinder.get('radius')) * np.sqrt(max(0., 1. - axis[2] ** 2))
                          + .5 * float(cylinder.get('length')) * abs(axis[2]))
                bottoms[link.get('name')] = float(center[2] - extent)
            except Exception as error:
                result['tf_error'] = str(error)
        result['wheel_bottom_z_m'] = bottoms
        checks['wheels_at_ground'] = len(bottoms) == 4 and max(map(abs, bottoms.values())) < .04
    checks['map_received'] = 'map' in state
    if 'map' in state:
        result['occupancy_plane_z_m'] = state['map'].info.origin.position.z
        checks['map_plane_at_zero'] = abs(result['occupancy_plane_z_m']) < 1e-6
    checks['cloud_received'] = 'xyz' in state
    if 'xyz' in state:
        xyz = state['xyz']
        radius = np.linalg.norm(xyz[:, :2], axis=1)
        floor = xyz[(radius > 2) & (radius < 10) & (np.abs(xyz[:, 2]) < .05)]
        result['floor_support'] = len(floor)
        result['floor_median_z_m'] = float(np.median(floor[:, 2])) if len(floor) else None
        result['floor_rms_z_m'] = float(np.sqrt(np.mean(floor[:, 2] ** 2))) if len(floor) else None
        checks['measured_floor_at_zero'] = len(floor) >= 200 and result['floor_rms_z_m'] < .015
    result['description_publishers'] = [i.node_name for i in node.get_publishers_info_by_topic('/b2w/robot_description')]
    checks['one_description_owner'] = result['description_publishers'] == ['b2w_robot_state_publisher']
    rviz_subscriptions = [i.node_name for i in node.get_subscriptions_info_by_topic('/b2w/robot_description')]
    checks['rviz_uses_b2w_description'] = 'rviz' in rviz_subscriptions or 'rviz2' in rviz_subscriptions
    foreign_route = [i.node_name for i in node.get_subscriptions_info_by_topic('/robot_description')]
    checks['rviz_avoids_shared_description'] = not any(n in ['rviz', 'rviz2'] for n in foreign_route)
    result['checks'], result['passed'] = checks, all(checks.values())
    Path(args.output).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
