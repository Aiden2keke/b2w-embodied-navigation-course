"""Own public localization TF; consume only real LIO-SAM estimates."""
import copy
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from .transforms import pose_matrix, assign_pose
from .ground_alignment import ground_alignment, map_odom_correction


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class PoseAdapter(Node):
    def __init__(self):
        super().__init__('pose_adapter')
        self.base = self.declare_parameter('base_frame', 'base_link').value
        self.odom = self.declare_parameter('odom_frame', 'odom').value
        self.map = self.declare_parameter('map_frame', 'map').value
        xyz = self.declare_parameter('base_to_lidar_xyz', [0.34218, 0.0, 0.17851]).value
        rpy = self.declare_parameter('base_to_lidar_rpy', [0.0, 0.0, 0.0]).value
        base_lidar = np.eye(4)
        base_lidar[:3, 3] = xyz
        base_lidar[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
        self.lidar_base = np.linalg.inv(base_lidar)
        alignment_xyz = self.declare_parameter('map_alignment_xyz', [0.0, 0.0, 0.0]).value
        alignment_rpy = self.declare_parameter('map_alignment_rpy', [0.0, 0.0, 0.0]).value
        self.alignment = np.eye(4)
        self.alignment[:3, 3] = alignment_xyz
        self.alignment[:3, :3] = Rotation.from_euler('xyz', alignment_rpy).as_matrix()
        self.auto_ground = self.declare_parameter('auto_ground_alignment', True).value
        # An explicit saved-map registration already defines the map datum.
        self.ground_ready = not self.auto_ground or not np.allclose(self.alignment, np.eye(4))
        self.initial_alignment = self.alignment.copy()
        self.fit_ground = not self.ground_ready
        self.correction = np.eye(4)
        self.local = deque(maxlen=200)
        self.optimized = deque(maxlen=200)
        self.previous = None
        self.pub = self.create_publisher(Odometry, '/slam/odom', 20)
        self.cloud_pub = self.create_publisher(PointCloud2, '/slam/map_points', 1)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/lio_sam/mapping/odometry_incremental', self.incremental, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/lio_sam/mapping/odometry', self.global_pose, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/lio_sam/mapping/map_global', self.cloud, qos_profile_sensor_data)
        self.get_logger().info('Public TF from LIO-SAM only; twist from differentiated base poses')

    def update_correction(self):
        for gt, global_lidar in reversed(self.optimized):
            for lt, local_lidar in reversed(self.local):
                if abs(gt - lt) < 0.002:
                    self.correction = map_odom_correction(self.alignment, global_lidar, local_lidar)
                    return

    def global_pose(self, msg):
        try:
            self.optimized.append((seconds(msg.header.stamp), pose_matrix(msg.pose.pose)))
            self.update_correction()
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5)

    def broadcast(self, parent, child, matrix, stamp, batch=None):
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id = stamp, parent
        transform.child_frame_id = child
        for axis, value in zip('xyz', matrix[:3, 3]):
            setattr(transform.transform.translation, axis, float(value))
        for axis, value in zip('xyzw', Rotation.from_matrix(matrix[:3, :3]).as_quat()):
            setattr(transform.transform.rotation, axis, float(value))
        if batch is None:
            self.tf.sendTransform(transform)
        else:
            batch.append(transform)

    def incremental(self, msg):
        try:
            t = seconds(msg.header.stamp)
            if self.previous and t <= self.previous[0]:
                self.previous = None
                self.local.clear()
                self.optimized.clear()
                self.correction = np.eye(4)
                if self.fit_ground:
                    self.alignment = self.initial_alignment.copy()
                    self.ground_ready = False
            lidar = pose_matrix(msg.pose.pose)
            self.local.append((t, lidar))
            self.update_correction()
            if not self.ground_ready:
                return
            base = self.alignment @ lidar @ self.lidar_base
            out = Odometry()
            out.header.stamp, out.header.frame_id, out.child_frame_id = msg.header.stamp, self.odom, self.base
            assign_pose(out.pose.pose, base)
            # LIO-SAM covariance[0] is a degeneracy flag, not a covariance matrix.
            # Zero covariance denotes unspecified; do not present fabricated uncertainty.
            if self.previous:
                dt = t - self.previous[0]
                if 0 < dt < 1:
                    old = self.previous[1]
                    velocity = base[:3, :3].T @ ((base[:3, 3] - old[:3, 3]) / dt)
                    omega = Rotation.from_matrix(old[:3, :3].T @ base[:3, :3]).as_rotvec() / dt
                    for field, values in [('linear', velocity), ('angular', omega)]:
                        for axis, value in zip('xyz', values):
                            setattr(getattr(out.twist.twist, field), axis, float(value))
            self.previous = (t, base)
            self.pub.publish(out)
            # Both edges describe this one estimate. A single TF callback must
            # insert both before a consumer checks the latest common stamp.
            transforms = []
            self.broadcast(self.odom, self.base, base, msg.header.stamp, transforms)
            self.broadcast(self.map, self.odom, self.correction, msg.header.stamp, transforms)
            self.tf.sendTransform(transforms)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5)

    def cloud(self, msg):
        from course_sim.cloud_format import decode
        if not self.ground_ready:
            points = decode(msg)
            alignment = ground_alignment(np.column_stack([points[k] for k in 'xyz']))
            if alignment is None:
                self.get_logger().warning('Waiting for a supported floor plane before publishing public localization',
                                          throttle_duration_sec=5)
                return
            self.alignment = alignment
            self.ground_ready = True
            self.previous = None
            self.update_correction()
            self.get_logger().info(f'Ground datum measured from LiDAR: translation z={alignment[2, 3]:.4f} m')
        result = copy.deepcopy(msg)
        if not np.allclose(self.alignment, np.eye(4)):
            points = decode(msg)
            xyz = np.column_stack([points[k] for k in 'xyz'])
            xyz = xyz @ self.alignment[:3, :3].T + self.alignment[:3, 3]
            for i, axis in enumerate('xyz'):
                points[axis] = xyz[:, i]
            result.data = points.tobytes()
            result.height, result.width = 1, len(points)
            result.row_step = result.width * result.point_step
        result.header.frame_id = self.map
        self.cloud_pub.publish(result)


def main():
    rclpy.init()
    node = PoseAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
