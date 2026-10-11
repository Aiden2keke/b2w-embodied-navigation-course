"""Project a real SLAM map and use localized live scans for free-space observations."""
import numpy as np
from collections import deque
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation
from course_sim.cloud_format import decode
from .map_projection import Grid


class MapProjector(Node):
    def __init__(self):
        super().__init__('map_projector')
        resolution = self.declare_parameter('resolution', 0.05).value
        origin = self.declare_parameter('origin', [-10., -8.]).value
        size = self.declare_parameter('size', [20., 16.]).value
        self.grid = Grid(resolution, origin, size)
        self.min_z = self.declare_parameter('min_height', 0.15).value
        self.max_z = self.declare_parameter('max_height', 2.).value
        self.frame = self.declare_parameter('map_frame', 'map').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(OccupancyGrid, '/map', qos)
        self.create_subscription(PointCloud2, '/slam/map_points', self.map, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/pointcloud', self.scan, qos_profile_sensor_data)
        self.create_timer(1., self.publish)
        self.pending = deque(maxlen=30)
        self.create_timer(0.05, self.process_scans)
        self.have_map = False

    @staticmethod
    def xyz(msg):
        points = decode(msg)
        return np.column_stack([points[k] for k in 'xyz'])

    def map(self, msg):
        if msg.header.frame_id != self.frame:
            self.get_logger().error('SLAM map frame mismatch', throttle_duration_sec=5)
            return
        self.grid.obstacles(self.xyz(msg), self.min_z, self.max_z)
        self.have_map = True

    def scan(self, msg):
        if self.pending and msg.header.stamp.sec < self.pending[-1].header.stamp.sec:
            self.pending.clear()
        self.pending.append(msg)

    def process_scans(self):
        remaining = deque(maxlen=30)
        while self.pending:
            msg = self.pending.popleft()
            if not self.project_scan(msg):
                remaining.append(msg)
        self.pending = remaining

    def project_scan(self, msg):
        try:
            tf = self.buffer.lookup_transform(self.frame, msg.header.frame_id,
                                              rclpy.time.Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.0))
            t = tf.transform
            quaternion = [getattr(t.rotation, k) for k in 'xyzw']
            rotation = Rotation.from_quat(quaternion)
            origin = np.array([getattr(t.translation, k) for k in 'xyz'])
            points = self.xyz(msg)
            # Exclude local self returns before clearing map cells.
            points = points[np.linalg.norm(points, axis=1) > 0.5]
            self.grid.observe(origin, rotation.apply(points) + origin)
            return True
        except (TransformException, ValueError) as error:
            return False  # Wait for the SLAM transform at this scan's time.

    def publish(self):
        if not self.have_map:
            return
        msg = OccupancyGrid()
        msg.header.frame_id, msg.header.stamp = self.frame, self.get_clock().now().to_msg()
        msg.info.resolution = self.grid.resolution
        msg.info.width, msg.info.height = self.grid.width, self.grid.height
        msg.info.origin.position.x, msg.info.origin.position.y = map(float, self.grid.origin)
        msg.info.origin.orientation.w = 1.
        msg.data = self.grid.data().reshape(-1).tolist()
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = MapProjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
