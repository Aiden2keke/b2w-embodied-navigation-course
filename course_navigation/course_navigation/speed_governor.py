"""Opt-in geometry governor for Humble RPP. Ground truth is never subscribed."""
from dataclasses import fields, asdict
import json
import math
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path
from nav2_msgs.msg import Costmap, SpeedLimit
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from .adaptive_speed import Config, CostGrid, Governor, fresh


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec*1e-9


def yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values)-1.) > .05:
        raise ValueError('Invalid quaternion')
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class SpeedGovernor(Node):
    def __init__(self):
        super().__init__('speed_governor')
        options = {f.name: self.declare_parameter(f.name, 0. if f.name == 'w_ref' else f.default).value
                   for f in fields(Config)}
        self.governor = Governor(Config(**options))
        self.footprint = json.loads(self.declare_parameter('footprint', '').value)
        self.map_timeout = self.declare_parameter('map_timeout_s', 1.5).value
        self.odom_timeout = self.declare_parameter('odom_timeout_s', .6).value
        self.path_timeout = self.declare_parameter('path_timeout_s', .75).value
        self.tf_timeout = self.declare_parameter('tf_timeout_s', .6).value
        self.cloud_timeout = self.declare_parameter('cloud_timeout_s', .75).value
        if any(not math.isfinite(v) or v <= 0 for v in
               (self.map_timeout, self.odom_timeout, self.path_timeout, self.tf_timeout, self.cloud_timeout)):
            raise ValueError('Freshness timeouts must be positive')
        self.grid = self.path = self.speed = None
        self.map_frame = None
        self.times = {}
        self.last_ros = None
        self.last_tick_wall = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        topic = self.declare_parameter('speed_limit_topic', '/course_navigation/speed_limit').value
        self.limit_pub = self.create_publisher(SpeedLimit, topic, 10)
        self.status_pub = self.create_publisher(String, '/course_navigation/speed_status', 10)
        self.create_subscription(Costmap, '/local_costmap/costmap_raw', self.map_received, 1)
        # Humble RPP republishes its *transformed and pruned* path here each control tick.
        self.create_subscription(Path, '/received_global_plan', self.path_received, 1)
        self.create_subscription(Odometry, '/slam/odom', self.odom_received, qos_profile_sensor_data)
        # A fresh costmap publication alone cannot establish obstacle-source freshness.
        # Inspect only the public scan header/size; geometry still comes from Nav2.
        self.create_subscription(PointCloud2, '/pointcloud', self.cloud_received, qos_profile_sensor_data)
        self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Adaptive RPP speed commands (not physical calibration): '+json.dumps(options))

    def note(self, name, stamp):
        previous = self.times.get(name)
        self.times[name] = (stamp, time.monotonic())
        if previous is not None and stamp < previous[0]:
            self.governor.mode, self.governor.clear_since = 'NARROW', None

    def map_received(self, msg):
        self.grid = None
        try:
            m = msg.metadata
            if not msg.header.frame_id or abs(m.origin.orientation.x)+abs(m.origin.orientation.y) > 1e-5:
                raise ValueError('Invalid map frame/origin')
            cells = np.asarray(msg.data, dtype=np.uint8).reshape(m.size_y, m.size_x)
            self.grid = CostGrid(cells, m.resolution,
                                 (m.origin.position.x, m.origin.position.y, yaw(m.origin.orientation)))
            if self.map_frame is not None and self.map_frame != msg.header.frame_id:
                self.path = None
                self.times.pop('path', None)
            self.map_frame = msg.header.frame_id
            # Humble 1.1.20 prepareCostmap() leaves metadata.update_time at zero
            # on the raw topic (unlike GetCostmap). Header+receipt are authoritative
            # there; honor a nonzero update timestamp if a publisher supplies one.
            header, update = stamp_s(msg.header.stamp), stamp_s(m.update_time)
            self.note('map', min(header, update) if update > 0 else header)
        except (ValueError, TypeError):
            self.times.pop('map', None)

    def path_received(self, msg):
        self.path = None
        try:
            if self.map_frame is None or not msg.header.frame_id:
                return
            if len(msg.poses) < 2:
                raise ValueError('Empty/degenerate path')
            points = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
            if not np.isfinite(points).all():
                raise ValueError('Nonfinite path')
            if msg.header.frame_id != self.map_frame:
                # Use the RPP message timestamp; a latest transform would shift old paths.
                tr = self.tf.lookup_transform(self.map_frame, msg.header.frame_id,
                                             Time.from_msg(msg.header.stamp))
                a = yaw(tr.transform.rotation)
                p = tr.transform.translation
                points = points @ np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]]) + (p.x, p.y)
            self.path = points
            self.note('path', stamp_s(msg.header.stamp))
        except (ValueError, TypeError, TransformException):
            self.times.pop('path', None)

    def odom_received(self, msg):
        self.speed = None
        try:
            p, t = msg.pose.pose.position, msg.twist.twist.linear
            yaw(msg.pose.pose.orientation)
            if msg.child_frame_id != 'base_link' or not all(math.isfinite(v) for v in (p.x, p.y, p.z, t.x, t.y)):
                raise ValueError('Invalid odometry')
            self.speed = math.hypot(t.x, t.y)
            self.note('odom', stamp_s(msg.header.stamp))
        except ValueError:
            self.times.pop('odom', None)

    def cloud_received(self, msg):
        if msg.width > 0 and msg.height > 0 and len(msg.data) >= msg.row_step*msg.height > 0:
            self.note('cloud', stamp_s(msg.header.stamp))
        else:
            self.times.pop('cloud', None)

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        wall = time.monotonic()
        tick_gap = 0. if self.last_tick_wall is None else wall-self.last_tick_wall
        self.last_tick_wall = wall
        if self.last_ros is not None and now < self.last_ros:
            self.times.clear()  # A new epoch must reacquire every source.
        self.last_ros = now
        ages = {}
        # After a scheduling/processing stall, re-establish sustained clearance.
        # Match the adapter heartbeat timeout rather than retaining an old OPEN hold.
        healthy = 0 <= tick_gap <= .5
        for name, timeout in [('map', self.map_timeout), ('path', self.path_timeout),
                              ('odom', self.odom_timeout), ('cloud', self.cloud_timeout)]:
            stamp, receipt = self.times.get(name, (None, None))
            ages[name] = None if stamp is None else now-stamp
            healthy &= fresh(now, wall, stamp, receipt, timeout)
        pose = None
        try:
            # map->base tests localization; odom->base supplies costmap-frame geometry.
            localization = self.tf.lookup_transform('map', 'base_link', Time())
            ages['localization_tf'] = now-stamp_s(localization.header.stamp)
            healthy &= 0 <= ages['localization_tf'] <= self.tf_timeout
            if self.map_frame is not None:
                tr = self.tf.lookup_transform(self.map_frame, 'base_link', Time())
                ages['costmap_tf'] = now-stamp_s(tr.header.stamp)
                healthy &= 0 <= ages['costmap_tf'] <= self.tf_timeout
                pose = (tr.transform.translation.x, tr.transform.translation.y, yaw(tr.transform.rotation))
        except (TransformException, ValueError):
            healthy = False
        decision = self.governor.update(now, self.grid, self.path, pose,
                                        self.footprint, self.speed, healthy)
        limit = SpeedLimit()
        limit.header.stamp = self.get_clock().now().to_msg()
        limit.percentage = False
        limit.speed_limit = decision.command_limit  # Always >0: zero restores Nav2 defaults.
        self.limit_pub.publish(limit)
        status = String()
        status.data = json.dumps({**asdict(decision), 'stamp_s': now, 'healthy': bool(healthy),
                                  'measured_slam_speed': self.speed, 'source_age_s': ages,
                                  'tick_gap_s': tick_gap}, allow_nan=False)
        self.status_pub.publish(status)


def main():
    rclpy.init()
    node = SpeedGovernor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
