"""Limit commands and stop when command/localization becomes stale."""
import math
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class VelocityGate:
    def __init__(self, max_linear=0.5, max_angular=0.6, timeout=0.5):
        if not all(math.isfinite(v) for v in [max_linear, max_angular, timeout]) or min(max_linear, max_angular, timeout) <= 0:
            raise ValueError('Limits/timeouts must be positive')
        self.limits = max_linear, max_angular
        self.timeout = timeout
        self.command = (0., 0., 0.)
        self.command_time = self.state_time = None

    def set_command(self, command, now):
        if not all(math.isfinite(x) for x in command):
            self.command_time = None
            return
        x, y, yaw = command
        scale = min(1., self.limits[0] / max(math.hypot(x, y), 1e-9))
        self.command = x * scale, y * scale, max(-self.limits[1], min(self.limits[1], yaw))
        self.command_time = now

    def output(self, now):
        if any(t is None or not 0 <= now - t <= self.timeout for t in [self.command_time, self.state_time]):
            return 0., 0., 0.
        return self.command


class VelocityAdapter(Node):
    def __init__(self):
        super().__init__('velocity_adapter')
        self.gate = VelocityGate(self.declare_parameter('max_linear', 0.5).value,
                                 self.declare_parameter('max_angular', 0.6).value,
                                 self.declare_parameter('timeout_s', 0.5).value)
        source = self.declare_parameter('input_topic', '/course_navigation/cmd_vel').value
        state_topic = self.declare_parameter('state_topic', '/slam/odom').value
        self.require_tf = self.declare_parameter('require_localization_tf', True).value
        self.pub = self.create_publisher(Twist, '/path_manager/path_manager_ros/nav_vel', 10)
        self.create_subscription(Twist, source, self.command, 10)
        self.create_subscription(Odometry, state_topic, self.state, 20)
        self.last_stamp = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_timer(0.05, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def state(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in [p.x, p.y, p.z, q.x, q.y, q.z, q.w]):
            self.gate.state_time = None
            return
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.gate.command_time = None
        self.last_stamp = stamp
        if 0 <= now - stamp <= self.gate.timeout and q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w > 0.5:
            self.gate.state_time = time.monotonic()

    def command(self, msg):
        self.gate.set_command((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic())

    def publish(self):
        out = Twist()
        try:
            if self.require_tf:
                self.tf.lookup_transform('map', 'base_link', Time())
            out.linear.x, out.linear.y, out.angular.z = self.gate.output(time.monotonic())
        except TransformException:
            pass
        self.pub.publish(out)


def main():
    rclpy.init()
    node = VelocityAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
