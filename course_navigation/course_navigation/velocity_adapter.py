"""Limit commands and stop when command/localization becomes stale."""
import math
import json
from . import m2_inverse
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
        self.m2_enabled = self.declare_parameter('m2_inverse_enabled', False).value
        self.m2_diagnostic = None
        if self.m2_enabled:
            from std_msgs.msg import String
            self.m2_message_type = String
            self.m2_diagnostic = self.create_publisher(String, m2_inverse.DIAGNOSTIC_TOPIC, 10)
            self.get_logger().info('M2 strict diagnostic candidate: ' + json.dumps(m2_inverse.identity(), sort_keys=True))
        self.gate = VelocityGate(self.declare_parameter('max_linear', 0.5).value,
                                 self.declare_parameter('max_angular', 0.6).value,
                                 self.declare_parameter('timeout_s', 0.5).value)
        limit_topic = self.declare_parameter('speed_limit_topic', '').value
        self.speed_limit = None
        if limit_topic:
            if self.m2_enabled:
                raise ValueError('Adaptive RPP uses direct policy commands; M2 must remain disabled')
            from nav2_msgs.msg import SpeedLimit
            from .adaptive_speed import CommandLimit
            self.speed_limit = CommandLimit(self.gate.limits[0], self.gate.timeout)
            self.create_subscription(SpeedLimit, limit_topic, self.receive_speed_limit, 10)
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
        if self.m2_enabled:
            # Check before legacy limiting as well: never hide an unsupported request.
            self.convert_m2((msg.linear.x, msg.linear.y, msg.angular.z))
        self.gate.set_command((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic())

    def receive_speed_limit(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.speed_limit.receive(msg.speed_limit, msg.percentage, stamp, time.monotonic())

    def convert_m2(self, command):
        try:
            result = m2_inverse.transform(command, enabled=True)
        except m2_inverse.UnsupportedVelocity as exc:
            self.report_m2(command, None, 'unsupported', str(exc))
            self.get_logger().fatal(str(exc) + '; strict candidate terminates; no inverse fallback')
            raise  # Stop at the design boundary; do not publish a substitute Twist.
        self.report_m2(command, result, 'accepted', None)
        return result

    def report_m2(self, desired, policy, status, reason):
        msg = self.m2_message_type()
        msg.data = json.dumps({'candidate': m2_inverse.identity(), 'status': status,
                               'desired': desired, 'policy': policy, 'reason': reason})
        self.m2_diagnostic.publish(msg)

    def publish(self):
        out = Twist()
        try:
            if self.require_tf:
                self.tf.lookup_transform('map', 'base_link', Time())
            command = self.gate.output(time.monotonic())
            if getattr(self, 'speed_limit', None) is not None:
                command = self.speed_limit.apply(command, self.get_clock().now().nanoseconds*1e-9,
                                                 time.monotonic())
            if self.m2_enabled:
                command = self.convert_m2(command)
            out.linear.x, out.linear.y, out.angular.z = command
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
