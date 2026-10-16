"""Translate goal poses into Nav2 actions and save measured per-goal outcomes."""
import json
import math
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from tf2_ros import Buffer, TransformListener, TransformException
from course_slam.transforms import pose_matrix
from .metrics import RunMetrics
from lifecycle_msgs.srv import GetState
from .lifecycle_readiness import LifecycleReadiness
from rosidl_runtime_py.convert import message_to_ordereddict


class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')
        self.timeout = self.declare_parameter('timeout_s', 120.).value
        self.tolerance = self.declare_parameter('goal_tolerance_m', 0.3).value
        self.force_threshold = self.declare_parameter('collision_force_threshold_n', 50.).value
        self.directory = Path(self.declare_parameter('results_directory', 'course_results/navigation').value)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.readiness = LifecycleReadiness([
            self.create_client(GetState, f'/{name}/get_state')
            for name in ['bt_navigator', 'planner_server', 'controller_server']])
        self.create_subscription(PoseStamped, '/goal_pose', self.goal, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/slam/odom', self.pose, 20)
        self.create_subscription(WrenchStamped, '/collision_plate/force_torque', self.collision, 20)
        self.create_service(Trigger, '/course_navigation/cancel_goal', self.cancel)
        self.active = None
        self.nav_handle = None
        self.planning_handle = None
        self.pending = None
        self.pending_poses = deque(maxlen=100)
        self.last_odom = None
        self.last_sim_time = None
        self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def goal(self, msg):
        try:
            pose_matrix(msg.pose)  # Reject non-finite positions/invalid quaternions.
        except ValueError as error:
            self.get_logger().error(f'Invalid goal: {error}')
            return
        if msg.header.frame_id != 'map':
            self.get_logger().error('Publish /goal_pose in map; use RViz fixed frame map')
            return
        self.finish(False, 'replaced')
        self.goal_message = message_to_ordereddict(msg)
        self.start_map_base_link = self.map_base_snapshot()
        self.nav2_status = None  # Only a received action result supplies this.
        self.pending = msg
        self.active = RunMetrics(self.sim_time(), (msg.pose.position.x, msg.pose.position.y), self.tolerance)
        if self.last_odom is not None:
            self.pending_poses.append(self.last_odom)
        self.started_wall = time.monotonic()
        self.goal_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.get_logger().info(f'Goal {self.goal_id}: waiting for Nav2')

    def cancel(self, request, response):
        response.success = self.active is not None
        response.message = 'Canceled active goal' if response.success else 'No active goal'
        self.finish(False, 'canceled_by_user')
        return response

    def sim_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        navigation_active = self.readiness.active(time.monotonic())
        pending = self.pending_poses
        self.pending_poses = deque(maxlen=100)
        for msg in pending:
            if not self.measure_pose(msg):
                self.pending_poses.append(msg)
        now = self.sim_time()
        if self.last_sim_time is not None and now < self.last_sim_time:
            self.finish(False, 'clock_reset')
        self.last_sim_time = now
        if self.active is None:
            return
        if now - self.active.start > self.timeout or time.monotonic() - self.started_wall > self.timeout * 5:
            self.finish(False, 'timeout')
            return
        if (self.pending is None or not navigation_active
                or not self.nav.server_is_ready() or not self.planner.server_is_ready()):
            return
        if not self.tf.can_transform('map', 'base_link', Time()):
            return
        target, token = self.pending, self.active
        self.pending = None
        goal = NavigateToPose.Goal()
        goal.pose = target
        self.nav.send_goal_async(goal).add_done_callback(lambda f: self.accepted(f, token))
        plan = ComputePathToPose.Goal()
        plan.goal, plan.planner_id, plan.use_start = target, 'GridBased', False
        started = time.monotonic()
        self.planner.send_goal_async(plan).add_done_callback(lambda f: self.planning_accepted(f, token, started))

    def accepted(self, future, token):
        try:
            handle = future.result()
        except Exception as error:
            if self.active is token:
                self.get_logger().error(str(error))
                self.finish(False, 'action_error')
            return
        if self.active is not token:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.finish(False, 'rejected')
            return
        self.nav_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.result(f, token))

    def result(self, future, token):
        if self.active is not token:
            return
        try:
            status = future.result().status
        except Exception:
            self.finish(False, 'action_error')
            return
        self.nav2_status = status
        self.finish(status == GoalStatus.STATUS_SUCCEEDED, {
            GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'aborted'}.get(status, 'action_error'))

    def planning_accepted(self, future, token, started):
        try:
            handle = future.result()
        except Exception:
            return
        if self.active is not token:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if handle.accepted:
            self.planning_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.planned(f, token, started))

    def planned(self, future, token, started):
        if self.active is token:
            try:
                status = future.result().status
                token.planning_latency_ms = (time.monotonic() - started) * 1000
                token.planning_status = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                    GoalStatus.STATUS_CANCELED: 'canceled', GoalStatus.STATUS_ABORTED: 'aborted'}.get(status, 'unknown')
            except Exception:
                pass

    def pose(self, msg):
        self.last_odom = msg
        if self.active is not None:
            self.pending_poses.append(msg)

    def measure_pose(self, msg):
        if self.active is None:
            return True
        try:
            odom_pose = pose_matrix(msg.pose.pose)
            transform = self.tf.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
            t = transform.transform
            from scipy.spatial.transform import Rotation
            import numpy as np
            world = np.eye(4)
            world[:3, :3] = Rotation.from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]).as_matrix()
            world[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
            point = (world @ odom_pose)[:2, 3]
        except (ValueError, TransformException):
            return False
        p = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.active.pose(p.x, p.y, tuple(point), stamp)
        return True

    def collision(self, msg):
        if self.active is not None:
            f = msg.wrench.force
            self.active.collision(self.sim_time(), math.sqrt(f.x*f.x + f.y*f.y + f.z*f.z), self.force_threshold)

    def map_base_snapshot(self):
        """Small, stamped boundary evidence; never substitute stale TF."""
        try:
            transform = self.tf.lookup_transform('map', 'base_link', Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            age = self.sim_time() - stamp
            if 0 <= age <= 0.5:
                return dict(message_to_ordereddict(transform), age_s=age)
        except TransformException:
            pass
        return None

    def finish(self, success, reason):
        if self.active is None:
            return
        end_map_base_link = self.map_base_snapshot()
        if end_map_base_link is not None:
            p = end_map_base_link['transform']['translation']
            self.active.goal_position = (p['x'], p['y'])
        record = self.active.finish(self.sim_time(), success, reason)
        record.update(goal_id=self.goal_id, goal_map_xy=list(self.active.goal), timeout_s=self.timeout,
                      goal_pose=self.goal_message, nav2_status=self.nav2_status,
                      start_map_base_link=self.start_map_base_link,
                      end_map_base_link=end_map_base_link,
                      collision_force_threshold_n=self.force_threshold,
                      final_goal_distance_m=math.dist(self.active.goal_position, self.active.goal)
                      if self.active.goal_position is not None else None)
        with (self.directory / (self.goal_id + '.json')).open('x') as out:
            json.dump(record, out, indent=2, allow_nan=False)
        self.active, self.pending = None, None
        self.pending_poses.clear()
        for handle in [self.nav_handle, self.planning_handle]:
            if handle is not None and rclpy.ok():
                handle.cancel_goal_async()
        self.nav_handle, self.planning_handle = None, None
        self.get_logger().info(f'Navigation {reason}; metrics saved for {self.goal_id}')


def main():
    rclpy.init()
    node = GoalManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish(False, 'shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
