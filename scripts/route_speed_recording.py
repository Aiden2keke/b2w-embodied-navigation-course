"""Passive route speed evidence. Truth is evaluation-only and never leaves here."""
from collections import defaultdict, deque
import json
import math
import time

import numpy as np


class SpeedEvidence:
    def __init__(self, directory):
        self.directory = directory
        self.trace = (directory/'speed-trace.jsonl').open('w', buffering=1)
        self.statuses = deque(maxlen=100)
        self.samples = defaultdict(list)
        self.invalid = 0

    def event(self, topic, data):
        self.trace.write(json.dumps({'topic': topic, 'receipt_wall_s': time.monotonic(), **data},
                                    allow_nan=False)+'\n')

    def status(self, data):
        if (data.get('mode') not in ('OPEN', 'NARROW') or not math.isfinite(data['stamp_s'])
                or not math.isfinite(data['command_limit']) or data['command_limit'] <= 0):
            raise ValueError('Invalid governor status')
        if self.statuses and data['stamp_s'] < self.statuses[-1]['stamp_s']:
            self.statuses.clear()
        self.statuses.append(data)
        self.event('speed_status', data)

    def odometry(self, source, stamp, vx, vy):
        if not all(math.isfinite(v) for v in (stamp, vx, vy)):
            self.invalid += 1
            return
        # Mode assignment is source-time causal; missing/stale status stays UNKNOWN.
        candidates = [s for s in self.statuses if 0 <= stamp-s['stamp_s'] <= .5]
        mode = candidates[-1]['mode'] if candidates else 'UNKNOWN'
        speed = math.hypot(vx, vy)
        self.samples[(source, mode)].append(speed)
        self.event(source, {'stamp_s': stamp, 'vx': vx, 'vy': vy, 'ground_speed_mps': speed, 'mode': mode})

    def close(self):
        if self.trace.closed:
            return
        self.trace.close()
        summary = {'definition': 'Planar hypot(vx,vy); physical truth and SLAM kept separate. '
                   'Mode labels use <=0.5s causal status, include acceleration, turns and stops. '
                   'A maximum alone does not prove sustained target tracking.',
                   'invalid_samples': self.invalid}
        for source in ('truth', 'slam'):
            summary[source] = {}
            for mode in ('OPEN', 'NARROW', 'UNKNOWN'):
                values = self.samples[(source, mode)]
                summary[source][mode] = dict(count=len(values),
                    median_mps=float(np.median(values)) if values else None,
                    p95_mps=float(np.percentile(values, 95)) if values else None,
                    max_mps=max(values) if values else None)
        (self.directory/'speed-summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')


class RouteSpeedRecorder:
    """Attach to the route runner's existing node, without a second process."""
    def __init__(self, node, directory):
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String
        from nav2_msgs.msg import SpeedLimit
        from rclpy.qos import qos_profile_sensor_data
        self.evidence = SpeedEvidence(directory)
        self.node, self.directory = node, directory
        self.subscriptions, self.clients, self.futures = [], [], {}
        def subscribe(kind, topic, callback, qos=10):
            self.subscriptions.append(node.create_subscription(kind, topic, callback, qos))
        def status(msg):
            try:
                self.evidence.status(json.loads(msg.data))
            except (ValueError, KeyError, TypeError):
                self.evidence.invalid += 1
        subscribe(String, '/course_navigation/speed_status', status)
        def stamp(msg):
            return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        def odometry(source):
            return lambda m: self.evidence.odometry(source, stamp(m), m.twist.twist.linear.x,
                                                     m.twist.twist.linear.y)
        subscribe(Odometry, '/sim/ground_truth/odom', odometry('truth'), qos_profile_sensor_data)
        subscribe(Odometry, '/slam/odom', odometry('slam'), qos_profile_sensor_data)
        for topic in ('/course_navigation/cmd_vel_raw', '/course_navigation/cmd_vel',
                      '/path_manager/path_manager_ros/nav_vel'):
            def command(msg, topic=topic):
                values = [msg.linear.x, msg.linear.y, msg.angular.z]
                if all(math.isfinite(v) for v in values):
                    self.evidence.event(topic, {'receipt_ros_s': node.get_clock().now().nanoseconds*1e-9,
                                               'command': values})
                else:
                    self.evidence.invalid += 1
            subscribe(Twist, topic, command)
        def limit(msg):
            if math.isfinite(msg.speed_limit):
                self.evidence.event('speed_limit', {'stamp_s': stamp(msg), 'percentage': msg.percentage,
                                                   'command_limit': msg.speed_limit})
        subscribe(SpeedLimit, '/course_navigation/speed_limit', limit)
        def goal(msg):
            values = [msg.pose.position.x, msg.pose.position.y]
            if all(math.isfinite(v) for v in values):
                self.evidence.event('/goal_pose', {'stamp_s': stamp(msg), 'frame_id': msg.header.frame_id,
                                                  'position': values})
        subscribe(PoseStamped, '/goal_pose', goal)
        # Read loaded runtime parameters once, asynchronously; never mutate them.
        from rcl_interfaces.srv import GetParameters
        from dataclasses import fields
        from course_navigation.adaptive_speed import Config
        from course_navigation.adaptive_profile import RPP
        requests = {
            'planner_server': ['planner_plugins', 'GridBased.plugin', 'GridBased.tolerance', 'GridBased.allow_unknown'],
            'speed_governor': [f.name for f in fields(Config)]+['footprint', 'map_timeout_s',
                'odom_timeout_s', 'path_timeout_s', 'tf_timeout_s', 'cloud_timeout_s'],
            'controller_server': ['FollowPath.'+name for name in RPP]+['speed_limit_topic'],
            'velocity_smoother': ['max_velocity', 'min_velocity', 'max_accel', 'max_decel'],
            'velocity_adapter': ['max_linear', 'max_angular', 'speed_limit_topic', 'm2_inverse_enabled'],
        }
        for name, names in requests.items():
            client = node.create_client(GetParameters, '/'+name+'/get_parameters')
            self.clients.append(client)
            self.futures[name] = (names, client.call_async(GetParameters.Request(names=names)))

    def close(self):
        from rclpy.parameter import parameter_value_to_python
        snapshots = {}
        for name, (names, future) in self.futures.items():
            if future.done() and future.exception() is None:
                snapshots[name] = dict(zip(names, [parameter_value_to_python(v) for v in future.result().values]))
            else:
                snapshots[name] = {'status': 'runtime_parameters_unavailable'}
        (self.directory/'speed-runtime-parameters.json').write_text(json.dumps(snapshots, indent=2)+'\n')
        self.evidence.close()
