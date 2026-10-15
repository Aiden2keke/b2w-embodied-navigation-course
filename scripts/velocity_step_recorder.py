"""Native-event recorder for the isolated velocity-step runner.

ROS packages are imported only by ``create_recorder`` so offline replay and
dry-run tooling remain usable on machines without ROS.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path


def _finite_payload(value, bad):
    if isinstance(value, float):
        if not math.isfinite(value):
            bad.append(True)
            return None
        return value
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    if isinstance(value, (list, tuple)):
        return [_finite_payload(v, bad) for v in value]
    if isinstance(value, dict):
        return {str(k): _finite_payload(v, bad) for k, v in value.items()}
    return str(value)


def create_recorder(output_dir: Path, joint_decimation_hz: float = 100.):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import Float64MultiArray
    from tf2_msgs.msg import TFMessage

    class VelocityStepRecorder(Node):
        topics = {
            '/harness/requested': (Twist, 'twist'),
            '/course_navigation/cmd_vel_raw': (Twist, 'twist'),
            '/course_navigation/cmd_vel': (Twist, 'twist'),
            '/path_manager/path_manager_ros/nav_vel': (Twist, 'twist'),
            '/sim/ground_truth/odom': (Odometry, 'odometry'),
            '/slam/odom': (Odometry, 'odometry'),
            '/clock': (Clock, 'clock'),
            '/sim/raw/imu': (Imu, 'imu'),
            '/sim/raw/joint_states': (JointState, 'joint_state'),
            '/network_input_debug': (Float64MultiArray, 'float64_multi_array'),
            '/joint_commands': (Float64MultiArray, 'float64_multi_array'),
            '/joint_torques': (Float64MultiArray, 'float64_multi_array'),
            '/tf': (TFMessage, 'tf'),
            '/tf_static': (TFMessage, 'tf_static'),
        }

        def __init__(self, directory):
            super().__init__('velocity_step_recorder')
            self.directory = Path(directory)
            self.stream = (self.directory / 'events.jsonl').open('a', encoding='utf-8', buffering=65536)
            self.started_wall = time.monotonic()
            self.sim_s = None
            self.epoch = 0
            self.order = 0
            self.last_clock = None
            self.last_joint_source = None
            self.joint_period = 1. / float(joint_decimation_hz)
            self.counts = {}
            self.received_counts = {}
            self.decimated_counts = {}
            self.last_received = {}
            self.invalid_counts = {}
            self.tf_static_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.publisher = self.create_publisher(Twist, '/course_navigation/cmd_vel_raw', 10)
            for topic, (msg_type, kind) in self.topics.items():
                qos=(qos_profile_sensor_data if kind in ('joint_state','imu','clock','tf') else
                     self.tf_static_qos if topic=='/tf_static' else 10)
                self.create_subscription(msg_type, topic, lambda msg, t=topic, k=kind: self._receive(t, k, msg), qos)

        @staticmethod
        def stamp_value(stamp):
            return int(stamp.sec) + int(stamp.nanosec) * 1e-9

        @staticmethod
        def stamp_parts(stamp):
            return {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)}

        def _receive(self, topic, kind, msg):
            self.received_counts[topic] = self.received_counts.get(topic, 0) + 1
            if kind == 'clock':
                stamp = self.stamp_value(msg.clock)
                if self.last_clock is not None and stamp < self.last_clock:
                    self.epoch += 1
                self.last_clock = stamp
                self.sim_s = stamp
                payload = {'clock_s': stamp}
                self._write(topic, kind, payload, source_s=stamp)
                return
            header = getattr(msg, 'header', None)
            source_s = self.stamp_value(header.stamp) if header and hasattr(header, 'stamp') else None
            frame = getattr(header, 'frame_id', None) if header else None
            child = getattr(msg, 'child_frame_id', None)
            sequence = getattr(header, 'seq', None) if header else None
            if kind == 'twist':
                payload = {'linear_x': msg.linear.x, 'linear_y': msg.linear.y, 'linear_z': msg.linear.z,
                           'angular_x': msg.angular.x, 'angular_y': msg.angular.y, 'angular_z': msg.angular.z}
            elif kind == 'odometry':
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                lv, av = msg.twist.twist.linear, msg.twist.twist.angular
                payload = {'position': [p.x, p.y, p.z], 'quaternion_xyzw': [q.x, q.y, q.z, q.w],
                           'linear': [lv.x, lv.y, lv.z], 'angular': [av.x, av.y, av.z],
                           'twist_frame_id': child}
            elif kind == 'imu':
                q, g, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
                payload = {'orientation_xyzw': [q.x, q.y, q.z, q.w], 'angular_velocity': [g.x, g.y, g.z],
                           'linear_acceleration': [a.x, a.y, a.z]}
                payload['roll_pitch_rad'] = [
                    math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),
                    math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))]
            elif kind == 'joint_state':
                if source_s is not None and self.last_joint_source is not None and source_s-self.last_joint_source < self.joint_period-1e-5:
                    self.decimated_counts[topic] = self.decimated_counts.get(topic, 0) + 1
                    return
                self.last_joint_source = source_s
                names = list(msg.name)
                wheels = {name: msg.velocity[i] for i, name in enumerate(names)
                          if name.endswith('_foot_joint') and i < len(msg.velocity)}
                payload = {'name': names, 'position': list(msg.position), 'velocity': list(msg.velocity),
                           'effort': list(msg.effort), 'wheel_velocity_by_name_radps': wheels}
            elif kind == 'float64_multi_array':
                payload = {'data': list(msg.data)}
            else:
                payload = {'transforms': [self._transform(t) for t in msg.transforms]}
            self._write(topic, kind, payload, source_s=source_s, frame_id=frame,
                        child_frame_id=child, source_sequence=sequence,
                        header_stamp=self.stamp_parts(header.stamp) if header and hasattr(header, 'stamp') else None)

        @staticmethod
        def _transform(msg: TransformStamped):
            t, r = msg.transform.translation, msg.transform.rotation
            return {'stamp': {'sec': int(msg.header.stamp.sec), 'nanosec': int(msg.header.stamp.nanosec)},
                    'frame_id': msg.header.frame_id, 'child_frame_id': msg.child_frame_id,
                    'translation': [t.x, t.y, t.z], 'quaternion_xyzw': [r.x, r.y, r.z, r.w]}

        def _write(self, topic, kind, payload, *, source_s=None, frame_id=None,
                   child_frame_id=None, source_sequence=None, header_stamp=None):
            bad = []
            data = _finite_payload(payload, bad)
            self.order += 1
            now = time.monotonic()
            event = {'order': self.order, 'topic': topic, 'type': kind,
                     'sim_s': self.sim_s, 'source_s': source_s, 'wall_s': now,
                     'epoch': self.epoch, 'frame_id': frame_id, 'child_frame_id': child_frame_id,
                     'header_stamp': header_stamp, 'source_sequence': source_sequence, 'data': data,
                     'valid': not bad, 'invalid_fields': ['nonfinite'] if bad else []}
            self.stream.write(json.dumps(event, allow_nan=False, separators=(',', ':')) + '\n')
            if self.order % 100 == 0:
                self.stream.flush()
            self.counts[topic] = self.counts.get(topic, 0) + 1
            self.last_received[topic] = {'sim_s': self.sim_s, 'source_s': source_s, 'wall_s': now}
            if bad:
                self.invalid_counts[topic] = self.invalid_counts.get(topic, 0) + 1

        def record_request(self, vx, phase, elapsed_s):
            msg = Twist()
            msg.linear.x = float(vx)
            self._write('/harness/requested', 'twist', {'linear_x': msg.linear.x, 'linear_y': 0., 'angular_z': 0.})
            self.publisher.publish(msg)

        def health(self):
            return {'captured_counts': dict(self.counts),'received_counts':dict(self.received_counts),
                    'last_received': dict(self.last_received),'invalid_counts': dict(self.invalid_counts),
                    'decimated_counts':dict(self.decimated_counts)}

        def close(self):
            self.stream.flush()
            self.stream.close()
            return self.health()

    return VelocityStepRecorder(output_dir)
