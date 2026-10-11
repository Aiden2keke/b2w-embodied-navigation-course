#!/usr/bin/python3
"""Check real Fortress standing, motion and stopping; retain failure measurements."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--locomotion-backend', choices=['legacy', 'rl_sar'], default='rl_sar')
    parser.add_argument('--scene', choices=['confined', 'warehouse'], default='confined')
    parser.add_argument('--horizontal-samples', type=int, default=2000)
    parser.add_argument('--ros-domain-id', type=int, default=87)
    parser.add_argument('--output', type=Path, default=Path('course_results/locomotion.json'))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    os.environ['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    os.environ['IGN_PARTITION'] = 'b2w-locomotion-check-' + uuid.uuid4().hex
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    dds = 'fastdds_local.xml' if os.environ['ROS_LOCALHOST_ONLY'] == '1' else 'fastdds.xml'
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(root/'course_sim/config'/dds)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    samples = []; state = {'joints': None}
    rclpy.init(); node = Node('b2w_locomotion_physics_check')
    def odom(message):
        pose = message.pose.pose; q = pose.orientation
        samples.append({'t': message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                        'xyz': [pose.position.x, pose.position.y, pose.position.z],
                        'tilt': math.degrees(math.acos(np.clip(1-2*(q.x*q.x+q.y*q.y), -1, 1))),
                        'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))})
    node.create_subscription(Odometry, '/sim/body_state', odom, qos_profile_sensor_data)
    node.create_subscription(JointState, '/joint_states', lambda m: state.update(joints=m), qos_profile_sensor_data)
    publisher = node.create_publisher(Twist, '/path_manager/path_manager_ros/nav_vel', 10)
    result = {'backend': args.locomotion_backend, 'scene': args.scene,
              'horizontal_samples': args.horizontal_samples, 'phases': [], 'success': False,
              'scope': '51 seconds of flat-ground motion; does not verify rough terrain or navigation'}
    if args.locomotion_backend == 'rl_sar':
        config = root/'course_sim/config/rl_sar.yaml'
        result['controller_parameters'] = yaml.safe_load(config.read_text())['b2w_rl_sar_controller']['ros__parameters']
        result['config_sha256'] = hashlib.sha256(config.read_bytes()).hexdigest()
        result['policy_sha256'] = hashlib.sha256((root/'third_party/b2w_rl_sar/policy/b2w/robot_lab/policy.onnx').read_bytes()).hexdigest()
    with args.output.with_suffix('.log').open('w') as log:
        process = subprocess.Popen([str(root/'scripts/run_course.sh'), 'mode:=sim',
            f'locomotion_backend:={args.locomotion_backend}', f'scene:={args.scene}',
            f'horizontal_samples:={args.horizontal_samples}', 'gui:=false', 'rviz:=false'],
            cwd=root, env=os.environ.copy(), stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wall = time.monotonic()
            while time.monotonic()-wall < 60 and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=.05)
                if samples and samples[-1]['t'] > 8 and state['joints'] is not None:
                    break
            if not samples or samples[-1]['t'] <= 8 or state['joints'] is None:
                raise RuntimeError('Simulation feedback did not become ready')
            if samples[-1]['tilt'] > 25 or samples[-1]['xyz'][2] < .4:
                raise RuntimeError('Robot already unstable after startup')
            for name, duration, command in [('stand', 15, (0., 0.)), ('forward', 8, (.2, 0.)),
                    ('stop_after_forward', 10, (0., 0.)), ('turn', 8, (0., .25)),
                    ('stop_after_turn', 10, (0., 0.))]:
                first = samples[-1]; index = len(samples); wall = time.monotonic()
                while samples[-1]['t']-first['t'] < duration and process.poll() is None and time.monotonic()-wall < duration*5:
                    message = Twist(); message.linear.x, message.angular.z = command
                    publisher.publish(message); rclpy.spin_once(node, timeout_sec=.03)
                    if samples[-1]['tilt'] > 60 or samples[-1]['xyz'][2] < .3:
                        break
                group = samples[index:]
                if not group:
                    raise RuntimeError('Simulation state stopped updating')
                last = group[-1]
                drift = [float(np.linalg.norm(np.array(m['xyz'][:2])-first['xyz'][:2])) for m in group]
                phase = {'name': name, 'command_vx_yaw': command, 'requested_duration_s': duration,
                         'sim_duration_s': last['t']-first['t'], 'displacement_m': drift[-1],
                         'max_displacement_from_start_m': max(drift),
                         'max_tilt_deg': max(m['tilt'] for m in group),
                         'min_base_height_m': min(m['xyz'][2] for m in group),
                         'start_xyz': first['xyz'], 'end_xyz': last['xyz'],
                         'yaw_delta_rad': float(np.unwrap([first['yaw']]+[m['yaw'] for m in group])[-1]-first['yaw'])}
                if command == (0., 0.):
                    response = max(drift) < .15
                elif command[0]:
                    response = abs(drift[-1]-command[0]*duration) <= command[0]*duration*.5
                else:
                    response = abs(phase['yaw_delta_rad']-command[1]*duration) <= command[1]*duration*.5
                phase['passed'] = phase['sim_duration_s'] >= duration*.95 and phase['max_tilt_deg'] < 25 and phase['min_base_height_m'] > .4 and response
                result['phases'].append(phase); print(json.dumps(phase), flush=True)
                if phase['max_tilt_deg'] > 60 or phase['min_base_height_m'] < .3:
                    break
            joints = state['joints']; result['final_joints'] = dict(zip(joints.name, joints.position))
            result['success'] = len(result['phases']) == 5 and all(m['passed'] for m in result['phases'])
        except Exception as error:
            result['error'] = str(error)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=10)
            node.destroy_node(); rclpy.shutdown()
            args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(f'Physical check {"passed" if result["success"] else "failed"}: {args.output}')
    raise SystemExit(0 if result['success'] else 1)


if __name__ == '__main__':
    main()
