#!/usr/bin/python3
"""Check the real controller with a controlled ROS clock and named joint feedback."""
import json
import os
import signal
import subprocess
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


def main():
    root = Path(__file__).resolve().parents[1]
    os.environ['ROS_DOMAIN_ID'] = '83'
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(root / 'course_sim/config/fastdds.xml')
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = str(root / 'b2w_sim/b2w_controllers/third_party/onnxruntime/lib64') + ':' + env.get('LD_LIBRARY_PATH', '')
    command = [str(root / 'install/b2w_controllers/lib/b2w_controllers/b2w_controllers'),
        '--ros-args', '-p', 'use_sim_time:=true',
        '-p', 'topics.odometry:=/probe/odom', '-p', 'topics.joint_state:=/probe/joints',
        '-p', 'topics.cmd_vel:=/probe/cmd']
    log = open('/tmp/course-locomotion-probe.log', 'w')
    process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = Node('locomotion_runtime_check')
    clock = node.create_publisher(Clock, '/clock', 10)
    odom = node.create_publisher(Odometry, '/probe/odom', 10)
    joints = node.create_publisher(JointState, '/probe/joints', 10)
    velocity = node.create_publisher(Twist, '/probe/cmd', 10)
    samples, observations = [], []
    node.create_subscription(Float64MultiArray, '/joint_commands', lambda m: samples.append(list(m.data)), qos_profile_sensor_data)
    node.create_subscription(Float64MultiArray, '/network_input_debug', lambda m: observations.append(list(m.data)), qos_profile_sensor_data)
    sim = 1.
    state = JointState()
    state.name = [leg + '_' + joint + '_joint' for leg in ['FL', 'FR', 'RL', 'RR'] for joint in ['hip', 'thigh', 'calf', 'foot']]
    state.position = [0., 0.4, -1.3, 0.] * 4
    state.velocity = [0.] * 16
    body = Odometry()
    body.pose.pose.orientation.w = 1.
    cmd = Twist()

    def run(duration, advance):
        nonlocal sim
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if advance:
                sim += 0.005  # Deliberately slower than wall time.
                msg = Clock()
                msg.clock.sec = int(sim)
                msg.clock.nanosec = int((sim % 1) * 1e9)
                clock.publish(msg)
                body.header.stamp = state.header.stamp = msg.clock
                odom.publish(body)
                joints.publish(state)
                velocity.publish(cmd)
            for _ in range(8):
                rclpy.spin_once(node, timeout_sec=0.)
            time.sleep(0.01)

    try:
        run(6., True)
        assert process.poll() is None, 'Controller exited; inspect /tmp/course-locomotion-probe.log'
        samples.clear(); observations.clear(); start = sim
        run(2., True)
        output = np.array(samples)
        assert len(output) > 100, 'No controller output'
        assert np.isfinite(output).all(), 'Non-finite policy commands'
        hz = len(samples) / (sim - start)
        inference_hz = len(observations) / (sim - start)
        assert 170 < hz < 230, f'Command frequency must follow simulation time: {hz}'
        assert 40 < inference_hz < 60, f'Inference frequency must follow simulation time: {inference_hz}'
        run(.5, False)  # Drain messages already in DDS queues.
        samples.clear(); observations.clear()
        run(1., False)
        assert not samples and not observations, 'Controller updates while simulation clock is paused'
        cmd.linear.x = .2
        run(2., True)
        assert samples and np.isfinite(samples).all(), 'No finite moving-policy output'
        assert abs(observations[-1][9] - .2) < 1e-6, 'Velocity request missing from policy observation'
        cmd.linear.x = 0.
        run(6., True)
        assert abs(observations[-1][9]) < 1e-6, 'Zero request missing from policy observation'
        result = {'success': True, 'command_sim_hz': hz, 'inference_sim_hz': inference_hz,
                  'paused_updates': 0, 'finite_policy_output': True,
                  'velocity_request_observed': True, 'zero_request_observed': True}
        Path('/tmp/course-locomotion-runtime.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result))
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
