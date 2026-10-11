#!/usr/bin/python3
"""Exercise the compiled rl_sar controller with reordered feedback and a ROS clock."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import yaml
import argparse


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=root/'docs/validation/2026-10-11-rl-sar-controller.json')
    args=parser.parse_args()
    os.environ['ROS_DOMAIN_ID']='88'
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(root/'course_sim/config/fastdds.xml')
    env=os.environ.copy()
    env['LD_LIBRARY_PATH']=str(root/'b2w_sim/b2w_controllers/third_party/onnxruntime/lib64')+':'+env.get('LD_LIBRARY_PATH','')
    cfg_path=root/'course_sim/config/rl_sar.yaml'
    cfg=yaml.safe_load(cfg_path.read_text())['b2w_rl_sar_controller']['ros__parameters']
    policy=root/'third_party/b2w_rl_sar/policy/b2w/robot_lab/policy.onnx'
    log=open('/tmp/b2w-rl-sar-controller-check.log','w')
    process=subprocess.Popen([str(root/'install/b2w_controllers/lib/b2w_controllers/b2w_rl_sar_controller'),
        '--ros-args','--params-file',str(cfg_path),'-p','policy_path:='+str(policy),'-p','stand_transition_s:=0.1',
        '-p','idle_hold:=false','-p','yaw_command_gain:=1.0',
        '-r','/sim/raw/joint_states:=/probe/joints','-r','/sim/raw/imu:=/probe/imu',
        '-r','/path_manager/path_manager_ros/nav_vel:=/probe/cmd'],env=env,stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();node=Node('b2w_rl_sar_runtime_check')
    clocks=node.create_publisher(Clock,'/clock',10)
    joints=node.create_publisher(JointState,'/probe/joints',10)
    odom=node.create_publisher(Imu,'/probe/imu',10)
    cmds=node.create_publisher(Twist,'/probe/cmd',10)
    obs=[];torques=[]
    node.create_subscription(Float64MultiArray,'/network_input_debug',lambda m:obs.append(list(m.data)),qos_profile_sensor_data)
    node.create_subscription(Float64MultiArray,'/joint_torques',lambda m:torques.append(list(m.data)),qos_profile_sensor_data)
    q=[v+.01*(i+1) for i,v in enumerate(cfg['default_dof_pos'])]
    state=JointState();state.name=cfg['joint_names'][::-1];state.position=q[::-1];state.velocity=[.02*(i+1) for i in range(16)][::-1]
    body=Imu();body.orientation.w=math.cos(.05);body.orientation.y=math.sin(.05)
    body.angular_velocity.x=.4;body.angular_velocity.y=.8;body.angular_velocity.z=1.2
    cmd=Twist();cmd.linear.x=.2;cmd.angular.z=.1
    sim=1.
    def run(duration,advance=True,feedback=True,commands=True):
        nonlocal sim
        end=time.monotonic()+duration
        while time.monotonic()<end:
            if advance:
                sim+=.005
                c=Clock();c.clock.sec=int(sim);c.clock.nanosec=int((sim%1)*1e9);clocks.publish(c)
                if feedback:
                    state.header.stamp=body.header.stamp=c.clock;joints.publish(state);odom.publish(body)
                if commands:cmds.publish(cmd)
            for _ in range(8):rclpy.spin_once(node,timeout_sec=0.)
            time.sleep(.01)
    try:
        run(4.)
        assert process.poll() is None,'Controller exited; inspect its log'
        obs.clear();torques.clear();t0=sim;run(2.)
        hz=len(torques)/(sim-t0);infer_hz=len(obs)/(sim-t0)
        assert 170<hz<230 and 40<infer_hz<60,(hz,infer_hz)
        x=np.array(obs[-1]);assert x.shape==(57,) and np.isfinite(x).all()
        assert np.allclose(x[:3],[.1,.2,.3])
        assert np.allclose(x[3:6],[math.sin(.1),0,-math.cos(.1)],atol=1e-6)
        assert np.allclose(x[6:9],[.2,0,.1])
        assert np.allclose(x[9:21],np.arange(1,13)*.01), 'Joint name mapping incorrect'
        assert np.allclose(x[21:25],0), 'Wheel positions must be masked'
        assert np.allclose(x[25:41],np.arange(1,17)*.001)
        assert np.isfinite(torques).all() and np.all(np.abs(torques)<=np.array(cfg['torque_limits'])+1e-6)
        # Reset to a time later than the initial start but before the last tick.
        # The old age<0 check misses this reset and accepts future-dated feedback.
        sim=2.0
        obs.clear();torques.clear();run(.3,feedback=False,commands=False)
        assert torques and np.allclose(torques,0),'Clock reset reused old feedback/torque'
        assert not obs,'Clock reset must wait for new joint and IMU feedback'
        obs.clear();run(.8,commands=False)
        assert obs and np.allclose(obs[0][41:57],0),'Previous policy action leaked across reset'
        assert np.allclose(obs[0][6:9],0),'Old command leaked across reset'
        run(.3,False);obs.clear();torques.clear();run(.5,False)
        assert not obs and not torques,'Updates while ROS clock paused'
        run(1.5,commands=False)
        assert np.allclose(obs[-1][6:9],0),'Stale speed request did not expire'
        run(.6,feedback=False,commands=False);torques.clear();run(.5,feedback=False,commands=False)
        assert torques and np.allclose(torques,0),'Missing feedback must produce zero torque'
        result={'success':True,'command_sim_hz':hz,'inference_sim_hz':infer_hz,'joint_name_mapping':True,'wheel_position_mask':True,'body_observation_scaling':True,'paused_updates':0,'stale_command_zeroed':True,'stale_feedback_zero_torque':True,'clock_reset_clears_feedback_actions_commands':True,'finite_limited_torques':True,'physical_balance_test':False}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result))
    finally:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=10)
        log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
