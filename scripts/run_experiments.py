#!/usr/bin/python3
"""Run isolated simulation cases serially and retain successes and failures."""
import argparse
import json
import hashlib
import os
import signal
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profiles',nargs='+',choices=['normal','mild','medium','severe'],default=['normal','mild','medium','severe'])
    p.add_argument('--seeds',nargs='+',type=int,default=[0])
    p.add_argument('--goal',nargs=2,type=float,default=[1.,0.])
    p.add_argument('--timeout',type=float,default=45.)
    p.add_argument('--output-dir',default='course_results/experiments')
    p.add_argument('--ros-domain-id',type=int,default=71)
    p.add_argument('--scene',choices=['confined','warehouse'],default='confined')
    p.add_argument('--locomotion-backend', choices=['legacy', 'rl_sar'], default='rl_sar')
    p.add_argument('--pretrained-directory', default=str(Path(__file__).resolve().parents[1] / 'third_party/b2w_rl_sar'))
    a=p.parse_args()
    root=Path(__file__).resolve().parents[1]
    run=Path(a.output_dir).resolve()/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    run.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_DOMAIN_ID']=str(a.ros_domain_id)
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    dds = 'fastdds_local.xml' if os.environ['ROS_LOCALHOST_ONLY'] == '1' else 'fastdds.xml'
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(root/'course_sim/config'/dds)
    manifest=vars(a)|{'scan_model':'instantaneous_snapshot','horizontal_samples':2000,
                     'start_wait_wall_s':20.,'physics_rng_seeded':False,
                     'ros_localhost_only': os.environ['ROS_LOCALHOST_ONLY'], 'dds_profile': dds,
                     'map_datum': 'measured_floor'}
    manifest.update(lidar_self_returns='excluded_by_visibility_mask', lidar_self_occlusion_simulated=False)
    if a.locomotion_backend == 'rl_sar':
        bundle = Path(a.pretrained_directory).resolve()
        manifest['locomotion_source'] = json.loads((bundle / 'POLICY_SOURCE.json').read_text())
        manifest['policy_sha256'] = hashlib.sha256((bundle / 'policy/b2w/robot_lab/policy.onnx').read_bytes()).hexdigest()
    (run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    results=[]
    for profile in a.profiles:
        for seed in a.seeds:
            case=run/f'{profile}-seed{seed}'
            case.mkdir()
            env=os.environ.copy()
            env['IGN_PARTITION']='course-'+run.name+'-'+case.name
            env['OPENBLAS_NUM_THREADS']='1'
            log=(case/'runtime.log').open('w')
            process=subprocess.Popen([str(root/'scripts/run_course.sh'),'mode:=navigation',
                f'scene:={a.scene}',
                f'locomotion_backend:={a.locomotion_backend}', f'pretrained_directory:={a.pretrained_directory}',
                f'fault_profile:={profile}',f'seed:={seed}',f'timeout_s:={a.timeout}',
                f'results_directory:={case}'],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,
                start_new_session=True)
            rclpy.init()
            node=Node('course_batch_goal')
            state={'odom':None,'map':False}
            def odom(m): state['odom']=m
            def grid(m): state['map']=bool(m.data)
            node.create_subscription(Odometry,'/slam/odom',odom,qos_profile_sensor_data)
            node.create_subscription(OccupancyGrid,'/map',grid,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            pub=node.create_publisher(PoseStamped,'/goal_pose',10)
            action=ActionClient(node,NavigateToPose,'/navigate_to_pose')
            start=time.monotonic();sent=None
            try:
                while process.poll() is None and time.monotonic()-start<max(70.,a.timeout*3):
                    rclpy.spin_once(node,timeout_sec=.1)
                    if sent is None and state['odom'] is not None and state['map'] and action.server_is_ready() and time.monotonic()-start>26:
                        goal=PoseStamped()
                        goal.header.frame_id='map'
                        goal.header.stamp=state['odom'].header.stamp
                        goal.pose.position.x,goal.pose.position.y=a.goal
                        goal.pose.orientation.w=1.
                        pub.publish(goal)
                        sent=time.monotonic()
                    if list(case.glob('20*.json')):
                        break
            finally:
                if process.poll() is None:
                    os.kill(process.pid,signal.SIGINT)
                    try: process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGTERM)
                        process.wait(timeout=10)
                node.destroy_node()
                rclpy.shutdown()
                log.close()
            files=sorted(case.glob('20*.json'))
            record=json.loads(files[-1].read_text()) if files else {
                'success':False,'end_reason':'startup_or_runtime_failure','collision_count':None,
                'path_length_m':None,'time_to_goal_s':None,'planning_latency_ms':None,
                'missing_reason':'No goal metrics produced; inspect runtime.log'}
            record.update(fault_profile=profile,seed=seed,case_directory=str(case),goal_sent=sent is not None)
            results.append(record)
            (run/'summary.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n')
            print(json.dumps(record),flush=True)
    print(f'Experiment results: {run}')


if __name__=='__main__':
    main()
