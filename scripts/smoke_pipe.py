#!/usr/bin/python3
"""Bounded stationary pipe smoke. Never publishes commands or navigation goals."""
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


def owned_processes(token):
    """PID + kernel start time identify surviving descendants even after launch exits."""
    found={}
    for path in Path('/proc').glob('[0-9]*'):
        try:
            env=(path/'environ').read_bytes().split(b'\0')
            if ('B2W_PIPE_SMOKE_TOKEN='+token).encode() not in env: continue
            stat=(path/'stat').read_text().rsplit(')',1)[1].split()
            if stat[0]=='Z': continue
            found[int(path.name)]=stat[19]
        except (OSError,IndexError): pass
    return found


def cleanup(token, process):
    snapshots=[]
    for sig,seconds in [(signal.SIGINT,8),(signal.SIGTERM,4),(signal.SIGKILL,2)]:
        targets=owned_processes(token); snapshots.append(dict(signal=sig.name,pids=list(targets)))
        for pid,start in targets.items():
            if owned_processes(token).get(pid)==start:
                try: os.kill(pid,sig)
                except ProcessLookupError: pass
        deadline=time.monotonic()+seconds
        while owned_processes(token) and time.monotonic()<deadline:
            process.poll();time.sleep(.1)
        if not owned_processes(token): break
    process.poll()
    return dict(signals=snapshots,remaining=owned_processes(token))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--domain',type=int,default=92)
    p.add_argument('--wall-timeout',type=float,default=55.)
    p.add_argument('--output',type=Path,default=Path('course_results/pipe_v0/smoke.json'))
    args=p.parse_args()
    if not 0 <= args.domain <= 232 or not 15 <= args.wall_timeout <= 90: p.error('domain 0..232 and timeout 15..90 s required')
    root=Path(__file__).resolve().parents[1]
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(root/'course_sim/config/fastdds_local.xml'))
    token=uuid.uuid4().hex
    env=os.environ.copy();env.update(B2W_PIPE_SMOKE_TOKEN=token,IGN_PARTITION='b2w-pipe-smoke-'+token)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    result=dict(passed=False,domain=args.domain,ign_partition=env['IGN_PARTITION'],
        world=str(root/'course_sim/worlds/pipe_v0.world'),
        world_sha256=hashlib.sha256((root/'course_sim/worlds/pipe_v0.world').read_bytes()).hexdigest(),
        scope='stationary sim only; no command/goal publisher; no SLAM/Nav2 validation',counts={})
    process=None;node=None;initialized=False
    with args.output.with_suffix('.log').open('w') as log:
        try:
            import numpy as np
            import rclpy
            from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
            from rosgraph_msgs.msg import Clock
            from sensor_msgs.msg import PointCloud2, Imu, JointState, Image
            from nav_msgs.msg import Odometry
            from geometry_msgs.msg import Twist
            from tf2_msgs.msg import TFMessage
            rclpy.init();initialized=True;node=rclpy.create_node('pipe_stationary_probe')
            # Do not start if a simulation/controller is already discoverable in this domain.
            until=time.monotonic()+1.5
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
            if any(node.get_publishers_info_by_topic(t) for t in ['/clock','/joint_torques','/b2w/robot_description']):
                raise RuntimeError('domain already has simulation publishers; choose an unused domain')
            samples={}; stamps={}; reversals=[]; frames={}; bodies=[]; cloud={}; commands=[]; last_wall={}
            def callback(topic,m):
                result['counts'][topic]=result['counts'].get(topic,0)+1
                last_wall[topic]=time.monotonic()
                if isinstance(m,Clock):stamp=m.clock
                elif hasattr(m,'header'):stamp=m.header.stamp
                else:stamp=None
                if stamp is not None:
                    t=stamp.sec+stamp.nanosec*1e-9
                    if t<stamps.get(topic,-1):reversals.append(topic)
                    stamps[topic]=t
                if isinstance(m,TFMessage):
                    for tf in m.transforms:frames[tf.child_frame_id]=tf.header.frame_id
                elif isinstance(m,Odometry):
                    pose=m.pose.pose;q=pose.orientation
                    bodies.append(dict(t=t,xyz=[pose.position.x,pose.position.y,pose.position.z],
                        tilt_deg=math.degrees(math.acos(max(-1,min(1,1-2*(q.x*q.x+q.y*q.y)))))))
                elif isinstance(m,PointCloud2):
                    fields={f.name:f for f in m.fields}
                    if not all(n in fields and fields[n].datatype==7 for n in ['x','y','z']):return
                    dtype=np.dtype(dict(names=['x','y','z'],formats=[('>' if m.is_bigendian else '<')+'f4']*3,
                                        offsets=[fields[n].offset for n in ['x','y','z']],itemsize=m.point_step))
                    points=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))
                    xyz=np.stack([points[n].ravel() for n in ['x','y','z']],axis=1)
                    finite=np.isfinite(xyz).all(axis=1);ranges=np.linalg.norm(xyz[finite],axis=1)
                    cloud.update(frame=m.header.frame_id,width=m.width,height=m.height,fields=list(fields),
                        finite_points=int(finite.sum()),min_range_m=float(ranges.min()) if len(ranges) else None,
                        max_range_m=float(ranges.max()) if len(ranges) else None)
                elif isinstance(m,Twist):
                    commands.append([m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z])
                elif isinstance(m,JointState):samples['joints']=dict(zip(m.name,m.position))
                elif isinstance(m,Image) and m.width and m.height:
                    samples['camera']=dict(width=m.width,height=m.height,encoding=m.encoding)
                    # Capture a real sensor frame once, not a synthetic scene illustration.
                    if 'camera_saved' not in samples and m.encoding in ['rgb8','bgr8']:
                        from PIL import Image as PILImage
                        frame=np.ndarray((m.height,m.width,3),dtype=np.uint8,buffer=m.data,strides=(m.step,3,1))
                        if m.encoding=='bgr8': frame=frame[:,:,::-1]
                        path=args.output.with_suffix('.camera.png');PILImage.fromarray(frame).save(path)
                        samples['camera_saved']=str(path.resolve())
            topics=[('/clock',Clock),('/pointcloud',PointCloud2),('/imu',Imu),('/joint_states',JointState),
                ('/sim/ground_truth/odom',Odometry),('/tf',TFMessage),('/zed/zed_node/depth/depth_registered',Image),
                ('/path_manager/path_manager_ros/nav_vel',Twist)]
            for topic,typ in topics:node.create_subscription(typ,topic,lambda m,t=topic:callback(t,m),qos_profile_sensor_data)
            node.create_subscription(TFMessage,'/tf_static',lambda m:callback('/tf_static',m),
                                     QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            process=subprocess.Popen([str(root/'scripts/run_pipe.sh'),'gui:=false'],cwd=root,env=env,
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            result['launch_pid']=process.pid;result['started_owned_pids']=owned_processes(token)
            start=time.monotonic()
            while time.monotonic()-start<args.wall_timeout and process.poll() is None:
                rclpy.spin_once(node,timeout_sec=.02)
                if bodies and bodies[-1]['t']>=16 and all(result['counts'].get(t,0)>=5 for t in ['/clock','/pointcloud','/imu','/joint_states']):break
            result['wall_duration_s']=time.monotonic()-start
            settled=[b for b in bodies if b['t']>=8]
            required=['/clock','/pointcloud','/imu','/joint_states','/sim/ground_truth/odom','/tf','/tf_static']
            result.update(last_stamps=stamps,time_reversals=reversals,tf_parents=frames,pointcloud=cloud,
                final_joints=samples.get('joints'),camera=samples.get('camera'),screenshot=samples.get('camera_saved'),
                nonzero_commands=sum(any(abs(v)>1e-9 for v in c) for c in commands),
                publishers={t:[n.node_name for n in node.get_publishers_info_by_topic(t)] for t in ['/clock','/joint_states','/joint_torques','/b2w/robot_description']})
            if settled:
                result['settled']=dict(start=settled[0],end=settled[-1],samples=len(settled),
                    sim_duration_s=settled[-1]['t']-settled[0]['t'],min_height_m=min(b['xyz'][2] for b in settled),
                    max_tilt_deg=max(b['tilt_deg'] for b in settled),
                    max_xy_drift_m=max(math.dist(b['xyz'][:2],settled[0]['xyz'][:2]) for b in settled))
            stable=result.get('settled',{})
            result['checks']=dict(process_alive=process.poll() is None,
                messages=all(result['counts'].get(t,0)>=1 for t in required),
                fresh=all(time.monotonic()-last_wall.get(t,0)<3 for t in required if t!='/tf_static'),
                monotonic=not reversals,clock_advanced=stamps.get('/clock',0)>=15,
                sensor_frame=cloud.get('frame')=='lidar_link',cloud_returns=cloud.get('finite_points',0)>100,
                sensor_tf=frames.get('lidar_link')=='base_link' and frames.get('imu_link')=='base_link',
                unique_publishers=all(len(v)==1 for v in result['publishers'].values()),
                standing=stable.get('min_height_m',0)>.4 and stable.get('max_tilt_deg',180)<25,
                idle_drift=stable.get('max_xy_drift_m',100)<.15,
                settled_duration=stable.get('sim_duration_s',0)>=5,zero_commands=result['nonzero_commands']==0)
            result['passed']=all(result['checks'].values())
        except Exception as error:
            result['error']=repr(error)
        finally:
            if process is not None:
                result['cleanup']=cleanup(token,process)
                result['passed'] &= not result['cleanup']['remaining']
            if node is not None:node.destroy_node()
            if initialized:rclpy.shutdown()
            args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
