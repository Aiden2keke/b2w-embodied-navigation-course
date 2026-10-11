#!/usr/bin/python3
import os,time,json,numpy as np
import argparse
from pathlib import Path
parser=argparse.ArgumentParser(description='Check an idle course session for mixed clocks/TF and physical instability')
parser.add_argument('--domain',type=int,default=91)
parser.add_argument('--duration',type=float,default=10.)
parser.add_argument('--output',type=Path,default=Path('/tmp/b2w-initialization.json'))
args=parser.parse_args()
os.environ['ROS_DOMAIN_ID']=str(args.domain)
os.environ.setdefault('ROS_LOCALHOST_ONLY','1')
dds='fastdds_local.xml' if os.environ['ROS_LOCALHOST_ONLY']=='1' else 'fastdds.xml'
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(Path(__file__).resolve().parents[1]/'course_sim/config'/dds)
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
rclpy.init();n=rclpy.create_node('b2w_initialization_probe');s={};frames={};reversed_tf=[]
def rec(topic,m):
 v={}
 if isinstance(m,Clock):v['t']=m.clock.sec+m.clock.nanosec*1e-9
 elif hasattr(m,'header'):v['t']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if isinstance(m,Odometry):
  p=m.pose.pose.position;q=m.pose.pose.orientation;v.update(xyz=[p.x,p.y,p.z],tilt=float(np.degrees(np.arccos(np.clip(1-2*(q.x*q.x+q.y*q.y),-1,1)))))
 elif isinstance(m,Imu):v['gyro']=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]
 elif isinstance(m,Float64MultiArray):v['torque_max']=float(max(map(abs,m.data),default=0))
 elif isinstance(m,Twist):v['command']=[m.linear.x,m.angular.z]
 elif isinstance(m,TFMessage):
  for tf in m.transforms:
   t=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
   if tf.child_frame_id in frames and t<frames[tf.child_frame_id]:reversed_tf.append({'frame':tf.child_frame_id,'from':frames[tf.child_frame_id],'to':t})
   frames[tf.child_frame_id]=t
 s.setdefault(topic,[]).append(v)
for topic,typ in [('/clock',Clock),('/sim/body_state',Odometry),('/slam/odom',Odometry),('/sim/raw/imu',Imu),('/sim/raw/joint_states',JointState),('/joint_torques',Float64MultiArray),('/tf',TFMessage),('/path_manager/path_manager_ros/nav_vel',Twist)]:n.create_subscription(typ,topic,lambda m,t=topic:rec(t,m),qos_profile_sensor_data)
end=time.monotonic()+args.duration
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
r={'publishers':{t:[{'name':p.node_name,'gid':bytes(p.endpoint_gid).hex()} for p in n.get_publishers_info_by_topic(t)] for t in ['/clock','/tf','/joint_states','/sim/raw/imu','/joint_torques','/b2w/robot_description']},'counts':{t:len(v) for t,v in s.items()},'tf_time_reversals':len(reversed_tf),'tf_examples':reversed_tf[:6]}
for t,values in s.items():
 if values and 't' in values[0]:
  times=[v['t'] for v in values];r.setdefault('time',{})[t]={'start':times[0],'end':times[-1],'reversals':sum(b<a for a,b in zip(times,times[1:]))}
for t in ['/sim/body_state','/slam/odom']:
 a=s.get(t,[])
 if a:r[t]={'max_tilt_deg':max(v['tilt'] for v in a),'min_height':min(v['xyz'][2] for v in a),'drift':float(np.linalg.norm(np.array(a[-1]['xyz'][:2])-a[0]['xyz'][:2])),'last':a[-1]}
r['checks']={'body_feedback':len(s.get('/sim/body_state',[]))>=10,'body_stable':r.get('/sim/body_state',{}).get('max_tilt_deg',180)<25 and r.get('/sim/body_state',{}).get('min_height',0)>.4,'idle_drift_below_15cm':r.get('/sim/body_state',{}).get('drift',100)<.15,'one_clock':len(r['publishers']['/clock'])==1,'one_controller':len(r['publishers']['/joint_torques'])==1,'one_description':len(r['publishers']['/b2w/robot_description'])==1,'tf_monotonic':r['tf_time_reversals']==0,'sensor_clock_monotonic':all(t['reversals']==0 for t in r.get('time',{}).values())}
r['passed']=all(r['checks'].values())
args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));n.destroy_node();rclpy.shutdown();raise SystemExit(0 if r['passed'] else 1)
