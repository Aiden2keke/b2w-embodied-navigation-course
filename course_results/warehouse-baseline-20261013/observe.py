import json,time,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from rclpy.action import ActionClient
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu,PointCloud2
from nav_msgs.msg import Odometry,OccupancyGrid,Path as NavPath
from geometry_msgs.msg import Twist,PoseStamped
from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from tf2_ros import Buffer,TransformListener
D=Path('/tmp/warehouse-baseline-20261013'); f=(D/'trace.jsonl').open('w')
rclpy.init(); n=Node('warehouse_baseline_observer'); tf=Buffer(); listener=TransformListener(tf,n)
start=time.monotonic(); sim=0.; last={}; counts={}; ages={}; lifecycle={}; pending={}; sent=None; ready_since=None; last_check=0.; outputs=[]; status=[]; valid_state=None; command_wall=None; last_cmd=None
names=['controller_server','smoother_server','planner_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother']
clients={x:n.create_client(GetState,'/'+x+'/get_state') for x in names}
nav=ActionClient(n,NavigateToPose,'/navigate_to_pose'); planner=ActionClient(n,ComputePathToPose,'/compute_path_to_pose'); pub=n.create_publisher(PoseStamped,'/goal_pose',10)
def write(kind,**data):
 f.write(json.dumps(dict(kind=kind,wall=time.monotonic(),sim=sim,**data))+'\n'); f.flush()
def clock(m):
 global sim
 sim=m.clock.sec+m.clock.nanosec*1e-9; counts['clock']=counts.get('clock',0)+1; last['clock']=time.monotonic()
n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)
def rec(key,m):
 global valid_state
 now=time.monotonic(); counts[key]=counts.get(key,0)+1; last[key]=now
 stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9; age=sim-stamp; ages.setdefault(key,[]).append(age)
 data=dict(stamp=stamp,age=age)
 if key=='odom':
  p=m.pose.pose.position;q=m.pose.pose.orientation;data['xyz']=[p.x,p.y,p.z]
  if 0<=age<=.5 and all(math.isfinite(v) for v in [p.x,p.y,p.z,q.x,q.y,q.z,q.w]) and q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w>.5:valid_state=now
 write(key,**data)
for key,topic,typ in [('cloud','/pointcloud',PointCloud2),('imu','/imu',Imu),('odom','/slam/odom',Odometry),('map_points','/slam/map_points',PointCloud2)]:n.create_subscription(typ,topic,lambda m,k=key:rec(k,m),qos_profile_sensor_data)
n.create_subscription(OccupancyGrid,'/map',lambda m:rec('map',m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
def velocity(key,m):
 global command_wall,last_cmd
 v=[m.linear.x,m.linear.y,m.angular.z];now=time.monotonic();data=dict(value=v)
 if key=='smoothed':command_wall=now;last_cmd=v
 if key=='final':
  try:
   t=tf.lookup_transform('map','base_link',Time()); age=sim-t.header.stamp.sec-t.header.stamp.nanosec*1e-9;data['tf_age']=age;data['tf_available']=True
  except Exception as e:data['tf_available']=False;data['tf_error']=str(e)
  data['valid_odom_wall_age']=now-valid_state if valid_state else None;data['command_wall_age']=now-command_wall if command_wall else None;data['smoothed_latest']=last_cmd
  if sent is not None:outputs.append(dict(wall=now,**data))
 write(key,**data)
for key,topic in [('raw','/course_navigation/cmd_vel_raw'),('smoothed','/course_navigation/cmd_vel'),('final','/path_manager/path_manager_ros/nav_vel')]:n.create_subscription(Twist,topic,lambda m,k=key:velocity(k,m),10)
def plan(m):write('global_plan',poses=len(m.poses),xy=[[p.pose.position.x,p.pose.position.y] for p in m.poses])
n.create_subscription(NavPath,'/plan',plan,10)
def statuses(key,m):
 vals=[dict(uuid=bytes(s.goal_info.goal_id.uuid).hex(),status=s.status) for s in m.status_list];status.append(dict(key=key,wall=time.monotonic(),values=vals));write(key,values=vals)
for key,topic in [('nav_status','/navigate_to_pose/_action/status'),('planner_status','/compute_path_to_pose/_action/status')]:n.create_subscription(GoalStatusArray,topic,lambda m,k=key:statuses(k,m),QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
try:
 while time.monotonic()-start<800:
  rclpy.spin_once(n,timeout_sec=.01);now=time.monotonic()
  if now-last_check<.5:continue
  last_check=now
  for name,c in clients.items():
   if name in pending and pending[name].done():
    try:lifecycle[name]=pending[name].result().current_state.id
    except Exception:lifecycle[name]=None
    del pending[name]
   if name not in pending and c.service_is_ready():pending[name]=c.call_async(GetState.Request())
  tf_ok=True;tf_age=None
  try:
   t=tf.lookup_transform('map','base_link',Time());tf_age=sim-t.header.stamp.sec-t.header.stamp.nanosec*1e-9
   tf.lookup_transform('map','odom',Time());tf.lookup_transform('odom','base_link',Time())
  except Exception:tf_ok=False
  checks={'clock':counts.get('clock',0)>5 and now-last.get('clock',0)<1,'cloud':counts.get('cloud',0)>5 and now-last.get('cloud',0)<1,'imu':counts.get('imu',0)>10 and now-last.get('imu',0)<1,'odom':counts.get('odom',0)>10 and now-last.get('odom',0)<1 and ages.get('odom',[999])[-1]<=.5,'map_points':counts.get('map_points',0)>1 and now-last.get('map_points',0)<6,'map':counts.get('map',0)>1 and now-last.get('map',0)<6,'lifecycle':all(lifecycle.get(x)==3 for x in names),'nav_action':nav.server_is_ready(),'planner_action':planner.server_is_ready(),'tf':tf_ok}
  write('readiness',checks=checks,lifecycle=lifecycle,tf_age=tf_age)
  if all(checks.values()):
   if ready_since is None:ready_since=now
  else:ready_since=None
  if sent is None and ready_since and now-ready_since>=5:
   g=PoseStamped();g.header.frame_id='map';g.pose.position.x=1.;g.pose.position.y=-1.;g.pose.orientation.w=1.;pub.publish(g);sent=now;write('goal_sent',goal=[1,-1]);print('READINESS PASS; sent exactly one goal (1,-1)',flush=True)
  if sent is not None and list((D/'metrics').glob('*.json')):break
  if int(now-start)%30==0:print(json.dumps(dict(elapsed=round(now-start),sim=sim,checks=checks,counts=counts)),flush=True)
finally:
 def dist(v):return dict(n=len(v),p50=float(np.percentile(v,50)),p95=float(np.percentile(v,95)),max=float(max(v))) if v else None
 tfages=[o['tf_age'] for o in outputs if 'tf_age' in o]
 unavailable=[o for o in outputs if not o.get('tf_available')]
 summary=dict(goal_sent=sent is not None,readiness=lifecycle,counts=counts,ages={k:dist(v) for k,v in ages.items()},tf_age_during_goal=dist(tfages),final_command_cycles=len(outputs),tf_unavailable_observer_cycles=len(unavailable),statuses=status,measurement_limit='External observer TF buffer and callback timing; causal gate counts are not internal instrumentation. Loaded adapter has no TF-age threshold gate; it checks TF availability only.')
 (D/'observer-summary.json').write_text(json.dumps(summary,indent=2));write('observer_finished');f.close();n.destroy_node();rclpy.shutdown();print(json.dumps(summary),flush=True)
