#!/usr/bin/python3
"""Read-only ROS presentation: waypoint markers and fresh map/world correspondence.

Publishes visualization markers ONLY. Ground truth is used for camera/evaluation,
never public TF, localization, Nav2 inputs or robot commands.
"""
import argparse,json,math,time
from collections import deque
from pathlib import Path
import numpy as np
from g1_g5_media import write,ROOT


def rviz_config(output,width=960,height=800,source=None):
    import yaml
    cfg=yaml.safe_load(Path(source or ROOT/'course_bringup/rviz/course.rviz').read_text());route=yaml.safe_load((ROOT/'course_navigation/config/warehouse_route.yaml').read_text())
    xy=np.array([[0,0]]+[g['position'] for g in route['goals']]);span=np.ptp(xy,axis=0)+4;center=(xy.max(0)+xy.min(0))/2
    vm=cfg['Visualization Manager'];vm['Tools']=[x for x in vm['Tools'] if 'SetGoal' not in x['Class']]
    vm['Displays']=[d for d in vm['Displays'] if d.get('Name')!='G1-G5 progression']
    vm['Displays'].append({'Class':'rviz_default_plugins/MarkerArray','Name':'G1-G5 progression','Enabled':True,'Marker Topic':{'Value':'/g1_g5/route_markers','Durability Policy':'Transient Local','Reliability Policy':'Reliable'}})
    if vm.get('Views',{}).get('Current',{}).get('Class')!='rviz_default_plugins/TopDownOrtho':vm['Views']['Current']={'Class':'rviz_default_plugins/TopDownOrtho','Target Frame':'map','Scale':float(min(width*.62/span[0],height*.75/span[1])),'X':float(center[0]),'Y':float(center[1]),'Angle':0.}
    Path(output).write_text(yaml.safe_dump(cfg));return route

def pair_correspondence(tf,history,now,wall):
    tm=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
    if not 0<=now-tm<=.5:return None
    candidates=[]
    for receipt,msg in history:
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if 0<=wall-receipt<=.5 and abs(stamp-tm)<=.02:candidates.append((abs(stamp-tm),stamp,msg))
    if not candidates:return None
    _,stamp,msg=min(candidates,key=lambda row:row[0]);t=tf.transform;p=msg.pose.pose.position;v=t.translation
    def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    a=math.atan2(math.sin(yaw(msg.pose.pose.orientation)-yaw(t.rotation)),math.cos(yaw(msg.pose.pose.orientation)-yaw(t.rotation)));c,s=math.cos(a),math.sin(a)
    return dict(world_from_map_se2=[p.x-c*v.x+s*v.y,p.y-s*v.x-c*v.y,a],world_ground_z=p.z-v.z,map_base=[v.x,v.y,yaw(t.rotation)],truth_world=[p.x,p.y,yaw(msg.pose.pose.orientation)],map_stamp=tm,truth_stamp=stamp)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    import rclpy,yaml
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from nav_msgs.msg import Odometry
    from visualization_msgs.msg import Marker,MarkerArray
    import tf2_ros
    from rclpy.time import Time
    from builtin_interfaces.msg import Duration
    rclpy.init();n=rclpy.create_node('g1_g5_readonly_presentation',parameter_overrides=[Parameter('use_sim_time',value=True)]);buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer,n)
    route=yaml.safe_load((ROOT/'course_navigation/config/warehouse_route.yaml').read_text());truth=deque(maxlen=256);samples=[]
    def receive(m):
        truth.append((time.monotonic(),m))
    n.create_subscription(Odometry,'/sim/ground_truth/odom',receive,__import__('rclpy.qos',fromlist=['qos_profile_sensor_data']).qos_profile_sensor_data)
    pub=n.create_publisher(MarkerArray,'/g1_g5/route_markers',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    start=time.monotonic();last=0.;colors=[(1.,.1,.8),(.1,1.,.8),(.8,.1,1.),(1.,.8,.1),(.3,1.,.1)]
    try:
        while rclpy.ok():
            rclpy.spin_once(n,timeout_sec=.02)
            if time.monotonic()-last<.2:continue
            last=time.monotonic();arr=MarkerArray();stamp=n.get_clock().now().to_msg()
            for i,g in enumerate(route['goals']):
                for typ,z,scale in [(Marker.SPHERE,.15,.25),(Marker.TEXT_VIEW_FACING,.8,.35)]:
                    m=Marker();m.header.frame_id='map';m.header.stamp=stamp;m.ns='g1_g5';m.id=i*2+(typ==Marker.TEXT_VIEW_FACING);m.type=typ;m.action=Marker.ADD;m.pose.orientation.w=1.;m.pose.position.x=g['position'][0];m.pose.position.y=g['position'][1];m.pose.position.z=z;m.scale.x=m.scale.y=m.scale.z=scale;m.color.r,m.color.g,m.color.b=colors[i];m.color.a=1.;m.text=g['name'];arr.markers.append(m)
            status=Marker();status.header.frame_id='map';status.header.stamp=stamp;status.ns='g1_g5';status.id=99;status.type=Marker.TEXT_VIEW_FACING;status.pose.orientation.w=1.;status.pose.position.x=-1.;status.pose.position.y=-2.;status.pose.position.z=1.;status.scale.z=.35;status.color.r=status.color.g=status.color.b=status.color.a=1.;status.text=f'G1 → G2 → G3 → G4 → G5 | sim {stamp.sec}.{stamp.nanosec//100000000}'
            arr.markers.append(status);pub.publish(arr)
            try:
                tr=buffer.lookup_transform('map','base_link',Time());now=n.get_clock().now().nanoseconds/1e9
                paired=pair_correspondence(tr,truth,now,time.monotonic())
                write(args.output/'presentation-status.json',dict(wall=time.monotonic(),clock=now,truth_samples=len(truth),tf_stamp=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9,paired=paired is not None))
                if paired is None:continue
                sample=paired['world_from_map_se2']+[paired['world_ground_z']];samples.append(sample);samples=samples[-10:]
                if len(samples)<5:continue
                delta=np.array(samples)-samples[-1]
                if np.max(np.linalg.norm(delta[:,:2],axis=1))>.02 or np.max(np.abs(delta[:,2]))>.01:continue
                path=args.output/'correspondence.json'
                if not path.exists():write(path,dict(paired,verified=True,wall=time.monotonic(),samples=5,source='fresh timestamp-paired public TF/evaluation truth; presentation only; no public transform published'))
            except tf2_ros.TransformException:pass
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
