#!/usr/bin/python3
"""Read-only runtime check; report received measurements, not merely topic names."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, Imu, JointState
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import WrenchStamped
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=20)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    rclpy.init()
    node=Node('course_runtime_probe')
    records,edges={},set()
    def callback(topic):
        def receive(msg):
            record=records.setdefault(topic,{'count':0})
            record['count']+=1
            if hasattr(msg,'header'):
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                record.setdefault('first_stamp',stamp)
                record['last_stamp']=stamp
                record['frame_id']=msg.header.frame_id
            if isinstance(msg,Odometry):
                p,q=msg.pose.pose.position,msg.pose.pose.orientation
                record['position']=[p.x,p.y,p.z]
                record['quaternion_xyzw']=[q.x,q.y,q.z,q.w]
                record['rpy_deg']=Rotation.from_quat(record['quaternion_xyzw']).as_euler('xyz',degrees=True).tolist()
                record['child_frame_id']=msg.child_frame_id
            if isinstance(msg,Imu):
                a=msg.linear_acceleration
                record['acceleration']=[a.x,a.y,a.z]
            if isinstance(msg,PointCloud2):
                record['points']=msg.width*msg.height
                record['fields']=[f.name for f in msg.fields]
            if isinstance(msg,JointState):
                record['joint_count']=len(msg.name)
            if isinstance(msg,OccupancyGrid):
                record['occupied_cells']=sum(x==100 for x in msg.data)
                record['free_cells']=sum(x==0 for x in msg.data)
                record['unknown_cells']=sum(x==-1 for x in msg.data)
            if isinstance(msg,WrenchStamped):
                f=msg.wrench.force
                record['force_norm_n']=math.sqrt(f.x*f.x+f.y*f.y+f.z*f.z)
            if isinstance(msg,TFMessage):
                edges.update((tf.header.frame_id,tf.child_frame_id) for tf in msg.transforms)
        return receive
    topics={'/pointcloud':PointCloud2,'/imu':Imu,'/joint_states':JointState,
            '/sim/ground_truth/odom':Odometry,'/slam/odom':Odometry,'/slam/map_points':PointCloud2,
            '/collision_plate/force_torque':WrenchStamped,'/map':OccupancyGrid,'/clock':Clock,
            '/tf':TFMessage,'/tf_static':TFMessage}
    for topic,kind in topics.items():
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic in ['/map','/tf_static'] else qos_profile_sensor_data
        node.create_subscription(kind,topic,callback(topic),qos)
    started=time.monotonic()
    while time.monotonic()-started<args.duration:
        rclpy.spin_once(node,timeout_sec=.1)
    for topic in topics:
        records.setdefault(topic,{'count':0})
    for record in records.values():
        elapsed=record.get('last_stamp',0)-record.get('first_stamp',0)
        if elapsed>0:
            record['measured_sim_hz']=(record['count']-1)/elapsed
    truth_consumers=[info.node_name for info in node.get_subscriptions_info_by_topic('/sim/ground_truth/odom')]
    result={'duration_wall_s':time.monotonic()-started,'topics':records,'tf_edges':sorted(edges),
            'truth_consumers':truth_consumers,
            'public_tf_has_truth_frame':any('sim_world' in pair for pair in edges)}
    Path(args.output).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
