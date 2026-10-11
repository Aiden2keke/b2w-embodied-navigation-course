"""Export the actual SLAM cloud as PCD; preserve its frame and acquisition time."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from course_sim.cloud_format import decode


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--timeout',type=float,default=30.)
    args=parser.parse_args()
    path=Path(args.output)
    if path.exists() or path.with_suffix('.json').exists():
        parser.error('Choose a new output path; existing maps are preserved')
    rclpy.init()
    node=Node('export_course_map')
    received=[]
    node.create_subscription(PointCloud2,'/slam/map_points',received.append,qos_profile_sensor_data)
    started=time.monotonic()
    try:
        while not received and time.monotonic()-started<args.timeout:
            rclpy.spin_once(node,timeout_sec=.1)
        if not received:
            raise RuntimeError('No actual SLAM map received before timeout')
        msg=received[-1]
        points=decode(msg)
        values=np.column_stack([points[k] for k in 'xyz'])
        values=values[np.isfinite(values).all(axis=1)]
        if not len(values):
            raise RuntimeError('SLAM map has no finite points')
        path.parent.mkdir(parents=True,exist_ok=True)
        header=f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(values)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(values)}\nDATA ascii'
        with path.open('x') as output:
            np.savetxt(output,values,fmt='%.6f',header=header,comments='')
        metadata={'source_topic':'/slam/map_points','frame_id':msg.header.frame_id,
                  'stamp_s':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'points':len(values)}
        path.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+'\n')
        print(path)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
