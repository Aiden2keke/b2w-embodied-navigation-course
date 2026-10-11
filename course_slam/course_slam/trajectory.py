"""Export base-frame odometry from rosbag2 to separate estimate/truth TUM files."""
import argparse
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from .transforms import pose_matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    root = Path(args.output_dir)
    root.mkdir(parents=True, exist_ok=False)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topics = {'/slam/odom': 'estimate.tum', '/sim/ground_truth/odom': 'truth.tum'}
    handles = {topic: (root / name).open('w') for topic, name in topics.items()}
    last = {topic: -1 for topic in topics}
    counts = {topic: 0 for topic in topics}
    try:
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic not in topics:
                continue
            msg = deserialize_message(data, Odometry)
            if msg.child_frame_id != 'base_link':
                raise ValueError(f'{topic}: expected base_link, got {msg.child_frame_id}')
            pose_matrix(msg.pose.pose)
            time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if time <= last[topic]:
                raise ValueError('Repeated or reset timestamps; export one run per bag')
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            row = [time, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            handles[topic].write(' '.join(f'{v:.9f}' for v in row) + '\n')
            last[topic], counts[topic] = time, counts[topic] + 1
    finally:
        for handle in handles.values():
            handle.close()
    if any(n < 2 for n in counts.values()):
        raise ValueError(f'Insufficient trajectory data: {counts}')
    print(counts)
