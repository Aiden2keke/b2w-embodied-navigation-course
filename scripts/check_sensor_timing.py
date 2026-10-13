#!/usr/bin/python3
"""Read-only raw/public sensor timing; never starts simulation or publishes."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2

TOPICS = {'/sim/raw/imu': Imu, '/imu': Imu,
          '/sim/raw/pointcloud': PointCloud2, '/pointcloud': PointCloud2}

def stats(values):
    a = np.asarray(values, dtype=float)
    return {'samples': len(a), 'p50': float(np.percentile(a, 50)) if len(a) else None,
            'p95': float(np.percentile(a, 95)) if len(a) else None,
            'max': float(np.max(a)) if len(a) else None}

def summarize(rows, duration):
    result = {}
    for topic, values in rows.items():
        wall = [v['wall'] for v in values]
        stamps = [v['stamp'] for v in values]
        dt = stamps[-1] - stamps[0] if len(stamps) > 1 else 0
        result[topic] = {'stamp_source': 'clock' if topic == '/clock' else 'header.stamp',
                         'received_count': len(values),
                         'effective_wall_hz': len(values) / duration,
                         'effective_sim_hz': (len(values)-1)/dt if dt > 0 else None,
                         'first_stamp_s': stamps[0] if stamps else None,
                         'last_stamp_s': stamps[-1] if stamps else None,
                         'first_header_stamp_s': stamps[0] if stamps and topic != '/clock' else None,
                         'last_header_stamp_s': stamps[-1] if stamps and topic != '/clock' else None,
                         'last_simulation_clock_s': values[-1]['clock'] if values else None,
                         'clock_minus_header_s': stats([v['clock']-v['stamp'] for v in values if v['clock'] is not None]),
                         'inter_message_wall_gap_s': stats(np.diff(wall)),
                         'inter_message_header_gap_s': stats(np.diff(stamps)),
                         'stamp_reversals': sum(b<a for a,b in zip(stamps, stamps[1:]))}
    return result

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--duration', type=float, default=20)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0:
        p.error('--duration must be positive and finite')
    rclpy.init(); node = rclpy.create_node('sensor_timing_probe')
    rows = {t: [] for t in ['/clock', *TOPICS]}; clock = [None]
    def receive_clock(msg):
        clock[0] = msg.clock.sec + msg.clock.nanosec*1e-9
        rows['/clock'].append({'wall': time.monotonic(), 'stamp': clock[0], 'clock': clock[0]})
    def receive(topic, msg):
        rows[topic].append({'wall': time.monotonic(),
                            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
                            'clock': clock[0]})
    node.create_subscription(Clock, '/clock', receive_clock, qos_profile_sensor_data)
    for topic, typ in TOPICS.items():
        node.create_subscription(typ, topic, lambda m,t=topic: receive(t,m), qos_profile_sensor_data)
    start = time.monotonic()
    try:
        while time.monotonic()-start < args.duration:
            rclpy.spin_once(node, timeout_sec=.01)
    finally:
        elapsed = time.monotonic()-start
        summary = summarize(rows, elapsed)
        clock_rows = rows['/clock']
        rtf = ((clock_rows[-1]['stamp']-clock_rows[0]['stamp']) /
               (clock_rows[-1]['wall']-clock_rows[0]['wall'])) if len(clock_rows)>1 else None
        result = {'duration_wall_s': elapsed, 'observed_rtf': rtf,
                  'age_reference': 'latest /clock received by this observer; signed, no timestamp rewrite',
                  'clock_note': '/clock has no header; its zero age is a self-reference, not a transport latency measurement',
                  'qos': 'BEST_EFFORT KEEP_LAST depth=5', 'topics': summary, 'samples': rows}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n')
        lines = [f'duration={elapsed:.2f}s RTF={rtf}', 'topic count sim_Hz age_p50/p95/max_s wall_gap_p50/p95/max_s']
        for t,s in summary.items():
            a=s['clock_minus_header_s'];g=s['inter_message_wall_gap_s']
            lines.append(f"{t} {s['received_count']} {s['effective_sim_hz']} {a['p50']}/{a['p95']}/{a['max']} {g['p50']}/{g['p95']}/{g['max']}")
        txt='\n'.join(lines)+'\n';args.output.with_suffix('.txt').write_text(txt);print(txt)
        node.destroy_node();rclpy.shutdown()

if __name__ == '__main__':
    main()
