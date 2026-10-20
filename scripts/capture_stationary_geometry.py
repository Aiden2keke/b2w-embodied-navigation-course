#!/usr/bin/python3
"""Bounded passive sensor/TF capture. No publishers, actions, launch or motion.

Simulator truth is retained under evaluation-only roles and cannot establish a
navigation registration. A capture result never authorizes route execution.
"""
import argparse
import json
import math
from pathlib import Path
import time
from uuid import uuid4

from stage4_route.journal import BinaryJournal
from stage4_route.stationary import check_zero_command, capture_ready, source_fresh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--wall-limit', type=float, default=150.)
    parser.add_argument('--sim-duration', type=float, default=30.)
    parser.add_argument('--collision-inventory', type=Path,
                        help='Optional source-SDF-bound evaluation-only inventory JSON')
    args = parser.parse_args()
    if not 30 <= args.sim_duration <= 90 or not 30 <= args.wall_limit <= 180:
        parser.error('Observation must be 30..90 sim seconds within 30..180 wall seconds')
    args.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.serialization import serialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.parameter import Parameter
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2, Imu, JointState
    from nav_msgs.msg import Odometry, OccupancyGrid
    from geometry_msgs.msg import Twist
    from tf2_msgs.msg import TFMessage
    from nav2_msgs.msg import Costmap
    from ros_gz_interfaces.msg import Contacts
    from std_msgs.msg import String
    contact_gate = None
    contact_pairs = {}
    last_support = None
    contact_metadata = {}
    if args.collision_inventory:
        import hashlib
        from stage4_route.contact_evidence import ContactEvidence
        inventory = json.loads(args.collision_inventory.read_text())
        source = Path(inventory['converted_sdf'])
        if hashlib.sha256(source.read_bytes()).hexdigest() != inventory['converted_sdf_sha256']:
            raise ValueError('Source collision inventory SDF identity changed')
        import xml.etree.ElementTree as ET
        actual = sorted(link.get('name')+'::'+collision.get('name')
                        for link in ET.parse(source).getroot().find('model').findall('link')
                        for collision in link.findall('collision'))
        if actual != inventory['collisions'] or len(actual) != 26:
            raise ValueError('Incomplete or mismatched converted-SDF collision inventory')
        contact_gate = ContactEvidence(actual)
    rclpy.init()
    node = rclpy.create_node('stationary_geometry_' + uuid4().hex[:8], parameter_overrides=[Parameter('use_sim_time', value=True)])
    journal = BinaryJournal(args.output / 'messages.sqlite3')
    fast = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
    retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    counts, frames, last_receipt, stamps, maps = {}, {}, {}, {}, []
    errors = []
    started = time.monotonic()
    last_graph = started
    graph = {}
    max_tilt = 0.
    truth_start = None
    max_drift = 0.
    sim_last = None
    first_map_receipt_sim = None
    dynamic_tf = {}
    topics = [('/clock', Clock), ('/pointcloud', PointCloud2), ('/sim/raw/pointcloud', PointCloud2),
              ('/slam/map_points', PointCloud2), ('/imu', Imu), ('/slam/odom', Odometry),
              ('/tf', TFMessage), ('/tf_static', TFMessage), ('/joint_states', JointState),
              ('/sim/raw/joint_states', JointState), ('/map', OccupancyGrid),
              ('/diagnostic/map_2cm', OccupancyGrid),
              ('/global_costmap/costmap_raw', Costmap), ('/local_costmap/costmap_raw', Costmap),
              ('/sim/ground_truth/odom', Odometry)]
    if contact_gate is not None:
        topics += [('/b2w/evaluation/full_contacts', Contacts),
                   ('/b2w/evaluation/collision_inventory', String)]
    commands = ['/course_navigation/cmd_vel_raw', '/course_navigation/cmd_vel', '/path_manager/path_manager_ros/nav_vel']

    def receive(topic, typ, msg):
        nonlocal sim_last, max_tilt, truth_start, max_drift, first_map_receipt_sim, last_support
        now = time.monotonic()
        sim = node.get_clock().now().nanoseconds * 1e-9
        source_stamp = None
        if hasattr(msg, 'header'):
            source_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frames[topic] = msg.header.frame_id
            stamps.setdefault(topic, []).append(source_stamp)
        try:
            journal.append(topic, typ, serialize_message(msg), dict(
                monotonic=now, wall=time.time(), sim=sim, source_stamp=source_stamp,
                role='evaluation_only' if topic == '/sim/ground_truth/odom' or '/evaluation/' in topic else 'sensor_or_public_tf'))
            counts[topic] = counts.get(topic, 0) + 1
            last_receipt[topic] = now
            if topic in commands:
                check_zero_command([getattr(getattr(msg, field), axis) for field in ('linear', 'angular') for axis in 'xyz'])
            if topic == '/b2w/evaluation/collision_inventory':
                value = json.loads(msg.data)
                if (type(value['sequence']) is not int or value['sequence'] <= 0 or
                        value['sampled_steps'] <= 0 or value['ever_forbidden_contact']):
                    raise ValueError('invalid_or_forbidden_cumulative_contact_interval')
                contact_gate.inventory(value['collisions'], stamp=value['stamp'], wall=now)
                contact_metadata['inventory_sequence'] = value['sequence']
                contact_metadata['interval'] = value
                last_support = value['last_ground_support_stamp']
            if topic == '/b2w/evaluation/full_contacts':
                prefix = 'b2w_evaluation_seq_'
                if not msg.header.frame_id.startswith(prefix) or not msg.header.frame_id[len(prefix):].isdigit():
                    raise ValueError('missing_contact_sequence')
                contact_metadata['contacts_sequence'] = int(msg.header.frame_id[len(prefix):])
                pairs = [[c.collision1.name, c.collision2.name] for c in msg.contacts]
                contact_gate.contacts(pairs, stamp=source_stamp, wall=now)
                from stage4_route.contact_evidence import GROUND
                for pair in pairs:
                    key = ' <> '.join(sorted(pair))
                    contact_pairs[key] = contact_pairs.get(key, 0)+1
            if contact_gate is not None and contact_gate.failure:
                raise ValueError(contact_gate.failure)
            if topic == '/clock':
                value = msg.clock.sec + msg.clock.nanosec * 1e-9
                if sim_last is not None and value < sim_last:
                    raise ValueError('simulation_clock_regression')
                sim_last = value
            if topic == '/slam/map_points':
                if source_fresh(source_stamp, sim, max_age=7.5):
                    if first_map_receipt_sim is None:
                        first_map_receipt_sim = sim
                    maps.append(source_stamp)
            if topic == '/tf':
                for tf in msg.transforms:
                    stamp = tf.header.stamp.sec + tf.header.stamp.nanosec*1e-9
                    key = tf.header.frame_id + '->' + tf.child_frame_id
                    old = dynamic_tf.get(key)
                    if old is None or stamp > old['stamp']:
                        dynamic_tf[key] = {'stamp': stamp, 'wall': now}
            if topic == '/sim/ground_truth/odom' and sim > 10.:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                tilt = math.degrees(math.acos(max(-1., min(1., 1 - 2 * (q.x*q.x + q.y*q.y)))))
                max_tilt = max(max_tilt, tilt)
                if truth_start is None:
                    truth_start = [p.x, p.y]
                max_drift = max(max_drift, math.hypot(p.x-truth_start[0], p.y-truth_start[1]))
                if tilt > 25. or p.z < .4 or max_drift > .15:
                    raise ValueError('unstable_stationary_body_evaluation')
        except (ValueError, TypeError) as exc:
            errors.append(str(exc))

    for topic, cls in topics + [(t, Twist) for t in commands]:
        typ = cls.__module__.split('.')[0] + '/msg/' + cls.__name__
        node.create_subscription(cls, topic, lambda m, t=topic, ty=typ: receive(t, ty, m),
                                 retained if topic in ('/map', '/tf_static') else fast)
    ready = False
    try:
        while time.monotonic() - started < args.wall_limit and not errors and not journal.failure:
            rclpy.spin_once(node, timeout_sec=.02)
            sim = node.get_clock().now().nanoseconds * 1e-9
            now = time.monotonic()
            if now - last_graph >= 1.:
                last_graph = now
                for topic in ['/clock', '/pointcloud', '/slam/odom', '/slam/map_points'] + commands:
                    graph[topic] = [p.node_name for p in node.get_publishers_info_by_topic(topic)]
                if any(graph[t] for t in commands):
                    errors.append('stationary_capture_requires_command_producers_inactive')
                if any(len(graph[t]) > 1 for t in ('/clock', '/slam/odom', '/slam/map_points')):
                    errors.append('duplicate_clock_or_localization_publisher')
            try:
                ready = capture_ready(maps, sim, first_observation_sim=first_map_receipt_sim,
                                      required_duration=args.sim_duration)
            except ValueError as exc:
                errors.append(str(exc))
            if ready and (contact_gate is None or
                          contact_metadata.get('inventory_sequence', -1) == contact_metadata.get('contacts_sequence', -2)):
                break
    except KeyboardInterrupt:
        errors.append('capture_interrupted')
    finally:
        final_sim = node.get_clock().now().nanoseconds * 1e-9
        end = time.monotonic()
        evidence = journal.close(timeout=10.)
        required = ('/clock', '/pointcloud', '/slam/odom', '/slam/map_points', '/tf', '/joint_states', '/map')
        fresh = {t: end-last_receipt.get(t, -1e20) for t in required}
        required_present = all(counts.get(t, 0) >= 3 for t in required)
        source_ages = {t: final_sim-v[-1] for t,v in stamps.items() if v}
        sources_current = all(source_fresh(stamps.get(t, [None])[-1], final_sim,
                                max_age=7.5 if t in ('/map', '/slam/map_points') else .5)
                              for t in required if t not in ('/clock', '/tf'))
        tf_current = all(key in dynamic_tf and
                         source_fresh(dynamic_tf[key]['stamp'], final_sim, max_age=.5) and
                         end-dynamic_tf[key]['wall'] <= 2.
                         for key in ('map->odom', 'odom->base_link'))
        contacts_current = contact_gate is None or (contact_gate.complete(sim=final_sim, wall=end)
                            and source_fresh(last_support, final_sim, max_age=.5)
                            and contact_metadata.get('inventory_sequence', -1) == contact_metadata.get('contacts_sequence', -2))
        passed = ready and not errors and evidence['complete'] and required_present and all(
            age <= (7.5 if t == '/slam/map_points' else 2.) for t, age in fresh.items()) and sources_current and tf_current and contacts_current
        report = dict(result='STATIONARY_CAPTURE_PASS' if passed else 'STATIONARY_CAPTURE_INCOMPLETE',
                      motion_authorized=False, full_route_pass=False, wall_s=end-started, final_sim=final_sim,
                      counts=counts, frames=frames, source_stamp_ranges={k:[min(v), max(v)] for k,v in stamps.items()},
                      structural_stamps=maps, first_map_receipt_sim=first_map_receipt_sim,
                      source_ages=source_ages, dynamic_tf=dynamic_tf,
                      source_fresh=sources_current, tf_fresh=tf_current,
                      contact_observation=None if contact_gate is None else dict(
                          complete=contacts_current, failure=contact_gate.failure,
                          expected=sorted(contact_gate.expected), covered=sorted(contact_gate.covered),
                          latest=contact_gate.latest, pairs=contact_pairs, last_support_stamp=last_support,
                          metadata=contact_metadata, self_intersection_coverage=False),
                      receipt_ages=fresh, errors=errors, graph=graph,
                      evaluation_only=dict(max_tilt_deg=max_tilt, max_xy_drift_m=max_drift,
                                           truth_observed=truth_start is not None), journal=evidence,
                      limitations=['No route registration/coverage certificate', 'No dynamic physical-envelope or self-intersection certificate',
                                   'Costmaps may be absent in SLAM-only observation'])
        (args.output / 'capture.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(json.dumps(report, indent=2, allow_nan=False))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if passed else 2


if __name__ == '__main__':
    raise SystemExit(main())
