#!/usr/bin/python3
"""Bounded historical A/B owner. Execution is explicitly gated; GT evaluates only."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

from owner_tf_delivery import localization_decision
from narrow_processes import OwnedProcesses
from run_narrow_baseline import digest
from warehouse_ab_supervisor import identity, same_process, write_json, pause_server

ROOT = Path(__file__).resolve().parents[1]


def open_pocket(grid, xy, radius=1.5):
    """Every grid cell intersecting a conservative body-centered disk must be free.

    Source is the online LiDAR map. Unknown/outside never counts as free.
    This admission test does not certify stopping distance or controller tracking.
    """
    try:
        r = grid['resolution']; ox, oy = grid['origin']; w, h = grid['width'], grid['height']
        if not math.isfinite(r) or r <= 0 or len(grid['data']) != w*h:
            return False
        for y in range(math.floor((xy[1]-radius-oy)/r), math.ceil((xy[1]+radius-oy)/r)+1):
            for x in range(math.floor((xy[0]-radius-ox)/r), math.ceil((xy[0]+radius-ox)/r)+1):
                near = (max(ox+x*r, min(xy[0], ox+(x+1)*r)),
                        max(oy+y*r, min(xy[1], oy+(y+1)*r)))
                if math.dist(near, xy) <= radius:
                    if not 0 <= x < w or not 0 <= y < h or grid['data'][y*w+x] != 0:
                        return False
        return True
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def motion_pocket_admission(grid, xy, *, stationary_containment):
    """Clearance admits commanded motion; stationary rest proves no clearance."""
    if stationary_containment:
        return dict(required=False, passed=None, radius_m=1.5, status='not_required_stationary')
    if not open_pocket(grid, xy):
        raise RuntimeError('Sensor-observed 1.5m open pocket absent')
    return dict(required=True, passed=True, radius_m=1.5, status='passed')


def pocket_snapshot(grid, xy, radius=1.5):
    """Explain the unchanged predicate; retain the complete checked grid for replay."""
    result = dict(radius_m=radius, sensor_xy=list(xy), grid=grid,
        decision=open_pocket(grid, xy, radius), intersecting_cells=0,
        rejected_cells=dict(unknown=0, occupied=0, outside=0, invalid=0), first_rejected=None)
    try:
        r = grid['resolution']; ox, oy = grid['origin']; w, h = grid['width'], grid['height']
        if not math.isfinite(r) or r <= 0 or len(grid['data']) != w*h:
            raise ValueError('Invalid grid resolution/data length')
        for y in range(math.floor((xy[1]-radius-oy)/r), math.ceil((xy[1]+radius-oy)/r)+1):
            for x in range(math.floor((xy[0]-radius-ox)/r), math.ceil((xy[0]+radius-ox)/r)+1):
                near = (max(ox+x*r, min(xy[0], ox+(x+1)*r)),
                        max(oy+y*r, min(xy[1], oy+(y+1)*r)))
                if math.dist(near, xy) > radius:
                    continue
                result['intersecting_cells'] += 1
                value = None
                if not 0 <= x < w or not 0 <= y < h:
                    kind = 'outside'
                else:
                    value = grid['data'][y*w+x]
                    if value == 0:
                        continue
                    kind = 'unknown' if value == -1 else 'occupied' if 0 < value <= 100 else 'invalid'
                result['rejected_cells'][kind] += 1
                if result['first_rejected'] is None:
                    result['first_rejected'] = dict(x=x, y=y, value=value, kind=kind)
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        result['rejected_cells']['invalid'] += 1
        result['diagnostic_error'] = repr(error)
    if isinstance(grid, dict) and 'data' in grid:
        result['grid'] = dict(grid, data=list(grid['data']))
    return result


def save_pocket_admission(monitor, output, label):
    """One snapshot per admission, persisted before any clearance exception."""
    pose = monitor.pose()
    result = pocket_snapshot(monitor.grid, pose[:2])
    result.update(admission=label, clock=monitor.clock, wall=time.monotonic(),
        sensor_pose=list(pose), tf_diagnostic=getattr(monitor, 'tf_diagnostic', None),
        map_evidence=getattr(monitor, 'map_evidence', None))
    path = output/('pocket-admission-'+label+'.json')
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(result, separators=(',', ':'), allow_nan=False)+'\n')
    temporary.replace(path)
    return result['decision']


def at_rest(command, velocity):
    return (len(command) == 6 and all(type(v) in (int, float) and math.isfinite(v) and v == 0 for v in command)
            and len(velocity) == 3 and all(math.isfinite(v) for v in velocity)
            and math.hypot(*velocity[:2]) <= .02 and abs(velocity[2]) <= .02)


def latch_failure(monitor, heartbeat, reason):
    """Publish the first exception immediately; independent containment follows."""
    monitor.failure=monitor.failure or reason
    try:
        write_json(heartbeat,dict(wall=time.monotonic(),clock_advanced_wall=monitor.clock_wall,
            phase='fault',failure=monitor.failure,clock=monitor.clock,contact_samples=monitor.samples))
    except (OSError,ValueError):
        pass  # The independent stale/malformed heartbeat fallback remains armed.


def stationary_passed(result, server, commands, rest, contact_samples):
    from warehouse_ab_supervisor import rest_proof
    pause = result.get('pause', {})
    return (result.get('reason') == 'monitor:stationary_qualification_injection'
        and result.get('emergency') is True and result.get('remaining') == {}
        and pause.get('contained') is True and pause.get('state') == 'T'
        and bool(pause.get('threads')) and all(s in ('T', 't', 'Z') for s in pause['threads'].values())
        and str(result.get('owned_pid_start_ticks', {}).get(str(server['pid']))) == str(server['start_ticks'])
        and commands['samples'] > 0 and commands['nonzero'] == 0
        and rest_proof(rest) and contact_samples > 0)


class StopEvidence:
    """Retain the entire coast while separately accepting a settled window."""
    def __init__(self):
        self.initial=self.start_sim=self.start_wall=self.settled=None
        self.whole_distance=self.whole_yaw=self.distance=self.yaw=0.

    def observe(self, *, sim, wall, pose, stationary, zero):
        yaw_delta=lambda a,b:abs(math.atan2(math.sin(a-b),math.cos(a-b)))
        if not zero:
            if self.initial is not None: raise RuntimeError('Nonzero final command reappeared during stop')
            return None
        if self.initial is None: self.initial=pose
        self.whole_distance=max(self.whole_distance,math.dist(self.initial[:2],pose[:2]))
        self.whole_yaw=max(self.whole_yaw,yaw_delta(self.initial[2],pose[2]))
        if self.whole_distance>.25 or self.whole_yaw>.35:
            raise RuntimeError('Whole-stop excursion exceeds precheck envelope')
        if not stationary:
            self.start_sim=self.start_wall=self.settled=None
            return None
        if self.settled is None:
            self.settled=pose;self.start_sim=sim;self.start_wall=wall;self.distance=self.yaw=0.
        self.distance=max(self.distance,math.dist(self.settled[:2],pose[:2]))
        self.yaw=max(self.yaw,yaw_delta(self.settled[2],pose[2]))
        if self.distance>.02 or self.yaw>.03:
            self.start_sim=self.start_wall=self.settled=None
            return None
        if sim-self.start_sim<5. or wall-self.start_wall<3.:return None
        return dict(start_sim=self.start_sim,end_sim=sim,wall_s=wall-self.start_wall,
            max_displacement_m=self.distance,max_yaw_rad=self.yaw,
            whole_stop_max_displacement_m=self.whole_distance,whole_stop_max_yaw_rad=self.whole_yaw,
            measured_lio_rest=True,measured_ground_truth_rest=True,six_axis_final_zero=True)


def world_copy(source, output, library):
    tree = ET.parse(source); world = tree.getroot().find('world')
    if world.get('name') != 'course_world':
        raise ValueError('Original course_world required')
    if any(p.get('name') == 'b2w_evaluation::Observer' for p in world.findall('plugin')):
        raise ValueError('Observer already present')
    original = ET.tostring(world)
    added = ET.SubElement(world, 'plugin', filename=str(library), name='b2w_evaluation::Observer')
    tree.write(output, encoding='unicode')
    world.remove(added)
    if ET.tostring(world) != original:
        raise ValueError('World geometry/settings changed')
    return dict(source_sha256=digest(source), output_sha256=digest(output),
                plugin_sha256=digest(library), geometry_and_settings_preserved=True)


def contact_bridge():
    return ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/b2w/evaluation/full_contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
        '/b2w/evaluation/collision_inventory@std_msgs/msg/String[ignition.msgs.StringMsg',
        '--ros-args', '-r', '__node:=warehouse_ab_evaluation_bridge', '-p', 'use_sim_time:=true']


def native_snapshot(owner, *, navigation, manifest=None):
    pins=json.loads((manifest or ROOT/'artifacts/recovery/root-installed-manifest.json').read_text())['native']
    required={path:metadata['sha256'] for path,metadata in pins.items() if navigation or path.startswith(str(ROOT/'install'))}
    observed={};processes=[]
    for pid,ticks in owner.scan().items():
        try:
            base=Path(f'/proc/{pid}')
            exe=str((base/'exe').resolve())
            cmdline=[part.decode(errors='replace') for part in (base/'cmdline').read_bytes().split(b'\0') if part]
            mappings={(line.split(None,5)[5]) for line in (base/'maps').read_text().splitlines() if len(line.split(None,5))==6}
            hits=({exe}|mappings)&set(required)
            if hits:
                values={path:digest(Path(path)) for path in sorted(hits)}
                for path,sha in values.items():
                    if sha!=required[path]:raise ValueError('Actual loaded native identity differs: '+path)
                    observed[path]=sha
                processes.append(dict(pid=pid,start_ticks=int(ticks),executable=exe,cmdline=cmdline,mapped_or_executed_sha256=values))
        except (FileNotFoundError,ProcessLookupError):
            continue
    missing=sorted(set(required)-set(observed))
    if missing:raise ValueError('Required actual native mappings/executables absent: '+repr(missing))
    return dict(processes=processes,required=required,missing=missing)


def construction_audit(output):
    """Fortress stdout is captured by launch in stack.log on this install."""
    import re
    files = [p for p in (output/'ros-log').glob('**/*.log') if 'ign' in p.name or 'gazebo' in p.name]
    lines = [(p, line) for p in files for line in p.read_text(errors='replace').splitlines()]
    stack = output/'stack.log'
    if stack.is_file():
        captured = [(stack, line) for line in stack.read_text(errors='replace').splitlines() if re.match(r'\[ign-\d+\]', line)]
        if captured:
            files.append(stack); lines.extend(captured)
    if not files:
        raise RuntimeError('Actual Gazebo startup logs unavailable for construction audit')
    errors = [dict(file=str(p), line=line) for p, line in lines if re.search(
        r'(failed|unable|error|unsupported|not found|cannot).*(collision|mesh|physics|geometry)|(collision|mesh|physics|geometry).*(failed|unable|error|unsupported|not found|cannot)', line, re.I)]
    if errors:
        raise RuntimeError('Physics construction warning/error: '+repr(errors[:3]))
    return [str(p) for p in files]


class Monitor:
    def __init__(self, node, inventory, heartbeat):
        from course_navigation.stage4_contact_stream import ContactStream
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from nav_msgs.msg import Odometry, OccupancyGrid
        from rosgraph_msgs.msg import Clock
        from ros_gz_interfaces.msg import Contacts
        from std_msgs.msg import String
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import PointCloud2, Imu
        from owner_tf_delivery import LocalizationIngress
        self.node, self.heartbeat = node, heartbeat
        self.contacts = ContactStream(inventory['collisions'], first_sequence=None, observation_only=True)
        self.clock = None; self.clock_wall = None; self.failure = None; self.phase = 'bootstrap'
        self.odom = self.final = self.grid = None
        self.latest = {}; self.support = None; self.samples = 0
        self.final_samples = self.nonzero_final_samples = 0
        self.support_feet = set()
        self.contact_armed=False; self.pending_contacts={}
        self.truth = None
        self.truth_history = deque(maxlen=128)
        self.truth_first_fault = None
        self.stamp_history = {}
        self.last_readiness = self.readiness_first_fault = None
        self.tf_diagnostic = None
        self.events = heartbeat.with_name('monitor-events.jsonl').open('x', buffering=1)
        self.node.create_subscription(Clock, '/clock', self.receive_clock, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, '/sim/ground_truth/odom', self.receive_truth, qos_profile_sensor_data)
        self.node.create_subscription(Twist, '/path_manager/path_manager_ros/nav_vel', self.receive_final, 10)
        self.node.create_subscription(Contacts, '/b2w/evaluation/full_contacts', self.receive_contacts, 10)
        self.node.create_subscription(String, '/b2w/evaluation/collision_inventory', self.receive_inventory, 10)
        for topic, kind in (('/pointcloud', PointCloud2), ('/imu', Imu), ('/slam/map_points', PointCloud2)):
            self.node.create_subscription(kind, topic, lambda msg, name=topic: self.receive_sensor(name, msg), qos_profile_sensor_data)
        self.node.create_subscription(OccupancyGrid, '/map', self.receive_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.ingress = LocalizationIngress(context=node.context)
        self.tf_delivery = self.ingress.delivery
        self.tf = self.ingress.buffer
        self.listener = self.ingress.listener

    def receive_clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec*1e-9
        if self.clock is not None and value < self.clock:
            self.failure = self.failure or 'clock_regression'
        if self.clock is None or value > self.clock:
            self.clock_wall = time.monotonic()
        self.clock = value

    def receive_odom(self, msg, receipt_wall=None):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.odom and stamp < self.odom[0]:
            self.failure = self.failure or 'lio_stamp_regression'
        self.odom = (stamp, msg)
        self.latest['odom'] = time.monotonic() if receipt_wall is None else receipt_wall
        self.remember('odom', stamp, self.latest['odom'], msg)

    def record(self, kind, msg):
        from rosidl_runtime_py.convert import message_to_ordereddict
        self.events.write(json.dumps(dict(kind=kind, wall=time.monotonic(), sim=self.clock,
            message=message_to_ordereddict(msg)))+'\n')

    def receive_truth(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.truth_history.append((stamp, time.monotonic(), msg))
        self.stamp_history['truth'] = self.truth_history

    def select_truth(self, wall):
        """Newest received evaluation sample consistent with this owner clock.

        DDS may deliver truth before its /clock callback. Keep a bounded history
        rather than replacing a still-fresh consistent sample with future truth.
        Both original 0.5s limits apply independently to every selected sample.
        """
        self.truth = max((sample for sample in reversed(self.truth_history)
            if self.clock is not None and 0 <= wall-sample[1] <= .5
            and 0 <= self.clock-sample[0] <= .5),
            key=lambda sample: (sample[0], sample[1]), default=None)
        return self.truth is not None

    def record_truth_fault(self, wall):
        """Capture only the first failed freshness decision, never per cycle."""
        if self.truth_first_fault is not None:
            return
        latest = self.truth_history[-1] if self.truth_history else None
        stamp, receipt = latest[:2] if latest else (None, None)
        wall_age = wall-receipt if receipt is not None else None
        sim_age = self.clock-stamp if self.clock is not None and stamp is not None else None
        if latest is None: reason = 'missing_truth'
        elif self.clock is None: reason = 'missing_clock'
        elif not 0 <= wall_age <= .5: reason = 'truth_wall_stale'
        elif sim_age < 0: reason = 'truth_ahead_of_clock'
        else: reason = 'truth_sim_stale'
        self.truth_first_fault = dict(owner_clock=self.clock, truth_stamp=stamp,
            truth_receipt_wall=receipt, measured_wall=wall, wall_age_s=wall_age,
            simulation_age_s=sim_age, failed_subpredicate=reason,
            phase=self.phase, buffered_samples=len(self.truth_history))
        try:
            write_json(self.heartbeat.with_name('truth-first-fault.json'), self.truth_first_fault)
        except OSError:
            pass  # In-memory report survives; independent stale-heartbeat abort stays armed.

    def receive_final(self, msg):
        self.final = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        self.final_samples += 1
        self.nonzero_final_samples += int(any(v != 0 for v in self.final))
        self.latest['final'] = time.monotonic()
        self.record('final', msg)

    def receive_contacts(self, msg):
        if not self.contact_armed:
            self.record('full_contacts',msg)
            try:
                sequence=int(msg.header.frame_id.removeprefix('b2w_evaluation_seq_'))
                self.pending_contacts[sequence]=msg
                for old in sorted(self.pending_contacts)[:-16]:del self.pending_contacts[old]
            except (ValueError,AttributeError):
                self.failure=self.failure or 'invalid_contact_sequence'
            return
        self.contacts.contact_message(msg, wall=time.monotonic())
        self.samples += 1
        self.remember_contacts()
        self.failure = self.failure or self.contacts.failure
        self.record('full_contacts', msg)
        from course_navigation.stage4_contact_evidence import GROUND, SUPPORTS
        for c in msg.contacts:
            pair = (c.collision1.name, c.collision2.name)
            for i in (0, 1):
                prefix = 'course_world::b2w::'
                if pair[i].startswith(prefix) and pair[i][len(prefix):] in SUPPORTS and pair[1-i] == GROUND:
                    self.support_feet.add(pair[i][len(prefix):])

    def receive_sensor(self, topic, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        previous = self.latest.get(topic)
        if previous and stamp < previous[1]:
            self.failure = self.failure or 'sensor_stamp_regression:' + topic
        self.latest[topic] = (time.monotonic(), stamp)
        self.remember(topic, stamp, self.latest[topic][0])

    def receive_inventory(self, msg):
        try:
            value = json.loads(msg.data)
            if not self.contact_armed:
                # The world reports before the robot spawns. Preserve those
                # raw reports, then bind the first complete source inventory.
                if sorted(value['collisions'])!=sorted(self.contacts.names):
                    self.record('collision_inventory',msg)
                    return
                from course_navigation.stage4_contact_stream import ContactStream
                self.contacts=ContactStream(self.contacts.names,first_sequence=value['sequence'])
                self.contact_armed=True
                matching=self.pending_contacts.pop(value['sequence'],None)
                self.pending_contacts.clear()
                if matching is not None:self.receive_contacts(matching)
            self.contacts.inventory(value, wall=time.monotonic())
            self.support = value['last_ground_support_stamp']
            self.latest['support'] = time.monotonic()
            self.remember('support', self.support, self.latest['support'])
            self.remember_contacts()
            self.failure = self.failure or self.contacts.failure
            self.record('collision_inventory', msg)
        except Exception as error:
            self.failure = self.failure or 'invalid_contact_inventory:' + repr(error)

    def receive_map(self, msg):
        from rosidl_runtime_py.convert import message_to_ordereddict
        q = msg.info.origin.orientation
        if abs(q.x)+abs(q.y)+abs(q.z) > 1e-8 or abs(q.w-1.) > 1e-8:
            self.failure = self.failure or 'rotated_map_not_supported'
        self.grid = dict(width=msg.info.width, height=msg.info.height, resolution=msg.info.resolution,
                         origin=(msg.info.origin.position.x, msg.info.origin.position.y), data=msg.data,
                         observed_free_present=0 in msg.data)
        self.latest['map'] = time.monotonic()
        # The cached message array is immutable after delivery. Preserve it in
        # grid, converting once at admission; never recurse through 640k cells
        # in a ROS callback or serialize the same array twice.
        self.map_evidence = dict(message=dict(header=message_to_ordereddict(msg.header),
            info=message_to_ordereddict(msg.info)), receipt_wall=self.latest['map'], data_path='grid.data')

    def remember(self, key, stamp, receipt, value=None):
        history = self.stamp_history.setdefault(key, deque(maxlen=128))
        # Equal-stamp reports cannot manufacture a new freshness deadline.
        if not history or stamp != history[-1][0]:
            history.append((stamp, receipt, value))

    def remember_contacts(self):
        """Keep only complete, already validated pairs; immediate veto stays live."""
        intervals = self.contacts.intervals
        if intervals is None or intervals.end is None or self.contacts.failure:
            return
        gate = intervals.gate
        stamps = dict(gate.latest)
        if set(stamps) != {'inventory', 'contacts'} or gate.covered != gate.expected:
            return
        self.remember('contacts', max(v[0] for v in stamps.values()),
                      min(v[1] for v in stamps.values()), stamps)

    def stream_status(self, key, wall, sim_limit, wall_limit, fallback=None):
        history = self.stamp_history.get(key) or ([fallback] if fallback is not None else [])
        selected = None
        # Clock/wall belong to this decision, not each history entry. Scan the
        # full unordered history and retain max(stamp,receipt), including the
        # original first-entry behavior for equal keys; never cache admission.
        clock = self.clock
        if (type(clock) in (int,float) and math.isfinite(clock)
                and type(wall) in (int,float) and math.isfinite(wall)):
            for item in history:
                stamp, receipt = item[:2]
                if (type(stamp) not in (int,float) or not math.isfinite(stamp)
                        or type(receipt) not in (int,float) or not math.isfinite(receipt)):
                    continue
                if (0 <= clock-stamp <= sim_limit and 0 <= wall-receipt
                        and (wall_limit is None or wall-receipt <= wall_limit)
                        and (selected is None or (stamp,receipt) > (selected[0],selected[1]))):
                    selected = item
        sample = selected or (history[-1] if history else None)
        return dict(passed=selected is not None, stamp=sample[0] if sample else None,
                    receipt_wall=sample[1] if sample else None,
                    simulation_age_s=self.clock-sample[0] if sample and type(sample[0]) in (int,float) and type(self.clock) in (int,float) else None,
                    wall_age_s=wall-sample[1] if sample and type(sample[1]) in (int,float) else None,
                    latest_stamp=history[-1][0] if history else None,
                    latest_receipt_wall=history[-1][1] if history else None,
                    buffered_samples=len(history), exception=None,
                    simulation_limit_s=sim_limit, wall_limit_s=wall_limit), selected

    @localization_decision
    def pose(self, wall=None):
        wall = time.monotonic() if wall is None else wall
        from rclpy.time import Time
        tf = self.tf.lookup_transform('map', 'base_link', Time())
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec*1e-9
        if self.clock is not None and stamp > self.clock:
            # Worker may lead /clock. Use a previously composed, source-stamped
            # chain; do not request interpolation at now to manufacture age0.
            history = getattr(getattr(self, 'tf_delivery', None), 'coherent', ())
            eligible = [item for item in history if item[0] <= self.clock]
            if eligible:
                stamp, _, tf = max(eligible, key=lambda item:item[0])
        # Retain actual stamped transforms; never extrapolate or stamp at now.
        self.remember('tf', stamp, wall, tf)
        status, selected = self.stream_status('tf', wall, .5, None)
        status['receipt_semantics'] = 'owner lookup time; TF callback receipt unavailable'
        self.tf_diagnostic = status
        if selected is None:
            raise ValueError('stale_map_base_tf')
        p, q = selected[2].transform.translation, selected[2].transform.rotation
        return (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    @localization_decision
    def readiness(self, require_final=True, wall=None):
        """Evaluate every predicate against ONE decision time, without short circuit."""
        wall = time.monotonic() if wall is None else wall
        checks = {}
        checks['failure'] = dict(passed=not self.failure, reason=self.failure)
        age = wall-self.clock_wall if self.clock_wall is not None else None
        checks['clock'] = dict(passed=self.clock is not None and age is not None and 0 <= age <= 1.,
                               stamp=self.clock, receipt_wall=self.clock_wall, wall_age_s=age, wall_limit_s=1.)
        checks['truth'], truth = self.stream_status('truth', wall, .5, .5)
        # The truth deque also feeds the independent physical veto.
        self.truth = truth
        fallback = (self.odom[0],self.latest.get('odom'),self.odom[1]) if self.odom else None
        checks['odom'], odom = self.stream_status('odom',wall,.5,.5,fallback)
        self.selected_odom = odom[2] if odom else None
        for topic, sim_limit, wall_limit in (('/imu',1.,1.),('/pointcloud',1.,1.),('/slam/map_points',6.,20.)):
            item = self.latest.get(topic)
            checks[topic], _ = self.stream_status(topic,wall,sim_limit,wall_limit,
                                                  (item[1],item[0],None) if item else None)
        checks['support'], _ = self.stream_status('support',wall,.5,None,
                                                  (self.support,self.latest.get('support'),None) if self.support is not None else None)
        checks['support_feet'] = dict(passed=len(self.support_feet)==4, observed=sorted(self.support_feet))
        if self.stamp_history.get('contacts'):
            checks['contacts'], selected_contact = self.stream_status('contacts',wall,.5,.5)
            checks['contacts']['selected_gate_stamps'] = selected_contact[2] if selected_contact else None
            checks['contacts']['failure'] = self.contacts.failure
            checks['contacts']['passed'] &= self.contacts.failure is None
        else:
            try:
                checks['contacts'] = dict(passed=self.contacts.ready(sim=self.clock,wall=wall),
                                          failure=getattr(self.contacts,'failure',None), exception=None)
            except Exception as exc:
                checks['contacts'] = dict(passed=False, exception=repr(exc))
        intervals = getattr(self.contacts,'intervals',None)
        if intervals is not None:
            checks['contacts'].update(interval_end=intervals.end, last_pair_wall=intervals.last_wall,
                next_sequence=intervals.next_sequence, pending_sequences=sorted(intervals.pending),
                latest_gate_stamps=dict(intervals.gate.latest))
        self.tf_diagnostic = None
        try:
            pose = self.pose(wall=wall)
            checks['tf'] = dict(self.tf_diagnostic or {}, passed=True, pose=pose, exception=None)
        except Exception as exc:
            previous_tf = self.tf_diagnostic or self.stream_status('tf',wall,.5,None)[0]
            checks['tf'] = dict(previous_tf, passed=False, exception=repr(exc))
        receipt = self.latest.get('final')
        age = wall-receipt if receipt is not None else None
        checks['final'] = dict(passed=not require_final or self.final is not None and age is not None and 0 <= age <= .5,
                               required=require_final, receipt_wall=receipt, wall_age_s=age,
                               wall_limit_s=.5, value=self.final,
                               six_axis_finite=self.final is not None and len(self.final)==6 and all(math.isfinite(v) for v in self.final),
                               six_axis_zero=self.final is not None and len(self.final)==6 and all(v==0 for v in self.final))
        result = dict(wall=wall, clock=self.clock, phase=self.phase, predicates=checks,
                      map_admission=dict(present=self.grid is not None, receipt_wall=self.latest.get('map'),
                                         purpose='separate bootstrap admission; no map age threshold in ready'),
                      passed=all(row['passed'] for row in checks.values()))
        # Faults can themselves contain NaN/Inf; preserve them as explicit strings
        # so strict JSON evidence cannot fail while reporting a safety fault.
        def finite_json(value):
            if isinstance(value,float) and not math.isfinite(value): return repr(value)
            if isinstance(value,dict): return {k:finite_json(v) for k,v in value.items()}
            if isinstance(value,(list,tuple)): return [finite_json(v) for v in value]
            return value
        if not result['passed'] and getattr(self, 'tf', None) is not None:
            result['tf_delivery'] = self.tf_delivery.snapshot(self.tf)
        self.last_readiness = finite_json(result)
        return self.last_readiness

    def ready(self, require_final=True):
        return self.readiness(require_final)['passed']

    def record_readiness_fault(self, snapshot=None):
        if self.readiness_first_fault is not None:
            return
        self.readiness_first_fault = snapshot if snapshot is not None else self.readiness()
        if hasattr(self, 'tf_delivery') and 'tf_delivery' not in self.readiness_first_fault:
            self.readiness_first_fault['tf_delivery'] = self.tf_delivery.snapshot(self.tf)
        try:
            write_json(self.heartbeat.with_name('readiness-first-fault.json'), self.readiness_first_fault)
        except (OSError, ValueError):
            pass  # In-memory snapshot survives; independent watchdog remains armed.

    def update(self):
        if self.phase not in ('bootstrap', 'shutdown'):
            snapshot = self.readiness()
            if not snapshot['passed']:
                self.record_readiness_fault(snapshot)
                self.failure = self.failure or 'monitor_sensor_contact_tf_or_command_stale'
        value = dict(wall=time.monotonic(), clock_advanced_wall=self.clock_wall,
            phase=self.phase, failure=self.failure, clock=self.clock, contact_samples=self.samples)
        if hasattr(self, 'normal_rest'): value['normal_rest'] = self.normal_rest
        write_json(self.heartbeat, value)
        if self.failure:
            raise RuntimeError(self.failure)

    def resting(self):
        if not self.ready():
            return False
        v = self.selected_odom.twist.twist
        tv = self.truth[2].twist.twist
        return at_rest(self.final, (v.linear.x, v.linear.y, v.angular.z)) and at_rest(self.final, (tv.linear.x,tv.linear.y,tv.angular.z))

    def truth_pose(self):
        p, q = self.truth[2].pose.pose.position, self.truth[2].pose.pose.orientation
        return (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def physical_veto(self):
        """Evaluation truth can veto; it never supplies navigation/control values."""
        wall = time.monotonic()
        if not self.select_truth(wall):
            self.record_truth_fault(wall)
            return 'evaluation_truth_stale'
        p, q = self.truth[2].pose.pose.position, self.truth[2].pose.pose.orientation
        values = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        if not all(math.isfinite(v) for v in values): return 'nonfinite_physical_pose'
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        if getattr(self,'route_motion_caps',None) is not None:
            v=self.truth[2].twist.twist;speed,yaw=self.route_motion_caps
            if (not all(math.isfinite(x) for x in (v.linear.x,v.linear.y,v.angular.z))
                    or math.hypot(v.linear.x,v.linear.y)>speed or abs(v.angular.z)>yaw):
                return 'qualified_physical_motion_envelope_exceeded'
        if max(abs(roll),abs(pitch)) > min(.4,getattr(self,'route_tilt_limit',.4)) or not .25 <= p.z <= 1.2:
            return 'body_orientation_or_height_limit'
        origin = getattr(self, 'precheck_motion_origin', None)
        if self.phase == 'precheck' and origin is not None and math.dist(origin[:2], (p.x,p.y)) > .25:
            return 'precheck_initial_reference_excursion'
        if self.final is None or not all(math.isfinite(v) for v in self.final): return 'invalid_final_command'
        if any(self.final[i] != 0 for i in (1,2,3,4)) or abs(self.final[0]) > min(.400001,getattr(self,'route_command_cap',.4)+.000001) or abs(self.final[5]) > .600001:
            return 'final_command_limit'
        return None


def capture_planning_snapshot(monitor):
    """Retain immutable received grid data; never traverse it in the live loop."""
    if not monitor.ready() or monitor.physical_veto():
        raise RuntimeError('Planning snapshot readiness/physical veto')
    return dict(captured_ready=True,grid=monitor.grid,sensor_pose=monitor.pose(),clock=monitor.clock,
        map_evidence=getattr(monitor,'map_evidence',None),
        tf_diagnostic=getattr(monitor,'tf_diagnostic',None),
        readiness=getattr(monitor,'last_readiness',None))


def finish_planning_capture(output,capture):
    """Offline evidence/geometry only; caller must first finish owned teardown."""
    from warehouse_route_admission import admit,admit_directional,collision_radius
    snapshot=capture['snapshot']
    if snapshot.get('captured_ready') is not True:raise ValueError('Snapshot lacks live readiness acceptance')
    pose=snapshot['sensor_pose'];grid=snapshot['grid']
    prefix=capture.get('prefix');radius=collision_radius(ROOT/'artifacts/recovery/robot.urdf')
    # ready=True means the captured snapshot passed its live checks, not that
    # a stopped simulator can supply fresh localization or authorize motion.
    result=admit(grid,pose=pose,path=prefix,radius=radius,padding=.05,
        reaction_stop_m=None,ready=True)
    result.update(articulated_collision_radius_m=radius,initial_prefix=prefix,
        operational_candidate=admit_directional(grid,pose=pose,path=prefix,
            half_extents=(.7,.38),padding=.05,stop=None,ready=True),
        operational_candidate_is_motion_authority=False,
        analyzed_after_owned_teardown=True)
    write_json(output/'route-start-admission.json',result)
    frozen=dict(snapshot,grid=dict(grid,data=list(grid['data'])),decision=result)
    target=output/'route-start-map.json';temporary=target.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(frozen,separators=(',',':'),allow_nan=False)+'\n')
    temporary.replace(target)
    return result


def route_start_admission(node, monitor, output, wait, prepare_navigation, *, planning_only=False, envelope=None, owned_goal_producer=None, deferred_capture=None):
    """Planning-only route entry. No NavigateToPose or command on rejection.

    Current evidence does not bound plant response at either A/B operating cap.
    Request and retain the real Smac path, but do not waive that missing input.
    Route mode consumes a separately reviewed, hash-pinned operational envelope.
    Planning capture grants no motion and has no footprint-only fallback.
    """
    from warehouse_route_admission import admit, admit_directional, collision_radius, initial_prefix
    from run_warehouse_route import load_route, critical_goals
    from rclpy.action import ActionClient
    from nav2_msgs.action import ComputePathToPose
    from geometry_msgs.msg import PoseStamped
    radius = None if planning_only else collision_radius(ROOT/'artifacts/recovery/robot.urdf')
    def save(result):
        write_json(output/'route-start-admission.json', result)
        # Retain exactly the sensor grid used by the predicate, once per
        # decision. Compact serialization avoids the historical indented-map
        # callback starvation. The watchdog stays armed through evidence I/O.
        grid = dict(monitor.grid, data=list(monitor.grid['data']))
        snapshot = dict(grid=grid,sensor_pose=monitor.pose(),clock=monitor.clock,
                        map_evidence=getattr(monitor,'map_evidence',None),
                        tf_diagnostic=getattr(monitor,'tf_diagnostic',None),
                        readiness=getattr(monitor,'last_readiness',None),decision=result)
        target=output/'route-start-map.json';temporary=target.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(snapshot,separators=(',',':'),allow_nan=False)+'\n')
        temporary.replace(target)
    if planning_only:
        if deferred_capture is None:raise ValueError('Planning-only requires deferred evidence storage')
        deferred_capture['snapshot']=capture_planning_snapshot(monitor)
    else:
        if envelope is None:
            result = admit(monitor.grid, pose=monitor.pose(), path=None, radius=radius,
                padding=.05, reaction_stop_m=None, ready=monitor.ready() and not monitor.physical_veto())
        else:
            result=admit_directional(monitor.grid,pose=monitor.pose(),path=None,half_extents=envelope['half_extents'],
                padding=envelope['padding_m']+envelope['localization_error_m']+envelope['tracking_error_m'],
                stop=envelope['stop'],ready=monitor.ready() and not monitor.physical_veto())
            result['current_footprint_passed']=result['footprint_passed']
        result['articulated_collision_radius_m'] = radius
        save(result)
        if not planning_only and not result['current_footprint_passed']:
            raise RuntimeError('Route admission: '+result['reason'])
    prepare_navigation()
    if not monitor.ready() or monitor.physical_veto():
        raise RuntimeError('Route planning readiness/physical veto')
    goal_producers=node.get_publishers_info_by_topic('/goal_pose')
    allowed_producer=(owned_goal_producer is not None and len(goal_producers)==1
        and goal_producers[0].node_name==owned_goal_producer)
    if (len(node.get_publishers_info_by_topic('/path_manager/path_manager_ros/nav_vel')) != 1
            or (goal_producers and not allowed_producer)
            or monitor.final is None or any(monitor.final)):
        raise RuntimeError('Route planning requires unique idle velocity owner and no goal producer')
    goal = critical_goals(load_route())[0]
    client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    try:
        wait(client.server_is_ready, 10.)
        request = ComputePathToPose.Goal()
        request.goal = PoseStamped()
        request.goal.header.frame_id = 'map'
        request.goal.header.stamp = node.get_clock().now().to_msg()
        position = goal['pose']['pose']['position']
        orientation = goal['pose']['pose']['orientation']
        request.goal.pose.position.x = position['x']; request.goal.pose.position.y = position['y']
        for axis in ('x','y','z','w'): setattr(request.goal.pose.orientation,axis,orientation[axis])
        request.planner_id = 'GridBased'; request.use_start = False
        request_pose=monitor.pose()
        if hasattr(monitor,'tf'):
            from rclpy.time import Time
            from rosidl_runtime_py.convert import message_to_ordereddict
            write_json(output/'planning-tf.json',{parent+'->'+child:message_to_ordereddict(
                monitor.tf.lookup_transform(parent,child,Time()))
                for parent,child in (('map','odom'),('odom','base_link'),('map','base_link'))})
        future = client.send_goal_async(request)
        wait(future.done, 10.)
        handle = future.result()
        if not handle.accepted: raise RuntimeError('Initial planning-only request rejected')
        future = handle.get_result_async()
        wait(future.done, 10.)
        response = future.result(); path = response.result.path
        def planar(p):
            q=p.pose.orientation
            if (p.header.frame_id not in ('','map')
                    or abs(q.x)+abs(q.y)>1e-6
                    or not all(math.isfinite(v) for v in (q.z,q.w))
                    or abs(q.z*q.z+q.w*q.w-1.)>1e-6):
                raise ValueError('Invalid initial path frame/orientation')
            return (p.pose.position.x,p.pose.position.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z))
        packet=dict(status=response.status,frame=path.header.frame_id,
            stamp=path.header.stamp.sec+path.header.stamp.nanosec*1e-9,
            poses=[planar(p) for p in path.poses],
            planner_id=request.planner_id,use_start=request.use_start,
            goal=goal,request_pose=request_pose,response_pose=monitor.pose(),
            tf_diagnostic=getattr(monitor,'tf_diagnostic',None))
        write_json(output/'initial-smac-plan.json',packet)
        prefix=initial_prefix(packet,monitor.pose(),(position['x'],position['y']),monitor.clock)
        if planning_only:
            if not monitor.ready() or monitor.physical_veto():
                raise RuntimeError('Planning capture readiness/physical veto')
            deferred_capture.update(snapshot=capture_planning_snapshot(monitor),prefix=prefix)
            return dict(planning_completed=True,motion_admitted=False,
                clearance='deferred_until_owned_teardown')
        if envelope is None:
            result=admit(monitor.grid,pose=monitor.pose(),path=prefix,radius=radius,
                padding=.05,reaction_stop_m=None,ready=monitor.ready() and not monitor.physical_veto())
        else:
            result=admit_directional(monitor.grid,pose=monitor.pose(),path=prefix,half_extents=envelope['half_extents'],
                padding=envelope['padding_m']+envelope['localization_error_m']+envelope['tracking_error_m'],
                stop=envelope['stop'],ready=monitor.ready() and not monitor.physical_veto())
        result.update(initial_prefix=prefix,articulated_collision_radius_m=radius,
            unavailable_bounds=['physical reaction/stopping and tracking deviation at A/B cap',
                                'metric localization error beyond existing configured padding'],
            qualification_observed_coast_m=.04259545786007883,
            observed_coast_is_operating_bound=False)
        if envelope is not None:result['unavailable_bounds']=[]
        save(result)
        if not result['passed']: raise RuntimeError('Route admission: '+result['reason'])
        return result
    finally:
        client.destroy()


def execute(args, output):
    """One simulator: isolated qualification pulses or planning-gated route."""
    import rclpy
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Twist
    session_start_wall=time.monotonic()
    token = uuid.uuid4().hex
    env = dict(os.environ, B2W_NARROW_OWNER=token, IGN_PARTITION='b2w-historical-ab-'+token,
               ROS_LOG_DIR=str(output/'ros-log'), PYTHONDONTWRITEBYTECODE='1')
    owner = OwnedProcesses(token); children = []; logs = []; watchdog = None; monitor = None; publisher = None
    planning_only=getattr(args,'planning_only',False)
    deferred_capture={}
    envelope=None
    if getattr(args,'route_envelope',None) is not None:
        from warehouse_route_admission import load_envelope
        config=ROOT/'course_navigation/config'/('nav2_smac2d_diag.yaml' if args.profile=='A' else 'nav2_smac2d_dwb_slow_diag.yaml')
        envelope=load_envelope(args.route_envelope,args.route_envelope_sha256,
            digest(ROOT/'artifacts/recovery/robot.urdf'),digest(config),[.4 if args.profile=='A' else .2,.6],digest(args.deployment_manifest))
    report = dict(profile=args.profile, token=token, invocations=[], success=False,planning_only=planning_only,
                  precheck=[], precheck_only=args.precheck_only, motion_authorization=args.authorize_motion,
                  physics_rng_seeded=False, seed=0, safety_qualification='bounded independent simulation diagnostic; not Stage4 qualification')
    spawn_x, spawn_y = getattr(args, 'precheck_spawn_x', None), getattr(args, 'precheck_spawn_y', None)
    report['requested_precheck_spawn'] = None if spawn_x is None else dict(x=spawn_x, y=spawn_y)
    report['route_mode'] = getattr(args, 'route_mode', 'critical')
    report['continuation_yaws'] = getattr(args, 'continuation_yaws', None)
    heartbeat = output/'heartbeat.json'; watchdog_result = output/'watchdog-result.json'
    def launch(argv, name):
        log = (output/(name+'.log')).open('xb'); logs.append(log)
        process = subprocess.Popen(argv, env=env, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(process); owner.scan()
        report['invocations'].append(dict(name=name, argv=argv, process=identity(process.pid)))
        return process
    def step(deadline):
        if time.monotonic() >= deadline:
            raise TimeoutError('Bounded phase wall deadline')
        if watchdog.poll() is not None:
            raise RuntimeError('Independent containment watchdog exited')
        monitor.tf_delivery.spin_start()
        try:
            rclpy.spin_once(node, timeout_sec=.02)
        finally:
            monitor.tf_delivery.spin_end()
        if monitor.phase not in ('bootstrap', 'shutdown'):
            monitor.failure = monitor.failure or monitor.physical_veto()
        if getattr(monitor,'planning_only',False) and monitor.final is not None and any(monitor.final):
            monitor.failure=monitor.failure or 'planning_only_nonzero_final'
        monitor.update(); owner.observe()
    def wait(predicate, duration):
        end = time.monotonic()+duration
        while not predicate():
            step(end)
    def parameters(name, fields):
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python
        client=node.create_client(GetParameters, '/'+name+'/get_parameters')
        wait(client.service_is_ready, 5.)
        future=client.call_async(GetParameters.Request(names=fields))
        wait(future.done, 5.)
        result=dict(zip(fields,[parameter_value_to_python(v) for v in future.result().values]))
        node.destroy_client(client)
        return result
    def stop_process(process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            deadline = time.monotonic()+3.
            while process.poll() is None:
                step(deadline)
        if process.returncode not in (0, -signal.SIGINT, 130):
            raise RuntimeError('Precheck helper did not exit cleanly')
    def zero():
        publisher.publish(Twist())
    def rest(duration=30., inject_zero=True):
        evidence=StopEvidence()
        end = time.monotonic()+duration
        while True:
            if inject_zero: zero()
            step(end)
            result=evidence.observe(sim=monitor.clock,wall=time.monotonic(),pose=monitor.truth_pose(),
                stationary=monitor.resting(),zero=monitor.final is not None and all(v==0 for v in monitor.final))
            if result and monitor.contacts.intervals.closed(start=result['start_sim'],end=result['end_sim'],sim=monitor.clock,wall=time.monotonic()):
                return result
    rclpy.init()
    node = rclpy.create_node('warehouse_ab_owner', parameter_overrides=[Parameter('use_sim_time', value=True)])
    try:
        # Domain must be empty before creating any simulation or velocity source.
        until = time.monotonic()+1.
        while time.monotonic()<until:
            rclpy.spin_once(node, timeout_sec=.02)
        if any(n != 'warehouse_ab_owner' for n in node.get_node_names()):
            raise RuntimeError('Domain contains pre-existing nodes')
        inventory = json.loads((ROOT/'artifacts/recovery/collision-inventory.json').read_text())
        if digest(Path(inventory['converted_sdf'])) != inventory['converted_sdf_sha256'] or len(inventory['collisions']) != 26:
            raise RuntimeError('Source collision inventory mismatch')
        monitor = Monitor(node, inventory, heartbeat)
        monitor.planning_only=planning_only
        if envelope is not None:
            monitor.route_tilt_limit=envelope['body_tilt_limit_rad']
            monitor.route_command_cap=envelope['command_caps'][0]
            monitor.route_motion_caps=(envelope['physical_speed_cap_mps'],envelope['physical_yaw_cap_rad_s'])
        monitor.update()
        config = dict(token=token, parent=identity(os.getpid()), server=None, heartbeat=str(heartbeat),
                      result=str(watchdog_result),world=str(output/'observer.world'),wall_limit_s=args.wall_limit,stale_s=1.)
        from warehouse_ab_supervisor import validate_config
        validate_config(config); write_json(output/'watchdog.json', config)
        watchdog_env = dict(os.environ); watchdog_env.pop('B2W_NARROW_OWNER', None)
        watchdog = subprocess.Popen(['/usr/bin/python3', str(ROOT/'scripts/warehouse_ab_supervisor.py'), '--watchdog', str(output/'watchdog.json')],
            env=watchdog_env, cwd=ROOT, start_new_session=True)
        world = output/'observer.world'
        report['world'] = world_copy(ROOT/'third_party/aws_warehouse/warehouse.world', world,
            ROOT/'artifacts/recovery/contact-build/libb2w_contact_observer.so')
        stack = launch(['/usr/bin/python3','/opt/ros/humble/bin/ros2','launch','course_bringup','course.launch.py',
            'mode:=slam','scene:=warehouse','gui:='+str(getattr(args,'gui',False)).lower(),'rviz:=false','headless_rendering:=false',
            'world:='+str(world),'fault_profile:=normal','seed:=0','horizontal_samples:=2000',
            'pretrained_directory:='+str(ROOT/'third_party/b2w_rl_sar')]+
            ([] if spawn_x is None else ['spawn_x:='+str(spawn_x), 'spawn_y:='+str(spawn_y)]), 'stack')
        bridge = launch(contact_bridge(), 'contact-bridge')
        observer = launch(['/usr/bin/python3', str(ROOT/'scripts/observe_warehouse_narrow_failure.py'),
            '--duration', str(args.wall_limit), '--label', args.profile, '--dwb-debug', '--output-root', str(output/'observer')], 'observer')
        projector = launch(['/usr/bin/python3','/opt/ros/humble/bin/ros2','run','course_navigation','map_projector','--ros-args',
            '-p','use_sim_time:=true','-p','size:=[40.0,40.0]','-p','origin:=[-20.0,-20.0]'], 'precheck-map')
        adapter = launch(['/usr/bin/python3','/opt/ros/humble/bin/ros2','run','course_navigation','velocity_adapter','--ros-args',
            '-p','use_sim_time:=true','-p','input_topic:=/warehouse_ab/precheck'], 'precheck-adapter')
        publisher = node.create_publisher(Twist, '/warehouse_ab/precheck', 10)
        if planning_only:
            # Finish cold planner startup while the existing bootstrap waits for
            # live localization. Never add a burst of forks/scans after rest.
            nav_config=ROOT/'course_navigation/config'/('nav2_smac2d_diag.yaml' if args.profile=='A' else 'nav2_smac2d_dwb_slow_diag.yaml')
            launch(['/usr/bin/python3','/opt/ros/humble/bin/ros2','run','nav2_planner','planner_server',
                '--ros-args','--params-file',str(nav_config)],'planning-server')
            launch(['/usr/bin/python3','/opt/ros/humble/bin/ros2','run','nav2_lifecycle_manager','lifecycle_manager',
                '--ros-args','-r','__node:=lifecycle_manager_planning','-p','use_sim_time:=true',
                '-p','autostart:=true','-p',"node_names:=['planner_server']"],'planning-lifecycle')
        # The first obstacle-map publication can precede all ray-cleared space.
        # Await a real free-space observation within the existing startup bound;
        # this is only readiness, never a replacement for the full disk gate.
        wait(lambda: monitor.ready() and monitor.grid is not None and (output/'server-pin.json').exists()
             and (args.stationary_containment or monitor.grid.get('observed_free_present', False)), 85.)
        server = json.loads((output/'server-pin.json').read_text())
        if not args.stationary_containment:
            report['stationary_startup_containment_test'] = pause_server(server, token)
            if not report['stationary_startup_containment_test']['contained']:
                raise RuntimeError('Startup containment test failed')
            # Deliberate startup probe only; never resume an emergency fault.
            if not same_process(server, token):
                raise RuntimeError('Server identity changed during stationary test')
            os.kill(server['pid'], signal.SIGCONT)
            report['stationary_startup_containment_test']['resumed_before_arming'] = True
            previous = monitor.clock
            wait(lambda: monitor.clock>previous+.5 and monitor.ready(), 5.)
        maps = Path(f'/proc/{server["pid"]}/maps').read_text()
        backend = Path('/usr/lib/x86_64-linux-gnu/ign-physics-5/engine-plugins/libignition-physics5-dartsim-plugin.so.5.4.0')
        if str(backend) not in maps:
            raise RuntimeError('Actual server DART backend mapping absent')
        report['actual_server'] = dict(server, backend=str(backend), backend_sha256=digest(backend))
        plugin = ROOT/'artifacts/recovery/contact-build/libb2w_contact_observer.so'
        if str(plugin) not in maps or digest(plugin) != report['world']['plugin_sha256']:
            raise RuntimeError('Actual server contact observer mapping/identity absent')
        report['actual_server']['contact_observer'] = dict(path=str(plugin), sha256=digest(plugin))
        actual_description=parameters('b2w_robot_state_publisher',['robot_description'])['robot_description']
        import hashlib
        if hashlib.sha256(actual_description.encode()).hexdigest()!=digest(ROOT/'artifacts/recovery/robot.urdf'):
            raise RuntimeError('Actual generated robot_description differs from engine-probed source')
        report['actual_server']['robot_description_sha256']=hashlib.sha256(actual_description.encode()).hexdigest()
        report['actual_server']['construction_log_files'] = construction_audit(output)
        for p in children:
            if p.poll() is not None:
                raise RuntimeError('Owned bootstrap process exited')
        report['pre_motion_snapshot'] = dict(clock=monitor.clock, pose=monitor.pose(), contacts=monitor.samples,
            actual_truth_evaluation=monitor.truth_pose(),
            support_feet=sorted(monitor.support_feet), expected_inventory=inventory,
            command_publishers=[i.node_name for i in node.get_publishers_info_by_topic('/path_manager/path_manager_ros/nav_vel')])
        report['pre_motion_snapshot']['native_deployment']=native_snapshot(owner,navigation=False,
            manifest=getattr(args, 'deployment_manifest', None))
        if len(node.get_publishers_info_by_topic('/path_manager/path_manager_ros/nav_vel')) != 1:
            raise RuntimeError('Final adapter source is not unique')
        if len(node.get_publishers_info_by_topic('/warehouse_ab/precheck')) != 1:
            raise RuntimeError('Precheck input source is not exclusive')
        if node.get_publishers_info_by_topic('/goal_pose') or node.get_node_names().count('goal_manager'):
            raise RuntimeError('Goal producer/manager unexpectedly active during precheck')
        if args.precheck_only:
            save_pocket_admission(monitor, output, 'initial')
        if args.precheck_only or args.stationary_containment:
            report['pre_motion_snapshot']['motion_pocket'] = motion_pocket_admission(
                monitor.grid, monitor.pose()[:2], stationary_containment=args.stationary_containment)
        else:
            report['pre_motion_snapshot']['route_admission'] = 'pending_planner_and_swept_footprint'
        if args.precheck_only:
            report['pre_motion_snapshot']['open_pocket_radius_m'] = 1.5
        report['pre_motion_snapshot']['engine_probe'] = json.loads(args.engine_evidence.read_text())
        write_json(output/'pre-motion-snapshot.json', report['pre_motion_snapshot'])
        # Synchronous audits/snapshots leave both cached clock and samples
        # unchanged. A cached ready() can pass without dispatching any callback.
        # Reuse the startup-resume clock window before arming motion; active
        # freshness/physical guards and the independent watchdog stay unchanged.
        if not args.stationary_containment:
            admission_sim, admission_wall = monitor.clock, time.monotonic()
            wait(lambda: monitor.clock > admission_sim+.5 and monitor.ready(), 5.)
            report['post_audit_readmission'] = dict(start_sim=admission_sim,
                end_sim=monitor.clock, wall_s=time.monotonic()-admission_wall,
                required_clock_advance_s=.5)
        else:
            wait(monitor.ready, 5.)
        if not args.stationary_containment:
            reason = monitor.physical_veto()
            if reason:
                raise RuntimeError(reason)
        monitor.phase = 'precheck' if args.precheck_only or args.stationary_containment else 'route_admission'
        report['initial_rest'] = rest()
        if args.stationary_containment:
            # End this session through the independent watchdog itself. No
            # pulses, goals, navigation launch, manual pause or resume here.
            latch_failure(monitor, heartbeat, 'stationary_qualification_injection')
            deadline = time.monotonic()+6.
            while watchdog.poll() is None and time.monotonic()<deadline:
                monitor.tf_delivery.spin_start()
                try:
                    rclpy.spin_once(node, timeout_sec=.02)
                finally:
                    monitor.tf_delivery.spin_end()
            if watchdog.poll() is None:
                raise RuntimeError('Stationary watchdog did not finish within 6 wall seconds')
            result = json.loads(watchdog_result.read_text())
            commands = dict(samples=monitor.final_samples, nonzero=monitor.nonzero_final_samples)
            report['stationary_commands'] = commands
            remaining = owner.scan()
            report['cleanup'] = dict(passed=not remaining, remaining=remaining,
                owned_pid_start_ticks=owner.seen, child_returncodes=[p.poll() for p in children])
            report['success'] = (watchdog.returncode == 1 and not remaining and stationary_passed(
                result, server, commands, report['initial_rest'], monitor.samples))
            report['stationary_containment'] = 'PASS' if report['success'] else 'FAIL'
            return 0 if report['success'] else 1
        if args.precheck_only:
            initial_precheck = monitor.pose(); initial_truth = monitor.truth_pose()
            monitor.precheck_motion_origin = initial_truth
            pulse_s = getattr(args, 'precheck_pulse_s', 2.)
            for kind in ('forward', 'turn'):
                if not save_pocket_admission(monitor, output, kind):
                    raise RuntimeError('Sensor open pocket absent before '+kind)
                start_sim, start_wall = monitor.clock, time.monotonic()
                start_truth = monitor.truth_pose(); start_sensor = monitor.pose()
                command = Twist()
                if kind == 'forward': command.linear.x = .05
                else: command.angular.z = .05
                entry = dict(kind=kind, command=dict(vx=command.linear.x,wz=command.angular.z),
                    start_sim=start_sim,start_sensor=start_sensor,start_truth_evaluation=start_truth,
                    command_publication_attempted=False, requested_pulse_sim_s=pulse_s)
                report['precheck'].append(entry)
                write_json(output/'session-report.json', report)
                # Snapshot/report IO can invalidate the previously checked samples.
                # Dispatch and apply every active guard before the first nonzero
                # publication, within the existing pulse wall deadline.
                before_dispatch_clock = monitor.clock
                step(start_wall+10.)
                while monitor.clock <= before_dispatch_clock:
                    step(start_wall+10.)
                if not open_pocket(monitor.grid, monitor.pose()[:2]):
                    raise RuntimeError('Sensor open pocket absent before '+kind)
                start_sim = monitor.clock
                start_truth = monitor.truth_pose(); start_sensor = monitor.pose()
                entry.update(start_sim=start_sim,start_sensor=start_sensor,start_truth_evaluation=start_truth)
                while monitor.clock-start_sim < pulse_s:
                    entry['command_publication_attempted'] = True
                    publisher.publish(command); step(start_wall+10.)
                    current = monitor.truth_pose(); v = monitor.truth[2].twist.twist
                    excursion = math.dist(start_truth[:2],current[:2])
                    yaw_excursion = abs(math.atan2(math.sin(current[2]-start_truth[2]),math.cos(current[2]-start_truth[2])))
                    if excursion > .25 or yaw_excursion > .35 or math.hypot(v.linear.x,v.linear.y) > .25 or abs(v.angular.z) > .4:
                        raise RuntimeError('Unexpected physical precheck motion')
                    expected = [command.linear.x,0.,0.,0.,0.,command.angular.z]
                    # Before the first bounded command reaches the adapter, zero
                    # is allowed; every nonzero sample must match this sole source.
                    if any(monitor.final) and any(abs(a-b)>1e-6 for a,b in zip(monitor.final, expected)):
                        raise RuntimeError('Unexpected precheck final command')
                zero()
                entry.update(end_pulse_sim=monitor.clock, pulse_wall_s=time.monotonic()-start_wall,
                    rest=rest(), end_sensor=monitor.pose(),end_truth_evaluation=monitor.truth_pose())
                write_json(output/'session-report.json', report)
            report['precheck_finish'] = dict(start_sensor=initial_precheck,end_sensor=monitor.pose(),
                start_truth_evaluation=initial_truth,end_truth_evaluation=monitor.truth_pose(),
                translation_drift_m=math.dist(initial_truth[:2],monitor.truth_pose()[:2]),
                yaw_drift_rad=math.atan2(math.sin(monitor.truth_pose()[2]-initial_truth[2]),math.cos(monitor.truth_pose()[2]-initial_truth[2])))
        if args.precheck_only:
            # Keep the same sole command owner through the final measured stop.
            # Route imports, handover, launch and goals are unreachable here.
            report['normal_rest'] = report['precheck'][-1]['rest']
            report['success'] = True
        else:
            from run_warehouse_route import RosGoalTransport, run_critical, load_route
            def prepare_navigation():
                nonlocal publisher
                if planning_only:
                    # Keep the existing map projector and zero-input adapter.
                    # Launch no command-producing navigation server or manager.
                    monitor.phase='planning'
                    from lifecycle_msgs.srv import GetState
                    from course_navigation.lifecycle_readiness import LifecycleReadiness
                    state_client=node.create_client(GetState,'/planner_server/get_state')
                    lifecycle=LifecycleReadiness([state_client])
                    try:
                        wait(lambda:lifecycle.active(time.monotonic()),30.)
                    finally:
                        report['planning_lifecycle']=dict(states=list(lifecycle.states),active=all(state==3 for state in lifecycle.states))
                        write_json(output/'planning-lifecycle-state.json',report['planning_lifecycle'])
                        node.destroy_client(state_client)
                    runtime=parameters('planner_server',['GridBased.plugin','GridBased.allow_unknown'])
                    if runtime!={'GridBased.plugin':'nav2_smac_planner/SmacPlanner2D','GridBased.allow_unknown':False}:
                        raise RuntimeError('Planning-only Smac configuration mismatch')
                    report['planning_runtime_parameters']=runtime
                    write_json(output/'planning-runtime-parameters.json',runtime)
                    return
                monitor.phase = 'bootstrap'
                node.destroy_publisher(publisher)
                publisher = None
                stop_process(adapter); stop_process(projector)
                wait(lambda: not node.get_publishers_info_by_topic('/path_manager/path_manager_ros/nav_vel'), 3.)
                nav_config = ROOT/'course_navigation/config'/('nav2_smac2d_diag.yaml' if args.profile=='A' else 'nav2_smac2d_dwb_slow_diag.yaml')
                results = output/'goal-manager'; results.mkdir()
                navigation = launch(['/usr/bin/python3','/opt/ros/humble/bin/ros2','launch','course_navigation','navigation.launch.py',
                    'map_mode:=online','map_width:=40.0','map_height:=40.0','nav_params_file:='+str(nav_config),
                    'results_directory:='+str(results),'timeout_s:=120.0','goal_tolerance_m:=0.3'], 'navigation')
                wait(lambda: monitor.ready() and len(node.get_publishers_info_by_topic('/path_manager/path_manager_ros/nav_vel'))==1, 30.)
                monitor.phase='navigation'
                runtime={
                    'controller_server':parameters('controller_server',['FollowPath.plugin','FollowPath.max_vel_x','FollowPath.max_speed_xy','FollowPath.max_vel_y','FollowPath.max_vel_theta','general_goal_checker.xy_goal_tolerance','general_goal_checker.yaw_goal_tolerance']),
                    'planner_server':parameters('planner_server',['GridBased.plugin','GridBased.allow_unknown']),
                    'velocity_smoother':parameters('velocity_smoother',['max_velocity','min_velocity','max_accel','max_decel']),
                    'velocity_adapter':parameters('velocity_adapter',['max_linear','max_angular','timeout_s','input_topic','state_topic']),
                    'b2w_rl_sar_controller':parameters('b2w_rl_sar_controller',['policy_path','command_timeout_s','feedback_timeout_s','yaw_command_gain'])}
                expected_cap=.4 if args.profile=='A' else .2
                if (runtime['controller_server']['FollowPath.max_vel_x']!=expected_cap or
                    runtime['controller_server']['FollowPath.max_speed_xy']!=.4 or
                    runtime['controller_server']['FollowPath.plugin']!='dwb_core::DWBLocalPlanner' or
                    runtime['planner_server']['GridBased.plugin']!='nav2_smac_planner/SmacPlanner2D' or
                    runtime['planner_server']['GridBased.allow_unknown'] is not False or
                    runtime['velocity_smoother']['max_velocity']!=[.4,0.,.6]):
                    raise RuntimeError('Loaded A/B Nav2 configuration differs from pinned profile')
                report['runtime_parameters']=runtime; write_json(output/'runtime-parameters.json',runtime)
            if planning_only:
                report['planning'] = route_start_admission(node,monitor,output,wait,prepare_navigation,planning_only=True,deferred_capture=deferred_capture)
                report['normal_rest']=rest()
                report['success']=report['planning']['planning_completed']
            else:
                report['route_start_admission'] = route_start_admission(
                    node, monitor, output, wait, prepare_navigation, envelope=envelope)
                class GuardedTransport(RosGoalTransport):
                    initial_admission_consumed=False
                    def send_and_wait(self,pose,on_sent):
                        video=getattr(args,'video_ready_file',None)
                        if video is not None:
                            from record_warehouse_video import is_ready
                            wait(lambda:is_ready(video,session_start_wall),60. if not self.initial_admission_consumed else 1.)
                        if not self.initial_admission_consumed:
                            # Refresh the actual plan after transport discovery
                            # and synchronous native audits, before the first goal.
                            route_start_admission(node,monitor,output,wait,lambda:None,
                                envelope=envelope,owned_goal_producer=self.node.get_name())
                            self.initial_admission_consumed=True
                        return super().send_and_wait(pose,on_sent)
                    def start(self):
                        result=super().start()
                        report['navigation_native_deployment']=native_snapshot(owner,navigation=True,
                            manifest=getattr(args, 'deployment_manifest', None))
                        write_json(output/'navigation-native-deployment.json',report['navigation_native_deployment'])
                        return result
                    def spin(self, deadline):
                        super().spin(deadline)
                        step(deadline)
                transport = GuardedTransport(ready_timeout=30., evidence_directory=output/'route', reuse_existing_context=True,
                    progress_callback=lambda:step(time.monotonic()+1.))
                (output/'route').mkdir()
                def checkpoint(value):
                    write_json(output/'route'/'report.json', value)
                report['route'] = run_critical(load_route(), execute=True, transport_factory=lambda:transport,checkpoint=checkpoint,
                    mode=getattr(args, 'route_mode', 'critical'), continuation_yaws=getattr(args, 'continuation_yaws', None))
                report['normal_rest'] = rest(inject_zero=False)
                report['success'] = bool(report['route']['success'])
        monitor.normal_rest = report['normal_rest']; monitor.phase='shutdown'; monitor.update()
        for process in reversed(children):
            if process.poll() is None: process.send_signal(signal.SIGINT)
        end = time.monotonic()+8.
        while owner.scan() and time.monotonic()<end:
            step(end)
            for process in children: process.poll()
        remaining=owner.scan()
        report['cleanup'] = dict(passed=not remaining,remaining=remaining,owned_pid_start_ticks=owner.seen)
        if remaining: raise RuntimeError('Normal owned cleanup incomplete')
        write_json(heartbeat, dict(wall=time.monotonic(),clock_advanced_wall=monitor.clock_wall,
            phase='shutdown',failure=None,complete=True,normal_rest=report['normal_rest'],cleanup_passed=True))
        watchdog.wait(timeout=2.)
        if watchdog.returncode != 0: raise RuntimeError('Watchdog rejected normal completion')
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = repr(error)
        if monitor is not None:
            wall = time.monotonic()
            if not monitor.select_truth(wall): monitor.record_truth_fault(wall)
            monitor.record_readiness_fault()
            latch_failure(monitor,heartbeat,report['error'])
        # Latch containment first; send zero only through the existing owner
        # input. This does not assert final delivery or physical stopping.
        if publisher is not None:
            report['fault_stop'] = dict(request_wall=time.monotonic(), zero_input_published=False, physical_rest_proven=False)
            try:
                zero()
                report['fault_stop']['zero_input_published'] = True
            except Exception as stop_error:
                report['fault_stop']['error'] = repr(stop_error)
    finally:
        # Normal completion is written only after measured rest and owned shutdown.
        # Any unfinished/error attempt remains under independent containment.
        # Let independent containment pause physics and drain the stop output
        # before owner cleanup can terminate the adapter/observer. Still bounded
        # if the watchdog itself fails; owned cleanup remains the fallback.
        if watchdog is not None:
            deadline = time.monotonic()+3.
            while watchdog.poll() is None and time.monotonic()<deadline:
                time.sleep(.02)
            if watchdog.poll() is None:
                report['watchdog_pending'] = True
        if children and not report.get('cleanup',{}).get('passed'):
            report['cleanup'] = owner.cleanup(children[0])
            for p in children:
                try: p.wait(timeout=1.)
                except subprocess.TimeoutExpired: pass
        if monitor is not None:
            report['truth_first_fault'] = monitor.truth_first_fault
            report['readiness_first_fault'] = monitor.readiness_first_fault
            report['contact_failure'] = monitor.contacts.failure
            report['contact_sample_count'] = monitor.samples
            try:
                monitor.ingress.close()
                report['localization_cleanup'] = dict(passed=True)
            except Exception as close_error:
                report['localization_cleanup'] = dict(passed=False, error=repr(close_error))
                report['error'] = report.get('error') or repr(close_error)
            monitor.events.close()
        if watchdog_result.is_file(): report['watchdog']=json.loads(watchdog_result.read_text())
        if report.get('error') or not report.get('cleanup',{}).get('passed'):
            report['success']=False
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        for stream in logs: stream.close()
        if deferred_capture and report.get('cleanup',{}).get('passed') and not report.get('watchdog_pending'):
            try:
                report['planning_geometry']=finish_planning_capture(output,deferred_capture)
            except Exception as analysis_error:
                report['planning_analysis_error']=repr(analysis_error);report['success']=False
        write_json(output/'session-report.json', report)
    return 0 if report['success'] else 1


def check_inputs(args):
    """Offline/read-only checks. No ROS node, simulator, goal or velocity."""
    import yaml
    from ament_index_python.packages import get_package_prefix
    if Path(get_package_prefix('course_navigation')).resolve()!=ROOT/'install/course_navigation':
        raise ValueError('Source only this isolated checkout install')
    profiles=[yaml.safe_load((ROOT/'course_navigation/config'/name).read_text()) for name in
        ('nav2_smac2d_diag.yaml','nav2_smac2d_dwb_slow_diag.yaml')]
    a,b=profiles
    if a['controller_server']['ros__parameters']['FollowPath']['max_vel_x']!=.4 or b['controller_server']['ros__parameters']['FollowPath']['max_vel_x']!=.2:
        raise ValueError('Invalid sole A/B change')
    import copy
    normalized=copy.deepcopy(b);normalized['controller_server']['ros__parameters']['FollowPath']['max_vel_x']=.4
    if normalized!=a:raise ValueError('More than one A/B parameter changed')
    inventory=json.loads((ROOT/'artifacts/recovery/collision-inventory.json').read_text())
    source=Path(inventory['converted_sdf'])
    if digest(source)!=inventory['converted_sdf_sha256'] or len(inventory['collisions'])!=26:
        raise ValueError('Collision source mismatch')
    backend=Path('/usr/lib/x86_64-linux-gnu/ign-physics-5/engine-plugins/libignition-physics5-dartsim-plugin.so.5.4.0')
    if digest(backend)!='44e5d8573b44ef1f555eba6f2fbf01c1f0ab657b0fd32016c441e884ec5e71a1':
        raise ValueError('Engine-probed backend identity changed')
    probe=ROOT/'artifacts/contact-engine-probe/contact_engine_probe'
    checked=subprocess.run([str(probe),str(source),str(ROOT/'artifacts/recovery/collision-inventory.json'),str(backend)],
        check=True,timeout=10.,capture_output=True,text=True)
    actual=json.loads(checked.stdout)
    if (actual.get('result')!='PASS' or actual.get('constructed_shape_handles')!=26 or
            sorted(s['name'] for s in actual.get('shapes',[]) if s.get('nonnull_shape_handle') is True)!=sorted(inventory['collisions'])):
        raise ValueError('Offline engine construction evidence does not match source')
    saved=json.loads(args.engine_evidence.read_text())
    if saved!=actual:raise ValueError('Engine evidence differs from fresh no-step diagnostic')
    manifest_path=getattr(args,'deployment_manifest',ROOT/'artifacts/recovery/root-installed-manifest.json')
    manifest=json.loads(manifest_path.read_text())
    historical=json.loads((ROOT/'artifacts/recovery/root-installed-manifest.json').read_text())
    scope=lambda value:[(i['source'],i['installed']) for i in value['source_install']]
    if scope(manifest)!=scope(historical) or manifest['native']!=historical['native']:
        raise ValueError('Deployment manifest cannot omit identities or change native/source path scope')
    for item in manifest['source_install']:
        if digest(Path(item['source']))!=item['source_sha256'] or digest(Path(item['installed']))!=item['installed_sha256']:
            raise ValueError('Source/install pin changed: '+item['source'])
    for path,metadata in manifest['native'].items():
        if digest(Path(path))!=metadata['sha256']:
            raise ValueError('Native pin changed: '+path)
    reviewed_envelope=None
    if getattr(args,'route_envelope',None) is not None:
        from warehouse_route_admission import load_envelope
        config=ROOT/'course_navigation/config'/('nav2_smac2d_diag.yaml' if args.profile=='A' else 'nav2_smac2d_dwb_slow_diag.yaml')
        load_envelope(args.route_envelope,args.route_envelope_sha256,digest(ROOT/'artifacts/recovery/robot.urdf'),
            digest(config),[.4 if args.profile=='A' else .2,.6],digest(args.deployment_manifest))
        reviewed_envelope=dict(path=str(args.route_envelope),sha256=args.route_envelope_sha256)
    domain=os.environ.get('ROS_DOMAIN_ID')
    if not domain or not domain.isdigit() or not 0<=int(domain)<=232 or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise ValueError('Explicit unused ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1 required')
    return dict(profile=args.profile,head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        source_install_pins=len(manifest['source_install']),engine_shape_handles=26,
        route_admission_source_pins={str(path.relative_to(ROOT)):digest(path) for path in
            (ROOT/'scripts/warehouse_ab_trial.py', ROOT/'scripts/warehouse_route_admission.py',ROOT/'scripts/record_warehouse_video.py',
             ROOT/'artifacts/recovery/robot.urdf')},
        profile_sha256={name:digest(ROOT/'course_navigation/config'/name) for name in
            ('nav2_smac2d_diag.yaml','nav2_smac2d_dwb_slow_diag.yaml')},
        engine_evidence_sha256=digest(args.engine_evidence),backend_sha256=digest(backend),
        deployment_manifest=str(manifest_path),deployment_manifest_sha256=digest(manifest_path),
        precheck_pulse_sim_s=getattr(args,'precheck_pulse_s',2.),
        no_world_or_motion_started=True,domain=int(domain),wall_limit_s=args.wall_limit,
        reviewed_envelope=reviewed_envelope,
        stationary_containment=args.stationary_containment, precheck_only=args.precheck_only,
        planning_only=getattr(args,'planning_only',False),
        motion_authorization=args.authorize_motion, route_mode=args.route_mode,
        continuation_yaws=args.continuation_yaws,
        requested_precheck_spawn=None if args.precheck_spawn_x is None else dict(x=args.precheck_spawn_x,y=args.precheck_spawn_y))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',choices=('A','B'),required=True)
    parser.add_argument('--execute',action='store_true',help='Consume this mode/profile once; start one bounded owned session')
    parser.add_argument('--check',action='store_true',help='Read-only/offline check (also default)')
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--stationary-containment',action='store_true',
        help='Zero-only qualification; end through watchdog containment before any pulse or goal')
    modes.add_argument('--precheck-only',action='store_true',
        help='Forward/rest/turn/rest only; no Nav2 launch or navigation goals')
    modes.add_argument('--planning-only',action='store_true',help='Canonical stationary Smac capture only; no route goals or qualification pulses')
    parser.add_argument('--authorize-planning',action='store_true',help='Separate authority for one bounded planning-only startup')
    parser.add_argument('--precheck-spawn-x',type=float,help='Explicit world X for precheck-only; requires Y')
    parser.add_argument('--precheck-spawn-y',type=float,help='Explicit world Y for precheck-only; requires X')
    parser.add_argument('--precheck-pulse-s',type=float,default=2.,help='Positive pulse cap in simulation seconds, at most 2')
    parser.add_argument('--route-mode',choices=('critical','full'),default='critical')
    parser.add_argument('--continuation-yaws',type=float,nargs=3,metavar=('G3','G4','G5'),
        help='Explicit map-frame radians required for full route; no inferred headings')
    parser.add_argument('--authorize-motion',choices=('precheck-only','route','route-full'),
        help='Fresh explicit motion authority matching this invocation; requires prior stationary qualification')
    parser.add_argument('--gui',action='store_true',help='Show the owned Gazebo GUI for continuous capture')
    parser.add_argument('--video-ready-file',type=Path,help='Wait for this live capture ledger before the first route goal; require it for every goal')
    parser.add_argument('--route-envelope',type=Path,help='Reviewed operational footprint and directional stopping evidence JSON')
    parser.add_argument('--route-envelope-sha256',help='Expected SHA256 from the independent envelope review')
    parser.add_argument('--output-root',type=Path,default=ROOT/'artifacts/trials')
    parser.add_argument('--engine-evidence',type=Path,default=ROOT/'artifacts/contact-engine-probe/result.json')
    parser.add_argument('--deployment-manifest',type=Path,default=ROOT/'artifacts/recovery/root-installed-manifest.json',
        help='Reviewed source/install/native identity manifest; historical default is preserved')
    args=parser.parse_args(argv);args.wall_limit=150. if args.stationary_containment else 420.
    if not math.isfinite(args.precheck_pulse_s) or not 0 < args.precheck_pulse_s <= 2.:
        parser.error('--precheck-pulse-s must be finite and in (0,2]')
    if args.check and args.execute:parser.error('--check and --execute are mutually exclusive')
    if args.stationary_containment and args.authorize_motion:
        parser.error('Stationary qualification cannot accept motion authorization')
    if args.authorize_planning and not args.planning_only:
        parser.error('--authorize-planning requires --planning-only')
    if args.planning_only and (args.authorize_motion or args.route_mode!='critical' or args.continuation_yaws):
        parser.error('Planning-only rejects motion authority and route continuation')
    if args.planning_only and args.execute and not args.authorize_planning:
        parser.error('Planning-only execution requires separate --authorize-planning')
    if args.video_ready_file and (not args.gui or args.planning_only or args.precheck_only or args.stationary_containment):
        parser.error('Video readiness belongs to GUI route mode')
    if bool(args.route_envelope)!=bool(args.route_envelope_sha256):
        parser.error('Route envelope requires its reviewed SHA256')
    if args.route_envelope and (args.planning_only or args.precheck_only or args.stationary_containment):
        parser.error('Operational envelope belongs only to route motion mode')
    spawn=(args.precheck_spawn_x,args.precheck_spawn_y)
    if any(v is not None for v in spawn):
        if not args.precheck_only or any(v is None or not math.isfinite(v) for v in spawn):
            parser.error('Alternate spawn requires --precheck-only and a finite X/Y pair')
    if args.route_mode=='full':
        if args.precheck_only or args.stationary_containment:
            parser.error('Full route cannot be combined with precheck-only or stationary')
        if args.continuation_yaws is None or not all(math.isfinite(v) for v in args.continuation_yaws):
            parser.error('Full route requires three finite --continuation-yaws')
    elif args.continuation_yaws is not None:
        parser.error('--continuation-yaws requires --route-mode full')
    motion_mode='precheck-only' if args.precheck_only else ('route-full' if args.route_mode=='full' else 'route')
    if args.execute and not args.stationary_containment and not args.planning_only and args.authorize_motion != motion_mode:
        parser.error('--execute requires fresh --authorize-motion '+motion_mode+' after stationary qualification')
    if args.execute and not (args.planning_only or args.precheck_only or args.stationary_containment) and not args.route_envelope:
        parser.error('Route motion requires a reviewed --route-envelope and --route-envelope-sha256')
    try:
        checked=check_inputs(args)
        if not args.execute:
            print(json.dumps(checked,indent=2));return 0
        output_root=args.output_root.resolve();output_root.mkdir(parents=True,exist_ok=True)
        # Exclusive mode/profile attempt accounting; failures consume the attempt.
        attempt = 'planning' if args.planning_only else ('stationary' if args.stationary_containment else ('precheck' if args.precheck_only else args.profile))
        fd=os.open(output_root/(attempt+'.used'),os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
        try:os.write(fd,json.dumps(dict(profile=args.profile,pid=os.getpid(),wall=time.monotonic(),
            motion_authorization=args.authorize_motion)).encode())
        finally:os.close(fd)
        output=output_root/(attempt+'-'+uuid.uuid4().hex);output.mkdir(mode=0o700)
        write_json(output/'preflight.json',checked)
        print('Owned session evidence: '+str(output),flush=True)
        return execute(args,output)
    except (Exception,KeyboardInterrupt) as error:
        print('A/B trial rejected: '+repr(error),flush=True);return 2


if __name__=='__main__':raise SystemExit(main())
