#!/usr/bin/env python3
"""Attach-only warehouse observer. No launch/action/command/parameter mutations.

Run from a sourced ROS Humble terminal in the same domain as the manual GUI.
Ctrl+C ends only this observer and writes summary.json. Duration is WALL time.
"""
import argparse
from collections import Counter, defaultdict, deque
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import time

from warehouse_observer_helpers import (ClockEpoch, align, classify_interval,
    footprint_geometry, relative_footprint, navigation_state, stationary_intervals, topic_health,
    MeasurementClock, measurement_health, observed_age, precursor_chronology)

# Names come from navigation.launch.py, run_narrow_baseline.py, INTERFACE.md,
# and Nav2 1.1.20 DWBPublisher. Subscribe only after graph/type verification.
TOPICS = {
    '/clock': 'rosgraph_msgs/msg/Clock',
    '/goal_pose': 'geometry_msgs/msg/PoseStamped',
    '/plan': 'nav_msgs/msg/Path',
    '/course_navigation/cmd_vel_raw': 'geometry_msgs/msg/Twist',
    '/course_navigation/cmd_vel': 'geometry_msgs/msg/Twist',
    '/path_manager/path_manager_ros/nav_vel': 'geometry_msgs/msg/Twist',
    '/slam/odom': 'nav_msgs/msg/Odometry',
    '/sim/ground_truth/odom': 'nav_msgs/msg/Odometry',
    '/collision_plate/force_torque': 'geometry_msgs/msg/WrenchStamped',
    '/navigate_to_pose/_action/status': 'action_msgs/msg/GoalStatusArray',
    '/follow_path/_action/status': 'action_msgs/msg/GoalStatusArray',
    '/behavior_tree_log': 'nav2_msgs/msg/BehaviorTreeLog',
    '/rosout': 'rcl_interfaces/msg/Log',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
    '/local_costmap/costmap_raw': 'nav2_msgs/msg/Costmap',
    '/global_costmap/costmap_raw': 'nav2_msgs/msg/Costmap',
    '/local_costmap/published_footprint': 'geometry_msgs/msg/PolygonStamped',
    '/global_costmap/published_footprint': 'geometry_msgs/msg/PolygonStamped',
    '/network_input_debug': 'std_msgs/msg/Float64MultiArray',
    '/joint_commands': 'std_msgs/msg/Float64MultiArray',
    '/joint_torques': 'std_msgs/msg/Float64MultiArray',
    '/joint_states': 'sensor_msgs/msg/JointState',
}
DWB_DEBUG_TOPICS = {
    '/evaluation': 'dwb_msgs/msg/LocalPlanEvaluation',
    '/received_global_plan': 'nav_msgs/msg/Path',
    '/transformed_global_plan': 'nav_msgs/msg/Path',
    '/local_plan': 'nav_msgs/msg/Path',
}
SIGNALS = {
    '/course_navigation/cmd_vel_raw': 'raw',
    '/course_navigation/cmd_vel': 'smooth',
    '/path_manager/path_manager_ros/nav_vel': 'final',
    '/slam/odom': 'slam', '/sim/ground_truth/odom': 'truth',
    '/collision_plate/force_torque': 'force_n',
}
PARAMETER_NODES = ['controller_server', 'planner_server', 'velocity_smoother',
                   'velocity_adapter', 'local_costmap/local_costmap',
                   'global_costmap/global_costmap', 'b2w_rl_sar_controller']


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def expected_topics(dwb_debug=False):
    return {**TOPICS, **(DWB_DEBUG_TOPICS if dwb_debug else {})}


def write_final_reports(output, manifest, summary, health):
    """Include report bytes themselves; converge after decimal size changes."""
    health['output_directory_bytes'] = 0
    manifest['measurement_health'] = summary['measurement_health'] = health
    for _ in range(10):
        write_json(output/'manifest.json', finite_json(manifest))
        write_json(output/'summary.json', finite_json(summary))
        size = sum(p.stat().st_size for p in output.rglob('*') if p.is_file())
        if health['output_directory_bytes'] == size:
            return
        health['output_directory_bytes'] = size
    raise RuntimeError('Final output directory byte count did not converge')


def finite_json(value):
    """Keep corrupt numeric samples explicit and JSON portable."""
    if isinstance(value, float) and not math.isfinite(value): return None
    if isinstance(value, bytes): return list(value)
    if isinstance(value, dict): return {k: finite_json(v) for k,v in value.items()}
    if isinstance(value, (list, tuple)): return [finite_json(v) for v in value]
    return value


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def stamp_s(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', default='warehouse', help='Run label, e.g. navfn or smac2d')
    parser.add_argument('--output-root', type=Path, default=Path('/tmp/b2w-warehouse-observer'))
    parser.add_argument('--duration', type=float, default=300., help='Wall seconds; 0 waits for Ctrl+C')
    parser.add_argument('--dwb-debug', action='store_true',
                        help='Opt in to expensive DWB evaluation and debug plan subscriptions; default is LIGHT')
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    topics = expected_topics(args.dwb_debug)
    mode = 'dwb-debug' if args.dwb_debug else 'light'
    # Delayed imports let --help and all pure unit tests work without ROS.
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from rcl_interfaces.srv import GetParameters, ListParameters
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from tf2_ros import Buffer, TransformException

    label = ''.join(c if c.isalnum() or c in '-_' else '_' for c in args.label)
    output = args.output_root / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')+'-'+label)
    output.mkdir(parents=True, exist_ok=False)
    (output/'costmaps').mkdir()
    (output/'context').mkdir()
    events_file = (output/'events.jsonl').open('w')
    samples_file = (output/'samples.jsonl').open('w')
    rclpy.init()
    # No rosout publisher or parameter services on this observer. It has only
    # explicit subscriptions and List/GetParameters clients (never SetParameters).
    # rclpy creates its own parameter-event housekeeping publisher at Node init.
    node = Node('warehouse_narrow_observer', enable_rosout=False, start_parameter_services=False)
    tf = Buffer(cache_time=Duration(seconds=30.))
    clock = ClockEpoch(); sim = None; counts = Counter(); graph = {}; errors = []
    measurement_clock = MeasurementClock()
    history = defaultdict(lambda: deque(maxlen=1000)); rows = []; logs = []
    statuses = {}; latest = {}; subs = {}; clients = []; futures = []
    publisher_presence = {}; pending_costmaps = []; last_pose_completion=0.; footprint_history = defaultdict(lambda: deque(maxlen=100))
    runtime = {}; param_attempts = {}; seq = 0
    started = time.monotonic(); stop_reason = 'duration'; last_sample = None

    def emit(topic, value, stamp=None):
        nonlocal seq
        seq += 1
        record = dict(seq=seq, topic=topic, epoch=clock.epoch,
                      sim_receive_s=sim, wall_monotonic_s=time.monotonic(),
                      wall_utc=datetime.now(timezone.utc).isoformat(), stamp_s=stamp,
                      value=finite_json(value))
        events_file.write(json.dumps(record, allow_nan=False)+'\n')
        return seq

    def remember(key, value, stamp=None, frame=None):
        if sim is None: return
        history[key].append(dict(t=sim if stamp is None else stamp, epoch=clock.epoch,
                                 receive_t=sim, value=value, frame=frame, seq=seq))

    def callback(topic, msg):
        nonlocal sim
        counts[topic] += 1
        if topic == '/clock':
            sim = stamp_s(msg.clock); old = clock.epoch; clock.update(sim)
            measurement_clock.update(sim)
            if old != clock.epoch:
                tf.clear(); latest.clear(); statuses.clear(); history.clear(); pending_costmaps.clear(); footprint_history.clear()
                emit('clock_reset', dict(old_epoch=old, new_epoch=clock.epoch))
            emit(topic, dict(clock_s=sim), sim); return
        stamp = stamp_s(msg.header.stamp) if hasattr(msg, 'header') else (
                stamp_s(msg.stamp) if topic == '/rosout' else None)
        if topic.endswith('/costmap_raw'):
            raw = bytes(msg.data); sha = hashlib.sha256(raw).hexdigest()
            raw_path = output/'costmaps'/f'{sha}.bin'
            if not raw_path.exists(): raw_path.write_bytes(raw)
            value = dict(header=message_to_ordereddict(msg.header),
                         metadata=message_to_ordereddict(msg.metadata), raw_file=str(raw_path.relative_to(output)),
                         sha256=sha, bytes=len(raw))
            ident = emit(topic, value, stamp)
            meta_path = output/'costmaps'/f'{ident:07d}.json'; write_json(meta_path, value)
            pending_costmaps.append(dict(file=meta_path,value=value, topic=topic, stamp_s=stamp, epoch=clock.epoch,wall_received=time.monotonic()))
            latest[topic] = dict(seq=ident, t=sim, stamp_s=stamp,
                                 file=str(meta_path.relative_to(output)), value=value)
            return
        if topic == '/evaluation':
            # Every evaluation: all sampled velocities/totals/critic scores and
            # rejection critics; retain full predicted poses only for best valid.
            # Short-circuited positive scores are not necessarily fully evaluated.
            rejected = Counter()
            best = None
            scores = []
            for i, candidate in enumerate(msg.twists):
                v=candidate.traj.velocity
                scores.append(dict(velocity=[v.x,v.y,v.theta], total=candidate.total,
                    scores=[dict(name=s.name,raw_score=s.raw_score,scale=s.scale) for s in candidate.scores]))
                if candidate.total < 0:
                    rejected.update(s.name for s in candidate.scores if s.raw_score < 0)
                elif i == msg.best_index:
                    best = message_to_ordereddict(candidate)
            value = dict(header=message_to_ordereddict(msg.header), trajectories=len(msg.twists),
                         invalid=sum(s.total<0 for s in msg.twists), invalid_by_critic=dict(rejected),
                         best_index=msg.best_index, best=best, trajectory_scores=scores,
                         predicted_poses_for_nonbest_omitted=True)
        else:
            value = message_to_ordereddict(msg)
        if topic == '/rosout' and not (any(k in msg.name.lower() for k in
                ['controller','dwb','bt_navigator','progress','velocity_adapter','rl_sar','planner']) or
                any(k in msg.msg.lower() for k in ['no valid trajectories','failed to make progress','controller patience'])):
            return
        ident = emit(topic, value, stamp)
        if topic.endswith('/published_footprint'):
            footprint_history[topic].append(dict(t=stamp,epoch=clock.epoch,value=value,seq=ident))
        if topic in SIGNALS:
            key = SIGNALS[topic]
            if key in ['raw','smooth','final']:
                v = [msg.linear.x,msg.linear.y,msg.angular.z]; remember(key,v)
            elif key in ['truth','slam']:
                p,q=msg.pose.pose.position,msg.pose.pose.orientation
                v=[p.x,p.y,yaw(q)]
                # Bad orientations and out-of-epoch stamps cannot become motion evidence.
                if (q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w < .5 or
                    not all(math.isfinite(x) for x in v)): v=None
                remember(key,v,stamp,msg.header.frame_id)
            else:
                f=msg.wrench.force; remember(key,math.sqrt(f.x*f.x+f.y*f.y+f.z*f.z),stamp)
        elif topic.endswith('/_action/status'):
            statuses[topic] = value
        elif topic in ['/tf','/tf_static']:
            for tr in msg.transforms:
                try:
                    if topic=='/tf_static': tf.set_transform_static(tr,'recorded_tf')
                    else: tf.set_transform(tr,'recorded_tf')
                except Exception as error: errors.append(dict(topic=topic,error=str(error)))
        elif topic == '/rosout':
            name = msg.name.lower(); text = msg.msg.lower()
            if any(k in name for k in ['controller','dwb','bt_navigator','progress','velocity_adapter','rl_sar','planner']) or any(
                    k in text for k in ['no valid trajectories','failed to make progress','controller patience']):
                chronology_valid=sim is not None and stamp is not None and abs(stamp-sim)<=1.
                logs.append(dict(t=stamp if chronology_valid else sim, epoch=clock.epoch,
                                 chronology_valid=chronology_valid,timestamp_basis='simulation_header' if chronology_valid else 'receive_only',
                                 source_stamp_s=stamp, receive_t=sim, name=msg.name, level=int.from_bytes(msg.level,'little') if isinstance(msg.level,bytes) else msg.level,
                                 text=msg.msg, seq=ident))
        if topic in ['/plan','/received_global_plan','/goal_pose','/behavior_tree_log'] or topic.endswith('/published_footprint'):
            context_file=output/'context'/f'{ident:07d}.json'
            write_json(context_file,value)
            latest[topic]=dict(seq=ident,t=sim,stamp_s=stamp,file=str(context_file.relative_to(output)))

    def parameters(prefix):
        # Discover exact read-only service endpoints instead of invoking CLI or
        # making guessed parameter calls on absent nodes.
        available=dict(node.get_service_names_and_types())
        list_name='/'+prefix+'/list_parameters'; get_name='/'+prefix+'/get_parameters'
        if 'rcl_interfaces/srv/ListParameters' not in available.get(list_name,[]) or 'rcl_interfaces/srv/GetParameters' not in available.get(get_name,[]): return
        param_attempts[prefix]=time.monotonic()
        lc=node.create_client(ListParameters,list_name); gc=node.create_client(GetParameters,get_name)
        clients.extend([lc,gc]); pending=lc.call_async(ListParameters.Request()); futures.append(pending)
        def listed(f):
            try:
                names=f.result().result.names
                request=GetParameters.Request(); request.names=names
                future=gc.call_async(request); futures.append(future)
                def got(g):
                    try:
                        values=g.result().values
                        runtime[prefix]=dict(names=names, values={n:message_to_ordereddict(v) for n,v in zip(names,values)},
                                             sim_receive_s=sim, wall_monotonic_s=time.monotonic())
                        write_json(output/'runtime-parameters.json',runtime)
                    except Exception as error: errors.append(dict(service=get_name,error=str(error)))
                future.add_done_callback(got)
            except Exception as error: errors.append(dict(service=list_name,error=str(error)))
        pending.add_done_callback(listed)

    def discover():
        nonlocal graph
        graph=dict(node.get_topic_names_and_types(no_demangle=False))
        for topic,kind in topics.items():
            info=node.get_publishers_info_by_topic(topic) if kind in graph.get(topic,[]) else []
            publisher_presence[topic]=bool(info)
            if not info:
                if topic.endswith('/_action/status'): statuses.pop(topic,None)
                continue
            # Match reliability to all offers; bridge sensor topics may need
            # BEST_EFFORT. Request durability only when every current publisher offers transient data.
            durability=(DurabilityPolicy.TRANSIENT_LOCAL if all(i.qos_profile.durability==DurabilityPolicy.TRANSIENT_LOCAL for i in info)
                        else DurabilityPolicy.VOLATILE)
            reliability=(ReliabilityPolicy.RELIABLE if all(i.qos_profile.reliability==ReliabilityPolicy.RELIABLE for i in info)
                         else ReliabilityPolicy.BEST_EFFORT)
            signature=(durability, reliability, tuple(sorted((i.node_namespace,i.node_name,tuple(i.endpoint_gid)) for i in info)))
            if topic in subs and subs[topic][1]==signature: continue
            if topic in subs: node.destroy_subscription(subs[topic][0])
            if topic.endswith('/_action/status'): statuses.pop(topic,None)
            qos=QoSProfile(depth=100, reliability=reliability,durability=durability)
            try:
                sub=node.create_subscription(get_message(kind),topic,lambda m,t=topic:callback(t,m),qos)
                subs[topic]=(sub,signature)
                emit('subscription',dict(topic=topic,type=kind,durability=str(durability),reliability=str(reliability),
                                         publishers=[dict(node=i.node_name,namespace=i.node_namespace) for i in info]))
            except Exception as error: errors.append(dict(topic=topic,error=str(error)))
        emit('graph',dict(topics=graph,services=dict(node.get_service_names_and_types())))
        for prefix in PARAMETER_NODES:
            if prefix not in runtime and time.monotonic()-param_attempts.get(prefix,-100)>15: parameters(prefix)

    def pose_tf(frame, stamp=None):
        if stamp is not None and stamp<=0: return None
        try:
            tr=tf.lookup_transform(frame,'base_link',Time() if stamp is None else Time(seconds=stamp))
            p,q=tr.transform.translation,tr.transform.rotation
            ts=stamp_s(tr.header.stamp)
            return dict(frame=frame,stamp_s=ts,xyyaw=[p.x,p.y,yaw(q)], age_s=None if sim is None else sim-ts)
        except TransformException: return None

    def complete_costmap_poses():
        nonlocal last_pose_completion
        if time.monotonic()-last_pose_completion < .1: return
        last_pose_completion=time.monotonic()
        # TF can lag a freshly published costmap. Retry read-only lookups while
        # retaining every master (not just latest); never substitute a latest
        # pose for a failed exact-stamp transform.
        for pending in list(pending_costmaps):
            if pending['epoch']!=clock.epoch or time.monotonic()-pending['wall_received']>15.:
                pending_costmaps.remove(pending); continue
            cm=pending['value']; frame=cm['header']['frame_id']; changed=False
            pose=cm.get('pose_at_costmap_stamp') or pose_tf(frame,pending['stamp_s'])
            if pose is None: continue
            if 'pose_at_costmap_stamp' not in cm:
                cm['pose_at_costmap_stamp']=pose; changed=True
            ft_topic=pending['topic'].replace('costmap_raw','published_footprint')
            fp=align(footprint_history[ft_topic],pending['stamp_s'],clock.epoch,.75)
            if fp and fp['value']['header']['frame_id']==frame:
                fp_pose=pose_tf(frame,fp['t'])
                if fp_pose:
                    points=[[p['x'],p['y']] for p in fp['value']['polygon']['points']]
                    cm['padded_footprint_at_costmap_stamp']=relative_footprint(points,fp_pose['xyyaw']); changed=True
                    cm['footprint_source']=dict(topic=ft_topic,seq=fp['seq'],stamp_s=fp['t'],
                        time_skew_s=pending['stamp_s']-fp['t'],already_padded=True,
                        shape_assumption='Shape constant across recorded <=0.75s footprint/costmap skew; transforms matched at both stamps.')
            # Keep retrying for footprint until enough matching TF arrives.
            if changed: write_json(pending['file'],cm)
            if 'padded_footprint_at_costmap_stamp' in cm: pending_costmaps.remove(pending)

    def sample():
        nonlocal last_sample
        if sim is None or last_sample is not None and last_sample[0]==clock.epoch and sim-last_sample[1]<.1: return
        r=dict(t=sim,epoch=clock.epoch,wall_monotonic_s=time.monotonic(),active=False,goal_ids=[], ages={}, observed_ages={}, evidence_seq={})
        for key in SIGNALS.values():
            matched=align(history[key],sim,clock.epoch,.3 if key in ['raw','smooth','final'] else .5)
            r[key]=matched['value'] if matched else None
            r['ages'][key]=sim-matched['t'] if matched else None
            r['observed_ages'][key]=observed_age(history[key],sim,clock.epoch)
            r['evidence_seq'][key]=matched['seq'] if matched else None
            if key in ['truth','slam']: r[key+'_frame']=matched['frame'] if matched else None
        status=statuses.get('/navigate_to_pose/_action/status')
        nav_state=navigation_state(status,publisher_presence.get('/navigate_to_pose/_action/status',False))
        r['active']=nav_state['active']; r['goal_ids']=nav_state['goal_ids']; r['activity_evidence']=nav_state['evidence']
        # Status arrays represent transitions, not periodic samples. Their
        # publisher generation must still exist; no cross-action UUID mapping.
        r['navigate_status']=status; r['follow_status']=statuses.get('/follow_path/_action/status')
        r['map_base_tf']=pose_tf('map'); r['odom_base_tf']=pose_tf('odom')
        r['context']={k:v for k,v in latest.items() if not k.endswith('/costmap_raw')}
        r['costmaps']={}
        for topic in ['/local_costmap/costmap_raw','/global_costmap/costmap_raw']:
            cm=latest.get(topic)
            if not cm: continue
            value=cm['value']; frame=value['header']['frame_id']; cm_pose=pose_tf(frame,cm['stamp_s'])
            r['costmaps'][topic]=dict(file=cm['file'],seq=cm['seq'],stamp_s=cm['stamp_s'],
                                      age_s=sim-cm['stamp_s'],pose_at_costmap_stamp=cm_pose)
        rows.append(r); samples_file.write(json.dumps(finite_json(r),allow_nan=False)+'\n')
        last_sample=(clock.epoch,sim)

    manifest=dict(label=args.label,started_utc=datetime.now(timezone.utc).isoformat(),
                  observer_pid=os.getpid(),domain=os.environ.get('ROS_DOMAIN_ID','0'),
                  localhost_only=os.environ.get('ROS_LOCALHOST_ONLY'),topics=topics,mode=mode,
                  omitted_dwb_debug_topics=[] if args.dwb_debug else list(DWB_DEBUG_TOPICS),
                  duration_wall_s=args.duration,read_only=True,child_processes=[],
                  motion_alignment='commands: simulation receive time; odometry/wrench: header stamp; causal bounded age',
                  geometry='offline filled convex padded footprint vs raw cell areas; not physical contact or exact DWB critic',
                  thresholds=dict(command_linear_m_s=.02,command_angular_rad_s=.05,
                      stationary_translation_m=.04,stationary_rotation_rad=.1,stationary_duration_sim_s=3.,plate_force_n=50.,footprint_costmap_skew_max_s=.75))
    write_json(output/'manifest.json',manifest)
    print(f'Attach observer ({mode}) output: {output}',flush=True)
    next_discovery=next_report=0.
    try:
        while rclpy.ok() and (args.duration==0 or time.monotonic()-started<args.duration):
            elapsed=time.monotonic()-started
            if elapsed>=next_discovery: discover(); next_discovery=elapsed+2.
            rclpy.spin_once(node,timeout_sec=.02); complete_costmap_poses(); sample()
            if elapsed>=next_report:
                events_file.flush(); samples_file.flush(); write_json(output/'topic-health.json',topic_health(topics,graph,counts))
                print(f'{elapsed:.0f}s wall; sim={sim}; {len(subs)} subscriptions; {len(rows)} aligned samples',flush=True)
                next_report=elapsed+30.
    except (KeyboardInterrupt,ExternalShutdownException): stop_reason='observer_interrupted'
    except Exception as error:
        stop_reason='observer_error'; errors.append(dict(error=repr(error)))
    finally:
        wall_duration = time.monotonic()-started
        # Tear down only observer-owned ROS resources; no process signals.
        for f in futures:
            if not f.done(): f.cancel()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        events_file.close(); samples_file.close()

    print('Observer detached; computing offline candidate summaries.',flush=True)
    # Parse only after detaching; no extra callbacks or live subscriptions.
    with (output/'events.jsonl').open() as recorded:
        precursor_events=[e for line in recorded if (e:=json.loads(line))['topic'] in
                          ['/rosout','/course_navigation/cmd_vel_raw',
                           '/navigate_to_pose/_action/status','/follow_path/_action/status']]
    results=[]
    for interval in stationary_intervals(rows):
        selected=interval.pop('rows')
        before=[r for r in rows if r['epoch']==interval['epoch'] and interval['start']-1.<=r['t']<interval['start']]
        result=classify_interval(selected,before,logs)
        result['precursor_chronology']=precursor_chronology(selected,precursor_events)
        result['motion_source']=interval['source']
        result['snapshots']=[]
        for r in ([before[-1]] if before else [])+[selected[0],selected[min(10,len(selected)-1)],selected[-1]]:
            snapshot=dict(epoch=r['epoch'],t=r['t'],sample=r,offline_local_geometry=None)
            local=r['costmaps'].get('/local_costmap/costmap_raw')
            cm=json.loads((output/local['file']).read_text()) if local else None
            pose=cm.get('pose_at_costmap_stamp') if cm else None
            poly=cm.get('padded_footprint_at_costmap_stamp') if cm else None
            if local and pose and poly and 0<=local['age_s']<=1.:
                try:
                    md=cm['metadata']; o=md['origin']
                    q=o['orientation']; angle=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
                    g=dict(width=md['size_x'],height=md['size_y'],resolution=md['resolution'],
                           origin=[o['position']['x'],o['position']['y'],angle], data=(output/cm['raw_file']).read_bytes())
                    snapshot['offline_local_geometry']=footprint_geometry(g,pose['xyyaw'],poly)
                    snapshot['geometry_input']=dict(costmap_file=local['file'],pose=pose,footprint=poly,footprint_source=cm['footprint_source'])
                except Exception as error: snapshot['geometry_unavailable_reason']=str(error)
            else: snapshot['geometry_unavailable_reason']='Missing/freshness-limited costmap, exact-stamp TF, or temporally matched published padded footprint'
            filename=f'stall-{len(results):03d}-{len(result["snapshots"]):02d}.json'
            write_json(output/filename,finite_json(snapshot)); result['snapshots'].append(filename)
        results.append(result)
    summary=dict(label=args.label,output=str(output),stop_reason=stop_reason,mode=mode,
                 wall_duration_s=wall_duration,clock_epochs=clock.epoch+1 if sim is not None else 0,
                 aligned_samples=len(rows),topic_health=topic_health(topics,graph,counts),
                 candidate_stationary_intervals=results,errors=errors,
                 runtime_parameter_nodes=list(runtime),missing_parameter_nodes=[n for n in PARAMETER_NODES if n not in runtime],
                 first_causal_layer=None,physical_collision_proven=False,
                 limitations=['Active-goal stationarity may be an intentional BT wait or goal rotation/arrival transition.',
                    'Plate wrench measures a small fixed-joint plate, includes dynamics; silent plate cannot exclude full-body contact.',
                    'Costmap clearing, projection, latency and SLAM error affect offline geometry; no swept-volume or 3D contact inference. Published footprint shape is assumed constant across <=0.75s skew.',
                    'Unstamped Twist timing is receive-clock timing; sub-sample causal ordering may remain ambiguous.',
                    'DWB detail is opt-in; positive short-circuited scores may be partial; nonbest predicted poses are omitted.',
                    'Observer receive rates and local TF ages do not establish controller timing or causality; LIGHT timing needs user validation.'],
                 cleanup=dict(observer_resources_destroyed=True,child_processes=[],session_processes_touched=False))
    write_json(output/'topic-health.json',topic_health(topics,graph,counts))
    health = measurement_health(rows,counts,topics,wall_duration,measurement_clock.duration_s,mode)
    write_final_reports(output,manifest,summary,health)
    print(f'Final summary: {output / "summary.json"}',flush=True)


if __name__ == '__main__':
    main()
