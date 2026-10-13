#!/usr/bin/python3
"""Serial unchanged-baseline A/B characterization. Each case restarts the full stack."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
import uuid
from datetime import datetime, timezone
from narrow_fixture import geometry, fixture, chosen_route, classify_failure
from narrow_processes import OwnedProcesses

ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT/'course_navigation/config/nav2.yaml'

def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')

def log_is_warning(message):
    # Humble Log.WARN is a byte constant; message.level is an integer.
    return message.level >= 30

def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()

def observation_complete(path):
    return path.is_file() and path.read_text().strip()=='观察完成'

def costmap_diagnostic(cm, start, goal):
    """Keep raw costs and both Costmap2D floor / Humble NavFn round cells."""
    meta=cm['metadata']; origin=meta['origin']['position']; r=meta['resolution']
    def cell(point, rounded=False):
        values=[(point[0]-origin['x'])/r, (point[1]-origin['y'])/r]
        indices=[math.floor(v+.5) if rounded else math.floor(v) for v in values]
        x,y=indices
        cost=int(cm['data'][y*meta['size_x']+x]) if (
            all(v>=0 for v in values) and 0<=x<meta['size_x'] and 0<=y<meta['size_y']) else None
        return dict(cell=indices,cost=cost)
    n=max(1,math.ceil(math.dist(start,goal)/r))
    points=[[a+(b-a)*i/n for a,b in zip(start,goal)] for i in range(n+1)]
    samples=[cell(p) for p in points]; costs=[s['cost'] for s in samples]
    def counts(values):
        return dict(unknown=values.count(255),cost_253=values.count(253),cost_254=values.count(254))
    return dict(start=cell(start),goal=cell(goal),navfn_start=cell(start,True),
        navfn_goal=cell(goal,True),cell_semantics='Costmap2D floor; NavFn 1.1.20 worldToMap std::round',
        centerline=dict(xy=points,cells=[s['cell'] for s in samples],costs=costs,
            histogram={str(c):costs.count(c) for c in sorted(set(costs),key=lambda c:-1 if c is None else c)},
            **counts(costs)),map_counts=counts(list(cm['data'])))

def run_case(args, case, design):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist, WrenchStamped
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
    from nav2_msgs.msg import Costmap, BehaviorTreeLog
    from nav2_msgs.srv import GetCostmap
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from action_msgs.msg import GoalStatusArray
    from rcl_interfaces.msg import Log
    from rcl_interfaces.srv import ListParameters, GetParameters
    from rclpy.parameter import parameter_value_to_python
    from lifecycle_msgs.srv import GetState
    from course_navigation.lifecycle_readiness import LifecycleReadiness
    from tf2_ros import Buffer, TransformListener, TransformException
    from rosidl_runtime_py.convert import message_to_ordereddict

    token = uuid.uuid4().hex
    env = os.environ.copy()
    env.update(IGN_PARTITION='b2w-narrow-'+token, B2W_NARROW_OWNER=token,
               IGN_LOG_PATH=str(case/'gazebo-log'), OMP_NUM_THREADS='2')
    diagnostic=args.planner_diagnostic or args.online_navigation
    if diagnostic: env['B2W_NARROW_NAVFN_DEBUG']='1'
    owner = OwnedProcesses(token)
    record = dict(scenario_id=design['scenario_id'], corridor_width_m=design['corridor_width_m'],
                  seed=design['seed'], measurement_scope=design['measurement_scope'],
                  goal_relative_start_xy=design['goal_relative_start_xy'], global_path_generated=False, chosen_route=None, success=False,
                  collision_count=None, path_length_m=None, time_to_goal_s=None,
                  final_goal_error_m=None, planning_latency_ms=None, minimum_clearance_m=None,
                  minimum_clearance_missing_reason='No reliable whole-body clearance sensor; footprint is a navigation approximation',
                  controller_abort_observed=False,
                  controller_abort_observed_definition='Any observed FollowPath ABORTED status; no primary failure attribution',
                  controller_action_status_observed=[], primary_controller_failure=None,
                  controller_attribution_missing_reason='No reliable NavigateToPose -> FollowPath UUID and terminal-cause association',
                  navigation_attempted=False,
                  controller_timeout=None, failure_taxonomy=None,
                  planning_latency_definition='Pre-navigation ComputePathToPose action wall round trip; excludes BT replans',
                  path_source='/slam/odom continuous odom xy; not global plan length',
                  collision_definition='plate force norm >50N rising edges, 1s cooldown; not whole-body collision coverage',
                  cmd_topics=['/course_navigation/cmd_vel_raw','/course_navigation/cmd_vel',
                              '/path_manager/path_manager_ros/nav_vel'])
    record.update(ros_domain_id=os.environ['ROS_DOMAIN_ID'],ign_partition=env['IGN_PARTITION'],
                  owner_token=token,nav_params_file=str(args.nav_params_file),
                  nav_params_sha256=digest(args.nav_params_file),gui=args.gui,rviz=args.rviz)
    os.environ['ROS_LOG_DIR']=str(case/'ros-log')
    env['ROS_LOG_DIR']=os.environ['ROS_LOG_DIR']
    rclpy.init()
    node = Node('narrow_baseline_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    buffer = Buffer(); listener = TransformListener(buffer,node)
    planner = ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
    costmap_client=node.create_client(GetCostmap,'/global_costmap/get_costmap')
    nav = ActionClient(node,NavigateToPose,'/navigate_to_pose')
    states = [node.create_client(GetState,f'/{n}/get_state') for n in
              ('planner_server','controller_server','bt_navigator')]
    readiness=LifecycleReadiness(states)
    state = dict(truth=None,truth_points=[],odom=None, map=None, costmap=None, collision_seen=False, contact=False,
                 last_collision=-math.inf, collisions=0, previous=None, length=0., samples=0,
                 active=False, plans=[], command_counts={}, command_max={}, logs=[])
    stream = (case/'observations.jsonl').open('w')
    def emit(kind, data):
        stream.write(json.dumps(dict(kind=kind, wall=time.monotonic(), sim=node.get_clock().now().nanoseconds*1e-9, **data))+'\n')
    def odom(msg):
        state['odom'] = (msg,time.monotonic())
        if not state['active']: return
        p=msg.pose.pose.position; stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        previous=state['previous']
        if previous is not None and stamp <= previous[2]: return
        if previous is not None and stamp-previous[2] <= .5:
            state['length'] += math.dist((p.x,p.y),previous[:2])
        elif previous is not None: record['odom_gap_detected']=True
        state['previous']=(p.x,p.y,stamp); state['samples']+=1
        emit('odom',dict(x=p.x,y=p.y,stamp=stamp,message=message_to_ordereddict(msg)))
    def truth(msg):
        p=msg.pose.pose.position; q=msg.pose.pose.orientation
        state['truth']=(msg,time.monotonic())
        if state['active']:
            state['truth_points'].append([p.x,p.y])
            emit('truth_eval_only',dict(x=p.x,y=p.y,frame=msg.header.frame_id,
                stamp=dict(sec=msg.header.stamp.sec,nanosec=msg.header.stamp.nanosec),
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
    def force(msg):
        if not state['active']: return
        state['collision_seen']=True
        f=msg.wrench.force; norm=math.sqrt(f.x*f.x+f.y*f.y+f.z*f.z)
        t=node.get_clock().now().nanoseconds*1e-9
        contact=norm>50
        emit('collision_plate',dict(force_n=norm,contact=contact,message=message_to_ordereddict(msg)))
        if contact and not state['contact'] and t-state['last_collision']>=1:
            state['collisions']+=1; state['last_collision']=t
        state['contact']=contact
        if contact: emit('contact',dict(force_n=norm))
    def command(msg, topic):
        if not state['active']: return
        values=[msg.linear.x,msg.linear.y,msg.angular.z]
        state['command_counts'][topic]=state['command_counts'].get(topic,0)+1
        old=state['command_max'].get(topic,[0.,0.,0.])
        state['command_max'][topic]=[max(a,abs(b)) for a,b in zip(old,values)]
        emit('cmd',dict(topic=topic,vx=values[0],vy=values[1],wz=values[2]))
    def plan(msg):
        if state['active']:
            points=[[p.pose.position.x,p.pose.position.y] for p in msg.poses]
            state['plans'].append(points); emit('global_plan',dict(frame=msg.header.frame_id,xy=points))
    def status(msg, action='controller'):
        if not state['active']: return
        entries=[dict(goal_id=bytes(s.goal_info.goal_id.uuid).hex(), status=s.status,
            goal_stamp=dict(sec=s.goal_info.stamp.sec,nanosec=s.goal_info.stamp.nanosec),
            goal_stamp_s=s.goal_info.stamp.sec+s.goal_info.stamp.nanosec*1e-9) for s in msg.status_list]
        emit(action+'_action_status',dict(entries=entries))
        if action=='controller':
            record['controller_action_status_observed'].append(entries)
            if any(s.status==6 for s in msg.status_list): record['controller_abort_observed']=True
    def log(msg):
        if diagnostic and msg.name in ('rclcpp','planner_server'):
            emit('navfn_runtime',dict(name=msg.name,level=msg.level,text=msg.msg,
                stamp=dict(sec=msg.stamp.sec,nanosec=msg.stamp.nanosec),file=msg.file,function=msg.function,line=msg.line))
        if state['active'] and log_is_warning(msg):
            data=dict(name=msg.name,level=msg.level,text=msg.msg)
            state['logs'].append(data); emit('rosout',data)
    node.create_subscription(Odometry,'/slam/odom',odom,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/sim/ground_truth/odom',truth,qos_profile_sensor_data)
    node.create_subscription(WrenchStamped,'/collision_plate/force_torque',force,20)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid,'/map',lambda m:state.update(map=m),qos)
    node.create_subscription(Costmap,'/global_costmap/costmap_raw',lambda m:state.update(costmap=m),qos)
    node.create_subscription(NavPath,'/plan',plan,10)
    node.create_subscription(BehaviorTreeLog,'/behavior_tree_log',
        lambda m:emit('behavior_tree_log',dict(message=message_to_ordereddict(m))) if state['active'] else None,100)
    node.create_subscription(GoalStatusArray,'/follow_path/_action/status',status,qos)
    for action, topic in [('planner','/compute_path_to_pose/_action/status'),('bt','/navigate_to_pose/_action/status')]:
        node.create_subscription(GoalStatusArray,topic,lambda m,a=action:status(m,a),qos)
    node.create_subscription(Log,'/rosout',log,100)
    for topic in record['cmd_topics']:
        node.create_subscription(Twist,topic,lambda m,t=topic:command(m,t),20)
    handles=[]
    last_readiness_wall=0.
    process=None; reason='startup_timeout'; started_sim=None
    wall_start=time.monotonic()
    target_xy=None
    def pose():
        try:
            tf=buffer.lookup_transform('map','base_link',Time())
            stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
            age=node.get_clock().now().nanoseconds*1e-9-stamp
            state['tf_age_s']=age
            state['tf_error']=None
            if not 0 <= age <= .5: return None
            return tf.transform
        except TransformException as error:
            state['tf_error']=str(error)
            return None
    def freshness():
        now=node.get_clock().now().nanoseconds*1e-9
        def age(msg): return now-msg.header.stamp.sec-msg.header.stamp.nanosec*1e-9
        return dict(tf_age_s=state.get('tf_age_s'),tf_error=state.get('tf_error'),
            odom_stamp_age_s=age(state['odom'][0]) if state['odom'] else None,
            odom_wall_age_s=time.monotonic()-state['odom'][1] if state['odom'] else None,
            map_stamp_age_s=age(state['map']) if state['map'] else None)
    def fresh_inputs():
        if pose() is None: return False
        f=freshness()
        return (f['odom_wall_age_s'] is not None and f['odom_wall_age_s']<.5 and
                0<=f['odom_stamp_age_s']<=.5 and f['map_stamp_age_s'] is not None and
                0<=f['map_stamp_age_s']<=3.)
    def spin_until(predicate, seconds):
        nonlocal last_readiness_wall
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            owner.observe()
            if process.poll() is not None: raise RuntimeError('runtime_exit')
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()-last_readiness_wall>2 and not state['active']:
                last_readiness_wall=time.monotonic()
                emit('readiness',dict(map=state['map'] is not None,costmap=state['costmap'] is not None,
                    odom=state['odom'] is not None,tf_fresh=pose() is not None,
                    planner=planner.server_is_ready(),nav=nav.server_is_ready(),
                    state_services=[c.service_is_ready() for c in states],
                    tf_age_s=state.get('tf_age_s'),tf_error=state.get('tf_error')))
                stream.flush()
            if predicate(): return True
        return False
    def await_future(future, seconds):
        if not spin_until(future.done,seconds): raise RuntimeError('timeout')
        return future.result()
    def request(client, goal, seconds):
        handle=await_future(client.send_goal_async(goal),min(seconds,10))
        if not handle.accepted: raise RuntimeError('planner_rejected' if client is planner else 'navigation_rejected')
        handles.append(handle)
        emit('action_goal_accepted',dict(action='ComputePathToPose' if client is planner else 'NavigateToPose',
             goal_id=bytes(handle.goal_id.uuid).hex()))
        return await_future(handle.get_result_async(),seconds)
    try:
        with (case/'runtime.log').open('w') as runtime:
            # The wrapper already loaded the environments. Direct launch avoids
            # a child shell with automatic-errexit semantics during diagnostics.
            process=subprocess.Popen(['/usr/bin/python3','/opt/ros/humble/bin/ros2','launch',
                'course_bringup','course.launch.py','mode:=navigation','scene:=confined',
                f'world:={case}/fixture.world','spawn_x:=0','spawn_y:=0','spawn_z:=1',
                f'gui:={str(args.gui).lower()}',f'rviz:={str(args.rviz).lower()}',
                'fault_profile:=normal',f"seed:={design['seed']}",
                f'nav_params_file:={args.nav_params_file}',f'results_directory:={case}/unused-goal-manager'],
                env=env,cwd=ROOT,stdout=runtime,stderr=subprocess.STDOUT,start_new_session=True)
            if not spin_until(lambda: state['map'] is not None and state['costmap'] is not None
                    and fresh_inputs() and planner.server_is_ready() and nav.server_is_ready()
                    and readiness.active(time.monotonic()), args.startup_timeout_s):
                raise RuntimeError('startup_timeout')
            record['nav_lifecycle_states']=readiness.states
            record['readiness_freshness']=freshness()
            if diagnostic:
                dumps={}
                for name in ('planner_server','global_costmap/global_costmap','controller_server',
                             'local_costmap/local_costmap','bt_navigator','velocity_smoother',
                             'behavior_server','smoother_server','map_projector','pose_adapter'):
                    listing=node.create_client(ListParameters,f'/{name}/list_parameters')
                    getting=node.create_client(GetParameters,f'/{name}/get_parameters')
                    if not spin_until(lambda:listing.service_is_ready() and getting.service_is_ready(),5):
                        dumps[name]={'error':'parameter services unavailable'}
                        continue
                    req=ListParameters.Request(); req.depth=0
                    names=await_future(listing.call_async(req),5).result.names
                    req=GetParameters.Request(); req.names=names
                    values=await_future(getting.call_async(req),5).values
                    dumps[name]={n:parameter_value_to_python(v) for n,v in zip(names,values)}
                write(case/'runtime-parameters.json',dumps)
                record['actual_planner_plugin']=dumps['planner_server'].get('GridBased.plugin')
            # A settled observation interval after actual readiness, not a substitute for it.
            spin_until(lambda:False,2.)
            if not spin_until(fresh_inputs,5): raise RuntimeError('localization_stale')
            origin=pose()
            if origin is None: raise RuntimeError('localization_stale')
            q=origin.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            ox,oy=origin.translation.x,origin.translation.y
            record['start_map_xy_yaw']=[ox,oy,yaw]
            if state['truth'] is not None:
                msg=state['truth'][0]; p=msg.pose.pose.position; q=msg.pose.pose.orientation
                record['start_truth_eval_xy_yaw']=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
                record['truth_frame']=msg.header.frame_id
            else: record['start_truth_eval_xy_yaw']=None
            def local(points):
                return [[math.cos(yaw)*(x-ox)+math.sin(yaw)*(y-oy),
                         -math.sin(yaw)*(x-ox)+math.cos(yaw)*(y-oy)] for x,y in points]
            goal=ComputePathToPose.Goal(); goal.goal.header.frame_id='map'
            goal.goal.header.stamp=node.get_clock().now().to_msg()
            distance=design['goal_relative_start_xy'][0]
            goal.goal.pose.position.x=ox+distance*math.cos(yaw); goal.goal.pose.position.y=oy+distance*math.sin(yaw)
            goal.goal.pose.orientation.z=math.sin(yaw/2); goal.goal.pose.orientation.w=math.cos(yaw/2)
            target_xy=(goal.goal.pose.position.x,goal.goal.pose.position.y)
            goal.planner_id='GridBased'; goal.use_start=False
            write(case/'map-before-plan.json',message_to_ordereddict(state['map']))
            if diagnostic:
                if not costmap_client.service_is_ready(): raise RuntimeError('costmap_service_not_ready')
                state['costmap']=await_future(costmap_client.call_async(GetCostmap.Request()),5).map
                record['costmap_snapshot_source']='/global_costmap/get_costmap (live master costmap)'
                record['snapshot_timing_limit']='Service snapshots bracket planning; use_start=false TF is sampled inside planner, not atomically with probe'
            raw_cm=message_to_ordereddict(state['costmap'])
            write(case/'costmap-before-plan.json',raw_cm)
            (case/'costmap-before-plan.raw.u8').write_bytes(bytes(raw_cm['data']))
            record['costmap_diagnostic']=costmap_diagnostic(raw_cm,[ox,oy],target_xy)
            write(case/'costmap-diagnostic.json',record['costmap_diagnostic'])
            record['map_unknown_fraction']=sum(v<0 for v in state['map'].data)/len(state['map'].data)
            # Sample both geometric candidate centerlines in the *actual raw global costmap*.
            cm=state['costmap']; metadata=cm.metadata
            def candidate(points):
                costs=[]
                for a,b in zip(points,points[1:]):
                    count=max(1,math.ceil(math.dist(a,b)/metadata.resolution))
                    for i in range(count+1):
                        x=a[0]+(b[0]-a[0])*i/count; y=a[1]+(b[1]-a[1])*i/count
                        mx=ox+math.cos(yaw)*x-math.sin(yaw)*y
                        my=oy+math.sin(yaw)*x+math.cos(yaw)*y
                        ix=math.floor((mx-metadata.origin.position.x)/metadata.resolution)
                        iy=math.floor((my-metadata.origin.position.y)/metadata.resolution)
                        costs.append(int(cm.data[iy*metadata.size_x+ix]) if
                            0<=ix<metadata.size_x and 0<=iy<metadata.size_y else 255)
                return dict(samples=len(costs),unknown=sum(c==255 for c in costs),
                            blocked=sum(c in (253,254) for c in costs),max_cost=max(costs))
            top=design['island_top_m']+design['wide_route_width_m']/2
            record['candidate_centerline_costs']={'narrow':candidate([(0,0),(6,0)])}
            if design['scenario_id']=='A':
                record['candidate_centerline_costs']['wide']=candidate([(0,0),(1,top),(5,top),(6,0)])
            record['route_comparison_eligible']=design['scenario_id']=='A' and all(
                c['unknown']==0 and c['blocked']==0 for c in record['candidate_centerline_costs'].values())
            planning_start=time.monotonic()
            start_tf=pose()
            record['planning_start_tf_estimate']=message_to_ordereddict(start_tf) if start_tf else None
            record['goal_pose_world_map']=message_to_ordereddict(goal.goal)
            record['planning_request_freshness']=freshness()
            emit('planning_request',dict(start_map_xy=[ox,oy],goal_map_xy=target_xy,
                use_start=False,planner_id=goal.planner_id))
            try:
                response=request(planner,goal,15)
            except RuntimeError as error:
                record['planning_request_latency_ms']=(time.monotonic()-planning_start)*1000
                if str(error)=='timeout': raise RuntimeError('planner_timeout') from error
                raise
            latency=(time.monotonic()-planning_start)*1000
            record['planning_status']=response.status
            record['planning_latency_ms']=latency
            if diagnostic:
                after=await_future(costmap_client.call_async(GetCostmap.Request()),5).map
                write(case/'costmap-after-plan.json',message_to_ordereddict(after))
                (case/'costmap-after-plan.raw.u8').write_bytes(bytes(after.data))
                spin_until(lambda:False,.25)
            points=[[p.pose.position.x,p.pose.position.y] for p in response.result.path.poses]
            write(case/'preplan.json',dict(frame=response.result.path.header.frame_id,xy=points))
            write(case/'compute-path-result.json',message_to_ordereddict(response.result))
            record['global_path_generated']=response.status==4 and len(points)>1
            record['planning_request_latency_ms']=latency
            if not record['global_path_generated']: raise RuntimeError('planner_aborted')
            record['global_path_length_m']=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
            record['chosen_route']=chosen_route(local(points),design['corridor_width_m']) if design['scenario_id']=='A' else 'forced'
            if args.planner_diagnostic:
                reason='planner_diagnostic_succeeded'
                return record
            state['active']=True
            record['navigation_attempted']=True
            started_sim=node.get_clock().now().nanoseconds*1e-9
            nav_goal=NavigateToPose.Goal(); nav_goal.pose=goal.goal
            handle=await_future(nav.send_goal_async(nav_goal,feedback_callback=lambda m:
                emit('navigation_feedback',dict(message=message_to_ordereddict(m.feedback)))),10)
            if not handle.accepted: raise RuntimeError('navigation_rejected')
            record['navigation_goal_id']=bytes(handle.goal_id.uuid).hex()
            emit('action_goal_accepted',dict(action='NavigateToPose',goal_id=record['navigation_goal_id']))
            handles.append(handle); result=handle.get_result_async(); wall_nav=time.monotonic(); stale_since=None
            while not result.done():
                owner.observe(); rclpy.spin_once(node,timeout_sec=.05)
                if process.poll() is not None: raise RuntimeError('runtime_exit')
                now=node.get_clock().now().nanoseconds*1e-9
                if now<started_sim: raise RuntimeError('clock_reset')
                if now-started_sim>args.timeout_s or time.monotonic()-wall_nav>args.timeout_s*5:
                    raise RuntimeError('timeout')
                if pose() is None:
                    stale_since=stale_since or time.monotonic()
                    if time.monotonic()-stale_since>3: raise RuntimeError('localization_stale')
                else: stale_since=None
                if time.monotonic()-state.get('last_freshness_emit',0)>.5:
                    state['last_freshness_emit']=time.monotonic()
                    emit('localization_freshness',freshness())
            response=result.result(); record['navigation_status']=response.status
            write(case/'navigate-result.json',dict(status=response.status,result=message_to_ordereddict(response.result)))
            reason={4:'succeeded',5:'canceled',6:'aborted'}.get(response.status,'action_error')
            final=pose()
            if final:
                record['final_goal_error_m']=math.dist((final.translation.x,final.translation.y),
                                                     (goal.goal.pose.position.x,goal.goal.pose.position.y))
            record['success']=reason=='succeeded' and record['final_goal_error_m'] is not None and record['final_goal_error_m']<=.25
            if reason=='succeeded' and not record['success']: reason='goal_error_or_missing_pose'
            record['bt_plan_routes']=[chosen_route(local(p),design['corridor_width_m']) for p in state['plans']]
    except KeyboardInterrupt:
        reason='interrupted'
    except Exception as error:
        reason=str(error); record['exception_type']=type(error).__name__
    finally:
        final=pose()
        if target_xy is not None and final is not None:
            record['final_goal_error_m']=math.dist((final.translation.x,final.translation.y),target_xy)
        record['final_goal_error_missing_reason']=None if record['final_goal_error_m'] is not None else (
            'No map-frame target constructed' if target_xy is None else 'No fresh map to base TF at finish')
        points=state['truth_points']
        record['physical_gate_route']=chosen_route(points,design['corridor_width_m']) if points else None
        record['corridor_entered_truth']=any(2<=x<=4 and abs(y)<design['corridor_width_m']/2 for x,y in points) if points else None
        record['corridor_passed_truth']=record['physical_gate_route']=='narrow' and points[-1][0]>4 if points else None
        record['startup_observations']=dict(map=state['map'] is not None,costmap=state['costmap'] is not None,
            odom=state['odom'] is not None,tf_fresh=pose() is not None,
            node_names=node.get_node_names(),topic_types=node.get_topic_names_and_types())
        end=node.get_clock().now().nanoseconds*1e-9
        state['active']=False
        if started_sim is not None:
            record['duration_sim_s']=end-started_sim
            if record['success']: record['time_to_goal_s']=end-started_sim
        for handle in handles:
            if handle.accepted:
                try:
                    future=handle.cancel_goal_async()
                    until=time.monotonic()+2
                    while not future.done() and time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.05)
                except Exception: pass
        state['active']=False
        record['collision_count']=state['collisions'] if state['collision_seen'] else None
        record['collision_missing_reason']=None if state['collision_seen'] else 'No plate feedback during observation'
        record['path_length_m']=state['length'] if state['samples']>1 else None
        record['cmd_samples']=state['command_counts']; record['cmd_max_abs_vx_vy_wz']=state['command_max']
        record['controller_evidence']=state['logs']
        record['controller_timeout']=any(m['name']=='controller_server' and 'progress' in m['text'].lower() for m in state['logs']) or None
        record['end_reason']=reason
        record['failure_taxonomy']='none' if reason=='planner_diagnostic_succeeded' else classify_failure(
            reason,record['global_path_generated'],record['controller_abort_observed'],record['collision_count'])
        record['duration_wall_s']=time.monotonic()-wall_start
        record['localization_freshness_at_finish']=freshness()
        record['global_plan_count']=len(state['plans'])
        record['replanning_count']=max(0,len(state['plans'])-1)
        write(case/'global-plans.json',state['plans'])
        if args.hold_for_observation and record['navigation_attempted'] and process and process.poll() is None:
            record['cleanup_passed']=None
            write(case/'result-before-cleanup.json',record)
            stream.flush()
            print(f'GUI retained; awaiting user 观察完成: {case}',flush=True)
            try:
                while not observation_complete(case/'observation-complete.txt'):
                    owner.observe()
                    rclpy.spin_once(node,timeout_sec=.1)
                    if process.poll() is not None:
                        record['gui_hold_runtime_exit']=process.returncode
                        break
                record['observation_completed']=observation_complete(case/'observation-complete.txt')
            except KeyboardInterrupt:
                record['observation_completed']=False
                record['gui_hold_interrupted']=True
        cleanup=owner.cleanup(process) if process else dict(passed=True,remaining={})
        write(case/'cleanup.json',cleanup)
        record['cleanup_passed']=cleanup['passed']
        stream.close(); node.destroy_node(); rclpy.shutdown()
        write(case/'result.json',record)
    return record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenarios',nargs='+',choices=['A','B'],default=['A','B'])
    parser.add_argument('--widths',nargs='+',type=float,help='Default: derived widths printed by --prepare-only')
    parser.add_argument('--seeds',nargs='+',type=int,default=[0,1,2])
    parser.add_argument('--timeout-s',type=float,default=90)
    parser.add_argument('--startup-timeout-s',type=float,default=90)
    parser.add_argument('--ros-domain-id',type=int,default=97)
    parser.add_argument('--output-dir',type=Path,default=ROOT/'course_results/narrow-baseline')
    parser.add_argument('--prepare-only',action='store_true',help='Generate fixtures/manifest without ROS or simulation')
    parser.add_argument('--smoke',action='store_true',help='One B fixture entry-bay 1m goal at widest width; checks harness, not corridor passage')
    parser.add_argument('--planner-diagnostic',action='store_true',help='One fixed B/1.70m/seed0 planning-only case with NavFn DEBUG; never sends NavigateToPose')
    parser.add_argument('--goal-distance',type=float,choices=[1.,3.,4.5,6.],default=6.)
    parser.add_argument('--nav-params-file',type=Path,default=BASELINE)
    parser.add_argument('--online-navigation',action='store_true',help='One B/1.70m/seed0/6m GUI navigation; holds until explicit observation acknowledgement')
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--rviz',action='store_true')
    parser.add_argument('--hold-for-observation',action='store_true')
    args=parser.parse_args()
    if args.timeout_s<=0 or args.startup_timeout_s<=0: parser.error('Timeouts must be positive')
    args.nav_params_file=args.nav_params_file.resolve()
    if not args.nav_params_file.is_file(): parser.error('nav params file does not exist')
    if args.online_navigation:
        if args.smoke or args.planner_diagnostic: parser.error('online navigation is a separate case')
        args.scenarios=['B']; args.widths=[1.7]; args.seeds=[0]; args.goal_distance=6.
        args.gui=args.rviz=args.hold_for_observation=True
    if args.hold_for_observation and (not args.gui or not args.rviz or args.planner_diagnostic):
        parser.error('GUI hold requires navigation with both GUI and RViz')
    g=geometry(BASELINE)
    widths=args.widths if args.widths is not None else g['widths_m']
    if args.smoke: args.scenarios=['B']; widths=[g['widths_m'][-1]]; args.seeds=[0]; args.timeout_s=30
    if args.planner_diagnostic:
        if args.smoke: parser.error('--smoke and --planner-diagnostic are separate short tests')
        args.scenarios=['B']; widths=[1.7]; args.seeds=[0]
    os.environ['ROS_DOMAIN_ID']=str(args.ros_domain_id)
    os.environ.setdefault('ROS_LOCALHOST_ONLY','1')
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(ROOT/'course_sim/config'/(
        'fastdds_local.xml' if os.environ['ROS_LOCALHOST_ONLY']=='1' else 'fastdds.xml'))
    run=args.output_dir.resolve()/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    run.mkdir(parents=True,exist_ok=False)
    manifest=dict(geometry=g, baseline_yaml=str(BASELINE), baseline_sha256=digest(BASELINE),
        scenarios=args.scenarios,widths_m=widths,seeds=args.seeds,ros_domain_id=args.ros_domain_id,
        map_mode='online',physics_rng_seeded=False,footprint_is_navigation_approximation=True,
        mapping_confounds='Unknown or blocked candidate cells invalidate route preference attribution',
        prepare_only=args.prepare_only,smoke=args.smoke,planner_diagnostic=args.planner_diagnostic,
        nav_params_file=str(args.nav_params_file),nav_params_sha256=digest(args.nav_params_file),
        gui=args.gui,rviz=args.rviz,hold_for_observation=args.hold_for_observation,
        goal_distance_m=args.goal_distance if args.planner_diagnostic or args.online_navigation else None)
    policy=ROOT/'third_party/b2w_rl_sar/policy/b2w/robot_lab/policy.onnx'
    if policy.exists(): manifest['policy_sha256']=digest(policy)
    write(run/'manifest.json',manifest)
    results=[]
    try:
        for scenario in args.scenarios:
            for width in widths:
                for seed in args.seeds:
                    case=run/f'{scenario}-w{width:.4f}-seed{seed}'; case.mkdir()
                    design=fixture(ROOT,case,scenario,width,g); design['seed']=seed
                    if args.smoke: design['goal_relative_start_xy']=[1.,0.]
                    if args.planner_diagnostic or args.online_navigation: design['goal_relative_start_xy']=[args.goal_distance,0.]
                    design['measurement_scope']='planner_only_diagnostic' if args.planner_diagnostic else (
                        'fixed_online_navigation' if args.online_navigation else (
                        'entry_bay_harness_smoke' if args.smoke else 'full_corridor_baseline'))
                    write(case/'fixture.json',design)
                    if args.prepare_only: continue
                    result=run_case(args,case,design); results.append(result)
                    write(run/'summary.json',results)
                    print(json.dumps({k:result[k] for k in ('scenario_id','corridor_width_m','seed',
                        'measurement_scope','global_path_generated','chosen_route','success','collision_count',
                        'path_length_m','time_to_goal_s','final_goal_error_m','planning_latency_ms',
                        'end_reason','failure_taxonomy','cleanup_passed')}),flush=True)
                    if result['end_reason']=='interrupted': raise KeyboardInterrupt
                    if not result['cleanup_passed']: raise RuntimeError('Cleanup failed; refusing next case')
    finally:
        write(run/'baseline-integrity.json',dict(before_sha256=manifest['baseline_sha256'],
            after_sha256=digest(BASELINE),unchanged=digest(BASELINE)==manifest['baseline_sha256']))
        print(f'Results: {run}',flush=True)
    if args.smoke and not args.prepare_only:
        raise SystemExit(0 if results and results[0]['success'] and results[0]['cleanup_passed'] else 1)

if __name__=='__main__': main()
