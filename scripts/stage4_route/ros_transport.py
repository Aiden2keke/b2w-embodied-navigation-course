"""USER-RUN only. Attach to one idle stack; never launch simulation or publish velocity.

A bounded raw journal preserves public messages. No goal is sent until candidate,
registration, installed endpoint guard, effective parameters and exclusive graph
checks succeed. All cancellation uses the exact owned action handle.
"""
import json
import math
from pathlib import Path
import shutil
import time
from collections import deque
from uuid import uuid4
import yaml
import hashlib
from .contract import ROOT,load_selection,digest,object_hash,write_new,check_registration
from .spatial import MapAlignment,footprint_clear,saved_grid,assess_path
from .freshness import StructuralMapCadence
from .journal import BinaryJournal


def stamp(t):return t.sec+t.nanosec*1e-9


def yaw(q):
    v=[q.x,q.y,q.z,q.w]
    if not all(math.isfinite(x) for x in v) or abs(sum(x*x for x in v)-1)>.01:raise ValueError('Invalid quaternion')
    return math.atan2(2*(q.w*q.z+q.x*q.y),sum(x*x for x in v)-2*(q.y*q.y+q.z*q.z))


def grid(msg,costmap=False):
    m=msg.metadata if costmap else msg.info;o=m.origin
    if abs(o.orientation.x)>1e-6 or abs(o.orientation.y)>1e-6:raise ValueError('Nonplanar grid origin')
    return dict(width=m.size_x if costmap else m.width,height=m.size_y if costmap else m.height,
                resolution=m.resolution,origin=[o.position.x,o.position.y,yaw(o.orientation)],data=list(msg.data))


def coherent_localization(buffer,odom,seen,*,sim,wall):
    """One exact source-time snapshot, checked against independently sent odom."""
    from rclpy.time import Time
    source=stamp(odom.header.stamp)
    if (odom.header.frame_id!='odom' or odom.child_frame_id!='base_link' or
            not math.isfinite(source) or not 0<=sim-source<=.5):
        raise ValueError('Stale or invalid SLAM odometry convention/source time')
    for key in [('map','odom'),('odom','base_link')]:
        edge=seen.get(key)
        if (edge is None or not all(math.isfinite(edge[k]) for k in ('stamp','wall')) or
                not 0<=sim-edge['stamp']<=.5 or not 0<=wall-edge['wall']<=.5):
            raise ValueError('Stale or missing advancing localization TF edge')
    target=Time.from_msg(odom.header.stamp)
    transforms=[buffer.lookup_transform(*key,target) for key in
                [('map','base_link'),('map','odom'),('odom','base_link')]]
    for tf in transforms:
        if abs(stamp(tf.header.stamp)-source)>1e-9:
            raise ValueError('TF lookup returned a different source time')
        yaw(tf.transform.rotation)
        if not all(math.isfinite(getattr(tf.transform.translation,k)) for k in 'xyz'):
            raise ValueError('Nonfinite localization TF')
    ob=transforms[2].transform;p=odom.pose.pose
    yaw(p.orientation)
    qa=[getattr(p.orientation,k) for k in 'xyzw'];qb=[getattr(ob.rotation,k) for k in 'xyzw']
    normalized_dot=abs(sum(a*b for a,b in zip(qa,qb)))/math.sqrt(sum(a*a for a in qa)*sum(b*b for b in qb))
    if (not all(math.isfinite(getattr(p.position,k)) for k in 'xyz') or
        math.dist([getattr(p.position,k) for k in 'xyz'],[getattr(ob.translation,k) for k in 'xyz'])>1e-5 or
        normalized_dot<math.cos(1e-5/2)):
        raise ValueError('Odometry and source-time TF pose disagreement')
    return transforms


def select_coherent_localization(buffer,history,seen,*,sim,wall):
    from tf2_ros import TransformException
    for odom,receipt in reversed(history):
        source=stamp(odom.header.stamp)
        if (any(key not in seen or seen[key]['stamp']+1e-9<source
                for key in [('map','odom'),('odom','base_link')]) or
                not 0<=sim-source<=.5 or not 0<=wall-receipt<=.5):continue
        try:
            return odom,coherent_localization(buffer,odom,seen,sim=sim,wall=wall),receipt
        except TransformException:
            # Listener and observation callbacks can be delivered separately.
            # An older complete source pair is allowed only within the same
            # strict source/receipt budgets, never by restamping or extrapolating.
            continue
    raise ValueError('Missing complete fresh source-time localization snapshot')


class RosTransport:
    def __init__(self,directory,*,deployment=None):
        # Importing this module for offline tests is safe; construction is live opt-in.
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        from nav2_msgs.action import FollowPath
        from tf2_ros import Buffer,TransformListener
        self.rclpy=rclpy;rclpy.init();self.node=Node('stage4_route_'+uuid4().hex[:8],parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.directory=Path(directory);self.raw_path=self.directory/('raw-'+uuid4().hex+'.sqlite3')
        self.deployment_path=Path(deployment) if deployment else self.directory/'deployment.json'
        self.grid_revisions={}
        self.journal=BinaryJournal(self.raw_path,max_bytes=64*1024**3)
        self.localization_tf={}
        self.odom_history=deque(maxlen=16)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self.node)
        self.nav=ActionClient(self.node,FollowPath,'/follow_path')
        self.messages={};self.receipts={};self.statuses={};self.failure=None;self.handle=None
        self.structural = StructuralMapCadence()
        self.send_future=None;self.result_future=None;self.owned_id=None;self.preflight_id=None;self.child_ids={};self.last_pose_stamp=None
        self.saved=None;self.audit=dict(child_processes=[],action_outcomes=[],cleanup={},collision_coverage='plate_only_full_body_unknown')
        self.qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.transient=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.action_qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        from geometry_msgs.msg import PoseStamped,Twist,WrenchStamped
        from nav_msgs.msg import Odometry,OccupancyGrid,Path as NavPath
        from sensor_msgs.msg import PointCloud2,Imu,JointState
        from ros_gz_interfaces.msg import Contacts
        from std_msgs.msg import String
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        from rcl_interfaces.msg import Log
        from action_msgs.msg import GoalStatusArray
        from nav2_msgs.msg import Costmap,BehaviorTreeLog
        topics=[('/clock',Clock),('/slam/odom',Odometry),('/slam/map_points',PointCloud2),('/pointcloud',PointCloud2),('/imu',Imu),
                ('/map',OccupancyGrid),('/tf',TFMessage),('/tf_static',TFMessage),
                ('/joint_states',JointState),('/sim/ground_truth/odom',Odometry),
                ('/b2w/evaluation/full_contacts',Contacts),('/b2w/evaluation/collision_inventory',String),
                ('/local_costmap/costmap_raw',Costmap),('/global_costmap/costmap_raw',Costmap),
                ('/collision_plate/force_torque',WrenchStamped),('/plan',NavPath),('/rosout',Log),('/behavior_tree_log',BehaviorTreeLog),
                ('/goal_pose',PoseStamped),('/course_navigation/unused_nav2_goal_pose',PoseStamped),
                ('/course_navigation/cmd_vel_raw',Twist),('/course_navigation/cmd_vel',Twist),('/path_manager/path_manager_ros/nav_vel',Twist)]
        for topic,typ in topics:
            self.node.create_subscription(typ,topic,lambda m,t=topic:self.receive(t,m),self.transient if topic in ('/map','/tf_static') else self.qos)
        for name in ('navigate_to_pose','navigate_through_poses','follow_waypoints','follow_path','compute_path_through_poses','spin','backup','drive_on_heading','assisted_teleop','wait'):
            topic='/'+name+'/_action/status'
            self.node.create_subscription(GoalStatusArray,topic,lambda m,t=topic:self.receive(t,m),self.action_qos)

    def sim(self):return self.node.get_clock().now().nanoseconds*1e-9

    def receive(self,topic,msg):
        from rclpy.serialization import serialize_message
        now=time.monotonic();self.messages[topic]=msg;self.receipts[topic]=now
        if topic=='/slam/odom':
            if self.odom_history and stamp(msg.header.stamp)<=stamp(self.odom_history[-1][0].header.stamp):
                if stamp(msg.header.stamp)<stamp(self.odom_history[-1][0].header.stamp):
                    self.failure=self.failure or 'odometry_clock_regression'
            else:self.odom_history.append((msg,now))
        data=serialize_message(msg)
        if topic in ('/map','/local_costmap/costmap_raw','/global_costmap/costmap_raw'):
            if not hasattr(self,'grid_revisions'):self.grid_revisions={}
            from course_navigation.stage4_geometry import grid_revision
            encoding='occupancy' if topic=='/map' else 'costmap'
            self.grid_revisions[topic]=dict(sha256=grid_revision(grid(msg,topic!='/map'),encoding)[1],frame=msg.header.frame_id,encoding=encoding)
            if not hasattr(self,'grid_observations'):self.grid_observations={}
            self.grid_observations[topic]=dict(cdr_sha256=hashlib.sha256(data).hexdigest(),source=stamp(msg.header.stamp),receipt=now)
        if not hasattr(self,'recording_first_wall'):
            self.recording_first_wall=self.recording_window_wall=now
            self.recorded_input_bytes=self.recording_window_bytes=0
            self.recording_peak_rate=0.
        self.recorded_input_bytes+=len(data)+1024
        self.recording_window_bytes+=len(data)+1024
        if now-self.recording_window_wall>=.5:
            self.recording_peak_rate=max(self.recording_peak_rate,self.recording_window_bytes/(now-self.recording_window_wall))
            self.recording_window_wall=now;self.recording_window_bytes=0
        cls=type(msg);typ=cls.__module__.split('.')[0]+'/msg/'+cls.__name__
        try:
            self.journal.append(topic,typ,data,dict(wall=time.time(),monotonic=now,sim=self.sim(),
                source_stamp=stamp(msg.header.stamp) if hasattr(msg,'header') else None,
                role='evaluation_only' if '/evaluation/' in topic or topic=='/sim/ground_truth/odom' else 'public_sensor_or_control'))
        except ValueError as exc:self.failure=self.failure or str(exc)
        if topic=='/tf':
            for tf in msg.transforms:
                key=(tf.header.frame_id,tf.child_frame_id)
                if key not in (('map','odom'),('odom','base_link')):continue
                source=stamp(tf.header.stamp);previous=self.localization_tf.get(key)
                if not math.isfinite(source) or previous and source<previous['stamp']:
                    self.failure=self.failure or 'localization_tf_clock_regression';continue
                if previous is None or source>previous['stamp']:
                    self.localization_tf[key]=dict(stamp=source,wall=now)
        if '/b2w/evaluation/' in topic and hasattr(self,'contacts'):self.observe_contact(topic,msg,now)
        if topic == '/slam/map_points':
            try:
                self.structural.observe(stamp(msg.header.stamp), self.sim(), now)
                self.audit['structural_map_cadence'] = dict(
                    startup_wall_periods_s=list(self.structural.periods),
                    wall_budget_s=self.structural.wall_budget, source_budget_s=7.5)
            except ValueError as exc:
                self.failure = str(exc)
        if topic in ('/goal_pose','/course_navigation/unused_nav2_goal_pose'):self.failure='competing_goal'
        if topic.endswith('/_action/status'):
            self.statuses[topic]=msg
            active=[bytes(s.goal_info.goal_id.uuid).hex() for s in msg.status_list if s.status in (1,2,3)]
            if topic.startswith('/follow_path/'):
                if any(i!=self.owned_id for i in active):self.failure='foreign_follow_path'
            elif topic.startswith('/compute_path_through_poses/'):
                if any(i!=self.preflight_id for i in active):self.failure='foreign_preflight_action'
            elif active:self.failure='foreign_navigation_or_behavior_action'
        if topic=='/collision_plate/force_torque':
            f=msg.wrench.force
            if not all(math.isfinite(v) for v in (f.x,f.y,f.z)) or math.sqrt(f.x*f.x+f.y*f.y+f.z*f.z)>50:self.failure='collision_or_invalid_wrench'
        if topic=='/rosout' and msg.level>=40 and msg.name in ('controller_server','DWBLocalPlanner','planner_server','bt_navigator'):
            self.audit['action_outcomes'].append(dict(sim=self.sim(),node=msg.name,message=msg.msg))

    def observe_contact(self,topic,msg,wall):
        try:
            if topic.endswith('/collision_inventory'):self.contacts.inventory(json.loads(msg.data),wall=wall)
            else:self.contacts.contact_message(msg,wall=wall)
            if self.contacts.failure:raise ValueError(self.contacts.failure)
        except Exception as exc:
            self.failure=self.failure or 'contact_evidence:'+repr(exc)
            try:self.request_independent_abort(self.failure)
            except Exception as abort_exc:self.audit['contact_abort_error']=repr(abort_exc)

    def contact_closure(self,start,end):
        closed=bool(hasattr(self,'contacts') and self.contacts.intervals.closed(start=start,end=end,sim=self.sim(),wall=time.monotonic()))
        return dict(reporting_closed_through_end=closed,accepted_start_sim=start,accepted_end_sim=end,
            physical_coverage_qualified=False,safety_complete=False,
            reason='Non-foot engine mapping/self coverage and conservative independent geometry remain unqualified')

    def spin(self):self.rclpy.spin_once(self.node,timeout_sec=.02)

    def graph(self):
        result={}
        for topic in ('/clock','/slam/odom','/slam/map_points'):
            info=self.node.get_publishers_info_by_topic(topic)
            if len(info)!=1:raise ValueError('Require unique publisher: '+topic)
            result[topic]=[bytes(info[0].endpoint_gid).hex()]
        for topic,expected in {
            '/course_navigation/cmd_vel_raw':['controller_server'],
            '/course_navigation/cmd_vel':['behavior_server']*5+['velocity_smoother'],
            '/path_manager/path_manager_ros/nav_vel':['velocity_adapter'],
        }.items():
            info=self.node.get_publishers_info_by_topic(topic)
            if sorted(x.node_name for x in info)!=expected:raise ValueError('Competing/missing command publisher: '+topic)
            result[topic]=sorted(bytes(x.endpoint_gid).hex() for x in info)
        names=self.node.get_node_names()
        if 'goal_manager' in names or 'speed_governor' in names:raise ValueError('Disable goal_manager and adaptive governor for Stage 4')
        if self.node.count_publishers('/goal_pose')>0:raise ValueError('Remove RViz Goal tool/other /goal_pose publishers before capture/run')
        # The default RViz 2D Goal tool advertises this topic even when idle.
        for topic in ('/course_navigation/unused_nav2_goal_pose',):
            if self.node.count_publishers(topic)>0:raise ValueError('Unexpected direct Nav2 topic goal publisher')
        from rclpy.action import get_action_client_names_and_types_by_node
        allowed={self.node.get_name(),'bt_navigator','bt_navigator_navigate_to_pose_rclcpp_node',
                 'bt_navigator_navigate_through_poses_rclcpp_node','waypoint_follower'}
        for name,ns in self.node.get_node_names_and_namespaces():
            clients=get_action_client_names_and_types_by_node(self.node,name,ns)
            if name not in allowed and any(n in ('/navigate_to_pose','/navigate_through_poses','/follow_path','/follow_waypoints','/compute_path_through_poses','/spin','/backup','/drive_on_heading','/assisted_teleop','/wait') for n,_ in clients):
                raise ValueError('Competing action client: '+ns+'/'+name)
        return result

    def age(self,topic):return time.monotonic()-self.receipts.get(topic,-1e20)

    def verify_recording_capacity(self,wall_limit,*,cleanup_wall=60.):
        now=time.monotonic();elapsed=now-getattr(self,'recording_first_wall',now)
        if (wall_limit is None or not all(math.isfinite(v) and v>0 for v in (wall_limit,cleanup_wall)) or
                elapsed<5 or not getattr(self,'recorded_input_bytes',0)):
            raise ValueError('Recording capacity requires bounded run and >=5 s observation')
        rate=max(self.recording_peak_rate,self.recorded_input_bytes/elapsed,
                 self.recording_window_bytes/max(.5,now-self.recording_window_wall))
        needed=math.ceil(2*rate*(wall_limit+cleanup_wall))
        remaining=self.journal.max_bytes-self.raw_path.stat().st_size
        free=shutil.disk_usage(self.raw_path.parent).free-self.journal.reserve
        if needed>min(remaining,free):
            raise ValueError('Insufficient recording capacity for trial plus cleanup')
        report=dict(required_remaining_bytes=needed,remaining_journal_bytes=remaining,
                    free_after_reserve_bytes=free,measured_peak_or_mean_bytes_per_wall_s=rate,
                    observed_wall_s=elapsed,trial_wall_limit_s=wall_limit,
                    cleanup_wall_s=cleanup_wall,headroom_factor=2.)
        self.audit['recording_capacity']=report
        return report

    def sample(self):
        now=self.sim()
        odom,(mb,mo,ob),odom_receipt=select_coherent_localization(
            self.tf,self.odom_history,self.localization_tf,sim=now,wall=time.monotonic())
        p=mb.transform.translation;angle=yaw(mb.transform.rotation)
        cmd=self.messages['/path_manager/path_manager_ros/nav_vel']
        # Actual motion from successive fresh map poses, independent of possibly filtered odom twist.
        speed=math.hypot(odom.twist.twist.linear.x,odom.twist.twist.linear.y);wz=odom.twist.twist.angular.z
        if getattr(self,'previous_sample',None):
            old=self.previous_sample;dt=stamp(mb.header.stamp)-old['tf_stamp']
            if dt>0:
                speed=max(speed,math.hypot(p.x-old['x'],p.y-old['y'])/dt)
                wz=max(abs(wz),abs(math.remainder(angle-old['yaw'],math.tau))/dt)
        safety='clear'
        for topic in ('/local_costmap/costmap_raw','/global_costmap/costmap_raw'):
            cm=self.messages[topic]
            if self.age(topic)>2 or not 0<=now-stamp(cm.header.stamp)<=2:safety='unknown';continue
            if cm.header.frame_id=='map':pose=[p.x,p.y,angle]
            elif cm.header.frame_id=='odom' and topic.startswith('/local_'):
                tr=ob.transform;pose=[tr.translation.x,tr.translation.y,yaw(tr.rotation)]
            else:raise ValueError('Unexpected costmap frame')
            if not footprint_clear(grid(cm,True),pose):safety='blocked'
        if self.saved is not None and not footprint_clear(self.saved,[p.x,p.y,angle]):safety='blocked_saved_map'
        plate=self.messages['/collision_plate/force_torque']
        if self.age('/collision_plate/force_torque')>.5 or not 0<=now-stamp(plate.header.stamp)<=.5:safety='unknown'
        if any(self.age(t)>.5 for t in ('/path_manager/path_manager_ros/nav_vel','/pointcloud','/imu')):safety='unknown'
        result=dict(sim=now,wall=time.monotonic(),x=p.x,y=p.y,yaw=angle,
                    tf_stamp=stamp(mb.header.stamp),map_odom_stamp=stamp(mo.header.stamp),odom_stamp=stamp(odom.header.stamp),
                    **self.structural.ages(now,time.monotonic()),
                    clock_age=self.age('/clock'),pose_age=time.monotonic()-odom_receipt,
                    tf_receipt_age=max(time.monotonic()-v['wall'] for v in self.localization_tf.values()),
                    speed=speed,wz=wz,command_speed=math.hypot(cmd.linear.x,cmd.linear.y),command_wz=cmd.angular.z,safety=safety)
        if (any(not math.isfinite(v) for v in result.values() if type(v) in (int,float)) or
            any(not 0<=now-result[k]<=.5 for k in ('tf_stamp','map_odom_stamp','odom_stamp')) or
            any(not 0<=result[k]<=.5 for k in ('clock_age','pose_age','tf_receipt_age')) or
            not 0<=result['slam_map_age']<=7.5 or
            not 0<=result['slam_map_wall_age']<=result['slam_map_wall_budget']):raise ValueError('Stale/missing/nonfinite SLAM/TF evidence')
        self.previous_sample=result
        return result

    def get_parameters(self,name,expected):
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python
        client=self.node.create_client(GetParameters,name+'/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=2.):raise ValueError('Missing parameter service '+name)
            future=client.call_async(GetParameters.Request(names=list(expected)))
            deadline=time.monotonic()+3
            while not future.done() and time.monotonic()<deadline:self.spin()
            if not future.done():raise ValueError('Parameter service timeout '+name)
            actual={k:parameter_value_to_python(v) for k,v in zip(expected,future.result().values)}
            if actual!=expected:raise ValueError('Runtime configuration differs: '+name+' '+str({k:v for k,v in actual.items() if v!=expected[k]}))
            self.audit.setdefault('effective_parameters',{})[name]=actual
        finally:self.node.destroy_client(client)

    def verify_runtime(self,selection):
        def flat(d,p=''):
            r={}
            for k,v in d.items():
                if isinstance(v,dict):r.update(flat(v,p+k+'.'))
                else:r[p+k]=v
            return r
        cfg=yaml.safe_load(Path(selection['runtime_config']).read_text())
        for name in ('planner_server','controller_server','bt_navigator','velocity_smoother','behavior_server'):
            expected=flat(cfg[name]['ros__parameters'])
            if name=='planner_server':expected.pop('GridBased.use_astar',None) # NavFn-only ignored key.
            self.get_parameters('/'+name,expected)
        for name in ('local_costmap','global_costmap'):
            self.get_parameters('/'+name+'/'+name,flat(cfg[name][name]['ros__parameters']))
        from .deployment import load_deployment,verify_controller
        self.deployment=load_deployment(self.deployment_path)
        from course_navigation.stage4_control import read_control
        self.guard_config=read_control(self.deployment['guard_manifest'])
        self.guard_manifest_sha256=digest(self.deployment['guard_manifest'])
        if not hasattr(self,'contacts'):
            from course_navigation.stage4_contact_stream import ContactStream
            inv=Path(self.guard_config['collision_inventory_path'])
            if digest(inv)!=self.guard_config['collision_inventory_sha256']:raise ValueError('Source collision inventory changed')
            self.contacts=ContactStream(json.loads(inv.read_text())['collisions'],first_sequence=self.guard_config['contact_first_sequence'])
        if (self.guard_manifest_sha256!=self.deployment['guard_manifest_sha256'] or
                self.guard_config['session_id']!=self.deployment['session_id']):
            raise ValueError('Guard deployment content/session mismatch')
        self.get_parameters('/velocity_adapter',dict(use_sim_time=True,m2_inverse_enabled=False,speed_limit_topic='',require_localization_tf=True,input_topic='/course_navigation/cmd_vel',state_topic='/slam/odom',max_linear=.5,max_angular=.6,timeout_s=.5,
            stage4_guard_manifest=self.deployment['guard_manifest'],stage4_guard_sha256=self.guard_manifest_sha256,
            stage4_session_id=self.deployment['session_id']))
        # Mandatory reviewed endpoint-transform guard: verify executable AND mapped library.
        policy=json.loads((ROOT/'course_navigation/config/stage4_runtime_identity.json').read_text())
        self.audit['controller_process']=verify_controller(self.deployment,policy)
        self.audit['deployment_sha256']=digest(self.deployment_path)
        # Lifecycle must be ACTIVE, not merely a responding action server.
        from lifecycle_msgs.srv import GetState
        for name in ('planner_server','controller_server','bt_navigator','velocity_smoother'):
            c=self.node.create_client(GetState,'/'+name+'/get_state')
            try:
                if not c.wait_for_service(timeout_sec=2.):raise ValueError('Missing lifecycle '+name)
                f=c.call_async(GetState.Request());deadline=time.monotonic()+2
                while not f.done() and time.monotonic()<deadline:self.spin()
                if not f.done() or f.result().current_state.id!=3:raise ValueError('Inactive lifecycle '+name)
            finally:self.node.destroy_client(c)

    def idle_capture(self,selection_path):
        selection=load_selection(selection_path);self.saved=saved_grid(selection['source_map']['yaml_path']);self.alignment=MapAlignment(selection['source_map']['yaml_path'])
        deadline=time.monotonic()+15;last_error=None
        while time.monotonic()<deadline:
            self.spin()
            if self.failure:raise ValueError(self.failure)
            try:
                graph=self.graph();s=self.sample()
                if s['safety']!='clear' or s['speed']>.03 or abs(s['wz'])>.03 or s['command_speed']>.01 or abs(s['command_wz'])>.01:raise ValueError('Robot not safely stationary')
                live=grid(self.messages['/map'])
                if self.messages['/map'].header.frame_id!='map' or self.age('/map')>2:raise ValueError('Missing live map')
                p95=self.alignment.check(live)
                self.verify_runtime(selection)
                c=dict(session_id=self.deployment['session_id'],graph=graph,wall=time.time(),sim=self.sim(),selection_sha256=digest(selection_path),
                       live_map_sha256=object_hash(live),live_map=live,pose=s,alignment_p95_m=p95, runtime=self.audit)
                return c
            except (KeyError,ValueError,LookupError) as exc:last_error=repr(exc)
        raise ValueError('Read-only capture failed: '+str(last_error))

    def ready(self,selection,session):
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:self.spin()
        if self.failure:raise ValueError(self.failure)
        self.saved=saved_grid(selection['source_map']['yaml_path'])
        self.alignment=MapAlignment(selection['source_map']['yaml_path'])
        cert=json.loads((self.directory/'registration.json').read_text())
        current={**session,'graph':self.graph()}
        check_registration(cert,current,wall=time.time(),sim=self.sim())
        self.verify_runtime(selection)
        if session['session_id']!=self.deployment['session_id']:raise ValueError('Deployment session differs from registration')
        self.selection=selection
        portal=self.directory/'corridor-portal.json'
        self.corridor_portal=None
        if portal.exists():
            from .portal import CorridorPortal
            value=json.loads(portal.read_text());CorridorPortal(value)
            if (value['session_id']!=session['session_id'] or value['registration_sha256']!=digest(self.directory/'registration.json') or
                    value['map_sha256']!=session['live_map_sha256']):raise ValueError('Portal differs from current sensor/registration session')
            sensor=Path(value['sensor_evidence_path'])
            if digest(sensor)!=value['sensor_sha256']:raise ValueError('Portal sensor evidence changed')
            self.corridor_portal=value
        self.alignment.check(grid(self.messages['/map']))
        self.registration_graph=session['graph'];self.last_alignment=0.;self.last_graph=0.
        sample=self.sample()
        if sample['safety']!='clear' or sample['speed']>.03 or abs(sample['wz'])>.03 or sample['command_speed']>.01 or abs(sample['command_wz'])>.01:
            raise ValueError('Execution requires fresh stationary start')
        if not self.nav.wait_for_server(timeout_sec=2.):raise ValueError('FollowPath unavailable')
        return sample

    def preflight(self,goals):
        from rclpy.action import ActionClient
        from nav2_msgs.action import ComputePathThroughPoses
        from geometry_msgs.msg import PoseStamped
        client=ActionClient(self.node,ComputePathThroughPoses,'/compute_path_through_poses')
        self.preflight_handle=None;self.preflight_future=None;self.preflight_result=None
        try:
            if not client.wait_for_server(timeout_sec=2.):raise ValueError('Preflight planner unavailable')
            action=ComputePathThroughPoses.Goal();action.use_start=True;action.planner_id='GridBased'
            s=self.sample();action.start.header.frame_id='map';action.start.header.stamp=self.node.get_clock().now().to_msg()
            action.start.pose.position.x=s['x'];action.start.pose.position.y=s['y']
            action.start.pose.orientation.z=math.sin(s['yaw']/2);action.start.pose.orientation.w=math.cos(s['yaw']/2)
            for g in goals:
                p=PoseStamped();p.header=action.start.header;p.pose.position.x,p.pose.position.y=g['position']
                for k,v in g['quaternion'].items():setattr(p.pose.orientation,k,v)
                action.goals.append(p)
            from unique_identifier_msgs.msg import UUID
            uid=uuid4();self.preflight_id=uid.hex
            self.preflight_future=client.send_goal_async(action,goal_uuid=UUID(uuid=list(uid.bytes)))
            deadline=time.monotonic()+8.
            while not self.preflight_future.done() and time.monotonic()<deadline:self.spin()
            if not self.preflight_future.done():raise ValueError('Preflight acceptance timeout')
            self.preflight_handle=self.preflight_future.result()
            if not self.preflight_handle.accepted:raise ValueError('Preflight planner rejected')
            self.preflight_result=self.preflight_handle.get_result_async()
            while not self.preflight_result.done() and time.monotonic()<deadline:self.spin()
            if not self.preflight_result.done():raise ValueError('Preflight planning timeout')
            result=self.preflight_result.result()
            if result.status!=4:raise ValueError('Preflight planner action did not succeed')
            path=result.result.path
            if path.header.frame_id!='map' or not 0<=self.sim()-stamp(path.header.stamp)<=2:raise ValueError('Invalid preflight frame/stamp')
            cm=self.messages['/global_costmap/costmap_raw']
            if cm.header.frame_id!='map' or self.age('/global_costmap/costmap_raw')>2:raise ValueError('Missing fresh global costmap')
            poses=[[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in path.poses]
            live=self.messages['/map']
            if live.header.frame_id!='map' or self.age('/map')>2 or not 0<=self.sim()-stamp(live.header.stamp)<=2:
                raise ValueError('Missing fresh registered live map')
            certified_revisions=dict(self.grid_revisions)
            live_grid=grid(live);live_grid['data']=[0 if v==0 else 255 if v<0 else 254 for v in live_grid['data']]
            report=assess_path(poses,goals,[s['x'],s['y']],[self.saved,grid(cm,True),live_grid],
                               start_pose=[s['x'],s['y'],s['yaw']],start_index=getattr(self,'segment_start_index',0),
                               all_goals=self.selection['goals'])
            report.update(action_status=result.status,path=poses,global_costmap_sha256=object_hash(grid(cm,True)))
            self.audit['preflight']=report
            self.checked_path=path
            if not report['passed']:raise ValueError('Pre-motion sampled path/corridor/footprint check FAILED; no navigation goal sent')
            if self.graph()!=self.registration_graph:raise ValueError('Session changed during preflight')
            # Ingest callbacks queued during expensive geometry. A change rejects
            # this attempt; it cannot silently acquire a fresh certificate.
            self.ingest_dispatch_updates()
            if certified_revisions!=self.grid_revisions:raise ValueError('Grid changed while certifying path')
            self.verify_dispatch_inputs()
            from .dispatch import DispatchCertificate
            self.certificate_start=s
            self.dispatch_certificate=DispatchCertificate.issue(self.dispatch_bindings(),wall=time.monotonic(),sim=self.sim())
        finally:
            # A planner cannot move the robot; cancel only this exact pending plan.
            deadline=time.monotonic()+2
            while self.preflight_future and not self.preflight_future.done() and time.monotonic()<deadline:self.spin()
            if self.preflight_future and self.preflight_future.done():
                h=self.preflight_future.result()
                if h.accepted and (self.preflight_result is None or not self.preflight_result.done()):
                    f=h.cancel_goal_async()
                    while not f.done() and time.monotonic()<deadline:self.spin()
                    self.audit['preflight_cancel_confirmed']=f.done()
            # Retain owned UUID: queued statuses can arrive after the result response.
            client.destroy()

    def ingest_dispatch_updates(self):
        deadline=time.monotonic()+.1
        while time.monotonic()<deadline:self.spin()
        if self.failure or self.journal.failure:raise ValueError(self.failure or self.journal.failure)

    def verify_dispatch_inputs(self):
        from course_navigation.stage4_control import read_control
        from course_navigation.stage4_geometry import grid_revision
        cert=json.loads((self.directory/'registration.json').read_text())
        current={**cert['capture'],'session_id':self.deployment['session_id'],'graph':self.graph()}
        check_registration(cert,current,wall=time.time(),sim=self.sim())
        if digest(self.deployment['guard_manifest'])!=self.guard_manifest_sha256:
            raise ValueError('Loaded guard manifest changed')
        health=read_control(self.guard_config['adapter_heartbeat_path'])
        if (health['session_id']!=self.deployment['session_id'] or health['adapter']!=self.deployment['adapter'] or
                type(health['wall']) not in (int,float) or not math.isfinite(health['wall']) or
                not 0<=time.monotonic()-health['wall']<=.15 or health['prepared']['failure']):
            raise ValueError('Missing fresh fault-free adapter preparation acknowledgement')
        expected={'saved':grid_revision(self.saved,'costmap')[1]}
        for name,topic,encoding,costmap,frame in (
                ('live','/map','occupancy',False,'map'),
                ('global','/global_costmap/costmap_raw','costmap',True,'map'),
                ('local','/local_costmap/costmap_raw','costmap',True,'odom')):
            msg=self.messages[topic];source=stamp(msg.header.stamp)
            if msg.header.frame_id!=frame or not 0<=self.sim()-source<=2. or not 0<=self.age(topic)<=2.:
                raise ValueError('Dispatch grid stale/frame mismatch')
            expected[name]=grid_revision(grid(msg,costmap),encoding)[1]
            ack=health['prepared']['grids'][name]
            if abs(ack['source']-source)>1e-9 or not 0<=time.monotonic()-ack['receipt']<=2.:
                raise ValueError('Prepared grid source/receipt acknowledgement differs')
        if {k:v['sha256'] for k,v in health['prepared']['grids'].items()}!=expected:
            raise ValueError('Adapter prepared revisions differ from certified geometry')

    def dispatch_bindings(self):
        from rclpy.serialization import serialize_message
        return dict(path=hashlib.sha256(serialize_message(self.checked_path)).hexdigest(),
            grids=dict(self.grid_revisions),saved_grid=object_hash(self.saved),
            registration=digest(self.directory/'registration.json'),deployment=digest(self.deployment_path),
            guard_manifest=digest(self.deployment['guard_manifest']),
            pinned_guard_manifest=self.guard_manifest_sha256,guard_session=self.guard_config['session_id'],
            graph=object_hash(self.graph()),start_pose=[self.certificate_start[k] for k in ('x','y','yaw')],
            start_source=self.certificate_start['tf_stamp'],frame=self.checked_path.header.frame_id)

    def validate_dispatch(self,current):
        start=self.certificate_start
        if (math.dist([current['x'],current['y']],[start['x'],start['y']])>.03 or
                abs(math.remainder(current['yaw']-start['yaw'],math.tau))>.02):
            raise ValueError('Measured Start changed after path certification')
        self.dispatch_certificate.validate(self.dispatch_bindings(),wall=time.monotonic(),sim=self.sim())
        self.verify_dispatch_inputs()

    def request_independent_abort(self,reason):
        from course_navigation.stage4_control import latch_inhibit
        # The loaded adapter configuration is immutable for this session. Never
        # let a replaced pathname redirect the abort away from the active guard.
        config=self.guard_config
        latch_inhibit(config['inhibit_path'],reason)
        if not hasattr(self,'abort_watermark'):self.abort_watermark=dict(sim=self.sim(),wall=time.monotonic())
        self.audit['independent_abort']=dict(requested=True,session_id=config['session_id'],reason=str(reason),
                                            physical_stop_verified=False)

    def send(self,goals,execution):
        from nav2_msgs.action import FollowPath
        from unique_identifier_msgs.msg import UUID
        self.failure=self.failure or self.journal.failure
        if self.failure:raise ValueError(self.failure)
        from .motion_qualification import require_motion_qualification
        require_motion_qualification()
        self.verify_recording_capacity(getattr(self,'run_wall_limit',None))
        if self.graph()!=self.registration_graph:raise ValueError('Session changed before preflight')
        self.preflight(goals)
        self.failure=self.failure or self.journal.failure
        if self.failure:raise ValueError(self.failure)
        self.verify_recording_capacity(getattr(self,'run_wall_limit',None))
        self.ingest_dispatch_updates()
        current=self.sample()
        if current['safety']!='clear' or current['speed']>.03 or abs(current['wz'])>.03 or current['command_speed']>.01 or abs(current['command_wz'])>.01:raise ValueError('Robot moved during preflight')
        self.validate_dispatch(current)  # no spin/wait between this check and submission
        # Execute the identical ROS Path object whose samples were checked above.
        from rclpy.serialization import serialize_message,deserialize_message
        action=FollowPath.Goal();action.path=deserialize_message(serialize_message(self.checked_path),type(self.checked_path))
        action.controller_id=execution['controller_id'];action.goal_checker_id=execution['goal_checker_id']
        uid=uuid4();self.owned_id=uid.hex
        self.audit['follow_path_uuid']=self.owned_id
        self.execution_start_sim=self.sim()
        self.send_future=self.nav.send_goal_async(action,goal_uuid=UUID(uuid=list(uid.bytes)))
        deadline=time.monotonic()+3
        while not self.send_future.done() and time.monotonic()<deadline:self.spin()
        if not self.send_future.done():raise ValueError('FollowPath acceptance timeout; cleanup must resolve pending owned handle')
        self.adopt_handle()

    def adopt_handle(self):
        if self.send_future and self.send_future.done() and self.handle is None:
            self.handle=self.send_future.result()
            if not self.handle.accepted:raise ValueError('FollowPath rejected')
            self.result_future=self.handle.get_result_async()

    def tick(self):
        self.spin()
        self.failure=self.failure or self.journal.failure
        now=time.monotonic()
        if now-getattr(self,'last_tick',0)<.1:return {}
        self.last_tick=now
        if self.failure:return dict(failure=self.failure)
        try:
            if now-self.last_graph>1:
                if self.graph()!=self.registration_graph:raise ValueError('Session graph identity changed')
                self.last_graph=now
            if now-self.last_alignment>1:
                if self.age('/map')>2:raise ValueError('Live map stale')
                self.alignment.check(grid(self.messages['/map']));self.last_alignment=now
            s=self.sample();out={}
            if self.last_pose_stamp is None or s['tf_stamp']>self.last_pose_stamp:
                self.last_pose_stamp=s['tf_stamp'];out['sample']=s;self.last_pose_wall=now
            elif now-self.last_pose_wall>.3:out['failure']='missing_fresh_tf_samples'
            if self.result_future and self.result_future.done():out['status']=self.result_future.result().status
            return out
        except Exception as exc:return dict(failure=repr(exc))

    def close(self):
        clean=False;cancel='no_owned_goal';cancel_requested=False;cancel_future=None
        started=time.monotonic();action_deadline=started+5.;stop_deadline=started+35.
        terminal=(self.send_future is None or (self.send_future.done() and self.handle is not None and
                   (not self.handle.accepted or (self.result_future is not None and self.result_future.done()))))
        terminal_wall=started if terminal else None
        terminal_watermark=None
        phase_failed=False;resting=None;last_sample=None
        source_span=wall_span=0.;independent_stop=None;accepted_end=None;contact_deadline=None
        self.audit['cleanup']={}
        try:
            if self.send_future is not None:
                try:self.request_independent_abort('owned_transport_closing')
                except Exception as exc:
                    phase_failed=True;self.failure=self.failure or 'independent_abort_failed:'+repr(exc)
                    self.audit['independent_abort']=dict(requested=False,error=repr(exc),physical_stop_verified=False)
            if hasattr(self,'abort_watermark'):
                from .stopping import IndependentStop
                independent_stop=IndependentStop(self.abort_watermark)
            started_sim=getattr(self,'execution_start_sim',getattr(self,'abort_watermark',{}).get('sim',self.sim()))
            if terminal:terminal_watermark=self.sim()
            while time.monotonic()<stop_deadline:
                self.spin()
                try:
                    self.adopt_handle()
                except ValueError as exc:
                    if self.handle is None or self.handle.accepted:raise
                    self.failure=self.failure or str(exc)
                if self.handle and self.handle.accepted and self.result_future and not self.result_future.done() and not cancel_requested:
                    cancel_requested=True;cancel='exact_owned_uuid_requested'
                    cancel_future=self.handle.cancel_goal_async()
                if cancel_future and cancel_future.done():
                    confirmed=any(bytes(g.goal_id.uuid).hex()==self.owned_id for g in cancel_future.result().goals_canceling)
                    cancel='exact_owned_uuid_confirmed' if confirmed else 'cancel_unconfirmed'
                terminal=(self.send_future is None or (self.send_future.done() and self.handle is not None and
                           (not self.handle.accepted or (self.result_future is not None and self.result_future.done()))))
                if terminal and terminal_wall is None:
                    terminal_wall=time.monotonic();terminal_watermark=self.sim()
                if ((terminal_wall is not None and terminal_wall>action_deadline) or
                        (time.monotonic()>action_deadline and not terminal)):
                    phase_failed=True;self.failure=self.failure or 'owned_action_terminal_timeout'
                try:
                    s=self.sample()
                    if independent_stop is not None:independent_stop.observe(s)
                    if hasattr(self,'cleanup_observer') and s['tf_stamp']>getattr(self,'last_cleanup_tf',getattr(self,'last_pose_stamp',None) or 0)+.05:
                        self.cleanup_observer(s);self.last_cleanup_tf=s['tf_stamp']
                    stopped=(terminal and terminal_watermark is not None and math.isfinite(terminal_watermark) and
                        s['tf_stamp']>=terminal_watermark and
                        all(math.isfinite(s[k]) for k in ('sim','wall','x','y','tf_stamp','odom_stamp','map_odom_stamp','pose_age','clock_age','speed','wz','command_speed','command_wz')) and
                        all(0<=s['sim']-s[k]<=.5 for k in ('tf_stamp','odom_stamp','map_odom_stamp')) and
                        0<=s['pose_age']<=.5 and 0<=s['clock_age']<=.5 and s['speed']<=.03 and abs(s['wz'])<=.03 and
                        s['command_speed']<=.01 and abs(s['command_wz'])<=.01)
                    if not stopped:
                        resting=None;accepted_end=None;contact_deadline=None
                    elif last_sample is None or s['tf_stamp']>last_sample['tf_stamp']:
                        if last_sample and (not 0<s['tf_stamp']-last_sample['tf_stamp']<=.3 or
                                            not 0<s['wall']-last_sample['wall']<=.5):resting=None
                        if resting is None:resting=dict(s)
                        if math.hypot(s['x']-resting['x'],s['y']-resting['y'])>.03:
                            resting=None
                        else:
                            source_span=s['tf_stamp']-resting['tf_stamp'];wall_span=s['wall']-resting['wall']
                            if terminal and source_span>=5. and wall_span>=3.:
                                last_sample=s
                                if accepted_end is None:
                                    accepted_end=s['sim'];contact_deadline=time.monotonic()+2.
                                needs_contact=self.send_future is not None
                                closure=self.contact_closure(started_sim,accepted_end) if needs_contact else dict(reporting_closed_through_end=True,read_only=True)
                                self.audit['contact_closure']=closure
                                if closure['reporting_closed_through_end']:
                                    clean=not phase_failed and not bool(getattr(getattr(self,'contacts',None),'failure',None));break
                                if time.monotonic()>=contact_deadline:
                                    phase_failed=True;self.failure=self.failure or 'contact_interval_not_closed_through_stop';break
                        last_sample=s
                    elif s['tf_stamp']<last_sample['tf_stamp']:
                        resting=None;last_sample=s
                    # Duplicate source poses neither advance nor renew a stop.
                except Exception:
                    resting=None;last_sample=None;accepted_end=None;contact_deadline=None
            self.audit['cleanup'].update(passed=clean,cancel=cancel,owned_action_terminal=bool(terminal),
                action_phase_failed=phase_failed,stationary_confirmed=clean,
                terminal_watermark_sim=terminal_watermark,
                terminal_observed_wall_s=terminal_wall-started if terminal_wall is not None else None,
                stop_source_start_s=resting['tf_stamp'] if resting is not None else None,
                stop_source_span_s=source_span,stop_wall_span_s=wall_span,
                final_sample=last_sample,accepted_stop_end_sim=accepted_end,independent_stop=independent_stop.report() if independent_stop else dict(confirmed=False,reason='no_inhibit_watermark'))
        except Exception as exc:
            self.failure=self.failure or 'cleanup_exception:'+repr(exc)
            self.audit['cleanup'].update(passed=False,stationary_confirmed=False,error=repr(exc))
        finally:
            recording=self.journal.close(timeout=10.)
            if not recording['complete']:
                clean=False;self.failure=self.failure or recording['failure'] or 'incomplete_raw_evidence'
            # An unjoined writer may still change the file after a timeout.
            # Do not claim a final file digest for incomplete recording.
            self.audit.update(raw_journal=recording,raw_sha256=digest(self.raw_path) if recording['complete'] else None,
                              raw_bytes=recording['bytes'],raw_file=str(self.raw_path))
            self.audit['cleanup'].update(passed=clean,stationary_confirmed=clean,recording_complete=recording['complete'])
            self.nav.destroy();self.node.destroy_node();self.rclpy.shutdown()
        return clean
