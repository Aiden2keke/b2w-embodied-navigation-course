"""USER-RUN only. Attach to one idle stack; never launch simulation or publish velocity.

A bounded raw journal preserves public messages. No goal is sent until candidate,
registration, installed endpoint guard, effective parameters and exclusive graph
checks succeed. All cancellation uses the exact owned action handle.
"""
import base64
import hashlib
import json
import math
import os
from pathlib import Path
import time
from uuid import uuid4
import yaml
from .contract import ROOT,load_selection,digest,object_hash,write_new,check_registration
from .spatial import MapAlignment,footprint_clear,saved_grid,assess_path


def stamp(t):return t.sec+t.nanosec*1e-9


def yaw(q):
    v=[q.x,q.y,q.z,q.w]
    if not all(math.isfinite(x) for x in v) or abs(sum(x*x for x in v)-1)>.01:raise ValueError('Invalid quaternion')
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def grid(msg,costmap=False):
    m=msg.metadata if costmap else msg.info;o=m.origin
    return dict(width=m.size_x if costmap else m.width,height=m.size_y if costmap else m.height,
                resolution=m.resolution,origin=[o.position.x,o.position.y,yaw(o.orientation)],data=list(msg.data))


class RosTransport:
    def __init__(self,directory):
        # Importing this module for offline tests is safe; construction is live opt-in.
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        from nav2_msgs.action import FollowPath
        from tf2_ros import Buffer,TransformListener
        self.rclpy=rclpy;rclpy.init();self.node=Node('stage4_route_'+uuid4().hex[:8],parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.directory=Path(directory);self.raw_path=self.directory/('raw-'+uuid4().hex+'.jsonl');self.raw=self.raw_path.open('xb');self.bytes=0;self.seq=0
        self.tf=Buffer();self.listener=TransformListener(self.tf,self.node)
        self.nav=ActionClient(self.node,FollowPath,'/follow_path')
        self.messages={};self.receipts={};self.statuses={};self.failure=None;self.handle=None
        self.send_future=None;self.result_future=None;self.owned_id=None;self.preflight_id=None;self.child_ids={};self.last_pose_stamp=None
        self.saved=None;self.audit=dict(child_processes=[],action_outcomes=[],cleanup={},collision_coverage='plate_only_full_body_unknown')
        self.qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.transient=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.action_qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        from geometry_msgs.msg import PoseStamped,Twist,WrenchStamped
        from nav_msgs.msg import Odometry,OccupancyGrid,Path as NavPath
        from sensor_msgs.msg import PointCloud2,Imu
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        from rcl_interfaces.msg import Log
        from action_msgs.msg import GoalStatusArray
        from nav2_msgs.msg import Costmap,BehaviorTreeLog
        topics=[('/clock',Clock),('/slam/odom',Odometry),('/slam/map_points',PointCloud2),('/pointcloud',PointCloud2),('/imu',Imu),
                ('/map',OccupancyGrid),('/tf',TFMessage),('/tf_static',TFMessage),
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
        self.seq+=1
        # PointCloud payloads can be multi-MiB: retain stamp/size/hash, not full payload.
        data=serialize_message(msg)
        record=dict(seq=self.seq,topic=topic,wall=time.time(),monotonic=now,sim=self.sim(),sha256=hashlib.sha256(data).hexdigest())
        if topic in ('/slam/map_points','/pointcloud'):
            record.update(header=dict(frame_id=msg.header.frame_id,stamp=stamp(msg.header.stamp)),bytes=len(data),payload_retained=False)
        else:record['cdr_base64']=base64.b64encode(data).decode()
        encoded=(json.dumps(record,separators=(',',':'))+'\n').encode()
        if self.bytes+len(encoded)>256*1024**2:self.failure='raw_evidence_limit';return
        self.raw.write(encoded);self.bytes+=len(encoded)
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

    def sample(self):
        from rclpy.time import Time
        now=self.sim();odom=self.messages['/slam/odom']
        if odom.header.frame_id!='odom' or odom.child_frame_id!='base_link':raise ValueError('Unexpected SLAM odom convention')
        mb=self.tf.lookup_transform('map','base_link',Time());mo=self.tf.lookup_transform('map','odom',Time())
        ob=self.tf.lookup_transform('odom','base_link',Time())
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
                    slam_map_age=max(self.age('/slam/map_points'),now-stamp(self.messages['/slam/map_points'].header.stamp)),
                    clock_age=self.age('/clock'),pose_age=self.age('/slam/odom'),tf_receipt_age=self.age('/tf'),
                    speed=speed,wz=wz,command_speed=math.hypot(cmd.linear.x,cmd.linear.y),command_wz=cmd.angular.z,safety=safety)
        if (any(not math.isfinite(v) for v in result.values() if type(v) in (int,float)) or
            any(not 0<=now-result[k]<=.5 for k in ('tf_stamp','map_odom_stamp','odom_stamp')) or
            any(not 0<=result[k]<=.5 for k in ('clock_age','pose_age','tf_receipt_age')) or
            not 0<=result['slam_map_age']<=2):raise ValueError('Stale/missing/nonfinite SLAM/TF evidence')
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
        self.get_parameters('/velocity_adapter',dict(use_sim_time=True,m2_inverse_enabled=False,speed_limit_topic='',require_localization_tf=True,input_topic='/course_navigation/cmd_vel',state_topic='/slam/odom',max_linear=.5,max_angular=.6,timeout_s=.5))
        # Mandatory reviewed endpoint-transform guard: verify executable AND mapped library.
        prefix=ROOT/'.deps/nav2-controller-1.1.20/install/nav2_controller'
        exe=prefix/'lib/nav2_controller/controller_server';library=prefix/'lib/libcontroller_server_core.so'
        processes=[]
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():continue
            try:
                if (proc/'exe').resolve()==exe.resolve() and str(library) in (proc/'maps').read_text():
                    processes.append(dict(pid=int(proc.name),exe_sha256=digest(exe),library_sha256=digest(library)))
            except (OSError,PermissionError):pass
        policy=json.loads((ROOT/'course_navigation/config/stage4_runtime_identity.json').read_text())
        if len(processes)!=1 or any(processes[0][k]!=policy[k] for k in ('exe_sha256','library_sha256')):
            raise ValueError('Require one verified endpoint-transform-guard controller process')
        self.audit['controller_process']=processes[0]
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
                c=dict(session_id=uuid4().hex,graph=graph,wall=time.time(),sim=self.sim(),selection_sha256=digest(selection_path),
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
            report=assess_path(poses,goals,[s['x'],s['y']],[self.saved,grid(cm,True)])
            report.update(action_status=result.status,path=poses,global_costmap_sha256=object_hash(grid(cm,True)))
            self.audit['preflight']=report
            self.checked_path=path
            if not report['passed']:raise ValueError('Pre-motion sampled path/corridor/footprint check FAILED; no navigation goal sent')
            if self.graph()!=self.registration_graph:raise ValueError('Session changed during preflight')
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

    def send(self,goals,execution):
        from nav2_msgs.action import FollowPath
        from unique_identifier_msgs.msg import UUID
        if self.failure:raise ValueError(self.failure)
        if self.graph()!=self.registration_graph:raise ValueError('Session changed before preflight')
        self.preflight(goals)
        if self.failure:raise ValueError(self.failure)
        current=self.sample()
        if current['safety']!='clear' or current['speed']>.03 or abs(current['wz'])>.03 or current['command_speed']>.01 or abs(current['command_wz'])>.01:raise ValueError('Robot moved during preflight')
        # Execute the identical ROS Path object whose samples were checked above.
        action=FollowPath.Goal();action.path=self.checked_path
        action.controller_id=execution['controller_id'];action.goal_checker_id=execution['goal_checker_id']
        uid=uuid4();self.owned_id=uid.hex
        self.audit['follow_path_uuid']=self.owned_id
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
        clean=False;cancel='no_owned_goal';deadline=time.monotonic()+5
        try:
            while self.send_future and not self.send_future.done() and time.monotonic()<deadline:self.spin()
            self.adopt_handle()
            if self.result_future and not self.result_future.done():
                f=self.handle.cancel_goal_async();cancel='exact_owned_uuid_requested'
                while not f.done() and time.monotonic()<deadline:self.spin()
                if not f.done() or not any(bytes(g.goal_id.uuid).hex()==self.owned_id for g in f.result().goals_canceling):cancel='cancel_unconfirmed'
                while not self.result_future.done() and time.monotonic()<deadline:self.spin()
            terminal=(self.send_future is None or (self.send_future.done() and self.handle and
                       (not self.handle.accepted or (self.result_future and self.result_future.done()))))
            resting=None;last_tf=None
            while time.monotonic()<deadline:
                self.spin()
                try:
                    s=self.sample()
                    if hasattr(self,'cleanup_observer') and s['tf_stamp']>getattr(self,'last_cleanup_tf',getattr(self,'last_pose_stamp',None) or 0)+.05:
                        self.cleanup_observer(s);self.last_cleanup_tf=s['tf_stamp']
                    if (s['tf_stamp']!=last_tf and all(0<=s['sim']-s[k]<=.5 for k in ('tf_stamp','odom_stamp','map_odom_stamp')) and
                        s['pose_age']<=.5 and s['clock_age']<=.5 and s['speed']<=.03 and abs(s['wz'])<=.03 and
                        s['command_speed']<=.01 and abs(s['command_wz'])<=.01):
                        resting=s['sim'] if resting is None else resting;last_tf=s['tf_stamp']
                        if s['sim']-resting>=1.:clean=bool(terminal);break
                    elif (s['tf_stamp']==last_tf and s['pose_age']<=.5 and s['clock_age']<=.5 and
                          s['speed']<=.03 and abs(s['wz'])<=.03 and s['command_speed']<=.01 and abs(s['command_wz'])<=.01):
                        pass  # Repeated fresh TF cannot extend or reset a measured stop interval.
                    else:resting=None
                except Exception:resting=None
            self.audit['cleanup']=dict(passed=clean,cancel=cancel,owned_action_terminal=bool(terminal),stationary_confirmed=clean,final_sample=getattr(self,'previous_sample',None))
        finally:
            self.raw.flush();self.raw.close();self.audit['raw_sha256']=digest(self.raw_path);self.audit['raw_bytes']=self.bytes;self.audit['raw_file']=str(self.raw_path)
            self.nav.destroy();self.node.destroy_node();self.rclpy.shutdown()
        return clean
