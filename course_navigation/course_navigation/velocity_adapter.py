"""Limit commands and stop when command/localization becomes stale."""
import math
import json
from . import m2_inverse
from .stage4_qualification import require_motion_qualification
import time
from pathlib import Path
import hashlib
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class VelocityGate:
    def __init__(self, max_linear=0.5, max_angular=0.6, timeout=0.5):
        if not all(math.isfinite(v) for v in [max_linear, max_angular, timeout]) or min(max_linear, max_angular, timeout) <= 0:
            raise ValueError('Limits/timeouts must be positive')
        self.limits = max_linear, max_angular
        self.timeout = timeout
        self.command = (0., 0., 0.)
        self.command_time = self.state_time = None

    def set_command(self, command, now):
        if not all(math.isfinite(x) for x in command):
            self.command_time = None
            return
        x, y, yaw = command
        scale = min(1., self.limits[0] / max(math.hypot(x, y), 1e-9))
        self.command = x * scale, y * scale, max(-self.limits[1], min(self.limits[1], yaw))
        self.command_time = now

    def output(self, now):
        if any(t is None or not 0 <= now - t <= self.timeout for t in [self.command_time, self.state_time]):
            return 0., 0., 0.
        return self.command


class VelocityAdapter(Node):
    def __init__(self):
        super().__init__('velocity_adapter')
        self.m2_enabled = self.declare_parameter('m2_inverse_enabled', False).value
        self.m2_diagnostic = None
        if self.m2_enabled:
            from std_msgs.msg import String
            self.m2_message_type = String
            self.m2_diagnostic = self.create_publisher(String, m2_inverse.DIAGNOSTIC_TOPIC, 10)
            self.get_logger().info('M2 strict diagnostic candidate: ' + json.dumps(m2_inverse.identity(), sort_keys=True))
        self.gate = VelocityGate(self.declare_parameter('max_linear', 0.5).value,
                                 self.declare_parameter('max_angular', 0.6).value,
                                 self.declare_parameter('timeout_s', 0.5).value)
        limit_topic = self.declare_parameter('speed_limit_topic', '').value
        self.speed_limit = None
        if limit_topic:
            if self.m2_enabled:
                raise ValueError('Adaptive RPP uses direct policy commands; M2 must remain disabled')
            from nav2_msgs.msg import SpeedLimit
            from .adaptive_speed import CommandLimit
            self.speed_limit = CommandLimit(self.gate.limits[0], self.gate.timeout)
            self.create_subscription(SpeedLimit, limit_topic, self.receive_speed_limit, 10)
        source = self.declare_parameter('input_topic', '/course_navigation/cmd_vel').value
        state_topic = self.declare_parameter('state_topic', '/slam/odom').value
        self.require_tf = self.declare_parameter('require_localization_tf', True).value
        self.pub = self.create_publisher(Twist, '/path_manager/path_manager_ros/nav_vel', 10)
        self.create_subscription(Twist, source, self.command, 10)
        self.create_subscription(Odometry, state_topic, self.state, 20)
        self.last_stamp = None
        self.localization_fault = False
        self.clock_stamp = self.clock_wall = self.last_wall = None
        self.tf_state = {}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.stage4_guard = None
        self.stage4_guard_manifest = self.declare_parameter('stage4_guard_manifest', '').value
        if self.stage4_guard_manifest:
            if self.m2_enabled or self.speed_limit is not None or not self.require_tf:
                raise ValueError('Stage4 requires direct commands, public SLAM and exclusive final guard')
            from .stage4_guard import FinalCommandGuard, GridPreparation
            from .stage4_control import read_control
            from nav_msgs.msg import OccupancyGrid
            from nav2_msgs.msg import Costmap
            config=read_control(self.stage4_guard_manifest)
            from rcl_interfaces.msg import ParameterDescriptor
            self.declare_parameter('stage4_guard_sha256',hashlib.sha256(Path(self.stage4_guard_manifest).read_bytes()).hexdigest(),
                descriptor=ParameterDescriptor(read_only=True),ignore_override=True)
            self.declare_parameter('stage4_session_id',config['session_id'],
                descriptor=ParameterDescriptor(read_only=True),ignore_override=True)
            evidence=Path(config['bounds_evidence_path'])
            if hashlib.sha256(evidence.read_bytes()).hexdigest()!=config['bounds_evidence_sha256']:
                raise ValueError('Stop/uncertainty evidence identity changed')
            from .stage4_bounds import validate_bounds
            self.stage4_bounds=validate_bounds(config,json.loads(evidence.read_text()))
            self.stage4_guard=FinalCommandGuard(config)
            from .stage4_contact_stream import ContactStream
            from std_msgs.msg import String
            from ros_gz_interfaces.msg import Contacts
            inventory=Path(config['collision_inventory_path'])
            if hashlib.sha256(inventory.read_bytes()).hexdigest()!=config['collision_inventory_sha256']:raise ValueError('Source collision inventory changed')
            names=json.loads(inventory.read_text())['collisions']
            self.stage4_guard.contacts=ContactStream(names,first_sequence=config['contact_first_sequence'])
            self.create_subscription(String,'/b2w/evaluation/collision_inventory',lambda m:self.stage4_contact('inventory',m),10)
            self.create_subscription(Contacts,'/b2w/evaluation/full_contacts',lambda m:self.stage4_contact('contacts',m),10)
            self.stage4_worker=GridPreparation(self.stage4_guard)
            saved=Path(config['saved_grid_path'])
            if hashlib.sha256(saved.read_bytes()).hexdigest()!=config['saved_grid_sha256']:
                raise ValueError('Registered saved grid identity changed')
            self.stage4_worker.submit('saved',json.loads(saved.read_text()),frame='map',source=0.,receipt=0.,
                encoding='costmap',epoch=config['session_id'])
            self.create_subscription(OccupancyGrid,'/map',lambda m:self.stage4_grid('live',m,False),10)
            self.create_subscription(Costmap,'/local_costmap/costmap_raw',lambda m:self.stage4_grid('local',m,True),10)
            self.create_subscription(Costmap,'/global_costmap/costmap_raw',lambda m:self.stage4_grid('global',m,True),10)
        self.create_timer(0.05, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def state(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        wall = time.monotonic()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        expected_frame = 'odom' if self.require_tf else 'sim_world'
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.localization_fault = True
        if (not self.clock_valid(now, wall) or msg.header.frame_id != expected_frame or
                msg.child_frame_id != 'base_link' or
                not 0 <= msg.header.stamp.nanosec < 1000000000 or
                not all(math.isfinite(x) for x in [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                                                   v.x, v.y, v.z, w.x, w.y, w.z]) or
                abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .01 or
                not 0 <= now - stamp <= self.gate.timeout):
            self.gate.state_time = None
            return
        if stamp == self.last_stamp:
            return  # A cached sample never renews the wall watchdog.
        self.last_stamp = stamp
        self.last_odom = msg
        self.gate.state_time = wall

    def stage4_contact(self,kind,msg):
        try:
            gate=self.stage4_guard.contacts
            if kind=='inventory':gate.inventory(json.loads(msg.data),wall=time.monotonic())
            else:gate.contact_message(msg,wall=time.monotonic())
            if gate.failure:self.stage4_guard.inhibit(gate.failure)
        except Exception as exc:self.stage4_guard.inhibit('contact_callback:'+repr(exc))

    def stage4_grid(self,name,msg,costmap):
        info=msg.metadata if costmap else msg.info;o=info.origin;q=o.orientation
        try:
            width=info.size_x if costmap else info.width;height=info.size_y if costmap else info.height
            if width<=0 or height<=0 or width*height!=len(msg.data) or len(msg.data)>1000000:
                raise ValueError('Oversized or malformed grid')
            if (abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01 or
                    abs(q.x)>1e-6 or abs(q.y)>1e-6):raise ValueError('Invalid/nonplanar grid quaternion')
            angle=math.atan2(2*(q.w*q.z+q.x*q.y),q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z)
            grid=dict(width=info.size_x if costmap else info.width,height=info.size_y if costmap else info.height,
                      resolution=info.resolution,origin=[o.position.x,o.position.y,angle],data=tuple(msg.data))
            self.stage4_worker.submit(name,grid,frame=msg.header.frame_id,
                source=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,receipt=time.monotonic(),
                encoding='costmap' if costmap else 'occupancy',epoch=self.stage4_guard.config['session_id'])
        except Exception as exc:self.stage4_guard.inhibit('grid_callback:'+repr(exc))

    def stage4_context(self):
        odom=self.last_odom;target=Time.from_msg(odom.header.stamp);poses={}
        for frame in ('map','odom'):
            tr=self.tf.lookup_transform(frame,'base_link',target)
            stamp=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9
            if abs(stamp-self.last_stamp)>1e-9 or tr.header.frame_id!=frame or tr.child_frame_id!='base_link':
                raise ValueError('Stage4 requires coherent source-time TF')
            p,q=tr.transform.translation,tr.transform.rotation
            if not all(math.isfinite(v) for v in [p.x,p.y,p.z,q.x,q.y,q.z,q.w]) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01:
                raise ValueError('Invalid Stage4 pose')
            poses[frame]=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z)]
            if frame=='odom':odom_tf=tr.transform
        oq=odom.pose.pose.orientation
        oyaw=math.atan2(2*(oq.w*oq.z+oq.x*oq.y),oq.w*oq.w+oq.x*oq.x-oq.y*oq.y-oq.z*oq.z)
        qa=[getattr(oq,k) for k in 'xyzw'];qb=[getattr(odom_tf.rotation,k) for k in 'xyzw']
        dot=abs(sum(a*b for a,b in zip(qa,qb)))/math.sqrt(sum(a*a for a in qa)*sum(b*b for b in qb))
        if (math.dist([getattr(odom_tf.translation,k) for k in 'xyz'],[getattr(odom.pose.pose.position,k) for k in 'xyz'])>1e-5 or
                dot<math.cos(1e-5/2) or abs(math.remainder(poses['odom'][2]-oyaw,math.tau))>1e-5):
            raise ValueError('Stage4 odometry/TF disagreement')
        speed=math.hypot(odom.twist.twist.linear.x,odom.twist.twist.linear.y);wz=abs(odom.twist.twist.angular.z)
        old=getattr(self,'stage4_previous_pose',None)
        if old is not None and self.last_stamp>old[0]:
            dt=self.last_stamp-old[0];speed=max(speed,math.dist(poses['map'][:2],old[1][:2])/dt)
            wz=max(wz,abs(math.remainder(poses['map'][2]-old[1][2],math.tau))/dt)
        if old is None or self.last_stamp>old[0]:self.stage4_previous_pose=(self.last_stamp,poses['map'])
        return dict(poses=poses,pose_stamp=self.last_stamp,pose_receipt=self.gate.state_time,
                    sim=self.get_clock().now().nanoseconds*1e-9,wall=time.monotonic(),speed=speed,wz=wz)

    def clock_valid(self, sim, wall):
        """Rollback latches the session closed; suspension expires in wall time."""
        if (not math.isfinite(sim) or not math.isfinite(wall) or sim < 0 or
                (self.clock_stamp is not None and sim < self.clock_stamp) or
                (self.last_wall is not None and wall < self.last_wall)):
            self.localization_fault = True
        if self.clock_stamp is None or sim > self.clock_stamp:
            self.clock_wall = wall
        self.clock_stamp, self.last_wall = sim, wall
        return (not self.localization_fault and self.clock_wall is not None and
                0 <= wall - self.clock_wall <= self.gate.timeout)

    def localization_valid(self):
        sim = self.get_clock().now().nanoseconds * 1e-9
        wall = time.monotonic()
        if (not self.clock_valid(sim, wall) or self.last_stamp is None or
                not 0 <= sim - self.last_stamp <= self.gate.timeout):
            return False
        for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
            tf = self.tf.lookup_transform(parent, child, Time())
            t, q = tf.transform.translation, tf.transform.rotation
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            old = self.tf_state.get((parent, child))
            if old is not None and stamp < old[0]:
                self.localization_fault = True
                return False
            if (tf.header.frame_id != parent or tf.child_frame_id != child or
                    not 0 <= tf.header.stamp.nanosec < 1000000000 or
                    not all(math.isfinite(v) for v in [t.x, t.y, t.z, q.x, q.y, q.z, q.w]) or
                    abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .01 or
                    not 0 <= sim - stamp <= self.gate.timeout):
                return False
            if old is None or stamp > old[0]:
                self.tf_state[parent, child] = stamp, wall
            if not 0 <= wall - self.tf_state[parent, child][1] <= self.gate.timeout:
                return False
        return True

    def command(self, msg):
        if self.m2_enabled:
            # Check before legacy limiting as well: never hide an unsupported request.
            self.convert_m2((msg.linear.x, msg.linear.y, msg.angular.z))
        self.gate.set_command((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic())

    def receive_speed_limit(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.speed_limit.receive(msg.speed_limit, msg.percentage, stamp, time.monotonic())

    def convert_m2(self, command):
        try:
            result = m2_inverse.transform(command, enabled=True)
        except m2_inverse.UnsupportedVelocity as exc:
            self.report_m2(command, None, 'unsupported', str(exc))
            self.get_logger().fatal(str(exc) + '; strict candidate terminates; no inverse fallback')
            raise  # Stop at the design boundary; do not publish a substitute Twist.
        self.report_m2(command, result, 'accepted', None)
        return result

    def report_m2(self, desired, policy, status, reason):
        msg = self.m2_message_type()
        msg.data = json.dumps({'candidate': m2_inverse.identity(), 'status': status,
                               'desired': desired, 'policy': policy, 'reason': reason})
        self.m2_diagnostic.publish(msg)

    def publish(self):
        publication_started=time.perf_counter()
        publication_wall_started=time.monotonic()
        out = Twist()
        try:
            if self.require_tf:
                if not self.localization_valid():
                    if getattr(self,'stage4_guard',None) is not None and self.stage4_guard.armed:
                        self.stage4_guard.inhibit('adapter_localization_lost')
                    self.pub.publish(out)
                    return
            if getattr(self,'stage4_guard',None) is not None and self.stage4_guard.armed:
                wall=time.monotonic()
                if any(t is None or not 0<=wall-t<=self.gate.timeout for t in (self.gate.command_time,self.gate.state_time)):
                    self.stage4_guard.inhibit('adapter_command_or_state_expired')
            command = self.gate.output(time.monotonic())
            if getattr(self, 'speed_limit', None) is not None:
                command = self.speed_limit.apply(command, self.get_clock().now().nanoseconds*1e-9,
                                                 time.monotonic())
            if self.m2_enabled:
                command = self.convert_m2(command)
            if getattr(self,'stage4_guard',None) is not None:
                context=self.stage4_context()
                if hasattr(self,'stage4_bounds'):
                    scope=self.stage4_bounds['scope'];x,y=context['poses']['map'][:2];a,b,c,d=scope['region']
                    if not (scope['valid_from_sim']<=context['sim']<=scope['valid_until_sim'] and a<=x<=c and b<=y<=d):
                        raise ValueError('Conservative bound scope expired or left')
                command=self.stage4_guard.output(command=command,**context)
                if any(abs(v)>1e-12 for v in command):require_motion_qualification()
            out.linear.x, out.linear.y, out.angular.z = command
        except TransformException:
            if getattr(self,'stage4_guard',None) is not None and self.stage4_guard.armed:
                self.stage4_guard.inhibit('localization_transform_failure')
        except Exception as exc:
            if getattr(self,'stage4_guard',None) is None:raise
            self.stage4_guard.inhibit('final_guard:'+repr(exc))
        guard=getattr(self,'stage4_guard',None)
        if guard is not None and time.perf_counter()-publication_started>=.05:
            guard.inhibit('adapter_publication_deadline');out=Twist()
        self.pub.publish(out)
        if guard is not None and guard.config.get('adapter_heartbeat_path'):
            from .stage4_control import write_control
            self.stage4_health_sequence=getattr(self,'stage4_health_sequence',0)+1
            try:write_control(guard.config['adapter_heartbeat_path'],dict(session_id=guard.config['session_id'],
                sequence=self.stage4_health_sequence,wall=publication_wall_started,adapter=guard.adapter_identity,
                prepared=guard.prepared_status()))
            except Exception as exc:guard.inhibit('adapter_health_write:'+repr(exc))
            if time.perf_counter()-publication_started>=.05:
                guard.inhibit('adapter_health_publication_deadline');self.pub.publish(Twist())

    def stop_stage4(self):
        if getattr(self,'stage4_guard',None) is not None:
            self.stage4_guard.inhibit('adapter_shutdown')
            try:self.pub.publish(Twist())
            finally:self.stage4_worker.close()
            return True
        return True


def main():
    rclpy.init()
    node = VelocityAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:node.stop_stage4()
        finally:
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
