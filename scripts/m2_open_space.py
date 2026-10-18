#!/usr/bin/env python3
"""ONE prospective M2 adapter open-space diagnostic; default dry-run, USER-RUN only.

Feed supported physical steps directly to the shared adapter input. No Nav2 or
velocity smoother is launched: its low-speed ramps have no validated inverse.
This checks adapter -> policy -> truth and never authorizes dynamic Nav2 use.
Reuse the existing fixture, native recorder and owned cleanup without changing
Protocol A/B/C tools or captures. Ground truth is evaluation/instrumentation only.
"""
from __future__ import annotations
import argparse
import datetime as dt
import hashlib
import json
import math
import os
import secrets
import shutil
import signal
import sys
import time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT)); sys.path.insert(0,str(ROOT/'course_navigation'))
from course_navigation import m2_inverse as m2
from scripts import run_velocity_steps as base
from scripts.velocity_step_core import analyze_run, make_sequence, requested_at, _event_time, _path_metrics
from scripts.fit_velocity_calibration import authenticate, series, window_stats

SPEEDS=[.12,.15,.20,.25]
GATES={'truth_max_abs_error_mps':.010,'truth_rmse_mps':.007,'truth_monotonic':True,
       'target_lateral_m_per_5s':.03,'target_yaw_rad_per_5s':.05,
       'truth_reverse_tolerance_mps':.01,'final_zero_to_stop_s':.8,'final_zero_to_stop_path_m':.05,
       'all_points_authenticated':True,'candidate_active':True,'no_unsupported_event':True,'cleanup_pass':True}
READINESS_CHECKS=('fresh_streams','source_fresh','valid_slam','valid_imu','valid_joints','body_frames',
                  'tf_available','unique_publishers','unique_clock','controller',
                  'no_navigation_or_smoother','candidate_active','epoch_stable','no_runtime_error')
CANDIDATE_SOURCES=['course_navigation/course_navigation/m2_inverse.py',
                   'course_navigation/course_navigation/velocity_adapter.py','scripts/m2_open_space.py']


def source_hashes():
    return {p:base.sha256(ROOT/p) for p in CANDIDATE_SOURCES}


def plan():
    return {'candidate':m2.identity(),'validated_condition':'pure_forward_vy0_wz0',
            'deferred_due_to':'LOW_SPEED_SUPPORT_BLOCKER','turning_lateral_invariance_validated':False,'desired_vx_mps':SPEEDS,'policy_u':[m2.inverse(v) for v in SPEEDS],
            'sequence':make_sequence(SPEEDS),'prospective_gates':GATES,
            'chain':[base.SHARED_TOPIC,'velocity_adapter:m2_inverse_enabled=true',base.FINAL_TOPIC,'ONNX obs[6:9]','truth evaluation'],
            'smoother':'not launched; direct supported step input; no ramp or low-speed surrogate',
            'dynamic_nav2_authorized':False,'default':'dry_run; --execute required',
            'unsupported':'diagnostic + exception; no substituted command; route design boundary',
            'stop_definition':'truth planar <=.02m/s and |yaw_rate|<=.05rad/s for supported .5s; same native metric as existing harness'}


def start_ready(checks, final_values, stand_s):
    return bool(checks and all(checks.values()) and final_values is not None and
                all(v == 0. for v in final_values) and stand_s >= 2.2)


def report(events,manifest):
    # Existing native metrics supply truth steady validity and conservative stop/path.
    analysis=analyze_run(events,{**manifest,'sequence':make_sequence(SPEEDS),'raw_topic':base.SHARED_TOPIC})
    auth=authenticate(manifest)
    diagnostics=[e for e in events if e.get('topic')==m2.DIAGNOSTIC_TOPIC]
    unsupported=sum(e.get('data',{}).get('status')!='accepted' for e in diagnostics)
    quality=analysis['quality']; reasons=[]
    if not auth['valid']: reasons+=auth['reasons']
    if manifest.get('candidate')!=m2.identity():reasons.append('candidate identity mismatch')
    if manifest.get('candidate_source_hashes')!=source_hashes():reasons.append('candidate source hashes mismatch')
    readiness=manifest.get('readiness',{})
    if not readiness.get('completed') or not all(readiness.get('checks',{}).get(k) is True for k in READINESS_CHECKS):reasons.append('readiness incomplete/failed checks')
    if manifest.get('sequence')!=make_sequence(SPEEDS):reasons.append('frozen sequence mismatch')
    if manifest.get('prospective_plan')!=plan():reasons.append('prospective plan/gates mismatch')
    if manifest.get('termination',{}).get('status')!='completed':reasons.append('run incomplete')
    if not manifest.get('cleanup',{}).get('passed'):reasons.append('cleanup failed')
    if unsupported:reasons.append('unsupported-domain/invalid diagnostic event')
    if quality['clock_epoch_crossing'] or quality['clock_rollback'] or quality['nonfinite_event_count']:reasons.append('invalid clock or nonfinite data')
    origin=manifest.get('experiment_start_sim_s')
    # Check all phases, including stopping, for unsupported commands or axis/sign
    # corruption. Transition timing jitter is allowed; unsupported values are not.
    if origin is not None:
        clocks=window_stats(series(events,'/clock',lambda d:d['clock_s']),origin,origin+48.,gap=.15,zoh=True)
        if clocks['mean'] is None:reasons.append('full-sequence clock coverage <95%')
        for topic in (base.REQUEST_TOPIC,base.SHARED_TOPIC,base.FINAL_TOPIC,'/network_input_debug'):
            extract=(lambda d:d['data'][6] if len(d['data'])==57 else None) if topic=='/network_input_debug' else (lambda d:d['linear_x'])
            coverage=window_stats(series(events,topic,extract),origin,origin+48.,gap=.05 if topic=='/network_input_debug' else .075,zoh=True)
            if coverage['mean'] is None:reasons.append('full-sequence command coverage <95%: '+topic)
        for event in events:
            topic=event.get('topic'); t=_event_time(event)
            if topic not in (base.REQUEST_TOPIC,base.SHARED_TOPIC,base.FINAL_TOPIC,'/network_input_debug') or t is None or not origin<=t<origin+48.:continue
            d=event.get('data',{})
            vals=d.get('data',[])[6:9] if topic=='/network_input_debug' else [d.get('linear_x'),d.get('linear_y'),d.get('angular_z')]
            domain=(m2.U_MIN,m2.U_MAX) if topic in (base.FINAL_TOPIC,'/network_input_debug') else (m2.V_MIN,m2.V_MAX)
            if len(vals)!=3 or any(v is None or not math.isfinite(v) for v in vals) or not (vals[0]==0. or domain[0]<=vals[0]<=domain[1]) or vals[1]!=0. or vals[2]!=0.:
                reasons.append('unsupported command/sign/axis corruption across full sequence: '+topic);break
    truth_events=[e for e in events if e.get('topic')==base.TRUTH_TOPIC]
    for event in truth_events:
        source=event.get('source_s');d=event.get('data',{});q=d.get('quaternion_xyzw',[]);pos=d.get('position',[])
        if not isinstance(source,(float,int)) or not math.isfinite(source) or len(q)!=4 or len(pos)!=3 or any(v is None or not math.isfinite(v) for v in [*q,*pos]) or abs(sum(v*v for v in q)-1.)>.01 or event.get('child_frame_id')!='base_link':
            reasons.append('truth source stamps/pose/quaternion invalid');break
    points=[]
    for row in analysis['per_speed']:
        desired=row['requested_vx_mps'];expected=m2.inverse(desired);a,b=row['steady_state_window_s'];t0,t1=row['target_window_s']
        invalid=[]
        clocks=window_stats(series(events,'/clock',lambda d:d['clock_s']),t0,t1,gap=.15,zoh=True)
        if clocks['mean'] is None:invalid.append('clock coverage <95%')
        def stats(topic,extract,gap=.075):return window_stats(series(events,topic,extract),a,b,gap=gap,zoh=True)
        def matches(s,value):return s['mean'] is not None and s['min'] is not None and max(abs(s['min']-value),abs(s['max']-value))<=1e-6
        final=stats(base.FINAL_TOPIC,lambda d:d['linear_x'])
        observed=stats('/network_input_debug',lambda d:d['data'][6] if len(d['data'])==57 else None,.05)
        for topic,target,fields in [(base.REQUEST_TOPIC,desired,('linear_x','linear_y','angular_z')),
                                    (base.SHARED_TOPIC,desired,('linear_x','linear_y','angular_z')),
                                    (base.FINAL_TOPIC,expected,('linear_x','linear_y','angular_z')),
                                    ('/network_input_debug',expected,(6,7,8))]:
            for field,value in zip(fields,(target,0.,0.)):
                extract=(lambda d,k=field:d['data'][k] if len(d['data'])==57 else None) if isinstance(field,int) else (lambda d,k=field:d[k])
                if not matches(stats(topic,extract,.05 if isinstance(field,int) else .075),value):invalid.append(f'plateau mismatch/missing {topic}:{field}')
        active=stats(m2.DIAGNOSTIC_TOPIC,lambda d:1. if d.get('status')=='accepted' and d.get('candidate')==m2.identity() else 0.)
        if not matches(active,1.):invalid.append('candidate not authenticated active throughout steady window')
        truth=row['truth']['steady_state_mean_vx_mps']
        if truth is None:invalid.append('truth steady window invalid')
        truth_events=[e for e in events if e.get('topic')==base.TRUTH_TOPIC and (t:=_event_time(e)) is not None and t0<=t<t1]
        if not truth_events or any(e.get('child_frame_id')!='base_link' for e in truth_events):invalid.append('truth frame unauthenticated')
        reversal=any(e.get('data',{}).get('linear',[0.])[0] < -GATES['truth_reverse_tolerance_mps'] for e in truth_events)
        pose=_path_metrics(events,base.TRUTH_TOPIC,t0,t1,.15)
        if pose['coverage_fraction']<.95:invalid.append('target pose coverage <95%')
        # Detect sign/axis corruption across every received command, including transitions.
        for e in events:
            if e.get('topic') not in (base.REQUEST_TOPIC,base.SHARED_TOPIC,base.FINAL_TOPIC,'/network_input_debug'):continue
            t=_event_time(e)
            if t is None or not t0+.15<=t<t1:continue
            d=e.get('data',{})
            vals=d.get('data',[])[6:9] if e['topic']=='/network_input_debug' else [d.get('linear_x'),d.get('linear_y'),d.get('angular_z')]
            if len(vals)!=3 or any(v is None or not math.isfinite(v) for v in vals) or vals[0]<0 or abs(vals[1])>1e-6 or abs(vals[2])>1e-6:
                invalid.append('sign reversal/axis corruption');break
        stop=dict(row['stop'])
        fz=stop['final_nav_zero_s']; ts=stop['truth_effective_stop_s']
        if fz is not None and ts is not None and ts>=fz:
            stop_pose=_path_metrics(events,base.TRUTH_TOPIC,fz,ts,.15)
            if ts==fz:
                exact=any(e.get('topic')==base.TRUTH_TOPIC and _event_time(e)==ts and e.get('valid',True) and e.get('data',{}).get('position') for e in events)
                if exact:
                    stop_pose={'path_distance_m':0.,'coverage_fraction':1.}
                    stop['instant_stop_same_stamp']=True
            stop['final_zero_to_stop_path_distance_m']=stop_pose['path_distance_m']
            stop['final_zero_to_stop_pose_coverage']=stop_pose['coverage_fraction']
            if stop_pose['coverage_fraction']<.95:invalid.append('stopping pose coverage <95%')
        points.append({'desired_physical_vx':desired,'calibrated_policy_u':expected,
                       'final_policy_command':final,'observed_onnx_command':observed,'truth_steady_vx':truth,
                       'truth_minus_desired':truth-desired if truth is not None else None,
                       'truth_sign_reversal':reversal,'target_phase':pose,'stop':stop,'valid':not invalid,'invalid_reasons':invalid})
    if [p['desired_physical_vx'] for p in points]!=SPEEDS:reasons.append('missing/reordered points')
    data=not reasons and all(p['valid'] for p in points) and len(points)==4
    errors=[p['truth_minus_desired'] for p in points]
    complete=bool(points) and all(e is not None for e in errors)
    rmse=math.sqrt(sum(e*e for e in errors)/len(errors)) if complete else None
    max_error=max(map(abs,errors)) if complete else None
    truth_values=[p['truth_steady_vx'] for p in points]
    monotonic=complete and all(b>=a for a,b in zip(truth_values,truth_values[1:]))
    tracking=bool(complete and max_error<=GATES['truth_max_abs_error_mps'] and rmse<=GATES['truth_rmse_mps'] and monotonic)
    def within(v,limit):return v is not None and math.isfinite(v) and abs(v)<=limit
    stability=bool(points) and all(not p['truth_sign_reversal'] and within(p['target_phase']['lateral_displacement_m'],.03) and
        within(p['target_phase']['yaw_drift_rad'],.05) and
        p['stop']['final_zero_to_truth_stop_s'] is not None and 0<=p['stop']['final_zero_to_truth_stop_s']<=.8 and
        within(p['stop']['final_zero_to_stop_path_distance_m'],.05) for p in points)
    return {'candidate':manifest.get('candidate'),'validated_condition':'pure_forward_vy0_wz0',
            'turning_lateral_invariance_validated':False,'prospective_gates':GATES,'points':points,
            'data_pass':data,'data_invalid_reasons':reasons,'tracking_pass':tracking,'stability_pass':stability,
            'truth_rmse':rmse,'truth_max_abs_error':max_error,'truth_monotonic':monotonic,
            'unsupported_event_count':unsupported,'passed':bool(data and tracking and stability),
            'dynamic_nav2_authorized':False,'no_refit':True}


def make_node(recorder):
    """Runtime imports only here. Ground-truth values never select commands."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu,JointState
    from geometry_msgs.msg import Twist
    from tf2_ros import Buffer,TransformListener,TransformException
    class NodeImpl(Node):
        def __init__(self):
            super().__init__('m2_open_space_runner')
            self.last={};self.frames={};self.bad_feedback=False;self.diagnostic=None;self.error=None;self.final_values=None
            self.tf=Buffer();self.listener=TransformListener(self.tf,self)
            self.create_subscription(String,m2.DIAGNOSTIC_TOPIC,self.diagnose,10)
            self.create_subscription(Twist,base.FINAL_TOPIC,self.final,10)
            for topic in (base.SLAM_TOPIC,base.TRUTH_TOPIC):
                self.create_subscription(Odometry,topic,lambda msg,t=topic:self.odom(t,msg),10)
            self.create_subscription(Imu,'/sim/raw/imu',self.imu,qos_profile_sensor_data)
            self.create_subscription(JointState,'/sim/raw/joint_states',self.joints,qos_profile_sensor_data)
        def final(self,msg):
            self.final_values=(msg.linear.x,msg.linear.y,msg.angular.z)
        def odom(self,topic,msg):
            self.frames[topic]=msg.child_frame_id
            # Truth only proves the required instrumentation/frame is present.
            if topic==base.SLAM_TOPIC:
                q=msg.pose.pose.orientation;p=msg.pose.pose.position
                self.last['slam_valid']=all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) and q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w>.5
        def imu(self,msg):
            q=msg.orientation;v=msg.angular_velocity;a=msg.linear_acceleration
            vals=[q.x,q.y,q.z,q.w,v.x,v.y,v.z,a.x,a.y,a.z]
            self.last['imu_valid']=all(map(math.isfinite,vals)) and q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w>.5
            if self.last['imu_valid']:
                roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
                pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
                if max(abs(roll),abs(pitch))>base.TILT_LIMIT_RAD:self.error='private IMU tilt limit'
        def joints(self,msg):
            ix={n:i for i,n in enumerate(msg.name)}
            self.last['joints_valid']=len(ix)==len(msg.name) and all(n in ix and ix[n]<len(msg.position) and ix[n]<len(msg.velocity) and math.isfinite(msg.position[ix[n]]) and math.isfinite(msg.velocity[ix[n]]) for n in base.JOINT_NAMES)
        def diagnose(self,msg):
            d=json.loads(msg.data);self.diagnostic=d
            recorder._write(m2.DIAGNOSTIC_TOPIC,'m2_diagnostic',d)
            if d.get('status')!='accepted' or d.get('candidate')!=m2.identity():self.error='unsupported-domain or wrong candidate diagnostic'
        def checks(self):
            now=time.monotonic();sim=recorder.sim_s
            required=[base.SHARED_TOPIC,base.FINAL_TOPIC,base.SLAM_TOPIC,base.TRUTH_TOPIC,
                      '/clock','/sim/raw/imu','/sim/raw/joint_states','/network_input_debug',m2.DIAGNOSTIC_TOPIC]
            fresh=all(t in recorder.last_received and now-recorder.last_received[t]['wall_s']<.5 for t in required)
            source_fresh=sim is not None and all(t in recorder.last_received and recorder.last_received[t]['source_s'] is not None and 0<=sim-recorder.last_received[t]['source_s']<=.5 for t in [base.SLAM_TOPIC,base.TRUTH_TOPIC,'/sim/raw/imu','/sim/raw/joint_states'])
            try:self.tf.lookup_transform('map','base_link',rclpy.time.Time());tf_ok=True
            except TransformException:tf_ok=False
            expected={base.SHARED_TOPIC:'velocity_step_recorder',base.FINAL_TOPIC:'velocity_adapter',m2.DIAGNOSTIC_TOPIC:'velocity_adapter'}
            expected.update({f'/model/b2w/joint/{joint}/cmd_force':'b2w_rl_sar_controller' for joint in base.JOINT_NAMES})
            publishers=all([(i.node_name,i.node_namespace) for i in self.get_publishers_info_by_topic(t)]==[(name,'/')] for t,name in expected.items())
            names=self.get_node_names()
            return {'fresh_streams':fresh,'source_fresh':source_fresh,'valid_slam':self.last.get('slam_valid',False),
                    'valid_imu':self.last.get('imu_valid',False),'valid_joints':self.last.get('joints_valid',False),
                    'body_frames':all(self.frames.get(t)=='base_link' for t in [base.SLAM_TOPIC,base.TRUTH_TOPIC]),
                    'tf_available':tf_ok,'unique_publishers':publishers,
                    'unique_clock':len(self.get_publishers_info_by_topic('/clock'))==1,
                    'controller':names.count('b2w_rl_sar_controller')==1,
                    'no_navigation_or_smoother':not set(names).intersection({'velocity_smoother','controller_server','planner_server','behavior_server','bt_navigator','goal_manager'}),
                    'candidate_active':bool(self.diagnostic and self.diagnostic.get('candidate')==m2.identity()),
                    'epoch_stable':recorder.epoch==0,'no_runtime_error':self.error is None}
    return NodeImpl()


def teardown(executor,node,recorder,runtime,launches,handlers):
    """Always attempt every owned teardown stage, even after earlier failures."""
    errors=[];health=None;cleanup={'passed':False,'reason':'cleanup not reached'}
    def attempt(label,fn):
        try:return fn()
        except BaseException as exc:errors.append(f'{label}: {type(exc).__name__}: {exc}')
    try:
        if executor is not None:
            attempt('executor shutdown',executor.shutdown)
            for n in (node,recorder):
                if n is not None:attempt('remove node',lambda n=n:executor.remove_node(n))
        if node is not None:attempt('destroy harness node',node.destroy_node)
        if recorder is not None:
            attempt('destroy recorder node',recorder.destroy_node)
            health=attempt('close recorder',recorder.close)
        if runtime is not None:
            attempt('runtime shutdown',lambda:runtime.shutdown() if runtime.ok() else None)
    finally:
        result=attempt('owned process cleanup',launches.cleanup)
        if result is not None:cleanup=result
        for sig,handler in handlers.items():attempt('restore signal',lambda sig=sig,handler=handler:signal.signal(sig,handler))
    return {'errors':errors,'cleanup':cleanup,'recorder_health':health}


def execute(args):
    if not 1<=args.ros_domain_id<=232:raise ValueError('ROS domain must be 1..232')
    ros2=shutil.which('ros2')
    if not ros2 or not base.POLICY.is_file():raise RuntimeError('source ROS/install setup and provide unchanged policy first')
    directory=Path(args.output_root).resolve()/dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    directory.mkdir(parents=True,exist_ok=False)
    fixture=base.generate_fixture_world(base.SOURCE_WORLD,directory/'velocity_step_fixture.sdf')
    token=secrets.token_hex(16)
    manifest=base.build_manifest(directory,args.ros_domain_id,token,not args.no_gui,fixture,SPEEDS)
    manifest.update({'mode':'m2_adapter_open_space','candidate':m2.identity(),'candidate_source_hashes':source_hashes(),
                     'prospective_plan':plan(),'experiment_start_sim_s':None,'termination':{'status':'preparing'},
                     'ign_partition':f'b2w-m2-{token}','tested_chain':plan()['chain'],
                     'smoother_rate_hz':None,'rate_basis':'20Hz wall request cadence derived from unchanged baseline; direct shared input, no smoother process'})
    env=dict(os.environ);env.update({'ROS_DOMAIN_ID':str(args.ros_domain_id),'ROS_LOCALHOST_ONLY':'1',
        'IGN_PARTITION':f'b2w-m2-{token}','FASTRTPS_DEFAULT_PROFILES_FILE':str(ROOT/'course_sim/config/fastdds_local.xml'),
        'PYTHONPATH':str(ROOT/'course_navigation')+os.pathsep+str(ROOT/'.venv/lib/python3.10/site-packages')+os.pathsep+env.get('PYTHONPATH',''),
        'LD_LIBRARY_PATH':str(ROOT/'.deps/lib')+os.pathsep+str(ROOT/'b2w_sim/b2w_controllers/third_party/onnxruntime/lib64')+os.pathsep+env.get('LD_LIBRARY_PATH','')})
    for key in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','FASTRTPS_DEFAULT_PROFILES_FILE','PYTHONPATH','LD_LIBRARY_PATH'):os.environ[key]=env[key]
    os.environ.pop('B2W_NARROW_OWNER',None)
    launches=base._OwnedLaunches(directory,token,env);recorder=node=executor=None;handlers={};runtime=None
    try:
        base._write_json(directory/'manifest.json',manifest)
        launches.start('course_stack',[ros2,'launch','course_bringup','course.launch.py','mode:=slam','control_source:=none',
            'locomotion_backend:=rl_sar','fault_profile:=normal',f"world:={fixture['generated_world']}",'scene:=confined',
            f"gui:={'false' if args.no_gui else 'true'}",'rviz:=false','spawn_x:=-4.0','spawn_y:=0.0','spawn_z:=1.0'])
        launches.start('velocity_adapter',[sys.executable,'-c','from course_navigation.velocity_adapter import main; main()',
            '--ros-args','-p','use_sim_time:=true','-p','m2_inverse_enabled:=true'])
        import rclpy
        from geometry_msgs.msg import Twist
        from scripts.velocity_step_recorder import create_recorder
        runtime=rclpy;handlers=base._initialize_ros()
        recorder=create_recorder(directory)
        recorder.destroy_publisher(recorder.publisher)
        recorder.publisher=recorder.create_publisher(Twist,base.SHARED_TOPIC,10)
        node=make_node(recorder);executor=base._make_executor(node,recorder)
        deadline=time.monotonic()+base.READINESS_TIMEOUT_S;last_send=last_check=0.;ready_since=None;start=None
        previous_sim=None;advances=0;run_wall=None;last_clock_change=time.monotonic()
        while True:
            executor.spin_once(timeout_sec=.01);wall=time.monotonic();sim=recorder.sim_s
            if sim is not None and sim!=previous_sim:
                advances+=1;previous_sim=sim;last_clock_change=wall
            if start is not None and wall-last_clock_change>base.CLOCK_FREEZE_TIMEOUT_S:raise RuntimeError('clock frozen')
            if any(p.poll() is not None for _,p,_ in launches.processes):raise RuntimeError('owned child exited')
            if node.error:raise RuntimeError(node.error)
            if recorder.epoch:raise RuntimeError('clock epoch crossed')
            if wall-last_check>=.2:
                last_check=wall;checks=node.checks()
                if start is None:
                    ready=start_ready(checks,node.final_values,2.2) and advances>=2
                    ready_since=(ready_since or wall) if ready else None
                    if ready_since is not None and start_ready(checks,node.final_values,wall-ready_since):
                        start=sim;run_wall=wall
                        manifest['experiment_start_sim_s']=start;manifest['readiness']={'completed':True,'checks':checks}
                    if wall>deadline:raise RuntimeError('readiness timeout')
                elif not all(checks.values()):raise RuntimeError('runtime checks lost: '+json.dumps(checks))
            if start is not None and wall-run_wall>base.RUN_WALL_TIMEOUT_S:raise RuntimeError('bounded run watchdog')
            if wall-last_send>=.05 and sim is not None:
                last_send=wall;elapsed=sim-start if start is not None else -1.
                vx,_,_=requested_at(manifest['sequence'],elapsed)
                recorder.record_request(vx,{},elapsed)
                if start is not None and elapsed>=48.:
                    manifest['termination']={'status':'completed'};break
    except (Exception,KeyboardInterrupt) as exc:
        manifest['termination']={'status':'aborted','reason':f'{type(exc).__name__}: {exc}'}
    finally:
        if recorder is not None and executor is not None and runtime and runtime.ok():
            try:
                until=time.monotonic()+base.FLUSH_S
                while time.monotonic()<until:
                    recorder.record_request(0.,{},0.);executor.spin_once(timeout_sec=.05)
            except BaseException as exc:manifest['zero_flush_error']=str(exc);manifest['termination']['status']='zero_flush_failed'
        teardown_result=teardown(executor,node,recorder,runtime,launches,handlers)
        manifest.update({'cleanup':teardown_result['cleanup'],'recorder_health':teardown_result['recorder_health'],
                         'teardown_errors':teardown_result['errors'],'processes':launches.records})
        if teardown_result['errors']:manifest['termination']['status']='teardown_failed'
        raw=directory/'events.jsonl';manifest['events_sha256']=base.sha256(raw) if raw.exists() else None
        base._write_json(directory/'manifest.json',manifest)
    events=base._read_events(directory/'events.jsonl') if (directory/'events.jsonl').exists() else []
    result=report(events,manifest);base._write_json(directory/'m2-report.json',result)
    print(json.dumps({'run_directory':str(directory),'passed':result['passed']},indent=2))
    return 0 if result['passed'] else 1


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);mode=p.add_mutually_exclusive_group()
    mode.add_argument('--execute',action='store_true');mode.add_argument('--analyze',metavar='RUN_DIR')
    p.add_argument('--output-root',default='/tmp/b2w-user-m2-open-space-01');p.add_argument('--ros-domain-id',type=int,default=195)
    p.add_argument('--no-gui',action='store_true');a=p.parse_args(argv)
    if a.analyze:
        d=Path(a.analyze).resolve();raw=d/'events.jsonl';manifest=json.loads((d/'manifest.json').read_text())
        if base.sha256(raw)!=manifest.get('events_sha256'):raise ValueError('capture hash mismatch')
        result=report(base._read_events(raw),manifest);base._write_json(d/'m2-report.json',result)
        print(json.dumps(result,indent=2));return 0 if result['passed'] else 1
    if not a.execute:print(json.dumps(plan(),indent=2));return 0
    return execute(a)


if __name__=='__main__':raise SystemExit(main())
