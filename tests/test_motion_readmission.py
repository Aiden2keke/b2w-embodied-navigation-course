"""Actual execute/step/Monitor callbacks, with external process/ROS edges inert.

Recorded domain230 stamps seed a clock-ahead handoff. The continuation is an
explicit deterministic callback interleaving, not a reconstructed DDS queue.
All publications below go only to the existing fixture's in-memory list.
"""
import ast
from collections import deque
import copy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from test_warehouse_precheck_only import session
import warehouse_ab_trial as trial

REAL_MONITOR = trial.Monitor


@pytest.fixture
def handoff(session, monkeypatch):
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from ros_gz_interfaces.msg import Contacts
    from rosidl_runtime_py.set_message import set_message_fields
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer
    from course_navigation.stage4_contact_stream import ContactStream
    from owner_tf_delivery import TFDelivery, observed_listener_type

    state = session
    state.mode = 'delayed'
    state.pulse_admissions = []
    state.callback_trace = []
    state.source_sequence = 0
    state.interval_end = 0.
    state.audit_wall = None
    state.queue = deque()
    run = trial.ROOT/'artifacts/trials/owner-dispatch-precheck-20261025/precheck-d1d1cdad7ae44d71b0194ddb1ad03efd'
    fault = json.loads((run/'readiness-first-fault.json').read_text())
    pocket = json.loads((run/'pocket-admission-initial.json').read_text())
    records = [json.loads(line) for line in (run/'monitor-events.jsonl').read_text().splitlines()]
    observer=next((run/'observer').iterdir())
    recorded_prefix=[]
    for line in (observer/'events.jsonl').open():
        e=json.loads(line)
        if not pocket['wall']-.1 <= e['wall_monotonic_s'] <= fault['wall']+.1:continue
        if e['topic']=='/clock' and e['value']['clock_s']>fault['clock']:
            recorded_prefix.append(('clock',e['value']['clock_s']))
        elif e['topic']=='/slam/odom' and e['stamp_s']>17.2:
            recorded_prefix.append(('odom',e['stamp_s']))
        elif e['topic']=='/tf':
            for t in e['value']['transforms']:
                if t['header']['frame_id']=='odom' and t['child_frame_id']=='base_link':
                    st=t['header']['stamp'];value=st['sec']+st['nanosec']*1e-9
                    if value>17.2:recorded_prefix.append(('tf',value))
    raw_contact = next(e['message'] for e in records if e['kind']=='full_contacts' and e['message']['contacts'])
    raw_inventory = json.loads(next(e['message']['data'] for e in records if e['kind']=='collision_inventory'
                                  and len(json.loads(e['message']['data'])['collisions'])==26))

    def stamp(value):
        return Time(sec=int(value), nanosec=round((value-int(value))*1e9))

    def odom(value, height=.65):
        msg=Odometry(); msg.header.stamp=stamp(value)
        msg.pose.pose.position.z=height; msg.pose.pose.orientation.w=1.
        return msg

    def tf(value):
        transforms=[]
        for parent,child in [('map','odom'),('odom','base_link')]:
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
            t.header.stamp=stamp(value);t.transform.rotation.w=1.;transforms.append(t)
        return TFMessage(transforms=transforms)

    class Monitor(REAL_MONITOR):
        def __init__(self,node,inventory,heartbeat):
            self.node=node;self.heartbeat=heartbeat;self.clock=10.;self.clock_wall=state.wall
            self.failure=None;self.phase='bootstrap';self.latest={};self.stamp_history={}
            self.truth_history=deque(maxlen=128);self.truth=None;self.odom=None
            self.truth_first_fault=self.readiness_first_fault=self.last_readiness=None
            self.tf_diagnostic=None;self.final=None;self.final_samples=self.nonzero_final_samples=0
            self.contact_armed=True;self.pending_contacts={};self.samples=0;self.support=None
            self.support_feet=set();self.contacts=ContactStream(inventory['collisions'],first_sequence=1)
            self.events=(state.output/'monitor-events.jsonl').open('w',buffering=1)
            self.grid=dict(width=40,height=40,resolution=.1,origin=(-2.,-2.),data=[0]*1600,
                           observed_free_present=True)
            self.tf_delivery=TFDelivery();self.tf=Buffer()
            listener_type=observed_listener_type()
            self.listener=listener_type.__new__(listener_type)
            self.listener.buffer=self.tf;self.listener.delivery=self.tf_delivery
            self.listener.node=NS(destroy_subscription=lambda _: None)
            self.listener.tf_static_sub=self.listener.tf_sub=None
            state.monitor=self
            refresh(self,10.)

    def contact(m,value):
        state.source_sequence+=1
        msg=Contacts();set_message_fields(msg,copy.deepcopy(raw_contact))
        msg.header.frame_id='b2w_evaluation_seq_'+str(state.source_sequence)
        msg.header.stamp=stamp(value)
        inv=dict(raw_inventory,sequence=state.source_sequence,stamp=value,
                 interval_start=state.interval_end,sampled_steps=1,last_ground_support_stamp=value,
                 ever_forbidden_contact=state.mode=='forbidden_contact' and state.audit_wall is not None)
        state.interval_end=value
        m.receive_contacts(msg);m.receive_inventory(String(data=json.dumps(inv)))

    def refresh(m,value,localization=None):
        loc=value if localization is None else localization
        m.receive_clock(NS(clock=stamp(value)))
        m.receive_truth(odom(value,2. if state.mode=='unsafe_body' and state.audit_wall is not None else .65))
        for topic in ('/imu','/pointcloud','/slam/map_points'):
            m.receive_sensor(topic,NS(header=NS(stamp=stamp(value))))
        if not ((state.mode=='stale_odom' and state.audit_wall is not None)
                or (state.mode=='post_entry_odom_stall' and m.phase=='precheck')):m.receive_odom(odom(loc))
        if not ((state.mode=='stale_tf' and state.audit_wall is not None)
                or (state.mode=='post_entry_tf_stall' and m.phase=='precheck')):m.listener.callback(tf(loc))
        contact(m,value);m.receive_final(Twist())
        m.select_truth(state.wall)

    monkeypatch.setattr(trial,'Monitor',Monitor)
    original_native=trial.native_snapshot
    def native(*args,**kwargs):
        result=original_native(*args,**kwargs)
        m=state.monitor
        # Exact recorded handoff clock/localization stamps, with fresh receipts.
        refresh(m,pocket['clock'],localization=fault['predicates']['odom']['stamp'])
        state.audit_wall=state.wall
        # First dispatch mirrors the live fault: clock jumps ahead before loc.
        state.queue.append(('clock',fault['clock']))
        state.queue.extend(recorded_prefix)
        last_clock=max([fault['clock']]+[t for kind,t in recorded_prefix if kind=='clock'])
        # High-rate clock/IMU interleavings with independently delayed loc.
        for i in range(1,61):
            t=last_clock+i*.0025
            state.queue.append(('clock',t))
            if i%2==0:state.queue.append(('imu',t))
            if i%20==0:state.queue.extend([('odom',t-.1),('tf',t-.1)])
        return result
    monkeypatch.setattr(trial,'native_snapshot',native)

    def spin(node,timeout_sec):
        state.wall+=.0025 if state.audit_wall is not None and state.queue and state.mode!='frozen' else .1
        if not hasattr(state,'monitor') or state.monitor is None:return
        if getattr(state,'next_callback_internal_clock',False):
            state.next_callback_internal_clock=False
            return  # TimeSource callback does not update Monitor's samples.
        m=state.monitor
        if state.audit_wall is not None and (m.phase=='bootstrap' or state.queue):
            if m.phase=='bootstrap':state.post_audit_bootstrap_spins+=1
            if state.mode=='frozen':return
            if state.queue:
                kind,t=state.queue.popleft();state.callback_trace.append((kind,t))
                if kind=='clock':
                    m.receive_clock(NS(clock=stamp(t)))
                    # Other safety streams remain fresh while localization lags.
                    m.receive_truth(odom(t));contact(m,t);m.receive_final(Twist())
                elif kind=='imu':m.receive_sensor('/imu',NS(header=NS(stamp=stamp(t))))
                elif kind=='odom' and state.mode not in ('stale_odom','backlog'):m.receive_odom(odom(t))
                elif kind=='tf' and state.mode not in ('stale_tf','backlog'):m.listener.callback(tf(t))
                return
        value=m.clock+.1
        if state.mode=='backlog' and state.audit_wall is not None:
            # Clock keeps dispatching; newer localization is never serviced.
            m.receive_clock(NS(clock=stamp(value)));m.receive_truth(odom(value))
            for topic in ('/imu','/pointcloud','/slam/map_points'):
                m.receive_sensor(topic,NS(header=NS(stamp=stamp(value))))
            contact(m,value);m.receive_final(Twist())
        else:refresh(m,value)
    monkeypatch.setattr(rclpy,'spin_once',spin)

    original_node=rclpy.create_node
    def node(*args,**kwargs):
        n=original_node(*args,**kwargs);original_pub=n.create_publisher
        def publisher(*args,**kwargs):
            p=original_pub(*args,**kwargs);publish=p.publish
            def send(msg):
                if msg.linear.x or msg.angular.z:
                    m=state.monitor;s=m.readiness()
                    state.pulse_admissions.append(s)
                    if not s['passed'] or m.physical_veto():
                        raise AssertionError('Nonzero publication with invalid admission')
                    assert s['predicates']['odom']['receipt_wall']>=state.audit_wall
                    assert s['predicates']['tf']['stamp']>pocket['clock']
                publish(msg)
            p.publish=send;return p
        n.create_publisher=publisher;return n
    monkeypatch.setattr(rclpy,'create_node',node)
    state.refresh=lambda value:refresh(state.monitor,value)
    state.recorded_clock=pocket['clock']
    return state


def test_original_execute_handoff_reproduces_recorded_stale_fault(handoff):
    path=trial.ROOT/'artifacts/recovery/first-motion-validation-20261025/warehouse_ab_trial.executed.py'
    tree=ast.parse(path.read_text());function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute')
    # Only execute() is compiled; its globals use the same inert external edges.
    namespace=dict(trial.__dict__)
    exec(compile(ast.Module(body=[function],type_ignores=[]),'<recorded execute>','exec'),namespace)
    assert namespace['execute'](handoff.args,handoff.output)==1
    r=handoff.report();f=r['readiness_first_fault']
    assert r['precheck']==[] and f['clock']==17.7475
    assert f['predicates']['odom']['stamp']==f['predicates']['tf']['stamp']==17.2
    assert f['predicates']['odom']['simulation_age_s']>.5
    assert not handoff.pulse_admissions


def test_corrected_handoff_requires_delayed_localization_and_fresh_pulses(handoff):
    assert handoff.run()==0,handoff.report().get('error')
    r=handoff.report();assert r['post_audit_readmission']['end_sim']>handoff.recorded_clock+.5
    assert handoff.pulse_admissions and r['cleanup']['passed']
    for s in handoff.pulse_admissions:
        assert s['passed']
        assert 0<=s['predicates']['odom']['simulation_age_s']<=.5
        assert 0<=s['predicates']['tf']['simulation_age_s']<=.5
    assert all(e['callback_wall']>=handoff.audit_wall
               for e in handoff.monitor.tf_delivery.edges.values())
    assert handoff.monitor.contacts.failure is None


@pytest.mark.parametrize('mode',['stale_tf','stale_odom','frozen','backlog'])
def test_clock_progress_alone_never_admits_missing_localization(handoff,mode):
    handoff.mode=mode
    assert handoff.run()==1
    r=handoff.report();assert r['precheck']==[] and r['cleanup']['passed']
    assert not handoff.pulse_admissions
    assert not any(any(v) for _,v in handoff.commands)
    assert r['error']=="TimeoutError('Bounded phase wall deadline')"


def test_physical_veto_precedes_entry_into_precheck(handoff):
    handoff.mode='unsafe_body'
    assert handoff.run()==1
    r=handoff.report();assert not handoff.pulse_admissions
    assert r['readiness_first_fault']['phase']=='bootstrap'


@pytest.mark.parametrize('mode',['post_entry_tf_stall','post_entry_odom_stall'])
def test_readmission_is_not_a_cached_pass_for_later_staleness(handoff,mode):
    handoff.mode=mode
    assert handoff.run()==1
    r=handoff.report()
    assert 'post_audit_readmission' in r and not handoff.pulse_admissions
    assert r['readiness_first_fault']['phase']=='precheck'
    key='tf' if mode=='post_entry_tf_stall' else 'odom'
    assert not r['readiness_first_fault']['predicates'][key]['passed']
    failed=r['readiness_first_fault']['predicates'][key]
    assert failed['simulation_age_s']>.5 or (key=='odom' and failed['wall_age_s']>.5)


def test_contact_failure_is_not_hidden_by_readmission_wait(handoff):
    handoff.mode='forbidden_contact'
    assert handoff.run()==1
    r=handoff.report()
    assert not handoff.pulse_admissions and r['precheck']==[]
    assert r['contact_failure']=='forbidden_robot_environment_contact'


def test_pulse_report_io_cannot_publish_before_freshness_check(handoff,monkeypatch):
    original=trial.write_json
    def write(path,value):
        original(path,value)
        if path.name=='session-report.json' and value.get('precheck') and not handoff.pulse_admissions:
            # An actual blocking output can expire wall freshness without clock callbacks.
            handoff.wall+=.6
            handoff.next_callback_internal_clock=True
    monkeypatch.setattr(trial,'write_json',write)
    assert handoff.run()==1
    assert not handoff.pulse_admissions
    assert not any(any(v) for _,v in handoff.commands)
    assert not handoff.report()['precheck'][0]['command_publication_attempted']
