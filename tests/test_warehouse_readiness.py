"""Offline callback-order and complete first-fault contract."""
import json
from collections import deque
from types import SimpleNamespace as NS
import pytest
from test_warehouse_truth_admission import owner, message
import warehouse_ab_trial as trial


@pytest.mark.parametrize('topic', ['/imu','/pointcloud','/slam/map_points','odom'])
def test_future_callback_keeps_original_fresh_sample(owner,topic):
    m,wall=owner;m.receive_truth(message(18.))
    receive=lambda stamp: m.receive_odom(message(stamp)) if topic=='odom' else m.receive_sensor(topic,message(stamp))
    receive(18.); wall[0]=100.01;receive(18.01)
    assert m.ready()
    row=m.last_readiness['predicates'][topic]
    assert row['stamp']==18. and row['latest_stamp']==18.01
    m.receive_clock(NS(clock=NS(sec=18,nanosec=10000000)))
    assert m.ready() and m.last_readiness['predicates'][topic]['stamp']==18.01
    wall[0]=100.51
    assert not m.ready()  # receipt of future data cannot renew original sample


def test_snapshot_evaluates_all_predicates_once_and_retains_first_fault(owner):
    m,wall=owner;m.phase='precheck';m.receive_truth(message(18.))
    m.latest['odom']=99.;m.latest['/imu']=(98.,16.)
    m.pose=lambda **kw: (_ for _ in ()).throw(LookupError('missing chain'))
    with pytest.raises(RuntimeError):m.update()
    p=m.readiness_first_fault['predicates']
    assert set(p)=={'clock','truth','contacts','support','odom','/imu','/pointcloud','/slam/map_points','tf','final','support_feet','failure'}
    assert not p['odom']['passed'] and not p['/imu']['passed'] and not p['tf']['passed']
    assert p['tf']['exception']=="LookupError('missing chain')"
    saved=m.heartbeat.with_name('readiness-first-fault.json').read_bytes()
    wall[0]+=1
    with pytest.raises(RuntimeError):m.update()
    assert m.heartbeat.with_name('readiness-first-fault.json').read_bytes()==saved
    assert json.loads(saved)==m.readiness_first_fault


def test_tf_exception_fails_closed_but_bootstrap_can_recover(owner):
    m,wall=owner;m.receive_truth(message(18.))
    m.pose=lambda **kw: (_ for _ in ()).throw(LookupError('not arrived'))
    assert not m.ready() and not m.resting()
    assert m.last_readiness['predicates']['tf']['exception']
    assert m.readiness_first_fault is None
    m.pose=lambda **kw:(0.,0.,0.)
    assert m.ready() and m.resting()


@pytest.mark.parametrize('topic,limit', [('/imu',1.),('/pointcloud',1.),('/slam/map_points',20.),('odom',.5)])
def test_missing_and_wall_stale_stream_fail_closed(owner,topic,limit):
    m,wall=owner;m.receive_truth(message(18.))
    if topic=='odom':m.odom=None
    else:del m.latest[topic]
    assert not m.ready() and not m.last_readiness['predicates'][topic]['passed']
    if topic=='odom':m.receive_odom(message(18.))
    else:m.receive_sensor(topic,message(18.))
    assert m.ready()
    wall[0]=100.+limit+.000001
    assert not m.ready() and not m.last_readiness['predicates'][topic]['passed']


def test_clock_freeze_and_negative_receipt_age_rejected(owner):
    m,wall=owner;m.receive_truth(message(18.));m.clock_wall=98.999999
    assert not m.ready() and not m.last_readiness['predicates']['clock']['passed']
    m.clock_wall=100.;m.latest['final']=100.01
    assert not m.ready() and not m.last_readiness['predicates']['final']['passed']


def test_rest_uses_one_decision_time_at_boundary(owner,monkeypatch):
    m,wall=owner;m.receive_truth(message(18.));wall[0]=100.5
    calls=[]
    def now():
        calls.append(1);return 100.5 if len(calls)==1 else 100.500001
    monkeypatch.setattr(trial.time,'monotonic',now)
    assert m.resting() and len(calls)==1


def transform(stamp):
    return NS(header=message(stamp).header,transform=NS(
        translation=NS(x=0.,y=0.,z=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))


def test_actual_tf_retains_stamped_transform_but_never_caches_lookup_exception(owner):
    m,wall=owner;m.receive_truth(message(18.));del m.pose
    tf=[transform(18.)];m.tf=NS(lookup_transform=lambda *args:tf[0])
    assert m.ready()
    tf[0]=transform(18.01)
    assert m.ready() and m.last_readiness['predicates']['tf']['stamp']==18.
    m.clock=18.01
    assert m.ready() and m.last_readiness['predicates']['tf']['stamp']==18.01
    m.tf.lookup_transform=lambda *args: (_ for _ in ()).throw(LookupError('unavailable'))
    assert not m.ready()  # A cached transform cannot hide a live lookup exception.
    m.tf.lookup_transform=lambda *args:tf[0]
    assert m.ready()
    m.clock=18.510001
    assert not m.ready() and not m.last_readiness['predicates']['tf']['passed']


def test_support_history_keeps_consistent_stamp_but_expiry_rejects(owner):
    m,wall=owner;m.receive_truth(message(18.))
    m.remember('support',18.,100.);m.support=18.01;m.remember('support',18.01,100.01)
    assert m.ready() and m.last_readiness['predicates']['support']['stamp']==18.
    m.clock=18.510001
    assert not m.ready() and not m.last_readiness['predicates']['support']['passed']


def test_contact_pairs_ahead_of_clock_do_not_erase_fresh_pair_or_veto(owner):
    from course_navigation.stage4_contact_stream import ContactStream
    from course_navigation.stage4_contact_evidence import GROUND,SUPPORTS
    m,wall=owner;m.receive_truth(message(18.));names=sorted(SUPPORTS)
    m.contacts=ContactStream(names,first_sequence=1)
    def pair(seq,start,end,receipt,forbidden=False):
        inv=dict(sequence=seq,interval_start=start,stamp=end,sampled_steps=1,
                 collisions=names,ever_forbidden_contact=forbidden,evaluation_only=True)
        m.contacts.intervals.contacts(seq,[('course_world::b2w::'+names[0],GROUND)],stamp=end,wall=receipt)
        m.contacts.inventory(inv,wall=receipt);m.remember_contacts()
    pair(1,17.9,18.,100.)
    assert m.ready()
    pair(2,18.,18.01,100.01);wall[0]=100.01
    assert m.ready() and m.last_readiness['predicates']['contacts']['stamp']==18.
    m.clock=18.01
    assert m.ready() and m.last_readiness['predicates']['contacts']['stamp']==18.01
    # Neither duplicate reports nor a valid older pair may hide a live safety veto.
    pair(3,18.01,18.02,100.02,forbidden=True)
    assert not m.ready() and not m.last_readiness['predicates']['contacts']['passed']


@pytest.mark.parametrize('topic,limit', [('/imu',1.),('/pointcloud',1.),('/slam/map_points',6.),('odom',.5)])
def test_future_only_and_sim_stale_fail_then_fresh_callback_recovers(owner,topic,limit):
    m,wall=owner;m.receive_truth(message(18.))
    receive=lambda stamp: m.receive_odom(message(stamp)) if topic=='odom' else m.receive_sensor(topic,message(stamp))
    receive(18.01)
    assert not m.ready()
    receive(18.-limit-.000001)
    assert not m.ready()  # Regression is itself a latched fault.
    m.failure=None  # New bootstrap fixture state for freshness-only assertion.
    assert not m.ready() and not m.last_readiness['predicates'][topic]['passed']
    receive(18.)
    assert m.ready()


def test_snapshot_serializes_nonfinite_final_and_missing_clock(owner):
    m,wall=owner;m.receive_truth(message(18.));m.clock=None;m.clock_wall=None
    m.final[0]=float('nan');m.phase='precheck'
    with pytest.raises(RuntimeError):m.update()
    saved=json.loads(m.heartbeat.with_name('readiness-first-fault.json').read_text())
    assert saved['predicates']['final']['value'][0]=='nan'
    assert saved['predicates']['clock']['passed'] is False


def test_contact_pair_receipts_do_not_renew_on_duplicate_and_wall_expire(owner):
    m,wall=owner;m.receive_truth(message(18.))
    m.contacts.failure=None
    m.remember('contacts',18.,100.,{'inventory':(18.,100.),'contacts':(18.,100.)})
    m.remember('contacts',18.,100.4,{'inventory':(18.,100.4),'contacts':(18.,100.4)})
    wall[0]=100.5
    assert m.ready()
    wall[0]=100.500001
    assert not m.ready() and not m.last_readiness['predicates']['contacts']['passed']


def test_active_lookup_loss_is_latched_despite_later_recovery(owner):
    m,wall=owner;m.phase='precheck';m.receive_truth(message(18.))
    m.pose=lambda **kw: (_ for _ in ()).throw(LookupError('lost'))
    with pytest.raises(RuntimeError):m.update()
    m.pose=lambda **kw:(0.,0.,0.)
    assert not m.ready() and m.last_readiness['predicates']['tf']['passed']
    assert not m.last_readiness['predicates']['failure']['passed']


def test_real_step_sensor_interleaving_and_expiry(owner):
    import ast
    from pathlib import Path
    m,wall=owner;m.receive_truth(message(18.));m.receive_sensor('/imu',message(18.));m.phase='precheck'
    tree=ast.parse(Path(trial.__file__).read_text())
    execute=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute')
    step=next(n for n in execute.body if isinstance(n,ast.FunctionDef) and n.name=='step')
    dispatch=NS(spin_once=lambda *a,**kw:m.receive_sensor('/imu',message(18.01)))
    namespace=dict(time=trial.time,watchdog=NS(poll=lambda:None),rclpy=dispatch,node=None,
                   monitor=m,owner=NS(observe=lambda:None))
    exec(compile(ast.Module(body=[step],type_ignores=[]),'<actual step>','exec'),namespace)
    namespace['step'](101.)
    assert m.failure is None
    wall[0]=101.000001
    with pytest.raises(RuntimeError):namespace['step'](102.)
    assert m.readiness_first_fault is not None


@pytest.mark.parametrize('contact_stamp,clock,expected', [
    (18.,18.5,True),
    (18.-5e-9,18.5,False),
    # Existing normalization: raw inventory age exceeds .5 by 2.5ns,
    # but both gate timestamps are the contact stamp, just inside .5.
    (18.+5e-9,18.5+2.5e-9,True),
])
def test_real_paired_stamp_normalization_preserves_original_gate_boundary(owner,contact_stamp,clock,expected):
    from course_navigation.stage4_contact_stream import ContactStream
    from course_navigation.stage4_contact_evidence import GROUND,SUPPORTS
    m,wall=owner;names=sorted(SUPPORTS)
    m.contacts=ContactStream(names,first_sequence=1)
    inventory=dict(sequence=1,interval_start=17.9,stamp=18.,sampled_steps=1,
                   collisions=names,ever_forbidden_contact=False,evaluation_only=True)
    m.contacts.intervals.contacts(1,[('course_world::b2w::'+names[0],GROUND)],
                                  stamp=contact_stamp,wall=100.)
    m.contacts.inventory(inventory,wall=100.)
    gate=m.contacts.intervals.gate
    assert m.contacts.failure is None and gate.covered==gate.expected
    assert gate.latest['inventory'][0]==gate.latest['contacts'][0]==contact_stamp
    m.remember_contacts();m.clock=clock
    original=m.contacts.ready(sim=clock,wall=100.)
    status,_=m.stream_status('contacts',100.,.5,.5)
    assert original is expected and status['passed'] is expected
