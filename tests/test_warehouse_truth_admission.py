"""Deterministic owner callback interleavings; no ROS participant or simulation."""
import ast
from collections import deque
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import warehouse_ab_trial as trial


def message(stamp, *, height=.65, speed=0.):
    return NS(header=NS(stamp=NS(sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9))),
        pose=NS(pose=NS(position=NS(x=0., y=0., z=height),
                        orientation=NS(x=0., y=0., z=0., w=1.))),
        twist=NS(twist=NS(linear=NS(x=speed, y=0.), angular=NS(z=0.))))


@pytest.fixture
def owner(tmp_path, monkeypatch):
    wall=[100.]
    monkeypatch.setattr(trial.time, 'monotonic', lambda: wall[0])
    m=trial.Monitor.__new__(trial.Monitor)
    from owner_tf_delivery import TFDelivery
    m.tf_delivery=TFDelivery(); m.tf=None
    m.clock=18.; m.clock_wall=100.; m.truth=None
    m.truth_history=deque(maxlen=128); m.truth_first_fault=None
    m.stamp_history={};m.last_readiness=m.readiness_first_fault=None;m.tf_diagnostic=None
    m.heartbeat=tmp_path/'heartbeat.json'; m.phase='bootstrap'; m.failure=None
    m.contacts=NS(ready=lambda **kw: True); m.support=18.
    m.odom=(18.,message(18.)); m.latest={'odom':100., 'final':100., 'support':100.,
        '/pointcloud':(100.,18.), '/imu':(100.,18.), '/slam/map_points':(100.,18.)}
    m.support_feet={'FL','FR','RL','RR'}; m.pose=lambda **kw:(0.,0.,0.)
    m.final=[0.]*6; m.samples=1;m.grid=None
    return m,wall


@pytest.mark.parametrize('stamp,receipt,reason', [
    (None,None,'missing_truth'), (18.,99.49,'truth_wall_stale'),
    (18.01,100.,'truth_ahead_of_clock'), (17.49,100.,'truth_sim_stale'),
])
def test_admission_rejects_invalid_truth_and_veto_records_exact_first_fault(owner, stamp, receipt, reason):
    m,wall=owner
    if stamp is not None:
        wall[0]=receipt; m.receive_truth(message(stamp)); wall[0]=100.
    assert not m.ready()  # Old ready() incorrectly admits every case.
    assert not m.resting()
    assert m.physical_veto()=='evaluation_truth_stale'
    saved=json.loads(m.heartbeat.with_name('truth-first-fault.json').read_text())
    assert saved['failed_subpredicate']==reason
    assert saved['owner_clock']==18.
    assert saved['truth_stamp']==stamp
    assert saved['truth_receipt_wall']==receipt
    if stamp is not None:
        assert saved['wall_age_s']==pytest.approx(100.-receipt)
        assert saved['simulation_age_s']==pytest.approx(18.-stamp)
    wall[0]=101.;m.physical_veto()
    assert json.loads(m.heartbeat.with_name('truth-first-fault.json').read_text())==saved


def test_truth_before_clock_uses_fresh_consistent_snapshot_then_promotes(owner):
    m,wall=owner
    m.receive_truth(message(18.,speed=0.))
    wall[0]=100.01; m.receive_truth(message(18.01,speed=.03))
    assert m.ready() and m.physical_veto() is None
    assert m.truth[0]==18. and m.resting()
    m.receive_clock(NS(clock=NS(sec=18,nanosec=10000000)))
    assert m.ready() and m.physical_veto() is None
    assert m.truth[0]==18.01 and not m.resting()
    assert not m.heartbeat.with_name('truth-first-fault.json').exists()


def test_future_only_recovers_when_clock_callback_arrives(owner):
    m,wall=owner;m.receive_truth(message(18.01))
    assert not m.ready()
    m.receive_clock(NS(clock=NS(sec=18,nanosec=10000000)))
    assert m.ready() and m.resting() and m.physical_veto() is None


def test_stale_history_cannot_mask_future_only_and_clock_freeze(owner):
    m,wall=owner;m.receive_truth(message(18.))
    wall[0]=100.51;m.receive_truth(message(18.01))
    assert not m.ready()
    assert m.physical_veto()=='evaluation_truth_stale'
    # Repeated future truth cannot renew a frozen clock's eligible sample.
    wall[0]=101.01;m.receive_truth(message(18.02))
    assert not m.ready()
    from warehouse_ab_supervisor import health_fault
    assert health_fault(dict(wall=101.01,clock_advanced_wall=m.clock_wall),101.01,1.)=='frozen_clock'


def test_new_fresh_truth_recovers_admission_without_weakening_limits(owner):
    m,wall=owner
    m.receive_truth(message(17.49));assert not m.ready()
    m.receive_truth(message(18.));assert m.ready()
    # Exact original boundaries remain allowed; either boundary exceeded rejects.
    wall[0]=100.5;m.clock=18.5
    assert m.ready() and m.physical_veto() is None
    wall[0]=100.500001;assert not m.ready()


def test_posture_and_command_veto_still_apply_to_selected_truth(owner):
    m,wall=owner;m.receive_truth(message(18.,height=.1))
    assert m.physical_veto()=='body_orientation_or_height_limit'
    m.receive_truth(message(18.));m.final[1]=.001
    assert m.physical_veto()=='final_command_limit'


def test_real_step_interleaving_does_not_abort_on_newer_truth_before_clock(owner):
    m,wall=owner;m.receive_truth(message(18.));m.phase='precheck'
    # Execute the actual nested step function, replacing only ROS dispatch and process IO.
    tree=ast.parse(Path(trial.__file__).read_text())
    execute=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute')
    step=next(n for n in execute.body if isinstance(n,ast.FunctionDef) and n.name=='step')
    dispatch=NS(spin_once=lambda *a,**kw:m.receive_truth(message(18.01)))
    namespace=dict(time=trial.time,watchdog=NS(poll=lambda:None),rclpy=dispatch,node=None,
                   monitor=m,owner=NS(observe=lambda:None))
    exec(compile(ast.Module(body=[step],type_ignores=[]),'<actual execute.step>','exec'),namespace)
    namespace['step'](101.)
    assert m.failure is None and m.truth[0]==18.


def test_simulation_age_limit_is_independent_of_wall_reception(owner):
    m,wall=owner;m.receive_truth(message(17.5))
    assert m.ready()
    m.clock=18.000001
    assert not m.ready() and m.physical_veto()=='evaluation_truth_stale'
    assert m.truth_first_fault['failed_subpredicate']=='truth_sim_stale'


def test_readiness_reacquired_after_identity_io_before_phase_transition(owner):
    m,wall=owner;m.receive_truth(message(18.))
    assert m.ready()
    wall[0]=100.51  # Simulate native hash/file work crossing the original .5s limit.
    assert not m.ready()
    tree=ast.parse(Path(trial.__file__).read_text())
    execute=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute')
    body=next(n for n in execute.body if isinstance(n,ast.Try)).body
    arm=next(i for i,n in enumerate(body) if isinstance(n,ast.Assign)
             and any(isinstance(t,ast.Attribute) and t.attr=='phase' for t in n.targets)
             and any(isinstance(v,ast.Constant) and v.value=='precheck' for v in ast.walk(n.value)))
    def wait(predicate, duration):
        assert duration == 5. and m.phase=='bootstrap' and not predicate()
        m.latest['final']=wall[0]
        m.receive_odom(message(18.51))
        m.receive_truth(message(18.51))
        m.support=18.51; m.latest['support']=wall[0]
        assert not predicate()  # Future samples cannot admit the frozen clock.
        m.receive_clock(NS(clock=NS(sec=18,nanosec=510000000)))
        assert predicate()
    namespace=dict(monitor=m,wait=wait,args=NS(stationary_containment=False,precheck_only=True),
                   report={},time=NS(monotonic=lambda:wall[0]))
    readmit=next(i for i,n in enumerate(body[:arm]) if isinstance(n,ast.If)
                 and any(isinstance(child,ast.Name) and child.id=='admission_sim'
                         for child in ast.walk(n)))
    exec(compile(ast.Module(body=body[readmit:arm+1],type_ignores=[]),
                 '<actual precheck admission>','exec'),namespace)
    assert m.phase=='precheck' and m.physical_veto() is None


def test_qualified_route_caps_tighten_existing_physical_veto(owner):
    m,wall=owner;m.receive_truth(message(18.,speed=.11));m.route_motion_caps=(.1,.2)
    assert m.physical_veto()=='qualified_physical_motion_envelope_exceeded'
    m.route_motion_caps=(.2,.2)
    assert m.physical_veto() is None
