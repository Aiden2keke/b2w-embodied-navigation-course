"""Recorded interleaving replay and dedicated ingress, no simulator or motion."""
import json
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import pytest
from test_owner_tf_delivery import listener, transform
from test_warehouse_truth_admission import owner, message
import warehouse_ab_trial as trial


def test_recorded_clock_before_ready_tf_reproduces_original_fault(owner):
    m,wall=owner;obj=listener();m.tf=obj.buffer;m.tf_delivery=obj.delivery
    m.pose=trial.Monitor.pose.__get__(m)
    trace=json.loads((Path(__file__).parent/'fixtures/owner_tf_scheduling_20261025.json').read_text())
    m.clock=trace['clock']
    stamp=trace['owner_common_stamp']
    obj.callback(NS(transforms=[transform(stamp=stamp),transform('odom','base_link',stamp)]))
    with pytest.raises(ValueError,match='stale_map_base_tf'):m.pose()
    # Model the pending24.2 callback. Historical DDS arrival is unrecorded.
    stamp=trace['observer']['stamp']
    obj.callback(NS(transforms=[transform(stamp=stamp),transform('odom','base_link',stamp)]))
    assert m.pose()==(0.,0.,0.)
    assert m.tf_diagnostic['simulation_age_s']==pytest.approx(.3025)


def test_tf_message_is_atomic_for_concurrent_owner_lookup():
    obj=listener();entered=threading.Event();release=threading.Event();finished=threading.Event()
    original=obj.buffer.set_transform
    def insert(tr,authority):
        original(tr,authority)
        if tr.header.frame_id=='map':
            entered.set();assert release.wait(2.)
    obj.buffer.set_transform=insert
    m=trial.Monitor.__new__(trial.Monitor)
    m.tf=obj.buffer;m.tf_delivery=obj.delivery;m.clock=14.2;m.stamp_history={}
    writer=threading.Thread(target=lambda:obj.callback(NS(transforms=[transform(),transform('odom','base_link')])))
    writer.start();assert entered.wait(2.)
    poses=[];errors=[]
    def lookup():
        try:poses.append(m.pose())
        except Exception as exc:errors.append(exc)
        finally:finished.set()
    reader=threading.Thread(target=lookup);reader.start()
    try: assert not finished.wait(.05)
    finally:release.set();writer.join(2.);reader.join(2.)
    assert poses==[(0.,0.,0.)] and not errors
    assert not writer.is_alive() and not reader.is_alive()


def test_tf_ahead_of_owner_clock_uses_buffer_history_without_extrapolation(owner):
    m,wall=owner;obj=listener();m.tf=obj.buffer;m.tf_delivery=obj.delivery
    m.pose=trial.Monitor.pose.__get__(m)
    for stamp in (18.,18.1):obj.callback(NS(transforms=[transform(stamp=stamp),transform('odom','base_link',stamp)]))
    assert m.pose()==(0.,0.,0.)
    assert m.tf_diagnostic['stamp']==18.


def test_real_ingress_services_tf_odom_while_owner_does_not_spin(tmp_path):
    import rclpy
    from rclpy.context import Context
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import qos_profile_sensor_data
    from owner_tf_delivery import LocalizationIngress
    context=Context();rclpy.init(context=context,domain_id=182)
    ingress=None;publisher=None
    try:
        ingress=LocalizationIngress(context=context)
        publisher=rclpy.create_node('offline_ingress_source',context=context)
        tf_pub=publisher.create_publisher(TFMessage,'/tf',100)
        odom_pub=publisher.create_publisher(Odometry,'/slam/odom',qos_profile_sensor_data)
        import time
        end=time.monotonic()+5.
        while tf_pub.get_subscription_count()<1 or odom_pub.get_subscription_count()<1:
            assert time.monotonic()<end
            time.sleep(.01)
        # Original executor takes only clock, then readiness faults. Here no
        # owner executor service occurs at all during the published interleaving.
        tf_pub.publish(TFMessage(transforms=[transform(stamp=24.),transform('odom','base_link',24.)]))
        tf_pub.publish(TFMessage(transforms=[transform(stamp=24.2),transform('odom','base_link',24.2)]))
        odom=Odometry();odom.header.stamp.sec=24;odom.header.stamp.nanosec=200000000
        odom_pub.publish(odom)
        from rclpy.time import Time
        while True:
            with ingress.delivery.lock:
                rows=ingress.odometry.copy()
                try:stamp=ingress.buffer.lookup_transform('map','base_link',Time()).header.stamp
                except Exception:stamp=None
            if rows and stamp and stamp.nanosec==200000000:break
            assert time.monotonic()<end
            time.sleep(.005)
        assert rows[-1][0]==24.2 and ingress.failure is None
    finally:
        if ingress is not None:
            ingress.close();assert not ingress.thread.is_alive()
        if publisher is not None:publisher.destroy_node()
        context.shutdown()


def mailbox():
    # Exercise the actual worker callbacks/handoff without a ROS participant.
    from collections import deque
    from owner_tf_delivery import LocalizationIngress, TFDelivery
    obj=LocalizationIngress.__new__(LocalizationIngress)
    obj.delivery=TFDelivery();obj.odometry=deque(maxlen=128)
    obj.failure=None;obj.last_odom_stamp=None
    obj.thread=NS(is_alive=lambda:True);obj.stop=threading.Event()
    return obj


def test_mailbox_keeps_receipt_deadline_and_main_thread_owns_histories(owner):
    m,wall=owner;m.ingress=mailbox();m.tf_delivery=m.ingress.delivery
    m.receive_truth(message(18.))
    m.ingress.receive_odom(message(18.))
    assert not m.stamp_history.get('odom')
    wall[0]=100.4;m.ingress.receive_odom(message(18.))
    assert m.ready()
    assert m.last_readiness['predicates']['odom']['receipt_wall']==100.
    wall[0]=100.500001
    assert not m.ready()
    assert not m.last_readiness['predicates']['odom']['passed']


def test_mailbox_retains_regression_fault_even_after_bounded_overflow(owner):
    m,wall=owner;ing=mailbox();m.ingress=ing;m.tf_delivery=ing.delivery
    ing.receive_odom(message(18.));ing.receive_odom(message(17.9))
    for i in range(300):ing.receive_odom(message(18.+i*.001))
    assert len(ing.odometry)==128
    m.receive_truth(message(18.));assert not m.ready()
    assert m.failure=='lio_stamp_regression' and len(ing.odometry)==0


def test_worker_exception_and_unexpected_exit_fail_closed(owner):
    m,wall=owner;ing=mailbox();m.ingress=ing;m.tf_delivery=ing.delivery
    def fail(**kw):raise RuntimeError('broken ingress')
    ing.executor=NS(spin_once=fail);ing.run()
    m.receive_truth(message(18.));assert not m.ready()
    assert 'broken ingress' in m.failure
    m.failure=None;ing.failure=None;ing.thread=NS(is_alive=lambda:False)
    assert not m.ready() and m.failure=='localization_executor_stopped'


def test_future_only_tf_is_rejected_without_buffer_coverage(owner):
    m,wall=owner;obj=listener();m.tf=obj.buffer;m.tf_delivery=obj.delivery
    m.pose=trial.Monitor.pose.__get__(m)
    obj.callback(NS(transforms=[transform(stamp=18.1),transform('odom','base_link',18.1)]))
    with pytest.raises(Exception):m.pose()


def test_freshness_boundary_is_unchanged(owner):
    m,wall=owner;obj=listener();m.tf=obj.buffer;m.tf_delivery=obj.delivery
    m.pose=trial.Monitor.pose.__get__(m)
    obj.callback(NS(transforms=[transform(stamp=17.5),transform('odom','base_link',17.5)]))
    assert m.pose()==(0.,0.,0.)
    m.clock=18.000001
    with pytest.raises(ValueError,match='stale_map_base_tf'):m.pose()


def test_future_tf_cannot_interpolate_away_stale_source_stamp(owner):
    m,wall=owner;obj=listener();m.tf=obj.buffer;m.tf_delivery=obj.delivery
    m.pose=trial.Monitor.pose.__get__(m)
    for stamp in (17.,18.1):obj.callback(NS(transforms=[transform(stamp=stamp),transform('odom','base_link',stamp)]))
    with pytest.raises(ValueError,match='stale_map_base_tf'):m.pose()
