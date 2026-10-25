"""Owner listener boundary tests; real tf2 buffer, no ROS participant."""
import copy
from types import SimpleNamespace as NS
import pytest
from test_warehouse_truth_admission import owner, message


def transform(parent='map', child='odom', stamp=14.1):
    from geometry_msgs.msg import TransformStamped
    tr = TransformStamped()
    tr.header.frame_id = parent; tr.child_frame_id = child
    tr.header.stamp.sec = int(stamp)
    tr.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    tr.transform.rotation.w = 1.
    return tr


def listener():
    from tf2_ros import Buffer
    from owner_tf_delivery import observed_listener_type, TFDelivery
    observed = observed_listener_type()
    obj = observed.__new__(observed)
    obj.node = NS(destroy_subscription=lambda sub: None); obj.tf_static_sub = None
    obj.buffer = Buffer(); obj.delivery = TFDelivery()
    return obj


def test_listener_preserves_buffer_results_including_rejected_and_old_tf():
    from tf2_ros import Buffer, TransformListener
    from rclpy.time import Time
    obj = listener(); plain = TransformListener.__new__(TransformListener); plain.buffer = Buffer()
    plain.node = NS(destroy_subscription=lambda sub: None); plain.tf_static_sub = None
    invalid = transform(stamp=14.2); invalid.transform.rotation.w = 0.
    for tr in [transform(), transform('odom','base_link'), transform(stamp=14.), invalid]:
        msg = NS(transforms=[tr]); original = copy.deepcopy(tr)
        obj.callback(msg); plain.callback(msg)
        assert tr == original
        assert obj.buffer.lookup_transform('map','odom',Time()) == plain.buffer.lookup_transform('map','odom',Time())
    edge = obj.delivery.snapshot(obj.buffer)['edges']['map->odom']
    assert edge['received_stamp'] == 14.2
    assert edge['inserted_latest_stamp'] == 14.1  # rejected quaternion cannot claim insertion
    assert edge['callbacks'] == 3
    assert obj.delivery.snapshot(obj.buffer)['latest_common_stamp'] == 14.1


def test_observation_storage_is_bounded_and_snapshot_detached():
    obj = listener()
    for i in range(300):
        obj.callback(NS(transforms=[transform(stamp=14.+i/1000), transform('arbitrary'+str(i),'child')]))
    snap = obj.delivery.snapshot(obj.buffer)
    assert set(snap['edges']) == {'map->odom','odom->base_link'}
    assert snap['callback_count'] == 300
    obj.callback(NS(transforms=[transform(stamp=15.)]))
    assert snap['edges']['map->odom']['received_stamp'] == pytest.approx(14.299)
    assert obj.delivery.edges['map->odom']['received_stamp'] == 15.


def test_monotonic_times_and_spin_gaps(monkeypatch):
    import owner_tf_delivery as mod
    wall = [100.]; monkeypatch.setattr(mod.time,'monotonic',lambda:wall[0])
    obj = listener(); d = obj.delivery
    d.spin_start(); wall[0]=100.02; d.spin_end()
    wall[0]=100.12; d.spin_start(); wall[0]=100.13; d.spin_end()
    obj.callback(NS(transforms=[transform()]))
    snap=d.snapshot(obj.buffer)
    assert snap['spin_count'] == 2
    assert snap['max_between_spins_s'] == pytest.approx(.10)
    assert snap['last_between_spins_s'] == pytest.approx(.10)
    assert snap['last_spin_duration_s'] == pytest.approx(.01)
    assert snap['max_spin_duration_s'] == pytest.approx(.02)
    edge=snap['edges']['map->odom']
    assert edge['callback_wall'] == edge['insertion_wall'] == 100.13


def test_first_fault_attaches_observation_once_without_changing_readiness(owner):
    m,wall=owner; m.receive_truth(message(18.)); m.phase='precheck'
    obj=listener(); m.tf=obj.buffer; m.tf_delivery=obj.delivery
    obj.callback(NS(transforms=[transform(stamp=17.4),transform('odom','base_link',17.4)]))
    from rclpy.time import Time
    def stale(**kw):
        tr=m.tf.lookup_transform('map','base_link',Time())
        assert tr.header.stamp.sec == 17
        raise ValueError('stale_map_base_tf')
    m.pose=stale
    with pytest.raises(RuntimeError): m.update()
    saved=copy.deepcopy(m.readiness_first_fault)
    assert saved['tf_delivery']['latest_common_stamp'] == 17.4
    assert not saved['predicates']['tf']['passed']
    obj.callback(NS(transforms=[transform(stamp=18.)]))
    m.record_readiness_fault()
    assert m.readiness_first_fault == saved


def test_same_listener_subscriptions_qos_group_and_no_spin_thread():
    from tf2_ros import Buffer, TransformListener
    from owner_tf_delivery import observed_listener_type, TFDelivery
    calls=[]
    node=NS(create_subscription=lambda *args,**kw:calls.append((args,kw)),
            destroy_subscription=lambda sub:None)
    plain=TransformListener(Buffer(),node)
    baseline=list(calls); calls.clear()
    observed=observed_listener_type()(Buffer(),node,TFDelivery())
    assert len(calls)==len(baseline)==2
    for (args,kw),(old,old_kw) in zip(calls,baseline):
        assert args[:2]==old[:2] and args[3]==old[3]
        assert type(kw['callback_group']) is type(old_kw['callback_group'])
    assert calls[0][1]['callback_group'] is calls[1][1]['callback_group']
    assert observed.node is node and not hasattr(observed,'executor')


def test_original_insertion_exception_propagates_without_claiming_completion():
    obj=listener()
    def fail(*args): raise RuntimeError('original buffer error')
    obj.buffer=NS(set_transform=fail)
    with pytest.raises(RuntimeError,match='original buffer error'):
        obj.callback(NS(transforms=[transform()]))
    edge=obj.delivery.edges['map->odom']
    assert edge['received_stamp']==14.1 and edge['insertion_wall'] is None
