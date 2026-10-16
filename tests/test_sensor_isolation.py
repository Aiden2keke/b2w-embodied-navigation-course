"""Sensor contracts and a real DDS/GIL contention regression (no simulator)."""
import ctypes
import inspect
import multiprocessing as mp
from pathlib import Path
import time
import uuid

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, JointState
from std_msgs.msg import Header
import yaml

from course_sim.sensor_disturbance import SensorDisturbance
from course_sim.cloud_format import OUTPUT, decode, encode

ROOT = Path(__file__).resolve().parents[1]


def require_split():
    assert 'stream' in inspect.signature(SensorDisturbance).parameters, (
        'IMU and heavy cloud processing still share the same Python process')


@pytest.fixture
def context():
    # Absolute production names must also be remapped in spawned workers.
    prefix = '/sensor_isolation_' + uuid.uuid4().hex
    topics = ['/imu', '/pointcloud', '/joint_states', '/sim/raw/imu',
              '/sim/raw/pointcloud', '/sim/raw/joint_states']
    mapping = {t: prefix + t for t in topics}
    args = ['--ros-args']
    for topic, target in mapping.items():
        args.extend(['-r', topic + ':=' + target])
    rclpy.init(args=args, domain_id=215)
    try:
        yield {'args': args, 'topics': mapping}
    finally:
        rclpy.shutdown()


def test_launch_owns_distinct_sensor_processes():
    # These must be distinct launch_ros Node actions: separate OS processes/GILs.
    import ast
    tree = ast.parse((ROOT/'course_sim/launch/simulation.launch.py').read_text())
    executables = []
    parameters = []
    for call in ast.walk(tree):
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == 'Node':
            kw = {k.arg: k.value for k in call.keywords}
            if isinstance(kw.get('package'), ast.Constant) and kw['package'].value == 'course_sim':
                executables.append(kw['executable'].value)
                parameters.append(ast.dump(kw['parameters']))
    assert sorted(executables) == ['sensor_cloud_disturbance', 'sensor_disturbance']
    assert len(parameters) == 2 and parameters[0] == parameters[1]
    setup = (ROOT/'course_sim/setup.py').read_text()
    assert 'sensor_cloud_disturbance = course_sim.sensor_disturbance:cloud_main' in setup


def test_stream_subscriptions_and_qos_remain_disjoint(context):
    require_split()
    nodes = [SensorDisturbance(stream=s) for s in ['imu', 'cloud']]
    try:
        expected = [
            {'/sim/raw/imu': (Imu, 200), '/sim/raw/joint_states': (JointState, 100)},
            {'/sim/raw/pointcloud': (PointCloud2, 5)}]
        raw_topics = {context['topics'][t] for t in context['topics'] if t.startswith('/sim/raw/')}
        for n, contract in zip(nodes, expected):
            contract = {context['topics'][t]: v for t, v in contract.items()}
            actual = {s.topic_name: s for s in n.subscriptions if s.topic_name in raw_topics}
            assert actual.keys() == contract.keys()
            for topic, (typ, depth) in contract.items():
                assert actual[topic].msg_type is typ
                assert actual[topic].qos_profile.depth == depth
                assert actual[topic].qos_profile.reliability == qos_profile_sensor_data.reliability
        public = {context['topics'][t] for t in ['/imu','/pointcloud','/joint_states']}
        assert {p.topic_name for p in nodes[0].publishers if p.topic_name in public} == {context['topics'][t] for t in ['/imu','/joint_states']}
        assert {p.topic_name for p in nodes[1].publishers if p.topic_name in public} == {context['topics']['/pointcloud']}
    finally:
        for n in nodes: n.destroy_node()


class Capture:
    def __init__(self): self.messages = []
    def publish(self, msg): self.messages.append(msg)


@pytest.mark.parametrize('profile', ['normal', 'mild', 'medium', 'severe'])
def test_seeded_outputs_stamps_frames_and_profiles_unchanged(context, profile):
    require_split()
    config = yaml.safe_load((ROOT/'course_sim/config/fault_profiles.yaml').read_text())[profile]
    seeds = np.random.SeedSequence(17).spawn(2)
    expected_lidar = np.random.default_rng(seeds[0])
    expected_imu = np.random.default_rng(seeds[1])
    imu = SensorDisturbance(stream='imu')
    cloud = SensorDisturbance(stream='cloud')
    assert imu.imu_rng.bit_generator.state == np.random.default_rng(np.random.SeedSequence(0).spawn(2)[1]).bit_generator.state
    assert cloud.lidar_rng.bit_generator.state == np.random.default_rng(np.random.SeedSequence(0).spawn(2)[0]).bit_generator.state
    imu.config.update(config);cloud.config.update(config)
    imu.imu_rng = np.random.default_rng(seeds[1])
    cloud.lidar_rng = np.random.default_rng(seeds[0])
    imu.imu_pub = Capture();cloud.cloud_pub = Capture()
    try:
        points = np.zeros(640, dtype=OUTPUT)
        el = np.deg2rad(np.repeat(np.arange(-16, 16), 20))
        points['x'] = 10*np.cos(el);points['z'] = 10*np.sin(el)
        src = encode(points, Header(frame_id='raw_lidar'))
        src.header.stamp.sec,src.header.stamp.nanosec = 123,456789
        from course_sim.cloud_format import xt32_points
        for i in range(3):
            raw = Imu();raw.header.stamp.sec = 123;raw.header.stamp.nanosec = 456789+i*5000000
            raw.header.frame_id = 'raw_imu';raw.linear_acceleration.z = 9.81
            # The exact existing stream-specific seed/noise sequence is the oracle.
            gyro = expected_imu.normal(config['bias_imu_gyro'],config['sigma_imu_gyro'],3)
            acc = expected_imu.normal(config['bias_imu_acc'],config['sigma_imu_acc'],3)+[0,0,9.81]
            imu.imu(raw);msg = imu.imu_pub.messages[-1]
            assert msg.header.stamp == raw.header.stamp and raw.header.frame_id == 'raw_imu'
            assert msg.header.frame_id == 'imu_link'
            np.testing.assert_array_equal([getattr(msg.angular_velocity,a) for a in 'xyz'],gyro)
            np.testing.assert_array_equal([getattr(msg.linear_acceleration,a) for a in 'xyz'],acc)
            np.testing.assert_array_equal(msg.angular_velocity_covariance,(np.eye(3)*config['sigma_imu_gyro']**2).reshape(-1))
            assert np.isfinite([getattr(msg.orientation,a) for a in 'xyzw']).all()
            expected = xt32_points(src,config['sigma_lidar'],config['p_lidar_drop'],expected_lidar)
            cloud.cloud(src);msg = cloud.cloud_pub.messages[-1]
            assert msg.header.stamp == src.header.stamp and src.header.frame_id == 'raw_lidar'
            assert msg.header.frame_id == 'lidar_link'
            # Structured-array padding (bytes 18/19) is not a point field.
            actual = decode(msg)
            for name in OUTPUT.names:
                np.testing.assert_array_equal(actual[name], expected[name])
            assert msg.point_step == 24 and msg.row_step == msg.width*24
            assert [(f.name,f.offset,f.datatype) for f in msg.fields] == [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('time',20,7)]
            assert np.all(expected['time'] == 0) and np.all(expected['ring'] < 32)
    finally:
        imu.destroy_node();cloud.destroy_node()


def _worker(stream, ready, entered, release, ros_args):
    """Same Node/executor as production; cloud callback holds GIL in native code."""
    rclpy.init(args=ros_args, domain_id=215)
    if stream == 'cloud':
        original = SensorDisturbance.cloud
        def heavy(self, msg):
            entered.set()
            # PyDLL deliberately keeps the GIL for the entire native call.
            ctypes.PyDLL(None).usleep(1_500_000)
            release.set()
            original(self, msg)
        SensorDisturbance.cloud = heavy
    n = SensorDisturbance(stream=stream);ex = SingleThreadedExecutor();ex.add_node(n)
    ready.set()
    try: ex.spin()
    except KeyboardInterrupt: pass
    finally:
        ex.shutdown();n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def test_imu_progresses_during_non_yielding_cloud_callback(context):
    require_split()
    ctx = mp.get_context('spawn')
    entered,release = ctx.Event(),ctx.Event()
    ready = [ctx.Event(),ctx.Event()]
    children = [ctx.Process(target=_worker,args=(s,r,entered,release,context['args'])) for s,r in zip(['imu','cloud'],ready)]
    n = rclpy.create_node('isolation_regression');ex = SingleThreadedExecutor();ex.add_node(n)
    received = []
    n.create_subscription(Imu,'/imu',lambda m:received.append(m.header.stamp.nanosec),qos_profile_sensor_data)
    ip = n.create_publisher(Imu,'/sim/raw/imu',10)
    cp = n.create_publisher(PointCloud2,'/sim/raw/pointcloud',10)
    try:
        assert ip.topic_name.startswith('/sensor_isolation_'), 'synthetic DDS traffic must use unique test topics'
        for child in children:child.start()
        assert all(r.wait(10) for r in ready)
        until = time.monotonic()+10
        while time.monotonic()<until and (ip.get_subscription_count()<1 or cp.get_subscription_count()<1 or not n.count_publishers(context['topics']['/imu'])):
            ex.spin_once(timeout_sec=.01)
        assert ip.get_subscription_count() and cp.get_subscription_count()
        points = np.zeros(32,dtype=OUTPUT)
        el = np.deg2rad(np.arange(-16,16));points['x']=10*np.cos(el);points['z']=10*np.sin(el)
        cp.publish(encode(points,Header(frame_id='lidar_link')))
        assert entered.wait(5)
        # These are published AFTER the cloud enters a GIL-holding callback.
        for i in range(20):
            msg = Imu();msg.header.stamp.sec=12;msg.header.stamp.nanosec=5000000*i;msg.linear_acceleration.z=9.81
            ip.publish(msg)
            end=time.monotonic()+.01
            while time.monotonic()<end:ex.spin_once(timeout_sec=.001)
        assert len(received)>=15 and max(received)>=75000000
        assert not release.is_set(), 'IMU only resumed after cloud completed'
    finally:
        for child in children:
            if child.is_alive():child.terminate()
            if child.pid is not None:
                child.join(5)
                if child.is_alive():child.kill();child.join(2)
                assert not child.is_alive()
        ex.shutdown();n.destroy_node()
