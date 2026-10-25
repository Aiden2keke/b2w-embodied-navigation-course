"""Offline actual callback/control-flow regressions; no ROS participant or motion."""
from collections import deque
from types import SimpleNamespace as NS
import math
import numpy as np
import pytest
from test_warehouse_precheck_only import session, trial
from test_owner_tf_delivery import listener, transform
from test_warehouse_truth_admission import owner, message


def pose_adapter(receiver, adapter_class=None):
    from course_slam.pose_adapter import PoseAdapter
    cls = adapter_class or PoseAdapter
    p = cls.__new__(cls)
    p.previous = None; p.local = deque(maxlen=200); p.optimized = deque(maxlen=200)
    p.alignment = p.lidar_base = p.correction = np.eye(4)
    p.ground_ready = True; p.fit_ground = False
    p.odom = 'odom'; p.base = 'base_link'; p.map = 'map'
    p.pub = NS(publish=lambda msg: None)
    p.tf = NS(sendTransform=receiver)
    return p


def test_coherent_pose_update_never_exposes_half_updated_common_tf():
    from rclpy.time import Time
    obj = listener()
    obj.callback(NS(transforms=[transform(stamp=33.8), transform('odom','base_link',33.8)]))
    common = []
    def receive(transforms):
        obj.callback(NS(transforms=transforms if isinstance(transforms,list) else [transforms]))
        stamp = obj.buffer.lookup_transform('map','base_link',Time()).header.stamp
        common.append(stamp.sec + stamp.nanosec*1e-9)
    p = pose_adapter(receive)
    from nav_msgs.msg import Odometry
    from builtin_interfaces.msg import Time as Stamp
    m = Odometry(); m.header.stamp = Stamp(sec=33,nanosec=900000000)
    m.pose.pose.orientation.w = 1.
    p.incremental(m)
    assert common == [33.9]  # old code exposes33.8 between two callbacks
    snapshot = obj.delivery.snapshot(obj.buffer)
    assert snapshot['edges']['map->odom']['received_stamp'] == 33.9
    assert snapshot['edges']['odom->base_link']['received_stamp'] == 33.9
    assert 34.305 - common[-1] < .5


def adapter_output():
    from course_navigation.velocity_adapter import VelocityAdapter, VelocityGate
    p = VelocityAdapter.__new__(VelocityAdapter)
    p.gate = VelocityGate(); p.require_tf = False
    output = []
    p.pub = NS(publish=lambda m: output.append((m.linear.x,m.linear.y,m.linear.z,
                                               m.angular.x,m.angular.y,m.angular.z)))
    return p, output


def test_zero_transition_reaches_final_output_without_timer_and_without_zero_flood():
    from geometry_msgs.msg import Twist
    from course_navigation import velocity_adapter as mod
    p, out = adapter_output()
    p.gate.state_time = mod.time.monotonic()
    cmd = Twist(); cmd.angular.z = .05
    p.command(cmd); p.publish()
    assert out[-1][-1] == .05
    p.command(Twist())
    assert out[-1] == (0.,)*6  # no timer dispatch required
    count = len(out)
    for _ in range(100): p.command(Twist())
    assert len(out) == count


def test_owner_fault_sends_zero_through_existing_adapter_before_owned_cleanup(session, monkeypatch):
    import rclpy
    from geometry_msgs.msg import Twist
    p, out = adapter_output()
    node = rclpy.create_node
    def create(*args, **kwargs):
        n = node(*args, **kwargs); original = n.create_publisher
        def publisher(*args, **kwargs):
            pub = original(*args, **kwargs); send = pub.publish
            def publish(msg):
                send(msg); p.command(msg)
                if msg.linear.x or msg.angular.z:
                    p.gate.state_time = session.wall; p.publish()
            pub.publish = publish
            return pub
        n.create_publisher = publisher
        return n
    monkeypatch.setattr(rclpy, 'create_node', create)
    session.fault = 'turn'
    assert session.run() == 1
    assert any(v[-1] == .05 for v in out)
    assert out[-1] == (0.,)*6
    assert session.commands[-1][1] == [0.]*6
    assert session.report()['fault_stop']['zero_input_published'] is True
    assert session.report()['cleanup']['passed']


def test_original_translation_boundary_covers_stop_from_initial_motion_reference(owner):
    m, wall = owner
    msg = message(18.); msg.pose.pose.position.x = .26
    m.receive_truth(msg); m.select_truth(wall[0])
    m.phase = 'precheck'; m.precheck_motion_origin = (0.,0.,0.)
    m.final = [0.]*6
    assert m.physical_veto() == 'precheck_initial_reference_excursion'
    m.phase = 'navigation'
    assert m.physical_veto() is None


def test_short_pulses_keep_original_amplitudes_and_zero_rest(session):
    session.args.precheck_pulse_s = 1.
    assert session.run() == 0, session.report().get('error')
    for p in session.report()['precheck']:
        assert 1. <= p['end_pulse_sim']-p['start_sim'] < 1.2
        assert p['rest']['six_axis_final_zero']
    assert max(v[0] for _,v in session.commands) == .05
    assert max(v[5] for _,v in session.commands) == .05


@pytest.mark.parametrize("baseline", [True,False])
def test_pulse_handoff_waits_for_owner_clock_after_internal_callback(session, monkeypatch, baseline):
    import rclpy
    write = trial.write_json; spin = rclpy.spin_once
    pending = [False]; handoffs = []; first_command = []
    def report(path, value):
        write(path,value)
        if (path.name == 'session-report.json' and value.get('precheck')
                and not value['precheck'][-1]['command_publication_attempted']):
            pending[0] = True
            handoffs.append(session.monitor.clock)
    def dispatch(node, timeout_sec):
        if pending[0]:
            pending[0] = False; session.wall += .01
            return  # actual TimeSource/internal callback, no Owner sample
        spin(node,timeout_sec)
    create = rclpy.create_node
    def node(*a, **kw):
        n = create(*a, **kw); original = n.create_publisher
        def publisher(*a, **kw):
            p = original(*a, **kw); send = p.publish
            def publish(m):
                if (m.linear.x or m.angular.z) and len(first_command) < len(handoffs):
                    first_command.append(session.monitor.clock)
                send(m)
            p.publish = publish
            return p
        n.create_publisher = publisher
        return n
    monkeypatch.setattr(trial,'write_json',report)
    monkeypatch.setattr(rclpy,'spin_once',dispatch)
    monkeypatch.setattr(rclpy,'create_node',node)
    if baseline:
        import ast
        path = trial.ROOT/'artifacts/recovery/first-motion-control-risks-20261025/warehouse_ab_trial.py.before'
        tree = ast.parse(path.read_text())
        function = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute')
        namespace = dict(trial.__dict__)
        exec(compile(ast.Module(body=[function],type_ignores=[]),str(path),'exec'),namespace)
        session.run = lambda: namespace['execute'](session.args,session.output)
    assert session.run() == 0, session.report().get('error')
    assert len(first_command) == 2
    assert all((new == old if baseline else new > old) for old,new in zip(handoffs,first_command))


@pytest.mark.parametrize('contained',[True,False])
def test_watchdog_stop_drain_follows_confirmed_pause_and_precedes_cleanup(tmp_path, monkeypatch, contained):
    import warehouse_ab_supervisor as s
    pin = dict(pid=123,start_ticks=456,state='T')
    order = []
    monkeypatch.setattr(s,'validate_config',lambda config: None)
    monkeypatch.setattr(s,'discover_servers',lambda *args: [pin])
    monkeypatch.setattr(s,'same_process',lambda *args: False)  # parent dead
    monkeypatch.setattr(s,'pause_server',lambda *args: order.append('pause') or dict(contained=contained))
    monkeypatch.setattr(s.time,'sleep',lambda seconds: order.append(('drain',seconds)))
    monkeypatch.setattr(s,'terminate_owned',lambda *args: order.append('cleanup') or dict(remaining={}))
    config=dict(token='inert',parent=pin,server=pin,wall_limit_s=420.,
                heartbeat=str(tmp_path/'health'),result=str(tmp_path/'result'),world='/tmp/inert')
    assert s.watchdog(config) == 1
    assert order == (['pause',('drain',.1),'cleanup'] if contained else ['pause','cleanup'])


@pytest.mark.parametrize("pulse", ["0", "-1", "2.01", "nan", "inf"])
def test_invalid_pulse_never_reaches_inputs_or_execution(tmp_path, monkeypatch, pulse):
    def forbidden(*a, **kw):
        raise AssertionError("Invalid pulse reached preflight")
    monkeypatch.setattr(trial,"check_inputs",forbidden)
    monkeypatch.setattr(trial,"execute",forbidden)
    with pytest.raises(SystemExit) as error:
        trial.main(["--profile","A","--precheck-only","--precheck-pulse-s",pulse,
                    "--output-root",str(tmp_path)])
    assert error.value.code == 2 and not list(tmp_path.iterdir())


def test_batched_tf_preserves_exact_estimate_frames_values_stamps_and_odom():
    from nav_msgs.msg import Odometry
    from builtin_interfaces.msg import Time
    from scipy.spatial.transform import Rotation
    path = trial.ROOT/"artifacts/recovery/first-motion-control-risks-20261025/pose_adapter.py.before"
    scope = {"__name__":"course_slam.baseline", "__package__":"course_slam"}
    exec(compile(path.read_text(),str(path),"exec"),scope)
    results = []
    for cls in (scope["PoseAdapter"], None):
        transforms = []; odometry = []
        p = pose_adapter(lambda value: transforms.extend(value if isinstance(value,list) else [value]), cls)
        p.pub = NS(publish=odometry.append)
        p.alignment = np.eye(4); p.alignment[:3,:3] = Rotation.from_euler("z",.2).as_matrix()
        p.lidar_base = np.eye(4); p.lidar_base[0,3] = -.34218
        p.correction = np.eye(4); p.correction[1,3] = .4
        msg = Odometry(); msg.header.stamp = Time(sec=33,nanosec=900000000)
        msg.pose.pose.position.x = .75; msg.pose.pose.orientation.w = 1.
        p.incremental(msg)
        results.append((transforms,odometry))
    assert results[0] == results[1]


@pytest.mark.parametrize('damage',['omit_source','change_source_path','omit_native'])
def test_deployment_override_cannot_omit_reviewed_identity_scope(tmp_path, damage):
    import json
    path = trial.ROOT/'artifacts/recovery/first-motion-control-risks-20261025/reviewed-deployment.json'
    value = json.loads(path.read_text())
    if damage == 'omit_source': value['source_install'].pop()
    elif damage == 'change_source_path': value['source_install'][0]['source'] = '/tmp/foreign.py'
    else: value['native'] = {}
    candidate = tmp_path/'manifest.json'; candidate.write_text(json.dumps(value))
    args = NS(deployment_manifest=candidate,
              engine_evidence=trial.ROOT/'artifacts/contact-engine-probe/result.json')
    with pytest.raises(ValueError,match='cannot omit identities'):
        trial.check_inputs(args)
