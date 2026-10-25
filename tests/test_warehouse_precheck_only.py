"""Run the real trial control flow with in-memory ROS/process boundaries.

No ROS initialization, DDS participant, OS signal, child or velocity publication.
Existing Monitor/readiness, StopEvidence and watchdog tests cover those components.
"""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import warehouse_ab_trial as trial


@pytest.fixture
def session(tmp_path, monkeypatch):
    import rclpy
    import run_warehouse_route as route
    from test_warehouse_truth_admission import message

    state = NS(wall=1000., monitor=None, commands=[], processes=[], fault=None,
               readiness_calls=0, physical_checks=0, cleanup_calls=0, goals=0,
               audit_finished=False, post_audit_bootstrap_spins=0)
    pin = dict(pid=123, start_ticks=456, state='R')
    description = (trial.ROOT/'artifacts/recovery/robot.urdf').read_text()
    plugin = trial.ROOT/'artifacts/recovery/contact-build/libb2w_contact_observer.so'

    def forbidden_goal(*args, **kwargs):
        state.goals += 1
        raise AssertionError('Precheck reached navigation')
    monkeypatch.setattr(route, 'run_critical', forbidden_goal)
    monkeypatch.setattr(route, 'RosGoalTransport', forbidden_goal)
    monkeypatch.setattr(route, 'load_route', forbidden_goal)

    class Publisher:
        def publish(self, command):
            values = [getattr(getattr(command, a), b) for a in ('linear', 'angular') for b in ('x', 'y', 'z')]
            state.commands.append((state.wall, values))
            state.monitor.final = values

    class Node:
        def get_node_names(self):
            return ['warehouse_ab_owner'] + (['goal_manager'] if state.fault == 'goal_source' and state.monitor else [])
        def get_publishers_info_by_topic(self, topic):
            if topic == '/goal_pose': return []
            if topic == '/path_manager/path_manager_ros/nav_vel' and any(
                    'input_topic:=/warehouse_ab/precheck' in p.argv and p.poll() is not None
                    for p in state.processes): return []
            count = 2 if state.fault == 'duplicate_owner' else 1
            return [NS(node_name='sole_owner') for _ in range(count)]
        def create_publisher(self, kind, topic, depth):
            assert topic == '/warehouse_ab/precheck'
            return Publisher()
        def create_client(self, kind, name):
            if name == '/planner_server/get_state':
                return NS(service_is_ready=lambda:True,call_async=lambda request:NS(done=lambda:True,result=lambda:NS(current_state=NS(id=3))))
            if name == '/planner_server/get_parameters':
                response=NS(values=[NS(type=4,string_value='nav2_smac_planner/SmacPlanner2D'),NS(type=1,bool_value=False)])
                return NS(service_is_ready=lambda:True,call_async=lambda request:NS(done=lambda:True,result=lambda:response))
            assert name == '/b2w_robot_state_publisher/get_parameters'
            response = NS(values=[NS(type=4, string_value=description)])
            return NS(service_is_ready=lambda: True,
                      call_async=lambda request: NS(done=lambda: True, result=lambda: response))
        def destroy_client(self, client): pass
        def destroy_publisher(self, publisher): pass
        def destroy_node(self): pass

    class Monitor:
        def __init__(self, node, inventory, heartbeat):
            self.clock = 10.; self.clock_wall = state.wall
            self.phase = 'bootstrap'; self.failure = None
            self.grid = dict(width=40, height=40, resolution=.1, origin=(-2., -2.), data=[0]*1600)
            self.grid['observed_free_present'] = True
            if state.fault in ('map_observation_delayed','map_observation_missing'):
                self.grid.update(data=[-1]*1600, observed_free_present=False)
                state.map_bootstrap_spins = 0
            if state.fault == 'clearance': self.grid['data'][820] = -1
            self.contacts = NS(failure=None, intervals=NS(closed=lambda **kwargs: True))
            self.support_feet = {'FL', 'FR', 'RL', 'RR'}; self.samples = 10
            self.truth_first_fault = self.readiness_first_fault = None
            self.truth = (self.clock, state.wall, message(self.clock))
            self.final = [0.]*6
            self.events = (tmp_path/'monitor-events.jsonl').open('w')
            self.tf_delivery = NS(spin_start=lambda: None, spin_end=lambda: None)
            def close_ingress():
                state.ingress_closed = True
                if state.fault == 'ingress_cleanup': raise RuntimeError('injected ingress cleanup')
            self.ingress = NS(close=close_ingress)
            state.monitor = self
        def ready(self):
            state.readiness_calls += 1
            return state.fault != 'readiness'
        def update(self):
            if (self.phase == 'precheck' and state.fault == 'post_audit_backlog'
                    and state.post_audit_bootstrap_spins < 6):
                self.failure = 'stale_after_cached_clock_admission'
            if self.failure: raise RuntimeError(self.failure)
        def pose(self): return (0., 0., 0.)
        def truth_pose(self): return (0., 0., 0.)
        def resting(self):
            return not any(self.final) and state.fault != 'physical_stop'
        def physical_veto(self):
            state.physical_checks += 1
            moving = self.final[0] != 0 if state.fault == 'forward' else self.final[5] != 0
            if state.fault in ('forward', 'turn') and moving: return 'injected_physical_veto'
        def select_truth(self, wall): return True
        def record_readiness_fault(self): self.readiness_first_fault = dict(injected=True)
    monkeypatch.setattr(trial, 'Monitor', Monitor)

    class Process:
        def __init__(self, argv, **kwargs):
            if 'navigation.launch.py' in argv: forbidden_goal()
            self.argv = argv; self.pid = len(state.processes)+100
            self.returncode = None
            self.watchdog = '--watchdog' in argv
            state.processes.append(self)
            if 'course.launch.py' in argv: trial.write_json(tmp_path/'server-pin.json', pin)
        def poll(self): return self.returncode
        def send_signal(self, sig): self.returncode = 0
        def wait(self, timeout=None):
            self.returncode = 1 if self.watchdog and state.fault == 'watchdog_completion' else 0
            if self.watchdog:
                trial.write_json(tmp_path/'watchdog-result.json', dict(reason='normal_shutdown', emergency=False))
            return self.returncode
    monkeypatch.setattr(trial.subprocess, 'Popen', Process)

    class Owned:
        def __init__(self, token): self.seen = {}; self.token = token
        def scan(self):
            live = {p.pid: '456' for p in state.processes if not p.watchdog and p.poll() is None}
            self.seen.update(live)
            return live
        def observe(self): self.scan()
        def send(self, sig):
            for p in state.processes:
                if not p.watchdog and p.poll() is None: p.send_signal(sig)
        def cleanup(self, process):
            state.cleanup_calls += 1
            for p in state.processes: p.returncode = 1
            return dict(passed=True, remaining={}, owned_pid_start_ticks=self.seen)
    monkeypatch.setattr(trial, 'OwnedProcesses', Owned)

    def spin(node, timeout_sec):
        state.wall += .1
        if state.monitor:
            if state.fault == 'map_observation_delayed' and not state.audit_finished:
                state.map_bootstrap_spins += 1
                if state.map_bootstrap_spins >= 10:
                    state.monitor.grid = dict(state.monitor.grid, data=[0]*1600,
                                              observed_free_present=True)
            if state.audit_finished and state.monitor.phase == 'bootstrap':
                state.post_audit_bootstrap_spins += 1
            if not ((state.fault == 'pulse_clock_freeze' and any(state.monitor.final))
                    or (state.fault == 'readmission_clock_freeze' and state.audit_finished)):
                state.monitor.clock += .1
            state.monitor.clock_wall = state.wall
    monkeypatch.setattr(trial.time, 'monotonic', lambda: state.wall)
    monkeypatch.setattr(trial.time, 'sleep', lambda seconds: setattr(state, 'wall', state.wall+seconds))
    monkeypatch.setattr(rclpy, 'init', lambda: None)
    monkeypatch.setattr(rclpy, 'create_node', lambda *args, **kwargs: Node())
    monkeypatch.setattr(rclpy, 'spin_once', spin)
    monkeypatch.setattr(rclpy, 'ok', lambda: False)
    monkeypatch.setattr(trial, 'identity', lambda pid: dict(pid=pid, start_ticks=456))
    monkeypatch.setattr(trial, 'same_process', lambda *args: True)
    monkeypatch.setattr(trial.os, 'kill', lambda *args: None)
    monkeypatch.setattr(trial, 'pause_server', lambda *args: dict(contained=True))
    def native_snapshot(*args, **kwargs):
        state.audit_finished = True
        return {}
    monkeypatch.setattr(trial, 'native_snapshot', native_snapshot)
    monkeypatch.setattr(trial, 'construction_audit', lambda output: ['fixture-stack.log'])
    monkeypatch.setattr(trial, 'world_copy', lambda *args: dict(plugin_sha256=trial.digest(plugin)))
    read_text = Path.read_text
    def read(path, *args, **kwargs):
        if str(path) == '/proc/123/maps':
            return str(plugin)+'\n/usr/lib/x86_64-linux-gnu/ign-physics-5/engine-plugins/libignition-physics5-dartsim-plugin.so.5.4.0'
        return read_text(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'read_text', read)
    args = NS(profile='A', stationary_containment=False, precheck_only=True,
              authorize_motion='precheck-only', wall_limit=420.,
              engine_evidence=trial.ROOT/'artifacts/contact-engine-probe/result.json')
    state.output = tmp_path
    state.args = args
    state.run = lambda: trial.execute(args, tmp_path)
    state.report = lambda: json.loads((tmp_path/'session-report.json').read_text())
    return state


def test_precheck_success_finishes_rest_and_owned_shutdown_without_navigation(session):
    assert session.run() == 0, session.report().get('error')
    report = session.report()
    assert report['success'] and 'route' not in report and session.goals == 0
    assert [p['kind'] for p in report['precheck']] == ['forward', 'turn']
    assert {tuple(values) for _, values in session.commands} == {
        (0., 0., 0., 0., 0., 0.), (.05, 0., 0., 0., 0., 0.), (0., 0., 0., 0., 0., .05)}
    for pulse in report['precheck']:
        assert 2. <= pulse['end_pulse_sim']-pulse['start_sim'] < 2.2
        assert pulse['pulse_wall_s'] <= 10.
        assert pulse['rest']['end_sim']-pulse['rest']['start_sim'] >= 5.
        assert pulse['rest']['measured_ground_truth_rest']
    assert report['normal_rest']['six_axis_final_zero']
    assert report['localization_cleanup']['passed'] and session.ingress_closed
    assert report['cleanup']['passed'] and report['cleanup']['remaining'] == {}
    assert report['watchdog'] == dict(reason='normal_shutdown', emergency=False)
    assert session.readiness_calls > 0 and session.physical_checks > 0
    assert session.commands[-1][1] == [0.]*6
    assert all(p.poll() is not None for p in session.processes)


@pytest.mark.parametrize('fault', ['clearance', 'duplicate_owner', 'goal_source', 'readiness',
                                  'forward', 'turn', 'physical_stop', 'pulse_clock_freeze'])
def test_precheck_rejections_never_fall_through_to_route_and_clean_owned(session, fault):
    session.fault = fault
    assert session.run() == 1
    report = session.report()
    assert not report['success'] and report['error']
    assert 'route' not in report and session.goals == 0
    assert report['cleanup']['passed'] and session.cleanup_calls == 1
    if fault in ('clearance', 'duplicate_owner', 'goal_source', 'readiness', 'physical_stop'):
        assert not any(any(values) for _, values in session.commands)
    assert all(p.poll() is not None for p in session.processes)


def test_precheck_watchdog_rejection_after_measured_stop_cannot_report_success(session):
    session.fault = 'watchdog_completion'
    assert session.run() == 1
    report = session.report()
    assert not report['success'] and report['error'] == "RuntimeError('Watchdog rejected normal completion')"
    assert len(report['precheck']) == 2 and 'route' not in report and session.goals == 0
    assert report['cleanup']['passed']


def test_clearance_failure_snapshot_is_written_before_raise(session):
    session.fault = 'clearance'
    assert session.run() == 1
    snapshot = session.output/'pocket-admission-initial.json'
    assert snapshot.exists()
    saved = json.loads(snapshot.read_text())
    assert saved['decision'] is False and saved['rejected_cells']['unknown'] == 1
    assert session.report()['precheck'] == []


def test_each_motion_admission_records_one_snapshot(session):
    assert session.run() == 0
    assert {p.name for p in session.output.glob('pocket-admission-*.json')} == {
        'pocket-admission-initial.json', 'pocket-admission-forward.json', 'pocket-admission-turn.json'}


def test_alternate_precheck_spawn_is_explicit_and_recorded_with_truth(session):
    session.args.precheck_spawn_x = 1.25
    session.args.precheck_spawn_y = -2.5
    assert session.run() == 0
    stack = next(p for p in session.processes if 'course.launch.py' in p.argv)
    assert 'spawn_x:=1.25' in stack.argv and 'spawn_y:=-2.5' in stack.argv
    report = session.report()
    assert report['requested_precheck_spawn'] == {'x': 1.25, 'y': -2.5}
    assert report['pre_motion_snapshot']['actual_truth_evaluation'] == [0., 0., 0.]


def test_motion_readmission_dispatches_past_frozen_clock_after_audit(session):
    # Cached ready() stays true across simulated synchronous audit work. The
    # motion owner must dispatch a new clock window before entering precheck.
    session.fault = 'post_audit_backlog'
    assert session.run() == 0, session.report().get('error')
    report = session.report()
    admission = report['post_audit_readmission']
    assert session.post_audit_bootstrap_spins >= 6
    assert admission['end_sim'] - admission['start_sim'] > .5
    assert admission['wall_s'] <= 5.
    assert [p['kind'] for p in report['precheck']] == ['forward', 'turn']


def test_motion_readmission_frozen_clock_times_out_without_pulse(session):
    session.fault = 'readmission_clock_freeze'
    assert session.run() == 1
    report = session.report()
    assert report['precheck'] == [] and report['cleanup']['passed']
    assert report['error'] == "TimeoutError('Bounded phase wall deadline')"
    assert not any(any(values) for _, values in session.commands)
    assert report['fault_stop']['request_wall'] < 1008.  # Five-second admission bound unchanged.
    assert session.wall < 1011.1  # Plus at most3s independent-containment wait.


def test_bootstrap_waits_for_observed_free_map_before_native_audit(session,monkeypatch):
    session.fault='map_observation_delayed'
    original=trial.native_snapshot
    def native(*args,**kwargs):
        assert session.monitor.grid['observed_free_present'], 'Unobserved map passed bootstrap'
        return original(*args,**kwargs)
    monkeypatch.setattr(trial,'native_snapshot',native)
    assert session.run()==0,session.report().get('error')
    assert session.map_bootstrap_spins>=10


def test_bootstrap_never_observed_map_times_out_without_commands(session):
    session.fault='map_observation_missing'
    assert session.run()==1
    report=session.report()
    assert report['error']=="TimeoutError('Bounded phase wall deadline')"
    assert report['precheck']==[] and report['cleanup']['passed']
    assert not any(any(v) for _,v in session.commands)
    assert report['fault_stop']['request_wall'] < 1088.
    assert session.wall < 1091.1


def test_route_skips_open_pocket_and_pulses_but_cannot_admit_without_path(session,monkeypatch):
    session.args.precheck_only=False
    session.args.authorize_motion='route'
    session.fault='clearance'  # qualification disk rejects, route must reach its own admission
    calls=[]
    def route_admission(*args,**kwargs):
        calls.append(True)
        raise RuntimeError('Missing validated initial route path')
    monkeypatch.setattr(trial,'route_start_admission',route_admission)
    # Test the real owner until its route startup boundary; no external launch/goal.
    assert session.run()==1
    report=session.report()
    assert calls and report['precheck']==[]
    assert not list(session.output.glob('pocket-admission-*.json'))
    assert not any(any(values) for _,values in session.commands)
    assert session.goals==0 and report['cleanup']['passed']


@pytest.mark.parametrize('reject',[False,True])
def test_planning_only_real_owner_never_launches_navigation_or_sends_nonzero(session,monkeypatch,reject):
    session.args.precheck_only=False;session.args.planning_only=True;session.args.authorize_motion=None
    def plan(node,monitor,output,wait,prepare,**kwargs):
        assert kwargs['planning_only'] is True and kwargs['deferred_capture']=={}
        prepare()
        if reject: raise RuntimeError('Planner unavailable')
        # Even a future successful admission must not fall through into route.
        return dict(planning_completed=True,motion_admitted=False,clearance={'passed':True})
    monkeypatch.setattr(trial,'route_start_admission',plan)
    assert session.run()==(1 if reject else 0),session.report().get('error')
    report=session.report()
    assert report['precheck']==[] and session.goals==0
    assert not any(any(values) for _,values in session.commands)
    assert report['cleanup']['passed'] and all(p.poll() is not None for p in session.processes)
    invocations=[x['name'] for x in report['invocations']]
    assert 'planning-server' in invocations and 'planning-lifecycle' in invocations
    assert 'navigation' not in invocations and 'route' not in report
    assert not any('navigation.launch.py' in p.argv or 'NavigateToPose' in p.argv for p in session.processes)


def test_planning_bulk_evidence_runs_only_after_all_owned_processes_exit(session,monkeypatch):
    session.args.precheck_only=False;session.args.planning_only=True;session.args.authorize_motion=None
    calls=[];real=trial.finish_planning_capture
    def plan(node,monitor,output,wait,prepare,**kwargs):
        prepare()
        kwargs['deferred_capture']['snapshot']=trial.capture_planning_snapshot(monitor)
        return dict(planning_completed=True,motion_admitted=False,clearance='deferred_until_owned_teardown')
    def finish(output,capture):
        assert all(p.poll() is not None for p in session.processes)
        assert session.monitor.events.closed
        calls.append('after_owned_exit')
        return real(output,capture)
    monkeypatch.setattr(trial,'route_start_admission',plan)
    monkeypatch.setattr(trial,'finish_planning_capture',finish)
    assert session.run()==0,session.report().get('error')
    assert calls==['after_owned_exit'] and session.goals==0
    assert session.report()['planning_geometry']['analyzed_after_owned_teardown']
    assert not any(any(values) for _,values in session.commands)


def test_planner_process_startup_finishes_in_existing_zero_bootstrap(session,monkeypatch):
    session.args.precheck_only=False;session.args.planning_only=True;session.args.authorize_motion=None
    original=trial.subprocess.Popen;started=[]
    def popen(argv,**kwargs):
        if 'nav2_planner' in argv or 'nav2_lifecycle_manager' in argv:
            assert session.monitor.phase=='bootstrap'
            started.append(True)
        return original(argv,**kwargs)
    def plan(node,monitor,output,wait,prepare,**kwargs):
        prepare()
        return dict(planning_completed=True,motion_admitted=False)
    monkeypatch.setattr(trial.subprocess,'Popen',popen)
    monkeypatch.setattr(trial,'route_start_admission',plan)
    assert session.run()==0,session.report().get('error')
    assert len(started)==2 and session.report()['planning_lifecycle']['states']==[3]


def test_ingress_cleanup_failure_prevents_success(session):
    session.fault='ingress_cleanup'
    assert session.run()==1
    report=session.report()
    assert not report['success'] and not report['localization_cleanup']['passed']
    assert report['cleanup']['passed'] and all(p.poll() is not None for p in session.processes)


def test_normal_cleanup_interrupts_owned_descendants_of_ros2_run(session, monkeypatch):
    # ros2 run catches a PID-only SIGINT and waits for its unsignaled child.
    # Removing descendant signaling must reproduce the recorded eight-second failure.
    owned = trial.OwnedProcesses
    class WrapperChildren(owned):
        leaf_alive = True
        def scan(self):
            live = super().scan()
            if self.leaf_alive: live[999] = '789'
            self.seen.update(live)
            return live
        def send(self, sig):
            assert sig == trial.signal.SIGINT
            super().send(sig)
            self.leaf_alive = False
    monkeypatch.setattr(trial, 'OwnedProcesses', WrapperChildren)
    assert session.run() == 0
    report = session.report()
    assert report['cleanup']['passed'] and session.cleanup_calls == 0
    assert report['watchdog']['reason'] == 'normal_shutdown'


def test_failed_normal_cleanup_evidence_survives_successful_fallback(session, monkeypatch):
    owned = trial.OwnedProcesses
    class StuckChild(owned):
        def scan(self):
            live = super().scan()
            live[999] = '789'
            self.seen.update(live)
            return live
    monkeypatch.setattr(trial, 'OwnedProcesses', StuckChild)
    assert session.run() == 1
    report = session.report()
    assert report['normal_cleanup']['remaining'] == {'999': '789'}
    assert not report['normal_cleanup']['passed']
    assert report['cleanup']['passed'] and session.cleanup_calls == 1
    assert not report['success']
