#!/usr/bin/python3
"""Canonical warehouse route benchmark; critical by default, dry-run unless --execute."""
import argparse
import copy
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import time

import yaml

from run_narrow_baseline import digest, write

ROOT = Path(__file__).resolve().parents[1]
ROUTE = ROOT / 'course_navigation/config/warehouse_route.yaml'


def load_route():
    config = yaml.safe_load(ROUTE.read_text())
    goals = config['goals']
    if (config['schema_version'] != 1 or config['frame_id'] != 'map'
            or config['critical_segment'] != ['G1', 'G2']
            or [g['name'] for g in goals] != ['G1', 'G2', 'G3', 'G4', 'G5']
            or [g['orientation'] for g in goals] != ['corridor_heading'] * 2 + [None] * 3
            or config['evaluation_tolerance_m'] != .05):
        raise ValueError('Invalid canonical warehouse route contract')
    for goal in goals:
        if len(goal['position']) != 2 or not all(math.isfinite(v) for v in goal['position']):
            raise ValueError('Invalid map XY')
    if goals[0]['position'] == goals[1]['position']:
        raise ValueError('Corridor heading needs distinct G1/G2')
    return config


def critical_goals(config):
    """No route index can reach G3–G5; no randomized or start-relative pose."""
    a, b = config['goals'][:2]
    yaw = math.atan2(b['position'][1] - a['position'][1],
                     b['position'][0] - a['position'][0])
    orientation = dict(x=0., y=0., z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    return [dict(name=g['name'], yaw_rad=yaw, pose={
        'header': {'frame_id': config['frame_id'], 'stamp': {'sec': 0, 'nanosec': 0}},
        'pose': {'position': dict(x=g['position'][0], y=g['position'][1], z=0.),
                 'orientation': dict(orientation)}}) for g in (a, b)]


def planned_goals(config, mode='critical', continuation_yaws=None):
    """Null canonical headings stay unresolved until three explicit yaws are supplied."""
    if mode not in ('critical', 'full'):
        raise ValueError('Invalid warehouse route mode')
    if continuation_yaws is not None:
        if (mode != 'full' or len(continuation_yaws) != 3
                or not all(math.isfinite(v) for v in continuation_yaws)):
            raise ValueError('Full route needs exactly three finite continuation yaws')
    goals = critical_goals(config)
    if mode == 'full':
        for i, goal in enumerate(config['goals'][2:]):
            yaw = None if continuation_yaws is None else continuation_yaws[i]
            goals.append(dict(name=goal['name'], yaw_rad=yaw, pose={
                'header': {'frame_id': config['frame_id'], 'stamp': {'sec': 0, 'nanosec': 0}},
                'pose': {'position': dict(x=goal['position'][0], y=goal['position'][1], z=0.),
                         'orientation': None if yaw is None else dict(
                             x=0., y=0., z=math.sin(yaw / 2), w=math.cos(yaw / 2))}}))
    return goals


def evaluate_goal(goal, metrics, tolerance):
    start, end = metrics.get('start_map_base_link'), metrics.get('end_map_base_link')
    error = None
    if end is not None:
        p = end['transform']['translation']
        g = goal['pose']['pose']['position']
        error = math.hypot(p['x'] - g['x'], p['y'] - g['y'])
        if not math.isfinite(error):
            error = None
    return dict(nav2_result={'status': metrics.get('nav2_status'),
                             'end_reason': metrics['end_reason']},
                navigation_success=metrics.get('nav2_status') == 4 and metrics['success'] is True,
                start_map_base_link=start, end_map_base_link=end,
                endpoint_error_m=error, evaluation_tolerance_m=tolerance,
                evaluation_passed=error is not None and error <= tolerance,
                evidence_complete=start is not None and end is not None and error is not None)


def run_critical(config, *, execute=False, transport_factory=None, checkpoint=lambda report: None,
                 mode='critical', continuation_yaws=None):
    """The sole execution gate precedes transport construction (and ROS imports)."""
    goals = planned_goals(config, mode, continuation_yaws)
    unresolved = [g['name'] for g in goals if g['pose']['pose']['orientation'] is None]
    if execute and unresolved:
        raise ValueError('Unresolved continuation orientations: supply three explicit yaws')
    report = dict(mode=mode, dry_run=not execute, success=False,
                  planned_goals=goals, unresolved_orientations=unresolved, sent_poses=[], segments=[],
                  stop_reason='dry_run', cleanup=dict(passed=True, child_processes=[],
                  ros_initialized=False, session_processes_touched=False))
    if not execute:
        checkpoint(report)
        return report
    transport = None
    try:
        transport = (transport_factory or RosGoalTransport)()
        report['goal_manager_runtime'] = transport.start()
        for i, goal in enumerate(report['planned_goals']):
            previous = 'Start' if i == 0 else report['planned_goals'][i - 1]['name']
            segment = dict(name=goal['name'], segment=f'{previous}→{goal["name"]}')
            report['segments'].append(segment)
            def on_sent(sent, state='published'):
                segment['publish_state'] = state
                if state == 'prepared':
                    segment['prepared_pose'] = copy.deepcopy(sent)
                    checkpoint(report)
                    return
                segment['sent_pose'] = copy.deepcopy(sent)
                report['sent_poses'].append(copy.deepcopy(sent))
                checkpoint(report)
            result = transport.send_and_wait(goal['pose'], on_sent)
            metrics = result['metrics']
            if metrics.get('goal_pose') != segment['sent_pose']:
                raise RuntimeError('Metrics do not match exact sent PoseStamped')
            segment.update(evaluate_goal(goal, metrics, config['evaluation_tolerance_m']))
            segment.update(goal_manager_metrics=metrics, metrics_file=result['metrics_file'],
                           metrics_sha256=result['metrics_sha256'])
            navigation_ok = segment['navigation_success'] and segment['evidence_complete']
            if goal['name'] == 'G1':
                if not navigation_ok:
                    report['stop_reason'] = 'staging_failed'
                    break
                if not segment['evaluation_passed']:
                    report['stop_reason'] = 'staging_evaluation_failed'
                    break
                checkpoint(report)
                continue
            stage = 'critical' if goal['name'] == 'G2' else 'continuation'
            if not navigation_ok or not segment['evaluation_passed']:
                report['stop_reason'] = stage + ('_failed' if not navigation_ok else '_evaluation_failed')
                break
            if i == len(report['planned_goals']) - 1:
                report['success'] = True
                report['stop_reason'] = mode + '_succeeded'
            checkpoint(report)
    except KeyboardInterrupt:
        report.update(success=False, stop_reason='interrupted')
    except Exception as error:
        report.update(success=False, stop_reason='transport_error', error=repr(error))
    finally:
        if transport is not None:
            try:
                report['cleanup'] = transport.close()
            except Exception as error:
                report['cleanup'] = dict(passed=False, error=repr(error))
        if not report['cleanup']['passed']:
            report['success'] = False
        checkpoint(report)
    return report


def find_metrics(directory, sent, previous):
    """Read existing goal_manager files; exact header+pose excludes old/same-XY goals."""
    matches = []
    for path in sorted(directory.glob('20*.json')):
        if path in previous:
            continue
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError):  # Existing writer can still be writing.
            continue
        if data.get('goal_pose') == sent:
            matches.append(dict(metrics=data, metrics_file=str(path), metrics_sha256=digest(path)))
    if len(matches) > 1:
        raise RuntimeError('Ambiguous goal_manager result association')
    return matches[0] if matches else None


class RosGoalTransport:
    """USER-RUN ONLY. Attach to the existing manager; no actions, launch or cmd_vel publisher."""
    def __init__(self, ready_timeout=45., evidence_directory=None, reuse_existing_context=False,
                 progress_callback=lambda: None):
        self.ready_timeout = ready_timeout
        self.node = None
        self.initialized = False
        self.reuse_existing_context = reuse_existing_context
        self.progress_callback = progress_callback
        self.pending = self.foreign_goal = False
        self.expected = None
        self.owned_poses = []
        self.statuses = None
        self.baseline_action_ids = set()
        self.owned_actions = {}
        self.cancel_confirmation_timeout = 3.
        self.evidence = None
        self.latest_map = None
        self.truth_registration_pending = deque(maxlen=32)
        if evidence_directory is not None:
            from warehouse_route_evidence import RouteEvidence
            self.evidence = RouteEvidence(evidence_directory, load_route())

    def start_context(self, rclpy):
        if self.reuse_existing_context:
            if not rclpy.ok():
                raise RuntimeError('Existing ROS context is not initialized')
        else:
            rclpy.init()
            self.initialized = True

    def start(self):
        # Delayed imports keep dry-run completely offline.
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter, parameter_value_to_python
        from geometry_msgs.msg import PoseStamped
        from rcl_interfaces.srv import GetParameters
        from std_srvs.srv import Trigger
        from lifecycle_msgs.srv import GetState
        from course_navigation.lifecycle_readiness import LifecycleReadiness
        from rosidl_runtime_py.convert import message_to_ordereddict
        from tf2_ros import Buffer, TransformListener

        self.rclpy, self.PoseStamped = rclpy, PoseStamped
        self.convert, self.Trigger = message_to_ordereddict, Trigger
        self.start_context(rclpy)
        self.node = Node('warehouse_route_benchmark', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self.node)
        self.publisher = self.node.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cancel = self.node.create_client(Trigger, '/course_navigation/cancel_goal')
        self.attach_monitors()
        try:
            self.attach_map_evidence()
        except Exception as error:
            self.evidence_error(error)
        try:
            self.attach_truth_evidence()
        except Exception as error:
            self.evidence_error(error)
        self.states = [self.node.create_client(GetState, f'/{n}/get_state') for n in
                       ['bt_navigator', 'planner_server', 'controller_server']]
        self.readiness = LifecycleReadiness(self.states)
        client = self.node.create_client(GetParameters, '/goal_manager/get_parameters')
        deadline = time.monotonic() + self.ready_timeout
        while not client.service_is_ready():
            self.spin(deadline)
        names = ['results_directory', 'timeout_s', 'goal_tolerance_m']
        future = client.call_async(GetParameters.Request(names=names))
        while not future.done():
            self.spin(deadline)
        runtime = dict(zip(names, [parameter_value_to_python(v) for v in future.result().values]))
        self.directory = Path(runtime['results_directory'])
        if not self.directory.is_absolute() or not self.directory.is_dir():
            raise RuntimeError('Launch goal_manager with an existing absolute results_directory')
        self.timeout = float(runtime['timeout_s'])
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise RuntimeError('Invalid existing goal_manager timeout')
        # Allow discovery/status replay, then require an idle, fresh existing stack.
        settled = time.monotonic() + 1.
        while time.monotonic() < settled:
            self.spin(deadline)
        self.require_ready(deadline)
        runtime['ros_domain_id'] = os.environ.get('ROS_DOMAIN_ID', '0')
        runtime['concurrency_contract'] = 'Fresh idle warehouse; no other goal producer during this run'
        return runtime

    def attach_map_evidence(self):
        if self.evidence is None:
            return
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        def received(msg):
            self.latest_map = msg  # Cache only; serialize once per executed segment.
        self.node.create_subscription(OccupancyGrid, '/map', received,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

    def evidence_error(self, error):
        if self.evidence is not None:
            message = repr(error)
            if message not in self.evidence.errors and len(self.evidence.errors) < 20:
                self.evidence.errors.append(message)

    def attach_truth_evidence(self):
        if self.evidence is None:
            return
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        def received(msg):
            try:
                pose = self.evidence.sample_truth(self.convert(msg))
                if pose is not None:
                    self.truth_registration_pending.append(pose)
            except Exception as error:
                self.evidence_error(error)
        self.node.create_subscription(Odometry, '/sim/ground_truth/odom', received, qos_profile_sensor_data)

    def pair_truth_evidence(self):
        # Registration reads synchronized TF only, never supplies any TF or goal.
        if self.evidence is None or self.evidence.current is None:
            self.truth_registration_pending.clear()
            return
        from rclpy.time import Time
        from tf2_ros import TransformException
        remaining = deque(maxlen=32)
        for truth in self.truth_registration_pending:
            stamp = truth['header']['stamp']
            try:
                tr = self.tf.lookup_transform('map', 'base_link', Time(
                    seconds=stamp['sec'], nanoseconds=stamp['nanosec']))
                self.evidence.add_registration_pair(truth, dict(header=self.convert(tr.header), pose=dict(
                    position=self.convert(tr.transform.translation), orientation=self.convert(tr.transform.rotation))))
            except TransformException:
                remaining.append(truth)
            except Exception as error:
                self.evidence_error(error)
        self.truth_registration_pending = remaining

    def capture_pose(self):
        if self.evidence is None:
            return
        try:
            now_ns = self.node.get_clock().now().nanoseconds
            if self.evidence.observe_clock(now_ns):
                self.latest_map = None
                self.truth_registration_pending.clear()
        except Exception as error:
            self.evidence_error(error)
            return
        if self.evidence.current is None:
            return
        if self.foreign_goal:
            self.evidence.current['invalid_reason'] = 'Concurrent goal producer; trajectory attribution lost'
            self.evidence.current['truth_invalid_reason'] = 'Concurrent goal producer; trajectory attribution lost'
            return
        from rclpy.time import Time
        from tf2_ros import TransformException
        try:
            tr = self.tf.lookup_transform('map', 'base_link', Time())
            now = self.node.get_clock().now().nanoseconds * 1e-9
            age = now - (tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9)
            if 0 <= age <= .5:
                self.evidence.sample(dict(header=self.convert(tr.header), pose=dict(
                    position=self.convert(tr.transform.translation), orientation=self.convert(tr.transform.rotation))))
        except TransformException:
            pass  # Missing TF becomes a visible path gap, never an interpolated sample.
        except Exception as error:
            self.evidence_error(error)
        try:
            self.pair_truth_evidence()
        except Exception as error:
            self.evidence_error(error)

    def finish_evidence(self):
        if self.evidence is None or self.evidence.current is None:
            return
        try:
            self.capture_pose()
            grid = self.convert(self.latest_map) if self.latest_map is not None else None
            self.evidence.finish(grid, interrupted=self.pending)
        except Exception as error:
            self.evidence_error(error)

    def attach_monitors(self):
        from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
        from action_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import PoseStamped
        def status(msg):
            self.statuses = msg.status_list
            if self.pending:
                for item in self.statuses:
                    key = bytes(item.goal_info.goal_id.uuid).hex()
                    if key not in self.baseline_action_ids:
                        self.owned_actions[key] = item.status
                if len(self.owned_actions) > 1:
                    self.foreign_goal = True  # Concurrent direct-action producer.
        def goal(msg):
            if self.convert(msg) not in self.owned_poses:
                self.foreign_goal = True
        self.node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', status,
                                      qos_profile_action_status_default)
        # Match goal_manager: a BEST_EFFORT external goal can reach it too.
        self.node.create_subscription(PoseStamped, '/goal_pose', goal, qos_profile_sensor_data)

    def spin(self, deadline):
        if not self.rclpy.ok() or time.monotonic() >= deadline:
            raise RuntimeError('ROS shutdown or bounded wait expired')
        if self.foreign_goal:
            raise RuntimeError('Another /goal_pose producer observed; stop without canceling its goal')
        self.rclpy.spin_once(self.node, timeout_sec=.05)
        self.capture_pose()

    def require_ready(self, deadline):
        from rclpy.time import Time
        from tf2_ros import TransformException
        while True:
            if self.foreign_goal:
                raise RuntimeError('Concurrent goal producer')
            managers = [n for n in self.node.get_node_names() if n == 'goal_manager']
            status_publishers = self.node.get_publishers_info_by_topic('/navigate_to_pose/_action/status')
            # With no goals yet, the action server may have no status sample.
            # A fresh idle session and exclusive goal production are prerequisites.
            idle = bool(status_publishers) and not any(s.status in (1, 2, 3) for s in (self.statuses or []))
            fresh = False
            try:
                tf = self.tf.lookup_transform('map', 'base_link', Time())
                now = self.node.get_clock().now().nanoseconds * 1e-9
                age = now - (tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9)
                fresh = now > 0 and 0 <= age <= .5
            except TransformException:
                pass
            if (len(managers) == 1 and idle and fresh and self.publisher.get_subscription_count() >= 2
                    and self.cancel.service_is_ready() and self.readiness.active(time.monotonic())):
                return
            self.spin(deadline)

    def send_and_wait(self, pose, on_sent):
        from rosidl_runtime_py.set_message import set_message_fields
        self.require_ready(time.monotonic() + self.ready_timeout)
        msg = self.PoseStamped()
        set_message_fields(msg, copy.deepcopy(pose))
        msg.header.stamp = self.node.get_clock().now().to_msg()
        sent = self.convert(msg)
        previous = set(self.directory.glob('20*.json'))
        self.expected = sent
        self.baseline_action_ids = {bytes(s.goal_info.goal_id.uuid).hex() for s in (self.statuses or [])}
        self.owned_actions = {}
        # A prepared request is not evidence that publish returned successfully.
        on_sent(sent, 'prepared')
        self.owned_poses.append(sent)
        if self.evidence is not None:
            try:
                self.evidence.begin(sent)
                self.truth_registration_pending.clear()
                self.capture_pose()
            except Exception as error:
                self.evidence_error(error)
        self.pending = True
        try:
            self.publisher.publish(msg)
            on_sent(sent)
            deadline = time.monotonic() + self.timeout * 5 + 10.
            while True:
                self.spin(deadline)
                result = find_metrics(self.directory, sent, previous)
                if result is not None:
                    metrics = result['metrics']
                    # Timeout/clock-reset metrics precede asynchronous cancellation.
                    # Only a terminal action result or rejection discharges ownership.
                    self.pending = (metrics.get('nav2_status') not in (4, 5, 6)
                                    and metrics.get('end_reason') != 'rejected')
                    return result
        finally:
            # On interruption/timeout, cancellation takes priority over map IO.
            # close() persists the remaining capture after cancellation handling.
            if not self.pending:
                self.finish_evidence()

    def close(self):
        passed = True
        cancel_outcome = 'no_pending_goal'
        try:
            if self.node is not None and self.pending:
                if self.foreign_goal:
                    cancel_outcome = 'foreign_goal_not_canceled'
                    passed = False
                elif self.rclpy.ok() and self.cancel.service_is_ready():
                    future = self.cancel.call_async(self.Trigger.Request())
                    until = time.monotonic()+3.
                    while not future.done() and self.rclpy.ok() and time.monotonic()<until:
                        self.rclpy.spin_once(self.node, timeout_sec=.05)
                        self.progress_callback()
                    acknowledged = future.done() and future.exception() is None
                    deadline = time.monotonic() + self.cancel_confirmation_timeout
                    passed = False
                    while acknowledged and self.rclpy.ok() and time.monotonic() < deadline:
                        self.rclpy.spin_once(self.node, timeout_sec=.05)
                        self.progress_callback()
                        if self.foreign_goal:
                            break
                        if self.owned_actions and all(s in (4, 5, 6) for s in self.owned_actions.values()):
                            passed = True
                            break
                    cancel_outcome = 'owned_action_terminal' if passed else 'manager_cancel_unconfirmed'
                else:
                    passed, cancel_outcome = False, 'manager_cancel_unavailable'
        finally:
            self.finish_evidence()
            if self.evidence is not None:
                try:
                    self.evidence.save()
                except Exception as error:
                    self.evidence_error(error)
            if self.node is not None:
                self.node.destroy_node()
            if self.initialized and self.rclpy.ok():
                self.rclpy.shutdown()
        return dict(passed=passed, cancel_outcome=cancel_outcome, child_processes=[],
                    session_processes_touched=False, ros_resources_destroyed=True,
                    owned_action_statuses=self.owned_actions)


def manifest():
    config_paths = sorted((ROOT / 'course_navigation/config').glob('*.yaml'))
    config_paths += sorted((ROOT / 'course_slam/config').glob('*.yaml'))
    result = dict(route_path=str(ROUTE), route_sha256=digest(ROUTE),
                  head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                  config_sha256={str(p.relative_to(ROOT)): digest(p) for p in config_paths},
                  source_sha256={str(p.relative_to(ROOT)): digest(p) for p in
                      [Path(__file__), ROOT / 'course_navigation/course_navigation/goal_manager.py',
                       ROOT / 'scripts/warehouse_route_evidence.py', ROOT / 'scripts/plot_warehouse_route.py',
                       ROOT / 'scripts/warehouse_truth_floorplan.py', ROOT / 'scripts/warehouse_truth_overlay.py']},
                  config_scope='On-disk artifacts; not proof of loaded Nav2 runtime configuration',
                  map_reproducibility='unverified_across_fresh_startups')
    world = ROOT / 'third_party/aws_warehouse/warehouse.world'
    result.update(warehouse_world_sha256=None, warehouse_world_geometry_sha256=None)
    # Geometry identity includes mesh bytes: unchanged SDF URIs alone do not
    # authenticate a canonical floorplan against locally edited assets.
    result['warehouse_collision_mesh_sha256'] = {}
    try:
        from warehouse_truth_floorplan import world_geometry_sha256
        if world.is_file():
            result['warehouse_world_sha256'] = digest(world)
            result['warehouse_world_geometry_sha256'] = world_geometry_sha256(world)
            import xml.etree.ElementTree as ET
            for uri in ET.parse(world).findall('.//collision/geometry/mesh/uri'):
                mesh = Path(uri.text.removeprefix('file://')).resolve()
                if not mesh.is_relative_to(ROOT / 'third_party/aws_warehouse/models'):
                    continue  # Unsupported source cannot match the canonical identity.
                result['warehouse_collision_mesh_sha256'][str(mesh.relative_to(ROOT))] = digest(mesh) if mesh.is_file() else None
    except Exception as error:
        result['warehouse_world_geometry_sha256'] = None
        result['warehouse_geometry_error'] = repr(error)  # Evaluation availability only.
    return result


def main(argv=None, *, transport_factory=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['critical', 'full'], default='critical')
    parser.add_argument('--continuation-yaws', type=float, nargs=3, metavar=('G3', 'G4', 'G5'),
                        help='Explicit map-frame radians for unresolved G3–G5 orientations in full mode')
    parser.add_argument('--execute', action='store_true',
                        help='USER-RUN ONLY: send selected route through existing goal_manager')
    parser.add_argument('--output-root', type=Path, default=Path('/tmp/b2w-warehouse-route'))
    parser.add_argument('--ready-timeout', type=float, default=45., help='Bounded attach/readiness wall seconds')
    args = parser.parse_args(argv)
    if not math.isfinite(args.ready_timeout) or args.ready_timeout <= 0:
        parser.error('--ready-timeout must be positive and finite')
    config = load_route()
    try:
        goals = planned_goals(config, args.mode, args.continuation_yaws)
        if args.execute and any(g['yaw_rad'] is None for g in goals):
            raise ValueError('Full execution needs --continuation-yaws G3 G4 G5')
    except ValueError as error:
        parser.error(str(error))
    provenance = manifest()
    provenance['route_id'] = config['route_id']
    output = args.output_root.resolve() / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    output.mkdir(parents=True, exist_ok=False)
    write(output / 'route-snapshot.json', config)
    def checkpoint(report):
        report['manifest'] = provenance
        temporary = output / 'report.json.tmp'
        write(temporary, report)
        temporary.replace(output / 'report.json')
    report = run_critical(config, execute=args.execute, checkpoint=checkpoint,
                          mode=args.mode, continuation_yaws=args.continuation_yaws,
                          transport_factory=transport_factory or (lambda: RosGoalTransport(args.ready_timeout, output)))
    # Rendering is offline, after run_critical's transport.close(); never changes
    # navigation success, staging admission or cleanup outcomes.
    if args.execute:
        try:
            from plot_warehouse_route import render_run
            overlays = render_run(output)
            report['visualization'] = dict(status='generated' if overlays['primary']['status'] == 'generated' or any(
                i['status'] == 'generated' for i in overlays['images']) else 'unavailable',
                report_file=str(output / 'overlays/trajectory-overlays.json'), images=overlays['images'],
                main_artifact=overlays['composite_image_path'], primary_status=overlays['primary']['status'])
        except Exception as error:
            report['visualization'] = dict(status='error', error=repr(error), images=[])
    else:
        report['visualization'] = dict(status='dry_run', images=[])
    checkpoint(report)
    print(f'{report["stop_reason"]}: {output / "report.json"}')
    return 0 if report['dry_run'] or report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
