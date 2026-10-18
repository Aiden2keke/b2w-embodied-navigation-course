#!/usr/bin/env python3
"""Run or replay an isolated, bounded B2-W open-space velocity experiment."""
from __future__ import annotations

import argparse
import bisect
import csv
import datetime as dt
import hashlib
import json
import math
import os
import re
import secrets
import signal
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.velocity_step_core import DEFAULT_SPEEDS, analyze_run, make_sequence, requested_at

SOURCE_WORLD = ROOT / 'course_sim/worlds/confined.world'
NAV_CONFIG = ROOT / 'course_navigation/config/nav2.yaml'
POLICY = ROOT / 'third_party/b2w_rl_sar/policy/b2w/robot_lab/policy.onnx'
RAW_TOPIC = '/course_navigation/cmd_vel_raw'
SHARED_TOPIC = '/course_navigation/cmd_vel'
FINAL_TOPIC = '/path_manager/path_manager_ros/nav_vel'
TRUTH_TOPIC = '/sim/ground_truth/odom'
SLAM_TOPIC = '/slam/odom'
REQUEST_TOPIC = '/harness/requested'
FLUSH_S = 1.2
READINESS_TIMEOUT_S = 120.
RUN_WALL_TIMEOUT_S = 240.
CLOCK_FREEZE_TIMEOUT_S = 2.
TILT_LIMIT_RAD = math.radians(25.)
READINESS_KEYS = ('clock_advancing','clock_epoch_stable','controller_alive','startup_stand_and_policy_warmup',
    'fresh_private_joint_state','fresh_private_imu','fresh_slam_odom','fresh_ground_truth_instrumentation',
    'truth_twist_child_frame_base_link','slam_twist_child_frame_base_link','valid_map_to_base_tf',
    'velocity_smoother_active','velocity_adapter_alive','final_command_fresh_and_zero',
    'expected_publishers_only','required_topics_discovered','all_feedback_fresh','recorder_receiving',
    'unique_clock_publisher','no_navigation_command_nodes')
JOINT_NAMES = ['FR_hip_joint','FR_thigh_joint','FR_calf_joint','FL_hip_joint','FL_thigh_joint','FL_calf_joint',
               'RR_hip_joint','RR_thigh_joint','RR_calf_joint','RL_hip_joint','RL_thigh_joint','RL_calf_joint',
               'FR_foot_joint','FL_foot_joint','RR_foot_joint','RL_foot_joint']


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def request_rate_hz(nav_config: Path = NAV_CONFIG) -> float:
    text = nav_config.read_text(encoding='utf-8')
    block = re.search(r'(?ms)^controller_server:\s*\n(.*?)(?=^[^ \t#][^\n]*:|\Z)', text)
    if not block:
        raise ValueError('baseline controller_server block is missing')
    match = re.search(r'(?m)^\s+controller_frequency:\s*([0-9.]+)\s*$', block.group(1))
    smoother = re.search(r'(?ms)^velocity_smoother:\s*\n(.*)\Z', text)
    smooth_match = re.search(r'(?m)^\s+smoothing_frequency:\s*([0-9.]+)\s*$', smoother.group(1)) if smoother else None
    if not match or not smooth_match:
        raise ValueError('cannot derive requested cadence from baseline YAML')
    controller_hz, smoother_hz = float(match.group(1)), float(smooth_match.group(1))
    if controller_hz != smoother_hz or controller_hz <= 0:
        raise ValueError(f'controller/smoother baseline frequencies disagree: {controller_hz}/{smoother_hz}')
    return controller_hz


def readiness_complete(snapshot: dict) -> bool:
    """Require every instrumentation and command-path gate before motion."""
    return all(snapshot.get(key) is True for key in READINESS_KEYS)


def publisher_contract_ok(audit: dict) -> bool:
    return bool(audit) and all(value.get('matches') is True for value in audit.values())


def clock_rolled_back(previous: float | None, current: float) -> bool:
    return previous is not None and current < previous


LANDMARKS = [
    {'name': 'landmark_west_north', 'center_m': [-6.3, 3.25, .85], 'size_m': [.35, .55, 1.7]},
    {'name': 'landmark_mid_north', 'center_m': [-1.8, 4.35, .65], 'size_m': [.85, .45, 1.3]},
    {'name': 'landmark_east_north', 'center_m': [4.7, 3.15, .95], 'size_m': [.4, .7, 1.9]},
    {'name': 'landmark_west_south', 'center_m': [-5.1, -4.2, .75], 'size_m': [.7, .35, 1.5]},
    {'name': 'landmark_mid_south', 'center_m': [.65, -4.75, .9], 'size_m': [.45, .8, 1.8]},
    {'name': 'landmark_east_south', 'center_m': [6.2, -3.45, .6], 'size_m': [.95, .4, 1.2]},
]
CLEAR_STRIP = {'x_min_m': -5., 'x_max_m': 5., 'y_min_m': -1.15, 'y_max_m': 1.15,
               'description': 'reserved obstacle-free central swept-path envelope; robot starts x=-4m and faces +x'}


def _sub(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def generate_fixture_world(source: Path, generated: Path) -> dict:
    """Keep source world systems/floor semantics and add asymmetric static boxes."""
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError('source world has no <world>')
    retained = []
    for node in list(world):
        if node.tag in ('physics', 'plugin', 'light'):
            retained.append(node)
        elif node.tag == 'model' and node.get('name') == 'floor':
            retained.append(node)
    for node in list(world):
        world.remove(node)
    for node in retained:
        world.append(node)
    floor = world.find("model[@name='floor']/link/visual/material/diffuse")
    floor_size = world.find("model[@name='floor']/link/collision/geometry/box/size")
    floor_pose = world.find("model[@name='floor']/pose")
    if floor is None or floor_size is None or floor_pose is None:
        raise ValueError('source floor lacks required collision, visual material or pose')
    boundary = [
        ('boundary_north', [0., 7.85, 1.], [19.6, .3, 2.]),
        ('boundary_south', [0., -7.85, 1.], [19.6, .3, 2.]),
        ('boundary_west', [-9.85, 0., 1.], [.3, 15.7, 2.]),
        ('boundary_east', [9.85, 0., 1.], [.3, 15.7, 2.]),
    ]
    geometry = []
    for name, center, size in [*boundary, *((item['name'], item['center_m'], item['size_m']) for item in LANDMARKS)]:
        model = _sub(world, 'model', name=name)
        _sub(model, 'static', 'true')
        _sub(model, 'pose', ' '.join(map(str, [*center, 0., 0., 0.])))
        link = _sub(model, 'link', name='link')
        for tag in ('collision', 'visual'):
            obj = _sub(link, tag, name=tag)
            _sub(_sub(_sub(obj, 'geometry'), 'box'), 'size', ' '.join(map(str, size)))
            if tag == 'visual':
                _sub(_sub(obj, 'material'), 'diffuse', floor.text)
        geometry.append({'name': name, 'center_m': center, 'size_m': size,
                         'purpose': 'world boundary' if name.startswith('boundary_') else 'static LiDAR observability landmark only'})
    ET.indent(tree, space='  ')
    generated.parent.mkdir(parents=True, exist_ok=True)
    tree.write(generated, encoding='utf-8', xml_declaration=True)
    for item in LANDMARKS:
        x, y, _ = item['center_m']; sx, sy, _ = item['size_m']
        if not (x+sx/2 < CLEAR_STRIP['x_min_m'] or x-sx/2 > CLEAR_STRIP['x_max_m'] or
                y+sy/2 < CLEAR_STRIP['y_min_m'] or y-sy/2 > CLEAR_STRIP['y_max_m']):
            raise ValueError(f"landmark intersects reserved strip: {item['name']}")
    return {'source_world': str(source.resolve()), 'source_sha256': sha256(source),
            'generated_world': str(generated.resolve()), 'generated_sha256': sha256(generated),
            'physics': {'max_step_size_s': float(world.find('physics/max_step_size').text),
                        'real_time_factor': float(world.find('physics/real_time_factor').text)},
            'floor': {'pose': floor_pose.text, 'box_size_m': floor_size.text,
                      'diffuse_rgba': floor.text},
            'spawn': {'x_m': -4., 'y_m': 0., 'z_m': 1., 'yaw_rad': 0., 'heading': '+x'},
            'clear_strip': CLEAR_STRIP, 'geometry': geometry,
            'landmarks_are_control_inputs': False,
            'ground_truth_usage': 'evaluation_only; never used for readiness values, phase timing, command selection, correction or safety control'}


def build_manifest(run_dir: Path, domain: int, token: str, gui: bool, fixture: dict, speeds=DEFAULT_SPEEDS, *, transition_diagnostic=False) -> dict:
    rate = request_rate_hz()
    smoother_rate = float(re.search(r'(?m)^\s+smoothing_frequency:\s*([0-9.]+)\s*$', NAV_CONFIG.read_text()).group(1))
    return {'schema_version': 1, 'run_directory': str(run_dir.resolve()),
            'source_revision': _git_head(), 'branch': _git_branch(),
            'ros_domain_id': domain, 'ign_partition': f'b2w-velocity-step-{token}',
            'owner_token_sha256': hashlib.sha256(token.encode()).hexdigest(), 'gui': gui,
            'requested_rate_hz': rate, 'rate_basis': 'unchanged course_navigation/config/nav2.yaml controller_frequency; verified against unchanged smoother frequency',
            'smoother_rate_hz': smoother_rate, 'sequence': make_sequence(speeds, transition_diagnostic=transition_diagnostic),
            'sequence_duration_sim_s': make_sequence(speeds, transition_diagnostic=transition_diagnostic)[-1]['end_s'], 'requested_topic': REQUEST_TOPIC,
            'raw_topic': RAW_TOPIC, 'shared_topic': SHARED_TOPIC, 'final_topic': FINAL_TOPIC,
            'truth_topic': TRUTH_TOPIC, 'slam_topic': SLAM_TOPIC,
            'executor':'rclpy SingleThreadedExecutor; runner and recorder registered for entire session',
            'clock_freeze_wall_limit_s': CLOCK_FREEZE_TIMEOUT_S,
            'readiness_wall_timeout_s': READINESS_TIMEOUT_S, 'run_wall_timeout_s': RUN_WALL_TIMEOUT_S,
            'zero_flush_wall_s': FLUSH_S, 'tilt_limit_rad': TILT_LIMIT_RAD,
            'max_gap_s': {'truth': .15, 'slam': .35, 'command': max(.075, 1.5/rate)},
            'source_hashes': {str(path.relative_to(ROOT)): sha256(path) for path in [
                NAV_CONFIG, ROOT/'course_sim/config/rl_sar.yaml', ROOT/'b2w_sim/b2w_controllers/src/rl_sar_controller.cpp',
                ROOT/'course_navigation/course_navigation/velocity_adapter.py',ROOT/'course_navigation/launch/navigation.launch.py',
                ROOT/'course_bringup/launch/course.launch.py',ROOT/'course_sim/launch/simulation.launch.py',
                ROOT/'course_sim/course_sim/description.py',ROOT/'course_sim/config/bridge_rl_sar.yaml',
                ROOT/'course_sim/config/sensors.yaml',ROOT/'course_sim/config/fault_profiles.yaml',
                ROOT/'course_slam/config/lio_sam.yaml',ROOT/'course_slam/config/adapter.yaml',
                ROOT/'b2w_sim/b2w_description_ros2/urdf/b2w_description_rsl.urdf',
                ROOT/'third_party/b2w_rl_sar/POLICY_SOURCE.json',
                ROOT/'third_party/b2w_rl_sar/policy/b2w/base.yaml',
                ROOT/'third_party/b2w_rl_sar/policy/b2w/robot_lab/config.yaml',
                ROOT/'third_party/b2w_rl_sar/robot_assets/b2w_description/urdf/b2w_description.urdf',POLICY]},
            'policy_path': str(POLICY.resolve()), 'policy_sha256': sha256(POLICY),
            'tested_chain': [REQUEST_TOPIC, RAW_TOPIC, SHARED_TOPIC, FINAL_TOPIC,
                             'b2w_rl_sar_controller', 'ONNX policy', 'leg PD/wheel velocity-error torque', 'Gazebo'],
            'fixture': fixture,
            'optional_streams': {'roll_pitch': {'available': None, 'reason': None},
                                 'wheel_velocity': {'available': None, 'reason': None}},
            'processes': [], 'readiness': {}, 'termination': None}


def _git_head():
    try:
        return subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True, timeout=3).stdout.strip() or None
    except Exception:
        return None


def _git_branch():
    try:
        return subprocess.run(['git', 'branch', '--show-current'], cwd=ROOT, capture_output=True, text=True, timeout=3).stdout.strip() or None
    except Exception:
        return None


def _write_json(path: Path, obj):
    path.write_text(json.dumps(obj, indent=2, sort_keys=True, allow_nan=False) + '\n', encoding='utf-8')


def _read_events(path: Path):
    with path.open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def _index_events(events):
    indexed={}
    for event in events:
        if not event.get('valid',True): continue
        for source_time,key in ((False,'sim_s'),(True,'source_s')):
            stamp=event.get(key)
            if stamp is not None:
                indexed.setdefault((event.get('topic'),source_time),[]).append((stamp,event))
    result={}
    for key,rows in indexed.items():
        rows.sort(key=lambda item:item[0])
        result[key]=([stamp for stamp,_ in rows],[event for _,event in rows])
    return result


def _latest(indexed, topic, t, *, source_time=False):
    stamps,rows=indexed.get((topic,source_time),([],[]))
    i=bisect.bisect_right(stamps,t)-1
    return rows[i] if i>=0 else None


def speed_filename(speed):
    label=f'{speed:.2f}' if speed == round(speed,2) else repr(float(speed))
    return f'speed-{label}.json'


def _write_outputs(run_dir: Path, manifest: dict, events: list[dict], analysis=None):
    analysis = analysis or analyze_run(events, manifest)
    (run_dir/'per-speed').mkdir(exist_ok=True)
    for row in analysis['per_speed']:
        filename=speed_filename(row['requested_vx_mps'])
        if manifest.get('mode')=='route_low_speed_transition':filename=f"trial-{row['trial']:02d}-{filename}"
        _write_json(run_dir/'per-speed'/filename, row)
    _write_json(run_dir/'summary.json', analysis)
    columns = ['requested_vx_mps', 'truth_steady_vx_mps', 'truth_gain', 'truth_bias_mps',
               'slam_steady_vx_mps', 'rise_time_10_90_s', 'truth_peak_vx_mps',
               'overshoot_requested_pct', 'overshoot_tail_pct', 'settling_time_s',
               'truth_stop_s', 'stop_time_s', 'stop_path_distance_m', 'stop_net_displacement_m',
               'target_lateral_m', 'target_yaw_drift_rad', 'target_peak_abs_vy_mps', 'target_peak_abs_wz_radps',
               'raw_plateau_vx_mps', 'shared_plateau_vx_mps', 'final_plateau_vx_mps',
               'raw_to_shared_zero_s', 'shared_to_final_zero_s']
    with (run_dir/'summary.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns); writer.writeheader()
        for row in analysis['per_speed']:
            truth, slam, stop, pipe = row['truth'], row['slam'], row['stop'], row['pipeline']
            target = row['coupling']['target_phase']
            writer.writerow({'requested_vx_mps': row['requested_vx_mps'],
                'truth_steady_vx_mps': truth['steady_state_mean_vx_mps'], 'truth_gain': truth['gain_actual_over_requested'],
                'truth_bias_mps': truth['bias_signed_mps'], 'slam_steady_vx_mps': slam['steady_state_mean_vx_mps'],
                'rise_time_10_90_s': truth['rise_time_10_90_s'], 'truth_peak_vx_mps': truth['peak_vx_mps'],
                'overshoot_requested_pct': truth['overshoot_requested_pct'], 'overshoot_tail_pct': truth['overshoot_tail_pct'],
                'settling_time_s': truth['settling_time_s'], 'truth_stop_s': stop['truth_effective_stop_s'],
                'stop_time_s': stop['requested_zero_to_truth_stop_s'], 'stop_path_distance_m': stop['stop_path_distance_m'],
                'stop_net_displacement_m': stop['stop_net_displacement_m'], 'target_lateral_m': target['lateral_displacement_m'],
                'target_yaw_drift_rad': target['yaw_drift_rad'], 'target_peak_abs_vy_mps': target['peak_abs_vy_mps'],
                'target_peak_abs_wz_radps': target['peak_abs_wz_radps'], 'raw_plateau_vx_mps': pipe['raw_plateau_vx_mps'],
                'shared_plateau_vx_mps': pipe['shared_plateau_vx_mps'], 'final_plateau_vx_mps': pipe['final_plateau_vx_mps'],
                'raw_to_shared_zero_s': pipe['raw_to_shared_zero_s'], 'shared_to_final_zero_s': pipe['shared_to_final_zero_s']})
    event_index=_index_events(events)
    start = manifest.get('experiment_start_sim_s')
    grid_dt = .05
    headers=['sim_s','elapsed_s','trial','phase','requested_vx_mps',
        'raw_vx_mps','raw_receive_age_s','shared_vx_mps','shared_receive_age_s','final_nav_vx_mps','final_receive_age_s',
        'truth_header_s','truth_receive_age_s','truth_source_age_s','truth_valid','truth_vx_mps','truth_vy_mps','truth_wz_radps','truth_x_m','truth_y_m','truth_yaw_rad',
        'slam_header_s','slam_receive_age_s','slam_source_age_s','slam_valid','slam_vx_mps','slam_vy_mps','slam_wz_radps','slam_x_m','slam_y_m','slam_yaw_rad']
    with (run_dir/'timeseries.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=headers); writer.writeheader()
        if start is not None:
            for i in range(round(manifest['sequence'][-1]['end_s']/grid_dt)):
                elapsed=i*grid_dt; now=start+elapsed; phase=next((p for p in manifest['sequence'] if p['start_s']<=elapsed<p['end_s']),None)
                sample={'sim_s':now,'elapsed_s':elapsed,'trial':phase['trial'] if phase else None,
                        'phase':phase['phase'] if phase else 'complete'}
                request_event=_latest(event_index,REQUEST_TOPIC,now)
                request_age=now-request_event['sim_s'] if request_event and request_event.get('sim_s') is not None else None
                sample['requested_vx_mps']=(request_event or {}).get('data',{}).get('linear_x') if request_age is not None and request_age<=manifest.get('max_gap_s',{}).get('command',.15) else None
                for name,topic in [('raw',RAW_TOPIC),('shared',SHARED_TOPIC),('final',FINAL_TOPIC)]:
                    e=_latest(event_index,topic,now)
                    column={'raw':'raw_vx_mps','shared':'shared_vx_mps','final':'final_nav_vx_mps'}[name]
                    age=now-e['sim_s'] if e and e.get('sim_s') is not None else None
                    sample[column]=(e or {}).get('data',{}).get('linear_x') if age is not None and age<=manifest.get('max_gap_s',{}).get('command',.15) else None
                    sample[f'{name}_receive_age_s']=age
                for name,topic in [('truth',TRUTH_TOPIC),('slam',SLAM_TOPIC)]:
                    e=_latest(event_index,topic,now,source_time=True); data=(e or {}).get('data',{})
                    sample[f'{name}_header_s']=(e or {}).get('source_s')
                    receive_age=now-e['sim_s'] if e and e.get('sim_s') is not None else None
                    source_age=now-e['source_s'] if e and e.get('source_s') is not None else None
                    gap=manifest.get('max_gap_s',{}).get(name,.15 if name=='truth' else .35)
                    valid=bool(e and e.get('valid',True) and receive_age is not None and receive_age<=gap and source_age is not None and source_age<=gap)
                    sample[f'{name}_valid']=valid
                    sample[f'{name}_receive_age_s']=receive_age
                    sample[f'{name}_source_age_s']=source_age
                    if not valid:data={}
                    lin=data.get('linear') or [None]*3; ang=data.get('angular') or [None]*3; pos=data.get('position') or [None]*3; q=data.get('quaternion_xyzw') or [None]*4
                    sample.update({f'{name}_vx_mps':lin[0],f'{name}_vy_mps':lin[1],f'{name}_wz_radps':ang[2],f'{name}_x_m':pos[0],f'{name}_y_m':pos[1]})
                    try:
                        sample[f'{name}_yaw_rad']=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]))
                    except (TypeError,IndexError): sample[f'{name}_yaw_rad']=None
                writer.writerow(sample)
    with (run_dir/'summary.md').open('w', encoding='utf-8') as stream:
        stream.write('# Velocity step response summary\n\n')
        stream.write('Derived offline from native `events.jsonl`. Ground truth and SLAM are separate; missing/censored values remain null.\n\n')
        stream.write('| Requested vx (m/s) | Truth steady (m/s) | Truth/request gain | Rise 10–90% (s) | Peak (m/s) | Stop time (s) | Stop path (m) | Truth steady valid |\n|---:|---:|---:|---:|---:|---:|---:|:---:|\n')
        for row in analysis['per_speed']:
            truth, stop = row['truth'], row['stop']
            values=[row['requested_vx_mps'],truth['steady_state_mean_vx_mps'],truth['gain_actual_over_requested'],truth['rise_time_10_90_s'],truth['peak_vx_mps'],stop['requested_zero_to_truth_stop_s'],stop['stop_path_distance_m'],truth['steady_state_valid']]
            stream.write('| '+' | '.join('null' if x is None else str(x) for x in values)+' |\n')
        stream.write('\nClock rollback/epoch crossing, missing streams, sample gaps, and unsupported windows are in `summary.json`. One run does not identify a unique cause.\n')


def _analyze_dir(run_dir: Path):
    raw=run_dir/'events.jsonl'; manifest_path=run_dir/'manifest.json'
    if not raw.is_file() or not manifest_path.is_file():
        raise ValueError('--analyze requires manifest.json and events.jsonl in RUN_DIR')
    before=sha256(raw); manifest=json.loads(manifest_path.read_text(encoding='utf-8')); events=_read_events(raw)
    analysis=analyze_run(events,manifest); _write_outputs(run_dir,manifest,events,analysis)
    if sha256(raw)!=before: raise RuntimeError('offline analysis modified raw events.jsonl')
    print(json.dumps({'analyzed':str(run_dir.resolve()),'event_count':len(events),'raw_sha256_unchanged':before},indent=2))
    return 0


def _dry_run(gui, speeds=DEFAULT_SPEEDS):
    print(json.dumps({'dry_run':True,'ros_imported':False,'children_launched':False,'motion_published':False,
        'request_hz':request_rate_hz(),'sequence':make_sequence(speeds),'duration_sim_s':make_sequence(speeds)[-1]['end_s'],
        'tested_chain':['requested Twist','/course_navigation/cmd_vel_raw','unchanged nav2_velocity_smoother',
                        '/course_navigation/cmd_vel','unchanged velocity_adapter',FINAL_TOPIC,'rl_sar','Gazebo'],
        'fixture_source_sha256':sha256(SOURCE_WORLD),'gui':bool(gui)},indent=2,allow_nan=False))
    return 0


class _OwnedLaunches:
    def __init__(self, run_dir, token, env):
        self.run_dir=run_dir; self.token=token; self.base_env=env; self.processes=[]; self.owner=None; self.records=[]

    def start(self, label, args, env=None):
        child_env=dict(self.base_env); child_env.update(env or {}); child_env['B2W_NARROW_OWNER']=self.token
        logfile=(self.run_dir/f'{label}.log').open('wb')
        try:process=subprocess.Popen(args,cwd=ROOT,env=child_env,stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
        except Exception:
            logfile.close()
            raise
        self.processes.append((label,process,logfile))
        self.records.append({'label':label,'pid':process.pid,'argv':list(args),'owner_token_environment':'B2W_NARROW_OWNER'})
        return process

    def cleanup(self):
        from scripts.narrow_processes import OwnedProcesses
        if not self.processes: return {'passed':True,'remaining':{}}
        self.owner=OwnedProcesses(self.token)
        initial=self.owner.scan()
        for pid,identity in initial.items():
            try:
                stat=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
                env=Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
                if stat[19]==identity and self.owner.marker in env:os.kill(pid,signal.SIGINT)
            except (ProcessLookupError,FileNotFoundError,OSError,IndexError):pass
        deadline=time.monotonic()+10.
        while time.monotonic()<deadline and self.owner.scan():time.sleep(.1)
        remaining=self.owner.scan()
        if remaining:
            self.owner.send(signal.SIGTERM)
            deadline=time.monotonic()+5.
            while time.monotonic()<deadline and self.owner.scan():time.sleep(.1)
        remaining=self.owner.scan()
        if remaining:
            self.owner.send(signal.SIGKILL)
            deadline=time.monotonic()+3.
            while time.monotonic()<deadline and self.owner.scan():time.sleep(.1)
        remaining=self.owner.scan()
        for _,process,_ in self.processes:process.poll()
        for _,_,logfile in self.processes:
            try: logfile.close()
            except OSError: pass
        return {'passed':not remaining,'signal_order':['INT','bounded_wait_10s','TERM_if_owned_survivors','bounded_wait_5s','KILL_if_owned_survivors','bounded_wait_3s'],
                'owned_pid_start_ticks':self.owner.seen,'remaining':remaining,
                'launch_returncodes':{label:process.poll() for label,process,_ in self.processes}}


def _make_ros_node(directory, recorder, speeds=DEFAULT_SPEEDS, *, transition_diagnostic=False):
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rosgraph_msgs.msg import Clock as ClockMessage
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from lifecycle_msgs.srv import GetState
    from tf2_ros import Buffer, TransformListener, TransformException

    class HarnessNode(Node):
        def __init__(self):
            super().__init__('velocity_step_runner')
            self.recorder=recorder; self.clock_s=None; self.last_clock_wall=None; self.clock_epoch=0; self.clock_prev=None
            self.last={}; self.final_zero_since=None; self.first_joint_sim=None; self.controller_debug_sim=None
            self.clock_rollback=False
            self.latest_imu=None; self.latest_final=None; self.experiment_start=None; self.run_start_wall=None
            self.abort_reason=None; self.complete=False; self.sequence=make_sequence(speeds, transition_diagnostic=transition_diagnostic); self.last_audit_wall=0.
            self.zero_only=True; self.clock_advances=0
            self.lifecycle_active=False; self.lifecycle_client=self.create_client(GetState,'/velocity_smoother/get_state')
            self.lifecycle_future=None; self.tf=Buffer(); self.tf_listener=TransformListener(self.tf,self)
            self.create_subscription(ClockMessage,'/clock',self.clock_cb,qos_profile_sensor_data)
            self.create_subscription(Odometry,TRUTH_TOPIC,lambda m:self.odom_cb(TRUTH_TOPIC,m),10)
            self.create_subscription(Odometry,SLAM_TOPIC,lambda m:self.odom_cb(SLAM_TOPIC,m),10)
            self.create_subscription(Imu,'/sim/raw/imu',self.imu_cb,qos_profile_sensor_data)
            self.create_subscription(__import__('sensor_msgs.msg',fromlist=['JointState']).JointState,'/sim/raw/joint_states',self.joint_cb,qos_profile_sensor_data)
            self.create_subscription(Twist,FINAL_TOPIC,self.final_cb,10)
            self.create_subscription(__import__('std_msgs.msg',fromlist=['Float64MultiArray']).Float64MultiArray,'/network_input_debug',self.debug_cb,10)
            self.timer=self.create_timer(1./request_rate_hz(),self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

        def clock_cb(self,msg):
            stamp=msg.clock.sec+msg.clock.nanosec*1e-9; wall=time.monotonic()
            if clock_rolled_back(self.clock_prev,stamp):
                self.clock_rollback=True; self.abort_reason='simulation clock rollback/epoch crossing'
            if self.clock_s != stamp:
                self.last_clock_wall=wall
                if self.clock_s is not None: self.clock_advances+=1
            self.clock_prev=stamp; self.clock_s=stamp

        def odom_cb(self,topic,msg):
            valid=None
            if topic==SLAM_TOPIC:
                p,q=msg.pose.pose.position,msg.pose.pose.orientation
                lv,av=msg.twist.twist.linear,msg.twist.twist.angular
                values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,lv.x,lv.y,lv.z,av.x,av.y,av.z]
                valid=all(math.isfinite(v) for v in values) and q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w>.5
            self.last[topic]={'wall':time.monotonic(),'sim':self.clock_s,'source':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                              'frame_id':msg.header.frame_id,'child_frame_id':msg.child_frame_id,
                              'valid':valid}

        def joint_cb(self,msg):
            indices={name:i for i,name in enumerate(msg.name)}
            valid=(all(name in indices for name in JOINT_NAMES) and len(indices)==len(msg.name) and
                   all(indices[name]<len(msg.position) and indices[name]<len(msg.velocity) and
                       math.isfinite(msg.position[indices[name]]) and math.isfinite(msg.velocity[indices[name]]) for name in JOINT_NAMES))
            self.last['/sim/raw/joint_states']={'wall':time.monotonic(),'sim':self.clock_s,
                'source':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'valid':valid}
            if self.first_joint_sim is None: self.first_joint_sim=self.clock_s

        def imu_cb(self,msg):
            q,g,a=msg.orientation,msg.angular_velocity,msg.linear_acceleration
            valid=all(math.isfinite(v) for v in [q.x,q.y,q.z,q.w,g.x,g.y,g.z,a.x,a.y,a.z]) and q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w>.5
            self.last['/sim/raw/imu']={'wall':time.monotonic(),'sim':self.clock_s,
                'source':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'valid':valid}
            self.latest_imu=msg

        def final_cb(self,msg):
            values=(msg.linear.x,msg.linear.y,msg.angular.z); self.latest_final=values
            self.last[FINAL_TOPIC]={'wall':time.monotonic(),'sim':self.clock_s,'source':None,'values':values}
            if all(abs(v)<=1e-6 for v in values):
                if self.final_zero_since is None:self.final_zero_since=self.clock_s
            else:self.final_zero_since=None

        def debug_cb(self,msg):
            if self.clock_s is not None: self.controller_debug_sim=self.clock_s

        def freshness(self,topic,wall_limit=1.):
            rec=self.last.get(topic)
            return bool(rec and time.monotonic()-rec['wall']<=wall_limit)

        def publisher_audit(self):
            expected={RAW_TOPIC:'velocity_step_recorder',SHARED_TOPIC:'velocity_smoother',FINAL_TOPIC:'velocity_adapter'}
            expected.update({f'/model/b2w/joint/{joint}/cmd_force':'b2w_rl_sar_controller' for joint in JOINT_NAMES})
            result={}
            for topic,name in expected.items():
                infos=self.get_publishers_info_by_topic(topic)
                nodes=[(i.node_name,i.node_namespace) for i in infos]
                result[topic]={'publishers':nodes,'expected':name,'matches':nodes==[(name,'/')]}
            return result

        def endpoint_audit(self):
            result={}
            for topic in self.recorder.topics:
                result[topic]=[{'node_name':i.node_name,'namespace':i.node_namespace,
                    'type':i.topic_type,'reliability':str(i.qos_profile.reliability),
                    'durability':str(i.qos_profile.durability),'depth':i.qos_profile.depth}
                    for i in self.get_publishers_info_by_topic(topic)]
            return result

        def lifecycle_poll(self):
            if self.lifecycle_future is not None and self.lifecycle_future.done():
                try:self.lifecycle_active=self.lifecycle_future.result().current_state.id==3
                except Exception:self.lifecycle_active=False
                self.lifecycle_future=None
            if self.lifecycle_future is None and self.lifecycle_client.service_is_ready():
                self.lifecycle_future=self.lifecycle_client.call_async(GetState.Request())

        def readiness(self):
            self.lifecycle_poll()
            names=self.get_node_names()
            identities=self.get_node_names_and_namespaces()
            controller_nodes=[(n,ns) for n,ns in identities if n=='b2w_rl_sar_controller']
            required={TRUTH_TOPIC,SLAM_TOPIC,'/sim/raw/imu','/sim/raw/joint_states',FINAL_TOPIC}
            topics=self.get_topic_names_and_types(); topicset={t for t,_ in topics}
            audit=self.publisher_audit()
            try:
                self.tf.lookup_transform('map','base_link',rclpy.time.Time())
                tf_ok=True
            except TransformException:tf_ok=False
            clock_ok=self.clock_advances>0 and self.last_clock_wall is not None and time.monotonic()-self.last_clock_wall<CLOCK_FREEZE_TIMEOUT_S
            stand_ok=(self.clock_s is not None and self.first_joint_sim is not None and self.clock_s-self.first_joint_sim>=2.2 and
                      self.controller_debug_sim is not None and self.clock_s-self.controller_debug_sim<.5)
            fresh=all(self.freshness(t,1.) for t in ['/sim/raw/imu','/sim/raw/joint_states',SLAM_TOPIC,TRUTH_TOPIC,FINAL_TOPIC])
            frames={topic:self.last.get(topic,{}).get('child_frame_id') for topic in (TRUTH_TOPIC,SLAM_TOPIC)}
            return {'clock_advancing':clock_ok,'clock_epoch_stable':not self.clock_rollback,
                    'controller_alive':len(controller_nodes)==1 and controller_nodes[0][1]=='/',
                    'startup_stand_and_policy_warmup':stand_ok,
                    'fresh_private_joint_state':self.freshness('/sim/raw/joint_states') and self.last.get('/sim/raw/joint_states',{}).get('valid',False),
                    'fresh_private_imu':self.freshness('/sim/raw/imu') and self.last.get('/sim/raw/imu',{}).get('valid',False),
                    'fresh_slam_odom':self.freshness(SLAM_TOPIC) and self.last.get(SLAM_TOPIC,{}).get('valid',False),
                    'fresh_ground_truth_instrumentation':self.freshness(TRUTH_TOPIC),'valid_map_to_base_tf':tf_ok,
                    'truth_twist_child_frame_base_link':frames[TRUTH_TOPIC]=='base_link',
                    'slam_twist_child_frame_base_link':frames[SLAM_TOPIC]=='base_link',
                    'velocity_smoother_active':self.lifecycle_active,'velocity_adapter_alive':'velocity_adapter' in names,
                    'final_command_fresh_and_zero':self.latest_final is not None and all(abs(v)<=1e-6 for v in self.latest_final) and self.freshness(FINAL_TOPIC),
                    'expected_publishers_only':publisher_contract_ok(audit),'publisher_audit':audit,
                    'required_topics_discovered':required.issubset(topicset),'all_feedback_fresh':fresh,
                    'recorder_receiving':self.recorder_ready(),
                    'unique_clock_publisher':len(self.get_publishers_info_by_topic('/clock'))==1,
                    'no_navigation_command_nodes':not {'controller_server','planner_server','behavior_server','bt_navigator','goal_manager'}.intersection(names)}

        def recorder_ready(self):
            required=('/clock',RAW_TOPIC,SHARED_TOPIC,FINAL_TOPIC,TRUTH_TOPIC,SLAM_TOPIC,
                      '/sim/raw/imu','/sim/raw/joint_states','/network_input_debug','/joint_commands','/joint_torques','/tf','/tf_static')
            wall=time.monotonic()
            return (self.recorder.sim_s is not None and self.clock_s is not None and
                    abs(self.recorder.sim_s-self.clock_s)<.2 and
                    all(self.recorder.counts.get(t,0)>0 and
                        (t=='/tf_static' or wall-self.recorder.last_received[t]['wall_s']<1.) for t in required))

        def validate_runtime(self):
            if self.clock_rollback:return 'simulation clock rollback/epoch crossing'
            audit=self.publisher_audit()
            if not publisher_contract_ok(audit):return 'competing or missing raw/shared/final command publisher'
            controller_nodes=[(n,ns) for n,ns in self.get_node_names_and_namespaces() if n=='b2w_rl_sar_controller']
            if len(controller_nodes)!=1 or controller_nodes[0][1]!='/':return 'rl_sar controller died or duplicate controller appeared'
            if not self.lifecycle_active:return 'velocity smoother lifecycle is not active'
            if not self.freshness(SLAM_TOPIC,.5) or not self.last.get(SLAM_TOPIC,{}).get('valid',False):return 'required SLAM odometry stale or invalid'
            if not self.freshness('/sim/raw/imu',.5) or not self.last.get('/sim/raw/imu',{}).get('valid',False):return 'required private IMU stale or invalid'
            if not self.freshness('/sim/raw/joint_states',.5) or not self.last.get('/sim/raw/joint_states',{}).get('valid',False):return 'required private joint state stale or invalid'
            if not self.freshness(FINAL_TOPIC,.5):return 'final command stream stale'
            now_wall=time.monotonic()
            for topic in (RAW_TOPIC,SHARED_TOPIC):
                received=self.recorder.last_received.get(topic)
                if not received or now_wall-received['wall_s']>.5:return f'{topic} stream stale'
            if self.last.get(SLAM_TOPIC,{}).get('child_frame_id')!='base_link' or self.last.get(TRUTH_TOPIC,{}).get('child_frame_id')!='base_link':
                return 'odometry body-twist child frame is unknown or changed'
            try:self.tf.lookup_transform('map','base_link',rclpy.time.Time())
            except TransformException:return 'map-to-base TF unavailable'
            if self.latest_imu is not None:
                q=self.latest_imu.orientation
                roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
                pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
                if max(abs(roll),abs(pitch))>TILT_LIMIT_RAD:return 'raw IMU tilt safety limit exceeded'
            return None

        def start_sequence(self):
            self.experiment_start=self.clock_s; self.run_start_wall=time.monotonic()
            self.zero_only=False

        def publish_zero(self):
            self.recorder.record_request(0.,{'phase':'safety_zero'},0.)

        def abort(self,reason):
            self.abort_reason=self.abort_reason or reason
            self.zero_only=True
            self.complete=True
            self.publish_zero()

        def tick(self):
            wall=time.monotonic()
            self.lifecycle_poll()
            if self.zero_only or self.complete:
                if self.recorder.sim_s is not None: self.publish_zero()
                return
            if self.abort_reason:
                self.abort(self.abort_reason);return
            if self.clock_s is None:return
            if self.experiment_start is None:return
            if self.last_clock_wall is None or wall-self.last_clock_wall>CLOCK_FREEZE_TIMEOUT_S:
                self.abort('simulation clock frozen for more than 2 wall seconds');return
            if wall-self.run_start_wall>RUN_WALL_TIMEOUT_S:
                self.abort('bounded 240-second wall watchdog exceeded');return
            elapsed=self.clock_s-self.experiment_start
            if elapsed<0:self.abort('simulation clock epoch crossed');return
            if wall-self.last_audit_wall>=.5:
                self.last_audit_wall=wall; reason=self.validate_runtime()
                if reason:self.abort(reason);return
            vx,vy,wz=requested_at(self.sequence,elapsed)
            phase=next((p for p in self.sequence if p['start_s']<=elapsed<p['end_s']),None)
            self.recorder.record_request(vx,phase or {'phase':'complete'},elapsed)
            if elapsed>=self.sequence[-1]['end_s']:
                self.complete=True

    return HarnessNode


def _make_executor(node, recorder):
    from rclpy.executors import SingleThreadedExecutor
    executor=SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    executor.add_node(recorder)
    return executor


def _initialize_ros():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    previous={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
    # Humble's default handlers shut down the context before finally can flush.
    # Turn signals into the runner's existing interruption/cleanup path instead.
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    for sig in previous: signal.signal(sig,interrupt)
    return previous


def run_experiment(args):
    if args.ros_domain_id < 1 or args.ros_domain_id > 232: raise ValueError('ROS_DOMAIN_ID must be in 1..232')
    if shutil.which('ros2') is None: raise RuntimeError('ros2 executable is not available; source ROS and install setup first')
    if not POLICY.is_file(): raise RuntimeError(f'missing deployed policy artifact: {POLICY}')
    root=Path(args.output_root).expanduser().resolve(); root.mkdir(parents=True,exist_ok=True)
    stamp=dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    run_dir=root/stamp; run_dir.mkdir(parents=False,exist_ok=False)
    token=secrets.token_hex(16); fixture=generate_fixture_world(SOURCE_WORLD,run_dir/'velocity_step_fixture.sdf')
    transition_diagnostic=getattr(args,'transition_diagnostic',False)
    if transition_diagnostic:
        from scripts.low_speed_transition import plan as transition_plan, source_hashes as transition_sources
        manifest=build_manifest(run_dir,args.ros_domain_id,token,args.gui,fixture,args.speeds,transition_diagnostic=True)
        manifest.update({'transition_protocol':transition_plan(),'transition_source_hashes':transition_sources(),
                         'm2_inverse_enabled':False,'validated_condition':'pure_forward_vy0_wz0',
                         'truth_stop_speed_mps':transition_plan()['gates']['truth_stop_speed_mps']})
    else:
        manifest=build_manifest(run_dir,args.ros_domain_id,token,args.gui,fixture,args.speeds)
    manifest['mode']='readiness_smoke' if args.readiness_smoke else 'characterization'
    if transition_diagnostic:manifest['mode']='route_low_speed_transition'
    _write_json(run_dir/'manifest.json',manifest)
    env=dict(os.environ); env.update({'ROS_DOMAIN_ID':str(args.ros_domain_id),'ROS_LOCALHOST_ONLY':'1',
        'IGN_PARTITION':f"b2w-velocity-step-{token}",'FASTRTPS_DEFAULT_PROFILES_FILE':str(ROOT/'course_sim/config/fastdds_local.xml'),
        'B2W_NARROW_OWNER':token,'PYTHONPATH':str(ROOT/'.venv/lib/python3.10/site-packages')+os.pathsep+env.get('PYTHONPATH',''),
        'LD_LIBRARY_PATH':str(ROOT/'.deps/lib')+os.pathsep+str(ROOT/'b2w_sim/b2w_controllers/third_party/onnxruntime/lib64')+os.pathsep+env.get('LD_LIBRARY_PATH','')})
    if transition_diagnostic:env['PYTHONPATH']=str(ROOT/'course_navigation')+os.pathsep+env['PYTHONPATH']
    # The recorder is the owning ROS participant too; keep it in the exact
    # isolated domain/profile passed to the children, without the child token.
    for key in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','FASTRTPS_DEFAULT_PROFILES_FILE','PYTHONPATH','LD_LIBRARY_PATH'):
        os.environ[key]=env[key]
    os.environ.pop('B2W_NARROW_OWNER',None)
    ros2=shutil.which('ros2'); launches=_OwnedLaunches(run_dir,token,env)
    manager_params=run_dir/'velocity_step_lifecycle.yaml'
    manager_params.write_text('/**:\n  ros__parameters:\n    use_sim_time: true\n    autostart: true\n    node_names: [velocity_smoother]\n',encoding='utf-8')
    (run_dir/'events.jsonl').touch(exist_ok=True)
    course_args=[ros2,'launch','course_bringup','course.launch.py','mode:=slam','control_source:=none',
        'locomotion_backend:=rl_sar','fault_profile:=normal',f"world:={fixture['generated_world']}",
        'scene:=confined',f"gui:={'true' if args.gui else 'false'}",'rviz:=false','spawn_x:=-4.0','spawn_y:=0.0','spawn_z:=1.0']
    try:
        launches.start('course_stack',course_args)
        smoother_args=[ros2,'run','nav2_velocity_smoother','velocity_smoother','--ros-args','--params-file',str(NAV_CONFIG),
            '-r','cmd_vel:=/course_navigation/cmd_vel_raw','-r','cmd_vel_smoothed:=/course_navigation/cmd_vel']
        launches.start('velocity_smoother',smoother_args)
        launches.start('smoother_lifecycle_manager',[ros2,'run','nav2_lifecycle_manager','lifecycle_manager','--ros-args',
            '-r','__node:=velocity_step_lifecycle_manager','--params-file',str(manager_params)])
        adapter_args=[ros2,'run','course_navigation','velocity_adapter','--ros-args','-p','use_sim_time:=true']
        if transition_diagnostic:
            adapter_args=[sys.executable,'-c','from course_navigation.velocity_adapter import main; main()',
                          '--ros-args','-p','use_sim_time:=true','-p','m2_inverse_enabled:=false']
        launches.start('velocity_adapter',adapter_args)
    except BaseException as exc:
        cleanup=launches.cleanup()
        manifest['termination']={'status':'launch_start_failed','reason':f'{type(exc).__name__}: {exc}'}
        manifest['cleanup']=cleanup; manifest['processes']=launches.records; _write_json(run_dir/'manifest.json',manifest)
        raise
    manifest['processes']=launches.records
    previous_handlers={}; rclpy=None; node=None; recorder=None; executor=None; termination='readiness_aborted'; reason=None
    try:
        import rclpy as ros_runtime
        from scripts.velocity_step_recorder import create_recorder
        rclpy=ros_runtime; previous_handlers=_initialize_ros()
        recorder=create_recorder(run_dir)
        Harness=(_make_ros_node(run_dir,recorder,args.speeds,transition_diagnostic=True) if transition_diagnostic
                 else _make_ros_node(run_dir,recorder,args.speeds))
        node=Harness()
        executor=_make_executor(node,recorder)
        ready_deadline=time.monotonic()+READINESS_TIMEOUT_S; readiness={}; next_ready_check=0.
        while time.monotonic()<ready_deadline:
            executor.spin_once(timeout_sec=.01)
            if time.monotonic()<next_ready_check: continue
            next_ready_check=time.monotonic()+.2
            readiness=node.readiness()
            if readiness_complete(readiness):
                break
            if any(p.poll() is not None for _,p,_ in launches.processes):
                reason='owned launch process exited before readiness'; break
        manifest['readiness']={'checks':readiness,'wall_timeout_s':READINESS_TIMEOUT_S,'completed':reason is None and readiness_complete(readiness)}
        manifest['experiment_start_sim_s']=None
        manifest['endpoint_audit']=node.endpoint_audit()
        manifest['readiness_sim_s']=node.clock_s
        if reason is None and readiness_complete(readiness) and args.readiness_smoke:
            smoke_start=time.monotonic(); checks=[]; next_smoke_check=0.
            while time.monotonic()-smoke_start<3.:
                executor.spin_once(timeout_sec=.01)
                if time.monotonic()<next_smoke_check: continue
                next_smoke_check=time.monotonic()+.2
                snapshot=node.readiness()
                if not readiness_complete(snapshot):
                    reason='readiness lost during zero-only smoke'; break
                if any(p.poll() is not None for _,p,_ in launches.processes):
                    reason='owned launch exited during zero-only smoke'; break
                checks.append({'wall_s':time.monotonic(),'sim_s':node.clock_s,'checks':snapshot})
            manifest['readiness_smoke']={'duration_wall_s':time.monotonic()-smoke_start,
                'snapshots':checks[::max(1,len(checks)//12)],'check_count':len(checks),
                'zero_only':True,'passed':reason is None,'end_sim_s':node.clock_s}
            termination='readiness_smoke_completed' if reason is None else 'readiness_smoke_failed'
        elif reason is None and readiness_complete(readiness):
            node.start_sequence(); manifest['experiment_start_sim_s']=node.experiment_start; termination='running'
            while not node.complete:
                executor.spin_once(timeout_sec=.1)
                reason=node.abort_reason
                if reason:termination='aborted';break
                if any(p.poll() is not None for _,p,_ in launches.processes):
                    node.abort('owned launch process exited during motion'); reason=node.abort_reason; termination='aborted'; break
            if reason is None:
                termination='completed'
            manifest['experiment_end_sim_s']=node.clock_s
            manifest['sequence_elapsed_sim_s']=node.clock_s-node.experiment_start
        elif reason is None:
            reason='readiness timeout or instrumentation precondition failed before motion'
            termination='instrumentation_precondition_failed'
        manifest['termination']={'status':termination,'reason':reason}
    except KeyboardInterrupt:
        termination='interrupted'; reason='user Ctrl+C'
    except Exception as exc:
        termination='exception'; reason=f'{type(exc).__name__}: {exc}'
    finally:
        # Stop sequence selection before servicing any further timers. Both
        # nodes remain owned by the same executor throughout repeated zero flush.
        if transition_diagnostic:
            from scripts.low_speed_transition import flush_zero
            flush_error=flush_zero(node,executor,rclpy)
            if flush_error:
                manifest['zero_flush_error']=flush_error;termination='zero_flush_failed';reason=flush_error
        else:
            if node is not None and executor is not None and rclpy and rclpy.ok():
                node.zero_only=True
                node.publish_zero(); flush_until=time.monotonic()+FLUSH_S
                try:
                    while time.monotonic()<flush_until:
                        executor.spin_once(timeout_sec=.05)
                except Exception as exc:
                    manifest['zero_flush_error']=f'{type(exc).__name__}: {exc}'
                    termination='zero_flush_failed'; reason=manifest['zero_flush_error']
        if transition_diagnostic:
            # The new diagnostic inherits the reviewed guaranteed cleanup path.
            from scripts.m2_open_space import teardown
            result=teardown(executor,node,recorder,rclpy,launches,previous_handlers)
            manifest['recorder_health']=result['recorder_health'];manifest['cleanup']=result['cleanup']
            manifest['teardown_errors']=result['errors']
            if result['errors']:termination='teardown_failed';reason='; '.join(result['errors'])
        else:
            if executor is not None:
                executor.shutdown()
                for owned_node in (node,recorder):
                    if owned_node is not None: executor.remove_node(owned_node)
            if node is not None: node.destroy_node()
            if recorder is not None:
                recorder.destroy_node()
                manifest['recorder_health']=recorder.close()
            if rclpy and rclpy.ok(): rclpy.shutdown()
            cleanup=launches.cleanup(); manifest['cleanup']=cleanup
            for sig,handler in previous_handlers.items(): signal.signal(sig,handler)
        if manifest['termination'] is None:manifest['termination']={'status':termination,'reason':reason}
        else:
            manifest['termination'].update({'final_status':termination,'reason':reason or manifest['termination'].get('reason')})
        if recorder is not None:
            raw_events=_read_events(run_dir/'events.jsonl')
            joint=next((e for e in reversed(raw_events) if e.get('topic')=='/sim/raw/joint_states'),None)
            imu=next((e for e in reversed(raw_events) if e.get('topic')=='/sim/raw/imu'),None)
            wheel=bool(joint and joint.get('data',{}).get('wheel_velocity_by_name_radps'))
            orientation=bool(imu and imu.get('data',{}).get('orientation_xyzw'))
            manifest['optional_streams']['wheel_velocity']={'available':wheel,'reason':None if wheel else 'private wheel joint velocity samples absent'}
            manifest['optional_streams']['roll_pitch']={'available':orientation,'reason':None if orientation else 'valid raw IMU orientation unavailable'}
        _write_json(run_dir/'manifest.json',manifest)
    events=_read_events(run_dir/'events.jsonl') if (run_dir/'events.jsonl').exists() else []
    try:
        result=analyze_run(events,manifest); _write_outputs(run_dir,manifest,events,result)
    except Exception as exc:
        manifest['analysis_error']=f'{type(exc).__name__}: {exc}';_write_json(run_dir/'manifest.json',manifest)
    print(json.dumps({'run_directory':str(run_dir),'termination':manifest.get('termination'),
        'cleanup_passed':manifest.get('cleanup',{}).get('passed'),'mode':'readiness_smoke' if args.readiness_smoke else 'characterization'},indent=2))
    return 0 if termination in ('completed','readiness_smoke_completed') and manifest.get('cleanup',{}).get('passed') else 1


def parse_speeds(value):
    try:
        values=tuple(float(part.strip()) for part in value.split(','))
        make_sequence(values)
    except (ValueError,TypeError) as exc:
        raise argparse.ArgumentTypeError(f'invalid --speeds: {exc}') from exc
    return values


def make_parser():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speeds',type=parse_speeds,default=DEFAULT_SPEEDS,
        help='distinct comma-separated policy-command experiment inputs in [0.04,0.40] m/s; default .10,.20,.30,.40; 12 sim seconds per point')
    parser.add_argument('--output-root',default='/tmp/b2w-velocity-steps')
    parser.add_argument('--ros-domain-id',type=int,default=194)
    parser.add_argument('--gui',dest='gui',action='store_true',default=True)
    parser.add_argument('--no-gui',dest='gui',action='store_false')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--readiness-smoke',action='store_true',help='launch the full stack, maintain real readiness for 3 wall seconds, publish only zero, record and clean up')
    mode.add_argument('--dry-run',action='store_true',help='print the bounded plan; import no ROS and launch/publish nothing')
    mode.add_argument('--analyze',metavar='RUN_DIR',help='recompute derived metrics offline; preserve events.jsonl bytes')
    return parser


def main(argv=None):
    args=make_parser().parse_args(argv)
    if args.dry_run:return _dry_run(args.gui,args.speeds)
    if args.analyze:return _analyze_dir(Path(args.analyze).expanduser().resolve())
    try:return run_experiment(args)
    except Exception as exc:
        print(f'velocity step runner failed before/within owned session: {type(exc).__name__}: {exc}',file=sys.stderr)
        return 1


if __name__=='__main__':
    raise SystemExit(main())
