#!/usr/bin/env python3
"""Read a saved observer run; replay installed critics without ROS initialization.

Build scripts/frozen_obstacle_critic with CMake in a sourced Humble environment.
No subscriptions, publishers, actions, simulator or navigation parameter writes.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

from warehouse_observer_helpers import (age_statistics, command_state,
    footprint_geometry, measurement_health, precursor_chronology, stationary_intervals)


def read_lines(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream]


def write(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False)+'\n')


def frozen_snapshot(source, meta_path, binary, filled=True):
    cm=json.loads(meta_path.read_text()); md=cm['metadata']; origin=md['origin']
    raw_path=source/cm['raw_file']; raw=raw_path.read_bytes()
    digest=hashlib.sha256(raw).hexdigest()
    if digest!=cm['sha256'] or len(raw)!=md['size_x']*md['size_y']:
        raise ValueError('Raw hash/size mismatch: '+str(meta_path))
    q=origin['orientation']
    if any(q[k]!=0 for k in ['x','y','z']) or q['w']!=1:
        raise ValueError('Nav2 Costmap2D requires identity origin orientation')
    pose=cm['pose_at_costmap_stamp']; poly=cm['padded_footprint_at_costmap_stamp']
    stamp=cm['header']['stamp']; t=stamp['sec']+stamp['nanosec']*1e-9
    if pose['frame']!=cm['header']['frame_id'] or abs(pose['stamp_s']-t)>1e-8:
        raise ValueError('Pose frame/stamp mismatch')
    fs=cm['footprint_source']
    if not fs['already_padded'] or not 0<=fs['time_skew_s']<=.75:
        raise ValueError('Footprint freshness/padding gate')
    args=[md['size_x'],md['size_y'],md['resolution'],origin['position']['x'],
          origin['position']['y'],*pose['xyyaw'],len(poly),*(v for p in poly for v in p)]
    response=subprocess.run([str(binary.resolve()),str(raw_path.resolve())],
                            input=' '.join(map(str,args)),text=True,capture_output=True,check=True)
    critic=json.loads(response.stdout)
    g=dict(width=md['size_x'],height=md['size_y'],resolution=md['resolution'],
           origin=[origin['position']['x'],origin['position']['y'],0.],data=raw)
    return dict(t=t,metadata_file=str(meta_path.relative_to(source)),raw_file=cm['raw_file'],
                sha256=digest,frame=cm['header']['frame_id'],pose=pose,
                padded_footprint=poly,footprint_source=fs,critic=critic,
                filled_geometry=footprint_geometry(g,pose['xyyaw'],poly) if filled else None)


def analyze(source, binary, output):
    if not source.is_dir(): raise FileNotFoundError(source)
    if output.resolve()==source.resolve() or source.resolve() in output.resolve().parents:
        raise ValueError('Output must not modify original evidence')
    output.mkdir(parents=True,exist_ok=True)
    events=read_lines(source/'events.jsonl'); rows=read_lines(source/'samples.jsonl')
    summary=json.loads((source/'summary.json').read_text())
    manifest=json.loads((source/'manifest.json').read_text())
    logs=[e for e in events if e['topic']=='/rosout']
    goal=next(e for e in events if e['topic']=='/goal_pose')
    failure=next(e for e in logs if 'No valid trajectories' in e['value']['msg'])
    begin=next(e for e in logs if e['sim_receive_s']>goal['sim_receive_s'] and
               'Received a goal' in e['value']['msg'])
    # Use event-defined boundaries, never whole-attach count / duration.
    bounds=dict(start_receive_s=begin['sim_receive_s'],end_receive_s=failure['sim_receive_s'],
                start_seq=begin['seq'],end_seq=failure['seq'],end_exclusive=True)
    command_topics=['/course_navigation/cmd_vel_raw','/course_navigation/cmd_vel',
                    '/path_manager/path_manager_ros/nav_vel']
    active=[]; rates={}
    for topic in command_topics:
        received=[e for e in events if e['topic']==topic and
                  begin['wall_monotonic_s']<=e['wall_monotonic_s']<failure['wall_monotonic_s']]
        sim_span=failure['sim_receive_s']-begin['sim_receive_s']
        wall_span=failure['wall_monotonic_s']-begin['wall_monotonic_s']
        rates[topic]=dict(count=len(received),wall_duration_s=wall_span,simulation_duration_s=sim_span,
                         receive_hz_wall=len(received)/wall_span,receive_hz_sim=len(received)/sim_span,
                         wall_interarrival_s=age_statistics(b['wall_monotonic_s']-a['wall_monotonic_s']
                                                           for a,b in zip(received,received[1:])),
                         first_seq=received[0]['seq'],last_seq=received[-1]['seq'])
    active=[r for r in rows if begin['sim_receive_s']<=r['t']<failure['sim_receive_s']]
    health=measurement_health(active,{}, {},1.,1.,'light')
    timing=dict(bounds=bounds,rates=rates,active_samples=len(active),
                active_tf_age_s=health['tf_age_s'],active_fresh_samples=health['fresh_samples'],
                whole_run=summary['measurement_health'],errors=summary['errors'],
                omitted_topics=manifest['omitted_dwb_debug_topics'])
    write(output/'timing.json',timing)
    # Exact source events retained, including clocks, action UUIDs and BT values.
    chronology=[e for e in events if e['topic'] in ['/goal_pose','/rosout',
                '/navigate_to_pose/_action/status','/follow_path/_action/status']]
    write(output/'chronology.json',chronology)
    (output/'bt-events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events
                                              if e['topic']=='/behavior_tree_log'))
    command_changes=[]; last_raw=None
    for topic in command_topics:
        old=None; last=None
        for e in events:
            if e['topic']!=topic: continue
            v=e['value']; state=command_state([v['linear']['x'],v['linear']['y'],v['angular']['z']])
            exact_zero=all(v[k][d]==0 for k in ['linear','angular'] for d in ['x','y','z'])
            if exact_zero!=old:
                command_changes.append(dict(event=e,state=state,exact_zero=exact_zero))
            old=exact_zero; last=e
        if topic==command_topics[0]: last_raw=last
    write(output/'command-chronology.json',dict(changes=command_changes,last_raw=last_raw))
    extended=[]
    for interval in stationary_intervals(rows):
        selected=interval.pop('rows')
        original=next(c for c in summary['candidate_stationary_intervals'] if c['start']==interval['start'])
        extended.append(dict(original_classification=original,
                             precursor_chronology=precursor_chronology(selected,events)))
    write(output/'extended-summary.json',extended)
    selected=[]; sweep=[]
    targets={'odom':[93.5975,94.1975,94.7975,95.3975,95.9975,96.5975,97.1975,97.7925],
             'map':[94.02,96.0175,98.015]}
    for path in sorted((source/'costmaps').glob('*.json')):
        cm=json.loads(path.read_text()); ts=cm['header']['stamp']; t=ts['sec']+ts['nanosec']*1e-9
        frame=cm['header']['frame_id']; wanted=any(abs(t-v)<.003 for v in targets.get(frame,[]))
        in_sweep=frame=='odom' and goal['sim_receive_s']<=t<=99.47
        if wanted or in_sweep:
            record=frozen_snapshot(source,path,binary,filled=wanted)
            if wanted: selected.append(record)
            if in_sweep: sweep.append(record)
    selected.sort(key=lambda e:(e['frame'],e['t']));sweep.sort(key=lambda e:e['t'])
    write(output/'geometry.json',selected);write(output/'local-sweep.json',sweep)
    progress=[e for e in logs if 'failed to make progress' in e['value']['msg'].lower()]
    force=[r['force_n'] for r in rows if r.get('force_n') is not None]
    write(output/'findings.json',dict(progress_logs=progress,
          plate_force_n=age_statistics(force),stationary_intervals=[
              {k:v for k,v in r.items() if k!='rows'} for r in stationary_intervals(rows)],
          mismatch_times=[r['t'] for r in sweep if r['critic']['base_legal'] and
                          not r['critic']['footprint_legal']],
          source=str(source),binary=str(binary),
          installed_library_sha256=hashlib.sha256(Path('/opt/ros/humble/lib/libdwb_critics.so').read_bytes()).hexdigest()))
    print(json.dumps(dict(snapshots=len(selected),sweep_snapshots=len(sweep),output=str(output))))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--critic-binary',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    analyze(args.source,args.critic_binary,args.output)


if __name__=='__main__': main()
