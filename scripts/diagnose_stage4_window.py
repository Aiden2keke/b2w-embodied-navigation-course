#!/usr/bin/python3
"""Read preserved DWB evidence; correlate public grids without claiming private state."""
import argparse
import base64
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
from stage4_route.contract import digest,write_new


def seconds(t):return t['sec']+t['nanosec']*1e-9


def analyze(window,native):
    records=[json.loads(l) for l in window.open()]
    for r in records:
        if hashlib.sha256(base64.b64decode(r['cdr_base64'])).hexdigest()!=r['cdr_sha256']:raise ValueError('Raw CDR integrity failure')
    evaluations=[r for r in records if r['topic']=='/evaluation' and r['derived'].get('all_invalid')]
    rows=[]
    def previous(topic,r):
        return next((x for x in reversed(records) if x['topic']==topic and x['receipt']['monotonic_ns']<=r['receipt']['monotonic_ns']),None)
    for r in evaluations:
        m=r['source_message'];cm=previous('/local_costmap/costmap_raw',r)
        if cm is None:continue
        c=cm['source_message'];meta=c['metadata'];res=meta['resolution'];o=meta['origin']['position'];w,h=meta['size_x'],meta['size_y']
        def cost(p):
            x,y=math.floor((p['x']-o['x'])/res),math.floor((p['y']-o['y'])/res)
            return c['data'][y*w+x] if 0<=x<w and 0<=y<h else 'outside'
        starts=Counter();bad=Counter();zero=[]
        for t in m['twists']:
            poses=t['traj']['poses'];starts[str(cost(poses[0]))]+=1
            invalid=[cost(p) for p in poses if cost(p) in (253,254,255,'outside')]
            if invalid:bad[str(invalid[0])]+=1
            if all(abs(v)<1e-8 for v in t['traj']['velocity'].values()):zero.append(dict(start_cost=cost(poses[0]),max_cost=max(cost(p) for p in poses)))
        odom=previous('/slam/odom',r);tf=previous('/tf',r)
        transforms={}
        for x in records:
            if x['topic']=='/tf' and x['receipt']['monotonic_ns']<=r['receipt']['monotonic_ns']:
                for t in x['source_message']['transforms']:
                    if (t['header']['frame_id'],t['child_frame_id']) in [('map','odom'),('odom','base_link')]:transforms[t['header']['frame_id']+'->'+t['child_frame_id']]=seconds(m['header']['stamp'])-seconds(t['header']['stamp'])
        rows.append(dict(seq=r['seq'],sim=seconds(m['header']['stamp']),candidates=len(m['twists']),rejecting_critics=r['derived']['rejecting_critics'],
                         center_start_cost_counts=dict(starts),first_blocking_public_cost_counts=dict(bad),zero_velocity_candidates=zero,
                         public_costmap_age_s=seconds(m['header']['stamp'])-seconds(c['header']['stamp']),
                         slam_odom_age_s=seconds(m['header']['stamp'])-seconds(odom['source_message']['header']['stamp']) if odom else None,
                         latest_public_tf_ages_s=transforms,
                         initial_rollout_pose=m['twists'][0]['traj']['poses'][0],
                         sampled_velocity_ranges={k:[min(t['traj']['velocity'][k] for t in m['twists']),max(t['traj']['velocity'][k] for t in m['twists'])] for k in ('x','y','theta')}))
    commands=[r for r in records if r['topic']=='/path_manager/path_manager_ros/nav_vel']
    truths=[r for r in records if r['topic']=='/sim/ground_truth/odom']
    residual=[]
    for r in commands:
        m=r['source_message']
        if all(abs(m[k][axis])<.01 for k,axis in [('linear','x'),('linear','y'),('angular','z')]):
            later=[t for t in truths if 0<=(t['receipt']['monotonic_ns']-r['receipt']['monotonic_ns'])*1e-9<=1.]
            if len(later)>1:
                a,b=[t['source_message']['pose']['pose']['position'] for t in (later[0],later[-1])]
                residual.append(math.hypot(a['x']-b['x'],a['y']-b['y']))
    logs=[l for l in native.read_text().splitlines() if 'Trajectory Hits Obstacle' in l]
    return dict(window=str(window),window_sha256=digest(window),native_log=str(native),native_sha256=digest(native),
                raw_cdr_hashes_verified=len(records),retained_records=len(records),all_invalid_evaluations=rows,
                native_obstacle_lines=len(logs),native_first=logs[:3],
                max_truth_displacement_during_first_second_after_near_zero_command_m=max(residual,default=None),
                residual_limits='Receipt-time association only; command may change within the interval; not isolated braking distance',
                goals=[r['source_message'] for r in records if r['topic']=='/goal_pose'],
                limitations=['Public nearest-previous master grids may differ from controller-private grid under mutex',
                             'BaseObstacle checks trajectory centers; it does not prove padded-footprint clearance',
                             'Historical endpoints/headings differ from selected candidate; no candidate runtime attribution',
                             'No turn-too-late or residual-motion causal proof from this window alone'])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--window',type=Path,required=True);p.add_argument('--native-log',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();write_new(a.output,analyze(a.window,a.native_log));print(a.output)
