#!/usr/bin/env python3
"""Deterministic OFFLINE audit of preserved warehouse commands, never a route replay.

Unstamped Twist uses observer simulation receipt time, not controller send time.
Time fractions have the entire explicit window as denominator; stale/unobserved
intervals remain missing. Count quantiles use linear rank interpolation.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
import sys
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from scripts.smoother_m2_support import audit as smoother_audit
V_MIN = .10690763385664151
V_MAX = .27052833048110847
TOPICS = {'raw': '/course_navigation/cmd_vel_raw', 'shared': '/course_navigation/cmd_vel',
          'final': '/path_manager/path_manager_ros/nav_vel'}
DEFAULT_PATHS = [ROOT/'course_results/warehouse-baseline-20261013/trace.jsonl',
                 *sorted(Path('/tmp/b2w-warehouse-observer').glob('*/events.jsonl')),
                 *sorted(Path('/tmp/b2w-online-planner-ab/navigation').glob('*/B-*/observations.jsonl'))]


def bucket(v):
    if v == 0: return 'zero'
    if v < 0: return 'negative'
    if v < V_MIN: return 'low'
    if v <= V_MAX: return 'supported'
    return 'high'


def statistics(rows, start, end, max_gap=.5):
    rows = sorted(((t,v) for t,v in rows if math.isfinite(t) and math.isfinite(v)), key=lambda row:row[0])
    selected = [(t,v) for t,v in rows if start <= t < end]
    counts = {k:0 for k in ('zero','low','supported','high','negative')}
    durations = {k:0. for k in counts}
    for _,v in selected: counts[bucket(v)] += 1
    # Last receipt at equal simulation time wins for integration only.
    dedup = sorted(dict(rows).items())
    for (a,v),(b,_) in zip(dedup,dedup[1:]):
        if 0 < b-a <= max_gap:
            durations[bucket(v)] += max(0.,min(end,b)-max(start,a))
    positives = sorted(v for _,v in selected if v>0)
    def quantile(q):
        x=(len(positives)-1)*q; i=int(x); f=x-i
        return positives[i]*(1-f)+positives[min(i+1,len(positives)-1)]*f
    n=len(selected); covered=sum(durations.values()); duration=end-start
    return {'count':n, 'counts':counts, 'count_fraction':{k:v/n for k,v in counts.items()} if n else None,
            'window_sim_s':[start,end], 'time_s':durations, 'covered_time_s':covered,
            'coverage_fraction':covered/duration if duration>0 else None,
            'time_fraction':{k:v/duration for k,v in durations.items()} if rows and duration>0 else None,
            'missing_time_s':max(0.,duration-covered), 'max_hold_gap_s':max_gap,
            'positive':{'min':positives[0], 'median':quantile(.5), 'p90':quantile(.9), 'max':positives[-1]} if positives else None}


def load(path):
    streams = {}; goals=[]; markers=[]; times=[]; recovery=[]
    with path.open() as h:
        for line_number,line in enumerate(h,1):
            e=json.loads(line); t=e.get('sim_receive_s',e.get('sim')); epoch=e.get('epoch',0)
            if not isinstance(t,(int,float)) or not math.isfinite(t): continue
            times.append((epoch,t)); topic=e.get('topic'); kind=e.get('kind')
            value=e.get('value',{}); vx=None
            if topic in TOPICS.values():
                if isinstance(value,dict) and 'linear' in value: vx=value['linear']['x']
                elif 'vx' in e: vx=e['vx']
            elif kind in ('raw','smoothed','final') and isinstance(value,list):
                topic=TOPICS[{'raw':'raw','smoothed':'shared','final':'final'}[kind]];vx=value[0]
            if vx is not None: streams.setdefault((epoch,topic),[]).append((t,float(vx)))
            if topic=='/goal_pose':
                p=value['pose']['position']
                goals.append({'epoch':epoch,'sim_receive_s':t,'source_stamp_s':e.get('stamp_s'),
                              'xy':[p['x'],p['y']],'line':line_number,'seq':e.get('seq')})
            if kind=='goal_sent': goals.append({'epoch':epoch,'sim_receive_s':t,'xy':e['goal'],'line':line_number})
            if topic=='/behavior_tree_log':
                for transition in value.get('event_log',[]):
                    if transition.get('node_name') in ('Spin','BackUp','Wait','ClearEntireCostmap','ClearLocalCostmap','ClearGlobalCostmap'):
                        recovery.append({'epoch':epoch,'sim_receive_s':t,'line':line_number,**transition})
            if topic=='/rosout' and any(s in value.get('msg','') for s in ['Begin navigating','Goal succeeded','Goal failed','Goal canceled']):
                markers.append({'epoch':epoch,'sim_receive_s':t,'msg':value['msg'],'line':line_number})
    reports=[]
    for epoch in sorted({epoch for epoch,_ in times}):
        ts=[t for ep,t in times if ep==epoch]; start,end=min(ts),max(ts)
        windows=[('whole_capture',start,end,None)]
        g=[g for g in goals if g['epoch']==epoch]
        begins=[m for m in markers if m['epoch']==epoch and m['msg'].startswith('Begin navigating')]
        # A transient observer can miss the initial /goal_pose. Preserve the
        # navigation log interval with unknown exact coordinates; never recover
        # full-precision canonical goals from rounded log text.
        if begins and (not g or begins[0]['sim_receive_s']<g[0]['sim_receive_s']):
            first=begins[0]
            g.insert(0,{'epoch':epoch,'sim_receive_s':first['sim_receive_s'],'xy':None,
                        'line':first['line'],'provenance':'navigation log only; rounded coordinates not reconstructed'})
        for i,goal in enumerate(g):
            a=goal['sim_receive_s']; next_goal=g[i+1]['sim_receive_s'] if i+1<len(g) else end
            begin=next((m for m in begins if a<=m['sim_receive_s']<next_goal),None)
            # A canceled old action can arrive after the new /goal_pose and
            # before its Begin message. Do not assign it to the new request.
            terminal=next((m for m in markers if begin and m['epoch']==epoch and
                           begin['sim_receive_s']<=m['sim_receive_s'] and (m['sim_receive_s']<next_goal or (i==len(g)-1 and m['sim_receive_s']==next_goal)) and m['msg'].startswith('Goal ')),None)
            b=terminal['sim_receive_s'] if terminal else next_goal
            windows.append((f'observed_goal_{i+1}',a,b,{'goal':goal,'begin':begin,'terminal':terminal,
                            'censored':terminal is None,'terminal_braking':'not established by these command/log markers','association':'observed goal interval; includes any recovery; noncanonical/unknown XY',
                            'recovery_transitions':[r for r in recovery if r['epoch']==epoch and a<=r['sim_receive_s']<b]}))
        for name,a,b,association in list(windows):
            if association:
                first_recovery=next((r['sim_receive_s'] for r in association['recovery_transitions'] if r.get('current_status')=='RUNNING'),None)
                if first_recovery is not None:
                    windows.extend([(name+'_before_first_recovery',a,first_recovery,{**association,'phase':'before first observed recovery; terminal braking not isolated'}),
                                    (name+'_from_first_recovery',first_recovery,b,{**association,'phase':'from first observed recovery to terminal/censor; may include further controller attempts'})])
        for name,a,b,association in windows:
            reports.append({'epoch':epoch,'window':name,'association':association,
                            'streams':{key:statistics(streams.get((epoch,topic),[]),a,b) for key,topic in TOPICS.items()}})
    return {'path':str(path.resolve()),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'goals':goals,'navigation_markers':markers,'windows':reports,
            'segment_warning':'No association to frozen G1/G2 unless exact goals, same-session boundary evidence and command timeline exist.'}


def audit(paths=DEFAULT_PATHS):
    runs=[load(Path(p)) for p in paths if Path(p).is_file()]
    metrics=[]
    for p in sorted((ROOT/'course_results/navigation').glob('*.json')):
        r=json.loads(p.read_text())
        metrics.append({'path':str(p.resolve()),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),
                        'goal_map_xy':r.get('goal_map_xy'),'success':r.get('success'),
                        'end_reason':r.get('end_reason'),'duration_sim_s_legacy_field':r.get('time_to_goal_s'),
                        'path_length_m':r.get('path_length_m'),'final_goal_distance_m':r.get('final_goal_distance_m'),
                        'command_timeline_present':False,'sim_start_end_present':False})
    structural=smoother_audit()
    return {'offline_only':True,'classification':structural['classification'],
            'NORMAL_NAV2_M2_READY':False,'structural_support':structural,
            'historical_limitations':['exact frozen-route support fractions','high-speed demand','braking vs traversal','recovery attribution','simultaneous angular-command distribution'],
            'support_desired_v_mps':[V_MIN,V_MAX], 'runs':runs,'navigation_metrics_inventory':metrics,
            'frozen_route_G1_G2':{'raw':None,'shared':None,'final':None,
                'reason':'Exact Oct16 G1/G2 manager metrics have no velocity samples or simulation boundary stamps. Oct13/14 streams use different goals and cannot establish frozen route support coverage.',
                'terminal_braking_vs_traversal':'unestablished for frozen route'},
            'smallest_next_check':'ONE route-relevant LOW-SPEED TRANSITION diagnostic with inverse OFF; structural low-speed blocker already proven. Exact G1/G2 capture is not required to establish it.',
            'method':'Receipt-time ZOH only between adjacent <=0.5s samples within one epoch; missing time never extended. Count fractions use all received samples, time fractions use whole window. Raw topic may include behavior commands; goal windows include recovery, never labeled pure corridor.'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);p.add_argument('paths',nargs='*')
    a=p.parse_args();result=audit(a.paths or DEFAULT_PATHS)
    Path(a.output).write_text(json.dumps(result,indent=2,sort_keys=True,allow_nan=False)+'\n')


if __name__=='__main__': main()
