#!/usr/bin/env python3
"""OFFLINE ONLY: audit completed native velocity captures and compare calibration models.

Writes new JSON/CSV/Markdown artifacts outside input directories. Never imports
ROS, starts a process, rewrites a capture, or changes a production command.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from statistics import mean, stdev

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from scripts.velocity_step_core import analyze_run, _event_time, _window_mean, _zoh_integral
from scripts.velocity_calibration_core import WHEELS, axis_forward_sign, forward_equivalent, compare_models

CPP='b2w_sim/b2w_controllers/src/rl_sar_controller.cpp'
CONFIG='course_sim/config/rl_sar.yaml'
URDF='b2w_sim/b2w_description_ros2/urdf/b2w_description_rsl.urdf'
GENERATOR='course_sim/course_sim/description.py'
AUTH_FILES=(CPP,CONFIG,URDF,GENERATOR)


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def authenticate(manifest):
    """Bind debug layout, target units/order and geometry to capture-time hashes."""
    hashes=manifest.get('source_hashes') or {}; reasons=[]
    for name in AUTH_FILES:
        if hashes.get(name)!=digest(ROOT/name):reasons.append(f'capture/current source hash not authenticated: {name}')
    if reasons:return {'valid':False,'reasons':reasons,'radius_m':None,'signs':None,'joint_names':None,'torque_limits':None}
    import yaml
    config=yaml.safe_load((ROOT/CONFIG).read_text())['b2w_rl_sar_controller']['ros__parameters']
    cpp=(ROOT/CPP).read_text()
    snippets=['obs[6+i]=command_valid?command_[i]:0;', 'obs[25+i]=dq_[i]*.05;',
        'm.data.assign(obs.begin(),obs.end()); debug_->publish(m);',
        'm.data[i]=velocity_target_[i]; actions_pub_->publish(m);',
        'velocity_target_[i]=i>=12?scale_[i]*last_[i]:0;']
    if not all(s in cpp for s in snippets):raise ValueError('authenticated controller layout no longer matches this offline decoder')
    names=config['joint_names']
    if tuple(names[12:])!=WHEELS or len(names)!=16:raise ValueError('unsupported policy wheel joint ordering')
    tree=ET.parse(ROOT/URDF).getroot();radii=[];signs={}
    for w in WHEELS:
        prefix=w.split('_')[0]
        for suffix,axis in [('hip','1 0 0'),('thigh','0 1 0'),('calf','0 1 0'),('foot','0 1 0')]:
            j=tree.find(f"joint[@name='{prefix}_{suffix}_joint']")
            if j is None or [float(v) for v in j.find('origin').get('rpy','0 0 0').split()]!=[0.,0.,0.] or j.find('axis').get('xyz')!=axis:
                raise ValueError('unsupported wheel chain orientation; forward signs require a new audit')
        signs[w]=axis_forward_sign(map(float,j.find('axis').get('xyz').split()))
        cylinder=tree.find(f"link[@name='{prefix}_foot']/collision/geometry/cylinder")
        radii.append(float(cylinder.get('radius')))
    if len(set(radii))!=1:raise ValueError('different wheel radii require a new geometry audit')
    return {'valid':True,'reasons':[],'radius_m':radii[0],'signs':signs,
        'joint_names':names,'torque_limits':config['torque_limits'],
        'evidence':{'paths':list(AUTH_FILES),'snippets':snippets},
        'forward_definition':'omega_fwd[j] = sign(axis_y in body at nominal pose) * dq[j]; order FR,FL,RR,RL; runtime hip |q|<pi/2 required',
        'kinematic_limit':'r*mean(omega_fwd) is a wheel-spin approximation, not a measured no-slip body model; calf/leg and base angular rates, load and contact remain unmodelled'}


def series(events,topic,extract):
    rows=[]
    for e in events:
        if e.get('topic')!=topic or not e.get('valid',True):continue
        t=_event_time(e)
        try:v=float(extract(e.get('data') or {}))
        except (ValueError,TypeError,KeyError,IndexError):continue
        if t is not None and math.isfinite(v):rows.append((t,v))
    # Equal-stamp last receipt wins, matching the existing core.
    return sorted(dict(rows).items())


def window_stats(rows,start,end,gap=.15,zoh=False):
    gap += 1e-9 # numerical tolerance at the declared native-time gap boundary
    if zoh:
        area,cov=_zoh_integral(rows,start,end,gap)
        value=area/((end-start)*cov) if cov else None
    else:value,cov=_window_mean(rows,start,end,gap)
    values=[v for t,v in rows if start<=t<end]
    return {'mean':value if cov>=.95-1e-9 else None,'coverage':cov,'sample_count':len(values),
        'min':min(values) if values else None,'max':max(values) if values else None,
        'reason':None if value is not None and cov>=.95-1e-9 else 'missing/invalid or <95% supported coverage'}


def _wheel_actual(data,auth):
    return forward_equivalent(data['name'],data['velocity'],auth['signs'])


def wheel_window(events,start,end,auth):
    empty={'order':list(WHEELS),'target_radps':None,'actual_radps':None,
        'target_mean_radps':None,'actual_mean_radps':None,'tracking_target_minus_actual_radps':None,
        'tracking_mean_radps':None,'target_kinematic_mps':None,'actual_kinematic_mps':None,
        'reason':'wheel geometry/order/signs not authenticated'}
    if not auth['valid']:return empty
    # Hip roll rotates transverse axes; positive cos keeps this forward SIGN valid.
    hip_rows=[series(events,'/sim/raw/joint_states',lambda d,n=n:d['position'][d['name'].index(n)]) for n in [auth['joint_names'][i] for i in (0,3,6,9)]]
    if any(window_stats(r,start,end)['mean'] is None or any(abs(q)>=math.pi/2 for t,q in r if start<=t<end) for r in hip_rows):
        return {**empty,'reason':'runtime hip-axis forward sign unsupported'}
    targets=[];actual=[];error=[];error_abs=[];error_square=[];coverage=[]
    for i,w in enumerate(WHEELS):
        target=series(events,'/joint_commands',lambda d,i=i:d['data'][12+i]*auth['signs'][WHEELS[i]] if len(d['data'])==16 else None)
        dq=series(events,'/sim/raw/joint_states',lambda d,i=i:(_wheel_actual(d,auth) or [None]*4)[i])
        targets.append(window_stats(target,start,end,zoh=True));actual.append(window_stats(dq,start,end))
        stamps=[t for t,_ in target];paired=[]
        for t,v in dq:
            k=bisect.bisect_right(stamps,t)-1
            if k>=0 and t-stamps[k]<=.05+1e-9:paired.append((t,target[k][1]-v))
        es=window_stats(paired,start,end);coverage.append(es['coverage']);error.append(es['mean'])
        error_abs.append(window_stats([(t,abs(v)) for t,v in paired],start,end)['mean'])
        error_square.append(window_stats([(t,v*v) for t,v in paired],start,end)['mean'])
    tv=[s['mean'] for s in targets];av=[s['mean'] for s in actual]
    def complete_mean(v):return mean(v) if all(x is not None for x in v) else None
    tm,am=complete_mean(tv),complete_mean(av);r=auth['radius_m']
    def asym(v):
        if any(x is None for x in v):return None
        return {'max_minus_min_radps':max(v)-min(v),'left_minus_right_radps':(v[1]+v[3]-v[0]-v[2])/2,
            'front_minus_rear_radps':(v[0]+v[1]-v[2]-v[3])/2}
    return {**empty,'reason':None if tm is not None and am is not None else 'missing/unsupported target or dq stream',
        'target_radps':tv,'actual_radps':av,'target_mean_radps':tm,'actual_mean_radps':am,
        'target_quality':targets,'actual_quality':actual,'tracking_coverage':coverage,
        'tracking_target_minus_actual_radps':error,'tracking_mean_radps':complete_mean(error),
        'tracking_mae_radps':error_abs,'tracking_rmse_radps':[math.sqrt(v) if v is not None else None for v in error_square],
        'tracking_aggregate_mae_radps':complete_mean(error_abs),
        'tracking_aggregate_rmse_radps':math.sqrt(complete_mean(error_square)) if complete_mean(error_square) is not None else None,
        'target_kinematic_mps':r*tm if tm is not None else None,'actual_kinematic_mps':r*am if am is not None else None,
        'target_asymmetry':asym(tv),'actual_asymmetry':asym(av),
        'tracking_alignment':'actual source time vs latest target receive-time proxy <=50ms; weighted supported segments; not exact torque application time'}


def extract_observations(events,manifest,run_id):
    analysis=analyze_run(events,manifest);auth=authenticate(manifest);result=[]
    for row in analysis['per_speed']:
        start,end=row['steady_state_window_s'];reasons=[];slam_reasons=[]
        policy=window_stats(series(events,'/network_input_debug',lambda d:d['data'][6] if len(d['data'])==57 else None),start,end,zoh=True,gap=.05)
        # Use the exact captured float plateau, never its rounded integral.
        # Otherwise receipt jitter can fabricate distinct speeds in repeats.
        constant=policy['mean'] is not None and policy['min']==policy['max']
        u=policy['min'] if auth['valid'] and constant else None
        if policy['mean'] is not None and not constant:reasons.append('policy vx plateau is not constant')
        if u is None:reasons.append('actual ONNX command missing/unsupported or semantics unauthenticated')
        req=row['requested_vx_mps']; pipeline={}
        for key,topic in [('requested',manifest.get('requested_topic','/harness/requested')),
                ('raw',manifest.get('raw_topic','/course_navigation/cmd_vel_raw')),
                ('shared',manifest.get('shared_topic','/course_navigation/cmd_vel')),
                ('final',manifest.get('final_topic','/path_manager/path_manager_ros/nav_vel'))]:
            pipeline[key]=window_stats(series(events,topic,lambda d:d['linear_x']),start,end,zoh=True,
                gap=manifest.get('max_gap_s',{}).get('command',.15))
        for name,stats in pipeline.items():
            if stats['mean'] is None:reasons.append(f'{name} command stream missing/unsupported')
        for component in (6,7,8):
            s=window_stats(series(events,'/network_input_debug',lambda d,i=component:d['data'][i] if len(d['data'])==57 else None),start,end,zoh=True,gap=.05)
            expected=req if component==6 else 0.
            if s['mean'] is None or s['min'] is None or max(abs(s['min']-expected),abs(s['max']-expected))>1e-6:
                reasons.append(f'policy obs[{component}] does not support the requested pure-forward plateau')
        command_valid=not reasons
        wheel=wheel_window(events,start,end,auth)
        truth=row['truth']['steady_state_mean_vx_mps'];slam=row['slam']['steady_state_mean_vx_mps']
        for name,topic in [('truth',manifest.get('truth_topic','/sim/ground_truth/odom')),('slam',manifest.get('slam_topic','/slam/odom'))]:
            selected=[e for e in events if e.get('topic')==topic and (t:=_event_time(e)) is not None and start<=t<end]
            if not selected or any(e.get('child_frame_id')!='base_link' for e in selected):
                if name=='truth':
                    truth=None;reasons.append('truth body twist frame missing or not base_link')
                else:
                    slam=None;slam_reasons.append('slam body twist frame missing or not base_link')
        if truth is None:reasons.append(row['truth'].get('steady_state_reason') or 'truth steady vx invalid')
        if slam is None:slam_reasons.append(row['slam'].get('steady_state_reason') or 'slam steady vx invalid')
        ratios=[]
        for e in events:
            if e.get('topic')!='/joint_torques' or not e.get('valid',True) or not auth['valid']:continue
            t=_event_time(e);data=e.get('data',{}).get('data',[])
            if t is not None and start<=t<end and len(data)==16:
                if all(v is not None and math.isfinite(v) for v in data):ratios.append([abs(v)/lim for v,lim in zip(data,auth['torque_limits'])])
        kin=wheel['actual_kinematic_mps'];target_kin=wheel['target_kinematic_mps']
        result.append({'run_id':run_id,'trial':row['trial'],'requested_vx':req,'window_s':[start,end],
            'u':u,'v':truth,'slam_v':slam,'policy_command':policy,'pipeline':pipeline,'wheel':wheel,
            'truth_minus_target_wheel_mps':truth-target_kin if truth is not None and target_kin is not None else None,
            'slam_minus_target_wheel_mps':slam-target_kin if slam is not None and target_kin is not None else None,
            'truth_minus_actual_wheel_mps':truth-kin if truth is not None and kin is not None else None,
            'slam_minus_actual_wheel_mps':slam-kin if slam is not None and kin is not None else None,
            'torque_peak_limit_ratio':max(max(r) for r in ratios) if ratios else None,
            'wheel_torque_peak_limit_ratio':[max(r[i] for r in ratios) for i in range(12,16)] if ratios else None,
            'coupling':row['coupling'],'stop':row['stop'],'command_valid_for_fit':command_valid, 'valid_for_fit':u is not None and truth is not None and not reasons,
            'valid_for_slam_fit':command_valid and slam is not None,
            'invalid_reasons':reasons,'slam_invalid_reasons':slam_reasons,'authentication':auth})
    return result


def metric_repeatability(observations):
    """Independent-run means for the Protocol A comparisons; missing stays null."""
    extractors={
        'policy_command':lambda r:r['u'], 'truth_vx':lambda r:r['v'], 'slam_vx':lambda r:r['slam_v'],
        'wheel_target_mean':lambda r:r['wheel']['target_mean_radps'],
        'wheel_actual_mean':lambda r:r['wheel']['actual_mean_radps'],
        'target_lateral_m':lambda r:r['coupling']['target_phase']['lateral_displacement_m'],
        'target_yaw_rad':lambda r:r['coupling']['target_phase']['yaw_drift_rad'],
        'stop_lateral_m':lambda r:r['coupling']['stopping_phase']['lateral_displacement_m'],
        'stop_yaw_rad':lambda r:r['coupling']['stopping_phase']['yaw_drift_rad'],
        'request_zero_to_stop_s':lambda r:r['stop']['requested_zero_to_truth_stop_s'],
        'final_zero_to_stop_s':lambda r:r['stop']['final_zero_to_truth_stop_s'],
        'stop_path_m':lambda r:r['stop']['stop_path_distance_m']}
    for index,w in enumerate(WHEELS):
        for group in ('target','actual'):
            extractors[f'{group}.{w}']=lambda r,i=index,g=group:(r['wheel'][g+'_radps'] or [None]*4)[i]
    result=[]
    for requested in sorted({r['requested_vx'] for r in observations}):
        rows=[r for r in observations if r['requested_vx']==requested];metrics={}
        for name,extract in extractors.items():
            by_run={}
            for r in rows:
                value=extract(r)
                if value is not None:by_run.setdefault(r['run_id'],[]).append(value)
            values=[mean(v) for v in by_run.values()]
            metrics[name]={'n_runs':len(values),'mean':mean(values) if values else None,
                'std':stdev(values) if len(values)>1 else None,
                'min':min(values) if values else None,'max':max(values) if values else None,
                'range':max(values)-min(values) if values else None,
                'reason':None if len(values)>1 else 'repeatability requires at least two supported independent runs'}
        result.append({'requested_vx':requested,'metrics':metrics})
    return result


def analyze_directories(directories):
    observations=[];inputs=[];seen=set();sessions=set();contract=None
    for directory in directories:
        manifest=json.loads((directory/'manifest.json').read_text());raw=directory/'events.jsonl'
        status=manifest.get('termination',{});status=status.get('final_status') or status.get('status')
        if status!='completed' or manifest.get('cleanup',{}).get('passed') is not True:
            raise ValueError(f'input is not a completed, cleaned velocity experiment: {directory}')
        h=digest(raw);session=manifest.get('owner_token_sha256')
        if h in seen or (session and session in sessions):raise ValueError('duplicate raw capture/session is not an independent run')
        seen.add(h)
        if session:sessions.add(session)
        hashes=manifest.get('source_hashes') or {}
        signature={'hashes':hashes,'fixture':{k:v for k,v in manifest.get('fixture',{}).items()
            if k not in ('generated_world','source_world')},'policy':manifest.get('policy_sha256')}
        if contract is not None and signature!=contract:raise ValueError('incompatible policy/control/geometry/fixture contracts; do not pool these runs')
        contract=signature
        snapshots={str(p):digest(p) for p in directory.rglob('*') if p.is_file()}
        events=[json.loads(line) for line in raw.read_text().splitlines() if line.strip()]
        observations.extend(extract_observations(events,manifest,h))
        inputs.append({'directory':str(directory),'raw_sha256':h,'file_hashes':snapshots})
    truth=[r for r in observations if r['valid_for_fit']]
    slam=[{**r,'v':r['slam_v']} for r in observations if r['valid_for_slam_fit']]
    return {'schema_version':1,'offline_only':True,'inputs':inputs,'observations':observations,
        'truth':compare_models(truth),'slam':compare_models(slam),'metric_repeatability':metric_repeatability(observations),
        'zero_state':'f(0)=0 is explicit; zero windows are not moving fit points or evidence for the unmeasured gap to the lowest knot',
        'production_calibration':'deferred; no production command or file changed by this tool'}


def markdown(result):
    lines=['# Offline velocity calibration diagnostic','',
        'No production model selected. Fresh USER-RUN repeats and held-out validation are required.',
        'Residual sign: observed minus predicted. Zero is explicit; no extrapolation or zero-to-first-knot bridge.',
        'R² is descriptive only. Single-run standard deviation is null. Endpoint LOSO is unsupported.',
        'Full per-wheel metrics, coverage, repeat statistics, domains, inverse diagnostics and exclusions are in calibration.json.', '',
        '| Request | Policy obs[6] | Wheel target mean | Wheel dq mean | r target | r actual | Truth | SLAM | Truth−r actual |',
        '|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    def fmt(v):return 'null' if v is None else f'{v:.6f}'
    for r in result['observations']:
        w=r['wheel'];lines.append('| '+' | '.join(fmt(v) for v in [r['requested_vx'],r['u'],w['target_mean_radps'],w['actual_mean_radps'],w['target_kinematic_mps'],w['actual_kinematic_mps'],r['v'],r['slam_v'],r['truth_minus_actual_wheel_mps']])+' |')
    for source in ('truth','slam'):
        group=result[source];lines+=['',f"{source}: {group['independent_runs']} independent runs; {group['distinct_speeds']} distinct policy inputs.",'',
            '| Model | Parameters / knots | MAE | RMSE | Max error | R² | LOSO MAE / supported / unsupported | Monotonic | Moving input / output domain |',
            '|---|---|---:|---:|---:|---:|---|---|---|']
        for kind,m in group['models'].items():
            e=m['metrics'];l=m['loso'];lines.append('| '+' | '.join([kind,json.dumps(m['knots'] or m['parameters']),*[fmt(e[k]) for k in ('mae','rmse','max_abs_error','r2')],f"{fmt(l['mae'])} / {l['predicted_count']} / {l['unsupported_count']}",str(m['monotonic']),f"{m['input_domain']['moving']} / {m['output_domain']['moving']}"])+' |')
        lines+=['','Repeatability (sample std across independent run means):','',
            '| Policy input | Runs | Mean | Std | Range |','|---:|---:|---:|---:|---:|']
        for r in group['repeatability']:lines.append('| '+' | '.join([fmt(r['u']),str(r['n_runs']),fmt(r['mean']),fmt(r['std']),fmt(r['range'])])+' |')
        lines+=['','Signed residual means by speed:','']
        for kind,m in group['models'].items():lines.append(f"- {kind}: "+', '.join(f"{fmt(r['u'])}: {fmt(r['mean_signed_residual'])}" for r in m['residuals_by_speed']))
    lines+=['','Exclusions: '+json.dumps([{'run':r['run_id'],'trial':r['trial'],'truth_reasons':r['invalid_reasons'],'slam_reasons':r['slam_invalid_reasons']} for r in result['observations'] if r['invalid_reasons'] or r['slam_invalid_reasons']]),'']
    return '\n'.join(lines)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dirs',nargs='+',type=Path)
    parser.add_argument('--output-dir',required=True,type=Path,help='new directory outside every input run; existing paths rejected')
    args=parser.parse_args(argv)
    try:
        dirs=[p.expanduser().resolve() for p in args.run_dirs];output=args.output_dir.expanduser().resolve()
        if output.exists() or any(output==p or p in output.parents for p in dirs):
            raise ValueError('output must be a fresh directory outside all input runs')
        result=analyze_directories(dirs)
        for record in result['inputs']:
            if record['file_hashes']!={str(p):digest(p) for p in Path(record['directory']).rglob('*') if p.is_file()}:
                raise RuntimeError('input files changed during offline analysis')
        output.mkdir(parents=True,exist_ok=False)
        (output/'calibration.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        (output/'report.md').write_text(markdown(result))
        columns=['run_id','trial','requested_vx','u','v','slam_v','valid_for_fit','invalid_reasons','wheel']
        with (output/'observations.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=columns);writer.writeheader()
            for row in result['observations']:writer.writerow({k:json.dumps(row[k],allow_nan=False) if isinstance(row[k],(dict,list)) else row[k] for k in columns})
        print(json.dumps({'output_directory':str(output),'observations':len(result['observations']),
            'truth_independent_runs':result['truth']['independent_runs'],'raw_inputs_unchanged':True,'production_model_selected':False}))
        return 0
    except (OSError,ValueError,KeyError,RuntimeError) as exc:
        print(f'offline calibration failed: {exc}',file=sys.stderr);return 1


if __name__=='__main__':raise SystemExit(main())
