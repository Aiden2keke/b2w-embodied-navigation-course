#!/usr/bin/env python3
"""Route-relevant low-speed transition diagnostic; default OFFLINE plan only.

ONE fresh USER-RUN session uses the existing raw -> unchanged smoother ->
default-off adapter -> unchanged rl_sar chain. This characterizes commands below
B support; it never fits an inverse, joins zero to a knot or authorizes routing.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import re
import struct
import time
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'course_navigation'))
from course_navigation import m2_inverse as m2
from scripts import run_velocity_steps as base
from scripts.velocity_step_core import make_sequence, analyze_run, _event_time, _path_metrics
from scripts.fit_velocity_calibration import authenticate, extract_observations, series, window_stats

SPEEDS=[0.,.005,.010,.015,.020,.030,.040,.010,.040]
PROTOCOL='route-low-speed-transition-v1'
INTERPRETATIONS=['A_CONTINUOUS_BRANCH_COMPATIBLE_ON_TESTED_GRID',
                 'B_DEAD_ZONE_MINIMUM_MOVING_SPEED_EVIDENCE','C_UNSUITABLE_OR_INSUFFICIENT']
GATES={'truth_stop_speed_mps':.005,'truth_stop_quiet_duration_s':.5,'truth_stop_yaw_rate_max_radps':.05,'steady_coverage_min':.95,'steady_abs_slope_max_mps2':.02,
       'zero_reference_abs_vx_max_mps':.005,'moving_resolution_mps':.005,
       'repeat_anchor_abs_difference_max_mps':.010,'resolution_monotonic_tolerance_mps':.005,
       'near_zero_moving_max_mps':.020,'adjacent_observed_jump_max_mps':.040,
       'B_endpoint_reference_vx_mps':.1068173727577565,'B_endpoint_difference_max_mps':.010,
       'target_lateral_max_m_per_5s':.03,'target_yaw_max_rad_per_5s':.05,
       'final_zero_to_stop_max_s':.8,'final_zero_to_stop_path_max_m':.05}
SOURCE_FILES=['scripts/low_speed_transition.py','scripts/run_velocity_steps.py','scripts/velocity_step_core.py',
              'scripts/velocity_step_recorder.py','scripts/m2_open_space.py',
              'course_navigation/course_navigation/velocity_adapter.py','course_navigation/course_navigation/m2_inverse.py']


def source_hashes():return {name:base.sha256(ROOT/name) for name in SOURCE_FILES}


def idle_hold_audit():
    import yaml
    cpp_path=ROOT/'b2w_sim/b2w_controllers/src/rl_sar_controller.cpp'
    config_path=ROOT/'course_sim/config/rl_sar.yaml';cpp=cpp_path.read_text()
    thresholds=re.findall(r'std::abs\(command_\[(\d)\]\)<([.\d]+)',cpp)
    if [i for i,_ in thresholds]!=['0','1','2'] or len({v for _,v in thresholds})!=1:
        raise ValueError('idle_hold strict command threshold semantics changed; re-audit required')
    threshold=float(thresholds[0][1]);enabled=yaml.safe_load(config_path.read_text())['b2w_rl_sar_controller']['ros__parameters']['idle_hold']
    required=['std::array<double,3> gyro_{},command_{}','std::array<float,57> obs{};',
              'obs[6+i]=command_valid?command_[i]:0;', 'if (idle && i>=12) last_[i]=0;',
              'm.data.assign(obs.begin(),obs.end()); debug_->publish(m);']
    if not all(s in cpp for s in required):raise ValueError('controller debug/idle layout changed; re-audit required')
    tick_ms=int(re.search(r'std::chrono::milliseconds\((\d+)\)',cpp).group(1))
    decimation=int(re.search(r'tick_%(\d+)==0',cpp).group(1))
    float32=struct.unpack('f',struct.pack('f',.005))[0]
    return {'source':str(cpp_path),'source_sha256':base.sha256(cpp_path),
            'config_sha256':base.sha256(config_path),'enabled':enabled,'strict_less_than':threshold,
            'command_space_before_float32':True,'point_005_idle':enabled and .005<threshold,
            'point_010_idle':enabled and .010<threshold,'point_005_float32':float32,
            'point_005_command_distinguishable_from_zero':float32!=0.,
            'physical_distinguishability':'unknown until truth measurement; idle wheel actions zero, learned legs retained',
            'wheel_override':'idle replaces only wheel actions with exact zero; body zero response is not assumed',
            'command_hz':base.request_rate_hz(),'controller_hz':1000./tick_ms,
            'policy_hz':1000./(tick_ms*decimation),'fixed_steady_window_s':2.,
            'nominal_command_samples_per_steady_window':2.*base.request_rate_hz(),
            'nominal_policy_samples_per_steady_window':2000./(tick_ms*decimation)}


def plan():
    p={'protocol':PROTOCOL,'policy_inputs':list(SPEEDS),
       'sequence':make_sequence(SPEEDS,transition_diagnostic=True),
       'validated_condition':'pure_forward_vy0_wz0','m2_inverse_enabled':False,
       'idle_hold_audit':idle_hold_audit(),'legacy_guard':{'kind':'experiment_range_not_safety_threshold',
          'normal_min_policy_input':.04,'diagnostic_extension':'explicit transition_diagnostic only; default guard unchanged'},
       'gates':dict(GATES),'interpretations':list(INTERPRETATIONS),
       'repeat_scope':'two same-session anchors at .010 and .040; not independent-session repeatability',
       'refit_performed':False,'production_selection':None,'production_authorized':False,
       'bridge_selected':False,'continuity_proven':False,'source_hashes':source_hashes(),
       'chain':['requested policy u',base.RAW_TOPIC,'unchanged OPEN_LOOP smoother',base.SHARED_TOPIC,
                'adapter m2_inverse_enabled=false',base.FINAL_TOPIC,'actual Float32 ONNX obs[6:9]','wheel targets/actual/torques','truth and SLAM evaluation'],
       'data_contract':'all nine trials valid, source/capture authenticated, exact schedule, pure-forward plateaus, truth+SLAM steady coverage >=95% with slope <=.02, required native policy/wheel/clock streams, successful owned cleanup; no exclusions',
       'inference_limit':'classifications describe the sampled grid only; no proof of arbitrary continuity, global minimum attainable speed or production command semantics'}
    return p


def interpret(points,data_valid):
    result={'classification':INTERPRETATIONS[2],'production_authorized':False,'bridge_selected':False,
            'continuity_proven':False,'minimum_observed_moving_vx':None,
            'note':'One session and finite input grid; minimum observed moving speed is not a proven global attainable minimum.'}
    if not data_valid or len(points)!=len(SPEEDS) or [p.get('requested_u') for p in points]!=SPEEDS:
        return {**result,'reason':'missing/invalid data or protocol mismatch'}
    if any(not p.get('valid') or not p.get('stable') or not isinstance(p.get('truth_steady_vx'),(int,float)) or not math.isfinite(p['truth_steady_vx']) for p in points):
        return {**result,'reason':'unstable/invalid trial'}
    values=[p['truth_steady_vx'] for p in points];baseline=values[0]
    if abs(baseline)>GATES['zero_reference_abs_vx_max_mps']:
        return {**result,'reason':'zero reference not quiet'}
    if abs(values[2]-values[7])>GATES['repeat_anchor_abs_difference_max_mps'] or abs(values[6]-values[8])>GATES['repeat_anchor_abs_difference_max_mps']:
        return {**result,'reason':'same-session anchor mismatch'}
    if any(abs(values[i]-GATES['B_endpoint_reference_vx_mps'])>GATES['B_endpoint_difference_max_mps'] for i in (6,8)):
        return {**result,'reason':'B lower-endpoint response not reproduced; changed operating condition or unsuitable response'}
    distinct=values[:7];adjusted=[v-baseline for v in distinct]
    if any(v < -GATES['moving_resolution_mps'] for v in adjusted) or any(b<a-GATES['resolution_monotonic_tolerance_mps'] for a,b in zip(distinct,distinct[1:])):
        return {**result,'reason':'reverse/nonmonotonic response beyond resolution tolerance'}
    moving=[v for v in adjusted[1:] if v>GATES['moving_resolution_mps']]
    if not moving:return {**result,'reason':'no resolved stable moving branch'}
    minimum=min(moving)
    result['minimum_observed_moving_vx']=min(v for v,difference in zip(distinct[1:],adjusted[1:]) if difference>GATES['moving_resolution_mps'])
    result['minimum_observed_moving_increment']=minimum
    result['observed_monotonic']=all(b>=a for a,b in zip(distinct,distinct[1:]))
    result['resolution_monotonic']=True
    if minimum<=GATES['near_zero_moving_max_mps'] and max(b-a for a,b in zip(adjusted,adjusted[1:]))<=GATES['adjacent_observed_jump_max_mps']:
        return {**result,'classification':INTERPRETATIONS[0],'reason':'resolved near-zero moving samples with bounded adjacent jumps; only compatible with continuity on this grid'}
    first=next(i for i,v in enumerate(adjusted[1:],1) if v>GATES['moving_resolution_mps'])
    if first>=2 and all(abs(v)<=GATES['moving_resolution_mps'] for v in adjusted[:first]) and minimum>=GATES['near_zero_moving_max_mps']:
        return {**result,'classification':INTERPRETATIONS[1],
                'reason':'observed dead-zone then minimum sampled moving speed; discrete stop/move semantics may be necessary, but require their own validation'}
    return {**result,'reason':'sampled response does not resolve a suitable transition interpretation'}


def report(events,manifest):
    expected=plan();reasons=[]
    if manifest.get('transition_protocol')!=expected:reasons.append('preregistered protocol/source identity mismatch')
    if manifest.get('transition_source_hashes')!=source_hashes():reasons.append('transition source hashes mismatch')
    if manifest.get('sequence')!=expected['sequence']:reasons.append('sequence mismatch')
    if manifest.get('m2_inverse_enabled') is not False:reasons.append('inverse disabled state not authenticated')
    termination=manifest.get('termination') or {}
    if termination.get('status')!='completed' or termination.get('final_status',termination.get('status'))!='completed':reasons.append('run incomplete')
    if not manifest.get('cleanup',{}).get('passed'):reasons.append('cleanup failed')
    readiness=manifest.get('readiness',{})
    if readiness.get('completed') is not True or not base.readiness_complete(readiness.get('checks',{})):reasons.append('readiness incomplete')
    nav_key=str(base.NAV_CONFIG.relative_to(ROOT))
    if manifest.get('source_hashes',{}).get(nav_key)!=base.sha256(base.NAV_CONFIG):reasons.append('smoother/Nav2 YAML not authenticated')
    if not base.POLICY.is_file() or manifest.get('policy_sha256')!=base.sha256(base.POLICY):reasons.append('deployed ONNX policy not authenticated')
    if manifest.get('truth_stop_speed_mps')!=GATES['truth_stop_speed_mps']:reasons.append('low-speed stopping resolution mismatch')
    auth=authenticate(manifest)
    if not auth['valid']:reasons+=auth['reasons']
    analysis=analyze_run(events,{**manifest,'sequence':expected['sequence'],'truth_stop_speed_mps':GATES['truth_stop_speed_mps']})
    q=analysis['quality']
    if q['clock_epoch_crossing'] or q['clock_rollback'] or q['nonfinite_event_count'] or q['missing_topics']:reasons.append('clock/finite/native streams invalid')
    required=(base.REQUEST_TOPIC,base.RAW_TOPIC,base.SHARED_TOPIC,base.FINAL_TOPIC,base.TRUTH_TOPIC,
              base.SLAM_TOPIC,'/clock','/network_input_debug','/joint_commands','/joint_torques','/sim/raw/joint_states')
    if any(not any(e.get('topic')==topic for e in events) for topic in required):reasons.append('required native instrumentation missing')
    for event in events:
        if event.get('topic') not in (base.TRUTH_TOPIC,base.SLAM_TOPIC):continue
        d=event.get('data',{});stamp=event.get('source_s');quat=d.get('quaternion_xyzw',[]);pos=d.get('position',[])
        if not isinstance(stamp,(float,int)) or not math.isfinite(stamp) or len(quat)!=4 or len(pos)!=3 or any(v is None or not math.isfinite(v) for v in [*quat,*pos]) or abs(sum(v*v for v in quat)-1.)>.01 or event.get('child_frame_id')!='base_link':
            reasons.append('odometry source stamp/pose/frame invalid');break
    origin=manifest.get('experiment_start_sim_s')
    if origin is None:reasons.append('missing experiment origin')
    else:
        for topic,extract,gap in [(base.REQUEST_TOPIC,lambda d:d['linear_x'],.075),(base.RAW_TOPIC,lambda d:d['linear_x'],.075),(base.SHARED_TOPIC,lambda d:d['linear_x'],.075),
              (base.FINAL_TOPIC,lambda d:d['linear_x'],.075),('/network_input_debug',lambda d:d['data'][6],.05),('/clock',lambda d:d['clock_s'],.15),('/joint_torques',lambda d:d['data'][0] if len(d['data'])==16 and all(v is not None and math.isfinite(v) for v in d['data']) else None,.05)]:
            if window_stats(series(events,topic,extract),origin,origin+108.,gap=gap,zoh=True)['mean'] is None:reasons.append('full protocol coverage missing: '+topic)
        for event in events:
            topic=event.get('topic');t=_event_time(event)
            if t is None or not origin<=t<origin+108.:continue
            d=event.get('data',{})
            if topic in (base.RAW_TOPIC,base.SHARED_TOPIC,base.FINAL_TOPIC):vals=[d.get('linear_x'),d.get('linear_y'),d.get('angular_z')]
            elif topic=='/network_input_debug':vals=(d.get('data') or [])[6:9]
            else:continue
            if len(vals)!=3 or any(v is None or not math.isfinite(v) for v in vals) or not 0<=vals[0]<=.04+1e-8 or vals[1]!=0 or vals[2]!=0:
                reasons.append('command sign/range/pure-forward corruption');break
    observations=extract_observations(events,{**manifest,'sequence':expected['sequence'],'truth_stop_speed_mps':GATES['truth_stop_speed_mps']},PROTOCOL)
    points=[]
    for row,metrics in zip(observations,analysis['per_speed']):
        errors=list(row['invalid_reasons'])+list(row['slam_invalid_reasons']);wheel=row['wheel']
        if wheel['reason'] is not None:errors.append('wheel instrumentation: '+wheel['reason'])
        for name,stats in row['pipeline'].items():
            target=row['requested_vx']
            if stats['min'] is None or stats['max'] is None or max(abs(stats['min']-target),abs(stats['max']-target))>1e-6:
                errors.append('pipeline plateau mismatch: '+name)
        if row['torque_peak_limit_ratio'] is None:errors.append('torque plateau instrumentation missing')
        start,end=row['window_s']
        tq=window_stats(series(events,'/joint_torques',lambda d:d['data'][0] if len(d['data'])==16 and all(v is not None and math.isfinite(v) for v in d['data']) else None),start,end,gap=.05,zoh=True)
        if tq['mean'] is None:errors.append('torque steady coverage <95%')
        coupling=row['coupling']['target_phase'];stop=dict(row['stop'])
        if coupling['coverage_fraction']<GATES['steady_coverage_min']:errors.append('target pose coverage <95%')
        def within(v,bound):return v is not None and math.isfinite(v) and abs(v)<=bound
        stop_path=None;fz=stop['final_nav_zero_s'];ts=stop['truth_effective_stop_s']
        if fz is not None and ts is not None and ts>=fz:
            measured=_path_metrics(events,base.TRUTH_TOPIC,fz,ts,.15)
            if ts==fz and any(e.get('topic')==base.TRUTH_TOPIC and _event_time(e)==ts and e.get('valid',True) and e.get('data',{}).get('position') for e in events):
                measured={'path_distance_m':0.,'coverage_fraction':1.}
            stop_path=measured['path_distance_m'];stop['final_zero_to_stop_pose_coverage']=measured['coverage_fraction']
            if measured['coverage_fraction']<GATES['steady_coverage_min']:errors.append('stopping pose coverage <95%')
        stable=within(coupling['lateral_displacement_m'],GATES['target_lateral_max_m_per_5s']) and within(coupling['yaw_drift_rad'],GATES['target_yaw_max_rad_per_5s']) and within(stop['final_zero_to_truth_stop_s'],GATES['final_zero_to_stop_max_s']) and stop.get('final_zero_to_truth_stop_s',-1) is not None and stop['final_zero_to_truth_stop_s']>=0 and within(stop_path,GATES['final_zero_to_stop_path_max_m'])
        idle=expected['idle_hold_audit']['enabled'] and row['requested_vx']<expected['idle_hold_audit']['strict_less_than']
        if idle and any(q['min']!=0. or q['max']!=0. for q in wheel.get('target_quality',[])):errors.append('idle wheel-action zero override not observed')
        points.append({'trial':row['trial'],'requested_u':row['requested_vx'],'actual_policy_u':row['u'],
             'truth_steady_vx':row['v'],'slam_steady_vx':row['slam_v'],'valid':not errors,'invalid_reasons':errors,
             'stable':bool(stable),'idle_hold_expected':idle,'policy':row['policy_command'],'pipeline':row['pipeline'],
             'wheel':wheel,'torque_steady_quality':tq,'torque_peak_limit_ratio':row['torque_peak_limit_ratio'],
             'wheel_torque_peak_limit_ratio':row['wheel_torque_peak_limit_ratio'],'target_phase':coupling,'stop':{**stop,'final_zero_to_stop_path_distance_m':stop_path}})
    data_valid=not reasons and len(points)==len(SPEEDS) and all(p['valid'] for p in points)
    return {'protocol':PROTOCOL,'validated_condition':'pure_forward_vy0_wz0','data_valid':data_valid,
            'data_invalid_reasons':reasons,'points':points,'preregistered_gates':dict(GATES),
            'interpretation':interpret(points,data_valid),'refit_performed':False,'production_selection':None,
            'NORMAL_NAV2_M2_READY':False,'candidate_open_space_deferred':True}


def flush_zero(node,executor,runtime):
    """Return errors so guaranteed process teardown cannot be bypassed."""
    try:
        if node is not None and executor is not None and runtime and runtime.ok():
            node.zero_only=True
            until=time.monotonic()+base.FLUSH_S
            while time.monotonic()<until:
                node.publish_zero();executor.spin_once(timeout_sec=.05)
    except BaseException as exc:
        return f'{type(exc).__name__}: {exc}'
    return None


def analyze_directory(directory):
    d=Path(directory).resolve();raw=d/'events.jsonl';manifest=json.loads((d/'manifest.json').read_text())
    if base.sha256(raw)!=manifest.get('transition_events_sha256'):raise ValueError('raw capture SHA256 mismatch')
    result=report(base._read_events(raw),manifest)
    base._write_json(d/'low-speed-transition-report.json',result)
    trials=d/'transition-trials';trials.mkdir(exist_ok=True)
    for point in result['points']:base._write_json(trials/f"trial-{point['trial']:02d}.json",point)
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);g=p.add_mutually_exclusive_group()
    g.add_argument('--execute',action='store_true');g.add_argument('--analyze',type=Path)
    p.add_argument('--output-root',type=Path,default=Path('/tmp/b2w-user-low-speed-transition-01'))
    p.add_argument('--ros-domain-id',type=int,default=196);p.add_argument('--no-gui',action='store_true');a=p.parse_args(argv)
    if a.analyze:
        result=analyze_directory(a.analyze);print(json.dumps(result,indent=2));return 0 if result['data_valid'] else 1
    protocol=plan()
    if not a.execute:print(json.dumps(protocol,indent=2));return 0
    root=a.output_root.expanduser().resolve();before=set(root.iterdir()) if root.exists() else set()
    args=SimpleNamespace(output_root=str(root),ros_domain_id=a.ros_domain_id,gui=not a.no_gui,
                         speeds=tuple(SPEEDS),readiness_smoke=False,transition_diagnostic=True)
    status=1
    try:status=base.run_experiment(args)
    finally:
        created=[d for d in (set(root.iterdir())-before if root.exists() else set()) if d.is_dir()]
        if len(created)==1:
            d=created[0];mfile=d/'manifest.json';raw=d/'events.jsonl'
            if mfile.exists() and raw.exists():
                m=json.loads(mfile.read_text());m['transition_events_sha256']=base.sha256(raw);base._write_json(mfile,m)
                result=analyze_directory(d)
                if not result['data_valid']:status=1
                print(json.dumps({'run_directory':str(d),'data_valid':result['data_valid'],
                     'interpretation':result['interpretation'],'NORMAL_NAV2_M2_READY':False},indent=2))
            else:status=1
        else:status=1
    return status


if __name__=='__main__':raise SystemExit(main())
