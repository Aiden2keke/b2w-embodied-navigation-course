"""Pure sequence and offline metric analysis for B2-W velocity steps.

This module intentionally has no ROS imports. Native events use simulation
receive time for unstamped commands and source header time for stamped odometry.
"""
from __future__ import annotations

import math
from statistics import mean
from typing import Any

DEFAULT_SPEEDS = (.10, .20, .30, .40)
DEFAULT_GAPS = {'truth': .15, 'slam': .35, 'command': .15}


def make_sequence(speeds=DEFAULT_SPEEDS, settle_s: float = 3., hold_s: float = 5., stop_s: float = 4., *, transition_diagnostic: bool = False) -> list[dict[str, Any]]:
    values = tuple(float(v) for v in speeds)
    durations = (float(settle_s), float(hold_s), float(stop_s))
    if transition_diagnostic:
        # Separate opt-in experiment range; the normal A/B/C guard stays intact.
        if not values or not all(math.isfinite(v) and 0. <= v <= .04 for v in values):
            raise ValueError('transition diagnostic inputs must be finite in [0, 0.04]')
    elif not values or not all(math.isfinite(v) and .04 <= v <= .40 for v in values):
        raise ValueError('speeds must be finite positive experiment inputs in [0.04, 0.40] m/s')
    if not transition_diagnostic and len(set(values)) != len(values):
        raise ValueError('duplicate speed targets are not allowed')
    if not all(math.isfinite(d) and d > 0 for d in durations):
        raise ValueError('all phase durations must be finite and positive')
    result = []
    cursor = 0.
    for trial, vx in enumerate(values, 1):
        for phase, duration, command in (
                ('settle', durations[0], 0.), ('target', durations[1], vx), ('stop', durations[2], 0.)):
            result.append({'trial': trial, 'phase': phase, 'start_s': cursor,
                           'end_s': cursor + duration, 'vx': command, 'vy': 0., 'wz': 0.})
            cursor += duration
    return result


def requested_at(sequence: list[dict], elapsed_s: float) -> tuple[float, float, float]:
    if not math.isfinite(float(elapsed_s)):
        return 0., 0., 0.
    for phase in sequence:
        if phase['start_s'] <= elapsed_s < phase['end_s']:
            return float(phase['vx']), float(phase.get('vy', 0.)), float(phase.get('wz', 0.))
    return 0., 0., 0.


def unwrap_yaw(values: list[float]) -> list[float]:
    """Unwrap finite yaw samples along shortest-angle increments; retain NaNs."""
    result: list[float] = []
    offset = 0.
    previous_raw = None
    previous_unwrapped = None
    for value in values:
        value = float(value)
        if not math.isfinite(value):
            result.append(float('nan'))
            previous_raw = previous_unwrapped = None
            offset = 0.
            continue
        if previous_raw is not None:
            delta = (value - previous_raw + math.pi) % (2 * math.pi) - math.pi
            offset = previous_unwrapped + delta - value
        current = value + offset
        result.append(current)
        previous_raw, previous_unwrapped = value, current
    return result


def _finite(value: Any) -> bool:
    if isinstance(value, bool) or value is None:
        return True
    if isinstance(value, (int, float)):
        return math.isfinite(float(value))
    if isinstance(value, dict):
        return all(_finite(v) for v in value.values())
    if isinstance(value, (tuple, list)):
        return all(_finite(v) for v in value)
    return True


def _event_time(event: dict) -> float | None:
    value = event.get('source_s')
    if value is None:
        value = event.get('sim_s')
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _stream(events: list[dict], topic: str, field: str, max_gap: float, component: int | None = None) -> list[tuple[float, float]]:
    rows = []
    for event in events:
        if event.get('topic') != topic or not event.get('valid', True):
            continue
        t = _event_time(event)
        data = event.get('data') or {}
        value = data.get(field)
        if component is not None and isinstance(value, (tuple, list)) and len(value) > component:
            value = value[component]
        try:
            value = float(value)
        except (TypeError, ValueError):
            continue
        if t is not None and math.isfinite(value):
            rows.append((t, value))
    rows.sort(key=lambda item: item[0])
    deduped = []
    for row in rows:
        if deduped and row[0] == deduped[-1][0]:
            deduped[-1] = row
        else:
            deduped.append(row)
    return deduped


def _clip_segments(rows: list[tuple[float, float]], start: float, end: float, max_gap: float):
    """Yield linearly interpolated supported subsegments clipped to a window."""
    for (ta, va), (tb, vb) in zip(rows, rows[1:]):
        if tb <= ta or tb - ta > max_gap:
            continue
        # Phase/window ends are half-open.  A sample stamped exactly at the
        # transition may already contain the next phase's value; do not
        # interpolate backward across that discontinuity.
        if tb >= end:
            continue
        lo, hi = max(start, ta), min(end, tb)
        if hi <= lo:
            continue
        slope = (vb - va) / (tb - ta)
        yield lo, va + slope * (lo - ta), hi, va + slope * (hi - ta)


def _integral(rows: list[tuple[float, float]], start: float, end: float, max_gap: float):
    area = coverage = 0.
    for a, va, b, vb in _clip_segments(rows, start, end, max_gap):
        dt = b - a
        area += (va + vb) * .5 * dt
        coverage += dt
    return area, min(1., coverage / (end - start)) if end > start else 0.


def _zoh_integral(rows: list[tuple[float, float]], start: float, end: float, max_gap: float):
    """Integrate received Twist values as zero-order holds, with stale gaps omitted."""
    area = covered = 0.
    for (ta, value), (tb, _) in zip(rows, rows[1:]):
        if tb <= ta or tb - ta > max_gap or tb <= start or ta >= end:
            continue
        lo, hi = max(start, ta), min(end, tb)
        if hi > lo:
            area += value * (hi - lo)
            covered += hi - lo
    return area, min(1., covered / (end - start)) if end > start else 0.


def _window_mean(rows, start, end, max_gap):
    area, coverage = _integral(rows, start, end, max_gap)
    return (area / ((end - start) * coverage), coverage) if coverage > 0 else (None, 0.)


def _window_points(rows, start, end):
    return [(t, v) for t, v in rows if start <= t < end]


def _slope(points):
    if len(points) < 2:
        return None
    mt = mean(t for t, _ in points); mv = mean(v for _, v in points)
    denom = sum((t - mt) ** 2 for t, _ in points)
    return sum((t - mt) * (v - mv) for t, v in points) / denom if denom else None


def _threshold_time(rows, start, end, threshold, max_gap):
    for a, va, b, vb in _clip_segments(rows, start, end, max_gap):
        if va >= threshold:
            return a
        if va < threshold <= vb and vb != va:
            return a + (threshold - va) * (b - a) / (vb - va)
    return None


def _settling(rows, start, end, target, band, max_gap):
    phase_rows = [(t,v) for t,v in rows if start <= t < end]
    if not phase_rows or end-phase_rows[-1][0]>max_gap:
        return None
    supported_end=phase_rows[-1][0]
    candidates = [t for t, _ in phase_rows]
    for t in candidates:
        if supported_end - t < .5:
            continue
        segments = list(_clip_segments(phase_rows, t, supported_end+1e-9, max_gap))
        covered = sum(b - a for a, _, b, _ in segments)
        if covered < supported_end - t - 1e-8:
            continue
        if all(abs(va - target) <= band+1e-12 and abs(vb - target) <= band+1e-12 for _, va, _, vb in segments):
            return t
    return None


def _interp(rows, t, max_gap):
    for a, va, b, vb in _clip_segments(rows, t - 1e-12, t + 1e-12, max_gap):
        if a <= t <= b:
            return va + (vb - va) * ((t - a) / (b - a))
    for ta, va in rows:
        if abs(ta - t) < 1e-9:
            return va
    return None


def _path_metrics(events, topic, start, end, max_gap):
    poses=[]
    for e in events:
        if e.get('topic') != topic or not e.get('valid', True):
            continue
        t=_event_time(e); data=e.get('data') or {}
        pos=data.get('position'); quat=data.get('quaternion_xyzw')
        if t is None or not isinstance(pos,(list,tuple)) or len(pos)<2 or not isinstance(quat,(list,tuple)) or len(quat)<4:
            continue
        try:
            x,y=float(pos[0]),float(pos[1]); qx,qy,qz,qw=map(float,quat[:4])
        except (TypeError,ValueError):
            continue
        if not all(map(math.isfinite,(x,y,qx,qy,qz,qw))):
            continue
        yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        poses.append((t,x,y,yaw))
    poses.sort(key=lambda x:x[0]); dedup=[]
    for row in poses:
        if dedup and row[0]==dedup[-1][0]: dedup[-1]=row
        else: dedup.append(row)
    def at_time(target):
        for pose in dedup:
            if abs(pose[0] - target) < 1e-9:
                return pose
        for a, b in zip(dedup, dedup[1:]):
            if a[0] < target < b[0] and b[0] - a[0] <= max_gap:
                f = (target-a[0])/(b[0]-a[0])
                yaw_b = unwrap_yaw([a[3], b[3]])[1]
                return (target, a[1]+f*(b[1]-a[1]), a[2]+f*(b[2]-a[2]), a[3]+f*(yaw_b-a[3]))
        return None
    first_at, last_at = at_time(start), at_time(end)
    points=[]
    inside=[p for p in dedup if start < p[0] < end]
    if first_at is not None and last_at is not None:
        clipped=[first_at, *inside, last_at]
        points=[(a,b) for a,b in zip(clipped,clipped[1:]) if 0 < b[0]-a[0] <= max_gap]
    if not points: return {'path_distance_m':None,'net_displacement_m':None,'lateral_displacement_m':None,'yaw_drift_rad':None,'coverage_fraction':0.}
    first=points[0][0]; last=points[-1][1]; path=0.
    for a,b in points: path += math.hypot(b[1]-a[1],b[2]-a[2])
    dx,dy=last[1]-first[1],last[2]-first[2]
    lateral=-math.sin(first[3])*dx+math.cos(first[3])*dy
    yaw=unwrap_yaw([p[3] for pair in points for p in pair])
    return {'path_distance_m':path,'net_displacement_m':math.hypot(dx,dy),
            'lateral_displacement_m':lateral,'yaw_drift_rad':yaw[-1]-yaw[0],
            'coverage_fraction':min(1.,sum(b[0]-a[0] for a,b in points)/(end-start)) if end>start else 0.}


def _first_zero(rows, start, end, epsilon=1e-6, confirm=.15, max_gap=.15):
    for i,(t,v) in enumerate(rows):
        if not start <= t < end or abs(v)>epsilon: continue
        previous=t
        for tt,vv in rows[i:]:
            if tt>=end or tt-previous>max_gap+1e-9 or abs(vv)>epsilon:
                break
            if tt-t>=confirm-1e-9:
                return t
            previous=tt
    return None


def _event_receive_zero(events, topic, start, end, epsilon=1e-6):
    zeros=[]
    for e in events:
        if e.get('topic') != topic or not e.get('valid', True):
            continue
        t=e.get('sim_s'); data=e.get('data') or {}
        try: values=[float(data.get(k,0.)) for k in ('linear_x','linear_y','angular_z')]
        except (TypeError,ValueError): continue
        if t is not None and start <= float(t) < end and all(math.isfinite(v) and abs(v)<=epsilon for v in values):
            zeros.append(float(t))
    return min(zeros) if zeros else None


def _first_nonzero(rows, start, end, epsilon=1e-6):
    return next((t for t, v in rows if start <= t < end and abs(v) > epsilon), None)


def _wall_time_for_sim_event(events, topic, sim_s):
    matches=[e for e in events if e.get('topic')==topic and e.get('sim_s')==sim_s]
    return min((e.get('wall_s') for e in matches if e.get('wall_s') is not None),default=None)


def _phase_summary(events, truth_topic, twist_rows, phase_start, phase_end, observed_end, max_gap):
    stop=min(phase_end,observed_end)
    path=_path_metrics(events,truth_topic,phase_start,stop,max_gap)
    truthvy=_stream(events,truth_topic,'linear',max_gap,1)
    truthwz=_stream(events,truth_topic,'angular',max_gap,2)
    vy=[abs(v) for t,v in truthvy if phase_start<=t<=stop]
    wz=[abs(v) for t,v in truthwz if phase_start<=t<=stop]
    path.update({'peak_abs_vy_mps':max(vy) if vy else None,'peak_abs_wz_radps':max(wz) if wz else None,
                 'observed_end_s':stop,'censored':stop<phase_end})
    return path


def analyze_run(events: list[dict], manifest: dict) -> dict:
    sequence=manifest.get('sequence') or make_sequence()
    stop_speed=float(manifest.get('truth_stop_speed_mps',.02))
    if not math.isfinite(stop_speed) or stop_speed<=0:raise ValueError('truth stop speed must be finite and positive')
    # Keep native event stamps untouched; move only the relative schedule into
    # the recorded simulation clock epoch. A smoke has no experiment windows.
    origin=manifest.get('experiment_start_sim_s')
    if origin is None and 'experiment_start_sim_s' in manifest:
        sequence=[]
    else:
        origin=float(origin or 0.)
        sequence=[{**p,'start_s':origin+p['start_s'],'end_s':origin+p['end_s']} for p in sequence]
    topics={
        'raw':manifest.get('raw_topic','/course_navigation/cmd_vel_raw'),
        'shared':manifest.get('shared_topic','/course_navigation/cmd_vel'),
        'final':manifest.get('final_topic','/path_manager/path_manager_ros/nav_vel'),
        'truth':manifest.get('truth_topic','/sim/ground_truth/odom'),
        'slam':manifest.get('slam_topic','/slam/odom'),
        'requested':manifest.get('requested_topic','/harness/requested')}
    epochs={e.get('epoch') for e in events if e.get('epoch') is not None}
    clock_rows=[(_event_time(e),e.get('epoch')) for e in events if e.get('type',e.get('kind'))=='clock' and _event_time(e) is not None]
    rollback=any(clock_rows[i][1]==clock_rows[i-1][1] and clock_rows[i][0]<clock_rows[i-1][0] for i in range(1,len(clock_rows)))
    nonfinite=sum(1 for e in events if not e.get('valid',True) or not _finite(e.get('data')))
    max_gaps={**DEFAULT_GAPS,**(manifest.get('max_gap_s') or {})}
    streams={}
    for name in ('raw','shared','final','requested'):
        streams[name]=_stream(events,topics[name],'linear_x',float(max_gaps.get('command',.15)))
    for name in ('truth','slam'):
        streams[name]={field: _stream(events,topics[name],'linear' if field=='lateral' else field,float(max_gaps[name]),comp)
                       for field,comp in [('linear',0),('lateral',1),('angular',2)]}
    crossing=len(epochs)>1
    per_speed=[]
    for phase in sequence:
        if phase['phase']!='target': continue
        trial=int(phase['trial']); requested=float(phase['vx']); t0=float(phase['start_s']); t1=float(phase['end_s'])
        stop_phase=next(p for p in sequence if p['trial']==trial and p['phase']=='stop')
        stop_start=float(stop_phase['start_s']); stop_end=float(stop_phase['end_s'])
        steady_start=t1-2.
        row={'trial':trial,'requested_vx_mps':requested,'target_window_s':[t0,t1], 'steady_state_window_s':[steady_start,t1]}
        truth=streams['truth']; slam=streams['slam']; outputs={}
        for name,group in [('truth',truth),('slam',slam)]:
            vx=group['linear']; gap=float(max_gaps[name])
            tail,coverage=_window_mean(vx,steady_start,t1,gap)
            slope=_slope(_window_points(vx,steady_start,t1))
            valid=(not crossing and not rollback and tail is not None and coverage>=.95-1e-9 and slope is not None and abs(slope)<=.02)
            baseline,_=_window_mean(vx,max(0.,t0-1.),t0,gap)
            peak_values=[v for t,v in vx if t0<=t<t1]
            peak=max(peak_values) if peak_values else None
            rise10=_threshold_time(vx,t0,t1,baseline+0.1*(requested-baseline),gap) if baseline is not None else None
            rise90=_threshold_time(vx,t0,t1,baseline+0.9*(requested-baseline),gap) if baseline is not None else None
            tail_abs_den=abs(tail-baseline) if tail is not None and baseline is not None else None
            pose=_path_metrics(events,topics[name],t0,t1,float(max_gaps[name]))
            outputs[name]={
                'baseline_vx_mps':baseline,'tail_mean_vx_mps':tail,'steady_state_valid':valid,
                'steady_state_slope_mps2':slope,'steady_state_coverage_fraction':coverage,
                'steady_state_mean_vx_mps':tail if valid else None,
                'steady_state_reason':None if valid else ('clock epoch crossing/rollback' if crossing or rollback else
                    'stream missing or unsupported' if tail is None else 'coverage below 95%' if coverage<.95-1e-9 else
                    'insufficient slope samples' if slope is None else 'steady-state slope exceeds 0.02 m/s^2'),
                'gain_actual_over_requested':tail/requested if valid and requested else None,
                'bias_signed_mps':tail-requested if valid else None,
                'bias_absolute_mps':abs(tail-requested) if valid else None,
                'peak_vx_mps':peak,
                'overshoot_requested_pct':100.*max(0.,peak-requested)/requested if peak is not None and requested else None,
                'overshoot_tail_pct':100.*max(0.,peak-tail)/tail_abs_den if peak is not None and tail is not None and tail_abs_den and tail_abs_den>1e-9 else None,
                'rise_time_10_90_s':rise90-rise10 if rise10 is not None and rise90 is not None else None,
                'rise_10_s':rise10,'rise_90_s':rise90,
                'rise_10_delay_from_request_s':rise10-t0 if rise10 is not None else None,
                'rise_90_delay_from_request_s':rise90-t0 if rise90 is not None else None,
                'settling_time_s':(_settling(vx,t0,t1,requested,max(.02,.05*requested),gap)-t0) if _settling(vx,t0,t1,requested,max(.02,.05*requested),gap) is not None else None,
                'settling_censored':_settling(vx,t0,t1,requested,max(.02,.05*requested),gap) is None,
                'settling_censor_reason':'threshold not maintained for supported 0.5 s within fixed target phase' if _settling(vx,t0,t1,requested,max(.02,.05*requested),gap) is None else None,
                'lateral_displacement_m':pose['lateral_displacement_m'],'yaw_drift_rad':pose['yaw_drift_rad'],
                'path_distance_m':pose['path_distance_m'],'net_displacement_m':pose['net_displacement_m'],
                'peak_abs_vy_mps':max((abs(v) for t,v in group['lateral'] if t0<=t<t1),default=None),
                'peak_abs_wz_radps':max((abs(v) for t,v in group['angular'] if t0<=t<t1),default=None),
                'missing_reason':None if tail is not None else f'{name} stream missing or unsupported',
                'stream_sample_count':len(vx)}
        outputs['truth']['final_command_gain_truth'] = None
        outputs['slam']['final_command_gain_truth'] = None
        final_rows=streams['final']; final_steady,_=_window_mean(final_rows,steady_start,t1,float(max_gaps['command']))
        outputs['truth']['final_command_steady_mean_vx_mps']=final_steady
        truth_tail=outputs['truth']['tail_mean_vx_mps']
        outputs['truth']['final_command_gain_truth']=truth_tail/final_steady if outputs['truth']['steady_state_valid'] and final_steady not in (None,0.) else None
        outputs['truth']['truth_over_final_command_gain']=outputs['truth']['final_command_gain_truth']
        outputs['slam']['final_command_steady_mean_vx_mps']=final_steady
        outputs['slam']['slam_over_final_command_gain']=outputs['slam']['tail_mean_vx_mps']/final_steady if outputs['slam']['steady_state_valid'] and final_steady not in (None,0.) else None
        observed_end=max((t for t,_ in truth['linear'] if stop_start<=t<=stop_end),default=stop_start)
        stopv=truth['linear']; truth_stop=None
        pose_intervals=[]
        poses=[]
        for e in events:
            if e.get('topic')!=topics['truth'] or not e.get('valid',True): continue
            t=_event_time(e); data=e.get('data') or {}; p=data.get('position');q=data.get('quaternion_xyzw')
            if t is not None and p is not None and q is not None and len(p)>=2 and len(q)>=4:
                try:
                    yaw=math.atan2(2*(float(q[3])*float(q[2])+float(q[0])*float(q[1])),1-2*(float(q[1])**2+float(q[2])**2))
                    poses.append((t,float(p[0]),float(p[1]),yaw))
                except (ValueError,TypeError): pass
        poses.sort(); yaws=unwrap_yaw([p[3] for p in poses])
        for i in range(1,len(poses)):
            dt=poses[i][0]-poses[i-1][0]
            if 0<dt<=float(max_gaps['truth']) and poses[i][0]>stop_start and poses[i-1][0]<stop_end:
                speed=math.hypot(poses[i][1]-poses[i-1][1],poses[i][2]-poses[i-1][2])/dt
                wz=abs(yaws[i]-yaws[i-1])/dt
                pose_intervals.append((max(stop_start,poses[i-1][0]),min(stop_end,poses[i][0]),speed,wz))
        for i,(candidate,interval_end,speed,wz) in enumerate(pose_intervals):
            if speed>stop_speed or wz>.05:continue
            covered_until=interval_end
            if covered_until-candidate>=.5-1e-9:
                truth_stop=candidate;break
            for next_start,next_end,next_speed,next_wz in pose_intervals[i+1:]:
                if next_start>covered_until+1e-9 or next_speed>stop_speed or next_wz>.05:break
                covered_until=max(covered_until,next_end)
                if covered_until-candidate>=.5-1e-9:
                    truth_stop=candidate;break
            if truth_stop is not None:break
        requested_zero=_event_receive_zero(events,topics['requested'],stop_start,stop_end)
        final_zero=_first_zero(final_rows,stop_start,stop_end,max_gap=float(max_gaps['command']))
        raw_rows=streams['raw']; shared_rows=streams['shared']
        pipeline={}
        for name,rows in [('raw',raw_rows),('shared',shared_rows),('final',final_rows),('requested',streams['requested'])]:
            _,cov=_zoh_integral(rows,t0,t1,float(max_gaps['command']))
            pipeline[f'{name}_coverage_fraction']=cov
            phase_values=[v for t,v in rows if t0<=t<t1]
            pipeline[f'{name}_plateau_vx_mps']=mean(phase_values[-max(1,len(phase_values)//3):]) if phase_values else None
            dt=[b[0]-a[0] for a,b in zip(rows,rows[1:]) if t0<=a[0]<t1 and 0<b[0]-a[0]<=float(max_gaps['command'])]
            pipeline[f'{name}_receive_hz']=1./mean(dt) if dt else None
            wall_rows=sorted(float(e['wall_s']) for e in events if e.get('topic')==topics[name] and
                             e.get('wall_s') is not None and t0<=(_event_time(e) if _event_time(e) is not None else -1)<t1)
            wall_dt=[b-a for a,b in zip(wall_rows,wall_rows[1:]) if b>a]
            pipeline[f'{name}_receive_wall_hz']=1./mean(wall_dt) if wall_dt else None
        raw_zero=_first_zero(raw_rows,stop_start,stop_end,max_gap=float(max_gaps['command']))
        shared_zero=_first_zero(shared_rows,stop_start,stop_end,max_gap=float(max_gaps['command']))
        raw_onset=_first_nonzero(raw_rows,t0,t1); shared_onset=_first_nonzero(shared_rows,t0,t1)
        final_onset=_first_nonzero(final_rows,t0,t1)
        pipeline.update({'raw_zero_receipt_s':raw_zero,
                         'shared_zero_receipt_s':shared_zero,
                         'final_zero_receipt_s':final_zero,
                         'raw_first_zero_receipt_s':_first_zero(raw_rows,stop_start,stop_end,confirm=0.,max_gap=float(max_gaps['command'])),
                         'shared_first_zero_receipt_s':_first_zero(shared_rows,stop_start,stop_end,confirm=0.,max_gap=float(max_gaps['command'])),
                         'final_first_zero_receipt_s':_first_zero(final_rows,stop_start,stop_end,confirm=0.,max_gap=float(max_gaps['command'])),
                         'raw_to_shared_zero_s':None,'shared_to_final_zero_s':None,
                         'raw_nonzero_onset_s':raw_onset,'shared_nonzero_onset_s':shared_onset,
                         'final_nonzero_onset_s':final_onset,
                         'raw_to_shared_onset_s':shared_onset-raw_onset if raw_onset is not None and shared_onset is not None else None,
                         'shared_to_final_onset_s':final_onset-shared_onset if final_onset is not None and shared_onset is not None else None,
                         'raw_to_shared_zero_wall_s':None,'shared_to_final_zero_wall_s':None,
                         'raw_to_shared_onset_wall_s':None,'shared_to_final_onset_wall_s':None,
                         'final_command_integral_m':_zoh_integral(final_rows,t0,stop_end,float(max_gaps['command']))[0],
                         'raw_command_integral_m':_zoh_integral(raw_rows,t0,stop_end,float(max_gaps['command']))[0],
                         'requested_zero_receipt_s':_event_receive_zero(events,topics['requested'],stop_start,stop_end),
                         'requested_receive_hz':pipeline.get('requested_receive_hz')})
        if raw_zero is not None and shared_zero is not None:
            pipeline['raw_to_shared_zero_s']=pipeline['shared_zero_receipt_s']-pipeline['raw_zero_receipt_s']
        if pipeline['shared_zero_receipt_s'] is not None and final_zero is not None:
            pipeline['shared_to_final_zero_s']=final_zero-pipeline['shared_zero_receipt_s']
        for field,topic,a,b in [('raw_to_shared_zero_wall_s',topics['shared'],raw_zero,shared_zero),
                                ('shared_to_final_zero_wall_s',topics['final'],shared_zero,final_zero),
                                ('raw_to_shared_onset_wall_s',topics['shared'],raw_onset,shared_onset),
                                ('shared_to_final_onset_wall_s',topics['final'],shared_onset,final_onset)]:
            if a is not None and b is not None:
                wa,wb=_wall_time_for_sim_event(events,topics['raw'] if field.startswith('raw_to_shared') else topics['shared'],a),_wall_time_for_sim_event(events,topic,b)
                if wa is not None and wb is not None:pipeline[field]=wb-wa
        stop_path=_path_metrics(events,topics['truth'],requested_zero,truth_stop if truth_stop is not None else stop_end,float(max_gaps['truth'])) if truth_stop is not None and requested_zero is not None else None
        final_zero_path=_path_metrics(events,topics['truth'],final_zero,truth_stop,float(max_gaps['truth'])) if truth_stop is not None and final_zero is not None and truth_stop>=final_zero else None
        stop={ 'requested_zero_s':requested_zero,'final_nav_zero_s':final_zero,'truth_effective_stop_s':truth_stop,
               'requested_zero_to_truth_stop_s':truth_stop-requested_zero if truth_stop is not None and requested_zero is not None else None,
               'final_zero_to_truth_stop_s':truth_stop-final_zero if truth_stop is not None and final_zero is not None else None,
               'final_zero_stop_reason':'stop_precedes_reference' if truth_stop is not None and final_zero is not None and truth_stop<final_zero else None,
               'stop_path_distance_m':stop_path['path_distance_m'] if stop_path else None,
               'stop_net_displacement_m':stop_path['net_displacement_m'] if stop_path else None,
               'final_zero_to_stop_path_distance_m':final_zero_path['path_distance_m'] if final_zero_path else None,
               'final_zero_to_stop_net_displacement_m':final_zero_path['net_displacement_m'] if final_zero_path else None,
               'requested_zero_missing_reason':None if requested_zero is not None else 'requested command zero receipt missing',
               'final_zero_missing_reason':None if final_zero is not None else 'confirmed final-nav zero receipt missing',
               'truth_stop_censor_reason':'no supported 0.5 s planar/yaw-quiet interval in observed stop window' if truth_stop is None else None,
               'censored_path_distance_m':_path_metrics(events,topics['truth'],requested_zero,stop_end,float(max_gaps['truth']))['path_distance_m'] if truth_stop is None and requested_zero is not None else None,
               'censored_net_displacement_m':_path_metrics(events,topics['truth'],requested_zero,stop_end,float(max_gaps['truth']))['net_displacement_m'] if truth_stop is None and requested_zero is not None else None,
               'stop_censored':truth_stop is None}
        if 'truth_stop_speed_mps' in manifest:stop['truth_stop_speed_threshold_mps']=stop_speed
        target_coupling=_phase_summary(events,topics['truth'],stopv,t0,t1,t1,float(max_gaps['truth']))
        stop_coupling=_phase_summary(events,topics['truth'],stopv,stop_start,truth_stop or stop_end,observed_end,float(max_gaps['truth']))
        row.update({'truth':outputs['truth'],'slam':outputs['slam'],'pipeline':pipeline,'stop':stop,
                    'coupling':{'target_phase':target_coupling,'stopping_phase':stop_coupling}})
        per_speed.append(row)
    stream_gaps={}
    for name,topic in topics.items():
        times=sorted(t for e in events if e.get('topic')==topic and (t:=_event_time(e)) is not None)
        limit=float(max_gaps.get(name,max_gaps.get('command',.15)))
        gaps=[b-a for a,b in zip(times,times[1:]) if b>a]
        stream_gaps[topic]={'sample_count':len(times),'max_gap_s':max(gaps) if gaps else None,
                            'over_limit_gap_count':sum(g>limit for g in gaps),'gap_limit_s':limit}
    return {'per_speed':per_speed,'aggregate':{'count':len(per_speed),'requested_speeds_mps':[r['requested_vx_mps'] for r in per_speed]},
            'quality':{'event_count':len(events),'clock_epoch_count':len(epochs),'clock_epoch_crossing':crossing,
                       'clock_rollback':rollback,'nonfinite_event_count':nonfinite,
                       'missing_topics':[topic for name,topic in topics.items() if not any(e.get('topic')==topic for e in events)],
                       'unsupported_interpolation_policy':'linear only within adjacent supported samples; unstamped Twist is receive-time ZOH; no extrapolation',
                       'source_gaps':stream_gaps,
                       'strict_json':True}}
