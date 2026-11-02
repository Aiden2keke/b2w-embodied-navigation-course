from pathlib import Path
import json,math,re,numpy as np
b=Path(__file__).parent;p=Path((b/'yaw-session-path.txt').read_text().strip());a=json.loads((b/'yaw-analysis.json').read_text());r=json.loads((p/'session-report.json').read_text());lo=a['pulse']['start_sim'];zero=a['delivery']['first_zero_final']['sim_receive_s'];end=a['pulse']['rest']['end_sim'];ref=float(re.search(r'Stop brake captured reference=([-0-9.]+)',(p/'stack.log').read_text()).group(1));es=[json.loads(l) for l in next(p.glob('observer/*/events.jsonl')).open()]
rows=lambda topic:[e for e in es if e['topic']==topic and e['sim_receive_s'] is not None and e['stamp_s'] is not None]
im=rows('/sim/raw/imu');truth=rows('/sim/ground_truth/odom');js=rows('/joint_states');tau=[e for e in es if e['topic']=='/joint_torques' and e['sim_receive_s'] is not None and zero+.15<=e['sim_receive_s']<=end];near=lambda rows,t:min(rows,key=lambda e:abs(e['stamp_s']-t));wrap=lambda a:math.atan2(math.sin(a),math.cos(a))
def yaw(q):return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
series=[]
for e in tau:
 t=e['sim_receive_s'];i=near(im,t);j=near(js,t);v=dict(zip(j['value']['name'],j['value']['velocity']));dq=np.array([v[l+'_foot_joint'] for l in ['FR','FL','RR','RL']]);gyro=i['value']['angular_velocity']['z'];heading=yaw(i['value']['orientation']);expected=float(np.clip(8*wrap(ref-heading)-.5*gyro,-.75,.75));scalar=float(np.dot(np.array(e['value']['data'][12:])+dq,[1,-1,1,-1])/4);series.append(dict(t=t,imu_yaw=heading,gyro=gyro,error=wrap(ref-heading),scalar_estimate=scalar,expected=expected,torques=e['value']['data'][12:]))
late=[x for x in series if x['t']>=end-1];errors=[];tilts=[]
for i in im:
 t=i['stamp_s']
 if not lo<=t<=end:continue
 tr=near(truth,t);errors.append(abs(wrap(yaw(i['value']['orientation'])-yaw(tr['value']['pose']['pose']['orientation']))))
for e in truth:
 if lo<=e['stamp_s']<=end:
  q=e['value']['pose']['pose']['orientation'];tilts.append(math.degrees(math.acos(np.clip(1-2*(q['x']**2+q['y']**2),-1,1))))
windows=[]
for name,t in [('initial',lo),('zero',zero),('zero_plus_.3',zero+.3),('rest_start',a['pulse']['rest']['start_sim']),('rest_end',end)]:
 i=near(im,t);tr=near(truth,t);windows.append(dict(name=name,imu_stamp=i['stamp_s'],imu_yaw=yaw(i['value']['orientation']),gyro=i['value']['angular_velocity']['z'],truth_yaw=yaw(tr['value']['pose']['pose']['orientation'])))
physical=a['physical'];ret=physical['net_yaw_rad']/physical['owner_window_yaw_rad'];out=dict(captured_private_imu_heading=ref,capture_count=(p/'stack.log').read_text().count('Stop brake captured reference='),private_imu_samples=len(im),imu_truth_max_abs_yaw_difference_rad=max(errors),heading_reference_error_end_rad=late[-1]['error'],late_mean_added_scalar_nm=float(np.mean([x['scalar_estimate'] for x in late])),late_mean_formula_scalar_nm=float(np.mean([x['expected'] for x in late])),late_mean_gyro_rad_s=float(np.mean([x['gyro'] for x in late])),peak_estimated_added_scalar_nm=max(abs(x['scalar_estimate']) for x in series),actual_stop_peak_wheel_torque_nm=max(abs(v) for x in series for v in x['torques']),wheel_motor_20nm_saturation_count=sum(abs(v)>=19.999 for x in series for v in x['torques']),estimated_added_cap_samples=sum(abs(x['scalar_estimate'])>=.749 for x in series),max_tilt_deg=max(tilts),retention_fraction=ret,body_yaw_pass=physical['net_yaw_rad']>=.04 and ret>=.5,runtime_safety_pass=r['success'],qualified_rest=a['pulse']['rest'],checkpoints=windows,interpretation='Hold remains active with one valid reference; late added wheel scalar is .75 Nm and formula saturates at .75 Nm. Persistent heading error remains. Peak scalar estimates use asynchronous torque/joint alignment and are not direct measurements of clamp output. This is measured failure of retained heading within the existing budget, not proof that raising torque alone would fix the plant.')
(b/'heading-analysis.json').write_text(json.dumps(out,indent=2)+'\n');(b/'heading-torque-series.json').write_text(json.dumps(series,separators=(',',':'))+'\n');print(json.dumps(out,indent=2))
