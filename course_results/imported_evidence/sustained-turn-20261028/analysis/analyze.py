"""Offline signed response comparison. Receive-clock alignment, never transport timestamps."""
from pathlib import Path
import json, math, hashlib
import numpy as np
import onnxruntime as ort
root=Path('/tmp/b2w-navigation-ab-20261023')
base=Path(__file__).resolve().parent
model_path=root/'third_party/b2w_rl_sar/policy/b2w/robot_lab/policy.onnx'
model=ort.InferenceSession(str(model_path),providers=['CPUExecutionProvider'])
order=['FR','FL','RR','RL']
def infer(o):return model.run(None,{'observations':np.asarray(o,dtype=np.float32)[None,:]})[0][0]
def differential(a):return float((a[0]+a[2]-a[1]-a[3])/2)
def angle(a,b):return math.atan2(math.sin(a-b),math.cos(a-b))
def yaw(p):
 q=p['pose']['pose']['orientation'];return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
def xy(p):
 p=p['pose']['pose']['position'];return [p['x'],p['y']]
def mean(v):return np.mean(v,axis=0).tolist() if len(v) else None

def analyze(path,index):
 r=json.loads((path/'session-report.json').read_text());pulse=r['precheck'][index]
 events_file=next(path.glob('observer/*/events.jsonl'))
 events=[json.loads(l) for l in events_file.open()]
 def topic(t):return [e for e in events if e['topic']==t and e['sim_receive_s'] is not None]
 network=topic('/network_input_debug');final=topic('/path_manager/path_manager_ros/nav_vel');truth=topic('/sim/ground_truth/odom')
 lo,hi=pulse['start_sim'],pulse['end_pulse_sim'];end=pulse['rest']['end_sim']
 def command(e):
  v=e['value'];return [v['linear'][k] for k in ('x','y','z')]+[v['angular'][k] for k in ('x','y','z')]
 moving=[e for e in final if lo-.05<=e['sim_receive_s']<=hi+.5 and any(command(e))]
 first=moving[0];zero=next(e for e in final if e['seq']>first['seq'] and not any(command(e)))
 nfirst=next(e for e in network if e['sim_receive_s']>=lo-.05 and abs(e['value']['data'][8])>.01)
 nzero=next(e for e in network if e['seq']>nfirst['seq'] and e['value']['data'][8]==0.)
 def poseat(t):return min(truth,key=lambda e:abs(e['stamp_s']-t))
 initial=poseat(lo);at_zero=poseat(zero['sim_receive_s']);last=poseat(end)
 def window(a,b):
  ns=[e for e in network if a<=e['sim_receive_s']<b]
  if not ns:return dict(window=[a,b],n=0)
  obs=np.array([e['value']['data'] for e in ns]);raw=np.array([infer(o) for o in obs]);applied=raw.copy()
  applied[np.all(np.abs(obs[:,6:9])<.01,axis=1),12:]=0
  raw_wheels=raw[:,12:]*5;targets=applied[:,12:]*5;dq=obs[:,37:41]*20
  commands=[e['value']['data'] for e in topic('/joint_commands') if a<=e['sim_receive_s']<b]
  torques=[e['value']['data'][12:] for e in topic('/joint_torques') if a<=e['sim_receive_s']<b]
  joints=[e['value'] for e in topic('/joint_states') if a<=e['sim_receive_s']<b]
  named=[]
  for j in joints:
   m=dict(zip(j['name'],j['velocity']))
   named.append([m[f'{f}_foot_joint'] for f in order])
  return dict(window=[a,b],n=len(ns),model_commands=sorted(set(tuple(o[6:9]) for o in obs)),
      raw_wheel_target_mean=mean(raw_wheels),applied_wheel_target_mean=mean(targets),
      raw_target_right_minus_left=differential(mean(raw_wheels)),applied_target_right_minus_left=differential(mean(targets)),
      wheel_dq_mean=mean(dq),dq_right_minus_left=differential(mean(dq)),gyro_z_mean=float(obs[:,2].mean()*4),
      raw_leg_action_mean=mean(raw[:,:12]),raw_leg_action_rms=float(np.sqrt(np.mean(raw[:,:12]**2))),
      recorded_joint_target_samples=len(commands),recorded_joint_target_mean=mean(commands),
      recorded_wheel_torque_samples=len(torques),recorded_wheel_torque_mean=mean(torques),
      named_wheel_dq_samples=len(named),named_wheel_dq_mean=mean(named))
 alltruth=[e for e in truth if lo<=e['stamp_s']<=end]
 posttruth=[e for e in alltruth if e['stamp_s']>=zero['sim_receive_s']]
 def extrema(es,origin):
  return dict(max_translation_m=max(math.dist(xy(e['value']),xy(origin['value'])) for e in es),
     max_abs_yaw_rad=max(abs(angle(yaw(e['value']),yaw(origin['value']))) for e in es),
     max_translational_speed_mps=max(math.hypot(e['value']['twist']['twist']['linear']['x'],e['value']['twist']['twist']['linear']['y']) for e in es),
     max_abs_yaw_speed_rad_s=max(abs(e['value']['twist']['twist']['angular']['z']) for e in es))
 def transition(e):return dict(sim_receive_s=e['sim_receive_s'],wall_monotonic_s=e['wall_monotonic_s'],seq=e['seq'])
 # Separate whole rest qualification from stopping time after first received zero.
 duration=zero['sim_receive_s']-first['sim_receive_s']
 out=dict(session=str(path),pulse=pulse,initial_rest=r['initial_rest'],
    delivery=dict(first_nonzero_final=transition(first),first_zero_final=transition(zero),
      receive_clock_nonzero_interval_sim_s=duration,receive_wall_interval_s=zero['wall_monotonic_s']-first['wall_monotonic_s'],
      first_nonzero_model=transition(nfirst),first_zero_model=transition(nzero),model_receive_interval_sim_s=nzero['sim_receive_s']-nfirst['sim_receive_s'],
      owner_observed_interval_sim_s=hi-lo,
      nonzero_final_values=sorted(set(tuple(command(e)) for e in moving))),
    physical=dict(owner_window_yaw_rad=angle(yaw(poseat(hi)['value']),yaw(initial['value'])),
      owner_zero_to_rest_yaw_rad=angle(yaw(last['value']),yaw(poseat(hi)['value'])),
      yaw_during_delivery_rad=angle(yaw(at_zero['value']),yaw(initial['value'])),
      yaw_after_zero_to_rest_rad=angle(yaw(last['value']),yaw(at_zero['value'])),net_yaw_rad=angle(yaw(last['value']),yaw(initial['value'])),
      net_translation_m=math.dist(xy(last['value']),xy(initial['value'])),whole=extrema(alltruth,initial),after_zero=extrema(posttruth,at_zero),
      zero_to_rest_end_sim_s=end-zero['sim_receive_s'],zero_to_rest_start_sim_s=pulse['rest']['start_sim']-zero['sim_receive_s'],
      pose_sample_times=[initial['stamp_s'],at_zero['stamp_s'],last['stamp_s']]),
    chronology=[window(lo-.2,lo),window(lo,lo+.1),window(lo+.1,lo+.25),window(lo,hi),window(hi,hi+.1),window(hi+.1,hi+.3),window(hi+.3,hi+1),window(hi+1,end)],
    cleanup=r.get('normal_cleanup'),fallback_cleanup=r.get('cleanup'),watchdog=r.get('watchdog'),contact_failure=r.get('contact_failure'),
    contact_samples=r.get('contact_sample_count'),truth_first_fault=r.get('truth_first_fault'),readiness_first_fault=r.get('readiness_first_fault'),
    source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [events_file,path/'session-report.json',path/'monitor-events.jsonl']})
 return out
p=next(Path("/tmp/b2w-sustained-turn-20261028/live").glob("precheck-*"))
r=analyze(p,0)
Path("/tmp/b2w-sustained-turn-20261028/analysis/yaw-analysis.json").write_text(json.dumps(r,indent=2)+"\n")
print(json.dumps({k:r[k] for k in ["delivery","physical","contact_failure","contact_samples","truth_first_fault","readiness_first_fault"]},indent=2))
