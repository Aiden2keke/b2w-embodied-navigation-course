"""Read-only remeasurement of the retained diagnostic; no ROS or commands."""
from pathlib import Path
import json,math,hashlib,bisect
p=next(Path('/tmp/b2w-sustained-turn-20261028/live').glob('precheck-*'))
f=next(p.glob('observer/*/events.jsonl'))
r=json.loads((p/'session-report.json').read_text());pulse=r['precheck'][0]
lo,hi,end=pulse['start_sim'],pulse['end_pulse_sim'],pulse['rest']['end_sim']
topics={t:[] for t in ['/sim/ground_truth/odom','/slam/odom','/path_manager/path_manager_ros/nav_vel','/network_input_debug']}
for line in f.open():
 e=json.loads(line)
 if e['topic'] in topics and e.get('sim_receive_s') is not None:topics[e['topic']].append(e)
truth=topics['/sim/ground_truth/odom'];slam=topics['/slam/odom'];final=topics['/path_manager/path_manager_ros/nav_vel']
angle=lambda a,b:math.remainder(a-b,2*math.pi)
def yaw(e):
 q=e['value']['pose']['pose']['orientation'];return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
def xy(e):
 p=e['value']['pose']['pose']['position'];return p['x'],p['y']
def at(es,t):return min(es,key=lambda e:abs(e['stamp_s']-t))
def cmd(e):return [e['value'][group][k] for group in ('linear','angular') for k in ('x','y','z')]
a=next(e for e in final if e['sim_receive_s']>=lo and any(cmd(e)))
z=next(e for e in final if e['seq']>a['seq'] and not any(cmd(e)))
initial,zero,last=(at(truth,t) for t in (lo,z['sim_receive_s'],end))
turn=angle(yaw(at(truth,hi)),yaw(initial));net=angle(yaw(last),yaw(initial))
# Relative heading cancels the arbitrary initial map/world orientation offset.
s0=at(slam,lo);t0=at(truth,s0['stamp_s']);errors=[];skew=[]
for e in slam:
 if lo<=e['stamp_s']<=end:
  t=at(truth,e['stamp_s']);skew.append(abs(t['stamp_s']-e['stamp_s']))
  if skew[-1]<=.02:errors.append(abs(angle(angle(yaw(e),yaw(s0)),angle(yaw(t),yaw(t0)))))
model=topics['/network_input_debug'];m=next(e for e in model if e['sim_receive_s']>=lo and abs(e['value']['data'][8])>.01)
mz=next(e for e in model if e['seq']>m['seq'] and e['value']['data'][8]==0)
result=dict(source=str(f),source_sha256=hashlib.sha256(f.read_bytes()).hexdigest(),session_report_sha256=hashlib.sha256((p/'session-report.json').read_bytes()).hexdigest(),owner_interval_s=hi-lo,delivered_interval_s=z['sim_receive_s']-a['sim_receive_s'],external_command_rad_s=cmd(a)[5],external_command_integral_rad=cmd(a)[5]*(z['sim_receive_s']-a['sim_receive_s']),model_command_rad_s=m['value']['data'][8],model_interval_s=mz['sim_receive_s']-m['sim_receive_s'],owner_turn_rad=turn,net_retained_rad=net,owner_retention_fraction=net/turn,delivery_turn_rad=angle(yaw(zero),yaw(initial)),delivery_retention_fraction=net/angle(yaw(zero),yaw(initial)),zero_to_final_rebound_rad=angle(yaw(last),yaw(zero)),zero_to_qualified_rest_start_s=pulse['rest']['start_sim']-z['sim_receive_s'],qualified_rest=pulse['rest'],slam_relative_heading_samples=len(errors),slam_truth_relative_heading_max_rad=max(errors),pair_skew_max_s=max(skew),runtime_pass=r['success'],functional_pass=net>=.04 and net/turn>=.5,watchdog=r['watchdog'],cleanup_pass=r['normal_cleanup']['passed'])
Path('/tmp/b2w-sustained-turn-20261028/analysis/yaw-remeasured.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
