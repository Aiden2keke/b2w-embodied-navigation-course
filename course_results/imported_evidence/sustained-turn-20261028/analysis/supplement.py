from pathlib import Path
import json,math,hashlib,bisect,statistics
out=Path(__file__).parent;p=Path((out/'yaw-session-path.txt').read_text().strip());r=json.loads((p/'session-report.json').read_text());a=json.loads((out/'yaw-analysis.json').read_text());lo=a['pulse']['start_sim'];zero=a['delivery']['first_zero_final']['sim_receive_s'];end=a['pulse']['rest']['end_sim']
es=[json.loads(l) for l in next(p.glob('observer/*/events.jsonl')).open()]
def topic(name):return [e for e in es if e['topic']==name and e.get('sim_receive_s') is not None]
truth=topic('/sim/ground_truth/odom');js=topic('/joint_states');im=topic('/sim/raw/imu');sl=topic('/slam/odom');tau=topic('/joint_torques')
def yaw(e):
 q=e['value']['pose']['pose']['orientation'];return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
def at(es,t):return min(es,key=lambda e:abs(e['stamp_s']-t))
base=at(js,lo);q0=dict(zip(base['value']['name'],base['value']['position']));jzero=at(js,zero);qz=dict(zip(jzero['value']['name'],jzero['value']['position']));jl=at(js,end);ql=dict(zip(jl['value']['name'],jl['value']['position']));names=base['value']['name'];wheel=[n for n in names if '_foot_joint' in n];legs=[n for n in names if n not in wheel]
window=[e for e in js if lo<=e['stamp_s']<=end];legq={n:max(abs(dict(zip(e['value']['name'],e['value']['position']))[n]-q0[n]) for e in window) for n in legs}
wi={n:max(abs(dict(zip(e['value']['name'],e['value']['velocity']))[n]) for e in window) for n in wheel}
ltau=[e for e in tau if end-1<=e['sim_receive_s']<=end];ts=[e for e in truth if lo<=e['stamp_s']<=end]
active=[e for e in truth if lo<=e['stamp_s']<zero];stop=[e for e in truth if zero<=e['stamp_s']<=end]
late=[e for e in truth if end-1<=e['stamp_s']<=end]
speed=lambda e:math.hypot(e['value']['twist']['twist']['linear']['x'],e['value']['twist']['twist']['linear']['y'])
contacts=[];inventories=[]
for line in (p/'monitor-events.jsonl').open():
 e=json.loads(line);t=e.get('sim')
 if t is None or not lo<=t<=end:continue
 if e['kind']=='full_contacts':contacts.append(e)
 if e['kind']=='collision_inventory':inventories.append(json.loads(e['message']['data']))
support_counts=[];wrenches=0
for e in contacts:
 feet=set()
 for c in e['message']['contacts']:
  names2=[c['collision1']['name'],c['collision2']['name']]
  for n in wheel:
   if any('::'+n.removesuffix('_joint')+'::' in x for x in names2):feet.add(n)
  wrenches+=len(c['wrenches'])
 support_counts.append(len(feet))
def pose_errors():
 first=at(sl,lo);tfirst=at(truth,first['stamp_s']);results=[]
 def xy(e):return [e['value']['pose']['pose']['position'][k] for k in ('x','y')]
 # Relative yaw is frame-offset invariant; relative XY norm is compared after initial yaw registration.
 angle=yaw(tfirst)-yaw(first);c,s=math.cos(angle),math.sin(angle);startsl=xy(first);starttr=xy(tfirst)
 for e in sl:
  if lo<=e['stamp_s']<=end:
   t=at(truth,e['stamp_s'])
   if abs(t['stamp_s']-e['stamp_s'])>.02:continue
   d=[x-y for x,y in zip(xy(e),startsl)];expected=[starttr[0]+c*d[0]-s*d[1],starttr[1]+s*d[0]+c*d[1]]
   results.append(math.dist(xy(t),expected))
 return dict(samples=len(results),relative_registered_xy_max_m=max(results))
result=dict(session=str(p),leg_peak_delta_from_initial_rad=legq,leg_zero_to_final_delta_rad={n:ql[n]-qz[n] for n in legs},wheel_order=wheel,wheel_initial_to_zero_delta_rad=[qz[n]-q0[n] for n in wheel],wheel_zero_to_final_delta_rad=[ql[n]-qz[n] for n in wheel],wheel_peak_velocity_rad_s=wi,late_wheel_torque_mean_nm=[statistics.mean(e['value']['data'][i] for e in ltau) for i in range(12,16)],active_max_yaw_rate_rad_s=max(abs(e['value']['twist']['twist']['angular']['z']) for e in active),stop_max_yaw_rate_rad_s=max(abs(e['value']['twist']['twist']['angular']['z']) for e in stop),last_second_max_speed_mps=max(speed(e) for e in late),last_second_max_yaw_rate_rad_s=max(abs(e['value']['twist']['twist']['angular']['z']) for e in late),contact_messages=len(contacts),contact_messages_all_four_feet=sum(n==4 for n in support_counts),contact_wrench_entries=wrenches,inventory_messages=len(inventories),forbidden_contact_flags=sum(bool(e.get('ever_forbidden_contact')) for e in inventories),localization=pose_errors(),signal_samples={name:sum(lo<=e['sim_receive_s']<=end for e in topic(name)) for name in ['/path_manager/path_manager_ros/nav_vel','/network_input_debug','/joint_states','/joint_commands','/joint_torques','/sim/raw/imu','/slam/odom','/sim/ground_truth/odom']})
(out/'supplement.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
