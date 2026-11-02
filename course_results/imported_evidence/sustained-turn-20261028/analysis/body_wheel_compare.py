from pathlib import Path
import json,math,numpy as np,re,xml.etree.ElementTree as ET
r=Path('/tmp/b2w-navigation-ab-20261023');b=Path(__file__).parent;tree=ET.parse(r/'artifacts/recovery/robot.urdf').getroot();legs=['FR','FL','RR','RL']
def rx(t):return np.array([[1,0,0],[0,math.cos(t),-math.sin(t)],[0,math.sin(t),math.cos(t)]])
def ry(t):return np.array([[math.cos(t),0,math.sin(t)],[0,1,0],[-math.sin(t),0,math.cos(t)]])
def rotation(q):
 x,y,z,w=[q[k] for k in ['x','y','z','w']];return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
results=[]
for path,label,analysis in [(next(Path('/tmp/b2w-sustained-turn-20261028/live').glob('precheck-*')),'heading',b/'yaw-analysis.json')]:
 a=json.loads(analysis.read_text());lo=a['pulse']['start_sim'];zero=a['delivery']['first_zero_model']['sim_receive_s'];end=a['pulse']['rest']['end_sim'];es=[json.loads(l) for l in next(path.glob('observer/*/events.jsonl')).open()];js=[e for e in es if e['topic']=='/joint_states' and e['stamp_s'] is not None and lo<=e['stamp_s']<=end];truth=[e for e in es if e['topic']=='/sim/ground_truth/odom'];ns=[e for e in es if e['topic']=='/network_input_debug' and e['sim_receive_s'] is not None];torques=[e for e in es if e['topic']=='/joint_torques' and e['sim_receive_s'] is not None and zero<=e['sim_receive_s']<=end];series=[]
 for e in js:
  t=e['stamp_s'];j=e['value'];q=dict(zip(j['name'],j['position']));dq=dict(zip(j['name'],j['velocity']));tr=min(truth,key=lambda x:abs(x['stamp_s']-t))['value']['pose']['pose'];R=rotation(tr['orientation']);pos=np.array([tr['position'][k] for k in ['x','y','z']]);o=min(ns,key=lambda x:abs(x['sim_receive_s']-t))['value']['data'];gyro=R@np.array(o[:3])*4;centers=[];directions=[];angular=[]
  for l in legs:
   T=np.eye(3);p=np.zeros(3)
   for part in ['hip','thigh','calf','foot']:
    joint=tree.find(f"joint[@name='{l}_{part}_joint']");p+=T@np.array(list(map(float,joint.find('origin').get('xyz').split())));T=T@(rx(q[l+'_'+part+'_joint']) if part=='hip' else ry(q[l+'_'+part+'_joint']))
   c=tree.find(f"link[@name='{l}_foot']/collision/origin");p+=T@np.array(list(map(float,c.get('xyz').split())))
   axle=R@T@np.array([0,1,0]);h=np.cross(axle,[0,0,1]);h=h/np.linalg.norm(h)
   centers.append(pos+R@p);directions.append(h);angular.append(dq[l+'_foot_joint']+dq[l+'_thigh_joint']+dq[l+'_calf_joint']+float(gyro@axle))
  series.append(dict(t=t,q=[q[l+'_foot_joint'] for l in legs],d=(q['FR_foot_joint']+q['RR_foot_joint']-q['FL_foot_joint']-q['RL_foot_joint'])/2,centers=np.array(centers),directions=np.array(directions),angular=np.array(angular)))
 windows=[]
 for name,low,high in [('active',lo,zero),('stop',zero,end)]:
  xs=[x for x in series if low<=x['t']<=high];travel=np.zeros(4);pred=np.zeros(4)
  for p,q in zip(xs,xs[1:]):
   h=(p['directions']+q['directions'])/2;travel+=np.sum((q['centers']-p['centers'])*h,axis=1);pred+=.1125*(p['angular']+q['angular'])/2*(q['t']-p['t'])
  windows.append(dict(window=name,world_wheel_center_longitudinal_travel_m=travel.tolist(),rotation_predicted_no_slip_distance_m=pred.tolist(),residual_m=(travel-pred).tolist()))
 scalar=[]
 for e in torques:
  t=e['sim_receive_s'];j=min(js,key=lambda x:abs(x['stamp_s']-t))['value'];m=dict(zip(j['name'],j['velocity']));tau=np.array(e['value']['data'][12:]);dq=np.array([m[l+'_foot_joint'] for l in legs]);scalar.append((t,float(np.dot(tau+dq,[1,-1,1,-1])/4)))
 cap_count=sum(abs(v)>=19.999 for e in torques for v in e['value']['data'][12:]);last=[v for t,v in scalar if end-1<=t<=end];pose=[]
 for t0 in [zero,zero+.3,end-1,end]:
  tr=min(truth,key=lambda e:abs(e['stamp_s']-t0));qq=tr['value']['pose']['pose']['orientation'];pose.append([tr['stamp_s'],math.atan2(2*(qq['w']*qq['z']+qq['x']*qq['y']),1-2*(qq['y']**2+qq['z']**2))])
 out=dict(label=label,physical=a['physical'],delivery=a['delivery'],wheel_differential_start=series[0]['d'],wheel_differential_end=series[-1]['d'],wheel_net_differential=series[-1]['d']-series[0]['d'],wheel_capture_to_end=None,estimated_added_scalar_last_second_mean=float(np.mean(last)),estimated_added_scalar_peak=max(abs(v) for t,v in scalar),actual_wheel_demand_max_abs=max(abs(v) for e in torques for v in e['value']['data'][12:]),wheel_motor_saturation_count=cap_count,pose_checkpoints=pose,kinematic_rolling_check=windows)
 if label=='brake':
  txt=(path/'stack.log').read_text();ref=float(re.search(r'Differential stop captured d=([-0-9.]+)',txt).group(1));out['captured_reference']=ref;out['wheel_capture_to_end']=ref-series[-1]['d'];out['predicted_unsaturated_scalar_end']=2*(ref-series[-1]['d']);out['position_error_unresolved_rad']=ref-series[-1]['d']
 results.append(out)
(b/'body-wheel-comparison.json').write_text(json.dumps(dict(interpretation='Named-joint FK and longitudinal rolling residual are approximate. Residual suggests slip/compliance or contact geometry effects, not measured contact wrench or a unique slip coefficient. Torque/dq samples are nearest receive/header aligned.',results=results),indent=2)+'\n')
for o in results:print(json.dumps({k:v for k,v in o.items() if k not in ['physical','delivery','kinematic_rolling_check']},indent=2));print(o['kinematic_rolling_check'])
