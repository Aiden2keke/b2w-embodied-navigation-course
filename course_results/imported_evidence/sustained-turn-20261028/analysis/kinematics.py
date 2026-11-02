from pathlib import Path
import json, numpy as np,math,xml.etree.ElementTree as ET
R=Path('/tmp/b2w-navigation-ab-20261023');tree=ET.parse(R/'artifacts/recovery/robot.urdf').getroot();legs=['FR','FL','RR','RL'];paths=[next(Path('/tmp/b2w-sustained-turn-20261028/live').glob('precheck-*'))]
def mat(axis,t):
 x,y,z=axis;c=np.cos(t);s=np.sin(t);v=1-c;return np.array([[x*x*v+c,x*y*v-z*s,x*z*v+y*s],[y*x*v+z*s,y*y*v+c,y*z*v-x*s],[z*x*v-y*s,z*y*v+x*s,z*z*v+c]])
def feet(q):
 ps=[]
 for l in legs:
  T=np.eye(3);p=np.zeros(3)
  for part in ['hip','thigh','calf','foot']:
   j=tree.find(f"joint[@name='{l}_{part}_joint']");origin=j.find('origin');assert origin.get('rpy')=='0 0 0';p+=T@np.array(list(map(float,origin.get('xyz').split())))
   T=T@mat(list(map(float,j.find('axis').get('xyz').split())),q[f'{l}_{part}_joint'])
  ps.append(p)
 return np.array(ps)
def theta(q):
 o=q['orientation'];x,y,z,w=[o[k] for k in ['x','y','z','w']];return [math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(np.clip(2*(w*y-z*x),-1,1)),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]
def angle(a,b):return math.atan2(math.sin(a-b),math.cos(a-b))
allout=[]
for path,idx in zip(paths,[0]):
 r=json.loads((path/'session-report.json').read_text());pulse=r['precheck'][idx];lo=pulse['start_sim'];hi=pulse['end_pulse_sim'];end=pulse['rest']['end_sim'];es=[json.loads(l) for l in next(path.glob('observer/*/events.jsonl')).open()]
 js=[e for e in es if e['topic']=='/joint_states'];tr=[e for e in es if e['topic']=='/sim/ground_truth/odom'];ns=[e for e in es if e['topic']=='/network_input_debug'];fn=lambda e:e['stamp_s'];near=lambda rows,t:min(rows,key=lambda e:abs(fn(e)-t))
 
 synthetic_joints=not js
 if not js:
  names=[l+'_'+part+'_joint' for l in legs for part in ['hip','thigh','calf']]+[l+'_foot_joint' for l in legs]
  js=[dict(stamp_s=e['sim_receive_s'],value=dict(name=names,position=(np.array(e['value']['data'][9:25])+np.array([0,.8,-1.5]*4+[0]*4)).tolist(),velocity=(np.array(e['value']['data'][25:41])*20).tolist())) for e in ns if e['sim_receive_s'] is not None]
 q0=near(js,lo)['value'];q0=dict(zip(q0['name'],q0['position']));f0=feet(q0);rp0=theta(near(tr,lo)['value']['pose']['pose']);ref=f0[:,:2]-f0[:,:2].mean(0)
 records=[]
 for t in [lo-.1,lo,lo+.1,min(lo+.2,hi),hi,hi+.1,hi+.3,hi+.6,hi+1.,pulse['rest']['start_sim'],end]:
  j=near(js,t);j=j['value'];q=dict(zip(j['name'],j['position']));vel=dict(zip(j['name'],j['velocity']));f=feet(q);now=f[:,:2]-f[:,:2].mean(0);A=ref.T@now;shape=math.atan2(A[0,1]-A[1,0],A[0,0]+A[1,1]);rp=theta(near(tr,t)['value']['pose']['pose']);dq=[vel[f'{l}_foot_joint'] for l in legs]
  records.append(dict(t=t,body_yaw_delta=angle(rp[2],rp0[2]),roll=rp[0],pitch=rp[1],stance_shape_yaw=shape,body_plus_stance_yaw=angle(rp[2],rp0[2])+shape,wheel_q_delta=None if synthetic_joints else [q[f'{l}_foot_joint']-q0[f'{l}_foot_joint'] for l in legs],wheel_dq=dq,hips=[q[f'{l}_hip_joint'] for l in legs]))
 print(path.name,json.dumps(records,indent=2));allout.append({'session':str(path),'records':records})
Path('/tmp/b2w-sustained-turn-20261028/analysis/kinematics.json').write_text(json.dumps(allout,indent=2)+'\n')
