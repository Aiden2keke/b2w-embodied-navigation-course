"""Offline circular culvert primitives. SI units; world x forward, floor z=0."""
from dataclasses import dataclass, asdict
import itertools
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from course_sim.description import make_world, sub


@dataclass(frozen=True)
class PipeConfig:
    inner_radius_m: float = 1.6
    wall_thickness_m: float = .18
    length_m: float = 16.
    axis_z_m: float = 1.25
    segments: int = 96
    floor_thickness_m: float = .4
    friction: float = 1.
    ambient: float = .025
    spawn_x_m: float = -6.
    spawn_y_m: float = 0.
    spawn_z_m: float = 1.

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError('all pipe parameters must be finite')
        if (isinstance(self.segments, bool) or not isinstance(self.segments, int)
                or not 32 <= self.segments <= 256):
            raise ValueError('segments must be an integer in [32,256]')
        if min(self.inner_radius_m, self.wall_thickness_m, self.length_m,
               self.floor_thickness_m, self.friction) <= 0:
            raise ValueError('dimensions and friction must be positive')
        if not 0 < self.axis_z_m < self.inner_radius_m:
            raise ValueError('axis_z_m must be between floor and inner radius')
        if self.floor_thickness_m < self.inner_radius_m-self.axis_z_m:
            raise ValueError('floor lining must cover the invert to the shell')
        if not 0 <= self.ambient <= .1:
            raise ValueError('ambient must be in [0,.1] for low-light V0')

    @property
    def floor_width_m(self):
        return 2*math.sqrt(self.inner_radius_m**2-self.axis_z_m**2)

    def manifest(self):
        return dict(schema='b2w_pipe_v0/1', parameters=asdict(self), world_name='pipe_world',
                    floor_z_m=0., clear_floor_width_m=self.floor_width_m,
                    crown_z_m=self.axis_z_m+self.inner_radius_m,
                    max_radial_faceting_error_m=self.inner_radius_m*(1/math.cos(math.pi/self.segments)-1),
                    connections=[dict(id='inlet',xyz=[-self.length_m/2,0,0],yaw=math.pi),
                                 dict(id='outlet',xyz=[self.length_m/2,0,0],yaw=0)],
                    spawn_xyz=[self.spawn_x_m,self.spawn_y_m,self.spawn_z_m],spawn_yaw=0.,
                    scope='dry straight culvert with flat invert lining; open ends; no water or terrain outside')


def make_pipe_world(config):
    root = ET.fromstring(make_world())
    world = root.find('world'); world.set('name', 'pipe_world')
    for element in list(world):
        if element.tag in ['model','light']: world.remove(element)
    scene = sub(world,'scene')
    sub(scene,'ambient',f'{config.ambient:g} {config.ambient:g} {config.ambient+.005:g} 1')
    sub(scene,'background','0.005 0.005 0.008 1'); sub(scene,'shadows','true')
    # Dim interior work light, not a sun. Physics and active LiDAR are independent of RGB light.
    light = sub(world,'light',name='inspection_light',type='point')
    sub(light,'pose','-5 0 2.4 0 0 0'); sub(light,'diffuse','0.12 0.11 0.09 1')
    sub(light,'specular','0.02 0.02 0.02 1'); sub(light,'cast_shadows','true')
    attenuation=sub(light,'attenuation')
    for k,v in [('range',8),('constant',1),('linear',.3),('quadratic',.1)]: sub(attenuation,k,v)

    def model(name):
        m=sub(world,'model',name=name); sub(m,'static','true')
        return sub(m,'link',name='link')

    def box(link,name,pose,size,color):
        for kind in ['collision','visual']:
            e=sub(link,kind,name=name+'_'+kind)
            sub(e,'pose',' '.join(f'{v:.10g}' for v in pose))
            sub(sub(sub(e,'geometry'),'box'),'size',' '.join(f'{v:.10g}' for v in size))
            if kind=='visual':
                material=sub(e,'material'); sub(material,'ambient',color); sub(material,'diffuse',color)
            else:
                ode=sub(sub(sub(e,'surface'),'friction'),'ode')
                sub(ode,'mu',config.friction); sub(ode,'mu2',config.friction)
    link=model('pipe_shell')
    r=config.inner_radius_m; t=config.wall_thickness_m
    # Tangent inner faces leave the bore empty. Outer corners overlap adjacent boxes,
    # closing seams through the full thickness, without concave mesh collision assumptions.
    width=2*(r+t)*math.tan(math.pi/config.segments)*1.001
    for i in range(config.segments):
        a=math.tau*i/config.segments
        box(link,f'wall_{i:03d}',(0,(r+t/2)*math.cos(a),config.axis_z_m+(r+t/2)*math.sin(a),a,0,0),
            (config.length_m,t,width),'0.32 0.34 0.33 1')
    box(model('invert'),'lining',(0,0,-config.floor_thickness_m/2,0,0,0),
        (config.length_m,config.floor_width_m+2*t,config.floor_thickness_m),'0.24 0.26 0.25 1')
    ET.indent(root)
    return ET.tostring(root,encoding='unicode')+'\n'


def collision_bounds(urdf, positions):
    """Exact primitive AABBs after URDF FK; no visual mesh or Nav2 footprint substitution."""
    import numpy as np
    root=ET.parse(urdf).getroot()
    def transform(origin):
        xyz=[0.,0.,0.]; rpy=[0.,0.,0.]
        if origin is not None:
            xyz=list(map(float,origin.get('xyz','0 0 0').split()))
            rpy=list(map(float,origin.get('rpy','0 0 0').split()))
        r,p,y=rpy
        rx=np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
        ry=np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
        rz=np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
        m=np.eye(4);m[:3,:3]=rz@ry@rx;m[:3,3]=xyz
        return m
    transforms={'base_link':np.eye(4)}; pending=list(root.findall('joint'))
    while pending:
        progressed=False
        for joint in pending[:]:
            parent=joint.find('parent').get('link')
            if parent not in transforms: continue
            m=transform(joint.find('origin'))
            if joint.get('type')!='fixed':
                axis=np.array(list(map(float,joint.find('axis').get('xyz').split())))
                axis=axis/np.linalg.norm(axis); angle=positions.get(joint.get('name'),0.)
                cross=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
                rot=np.eye(4); rot[:3,:3]=np.eye(3)+math.sin(angle)*cross+(1-math.cos(angle))*(cross@cross)
                m=m@rot
            transforms[joint.find('child').get('link')]=transforms[parent]@m
            pending.remove(joint);progressed=True
        if not progressed: raise ValueError('URDF joint tree disconnected from base_link')
    bounds=[]
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            m=transforms[link.get('name')]@transform(collision.find('origin'))
            geometry=collision.find('geometry'); shape=geometry[0]
            if shape.tag=='box':
                half=np.array(list(map(float,shape.get('size').split())))/2
                extent=np.abs(m[:3,:3])@half
            elif shape.tag=='cylinder':
                extent=float(shape.get('radius'))*np.sqrt(np.sum(m[:3,:2]**2,axis=1))+float(shape.get('length'))/2*np.abs(m[:3,2])
            elif shape.tag=='sphere': extent=np.full(3,float(shape.get('radius')))
            else: raise ValueError(f'Unsupported collision geometry: {shape.tag}')
            bounds.append(dict(link=link.get('name'),min=(m[:3,3]-extent).tolist(),max=(m[:3,3]+extent).tolist()))
    return bounds


def clearance(config, urdf):
    import numpy as np
    root=ET.parse(urdf).getroot()
    nominal={j.get('name'): (.8 if '_thigh_joint' in j.get('name') else -1.5 if '_calf_joint' in j.get('name') else 0.) for j in root.findall('joint')}
    report=dict(method='URDF FK primitive AABBs; conservative radial corner test; zero and nominal only, not swept joint limits',poses={})
    spawn=np.array([config.spawn_x_m,config.spawn_y_m,config.spawn_z_m])
    for name,positions in [('zero',{}),('nominal',nominal)]:
        bounds=collision_bounds(urdf,positions)
        lo=np.min([b['min'] for b in bounds],axis=0); hi=np.max([b['max'] for b in bounds],axis=0)
        corners=[np.array(p)+spawn for b in bounds for p in itertools.product(*zip(b['min'],b['max']))]
        radial=max(math.hypot(p[1],p[2]-config.axis_z_m) for p in corners)
        floor=float(lo[2]+spawn[2]); end=float(config.length_m/2-max(abs(lo[0]+spawn[0]),abs(hi[0]+spawn[0])))
        report['poses'][name]=dict(aabb_min_m=lo.tolist(),aabb_max_m=hi.tolist(),aabb_dimensions_m=(hi-lo).tolist(),
            floor_clearance_m=floor, radial_clearance_m=config.inner_radius_m-radial,end_clearance_m=end,primitives=len(bounds),
            passed=floor>0 and radial<config.inner_radius_m and end>0)
    report['passed']=all(p['passed'] for p in report['poses'].values())
    return report
