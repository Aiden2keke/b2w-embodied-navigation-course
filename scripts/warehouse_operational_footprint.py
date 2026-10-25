"""Offline collision projection of synchronized observed configurations.

An empirical gait envelope is evidence, not permission to shrink a live footprint.
No ROS imports or motion. Wheels without recorded phase are bounded over all phases.
"""
import math
import xml.etree.ElementTree as ET
import numpy as np


def rotation(axis,angle):
    axis=np.asarray(axis,dtype=float);axis=axis/np.linalg.norm(axis)
    x,y,z=axis;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)


def origin(node):
    T=np.eye(4)
    if node is not None:
        T[:3,3]=[float(v) for v in node.get('xyz','0 0 0').split()]
        roll,pitch,yaw=[float(v) for v in node.get('rpy','0 0 0').split()]
        T[:3,:3]=rotation([0,0,1],yaw)@rotation([0,1,0],pitch)@rotation([1,0,0],roll)
    if not np.all(np.isfinite(T)):raise ValueError('Invalid origin')
    return T


class CollisionProjection:
    def __init__(self,urdf):
        root=ET.parse(urdf).getroot();self.links={l.get('name'):l for l in root.findall('link')}
        self.joints={j.find('child').get('link'):j for j in root.findall('joint')}
        self.movable=[j.get('name') for j in self.joints.values() if j.get('type')=='revolute']

    def bounds(self,positions,roll=0.,pitch=0.):
        frames={'base_link':np.eye(4)}
        def frame(name):
            if name in frames:return frames[name]
            j=self.joints[name];kind=j.get('type');T=frame(j.find('parent').get('link'))@origin(j.find('origin'))
            if kind in ('revolute','continuous'):
                q=positions.get(j.get('name'),0. if kind=='continuous' else None)
                if q is None or not math.isfinite(q):raise ValueError('Missing/nonfinite observed joint')
                if kind=='revolute':
                    limit=j.find('limit')
                    if not float(limit.get('lower'))<=q<=float(limit.get('upper')):raise ValueError('Observed joint exceeds URDF limit')
                R=np.eye(4);R[:3,:3]=rotation([float(v) for v in j.find('axis').get('xyz').split()],q);T=T@R
            elif kind!='fixed':raise ValueError('Unsupported joint')
            frames[name]=T;return T
        body=np.eye(4);body[:3,:3]=rotation([0,1,0],pitch)@rotation([1,0,0],roll)
        bounds=np.array([math.inf,-math.inf,math.inf,-math.inf])
        for name,link in self.links.items():
            T=body@frame(name)
            for collision in link.findall('collision'):
                C=origin(collision.find('origin'));world=T@C;shape=collision.find('geometry')[0];R=world[:3,:3]
                if shape.tag=='box':
                    half=np.array([float(v)/2 for v in shape.get('size').split()]);extent=np.abs(R)@half
                elif shape.tag=='sphere':extent=np.full(3,float(shape.get('radius')))
                elif shape.tag=='cylinder':
                    axis=R[:,2];radius=float(shape.get('radius'));half=float(shape.get('length'))/2
                    extent=radius*np.sqrt(np.maximum(0,1-axis*axis))+half*np.abs(axis)
                    j=self.joints.get(name)
                    if j is not None and j.get('type')=='continuous' and j.get('name') not in positions:
                        # The collision cylinder is nearly coaxial with wheel Y.
                        # Bound rounding in URDF rpy plus any off-axis offset over
                        # every wheel phase rather than assigning an invented phase.
                        spin=np.array([float(v) for v in j.find('axis').get('xyz').split()]);spin/=np.linalg.norm(spin)
                        wobble=2*np.linalg.norm(np.cross(C[:3,3],spin))+2*math.hypot(radius,half)*np.linalg.norm(np.cross(C[:3,2],spin))
                        extent+=wobble
                else:raise ValueError('Unsupported collision primitive')
                lo,hi=world[:3,3]-extent,world[:3,3]+extent
                bounds=np.array([min(bounds[0],lo[0]),max(bounds[1],hi[0]),min(bounds[2],lo[1]),max(bounds[3],hi[1])])
        if not np.all(np.isfinite(bounds)):raise ValueError('Empty/nonfinite collision projection')
        return bounds.tolist()
