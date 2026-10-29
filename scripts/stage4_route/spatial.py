"""Conservative filled-footprint and saved/live occupied-grid comparison."""
import math
import numpy as np
from scipy.spatial import cKDTree
from waypoint_editor.core import Map,transform_polygon,map_to_pixel,polygons_overlap

FOOTPRINT=[[.75,.43],[.75,-.43],[-.75,-.43],[-.75,.43]]


def footprint_clear(grid,pose):
    w,h=grid['width'],grid['height'];data=grid['data']
    if w<=0 or h<=0 or len(data)!=w*h:return False
    poly=[map_to_pixel(grid,*p) for p in transform_polygon(FOOTPRINT,pose)]
    if any(x<-.5 or x>w-.5 or y<-.5 or y>h-.5 for x,y in poly):return False
    u,v=map_to_pixel(grid,*pose[:2]);cx,cy=int(math.floor(u+.5)),int(math.floor(v+.5))
    if data[(h-1-cy)*w+cx]>=253:return False
    # Incoming ROS grids start at bottom-left; pixel coordinates start at top-left.
    for y in range(max(0,math.ceil(min(p[1] for p in poly)-.5)),min(h,math.floor(max(p[1] for p in poly)+.5)+1)):
        for x in range(max(0,math.ceil(min(p[0] for p in poly)-.5)),min(w,math.floor(max(p[0] for p in poly)+.5)+1)):
            if data[(h-1-y)*w+x]>=254 and polygons_overlap(poly,[[x-.5,y-.5],[x+.5,y-.5],[x+.5,y+.5],[x-.5,y+.5]]):return False
    return True


def occupied_points(grid):
    values=np.asarray(grid['data']).reshape(grid['height'],grid['width'])
    y,x=np.where((values>=65)&(values<=100))
    if len(x)<200:raise ValueError('Insufficient live occupied-map evidence')
    # Bound nearest-neighbor work while preserving spatial coverage.
    step=max(1,len(x)//10000);x=x[::step];y=y[::step]
    a=grid['origin'][2];c,s=math.cos(a),math.sin(a);r=grid['resolution']
    return np.column_stack((grid['origin'][0]+c*(x+.5)*r-s*(y+.5)*r,
                            grid['origin'][1]+s*(x+.5)*r+c*(y+.5)*r))


class MapAlignment:
    def __init__(self,map_path):
        m=Map(map_path);a=np.array(m.cells);y,x=np.where(a=='occupied')
        r=m.meta['resolution'];o=m.meta['origin'];c,s=math.cos(o[2]),math.sin(o[2])
        points=np.column_stack((o[0]+c*(x+.5)*r-s*(m.meta['height']-y-.5)*r,
                               o[1]+s*(x+.5)*r+c*(m.meta['height']-y-.5)*r))
        self.tree=cKDTree(points)

    def check(self,live):
        p=occupied_points(live)
        if min(np.ptp(p,axis=0))<4.:raise ValueError('Live map lacks distributed structure for alignment')
        distances,_=self.tree.query(p)
        p95=float(np.quantile(distances,.95))
        if p95>.10:raise ValueError('Live occupied map disagrees with saved map')
        return p95


def saved_grid(map_path):
    m=Map(map_path)
    return dict(width=m.meta['width'],height=m.meta['height'],resolution=m.meta['resolution'],origin=m.meta['origin'],
                data=[0 if c=='free' else 255 if c=='unknown' else 254 for row in reversed(m.cells) for c in row])


def assess_path(path,goals,start,grids,*,start_pose=None,start_index=0,all_goals=None):
    """Necessary sampled planning gate only; never supplies physical passage events."""
    from .evidence import distance
    if not path:raise ValueError('Empty preflight path')
    from .constraints import segment_band
    if type(start_index) is not int or not 0<=start_index<5:raise ValueError('Original phase required')
    if all_goals is None:all_goals=goals
    if goals!=all_goals[start_index:start_index+len(goals)]:raise ValueError('Original goal identities required')
    if start_pose is not None:
        if (len(start_pose)!=3 or not all(math.isfinite(v) for v in start_pose) or
                math.dist(start_pose[:2],start)>1e-9):raise ValueError('Invalid measured Start connection')
        path=[list(start_pose),*path]
    samples=[]
    for p,q in zip(path,path[1:]):
        delta=math.remainder(q[2]-p[2],math.tau)
        n=max(1,math.ceil(math.dist(p[:2],q[:2])/.025),math.ceil(abs(delta)/.025))
        if n>10000 or len(samples)+n>50000:raise ValueError('Path exceeds bounded sampling limits')
        samples.extend([[p[0]+(q[0]-p[0])*i/n,p[1]+(q[1]-p[1])*i/n,p[2]+delta*i/n] for i in range(n)])
    samples.append(path[-1]);index=0;minimum=[math.inf]*len(goals);bad=[];detours=[];out_of_order=[]
    for i,p in enumerate(samples):
        if not all(math.isfinite(x) for x in p):raise ValueError('Nonfinite path')
        if any(not footprint_clear(g,p) for g in grids):bad.append(i)
        errors=[math.dist(p[:2],g['position']) for g in goals]
        minimum=[min(a,b) for a,b in zip(minimum,errors)]
        if index==len(goals) and any(math.dist(p[:2],g['position'])<=.25 for g in all_goals[start_index+index:]):out_of_order.append(i)
        if index<len(goals):
            if any(math.dist(p[:2],g['position'])<=.25 for g in all_goals[start_index+index+1:]):out_of_order.append(i)
            a=start if index==0 else goals[index-1]['position'];b=goals[index]['position']
            if distance(p[:2],a,b)>segment_band(start_index+index):detours.append(i)
            if errors[index]<=.25:index+=1
    from .continuous import certify_path
    continuous=certify_path(path,grids)
    return dict(passed=continuous['continuous_clearance_proven'] and not bad and not detours and not out_of_order and index==len(goals) and
                math.dist(path[-1][:2],goals[-1]['position'])<=.25 and
                abs(math.remainder(path[-1][2]-goals[-1]['yaw_rad'],math.tau))<=.3,
                sampled_footprint_blocked=len(bad),first_blocked_pose=samples[bad[0]] if bad else None,
                disallowed_detour_samples=len(detours),out_of_order_planned_samples=len(out_of_order),ordered_planned_regions=index,
                per_goal_planned_minimum_xy_m=minimum,samples=len(samples),
                physical_passage_proven=False,
                continuous_clearance_proven=continuous['continuous_clearance_proven'],
                initial_connection_checked=start_pose is not None,
                continuous_certificate=continuous)
