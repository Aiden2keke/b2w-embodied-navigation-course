#!/usr/bin/python3
"""Offline real Smac Node2D + smoother on a labeled reconstructed static costmap.
No ROS node, simulation, action or publisher is created.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
from stage4_route.contract import load_selection,digest,write_new
from waypoint_editor.core import Editor


def analyze(selection,probe,output,start):
    s=load_selection(selection);e=Editor(s['source_map']['yaml_path'],Path(__file__).resolve().parents[1]/'course_navigation/config/nav2.yaml',Path(__file__).resolve().parents[1]/'course_navigation/config/warehouse_route.yaml')
    if e.map.meta['origin'][2]!=0:raise ValueError('Harness supports axis-aligned source maps only')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    cells=np.array(e.map.cells)[::-1];occupied=cells=='occupied'
    distances=distance_transform_edt(~occupied)*e.map.meta['resolution']
    # Static lethal + official inflation formula, inscribed radius .43, .8 m cutoff.
    costs=np.zeros(cells.shape,dtype=np.uint8)
    band=distances<=.8;hard=distances<=.43
    costs[band]=np.floor(252*np.exp(-3*np.maximum(0,distances[band]-.43))).astype(np.uint8)
    costs[hard]=253;costs[occupied]=254;costs[cells=='unknown']=255
    costfile=output/'reconstructed-static.bin';costs.tofile(costfile)
    by,bx=np.where(cells!='free');blocked_points=np.column_stack((e.map.meta['origin'][0]+(bx+.5)*e.map.meta['resolution'],e.map.meta['origin'][1]+(by+.5)*e.map.meta['resolution']));tree=cKDTree(blocked_points)
    def clearance(p):
        ids=tree.query_ball_point(p[:2],2.)
        if not ids:return 1.0
        d=blocked_points[ids]-np.asarray(p[:2]);c,a=math.cos(p[2]),math.sin(p[2])
        x=np.abs(c*d[:,0]+a*d[:,1]);y=np.abs(-a*d[:,0]+c*d[:,1])
        return float(np.min(np.hypot(np.maximum(x-.75,0),np.maximum(y-.43,0)))-e.map.meta['resolution']/math.sqrt(2))
    goals=s['goals'];segments=[];points=([dict(name='Start',position=start,yaw_rad=0.)] if start is not None else [])+goals
    for a,b in zip(points,points[1:]):
        result=json.loads(subprocess.check_output([str(probe),str(costfile),str(e.map.meta['width']),str(e.map.meta['height']),str(e.map.meta['resolution']),*map(str,e.map.meta['origin'][:2]),*map(str,a['position']),*map(str,b['position']),str(b['yaw_rad'])],text=True,timeout=5).splitlines()[-1])
        path=result['path'];checks=[];swept=[]
        for i,p in enumerate(path):
            if i+1<len(path):
                q=path[i+1];steps=max(1,math.ceil(math.dist(p[:2],q[:2])/.025),math.ceil(abs(math.remainder(q[2]-p[2],math.tau))/.025))
                for k in range(steps):
                    t=k/steps;pose=[p[0]+t*(q[0]-p[0]),p[1]+t*(q[1]-p[1]),p[2]+t*math.remainder(q[2]-p[2],math.tau)]
                    checks.append(e.check_pose(pose));swept.append(pose)
            else:checks.append(e.check_pose(p));swept.append(p)
        from stage4_route.evidence import distance
        deviation=max((distance(p[:2],a['position'],b['position']) for p in path),default=None)
        segments.append(dict(segment=a['name']+'->'+b['name'],**result,sampled_padded_footprint_clear=bool(checks) and all(c['clear'] for c in checks),
                             blocked_samples=sum(not c['clear'] for c in checks),samples=len(checks),
                             max_centerline_deviation_m=deviation,narrow_center_band_satisfied=(deviation<=.35 if deviation is not None and a['name']=='G1' else None),
                             blocked_occupied_samples=sum(c['occupied']>0 for c in checks),blocked_unknown_samples=sum(c['unknown']>0 for c in checks),
                             first_blocked_pose=next((p for p,c in zip(swept,checks) if not c['clear']),None),
                             min_padded_clearance_lower_bound_m=min(map(clearance,swept),default=None),
                             clearance_definition='distance from padded rectangle to nearest nonfree cell center minus cell half-diagonal; conservative cell-square bound, not physical measurement',
                             continuous_physical_pass=False))
    report=dict(selection_sha256=digest(selection),probe_sha256=digest(probe),reconstructed_costmap_sha256=digest(costfile),
                contract='Installed Nav2 1.1.20 AStarAlgorithm<Node2D>, TWOD, allow_unknown=false, cost_penalty=1, tolerance=.3 m, max_iterations=1000000, max_on_approach_iterations=1000, max_planning_time=2 s; installed smoother defaults; no node/DDS',
                limitations=['Reconstructed static grid: not captured live master costmap; no dynamic ObstacleLayer or local costmap',
                             'Smac2D radius checking does not guarantee non-circular padded-footprint clearance',
                             'Sampled swept geometry at <=.025m/rad is not a continuous or physical guarantee',
                             'Start unknown unless --start provided; supplied Start is a hypothesis, not registered pose',
                             'Per-leg smoothing differs from concatenated multi-pose controller path and does not certify turns',
                             'No SLAM registration established offline; no runtime planning PASS'],
                start_status='UNAVAILABLE' if start is None else 'UNVERIFIED_HYPOTHESIS',segments=segments,
                endpoints=e.check_state({**e.initial,'goals':goals}),turn_sweeps={g['name']:dict(clear_samples=sum(c['clear'] for c in e.sweep([*g['position'],g['yaw_rad']])['samples']),total_samples=73) for g in goals},
                runtime_planning_pass=False)
    write_new(output/'planning.json',report);return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--selection',type=Path,required=True);p.add_argument('--probe',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--start',type=float,nargs=2)
    a=p.parse_args();r=analyze(a.selection,a.probe,a.output,a.start);print(a.output/'planning.json')
