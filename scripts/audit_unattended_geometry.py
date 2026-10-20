#!/usr/bin/python3
"""Offline necessary geometry audit. Creates a new report; never initializes ROS.

Exit 2 means robust route geometry remains blocked/unresolved. A slice certificate
is not a physical impossibility proof or an authorization to run any experiment.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np
from stage4_route.contract import select_candidate, load_selection, digest, write_new, ROOT
from stage4_route.route_geometry import RectangleGrid, capsule_slice, exclude_slice, geometry_analysis
from stage4_route.spatial import saved_grid
from waypoint_editor.core import Map, transform_polygon, polygons_overlap


def audit(candidate, map_path, output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    selected=select_candidate(candidate,map_path,output/'selection');s=load_selection(selected)
    source=saved_grid(map_path);m=Map(map_path)
    if source['origin'][2] != 0:raise ValueError('Selected slice is in the pinned axis-aligned saved-map frame')
    a,b=[g['position'] for g in s['goals'][:2]];cut=-2.5
    if not min(a[0],b[0])+.25 < cut < max(a[0],b[0])-.25:
        raise ValueError('Slice must separate the entire G1/G2 acceptance disks')
    y_range=capsule_slice(a,b,cut,s['constraints']['narrow_center_band_m'])
    # Keep every source unknown blocked. Record witness cell values so the
    # obstruction's occupied-only basis is independently inspectable.
    checker=RectangleGrid(source)
    results=[]
    for extra in [0.,.0025,.005,.0075,.01]:
        local_y=[y-source['origin'][1] for y in y_range]
        result=exclude_slice(checker,cut-source['origin'][0],local_y,extra=extra)
        if extra == .005 and not result['all_poses_blocked'] and result['clear_example'] is None:
            coarse=result
            result=exclude_slice(checker,cut-source['origin'][0],local_y,extra=extra,
                                 y_step=.000625,yaw_step=math.radians(.03125))
            result['previous_coarse_search']=coarse
        if result['clear_example'] is not None:
            result['clear_example_source_grid_clear']=RectangleGrid(source).clear_local(result['clear_example'],extra=extra)
        # Independent polygon checks at corners and centers of each certified
        # interval catch axis/frame/cell-index errors; the continuous bound is
        # supplied by SAT's interval inequalities, not these finite samples.
        for witness in result['witnesses']:
            witness['source_cell_cost']=source['data'][witness['cell_index']]
            row,col=divmod(witness['cell_index'],source['width']);r=source['resolution']
            square=[[col*r,row*r],[(col+1)*r,row*r],[(col+1)*r,(row+1)*r],[col*r,(row+1)*r]]
            lo,hi,alo,ahi=witness['bounds']
            for y in [lo,(lo+hi)/2,hi]:
                for yaw in [alo,(alo+ahi)/2,ahi]:
                    poly=transform_polygon([[.75+extra,.43+extra],[.75+extra,-.43-extra],[-.75-extra,-.43-extra],[-.75-extra,.43+extra]],[result['local_x'],y,yaw])
                    if not polygons_overlap(poly,square):raise ValueError('Independent interval-witness check failed')
        if result['all_poses_blocked']:
            area=sum((q['bounds'][1]-q['bounds'][0])*(q['bounds'][3]-q['bounds'][2]) for q in result['witnesses'])
            if abs(area-(y_range[1]-y_range[0])*math.pi)>1e-8:raise ValueError('Incomplete certificate area')
        result['independent_witness_samples_checked']=9*len(result['witnesses'])
        results.append(result)
    excluded=[r['extra_half_extent_m'] for r in results if r['all_poses_blocked']]
    state=geometry_analysis(next(r for r in results if r['extra_half_extent_m']==.0075))
    report=dict(result='BLOCKED_GEOMETRY' if excluded else 'GEOMETRY_UNRESOLVED',
                **state, full_route_pass=False,motion_authorized_by_report=False,
                selection_sha256=digest(selected),source_map=s['source_map'],goals=s['goals'],
                code_sha256={str(p.relative_to(ROOT)):digest(p) for p in [Path(__file__).resolve(),ROOT/'scripts/stage4_route/route_geometry.py']},
                padded_footprint_m=[1.5,.86],existing_padding_per_side_m=.05,
                mandatory_slice=dict(map_x_m=cut,map_y_range_m=y_range,band_m=.35,
                    reason='Every continuous G1-region to G2-region traversal crosses this x inside the full perpendicular capsule'),
                extra_envelope_excluded_at_or_above_m=min(excluded) if excluded else None,
                interval_certificates=results,
                uncertainty_budget=dict(map_boundary_m=None,registration_local_m=None,tracking_xy_m=None,
                    tracking_yaw_rad=None,latency_displacement_m=None,body_variation_m=None,braking_reach_m=None,
                    note='No measured sub-centimetre combined bound. Existing padding is preserved in full; no double spending.'),
                gate_G2_passed=False,
                limitations=['Nominal connected SE(2) path unresolved; a clear slice pose is not a route',
                    'Obstruction is conditional on the saved raster and additional rectangular envelope',
                    'No same-session sensor registration, source-map projection defect proof, or physical width measurement',
                    'World collision geometry has no independently established map/world transform',
                    'No Start pose, measured braking envelope, complete contact coverage or predictive DWB body critic'],
                next_action='Continue bounded stationary/offline sensor boundary and registration investigation. A reference-raster exclusion does not block passive evidence acquisition. Prove corrections in a separately bound derived map; certify connected swept-body geometry and uncertainty before motion.')
    write_new(output/'geometry-report.json',report)
    plot(m,s,report,output/'geometry.png')
    return report


def plot(m,selection,report,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle,Polygon
    fig,ax=plt.subplots(figsize=(12,6));o=m.meta['origin'];res=m.meta['resolution']
    ax.imshow(m.image,cmap='gray',origin='upper',extent=[o[0],o[0]+m.meta['width']*res,o[1],o[1]+m.meta['height']*res],vmin=0,vmax=255)
    a,b=[g['position'] for g in selection['goals'][:2]]
    dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy);nx,ny=-dy/length*.35,dx/length*.35
    ax.add_patch(Polygon([[a[0]+nx,a[1]+ny],[b[0]+nx,b[1]+ny],[b[0]-nx,b[1]-ny],[a[0]-nx,a[1]-ny]],fill=False,edgecolor='blue',linestyle='--',label='.35 m center band'))
    for g in selection['goals'][:2]:
        ax.add_patch(Circle(g['position'],.25,fill=False,color='green'));ax.text(*g['position'],g['name'],color='green')
    q=report['mandatory_slice'];ax.plot([q['map_x_m']]*2,q['map_y_range_m'],color='red',linewidth=3,label='Mandatory blocked-envelope slice')
    for result in report['interval_certificates']:
        if result['extra_half_extent_m']==.005 and result['clear_example']:
            p=result['clear_example'];p=[p[0]+o[0],p[1]+o[1],p[2]]
            ax.add_patch(Polygon(transform_polygon([[.755,.435],[.755,-.435],[-.755,-.435],[-.755,.435]],p),fill=False,edgecolor='orange',label='Nominal slice pose + 5 mm (no route proof)'))
    ax.set(xlim=(-5.3,1),ylim=(5.5,8.1),xlabel='Saved map x (m)',ylabel='Saved map y (m)',title='G1→G2 necessary geometry gate: no certified physical margin')
    ax.set_aspect('equal');ax.legend(loc='upper left');fig.tight_layout();fig.savefig(path,dpi=160);plt.close(fig)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',type=Path,required=True);p.add_argument('--map',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(argv);r=audit(a.candidate,a.map,a.output)
    print(json.dumps({k:r[k] for k in ['result','full_route_pass','extra_envelope_excluded_at_or_above_m','gate_G2_passed']}))
    return 2 if r['result']=='BLOCKED_GEOMETRY' else 3


if __name__=='__main__':sys.exit(main())
