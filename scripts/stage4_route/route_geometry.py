"""Closed-cell SAT and conservative continuous cross-section exclusion.

A blocked slice is a necessary route obstruction for the stated envelope only.
It neither certifies a connected nominal route nor measures physical clearance.
Interval witnesses use upper bounds on separating-axis distances and lower
bounds on supports, with a numerical guard directed against false exclusion.
"""
import math
import time
import numpy as np


def geometry_analysis(slice_result):
    """Describe a necessary geometry test without claiming path or motion readiness.

    A reference-grid exclusion blocks that envelope's route, while still allowing
    passive evidence collection to identify map/model errors. A sampled clear
    pose proves only that the slice is not wholly excluded.
    """
    if slice_result['all_poses_blocked']:
        status = 'EXCLUDED'
    elif slice_result.get('clear_example') is not None:
        status = 'CLEAR_POSE_FOUND'
    else:
        status = 'UNRESOLVED'
    return dict(slice_status=status, connected_path_status='UNRESOLVED',
                sensor_map_status='UNRESOLVED', motion_ready=False,
                next_stage='GEOMETRY_EVIDENCE')


from course_navigation.stage4_geometry import RectangleGrid

def capsule_slice(a, b, x, radius):
    """Conservative full vertical intersection of a segment's round capsule."""
    from .evidence import distance
    if radius <= 0 or not min(a[0], b[0]) <= x <= max(a[0], b[0]) or a[0] == b[0]:
        raise ValueError('Slice must be strictly between nonvertical segment endpoints')
    mid = a[1]+(x-a[0])*(b[1]-a[1])/(b[0]-a[0])
    lower, upper = min(a[1], b[1])-radius, max(a[1], b[1])+radius
    lo, hi = lower, mid
    for _ in range(60):
        m = (lo+hi)/2
        if distance((x,m),a,b) > radius:lo=m
        else:hi=m
    lower=lo-1e-10
    lo, hi = mid, upper
    for _ in range(60):
        m = (lo+hi)/2
        if distance((x,m),a,b) > radius:hi=m
        else:lo=m
    return [lower,hi+1e-10]


def exclude_slice(checker, x, y_range, *, extra, cpu_limit=120., y_step=.005,
                  yaw_step=math.radians(.25), max_boxes=100000):
    if (extra < 0 or cpu_limit <= 0 or y_step <= 0 or yaw_step <= 0 or
            len(y_range) != 2 or y_range[0] >= y_range[1] or
            not all(math.isfinite(v) for v in [x,*y_range,extra,cpu_limit,y_step,yaw_step])):
        raise ValueError('Invalid bounded slice search')
    initial=[*y_range,-math.pi/2,math.pi/2];todo=[initial];witnesses=[]
    unresolved=0;examples=[];tested=0;clear_example=None;started=time.process_time()
    while todo and time.process_time()-started < cpu_limit and tested < max_boxes:
        bounds=todo.pop();tested+=1
        cell=checker.blocking_cell(x,bounds,extra=extra)
        if cell is not None:
            witnesses.append(dict(bounds=bounds,cell_index=cell));continue
        lo,hi,alo,ahi=bounds
        pose=[x,(lo+hi)/2,(alo+ahi)/2]
        if checker.clear_local(pose,extra=extra):
            clear_example=pose;break
        if hi-lo <= y_step and ahi-alo <= yaw_step:
            unresolved+=1
            if len(examples)<10:examples.append(bounds)
        elif (hi-lo)/y_step >= (ahi-alo)/yaw_step:
            mid=(lo+hi)/2;todo.extend([[lo,mid,alo,ahi],[mid,hi,alo,ahi]])
        else:
            mid=(alo+ahi)/2;todo.extend([[lo,hi,alo,mid],[lo,hi,mid,ahi]])
    blocked=not todo and not unresolved and clear_example is None
    return dict(all_poses_blocked=blocked,initial_bounds=initial,local_x=x,
                extra_half_extent_m=extra,boxes_tested=tested,witnesses=witnesses,
                unresolved_boxes=unresolved,pending_boxes=len(todo),
                unresolved_examples=examples,clear_example=clear_example,
                cpu_s=time.process_time()-started,cpu_limit_s=cpu_limit,
                y_resolution_m=y_step,yaw_resolution_rad=yaw_step,
                result='BLOCKED_SLICE' if blocked else 'UNRESOLVED',
                scope='Saved closed cells and stated rectangle/envelope; no physical/registration or connected-route proof')
