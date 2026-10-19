"""Closed-cell SAT and conservative continuous cross-section exclusion.

A blocked slice is a necessary route obstruction for the stated envelope only.
It neither certifies a connected nominal route nor measures physical clearance.
Interval witnesses use upper bounds on separating-axis distances and lower
bounds on supports, with a numerical guard directed against false exclusion.
"""
import math
import time
import numpy as np


class RectangleGrid:
    def __init__(self, grid, *, half_length=.75, half_width=.43):
        self.grid = grid
        self.length, self.width = half_length, half_width
        self.resolution = grid['resolution']
        self.data = np.asarray(grid['data']).reshape(grid['height'], grid['width'])
        if (self.resolution <= 0 or min(half_length, half_width) <= 0 or
                not all(math.isfinite(v) for v in [self.resolution, half_length, half_width, *grid['origin']])):
            raise ValueError('Invalid geometry')
        rows, cols = np.where(self.data >= 254)
        self.ids = rows * grid['width'] + cols
        self.points = np.column_stack(((cols+.5)*self.resolution, (rows+.5)*self.resolution))

    def local_pose(self, pose):
        ox, oy, a = self.grid['origin'];c, s = math.cos(a), math.sin(a)
        x, y = pose[0]-ox, pose[1]-oy
        return [c*x+s*y, -s*x+c*y, pose[2]-a]

    def clear_local(self, pose, *, extra=0.):
        x, y, yaw = pose;c, s = math.cos(yaw), math.sin(yaw)
        length, width = self.length+extra, self.width+extra;h = self.resolution/2
        extent_x, extent_y = length*abs(c)+width*abs(s), length*abs(s)+width*abs(c)
        if (x-extent_x < 0 or y-extent_y < 0 or
                x+extent_x > self.grid['width']*self.resolution or
                y+extent_y > self.grid['height']*self.resolution):
            return False
        col, row = math.floor(x/self.resolution), math.floor(y/self.resolution)
        if self.data[row, col] >= 253:
            return False
        dx, dy = (self.points-[x, y]).T
        return not bool(np.any(
            (abs(dx) <= extent_x+h+1e-10) & (abs(dy) <= extent_y+h+1e-10) &
            (abs(c*dx+s*dy) <= length+h*(abs(c)+abs(s))+1e-10) &
            (abs(-s*dx+c*dy) <= width+h*(abs(c)+abs(s))+1e-10)))

    def clear(self, pose, *, extra=0.):
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose) or not math.isfinite(extra) or extra < 0:
            raise ValueError('Invalid pose/envelope')
        return self.clear_local(self.local_pose(pose), extra=extra)

    def blocking_cell(self, x, bounds, *, extra):
        """Return a single cell intersecting EVERY pose in this local y/yaw box.

        sin/cos are 1-Lipschitz. Rotation changes a projected relative vector
        by at most its norm*half_yaw_range; translation adds half_y_range.
        Four strict SAT overlaps bounded this way suffice for every pose.
        Rectangle orientation has pi symmetry, so [-pi/2,pi/2] covers all yaw.
        """
        lo, hi, alo, ahi = bounds
        y, yaw = (lo+hi)/2, (alo+ahi)/2
        hy, ha = (hi-lo)/2, (ahi-alo)/2
        c, s = math.cos(yaw), math.sin(yaw)
        length, width = self.length+extra, self.width+extra;h = self.resolution/2
        radius = math.hypot(length, width)+h*math.sqrt(2)
        near = (abs(self.points[:, 0]-x) <= radius) & (abs(self.points[:, 1]-y) <= radius+hy)
        points, ids = self.points[near], self.ids[near]
        if not len(points):return None
        dx, dy = (points-[x, y]).T
        ac, ass = max(0., abs(c)-ha), max(0., abs(s)-ha)
        error = np.hypot(dx, dy)*ha+hy
        witness = np.flatnonzero(
            (abs(dx) <= length*ac+width*ass+h-1e-10) &
            (abs(dy)+hy <= length*ass+width*ac+h-1e-10) &
            (abs(c*dx+s*dy)+error <= length+h*(ac+ass)-1e-10) &
            (abs(-s*dx+c*dy)+error <= width+h*(ac+ass)-1e-10))
        return int(ids[witness[0]]) if len(witness) else None


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
