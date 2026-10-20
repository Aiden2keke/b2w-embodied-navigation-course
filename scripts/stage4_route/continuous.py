"""Conservative closed-cell rectangular-body certificates for SE(2) primitives.

Proof uses separating-axis clearance lower bounds and a bound on every body
point's motion, with adaptive subdivision. Numerical search failure is unresolved;
this never certifies localization, controller tracking, contacts or dynamics.
"""
import math
import time
import numpy as np
from scipy.spatial import cKDTree

from .route_geometry import RectangleGrid


class ClearanceGrid(RectangleGrid):
    def __init__(self, grid):
        super().__init__(grid)
        self.tree = cKDTree(self.points)

    def clearance(self, pose, *, cap=.5):
        x, y, a = self.local_pose(pose)
        c, s = math.cos(a), math.sin(a)
        lx, ly = self.length*abs(c)+self.width*abs(s), self.length*abs(s)+self.width*abs(c)
        boundary = min(x-lx, y-ly, self.grid['width']*self.resolution-x-lx,
                       self.grid['height']*self.resolution-y-ly)
        if boundary <= 0:
            return boundary
        col, row = math.floor(x/self.resolution), math.floor(y/self.resolution)
        if self.data[row, col] >= 253:
            return 0.
        h = self.resolution/2
        radius = math.hypot(self.length, self.width) + h*math.sqrt(2) + cap
        ids = self.tree.query_ball_point([x, y], radius)
        if not ids:
            return min(boundary, cap)
        dx, dy = (self.points[ids]-[x,y]).T
        gaps = np.maximum.reduce([
            abs(dx)-lx-h, abs(dy)-ly-h,
            abs(c*dx+s*dy)-self.length-h*(abs(c)+abs(s)),
            abs(-s*dx+c*dy)-self.width-h*(abs(c)+abs(s))])
        # Each positive SAT gap is a lower bound on Euclidean separation;
        # points outside the search radius have separation >=cap.
        return float(min(boundary, cap, np.min(gaps)))


def certify_path(path, grids, *, extra=0., max_depth=20, cpu_limit=30., max_checks=100000):
    if (not path or not grids or not math.isfinite(extra) or extra < 0 or
            max_depth < 0 or cpu_limit <= 0 or max_checks <= 0 or
            any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in path)):
        raise ValueError('Invalid continuous geometry inputs')
    started = time.process_time()
    checkers = [ClearanceGrid(g) for g in grids]
    checked = 0
    leaves = 0
    failure = None
    witness = None
    minimum = math.inf
    radius = math.hypot(.75,.43)
    cache = {}

    def margin(p):
        nonlocal checked, minimum
        k = tuple(p)
        if k not in cache:
            cache[k] = min(g.clearance(p) for g in checkers)-extra
            checked += 1
            minimum = min(minimum, cache[k])
        return cache[k]

    todo = [(list(a),list(b),0) for a,b in zip(path,path[1:])]
    if len(path)==1:
        todo = [(list(path[0]),list(path[0]),0)]
    while todo:
        if checked >= max_checks or time.process_time()-started >= cpu_limit:
            failure = 'continuous_search_budget_unresolved';break
        a,b,depth = todo.pop()
        ma,mb = margin(a),margin(b)
        if min(ma,mb) <= 1e-9:
            failure = 'filled_footprint_collision_or_unknown'
            witness = a if ma<=mb else b
            break
        yaw_delta = math.remainder(b[2]-a[2],math.tau)
        movement = math.dist(a[:2],b[:2])+radius*abs(yaw_delta)
        if movement < min(ma,mb)-1e-9:
            leaves += 1;continue
        if depth>=max_depth:
            failure = 'continuous_subdivision_unresolved';break
        mid = [(a[0]+b[0])/2,(a[1]+b[1])/2,a[2]+yaw_delta/2]
        todo.extend([(a,mid,depth+1),(mid,b,depth+1)])
    return dict(continuous_clearance_proven=failure is None, motion_ready=False,
                failure=failure, first_blocked_pose=witness, checked_poses=checked,
                certified_intervals=leaves, unresolved_intervals=len(todo),
                minimum_checked_clearance_lower_bound_m=minimum if math.isfinite(minimum) else None,
                extra_isotropic_body_margin_m=extra, cpu_s=time.process_time()-started,
                scope='Interpolated center/yaw primitives against closed grid cells only; no controller or physical claim')
