"""Shared closed-cell filled-body geometry; does not qualify physical safety."""
import math
import numpy as np
from scipy.spatial import cKDTree

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



def grid_revision(grid,encoding):
    """Shared immutable geometry identity; receipt/source times stay separate."""
    import hashlib
    import json
    data=tuple((0 if v==0 else 254) if encoding=='occupancy' else v for v in grid['data'])
    frozen=dict(width=grid['width'],height=grid['height'],resolution=grid['resolution'],origin=tuple(grid['origin']),data=data)
    return frozen,hashlib.sha256(json.dumps(frozen,sort_keys=True,allow_nan=False).encode()).hexdigest()
