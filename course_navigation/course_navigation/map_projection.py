"""Bounded occupancy grid: observed rays are free, unobserved space is unknown."""
import math
import numpy as np


class Grid:
    def __init__(self, resolution=0.05, origin=(-10., -8.), size=(20., 16.)):
        if resolution <= 0 or min(size) <= 0:
            raise ValueError('Invalid map dimensions')
        self.resolution = resolution
        self.origin = np.array(origin)
        self.width, self.height = [math.ceil(v / resolution) for v in size]
        self.free = np.zeros((self.height, self.width), dtype=bool)
        self.occupied = np.zeros_like(self.free)

    def cells(self, xy):
        return np.floor((np.asarray(xy) - self.origin) / self.resolution).astype(int)

    def inside(self, cells):
        return ((cells[..., 0] >= 0) & (cells[..., 0] < self.width)
                & (cells[..., 1] >= 0) & (cells[..., 1] < self.height))

    def obstacles(self, points, min_height=0.15, max_height=2.):
        points = np.asarray(points).reshape(-1, 3)
        valid = np.isfinite(points).all(axis=1) & (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
        cells = self.cells(points[valid, :2])
        cells = cells[self.inside(cells)]
        self.occupied.fill(False)
        if len(cells):
            self.occupied[cells[:, 1], cells[:, 0]] = True

    def observe(self, origin, points, max_rays=720):
        start = self.cells(np.asarray(origin)[:2])
        if not self.inside(start):
            return
        points = np.asarray(points).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        stride = max(1, math.ceil(len(points) / max_rays))
        for endpoint in self.cells(points[::stride, :2]):
            # Integer DDA traversal, ending before the observed surface.
            steps = int(np.max(np.abs(endpoint - start)))
            if steps == 0:
                continue
            # Bound ray work even for a 120 m return outside this finite map.
            steps_used = min(steps, max(self.width, self.height) * 2)
            indices = np.arange(steps_used)
            line = np.rint(start + indices[:, None] * (endpoint - start) / steps).astype(int)
            line = line[self.inside(line)]
            self.free[line[:, 1], line[:, 0]] = True

    def data(self):
        grid = np.full(self.free.shape, -1, dtype=np.int8)
        grid[self.free] = 0
        grid[self.occupied] = 100
        return grid
