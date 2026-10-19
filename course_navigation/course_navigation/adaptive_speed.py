"""Geometry governor; no ROS, truth, waypoint coordinates or inverse calibration."""
from dataclasses import dataclass
import math

import numpy as np


def fresh(ros_now, wall_now, stamp, receipt, timeout):
    return (stamp is not None and receipt is not None
            and 0 <= ros_now - stamp <= timeout and 0 <= wall_now - receipt <= timeout)


class CommandLimit:
    """Optional adapter watchdog, also caps smoother tails/recovery translation.

    Nav2 zero means 'no limit', so reject it here. A real stop is a zero Twist.
    Moving twists scale uniformly to retain curvature; intentional in-place
    rotation (planar speed <=1e-6 m/s) retains yaw with a valid heartbeat.
    """
    def __init__(self, ceiling, timeout):
        self.ceiling, self.timeout = ceiling, timeout
        self.limit, self.stamp, self.receipt = None, None, None

    def receive(self, limit, percentage, stamp, receipt):
        self.limit = min(limit, self.ceiling) if math.isfinite(limit) and limit > 0 and not percentage else None
        self.stamp, self.receipt = stamp, receipt

    def apply(self, command, ros_now, wall_now):
        if self.limit is None or not fresh(ros_now, wall_now, self.stamp, self.receipt, self.timeout):
            return 0., 0., 0.
        x, y, yaw = command
        speed = math.hypot(x, y)
        scale = min(1., self.limit/max(speed, 1e-9))
        yaw_scale = scale if speed > 1e-6 else 1.
        return x*scale, y*scale, yaw*yaw_scale


@dataclass(frozen=True)
class Config:
    w_ref: float  # Required approximate user-measured obstacle-to-obstacle width, metres.
    open_command: float = .43
    narrow_command: float = .11
    open_physical: float = .50
    narrow_physical: float = .18
    width_margin_m: float = .10
    min_side_clearance_m: float = .15
    release_margin_m: float = .03
    release_hold_s: float = .5
    lookahead_m: float = 1.0
    deceleration: float = .20  # Conservative physical m/s²; not smoother acceleration.
    latency_s: float = .80  # Costmap publish + governor + smoother/actuation allowance.
    braking_margin_m: float = .20
    sample_step_m: float = .10
    side_search_m: float = 2.

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('Measured w_ref and all governor parameters must be finite and positive')
        if self.narrow_command > self.open_command or self.narrow_physical >= self.open_physical:
            raise ValueError('NARROW must be slower than OPEN')
        if self.w_ref + self.width_margin_m + self.release_margin_m >= 2*self.side_search_m:
            raise ValueError('side_search_m too short to establish OPEN clearance')


class CostGrid:
    """Raw Nav2 costs: lethal=254, unknown=255. Inflation is not wall geometry."""
    def __init__(self, cells, resolution, origin):
        self.cells = np.asarray(cells)
        if (self.cells.ndim != 2 or min(self.cells.shape) == 0
                or not math.isfinite(resolution) or resolution <= 0
                or len(origin) != 3 or not all(math.isfinite(v) for v in origin)
                or not np.isfinite(self.cells).all()
                or np.any(self.cells < 0) or np.any(self.cells > 255)):
            raise ValueError('Invalid raw costmap')
        self.resolution, self.origin = resolution, origin
        self.c, self.s = math.cos(origin[2]), math.sin(origin[2])
        # Represent off-map space by a one-cell unknown boundary, never free space.
        padded = np.pad(self.cells, 1, constant_values=255)
        rows, cols = np.nonzero(padded >= 254)
        xy = np.column_stack(((cols-.5)*resolution, (rows-.5)*resolution))
        self.obstacles = xy @ np.array([[self.c, self.s], [-self.s, self.c]]) + origin[:2]

    def clearance(self, pose, footprint, search):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        local = (self.obstacles - (x, y)) @ np.array([[c, -s], [s, c]])
        # Project cell *areas*, not just centers; conservative for rotated cells.
        angle = yaw - self.origin[2]
        half = self.resolution/2 * (abs(math.cos(angle)) + abs(math.sin(angle)))
        xmin, ymin = np.min(footprint, axis=0)
        xmax, ymax = np.max(footprint, axis=0)
        strip = local[(local[:, 0]+half >= xmin) & (local[:, 0]-half <= xmax)]
        left, right = search, search
        if len(strip):
            if np.any((strip[:, 1]-half <= ymax) & (strip[:, 1]+half >= ymin)):
                return 0., 0., 0.  # Represented obstacle/unknown overlaps the footprint envelope.
            above, below = strip[strip[:, 1] > ymax], strip[strip[:, 1] < ymin]
            if len(above):
                left = min(left, max(0., float(np.min(above[:, 1]-half-ymax))))
            if len(below):
                right = min(right, max(0., float(np.min(ymin-below[:, 1]-half))))
        # Outside map cannot produce spurious OPEN even beyond the boundary ring.
        center = np.array([x-self.origin[0], y-self.origin[1]]) @ np.array(
            [[self.c, -self.s], [self.s, self.c]])
        if not (0 <= center[0] < self.cells.shape[1]*self.resolution
                and 0 <= center[1] < self.cells.shape[0]*self.resolution):
            return 0., 0., 0.
        return left + (ymax-ymin) + right, left, right


def path_samples(path, pose, horizon, step):
    points = np.asarray(path, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('Missing/invalid path')
    # RPP already prunes the path; bound the projection search before later loops.
    # Project onto segments rather than connecting backwards to a nearest vertex:
    # that connector would invent a pi-radian turn on an ordinary straight path.
    delta = np.diff(points, axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    cumulative = np.r_[0., np.cumsum(lengths)]
    candidates = np.flatnonzero((cumulative[:-1] <= horizon+1.) & (lengths > 1e-6))
    if not len(candidates):
        raise ValueError('Degenerate path')
    offsets = np.array(pose[:2])-points[candidates]
    ratios = np.clip(np.sum(offsets*delta[candidates], axis=1)/lengths[candidates]**2, 0., 1.)
    projections = points[candidates]+ratios[:, None]*delta[candidates]
    closest = int(np.argmin(np.linalg.norm(projections-pose[:2], axis=1)))
    start = candidates[closest]
    points = np.vstack((projections[closest], points[start+1:]))
    samples, distance = [], 0.
    for a, b in zip(points[:-1], points[1:]):
        delta = b-a
        length = float(np.linalg.norm(delta))
        if length < 1e-6:
            continue
        yaw = math.atan2(delta[1], delta[0])
        extent = min(length, horizon-distance)
        for offset in np.linspace(0., extent, max(1, math.ceil(extent/step))+1):
            p = a + delta*(offset/length)
            samples.append((float(p[0]), float(p[1]), yaw, distance+offset))
        distance += extent
        if distance >= horizon:
            break
    if not samples:
        raise ValueError('Degenerate path')
    return samples


@dataclass
class Decision:
    mode: str
    command_limit: float
    reason: str
    horizon: float
    width: float = None
    left: float = None
    right: float = None
    curvature: float = None
    heading_error: float = None
    tight: bool = None
    release_eligible: bool = False
    clear_duration_s: float = 0.


class Governor:
    def __init__(self, config):
        self.config = config
        self.mode, self.clear_since, self.last_time = 'NARROW', None, None
        self.restriction_horizon = config.lookahead_m

    def update(self, now, grid, path, pose, footprint, speed, healthy=True):
        cfg = self.config
        valid = (healthy and speed is not None and math.isfinite(speed) and speed >= 0
                 and math.isfinite(now) and grid is not None and pose is not None
                 and footprint is not None)
        rewind = self.last_time is not None and now < self.last_time
        self.last_time = now
        horizon = cfg.lookahead_m
        width = left = right = curvature = None
        heading_error = tight = None
        reason, release = 'stale_or_missing', False
        if valid and not rewind:
            # Braking distance to NARROW in physical units, plus latency and margin.
            horizon = max(horizon, self.restriction_horizon, max(0., speed*speed-cfg.narrow_physical**2)
                          /(2*cfg.deceleration) + speed*cfg.latency_s + cfg.braking_margin_m)
            try:
                fp = np.asarray(footprint, dtype=float)
                if fp.ndim != 2 or fp.shape[1] != 2 or len(fp) < 3 or not np.isfinite(fp).all():
                    raise ValueError('Invalid footprint')
                if not all(math.isfinite(v) for v in pose):
                    raise ValueError('Invalid pose')
                samples = path_samples(path, pose, horizon, cfg.sample_step_m)
                if samples[-1][3] < horizon-1e-6:
                    # A short fresh path cannot establish clearance throughout
                    # the braking/restriction horizon, even near a goal.
                    raise ValueError('Path does not cover checking horizon')
                # Keep the actual full footprint, including its rear, in the
                # minimum. A clear path ahead alone cannot release an exit.
                values = [grid.clearance(pose, fp, cfg.side_search_m)]
                values += [grid.clearance(p[:3], fp, cfg.side_search_m) for p in samples]
                width, left, right = np.min(values, axis=0).tolist()
                headings = np.unwrap([p[2] for p in samples])
                distances = np.array([p[3] for p in samples])
                # Accumulate turn over a >=.20m window to avoid grid stair-step curvature.
                curvature = 0.
                for i in range(len(samples)):
                    j = np.searchsorted(distances, distances[i]+.3)
                    if j < len(samples):
                        curvature = max(curvature, abs(headings[j]-headings[i])/
                                        max(.2, distances[j]-distances[i]))
                heading_error = abs(math.atan2(math.sin(headings[0]-pose[2]), math.cos(headings[0]-pose[2])))
                tight = width <= cfg.w_ref+cfg.width_margin_m or min(left, right) <= cfg.min_side_clearance_m
                release = (width > cfg.w_ref+cfg.width_margin_m+cfg.release_margin_m
                           and min(left, right) > cfg.min_side_clearance_m+cfg.release_margin_m)
                # RPP owns heading/curvature speed regulation. Those diagnostics
                # never latch NARROW without restrictive footprint geometry.
                reason = ('geometry' if tight else 'release_hold' if release else 'release_margin')
                if not tight and self.mode == 'OPEN':
                    reason = 'open'
            except (ValueError, TypeError, OverflowError):
                valid = False
                reason = 'invalid_geometry'
        if not valid or rewind or tight:
            self.mode, self.clear_since = 'NARROW', None
            if valid and not rewind and tight:
                # Retain detection reach while braking. Otherwise the shrinking
                # horizon can hide a choke before entry and re-enable OPEN.
                self.restriction_horizon = horizon
        elif self.mode == 'NARROW':
            if release:
                if self.clear_since is None:
                    self.clear_since = now
                if now-self.clear_since >= cfg.release_hold_s:
                    self.mode, reason = 'OPEN', 'open'
            else:
                self.clear_since = None
        if self.mode == 'OPEN':
            self.restriction_horizon = cfg.lookahead_m
        limit = cfg.open_command if self.mode == 'OPEN' else cfg.narrow_command
        # Downward-only guard against measured overspeed. This is not a calibrated
        # inverse or a certified physical speed bound; independent truth is recorded.
        if valid:
            target = cfg.open_physical if self.mode == 'OPEN' else cfg.narrow_physical
            limit *= min(1., target/max(speed, target))
        duration = 0. if self.clear_since is None else max(0., now-self.clear_since)
        return Decision(self.mode, limit, reason, horizon, width, left, right, curvature,
                        heading_error, tight, bool(valid and not rewind and release), duration)
