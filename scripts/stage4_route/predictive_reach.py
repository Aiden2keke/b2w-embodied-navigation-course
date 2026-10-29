"""Offline prepared reachable-set geometry kernel, NOT a qualified control path.

Caller must independently certify maximum center translation and absolute yaw
excursion over source/receipt/abort latency PLUS the entire braking transient.
This kernel neither estimates those bounds from commands nor authorizes output.
Every body point moves by <=translation + circumradius*yaw. Eroding the current
filled-body clearance by that amount and uncertainty therefore bounds *all*
intermediate reachable poses, including rotation, lateral drift and interiors.
The conservative isotropic bound can refuse a feasible narrow maneuver.
"""
import hashlib
import json
import math
import time
import numpy as np
from .continuous import ClearanceGrid


class PreparedReachGrid:
    def __init__(self, grid, *, frame, epoch, source, receipt, encoding):
        if (frame not in ('map', 'odom') or not isinstance(epoch, str) or not epoch or
                any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in (source, receipt)) or
                encoding not in ('occupancy', 'costmap')):
            raise ValueError('Invalid grid frame/epoch/time/encoding')
        data = tuple(grid['data'])
        if (type(grid['width']) is not int or type(grid['height']) is not int or
                min(grid['width'], grid['height']) <= 0 or len(data) != grid['width']*grid['height'] or
                any(type(v) is not int or not (-1 <= v <= 100 if encoding=='occupancy' else 0 <= v <= 255) for v in data)):
            raise ValueError('Invalid grid cells')
        # Occupancy values other than explicitly observed free remain blocked.
        # Costmap inflation is retained; lethal, inscribed and unknown all block.
        normalized = tuple((0 if v==0 else 254) if encoding=='occupancy' else (254 if v>=253 else v) for v in data)
        frozen = dict(width=grid['width'], height=grid['height'], resolution=grid['resolution'],
                      origin=tuple(grid['origin']), data=normalized)
        self.revision = hashlib.sha256(json.dumps(dict(grid=frozen, frame=frame, epoch=epoch,
                                                       encoding=encoding), sort_keys=True, allow_nan=False).encode()).hexdigest()
        self.checker = ClearanceGrid(frozen)
        self.checker.data.flags.writeable = False
        self.checker.points.flags.writeable = False
        self.frame, self.epoch = frame, epoch
        self.source, self.receipt = source, receipt

    def evaluate(self, pose, *, frame, epoch, sim, wall, translation_reach, yaw_reach,
                 uncertainty, max_age_s=2., cycle_budget_s=.05):
        started = time.monotonic()
        reason = None;clearance = reserve = None
        values = [sim, wall, translation_reach, yaw_reach, uncertainty, max_age_s, cycle_budget_s]
        if (len(pose)!=3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in [*pose, *values]) or
                min(translation_reach, yaw_reach, uncertainty) < 0 or min(max_age_s, cycle_budget_s) <= 0):
            reason = 'invalid_reach_inputs'
        elif frame != self.frame or epoch != self.epoch:
            reason = 'reach_frame_or_epoch_changed'
        elif not 0 <= sim-self.source <= max_age_s or not 0 <= wall-self.receipt <= max_age_s:
            reason = 'stale_reach_grid'
        else:
            reach = translation_reach+math.hypot(.75, .43)*yaw_reach+uncertainty
            clearance = self.checker.clearance(pose)
            reserve = clearance-reach
            if reserve <= 1e-9:reason = 'reachable_filled_body_blocked_or_unresolved'
        elapsed = time.monotonic()-started
        if elapsed > cycle_budget_s:reason = 'reach_cycle_deadline'
        return dict(clear=reason is None, motion_ready=False, failure=reason,
                    current_clearance_lower_bound_m=clearance, reachable_reserve_lower_bound_m=reserve,
                    grid_revision_sha256=self.revision, wall_s=elapsed,
                    scope='Offline geometry only; caller-supplied stop/uncertainty bounds are NOT qualified')
