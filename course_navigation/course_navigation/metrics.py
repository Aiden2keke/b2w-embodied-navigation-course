"""One goal's navigation measurements; missing evidence stays null."""
import math


class RunMetrics:
    def __init__(self, start, goal, tolerance=0.3, cooldown=1.):
        self.start, self.goal = start, goal
        self.tolerance, self.cooldown = tolerance, cooldown
        self.previous = None
        self.path_length = 0.
        self.collisions = 0
        self.last_collision = -math.inf
        self.collision_observed = False
        self.in_contact = False
        self.goal_position = None
        self.planning_latency_ms = None
        self.planning_status = None
        self.pose_stamp = None

    def pose(self, x, y, goal_position=None, stamp=None):
        if not math.isfinite(x) or not math.isfinite(y):
            return
        if stamp is not None:
            if not math.isfinite(stamp) or (self.pose_stamp is not None and stamp <= self.pose_stamp):
                return
            self.pose_stamp = stamp
        if self.previous is not None:
            self.path_length += math.dist(self.previous, (x, y))
        self.previous = (x, y)
        self.goal_position = goal_position if goal_position is not None else self.previous

    def collision(self, time, force_norm, threshold):
        if not math.isfinite(force_norm):
            return
        self.collision_observed = True
        contact = force_norm > threshold
        if contact and not self.in_contact and time - self.last_collision >= self.cooldown:
            self.collisions += 1
            self.last_collision = time
        self.in_contact = contact

    def finish(self, end, action_succeeded, reason):
        reached = self.goal_position is not None and math.dist(self.goal_position, self.goal) <= self.tolerance
        return {'success': bool(action_succeeded and reached), 'end_reason': reason,
                'collision_count': self.collisions if self.collision_observed else None,
                'collision_missing_reason': None if self.collision_observed else 'No collision feedback',
                'path_length_m': self.path_length if self.previous is not None else None,
                'path_length_missing_reason': None if self.previous is not None else 'No valid localized base poses',
                'time_to_goal_s': end - self.start if end >= self.start else None,
                'time_missing_reason': None if end >= self.start else 'Clock moved backwards',
                'planning_latency_ms': self.planning_latency_ms,
                'planning_latency_missing_reason': None if self.planning_latency_ms is not None else 'No planner action result',
                'planning_measurement_status': self.planning_status,
                'planning_latency_definition': 'ComputePathToPose action round trip, wall clock',
                'path_source': 'slam_base_link', 'goal_tolerance_m': self.tolerance}
