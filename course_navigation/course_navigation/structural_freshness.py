"""Structural-cloud cadence; never substitutes for a fast sensor watchdog."""
import math


class StructuralMapCadence:
    source_budget = 7.5

    def __init__(self):
        self.last = None
        self.periods = []
        self.wall_budget = None
        self.clock = None
        self.failed = False

    def check_time(self, sim, wall):
        if (self.failed or not all(math.isfinite(t) and t >= 0 for t in (sim, wall)) or
                (self.clock is not None and any(a < b for a, b in zip((sim, wall), self.clock)))):
            self.failed = True
            raise ValueError('Structural-map clock invalid or regressed')
        self.clock = sim, wall

    def observe(self, stamp, sim, wall):
        self.check_time(sim, wall)
        if (not math.isfinite(stamp) or not 0 <= sim - stamp <= self.source_budget or
                (self.last is not None and not 0 <= stamp - self.last[0] <= self.source_budget)):
            self.failed = True
            raise ValueError('Invalid structural-map source stamp')
        if self.last is not None:
            if stamp == self.last[0]:
                return  # Repeated payload/stamp cannot renew the heartbeat.
            period = wall - self.last[1]
            if period <= 0 or (self.wall_budget is not None and period > self.wall_budget):
                self.failed = True
                raise ValueError('Structural-map missed wall heartbeat')
            if self.wall_budget is None:
                self.periods.append(period)
                if len(self.periods) == 2:
                    budget = max(7.5, 1.5 * max(self.periods))
                    if budget > 20.:
                        self.failed = True
                        raise ValueError('Structural-map cadence exceeds 20 wall-second ceiling')
                    self.wall_budget = budget
        self.last = stamp, wall

    def ages(self, sim, wall):
        self.check_time(sim, wall)
        if self.last is None or self.wall_budget is None:
            raise ValueError('Need three distinct structural-map publications before readiness')
        try:return structural_ages(self.last, self.wall_budget, sim=sim, wall=wall)
        except ValueError:
            self.failed=True
            raise


def structural_ages(last, wall_budget, *, sim, wall):
    """Shared age policy; caller owns monotonic-clock and startup qualification."""
    if last is None or wall_budget is None or not 7.5 <= wall_budget <= 20.:
        raise ValueError('Need three distinct structural-map publications before readiness')
    source_age, wall_age = sim-last[0], wall-last[1]
    if not (0 <= source_age <= StructuralMapCadence.source_budget and 0 <= wall_age <= wall_budget):
        raise ValueError('Structural-map heartbeat expired')
    return dict(slam_map_age=source_age, slam_map_wall_age=wall_age,
                slam_map_wall_budget=wall_budget)
