"""Evaluation-only contact veto; absence is meaningful only with fresh coverage.

This gate cannot construct a navigation map, goal, transform or velocity.
Inventory must come from an independently pinned converted robot SDF.
"""
import math


GROUND = 'course_world::aws_robomaker_warehouse_GroundB_01_001::link::collision'
SUPPORTS = {f'{leg}_foot::{leg}_foot_collision' for leg in ('FL', 'FR', 'RL', 'RR')}


class ContactEvidence:
    def __init__(self, expected_names):
        self.expected = frozenset(expected_names)
        if not self.expected or len(self.expected) != len(expected_names):
            raise ValueError('Nonempty unique source collision inventory required')
        self.failure = None
        self.covered = frozenset()
        self.latest = {}

    def _update(self, key, stamp, wall):
        if not all(math.isfinite(v) and v >= 0 for v in (stamp, wall)):
            self.failure = self.failure or 'invalid_contact_clock'
            return False
        previous = self.latest.get(key)
        if previous and (stamp < previous[0] or wall < previous[1]):
            self.failure = self.failure or 'contact_clock_regression'
            return False
        if previous and stamp == previous[0]:
            return False  # replayed heartbeat cannot renew the wall deadline
        self.latest[key] = (stamp, wall)
        return True

    def inventory(self, names, *, stamp, wall):
        if self._update('inventory', stamp, wall):
            self.covered = frozenset(names)
            if len(names) != len(self.covered) or self.covered - self.expected:
                self.failure = self.failure or 'collision_inventory_mismatch'

    def _robot(self, name):
        for prefix in ('course_world::b2w::', 'b2w::'):
            if name.startswith(prefix):
                return name[len(prefix):]
        return None

    def contacts(self, pairs, *, stamp, wall):
        self._update('contacts', stamp, wall)
        self.check_pairs(pairs)

    def check_pairs(self, pairs):
        """Veto immediately, before paired metadata can establish freshness."""
        for pair in pairs:
            if len(pair) != 2 or any(not isinstance(n, str) or not n for n in pair):
                self.failure = self.failure or 'invalid_contact_pair'
                continue
            robots = [self._robot(n) for n in pair]
            if all(n is None for n in robots):
                self.failure = self.failure or 'unexpected_nonrobot_contact'
                continue
            allowed = any(robots[i] in self.expected & SUPPORTS and pair[1-i] == GROUND
                          for i in (0, 1))
            if not allowed:
                self.failure = self.failure or 'forbidden_robot_environment_contact'

    def complete(self, *, sim, wall):
        return (self.failure is None and self.covered == self.expected and
                all(math.isfinite(v) for v in (sim, wall)) and
                all(key in self.latest and 0 <= sim-self.latest[key][0] <= .5 and
                    0 <= wall-self.latest[key][1] <= .5 for key in ('inventory', 'contacts')))
