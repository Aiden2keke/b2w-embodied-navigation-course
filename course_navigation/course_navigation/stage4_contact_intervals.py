"""Evaluation-only paired observer intervals; no engine coverage claim or control."""
import math
from .stage4_contact_evidence import ContactEvidence


class ContactIntervals:
    def __init__(self, expected_names, *, first_sequence, max_pending=16):
        if (type(first_sequence) is not int or first_sequence <= 0 or
                type(max_pending) is not int or not 1 <= max_pending <= 1024):
            raise ValueError('Explicit positive first sequence and bounded buffer required')
        self.gate = ContactEvidence(expected_names)
        self.expected = self.gate.expected
        self.next_sequence = first_sequence
        self.max_pending = max_pending
        self.pending = {}
        self.history = {}
        self.first_start = self.end = self.last_wall = None
        self.failure = None

    def fail(self, reason):
        self.failure = self.failure or reason

    def _put(self, sequence, kind, value, wall):
        if (type(sequence) is not int or sequence <= 0 or
                not isinstance(wall, (int, float)) or not math.isfinite(wall) or wall < 0):
            self.fail('invalid_contact_interval_metadata');return
        previous = self.history.get(sequence, {}).get(kind)
        if previous is not None:
            if previous != value:self.fail('conflicting_contact_duplicate')
            return  # Cannot renew freshness, including identical repeats.
        if sequence < self.next_sequence:
            self.fail('contact_sequence_regression');return
        if sequence-self.next_sequence >= self.max_pending or len(self.pending) >= self.max_pending and sequence not in self.pending:
            self.fail('contact_interval_buffer_overflow');return
        row = self.pending.setdefault(sequence, {})
        if kind in row:
            if row[kind][0] != value:self.fail('conflicting_contact_duplicate')
            return
        row[kind] = (value, wall)
        self._drain()

    def inventory(self, value, *, wall):
        try:
            sequence = value['sequence']
            start, end = value['interval_start'], value['stamp']
            if (type(sequence) is not int or sequence <= 0 or
                    any(type(v) not in (int, float) or not math.isfinite(v) for v in (start, end)) or
                    not (-1 <= start < end) or end < 0 or start < 0 and sequence != 1 or
                    type(value['sampled_steps']) is not int or value['sampled_steps'] <= 0 or
                    value['evaluation_only'] is not True or type(value['ever_forbidden_contact']) is not bool):
                raise ValueError()
            names = value['collisions']
            if len(names) != len(set(names)) or frozenset(names) != self.expected:
                self.fail('collision_inventory_mismatch');return
            if value['ever_forbidden_contact']:
                self.fail('forbidden_robot_environment_contact')
            # Freeze callback-owned mutable dictionaries before retaining them.
            clean = dict(sequence=sequence, stamp=end, interval_start=start,
                         sampled_steps=value['sampled_steps'], collisions=tuple(sorted(names)),
                         ever_forbidden_contact=value['ever_forbidden_contact'], evaluation_only=True)
            self._put(sequence, 'inventory', clean, wall)
        except (KeyError, TypeError, ValueError):
            self.fail('invalid_contact_interval_metadata')

    def contacts(self, sequence, pairs, *, stamp, wall):
        try:
            if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp < 0:
                raise ValueError()
            # A forbidden contact must veto immediately, even without its metadata.
            self.gate.check_pairs(pairs)
            if self.gate.failure:self.fail(self.gate.failure)
            clean = (stamp, tuple(tuple(pair) for pair in pairs))
            self._put(sequence, 'contacts', clean, wall)
        except (TypeError, ValueError):
            self.fail('invalid_contact_interval_metadata')

    def _drain(self):
        while not self.failure and self.next_sequence in self.pending:
            row = self.pending[self.next_sequence]
            if set(row) != {'inventory', 'contacts'}:return
            inv, iw = row['inventory'];(stamp, pairs), cw = row['contacts']
            if abs(stamp-inv['stamp']) > 1e-8:
                self.fail('contact_interval_stamp_mismatch');return
            if self.end is not None and abs(inv['interval_start']-self.end) > 1e-8:
                self.fail('contact_interval_gap_or_overlap');return
            wall = max(iw, cw)
            if self.last_wall is not None and wall < self.last_wall:
                self.fail('contact_clock_regression');return
            self.gate.inventory(inv['collisions'], stamp=stamp, wall=iw)
            self.gate.contacts(pairs, stamp=stamp, wall=cw)
            # contacts() already performed immediate veto; pairing cannot erase it.
            if self.gate.failure:self.fail(self.gate.failure);return
            if self.first_start is None:self.first_start = max(0., inv['interval_start'])
            self.end, self.last_wall = stamp, wall
            self.history[self.next_sequence] = {kind: item[0] for kind, item in row.items()}
            del self.pending[self.next_sequence]
            self.next_sequence += 1
            for old in list(self.history):
                if old < self.next_sequence-self.max_pending:del self.history[old]

    def closed(self, *, start, end, sim, wall):
        return bool(not self.failure and
                    all(type(v) in (int, float) and math.isfinite(v) for v in (start, end, sim, wall)) and
                    self.first_start is not None and self.first_start <= start <= end <= self.end and
                    self.last_wall is not None and 0 <= wall-self.last_wall <= .5 and
                    self.gate.complete(sim=sim, wall=wall))
