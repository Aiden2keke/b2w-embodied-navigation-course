"""Fresh reports must cover the complete accepted interval, including its tail."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
BODY = 'base_link::base_link_collision'


def gate():
    from stage4_route.contact_intervals import ContactIntervals
    return ContactIntervals([BODY], first_sequence=10)


def interval(e, seq, start, end, wall, *, pairs=None, forbidden=False):
    e.inventory(dict(sequence=seq, stamp=end, interval_start=start, sampled_steps=50,
                     collisions=[BODY], ever_forbidden_contact=forbidden, evaluation_only=True), wall=wall)
    e.contacts(seq, [] if pairs is None else pairs, stamp=end, wall=wall+.001)


def test_fresh_pre_end_report_does_not_close_unflushed_tail():
    e = gate();interval(e, 10, 9.95, 10., 20.)
    assert not e.closed(start=10., end=10.025, sim=10.03, wall=20.03)
    interval(e, 11, 10., 10.05, 20.05)
    assert e.closed(start=10., end=10.025, sim=10.06, wall=20.06)


def test_late_tail_contact_latches_failure_even_before_matching_inventory():
    e = gate();interval(e, 10, 9.95, 10., 20.)
    e.contacts(11, [['b2w::'+BODY, 'rack::collision']], stamp=10.05, wall=20.05)
    assert e.failure == 'forbidden_robot_environment_contact'
    assert not e.closed(start=10., end=10., sim=10.06, wall=20.06)


def test_missing_sequence_or_discontinuous_interval_cannot_close():
    e = gate();interval(e, 10, 9.95, 10., 20.)
    interval(e, 12, 10.05, 10.1, 20.1)
    assert not e.closed(start=10., end=10.1, sim=10.1, wall=20.11)
    interval(e, 11, 10.001, 10.05, 20.15)
    assert e.failure == 'contact_interval_gap_or_overlap'


def test_callback_order_is_tolerated_but_stamp_mismatch_and_sticky_veto_fail():
    e = gate()
    e.contacts(10, [], stamp=10., wall=20.)
    e.inventory(dict(sequence=10, stamp=10., interval_start=9.95, sampled_steps=50,
                     collisions=[BODY], ever_forbidden_contact=False, evaluation_only=True), wall=20.01)
    assert e.closed(start=9.96, end=10., sim=10.02, wall=20.02)
    e.contacts(11, [], stamp=10.06, wall=20.06)
    e.inventory(dict(sequence=11, stamp=10.05, interval_start=10., sampled_steps=50,
                     collisions=[BODY], ever_forbidden_contact=False, evaluation_only=True), wall=20.06)
    assert e.failure == 'contact_interval_stamp_mismatch'
    f = gate();interval(f, 10, 9.95, 10., 20., forbidden=True)
    assert not f.closed(start=9.96, end=10., sim=10., wall=20.02)


def test_duplicates_cannot_extend_freshness_and_slow_rtf_uses_wall_deadline():
    e = gate();interval(e, 10, 9.95, 10., 20.)
    interval(e, 10, 9.95, 10., 20.4)
    assert not e.closed(start=9.96, end=10., sim=10.1, wall=20.6)
    assert not e.closed(start=9.9, end=10., sim=10.1, wall=20.1)


def test_inventory_omission_and_conflicting_duplicate_are_sticky_failures():
    e = gate();interval(e, 10, 9.95, 10., 20.)
    e.contacts(10, [], stamp=10.001, wall=20.1)
    assert e.failure == 'conflicting_contact_duplicate'
    f = gate()
    f.inventory(dict(sequence=10, stamp=10., interval_start=9.95, sampled_steps=50,
                     collisions=[], ever_forbidden_contact=False, evaluation_only=True), wall=20.)
    assert f.failure == 'collision_inventory_mismatch'


def test_buffer_limit_cannot_be_nonfinite_or_unbounded():
    import pytest
    from stage4_route.contact_intervals import ContactIntervals
    for limit in [float('inf'), float('nan'), 0, -1, 1000000, True]:
        with pytest.raises(ValueError):ContactIntervals([BODY], first_sequence=10, max_pending=limit)
