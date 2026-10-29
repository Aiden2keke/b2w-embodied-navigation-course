"""Passive capture never treats a moving command or missing cloud as readiness."""
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))


def test_passive_command_gate_checks_every_axis_and_finite_values():
    from stage4_route.stationary import check_zero_command
    check_zero_command([0.] * 6)
    for value in [.0001, float('nan'), float('inf')]:
        with pytest.raises(ValueError):
            check_zero_command([0., 0., value, 0., 0., 0.])


def test_three_distinct_structural_maps_and_30_sim_seconds_required():
    from stage4_route.stationary import capture_ready
    assert not capture_ready([10., 15., 15.], 50., first_observation_sim=10.)
    assert not capture_ready([10., 15., 39.], 39.9, first_observation_sim=10.)
    assert capture_ready([10., 15., 39.], 40., first_observation_sim=10.)


def test_clock_reset_never_counts_as_a_long_capture():
    from stage4_route.stationary import capture_ready
    with pytest.raises(ValueError):
        capture_ready([10., 15., 9.], 40., first_observation_sim=10.)


def test_old_or_queued_maps_cannot_substitute_for_observed_interval():
    from stage4_route.stationary import capture_ready, source_fresh
    assert not capture_ready([1., 2., 3.], 100., first_observation_sim=99.)
    assert not capture_ready([1., 2., 3.], 100., first_observation_sim=1.)
    assert not source_fresh(3., 100., max_age=7.5)
    assert not source_fresh(101., 100., max_age=7.5)
    assert source_fresh(99.9, 100., max_age=.5)
    assert not source_fresh(float('nan'), 100., max_age=.5)
