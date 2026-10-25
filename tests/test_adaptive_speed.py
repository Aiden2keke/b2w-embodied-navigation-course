"""Offline geometry contracts. Fixtures use raw costs, metres and physical speed."""
import math

import numpy as np
import pytest

from course_navigation.adaptive_speed import Config, CostGrid, Governor, fresh

FOOTPRINT = ((.75, .43), (.75, -.43), (-.75, -.43), (-.75, .43))


def grid(width=4., narrowing_at=None, unknown=False):
    cells = np.zeros((120, 120), dtype=np.uint8)
    ys = -3 + (np.arange(120) + .5) * .05
    for col in range(120):
        x = -3 + (col + .5) * .05
        w = 1.5 if narrowing_at is not None and x >= narrowing_at else width
        cells[np.abs(ys) >= w / 2, col] = 255 if unknown else 254
    return CostGrid(cells, .05, (-3., -3., 0.))


def decide(gov, t=0., costmap=None, path=None, speed=.18, healthy=True, pose=(0., 0., 0.)):
    return gov.update(t, costmap or grid(), path if path is not None else [(0., 0.), (2., 0.)],
                      pose, FOOTPRINT, speed, healthy)


def test_narrow_width_is_obstacle_to_obstacle_and_clearance_is_from_footprint():
    g = Governor(Config(w_ref=1.5))
    d = decide(g, costmap=grid(width=1.5))
    assert d.mode == 'NARROW' and d.command_limit == .11
    assert d.width == pytest.approx(1.5)
    assert d.left == pytest.approx(.32) and d.right == pytest.approx(.32)


def test_open_requires_sustained_clearance_and_reentry_is_immediate():
    g = Governor(Config(w_ref=1.5, release_hold_s=1.))
    assert decide(g, 0.).mode == 'NARROW'
    assert decide(g, .9).mode == 'NARROW'
    assert decide(g, 1.).mode == 'OPEN'
    assert decide(g, 1.1, grid(width=1.5)).mode == 'NARROW'
    assert decide(g, 1.2).mode == 'NARROW'
    assert decide(g, 2.19).mode == 'NARROW'
    assert decide(g, 2.21).mode == 'OPEN'


def test_either_side_can_force_narrow_in_a_wide_aisle():
    g = Governor(Config(w_ref=1.5))
    d = decide(g, costmap=grid(width=3.), pose=(0., 1., 0.),
               path=[(0., 1.), (1., 1.)])
    assert d.mode == 'NARROW'
    assert d.left < .15 and d.width > 2.5


def test_braking_uses_physical_speed_and_increases_forward_horizon():
    cfg = Config(w_ref=1.5, lookahead_m=.3, deceleration=.2, latency_s=.4, braking_margin_m=.15)
    slow, fast = Governor(cfg), Governor(cfg)
    m = grid(narrowing_at=1.5)
    # Front of padded footprint is .75: narrowing lies beyond the slow horizon.
    assert decide(slow, costmap=m, speed=.18).reason == 'release_hold'
    d = decide(fast, costmap=m, speed=.6)
    assert d.mode == 'NARROW' and d.reason == 'geometry'
    assert d.horizon == pytest.approx(1.209)  # (.6²-.18²)/(.4) + .6*.4 + .15


def test_curvature_and_robot_alignment_leave_speed_regulation_to_rpp():
    g = Governor(Config(w_ref=1.5, release_hold_s=.1))
    path = [(0., 0.), (.4, 0.), (.4, 1.)]
    d = decide(g, path=path)
    assert d.reason == 'release_hold' and d.curvature > .8
    assert decide(g, .2, path=path).mode == 'OPEN'
    d = decide(g, .3, path=[(0., 0.), (0., 2.)])
    assert d.mode == 'OPEN' and d.heading_error == pytest.approx(math.pi/2)


@pytest.mark.parametrize('bad', [None, -1., float('nan'), float('inf')])
def test_invalid_speed_cannot_open(bad):
    g = Governor(Config(w_ref=1.5, release_hold_s=.1))
    decide(g, 0.)
    assert decide(g, 1., speed=bad).mode == 'NARROW'


def test_failures_reset_release_and_never_publish_zero_limit():
    g = Governor(Config(w_ref=1.5, release_hold_s=.1))
    decide(g, 0.)
    assert decide(g, .1).mode == 'OPEN'
    for t, kwargs in [(1., {'healthy': False}), (2., {'path': []}), (3., {'speed': math.nan})]:
        d = decide(g, t, **kwargs)
        assert d.mode == 'NARROW' and 0 < d.command_limit <= .11
    assert decide(g, 3.01).mode == 'NARROW'
    assert decide(g, 2.).mode == 'NARROW'  # Clock rewind resets hold.


def test_unknown_and_map_boundary_cannot_be_assumed_free():
    assert decide(Governor(Config(w_ref=1.5)), costmap=grid(width=1.5, unknown=True)).mode == 'NARROW'
    assert decide(Governor(Config(w_ref=1.5)), pose=(2.7, 0., 0.),
                  path=[(2.7, 0.), (3.5, 0.)]).mode == 'NARROW'


def test_width_hysteresis_band_does_not_release():
    g = Governor(Config(w_ref=1.5, width_margin_m=.1, release_margin_m=.2))
    for t in (0., 2., 4.):
        assert decide(g, t, costmap=grid(width=1.7)).mode == 'NARROW'


def test_release_diagnostic_distinguishes_blocked_margin_from_running_timer():
    # The modest band still blocks release, without reporting a running hold.
    g = Governor(Config(w_ref=1.0, release_hold_s=2.))
    for t in (0., 3.):
        d = decide(g, t, costmap=grid(width=1.2))
        assert d.reason == 'release_margin'
        assert d.release_eligible is False and d.clear_duration_s == 0.
        assert d.mode == 'NARROW' and d.command_limit == .11
    d = decide(g, 4.)
    assert d.reason == 'release_hold' and d.release_eligible is True
    assert d.clear_duration_s == 0.
    d = decide(g, 5.)
    assert d.clear_duration_s == pytest.approx(1.) and d.mode == 'NARROW'
    assert decide(g, 6.).mode == 'OPEN'


def test_heading_and_overlap_are_visible_even_when_curvature_is_zero():
    d = decide(Governor(Config(w_ref=1.0)), path=[(0., 0.), (0., 2.)])
    assert d.reason == 'release_hold' and d.curvature == 0.
    assert d.heading_error == pytest.approx(math.pi/2)
    assert d.tight is False and d.release_eligible is True
    d = decide(Governor(Config(w_ref=1.0)), costmap=grid(width=.8),
               path=[(0., 0.), (0., 2.)])
    assert d.reason == 'geometry' and d.tight is True
    assert d.heading_error == pytest.approx(math.pi/2)


def test_stale_diagnostic_cannot_reuse_clearance_or_release_duration():
    g = Governor(Config(w_ref=1.0, release_hold_s=.1))
    decide(g, 0.)
    assert decide(g, 1.).mode == 'OPEN'
    d = decide(g, 1.1, healthy=False)
    assert d.reason == 'stale_or_missing' and d.mode == 'NARROW'
    assert d.release_eligible is False and d.clear_duration_s == 0.
    assert d.heading_error is None and d.tight is None


def test_fresh_requires_both_source_and_receipt_time_and_rejects_future():
    assert fresh(10., 20., 9.8, 19.8, .5)
    for stamp, receipt in [(None, 20.), (8., 20.), (10., 18.), (11., 20.), (10., 21.)]:
        assert not fresh(10., 20., stamp, receipt, .5)


@pytest.mark.parametrize('kwargs', [{'w_ref': 0.}, {'w_ref': math.nan},
    {'w_ref': 1.5, 'narrow_command': 0.}, {'w_ref': 1.5, 'deceleration': 0.}])
def test_required_width_and_invalid_safety_parameters_rejected(kwargs):
    with pytest.raises(ValueError):
        Config(**kwargs)


def test_rotated_grid_has_identical_clearances():
    original = grid(width=1.5)
    rotated = CostGrid(original.cells, .05, (3., -3., math.pi/2))
    d = decide(Governor(Config(w_ref=1.5)), costmap=rotated,
               path=[(0., 0.), (0., 1.)], pose=(0., 0., math.pi/2))
    assert d.width == pytest.approx(1.5)


def test_measured_overspeed_only_reduces_command_limit():
    g = Governor(Config(w_ref=1.5, release_hold_s=.1))
    decide(g, 0.)
    d = decide(g, 1., speed=.55)
    assert 0 < d.command_limit < .43


def test_nearest_path_vertex_behind_robot_does_not_invent_a_uturn():
    g = Governor(Config(w_ref=1.5, release_hold_s=1.))
    path = [(x, 0.) for x in np.arange(0., 2.1, .05)]
    assert decide(g, 0., path=path, pose=(.01, 0., 0.)).reason == 'release_hold'
    assert decide(g, 1., path=path, pose=(.01, 0., 0.)).mode == 'OPEN'


def test_projection_keeps_a_real_upcoming_turn():
    g = Governor(Config(w_ref=1.5))
    d = decide(g, path=[(0., 0.), (.4, 0.), (.4, 1.)], pose=(.01, 0., 0.))
    assert d.reason == 'release_hold' and d.curvature > .8
