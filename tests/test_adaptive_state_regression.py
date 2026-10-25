"""Oct 18 reduced geometry cases plus synthetic entry/exit safety regressions.

The trace retains independent minima, not costmaps/paths. Reconstructed strips
exercise the real cell-area checker; they do not reproduce the historical run.
"""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from course_navigation.adaptive_speed import Config, CostGrid, Governor

FP = ((.75, .43), (.75, -.43), (-.75, -.43), (-.75, .43))
EVIDENCE = json.loads((Path(__file__).parent/'fixtures/oct18_adaptive_status.json').read_text())


def strip(left=1.57, right=1.57, restriction=None, unknown=False):
    cells = np.zeros((200, 200), dtype=np.uint8)
    centers = -5 + (np.arange(200)+.5)*.05
    walls = (centers >= .43+left) | (centers <= -.43-right)
    cells[walls, :] = 254
    if restriction is not None:
        columns = (centers >= restriction[0]) & (centers <= restriction[1])
        cells[np.ix_(np.abs(centers) >= .5, columns)] = 255 if unknown else 254
    return CostGrid(cells, .05, (-5., -5., 0.))


def update(g, t, grid, x=0., path=None, pose=None, speed=.18, healthy=True):
    return g.update(t, grid, path if path is not None else [(x, 0.), (x+2., 0.)],
                    pose or (x, 0., 0.), FP, speed, healthy)


@pytest.mark.parametrize('index', [0, 1, 2, 3])
def test_oct18_safe_start_strips_release_within_half_second(index):
    # Catches the old .35m bilateral release margin and 2s start-area latch.
    sample = EVIDENCE['samples'][index]
    m = strip(sample['left'], sample['right'])
    g = Governor(Config(w_ref=1.0))
    assert update(g, 0., m, speed=sample['measured_slam_speed']).mode == 'NARROW'
    assert update(g, .49, m).mode == 'NARROW'
    d = update(g, .51, m)
    assert d.mode == 'OPEN' and d.command_limit == .43
    assert d.release_eligible and d.clear_duration_s == pytest.approx(.51)


@pytest.mark.parametrize('key', ['start_sequence', 'previously_eligible_sequence'])
def test_recorded_safe_intervals_can_finish_confirmation(key):
    # Includes the actual source sample cadence, rather than assuming a 2s hold
    # can complete inside the retained ~1.1s old release-eligible interval.
    g = Governor(Config(w_ref=1.0))
    sequence = EVIDENCE[key]
    decisions = [update(g, sample['stamp_s'], strip(sample['left'], sample['right']),
                        speed=sample['measured_slam_speed'], healthy=sample['healthy'])
                 for sample in sequence]
    assert all(d.mode == 'NARROW' for sample, d in zip(sequence, decisions)
               if sample['stamp_s']-sequence[0]['stamp_s'] < .5)
    assert decisions[-1].mode == 'OPEN'


@pytest.mark.parametrize('index', [4, 5, 6])
def test_oct18_unsafe_side_or_overlap_still_forces_narrow(index):
    sample = EVIDENCE['samples'][index]
    g = Governor(Config(w_ref=1.0))
    m = strip(sample['left'], sample['right'])
    g.mode = 'OPEN'
    d = update(g, 0., m)
    assert d.mode == 'NARROW' and d.reason == 'geometry'
    assert d.tight and not d.release_eligible


@pytest.mark.parametrize('path,pose', [
    ([(0., 0.), (0., 2.)], (0., 0., 0.)),
    ([(0., 0.), (.4, 0.), (.4, 2.)], (0., 0., 0.)),
    ([(0., 0.), (2., 0.)], (0., 0., math.pi)),
])
def test_safe_turns_release_and_do_not_reenter_narrow(path, pose):
    # Catches both heading and path-curvature state triggers; RPP owns turning.
    g = Governor(Config(w_ref=1.0, release_hold_s=.1))
    m = strip()
    update(g, 0., m, path=path, pose=pose)
    assert update(g, .51, m, path=path, pose=pose).mode == 'OPEN'
    assert update(g, .61, m, path=path, pose=pose).mode == 'OPEN'


def test_full_rear_must_clear_before_release_timer_starts():
    g = Governor(Config(w_ref=1.0))
    m = strip(restriction=(-.5, .5))
    # Robot center/front are out, but rear is x=.35 inside the restriction.
    for t in (0., .6, 2.):
        d = update(g, t, m, x=1.1)
        assert d.mode == 'NARROW' and d.tight and d.clear_duration_s == 0.
    # Rear is now x=.55, outside the last occupied cell's x=.5 boundary.
    assert update(g, 2.1, m, x=1.3).reason == 'release_hold'
    assert update(g, 2.59, m, x=1.3).mode == 'NARROW'
    assert update(g, 2.61, m, x=1.3).mode == 'OPEN'


@pytest.mark.parametrize('unknown', [False, True])
def test_measured_speed_brakes_before_front_enters_upcoming_choke(unknown):
    m = strip(restriction=(1.85, 3.), unknown=unknown)
    g = Governor(Config(w_ref=1.0, release_hold_s=.1))
    update(g, 0., m, speed=.18)
    assert update(g, .51, m, speed=.18).mode == 'OPEN'
    # At .18, front x=.75 + 1m horizon remains clear. At .50, braking
    # horizon is 1.144m, reaching the restriction before the current front.
    d = update(g, .61, m, speed=.50)
    assert d.horizon == pytest.approx(1.144)
    assert d.mode == 'NARROW' and d.reason == 'geometry'
    assert d.command_limit < .11  # Existing downward-only overspeed guard.


def test_latency_and_conservative_deceleration_extend_anticipation():
    m = strip(restriction=(1.85, 3.))
    for cfg, expected in [
        (Config(w_ref=1.0, latency_s=.2, release_hold_s=.1), 'OPEN'),
        (Config(w_ref=1.0, release_hold_s=.1), 'NARROW'),
        (Config(w_ref=1.0, latency_s=.2, deceleration=.1, release_hold_s=.1), 'NARROW'),
    ]:
        g = Governor(cfg)
        update(g, 0., m, speed=.5)
        assert update(g, .51, m, speed=.5).mode == expected


def test_braking_horizon_cannot_shrink_and_release_before_passing_choke():
    g = Governor(Config(w_ref=1.0))
    m = strip(restriction=(1.85, 2.))
    update(g, 0., m)
    assert update(g, .51, m).mode == 'OPEN'
    assert update(g, .6, m, speed=.5).mode == 'NARROW'
    # Speed now falls to zero without advancing. Restriction is outside the
    # low-speed 1m horizon, but must remain in the latched checking distance.
    update(g, .7, m, speed=0.)
    d = update(g, 1.3, m, speed=0.)
    assert d.mode == 'NARROW' and d.reason == 'geometry'
    assert d.horizon >= 1.144
    assert update(g, 1.4, m, x=2.6).mode == 'NARROW'  # Rear still inside.
    assert update(g, 1.5, m, x=2.8).reason == 'release_hold'
    assert update(g, 2.01, m, x=2.8).mode == 'OPEN'
    assert update(g, 2.1, m, x=2.8).horizon == 1.0  # Entry memory released.


def test_braking_restriction_memory_survives_temporary_data_loss():
    g = Governor(Config(w_ref=1.0))
    m = strip(restriction=(1.85, 2.))
    update(g, 0., m, speed=.5)
    assert update(g, .1, m, healthy=False).mode == 'NARROW'
    update(g, .2, m, speed=0.)
    d = update(g, 1., m, speed=0.)
    assert d.mode == 'NARROW' and d.reason == 'geometry'


def test_short_path_cannot_hide_restriction_inside_retained_horizon():
    g = Governor(Config(w_ref=1.0))
    m = strip(restriction=(1.85, 2.))
    update(g, 0., m, speed=.5)
    # A truncated update cannot prove clearance where the choke was detected.
    short_path = [(0., 0.), (.3, 0.)]
    update(g, .1, m, path=short_path, speed=0.)
    d = update(g, 1., m, path=short_path, speed=0.)
    assert d.mode == 'NARROW' and not d.release_eligible
    assert d.reason == 'invalid_geometry'


def test_short_fresh_path_cannot_confirm_open_on_startup():
    g = Governor(Config(w_ref=1.0))
    m = strip()
    update(g, 0., m, path=[(0., 0.), (.3, 0.)])
    assert update(g, .51, m, path=[(0., 0.), (.3, 0.)]).mode == 'NARROW'
    # A complete valid path can still establish OPEN without restarting.
    update(g, .6, m)
    assert update(g, 1.11, m).mode == 'OPEN'


def test_modest_hysteresis_prevents_chatter_but_not_safe_release():
    g = Governor(Config(w_ref=1.0))
    safe, band, unsafe = strip(left=.22), strip(left=.17), strip(left=.12)
    update(g, 0., safe)
    assert update(g, .51, safe).mode == 'OPEN'
    assert update(g, .6, band).mode == 'OPEN'
    assert update(g, .7, unsafe).mode == 'NARROW'
    assert update(g, 2., band).reason == 'release_margin'
    assert update(g, 2.1, safe).reason == 'release_hold'
    assert update(g, 2.4, band).clear_duration_s == 0.
    assert update(g, 2.5, safe).mode == 'NARROW'
    assert update(g, 3.01, safe).mode == 'OPEN'


@pytest.mark.parametrize('bad', ['stale', 'path', 'pose', 'footprint', 'speed', 'rewind'])
def test_required_data_failure_resets_open_and_requires_confirmation(bad):
    g = Governor(Config(w_ref=1.0, release_hold_s=.1))
    m = strip()
    update(g, 0., m)
    assert update(g, .51, m).mode == 'OPEN'
    kwargs = dict(now=.6, grid=m, path=[(0., 0.), (2., 0.)],
                  pose=(0., 0., 0.), footprint=FP, speed=.18)
    kwargs.update({'stale': dict(healthy=False), 'path': dict(path=[]),
                   'pose': dict(pose=(math.nan, 0., 0.)), 'footprint': dict(footprint=[]),
                   'speed': dict(speed=None), 'rewind': dict(now=.1)}[bad])
    d = g.update(**kwargs)
    assert d.mode == 'NARROW' and not d.release_eligible
    assert update(g, 1., m).mode == 'NARROW'
    assert update(g, 1.51, m).mode == 'OPEN'
