"""Filled-body certificates must cover interior primitives and rotation sweeps."""
import math
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))


def grid():
    return dict(width=160, height=160, resolution=.025, origin=[-2., -2., 0.], data=[0]*25600)


def test_safe_translation_has_a_continuous_certificate():
    from stage4_route.continuous import certify_path
    r = certify_path([[-.5, 0., 0.], [.9, 0., 0.]], [grid()], extra=.02)
    assert r['continuous_clearance_proven']
    assert r['motion_ready'] is False
    assert r['checked_poses'] > 2


def test_clear_endpoints_do_not_hide_an_interior_obstacle():
    from stage4_route.continuous import certify_path
    g = grid();g['data'][80*160+80] = 254  # cell at x=0,y=0
    r = certify_path([[-1., 0., 0.], [1., 0., 0.]], [g])
    assert not r['continuous_clearance_proven']
    assert r['failure'] == 'filled_footprint_collision_or_unknown'


def test_rotation_arc_is_checked_even_when_both_endpoints_are_clear():
    from stage4_route.continuous import certify_path
    g = grid();g['data'][104*160+104] = 254  # (.6,.6), only intermediate orientation collides
    r = certify_path([[0., 0., 0.], [0., 0., math.pi/2]], [g])
    assert not r['continuous_clearance_proven']


def test_unknown_offmap_and_unresolved_bound_never_pass():
    from stage4_route.continuous import certify_path
    g = grid();g['data'][80*160+80] = 255
    assert not certify_path([[0.,0.,0.]], [g])['continuous_clearance_proven']
    assert not certify_path([[-1.9,0.,0.]], [grid()])['continuous_clearance_proven']
    assert not certify_path([[-.5,0.,0.],[.9,0.,0.]], [grid()], max_depth=0)['continuous_clearance_proven']


def test_search_budget_is_serializable_unresolved_evidence():
    from stage4_route.continuous import certify_path
    result = certify_path([[0.,0.,0.]], [grid()], cpu_limit=1e-12)
    assert not result['continuous_clearance_proven']
    json.dumps(result, allow_nan=False)
