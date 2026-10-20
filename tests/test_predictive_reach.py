"""Offline reachable-set veto: preserve filled body, unknowns and deadlines."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))


def grid():
    return dict(width=200, height=200, resolution=.025, origin=[-2.5, -2.5, 0.], data=[0]*40000)


def evaluate(g, *, reach=.1, yaw=.1, sim=10.1, wall=20.1, frame='map'):
    from stage4_route.predictive_reach import PreparedReachGrid
    prepared = PreparedReachGrid(g, frame=frame, epoch='one', source=10., receipt=20., encoding='costmap')
    return prepared.evaluate([0., 0., 0.], frame='map', epoch='one', sim=sim, wall=wall,
                             translation_reach=reach, yaw_reach=yaw, uncertainty=.02)


def test_clear_current_body_with_obstacle_in_stopping_reach_fails():
    g = grid();g['data'][100*200+134] = 254  # x=.85, beyond current .75 front
    assert not evaluate(g, reach=.3, yaw=0.)['clear']
    assert evaluate(grid())['clear']
    assert not evaluate(grid())['motion_ready']


def test_interior_unknown_offmap_rotation_and_bad_frame_fail():
    g = grid();g['data'][100*200+100] = 254
    assert not evaluate(g)['clear']
    g['data'][100*200+100] = 255
    assert not evaluate(g)['clear']
    assert not evaluate(grid(), reach=2.)['clear']
    g = grid();g['data'][124*200+124] = 254  # corner sweep at (.6,.6)
    assert not evaluate(g, reach=0., yaw=1.57)['clear']
    assert not evaluate(grid(), frame='odom')['clear']


def test_occupancy_unknown_and_unclassified_cells_are_blocked():
    from stage4_route.predictive_reach import PreparedReachGrid
    for value in (-1, 1, 64, 65, 100):
        g = grid();g['data'][100*200+100] = value
        p = PreparedReachGrid(g, frame='map', epoch='one', source=10., receipt=20., encoding='occupancy')
        assert not p.evaluate([0., 0., 0.], frame='map', epoch='one', sim=10., wall=20.,
                              translation_reach=0., yaw_reach=0., uncertainty=0.)['clear']


def test_expiry_source_receipt_epoch_and_invalid_reach_fail():
    assert not evaluate(grid(), sim=12.1)['clear']
    assert not evaluate(grid(), wall=22.1)['clear']
    assert not evaluate(grid(), reach=-1.)['clear']
    assert not evaluate(grid(), reach=float('nan'))['clear']
    from stage4_route.predictive_reach import PreparedReachGrid
    p = PreparedReachGrid(grid(), frame='map', epoch='one', source=10., receipt=20., encoding='costmap')
    assert not p.evaluate([0.,0.,0.], frame='map', epoch='other', sim=10., wall=20.,
                          translation_reach=0., yaw_reach=0., uncertainty=0.)['clear']


def test_prepared_grid_freezes_input_and_expired_compute_cannot_pass():
    from stage4_route.predictive_reach import PreparedReachGrid
    g = grid();g['data'][100*200+100] = 254
    p = PreparedReachGrid(g, frame='map', epoch='one', source=10., receipt=20., encoding='costmap')
    g['data'][100*200+100] = 0
    r = p.evaluate([0.,0.,0.], frame='map', epoch='one', sim=10., wall=20.,
                   translation_reach=0., yaw_reach=0., uncertainty=0.)
    assert not r['clear']
    p = PreparedReachGrid(grid(), frame='map', epoch='one', source=10., receipt=20., encoding='costmap')
    r = p.evaluate([0.,0.,0.], frame='map', epoch='one', sim=10., wall=20.,
                   translation_reach=0., yaw_reach=0., uncertainty=0., cycle_budget_s=1e-12)
    assert not r['clear'] and r['failure'] == 'reach_cycle_deadline'
