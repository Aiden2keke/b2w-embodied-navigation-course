"""Independent closed-cell and interval exclusion tests; no planner or motion."""
import math
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))


def grid():return dict(width=100,height=100,resolution=.05,origin=[-2.5,-2.5,0.],data=[0]*10000)


def test_internal_obstacle_closed_contact_unknown_and_offmap():
    from stage4_route.route_geometry import RectangleGrid
    g=grid();g['data'][50*100+58]=254
    assert not RectangleGrid(g).clear([0.,0.,0.])  # Occupied cell fully inside.
    g['data'][50*100+58]=255
    assert not RectangleGrid(g).clear([0.,0.,0.])
    g=grid();g['data'][50*100+65]=254  # Cell starts at x=.75: closed edge.
    assert not RectangleGrid(g).clear([0.,0.,0.])
    g['data'][50*100+65]=0
    assert RectangleGrid(g).clear([0.,0.,0.])
    assert not RectangleGrid(g).clear([2.4,0.,0.])


def test_rotated_map_origin_preserves_polygon_check():
    from stage4_route.route_geometry import RectangleGrid
    g=grid();g['origin']=[5.,7.,math.pi/2];g['data'][50*100+58]=254
    assert not RectangleGrid(g).clear([2.5,9.5,math.pi/2])
    assert RectangleGrid(g).clear([1.,9.5,math.pi/2])


def test_capsule_slice_contains_exact_perpendicular_band():
    from stage4_route.route_geometry import capsule_slice
    lo,hi=capsule_slice([0.,0.],[4.,1.],2.,.35)
    assert lo==pytest.approx(.5-.35*math.sqrt(17)/4)
    assert hi==pytest.approx(.5+.35*math.sqrt(17)/4)


def test_interval_proves_wall_but_does_not_invent_no_path_in_open_space():
    from stage4_route.route_geometry import RectangleGrid, exclude_slice
    g=grid()
    for y in range(100):g['data'][y*100+50]=254
    r=exclude_slice(RectangleGrid(g),2.5,[2.,3.],extra=.01)
    assert r['all_poses_blocked'] and r['unresolved_boxes']==0
    # Check the certificate covers the initial continuous y/yaw domain.
    area=sum((q['bounds'][1]-q['bounds'][0])*(q['bounds'][3]-q['bounds'][2]) for q in r['witnesses'])
    assert area==pytest.approx(math.pi)
    r=exclude_slice(RectangleGrid(grid()),2.5,[2.,3.],extra=.01)
    assert not r['all_poses_blocked'] and r['clear_example'] is not None


def test_interval_never_excludes_a_clear_corner_pose():
    from stage4_route.route_geometry import RectangleGrid
    g=grid();g['data'][50*100+65]=254;checker=RectangleGrid(g)
    # At yaw 0 and local x=2.5, closed contact blocks; translated y has space.
    assert checker.blocking_cell(2.5,[2.45,2.55,-.001,.001],extra=0.) is None


def test_sat_agrees_with_independent_polygon_checker_randomized():
    from stage4_route.route_geometry import RectangleGrid
    from stage4_route.spatial import footprint_clear
    g=grid();rng=np.random.default_rng(719)
    for i in rng.choice(10000,100,replace=False):g['data'][i]=254
    checker=RectangleGrid(g)
    for _ in range(100):
        p=[float(rng.uniform(-2,2)),float(rng.uniform(-2,2)),float(rng.uniform(-math.pi,math.pi))]
        assert checker.clear(p)==footprint_clear(g,p)
