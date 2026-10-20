"""A necessary slice result must not certify connectivity or motion."""
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))


def test_clear_slice_is_not_a_connected_route_or_motion_permission():
    from stage4_route.route_geometry import RectangleGrid, exclude_slice, geometry_analysis
    g = dict(width=100, height=100, resolution=.05, origin=[0., 0., 0.], data=[0]*10000)
    result = exclude_slice(RectangleGrid(g), 2.5, [2., 3.], extra=0.)
    state = geometry_analysis(result)
    assert state['slice_status'] == 'CLEAR_POSE_FOUND'
    assert state['connected_path_status'] == 'UNRESOLVED'
    assert state['motion_ready'] is False
    assert state['next_stage'] == 'GEOMETRY_EVIDENCE'


def test_excluded_slice_still_allows_stationary_boundary_investigation():
    from stage4_route.route_geometry import RectangleGrid, exclude_slice, geometry_analysis
    data = [0]*10000
    for row in range(100):
        data[row*100+50] = 254
    g = dict(width=100, height=100, resolution=.05, origin=[0., 0., 0.], data=data)
    state = geometry_analysis(exclude_slice(RectangleGrid(g), 2.5, [2., 3.], extra=.0075))
    assert state['slice_status'] == 'EXCLUDED'
    assert state['next_stage'] == 'GEOMETRY_EVIDENCE'
    assert not state['motion_ready']


def test_unresolved_search_is_not_a_proof_of_nonexistence():
    from stage4_route.route_geometry import geometry_analysis
    state = geometry_analysis(dict(all_poses_blocked=False, clear_example=None, unresolved_boxes=1))
    assert state['slice_status'] == 'UNRESOLVED'
    assert not state['motion_ready']
