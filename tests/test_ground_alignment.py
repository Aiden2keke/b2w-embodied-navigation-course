import numpy as np
from scipy.spatial.transform import Rotation
from course_slam.ground_alignment import ground_alignment, map_odom_correction


def test_floor_with_walls_outliers_and_tilt_aligns_to_zero():
    rng = np.random.default_rng(41)
    xy = rng.uniform(-8, 8, (2500, 2))
    floor = np.column_stack([xy, .018 * xy[:, 0] - .012 * xy[:, 1] - .81])
    floor[:, 2] += rng.normal(0, .004, len(floor))
    walls = rng.uniform([-8, -8, -.8], [8, 8, 3], (6000, 3))
    cloud = np.vstack([floor, walls, [[np.nan, 0, 0], [0, 0, -20]]])
    alignment = ground_alignment(cloud)
    assert alignment is not None
    aligned = floor @ alignment[:3, :3].T + alignment[:3, 3]
    assert np.sqrt(np.mean(aligned[:, 2] ** 2)) < .008


def test_insufficient_or_vertical_points_do_not_invent_ground():
    assert ground_alignment(np.zeros((20, 3))) is None
    rng = np.random.default_rng(3)
    wall = np.column_stack([np.full(3000, 4.), rng.uniform(-5, 5, 3000), rng.uniform(-2, 3, 3000)])
    assert ground_alignment(wall) is None


def test_both_public_frames_and_map_share_alignment():
    alignment = np.eye(4)
    alignment[:3, :3] = Rotation.from_euler('xy', [.02, -.03]).as_matrix()
    alignment[2, 3] = .81
    local, optimized = np.eye(4), np.eye(4)
    local[:3, 3] = [1., 2., -.18]
    optimized[:3, :3] = Rotation.from_euler('z', .1).as_matrix()
    optimized[:3, 3] = [1.1, 2.2, -.17]
    correction = map_odom_correction(alignment, optimized, local)
    np.testing.assert_allclose(correction @ (alignment @ local), alignment @ optimized, atol=1e-12)
