"""Establish an online map's ground datum from measured LiDAR returns."""
import numpy as np
from scipy.spatial.transform import Rotation


def ground_alignment(xyz):
    """Fit a supported, nearly horizontal floor below the initial LiDAR origin.

    Return None when evidence is insufficient. No robot height or simulator
    truth enters this estimate. The transform is fixed for the mapping session.
    """
    points = np.asarray(xyz, dtype=float)
    radius = np.linalg.norm(points[:, :2], axis=1)
    points = points[np.isfinite(points).all(axis=1) & (radius > 1.) & (radius < 10.)
                    & (points[:, 2] > -2.) & (points[:, 2] < -.2)]
    if len(points) < 250:
        return None
    rng = np.random.default_rng(0)
    if len(points) > 8000:
        points = points[rng.choice(len(points), 8000, replace=False)]
    design = np.column_stack([points[:, :2], np.ones(len(points))])
    seeds = np.flatnonzero(points[:, 2] <= np.percentile(points[:, 2], 30))
    best = None
    count = 0
    for _ in range(160):
        indices = rng.choice(seeds, 3, replace=False)
        if abs(np.linalg.det(design[indices])) < .5:
            continue
        plane = np.linalg.solve(design[indices], points[indices, 2])
        if np.linalg.norm(plane[:2]) > np.tan(np.radians(10)):
            continue
        mask = np.abs(design @ plane - points[:, 2]) < .025
        if np.count_nonzero(mask) > count:
            best, count = mask, np.count_nonzero(mask)
    if best is None or count < max(200, .12 * len(points)):
        return None
    for _ in range(3):
        plane = np.linalg.lstsq(design[best], points[best, 2], rcond=None)[0]
        best = np.abs(design @ plane - points[:, 2]) < .025
    if (np.linalg.matrix_rank(design[best]) < 3
            or np.any(np.ptp(points[best, :2], axis=0) < 1.)
            or np.linalg.norm(plane[:2]) > np.tan(np.radians(10))):
        return None
    normal = np.r_[-plane[:2], 1.]
    length = np.linalg.norm(normal)
    normal /= length
    axis = np.cross(normal, [0., 0., 1.])
    angle = np.arctan2(np.linalg.norm(axis), normal[2])
    rotation = (Rotation.from_rotvec(axis / np.linalg.norm(axis) * angle).as_matrix()
                if angle > 1e-10 else np.eye(3))
    alignment = np.eye(4)
    alignment[:3, :3] = rotation
    alignment[2, 3] = -plane[2] / length
    return alignment


def map_odom_correction(alignment, optimized_lidar, local_lidar):
    # Both public map and odom carry the ground datum; conjugation is necessary.
    return alignment @ optimized_lidar @ np.linalg.inv(alignment @ local_lidar)
