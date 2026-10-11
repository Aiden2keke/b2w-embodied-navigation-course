"""Rigid transform operations in ROS xyz/xyzw conventions."""
import numpy as np
from scipy.spatial.transform import Rotation


def pose_matrix(pose):
    q = np.array([getattr(pose.orientation, k) for k in 'xyzw'])
    xyz = np.array([getattr(pose.position, k) for k in 'xyz'])
    if not np.isfinite(np.r_[q, xyz]).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('Invalid pose/quaternion')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(q).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def assign_pose(pose, matrix):
    for axis, value in zip('xyz', matrix[:3, 3]):
        setattr(pose.position, axis, float(value))
    for axis, value in zip('xyzw', Rotation.from_matrix(matrix[:3, :3]).as_quat()):
        setattr(pose.orientation, axis, float(value))


def skew(x):
    a, b, c = x
    return np.array([[0., -c, b], [c, 0., -a], [-b, a, 0.]])


def covariance_at_base(covariance, world_rotation, lidar_to_base):
    cov = np.asarray(covariance).reshape(6, 6)
    jacobian = np.eye(6)
    jacobian[:3, 3:] = -skew(world_rotation @ np.asarray(lidar_to_base))
    return jacobian @ cov @ jacobian.T
