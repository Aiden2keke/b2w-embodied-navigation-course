"""Evaluate TUM trajectories: timestamp association, SE(3) ATE and time-based RPE."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def load_trajectory(path):
    rows = np.loadtxt(path, comments='#', ndmin=2)
    if rows.shape[1] != 8 or len(rows) < 2 or not np.isfinite(rows).all():
        raise ValueError('Trajectory requires at least two finite TUM rows (t x y z qx qy qz qw)')
    if np.any(np.diff(rows[:, 0]) <= 0) or np.any(np.linalg.norm(rows[:, 4:], axis=1) < 1e-8):
        raise ValueError('Timestamps must be strictly increasing and quaternions valid')
    rows[:, 4:] /= np.linalg.norm(rows[:, 4:], axis=1)[:, None]
    return rows


def associate(estimate, truth, tolerance):
    if tolerance <= 0:
        raise ValueError('Association tolerance must be positive')
    pairs, used = [], set()
    for i, timestamp in enumerate(estimate[:, 0]):
        right = np.searchsorted(truth[:, 0], timestamp)
        candidates = [k for k in (right - 1, right) if 0 <= k < len(truth) and k not in used]
        if candidates:
            j = min(candidates, key=lambda k: abs(truth[k, 0] - timestamp))
            if abs(truth[j, 0] - timestamp) <= tolerance:
                pairs.append((i, j))
                used.add(j)
    if len(pairs) < 2:
        raise ValueError('Insufficient associated poses')
    e, g = np.asarray(pairs).T
    return estimate[e], truth[g]


def rigid_alignment(source, target):
    s, t = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - s).T @ (target - t))
    sign = np.eye(3)
    sign[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ sign @ u.T
    return rotation, t - rotation @ s


def evaluate(estimate, truth, tolerance=0.02, rpe_interval=1.0):
    if rpe_interval <= 0:
        raise ValueError('RPE interval must be positive')
    e, g = associate(estimate, truth, tolerance)
    rotation, translation = rigid_alignment(e[:, 1:4], g[:, 1:4])
    errors = (rotation @ e[:, 1:4].T).T + translation - g[:, 1:4]
    est_rotation, gt_rotation = Rotation.from_quat(e[:, 4:]), Rotation.from_quat(g[:, 4:])
    trans, angles = [], []
    for i, time in enumerate(e[:, 0]):
        target_time = time + rpe_interval
        j = np.searchsorted(e[:, 0], target_time)
        candidates = [k for k in (j - 1, j) if i < k < len(e)]
        if not candidates:
            continue
        j = min(candidates, key=lambda k: abs(e[k, 0] - target_time))
        if abs(e[j, 0] - target_time) > tolerance:
            continue
        ed = est_rotation[i].inv().apply(e[j, 1:4] - e[i, 1:4])
        gd = gt_rotation[i].inv().apply(g[j, 1:4] - g[i, 1:4])
        trans.append(np.linalg.norm(ed - gd))
        relative_e = est_rotation[i].inv() * est_rotation[j]
        relative_g = gt_rotation[i].inv() * gt_rotation[j]
        angles.append(np.degrees((relative_g.inv() * relative_e).magnitude()))
    rmse = lambda x: float(np.sqrt(np.mean(np.square(x))))
    return {'ATE_RMSE_m': rmse(np.linalg.norm(errors, axis=1)),
            'ATE_unaligned_RMSE_m': rmse(np.linalg.norm(e[:, 1:4] - g[:, 1:4], axis=1)),
            'RPE_trans_m': rmse(trans) if trans else None,
            'RPE_rot_deg': rmse(angles) if angles else None,
            'alignment': 'SE3_no_scale', 'matched_poses': len(e),
            'estimate_coverage': len(e) / len(estimate), 'rpe_pairs': len(trans),
            'rpe_interval_s': rpe_interval, 'timestamp_tolerance_s': tolerance,
            'rpe_missing_reason': None if trans else 'No pairs at the configured time interval'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('estimate')
    parser.add_argument('truth')
    parser.add_argument('--output', required=True)
    parser.add_argument('--tolerance', type=float, default=0.02)
    parser.add_argument('--rpe-interval', type=float, default=1.0)
    args = parser.parse_args()
    out = Path(args.output)
    if out.exists():
        parser.error('Output already exists; choose a new path')
    result = evaluate(load_trajectory(args.estimate), load_trajectory(args.truth), args.tolerance, args.rpe_interval)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(result, indent=2))
