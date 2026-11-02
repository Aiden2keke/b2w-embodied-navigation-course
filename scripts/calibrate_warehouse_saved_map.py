#!/usr/bin/env python3
"""Calibrate saved warehouse SLAM map to world using paired visual landmarks.

Click each landmark on the left world geometry, then the same landmark on the
right saved SLAM PGM. The final pair is held out for independent validation.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MAP_DIR = ROOT / 'experiments/g1_g5/waypoints/20261019-v1'
PGM = MAP_DIR / 'warehouse.pgm'
YAML = MAP_DIR / 'warehouse.yaml'
WORLD_MAP = ROOT / 'course_navigation/maps/warehouse_ground_truth_floorplan.png'
WORLD_META = ROOT / 'course_navigation/maps/warehouse_ground_truth_floorplan.json'
DEFAULT_OUTPUT = ROOT / 'course_navigation/maps/warehouse_saved_map_registration.json'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fit_se2(source, target, *, max_fit_error=0.15, max_holdout_error=0.15,
            min_span=2.0, scale_tolerance=0.02):
    """Fit source->target from all but final pair; final point is holdout."""
    a, b = np.asarray(source, float), np.asarray(target, float)
    if a.ndim != 2 or a.shape[1:] != (2,) or b.shape != a.shape or len(a) < 4:
        raise ValueError('Need at least 3 fit pairs and 1 independent holdout pair')
    if not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError('Landmark coordinates must be finite')
    train_a, train_b = a[:-1], b[:-1]
    if np.linalg.norm(np.ptp(train_a, axis=0)) < min_span or np.linalg.norm(np.ptp(train_b, axis=0)) < min_span:
        raise ValueError('Fit landmarks are not sufficiently separated')
    ac, bc = train_a.mean(0), train_b.mean(0)
    A, B = train_a-ac, train_b-bc
    u, singular, vt = np.linalg.svd(A.T @ B)
    raw = vt.T @ u.T
    if np.linalg.det(raw) < 0:
        raise ValueError('Landmark pairing implies a reflection')
    scale = float(singular.sum() / np.sum(A*A))
    if not math.isfinite(scale) or abs(scale-1) > scale_tolerance:
        raise ValueError(f'Landmark scale mismatch: fitted scale {scale:.6f}')
    rotation = raw
    translation = bc - rotation @ ac
    prediction = a @ rotation.T + translation
    residuals = np.linalg.norm(prediction-b, axis=1)
    fit_errors, holdout = residuals[:-1], residuals[-1]
    if float(fit_errors.max()) > max_fit_error:
        raise ValueError(f'Fit residual exceeds limit: max {fit_errors.max():.3f} m')
    if float(holdout) > max_holdout_error:
        raise ValueError(f'Independent holdout residual exceeds limit: {holdout:.3f} m')
    angle = math.atan2(rotation[1, 0], rotation[0, 0])
    return dict(kind='warehouse_saved_map_registration', version=1,
        transform=dict(source_frame='world', target_frame='saved_slam_map',
                       x_m=float(translation[0]), y_m=float(translation[1]), yaw_rad=angle),
        fitted_scale=scale, fit_pair_count=len(train_a), holdout_pair_count=1,
        fit_rmse_m=float(np.sqrt(np.mean(fit_errors**2))),
        fit_max_error_m=float(fit_errors.max()), holdout_error_m=float(holdout),
        residuals_m=[float(v) for v in residuals],
        source_landmarks_world=a.tolist(), target_landmarks_saved_map=b.tolist(),
        accepted=True)


def image_pixel_to_world(pixel, meta):
    col, row = pixel
    return [meta['xmin'] + (col + .5)*meta['resolution'],
            meta['ymax'] - (row + .5)*meta['resolution']]


def image_pixel_to_saved_map(pixel, width, height, resolution, origin):
    col, row = pixel
    return [origin[0] + (col + .5)*resolution,
            origin[1] + (height - row - .5)*resolution]


def open_calibrator(output=DEFAULT_OUTPUT, pair_count=5):
    import yaml
    from PIL import Image
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button
    world_meta = json.loads(WORLD_META.read_text())
    saved = yaml.safe_load(YAML.read_text())
    if (saved.get('image') != PGM.name or saved['mode'] != 'trinary'
            or len(saved['origin']) != 3 or saved['origin'][2] != 0):
        raise ValueError('Saved map YAML/image identity or planar conventions invalid')
    world_image = np.asarray(Image.open(WORLD_MAP).convert('RGB'))
    saved_image = np.asarray(Image.open(PGM).convert('L'))
    fig, axes = plt.subplots(1, 2, figsize=(15, 8))
    axes[0].imshow(world_image, origin='upper'); axes[0].set_title('World geometry (click landmarks)')
    axes[1].imshow(saved_image, origin='upper', cmap='gray', vmin=0, vmax=255); axes[1].set_title('Saved SLAM PGM (same landmarks)')
    for ax in axes: ax.axis('off')
    fig.suptitle(f'Pick {pair_count} pairs, alternating left then right; last pair is independent validation')
    world_pixels, saved_pixels, marks = [], [], [[], []]
    done = {'value': False}
    status = fig.text(.5, .02, 'Click a landmark on the left, then its match on the right.', ha='center')
    def click(event):
        if event.inaxes not in axes or event.xdata is None: return
        side = 0 if event.inaxes is axes[0] else 1
        expected = 0 if len(world_pixels) == len(saved_pixels) else 1
        if side != expected: status.set_text('Click on '+('left world map' if expected == 0 else 'right saved map')); fig.canvas.draw_idle(); return
        pixel=(float(event.xdata),float(event.ydata)); index=len(world_pixels)+1
        marks[side].append(event.inaxes.plot(*pixel,'o',ms=7,mfc='none',mec='#ff3b30')[0])
        event.inaxes.annotate(str(index),pixel,xytext=(5,5),textcoords='offset points',color='#ff3b30',weight='bold')
        if side == 0: world_pixels.append(pixel)
        else: saved_pixels.append(pixel)
        if len(saved_pixels) == pair_count: done['value']=True; plt.close(fig)
        else: status.set_text(f'Pair {len(saved_pixels)+1}/{pair_count}: click '+('saved-map match' if side == 0 else 'next world landmark')); fig.canvas.draw_idle()
    fig.canvas.mpl_connect('button_press_event', click)
    plt.show()
    if len(saved_pixels) != pair_count: raise RuntimeError('Calibration cancelled before all landmark pairs were selected')
    world_xy=[image_pixel_to_world(p,world_meta) for p in world_pixels]
    slam_xy=[image_pixel_to_saved_map(p,saved_image.shape[1],saved_image.shape[0],saved['resolution'],saved['origin']) for p in saved_pixels]
    result=fit_se2(world_xy,slam_xy)
    result.update(image_hashes=dict(world_geometry=sha256(WORLD_MAP),world_metadata=sha256(WORLD_META),
                                    saved_pgm=sha256(PGM),saved_yaml=sha256(YAML)),
        coordinate_conventions=dict(world_frame='world',world_pixel_contract=world_meta['pixel_contract'],
            saved_frame='saved_slam_map',saved_resolution_m_per_pixel=saved['resolution'],
            saved_origin=saved['origin'],saved_image_row0='highest map y; image origin upper'),
        validation=dict(method='visual paired landmarks; final pair excluded from SE2 fit',
                        fit_threshold_m=.15,holdout_threshold_m=.15,min_fit_span_m=2.0))
    result['landmarks']=[dict(index=i+1,world_pixel=list(world_pixels[i]),saved_map_pixel=list(saved_pixels[i]),
                             world_xy=world_xy[i],saved_map_xy=slam_xy[i],residual_m=result['residuals_m'][i])
                        for i in range(pair_count)]
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=DEFAULT_OUTPUT)
    parser.add_argument('--pairs',type=int,default=5)
    args=parser.parse_args()
    if args.pairs < 4: parser.error('--pairs must be >=4 (3 fit plus independent holdout)')
    open_calibrator(args.output,args.pairs)

if __name__ == '__main__': main()
