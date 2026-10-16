"""Evaluation-only truth overlay/registration/composite; never returns navigation inputs."""
import textwrap

import numpy as np
from PIL import Image

from warehouse_route_evidence import path_chunks
from warehouse_truth_floorplan import METADATA, load_floorplan

FLOORPLAN = METADATA


def fit_registration(pairs):
    """All-sample SE(2) map→world fit, for plot labels only; no outlier removal."""
    result = dict(accepted=False, minimum_pairs=30, minimum_span_m=1.,
                  rms_limit_m=.05, maximum_residual_limit_m=.10, pair_count=0)
    try:
        unique = {}
        for pair in pairs:
            stamp = pair['stamp_ns']
            if stamp in unique and unique[stamp] != pair:
                raise ValueError('Conflicting synchronized samples')
            unique[stamp] = pair
        result['pair_count'] = len(unique)
        if len(unique) < result['minimum_pairs']:
            raise ValueError('Fewer than 30 distinct synchronized samples')
        source = np.array([p['map_xy'] for p in unique.values()], dtype=float)
        target = np.array([p['world_xy'] for p in unique.values()], dtype=float)
        if source.shape != (len(unique), 2) or target.shape != source.shape or not (
                np.isfinite(source).all() and np.isfinite(target).all()):
            raise ValueError('Invalid registration coordinates')
        result['map_span_m'] = float(np.linalg.norm(np.ptp(source, axis=0)))
        if result['map_span_m'] < result['minimum_span_m']:
            raise ValueError('Insufficient spatial extent')
        a, b = source.mean(axis=0), target.mean(axis=0)
        u, _, vt = np.linalg.svd((source-a).T @ (target-b))
        correction = np.diag([1., np.linalg.det(vt.T @ u.T)])
        rotation = vt.T @ correction @ u.T
        translation = b - rotation @ a
        errors = np.linalg.norm(source @ rotation.T + translation - target, axis=1)
        result.update(rms_residual_m=float(np.sqrt(np.mean(errors**2))),
                      maximum_residual_m=float(errors.max()))
        if result['rms_residual_m'] > .05 or result['maximum_residual_m'] > .10:
            raise ValueError('Rigid registration residual exceeds acceptance rule')
        result.update(accepted=True, rotation=rotation.tolist(), translation=translation.tolist(),
                      yaw_rad=float(np.arctan2(rotation[1, 0], rotation[0, 0])))
    except (KeyError, TypeError, ValueError, np.linalg.LinAlgError) as error:
        result['reason'] = str(error)
    return result


def draw_primary(destination, metadata, raster, paths, waypoints, title, unavailable=None):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig = Figure(figsize=(10, 9))
    FigureCanvasAgg(fig)
    ax = fig.subplots()
    if metadata is not None:
        ax.imshow(raster, extent=[metadata[k] for k in ('xmin', 'xmax', 'ymin', 'ymax')],
                  origin='upper', interpolation='nearest')
        for i, (name, chunks) in enumerate(paths):
            color = f'C{i % 10}'
            for j, chunk in enumerate(chunks):
                xy = np.asarray(chunk)
                ax.plot(xy[:, 0], xy[:, 1], '.-', color=color, linewidth=1.6,
                        markersize=2, label=f'{name} truth' if j == 0 else None)
            if chunks:
                ax.scatter(*chunks[0][0], marker='^', color=color, edgecolor='black', s=75,
                           label=f'{name} start', zorder=5)
                ax.scatter(*chunks[-1][-1], marker='X', color=color, edgecolor='black', s=75,
                           label=f'{name} end', zorder=5)
        for name, xy in waypoints.items():
            ax.scatter(*xy, facecolors='none', edgecolors='#bd1550', s=65, zorder=4)
            ax.annotate(name, xy, xytext=(5, 5), textcoords='offset points', color='#9c003b',
                        weight='bold', clip_on=True)
        # Include every actual sample: an out-of-map detour must not be clipped.
        points = [[metadata['xmin'], metadata['ymin']], [metadata['xmax'], metadata['ymax']]]
        points += list(waypoints.values())
        points += [xy for _, chunks in paths for chunk in chunks for xy in chunk]
        low, high = np.min(points, axis=0)-.3, np.max(points, axis=0)+.3
        ax.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), xlabel='world x (m)', ylabel='world y (m)')
        ax.set_aspect('equal')
        if paths: ax.legend(fontsize=7, loc='best')
        ax.grid(alpha=.15)
    else:
        ax.axis('off')
    if unavailable:
        ax.text(.5, .5, '\n'.join(textwrap.wrap('UNAVAILABLE: '+unavailable, 75)),
                transform=ax.transAxes, ha='center', va='center', backgroundcolor='white')
    wrapped = '\n'.join(textwrap.fill(line, width=100) for line in title.splitlines())
    fig.suptitle(wrapped, fontsize=11)
    foot = ('Evaluation only. All-height authored collision projection (including overhead objects).\n'
            'Truth gaps >0.5 s disconnected. Waypoints: '+('accepted SE(2) registration' if waypoints else 'unavailable; no guessed labels'))
    fig.text(.5, .01, foot, ha='center', fontsize=8)
    fig.tight_layout(rect=[0, .06, 1, max(.72, .95-.03*(len(wrapped.splitlines())-1))])
    fig.savefig(destination, dpi=150)
    fig.clear()


def render_primary_and_composite(run, output, route, report, index, diagnostic, label=''):
    # Local import avoids a cycle with the existing offline diagnostic renderer.
    from plot_warehouse_route import checked_json
    primary = dict(role='PRIMARY', status='unavailable', images=[], registration=fit_registration([]),
                   floorplan_source=None, capture_errors=[])
    metadata, raster, reason = None, None, None
    try:
        metadata, raster = load_floorplan(FLOORPLAN)
        if report['manifest'].get('warehouse_world_geometry_sha256') != metadata['world_geometry_sha256']:
            raise ValueError('Run warehouse geometry identity not authenticated against canonical floorplan')
        meshes = {p: sha for p, sha in metadata['source_sha256'].items()
                  if '/meshes/' in p and p.lower().endswith('.dae')}
        if report['manifest'].get('warehouse_collision_mesh_sha256') != meshes:
            raise ValueError('Run collision mesh bytes not authenticated against canonical floorplan')
        primary['floorplan_source'] = dict(metadata_path=str(FLOORPLAN),
            image_sha256=metadata['image_sha256'], geometry_sha256=metadata['geometry_sha256'],
            world_geometry_sha256=metadata['world_geometry_sha256'], frame_id='world',
            image_world_contract={k: metadata[k] for k in ('xmin', 'xmax', 'ymin', 'ymax', 'width', 'height', 'resolution')})
    except (OSError, ValueError, KeyError) as error:
        metadata, raster, reason = None, None, str(error)
    captured, pairs = [], []
    for segment in report['segments']:
        matches = [s for s in index['segments'] if s['sent_pose'] == segment.get('sent_pose')]
        if len(matches) != 1:
            continue
        saved = matches[0]
        try:
            if saved.get('truth_invalid_reason') or not saved.get('truth_path_file'):
                raise ValueError(saved.get('truth_invalid_reason') or 'No captured truth trajectory')
            path = checked_json(run, saved['truth_path_file'], saved['truth_path_sha256'])
            chunks = path_chunks(path, frame_id='world')
            if not chunks: raise ValueError('Empty truth trajectory')
            captured.append((segment, saved, chunks))
            if saved.get('registration_file'):
                pairs.extend(checked_json(run, saved['registration_file'], saved['registration_sha256']))
        except (OSError, ValueError, KeyError) as error:
            primary['capture_errors'].append(dict(segment_id=saved['segment_id'], error=str(error)))
    primary['registration'] = fit_registration(pairs)
    waypoints = {}
    if primary['registration']['accepted']:
        fit = primary['registration']
        waypoints = {g['name']: (np.asarray(g['position']) @ np.asarray(fit['rotation']).T + fit['translation']).tolist()
                     for g in route['goals']}
    if not captured:
        reason = reason or 'No valid same-run truth trajectory'
    if reason:
        primary['unavailable_reason'] = reason
    else:
        primary['status'] = 'generated'
    def row(segment_id, selected, unavailable=None):
        destination = output / f'world-truth-{segment_id}.png'
        title = f'PRIMARY: world truth floorplan + truth trajectory\n{label} | {route["route_id"]} | {segment_id} | HEAD {report["manifest"]["head"][:12]}'
        missing = [s.get('segment', s.get('name')) for s in report['segments']
                   if not any(s is v[0] for v in captured)]
        if segment_id == 'full-route' and missing:
            title += '\nINCOMPLETE CAPTURE — missing: '+', '.join(missing)
        draw_primary(destination, metadata, raster,
                     [(v[1]['segment_id'], v[2]) for v in selected] if not unavailable else [],
                     waypoints if metadata else {}, title, unavailable)
        lengths = [float(np.linalg.norm(np.diff(np.asarray(chunk), axis=0), axis=1).sum())
                   for v in selected for chunk in v[2]]
        item = dict(role='PRIMARY', route_id=route['route_id'], segment_id=segment_id,
            head=report['manifest']['head'], frame_id='world', image_path=str(destination),
            status='unavailable' if unavailable else 'generated', unavailable_reason=unavailable,
            floorplan_source=primary['floorplan_source'], waypoints=waypoints if metadata else {},
            map_waypoints={g['name']: g['position'] for g in route['goals']},
            included_segments=[v[1]['segment_id'] for v in selected], missing_segments=missing,
            path_length_m=sum(lengths) if lengths else None,
            endpoint_error_m=selected[0][0].get('endpoint_error_m') if len(selected) == 1 else None,
            endpoint_error_frame_id='map', path_length_note='Sum within captured world path chunks; gaps excluded')
        primary['images'].append(item)
        return destination
    left = row('full-route', captured, reason)
    if not reason:
        for segment in captured:
            row(segment[1]['segment_id'], [segment])
    right = diagnostic['images'][0]['image_path']
    if right is None:
        right = output / 'diagnostic-unavailable.png'
        draw_primary(right, None, None, [], {},
                     'DIAGNOSTIC: session map + SLAM trajectory',
                     diagnostic['images'][0].get('unavailable_reason', 'No diagnostic capture'))
    left_image, right_image = Image.open(left).convert('RGB'), Image.open(right).convert('RGB')
    height = max(left_image.height, right_image.height)
    left_y, right_y = (height-left_image.height)//2, (height-right_image.height)//2
    # Copy complete standalone panels without resizing/cropping; center shorter
    # panels vertically on white padding. Both standard panels are 1500×1350.
    composite = Image.new('RGB', (left_image.width+right_image.width, height), 'white')
    composite.paste(left_image, (0, left_y)); composite.paste(right_image, (left_image.width, right_y))
    destination = output / 'route-comparison.png'
    composite.save(destination)
    return primary, str(destination), dict(left='PRIMARY', right='DIAGNOSTIC',
        primary_rectangle=[0, left_y, left_image.width, left_y+left_image.height],
        diagnostic_rectangle=[left_image.width, right_y, left_image.width+right_image.width, right_y+right_image.height])
