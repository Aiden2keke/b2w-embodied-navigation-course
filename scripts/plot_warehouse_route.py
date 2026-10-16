#!/usr/bin/python3
"""Render saved warehouse route evidence offline; never starts ROS or sends goals."""
import argparse
import json
import os
from pathlib import Path
import textwrap

import numpy as np

from run_narrow_baseline import digest, write
from warehouse_route_evidence import map_geometry, path_chunks, route_content_sha256


def checked_json(run, filename, sha):
    path = (run / filename).resolve()
    if not path.is_relative_to(run) or not path.is_file() or digest(path) != sha:
        raise ValueError(f'SHA/path mismatch for same-run artifact: {filename}')
    return json.loads(path.read_text())


def draw_overlay(destination, grid, paths, waypoints, title, segment_id):
    # Imported only for offline plotting, after transport teardown in the harness.
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.colors import ListedColormap, BoundaryNorm
    from matplotlib.transforms import Affine2D
    from matplotlib.patches import Patch

    geom = map_geometry(grid)
    fig = Figure(figsize=(14, 7) if segment_id != 'full-route' else (10, 9))
    FigureCanvasAgg(fig)
    axes = fig.subplots(1, 2) if segment_id != 'full-route' else [fig.subplots()]
    colors = [(v, v, v, 1.) for v in np.linspace(1., .05, 101)] + [(.68, .72, .76, 1.)]
    cmap = ListedColormap(colors)
    raster = np.where(geom['data'] == -1, 101, geom['data'])
    h, w = raster.shape
    ox, oy, yaw = geom['origin']
    all_xy = list(geom['corners']) + list(waypoints.values())
    zoom_xy = []
    for _, chunks in paths:
        for chunk in chunks:
            all_xy.extend(chunk)
            zoom_xy.extend(chunk)
    for name in segment_id.split('-'):
        if name in waypoints:
            zoom_xy.append(waypoints[name])
    for i, ax in enumerate(axes):
        transform = Affine2D().rotate(yaw).translate(ox, oy) + ax.transData
        ax.imshow(raster, origin='lower', interpolation='nearest', cmap=cmap,
                  norm=BoundaryNorm(np.arange(-.5, 102.5), cmap.N),
                  extent=[0, w*geom['resolution'], 0, h*geom['resolution']],
                  transform=transform, zorder=0)
        for j, (name, chunks) in enumerate(paths):
            color = f'C{j % 10}'
            for k, chunk in enumerate(chunks):
                xy = np.asarray(chunk)
                ax.plot(xy[:, 0], xy[:, 1], '.-', color=color, linewidth=1.6,
                        markersize=2, label=f'{name} actual' if k == 0 else None)
            if chunks:
                ax.scatter(*chunks[0][0], marker='^', s=75, color=color, edgecolor='black',
                           label=f'{name} start', zorder=5)
                ax.scatter(*chunks[-1][-1], marker='X', s=75, color=color, edgecolor='black',
                           label=f'{name} end', zorder=5)
        for name, xy in waypoints.items():
            ax.scatter(*xy, marker='o', s=65, facecolors='none', edgecolors='#bd1550', zorder=4)
            ax.annotate(name, xy, xytext=(5, 5), textcoords='offset points',
                        color='#9c003b', weight='bold', clip_on=True)
        bounds = np.asarray(zoom_xy if i == 1 and zoom_xy else all_xy)
        low, high = bounds.min(axis=0)-1., bounds.max(axis=0)+1.
        ax.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]),
               xlabel='map x (m)', ylabel='map y (m)',
               title='Segment detail (all captured detours included)' if i == 1 else 'Map context')
        ax.set_aspect('equal')
        ax.grid(alpha=.15)
        handles, labels = ax.get_legend_handles_labels()
        handles += [Patch(facecolor=colors[100], label='Occupied'),
                    Patch(facecolor=colors[-1], label='Unknown')]
        ax.legend(handles=handles, fontsize=7, loc='best')
    title = '\n'.join(textwrap.fill(line, width=100 if segment_id == 'full-route' else 140)
                      for line in title.splitlines())
    fig.suptitle(title, fontsize=11)
    fig.text(.5, .01, 'Localization trajectory; gaps >0.5 s are disconnected. '
             'Hollow circles: nominal waypoints. Visual detour judgment is manual.', ha='center', fontsize=8)
    fig.tight_layout(rect=[0, .04, 1, max(.75, .95-.03*(len(title.splitlines())-1))])
    fig.savefig(destination, dpi=150)
    fig.clear()


def render_run(run_directory, output_directory=None, label=''):
    run = Path(run_directory).resolve()
    route = json.loads((run / 'route-snapshot.json').read_text())
    report = json.loads((run / 'report.json').read_text())
    index_file = run / 'evidence/index.json'
    index = json.loads(index_file.read_text()) if index_file.exists() else dict(
        route_id=route['route_id'], run_id=run.name, segments=[], errors=['No captured evidence'],
        route_content_sha256=route_content_sha256(route))
    if index['run_id'] != run.name or index['route_id'] != route['route_id'] or route['frame_id'] != 'map':
        raise ValueError('Evidence run/route/frame mismatch; no cross-startup alignment')
    if index.get('route_content_sha256') != route_content_sha256(route):
        raise ValueError('Captured route content SHA mismatch')
    waypoints = {g['name']: g['position'] for g in route['goals']}
    loaded, diagnostic_errors = [], []
    for segment in report['segments']:
        if 'sent_pose' not in segment:
            continue
        xy = [segment['sent_pose']['pose']['position'][k] for k in 'xy']
        if (segment['sent_pose']['header']['frame_id'] != 'map'
                or segment['name'] not in waypoints or xy != waypoints[segment['name']]):
            raise ValueError('Sent waypoint does not match captured route snapshot')
        matches = [s for s in index['segments'] if s['sent_pose'] == segment['sent_pose']]
        if len(matches) > 1:
            raise ValueError('Ambiguous evidence segment association')
        saved = matches[0] if matches else None
        path, grid = None, None
        try:
            if saved and saved['path_file']:
                path = checked_json(run, saved['path_file'], saved['path_sha256'])
                path_chunks(path)
                if saved['map_source']:
                    source = saved['map_source']
                    grid = checked_json(run, source['file'], source['sha256'])
                    map_geometry(grid)
        except (OSError, ValueError, KeyError, TypeError) as error:
            diagnostic_errors.append(dict(segment_id=saved['segment_id'], error=str(error)))
            saved = dict(saved, invalid_reason=str(error))
            path, grid = None, None
        loaded.append((segment, saved, path, grid))
    output = Path(output_directory).resolve() if output_directory else run / 'overlays'
    output.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault('MPLCONFIGDIR', str(output / 'mpl-cache'))
    executed = [s[0]['name'] for s in loaded]
    result = dict(route_id=route['route_id'], head=report['manifest']['head'], label=label,
                  executed_waypoints=executed,
                  unexecuted_waypoints=[name for name in waypoints if name not in executed],
                  detour_judgment=None,
                  primary_rule='Human composite review, PRIMARY world truth panel: no large loop/major bypass; intended corridor traversed.',
                  map_limit='Exact same-run /map; unknown is not free, map is not ground truth. '
                            'Cross-startup physical map reproducibility remains unverified.',
                  capture_errors=index.get('errors', []) + diagnostic_errors, images=[])
    def row(segment_id, source, length=None, error=None):
        return dict(role='DIAGNOSTIC', route_id=route['route_id'], segment_id=segment_id, head=result['head'],
                    map_source=source, image_path=None, waypoints=waypoints,
                    path_length_m=length, endpoint_error_m=error, status='unavailable')
    valid = [s for s in loaded if s[1] and not s[1].get('invalid_reason') and s[2] and s[2]['poses'] and s[3] is not None]
    # A full-route image always means the executed prefix; do not connect separate segments.
    full = row('full-route', valid[-1][1]['map_source'] if valid else None)
    result['images'].append(full)
    if valid:
        destination = output / 'full-route.png'
        full.update(image_path=str(destination), status='generated',
                    included_segments=[v[1]['segment_id'] for v in valid],
                    missing_segments=[s[0]['name'] for s in loaded if s not in valid])
        title = 'DIAGNOSTIC: session map + SLAM trajectory\n'
        title += f'{label} | {route["route_id"]} | Full route — executed prefix '
        title += f'({len(executed)}/{len(waypoints)} waypoints); HEAD {result["head"][:12]}'
        if full['missing_segments']:
            title += '\nINCOMPLETE CAPTURE — missing: ' + ', '.join(
                (s[1] or {}).get('segment_id', s[0]['segment']) for s in loaded if s not in valid)
        draw_overlay(destination, valid[-1][3],
                     [(v[1]['segment_id'], path_chunks(v[2])) for v in valid], waypoints, title, 'full-route')
    else:
        full['unavailable_reason'] = 'No matching map + localized trajectory; no geometry invented'
    for segment, saved, path, grid in loaded:
        segment_id = saved['segment_id'] if saved else segment['segment'].replace('→', '-')
        item = row(segment_id, saved['map_source'] if saved else None,
                   segment.get('goal_manager_metrics', {}).get('path_length_m'), segment.get('endpoint_error_m'))
        result['images'].append(item)
        if saved and not saved.get('invalid_reason') and path and path['poses'] and grid is not None:
            destination = output / f'{segment_id}.png'
            item.update(image_path=str(destination), status='generated', interrupted=saved['interrupted'])
            draw_overlay(destination, grid, [(segment_id, path_chunks(path))], waypoints,
                         'DIAGNOSTIC: session map + SLAM trajectory\n'
                         f'{label} | {route["route_id"]} | {segment_id} | HEAD {result["head"][:12]}', segment_id)
        else:
            item['unavailable_reason'] = (saved or {}).get('invalid_reason') or 'Missing map or localized trajectory'
    from warehouse_truth_overlay import render_primary_and_composite
    result['primary'], result['composite_image_path'], result['composite_layout'] = render_primary_and_composite(
        run, output, route, report, index, result, label)
    write(output / 'trajectory-overlays.json', result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    parser.add_argument('--output-directory', type=Path, help='New directory; never overwrites previous images')
    parser.add_argument('--label', default='', help='Method/run label; use MOCK for synthetic inputs')
    args = parser.parse_args(argv)
    result = render_run(args.run_directory, args.output_directory, args.label)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
