"""Small passive evidence helpers; standard map/Path dictionaries, no ROS initialization."""
import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from run_narrow_baseline import digest, write


def route_content_sha256(route):
    return hashlib.sha256(json.dumps(route, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def stamp_ns(header):
    stamp = header['stamp']
    sec, ns = stamp['sec'], stamp['nanosec']
    if (not isinstance(sec, int) or not isinstance(ns, int)
            or sec < 0 or not 0 <= ns < 1_000_000_000):
        raise ValueError('Invalid ROS timestamp')
    return sec * 1_000_000_000 + ns


def map_geometry(grid):
    """Exact ROS grid cells/origin, including planar origin yaw; no reconstruction."""
    if grid['header']['frame_id'] != 'map':
        raise ValueError('Map frame must be map; no implicit transform')
    info = grid['info']
    w, h, res = info['width'], info['height'], info['resolution']
    if (not isinstance(w, int) or not isinstance(h, int) or min(w, h) <= 0
            or not math.isfinite(res) or res <= 0):
        raise ValueError('Invalid grid dimensions/resolution')
    values = np.asarray(grid['data'])
    if (values.size != w * h or values.ndim != 1 or not np.isfinite(values).all()
            or not np.equal(values, np.floor(values)).all()
            or np.any(values < -1) or np.any(values > 100)):
        raise ValueError('Invalid OccupancyGrid data; expected -1..100')
    q = info['origin']['orientation']
    quat = [q[k] for k in 'xyzw']
    origin = info['origin']['position']
    if (not all(math.isfinite(v) for v in quat + [origin[k] for k in 'xyz'])
            or abs(sum(v*v for v in quat) - 1.) > 1e-5
            or abs(q['x']) > 1e-8 or abs(q['y']) > 1e-8):
        raise ValueError('Invalid/nonplanar grid origin quaternion')
    yaw = math.atan2(2*q['w']*q['z'], 1-2*q['z']**2)
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [[origin['x'] + c*x-s*y, origin['y'] + s*x+c*y]
               for x, y in [(0, 0), (w*res, 0), (w*res, h*res), (0, h*res)]]
    return dict(data=values.reshape(h, w), resolution=res,
                origin=(origin['x'], origin['y'], yaw), corners=corners)


def path_chunks(path, max_gap_s=.5, frame_id='map'):
    """Validate a map-frame nav_msgs/Path; never bridge missing-data intervals."""
    if path['header']['frame_id'] != frame_id:
        raise ValueError(f'Path frame must be {frame_id}')
    chunks, last = [], None
    for pose in path['poses']:
        if pose['header']['frame_id'] != frame_id:
            raise ValueError(f'Pose frame must be {frame_id}')
        t = stamp_ns(pose['header'])
        p, q = pose['pose']['position'], pose['pose']['orientation']
        values = [p[k] for k in 'xyz'] + [q[k] for k in 'xyzw']
        if not all(math.isfinite(v) for v in values) or abs(sum(q[k]**2 for k in 'xyzw')-1) > 1e-5:
            raise ValueError('Invalid localized pose')
        if last is not None and t <= last:
            raise ValueError('Nonmonotonic Path timestamps; cannot combine clock epochs')
        if last is None or (t-last) * 1e-9 > max_gap_s:
            chunks.append([])
        chunks[-1].append([p['x'], p['y']])
        last = t
    return chunks


class RouteEvidence:
    """At most one map and one localization Path per executed segment, plus index."""
    def __init__(self, run_directory, route):
        self.run = Path(run_directory)
        self.route = copy.deepcopy(route)
        self.current = None
        self.segments, self.errors = [], []
        self.last_clock_ns = None
        self.epoch_invalid = False

    def observe_clock(self, now_ns):
        reset = self.last_clock_ns is not None and now_ns < self.last_clock_ns
        self.last_clock_ns = now_ns
        if reset:
            self.epoch_invalid = True
            reason = 'Clock reset across run; map epochs cannot be composed'
            self.errors.append(reason)
            for segment in self.segments + ([self.current] if self.current else []):
                segment['invalid_reason'] = reason
                segment['truth_invalid_reason'] = reason
        return reset

    def begin(self, sent):
        if self.current is not None:
            raise ValueError('Previous evidence segment is still open')
        xy = [sent['pose']['position'][k] for k in 'xy']
        name = next((g['name'] for g in self.route['goals'] if g['position'] == xy), None)
        if name is None or sent['header']['frame_id'] != 'map':
            raise ValueError('Sent goal is not a canonical map waypoint')
        goal_index = [g['name'] for g in self.route['goals']].index(name)
        previous = self.route['goals'][goal_index-1]['name'] if goal_index else 'Start'
        self.current = dict(segment_id=f'{previous}-{name}', waypoint=name,
                            sent_pose=copy.deepcopy(sent), poses=[], invalid_reason=(
                                'Clock reset across run' if self.epoch_invalid else None),
                            truth_poses=[], registration_pairs=[], truth_frames=[],
                            truth_invalid_reason='Clock reset across run' if self.epoch_invalid else None)

    def sample_truth(self, odometry):
        """Evaluation-only alias sim_world→world; original WORLD XY unchanged."""
        if self.current is None or self.current['truth_invalid_reason']:
            return None
        source_frame = odometry['header']['frame_id']
        if source_frame not in ('sim_world', 'world') or odometry['child_frame_id'] != 'base_link':
            self.current['truth_invalid_reason'] = 'Unsupported truth source frame/child'
            return None
        pose = dict(header=copy.deepcopy(odometry['header']), pose=copy.deepcopy(odometry['pose']['pose']))
        pose['header']['frame_id'] = 'world'
        path_chunks({'header': {'frame_id': 'world'}, 'poses': [pose]}, frame_id='world')
        poses = self.current['truth_poses']
        if poses:
            delta = stamp_ns(pose['header']) - stamp_ns(poses[-1]['header'])
            if delta < 0:
                self.current['truth_invalid_reason'] = 'Truth clock reset'
                return None
            if delta < 100_000_000:
                return None
        poses.append(pose)
        if source_frame not in self.current['truth_frames']:
            self.current['truth_frames'].append(source_frame)
        return copy.deepcopy(pose)

    def add_registration_pair(self, truth_pose, map_pose):
        if self.current is None or truth_pose is None:
            return
        path_chunks({'header': {'frame_id': 'map'}, 'poses': [map_pose]})
        if stamp_ns(truth_pose['header']) != stamp_ns(map_pose['header']):
            raise ValueError('Registration requires exact synchronized TF/truth stamp')
        self.current['registration_pairs'].append(dict(stamp_ns=stamp_ns(truth_pose['header']),
            map_xy=[map_pose['pose']['position'][k] for k in 'xy'],
            world_xy=[truth_pose['pose']['position'][k] for k in 'xy']))

    def sample(self, pose):
        if self.current is None or self.current['invalid_reason']:
            return
        path_chunks({'header': {'frame_id': 'map'}, 'poses': [pose]})
        t = stamp_ns(pose['header'])
        poses = self.current['poses']
        if poses:
            delta = t - stamp_ns(poses[-1]['header'])
            if delta < 0:
                self.current['invalid_reason'] = 'Clock reset; map registration may have changed'
                return
            if delta < 100_000_000:  # Evidence only, <=10 Hz; not a control setting.
                return
        poses.append(copy.deepcopy(pose))

    def finish(self, grid, interrupted=False):
        if self.current is None:
            return
        # Never mutate active poses before IO succeeds; failed storage is an
        # unavailable segment, not a poisoned state for the next waypoint.
        current = copy.deepcopy(self.current)
        directory = self.run / 'evidence'
        poses = current.pop('poses')
        truth_poses = current.pop('truth_poses')
        pairs = current.pop('registration_pairs')
        source_frames = current.pop('truth_frames')
        path = dict(header=copy.deepcopy(poses[-1]['header']) if poses else
                    {'frame_id': 'map', 'stamp': {'sec': 0, 'nanosec': 0}}, poses=poses)
        filename = f'evidence/{len(self.segments):02d}-{current["segment_id"]}.path.json'
        current.update(path_file=None, path_sha256=None,
                       path_source='map→base_link TF (SLAM localization), not simulation truth',
                       map_source=None, interrupted=interrupted)
        try:
            directory.mkdir(parents=True, exist_ok=True)
            write(self.run / filename, path)
            current.update(path_file=filename, path_sha256=digest(self.run / filename))
            if grid is not None and not current['invalid_reason']:
                map_geometry(grid)
                # Preserve the original OccupancyGrid fields/data without raster synthesis.
                temporary = directory / 'map.tmp.json'
                write(temporary, grid)
                sha = digest(temporary)
                map_file = directory / f'map-{sha}.json'
                temporary.replace(map_file)
                current['map_source'] = dict(topic='/map', message_type='nav_msgs/msg/OccupancyGrid',
                    file=str(map_file.relative_to(self.run)), sha256=sha, header=grid['header'])
        except Exception as error:
            current['invalid_reason'] = repr(error)
            self.errors.append(repr(error))
        finally:
            current.update(truth_path_file=None, truth_path_sha256=None,
                           registration_file=None, registration_sha256=None,
                           truth_source=dict(topic='/sim/ground_truth/odom', source_frame_ids=source_frames,
                           evaluation_frame_id='world', contract='zero-offset simulator worldPose; evaluation-only frame alias'))
            try:
                if truth_poses:
                    truth_file = filename.replace('.path.json', '.truth.path.json')
                    write(self.run / truth_file, dict(header=truth_poses[-1]['header'], poses=truth_poses))
                    current.update(truth_path_file=truth_file, truth_path_sha256=digest(self.run / truth_file))
                if pairs:
                    pair_file = filename.replace('.path.json', '.registration.json')
                    write(self.run / pair_file, pairs)
                    current.update(registration_file=pair_file, registration_sha256=digest(self.run / pair_file))
            except Exception as error:
                current['truth_invalid_reason'] = repr(error)
                self.errors.append(repr(error))
            self.segments.append(current)
            self.current = None
        self.save()

    def save(self):
        directory = self.run / 'evidence'
        directory.mkdir(parents=True, exist_ok=True)
        temporary = directory / 'index.tmp.json'
        write(temporary, dict(route_id=self.route['route_id'], run_id=self.run.name,
                             route_content_sha256=route_content_sha256(self.route),
                             segments=self.segments, errors=self.errors))
        temporary.replace(directory / 'index.json')
