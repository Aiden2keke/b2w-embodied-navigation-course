"""Offline installed-plugin replay of an explicitly supplied frozen case. Never regenerate inputs."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import struct
import time
import uuid

import yaml

from narrow_fixture import chosen_route
from narrow_planner_comparison import PLUGIN_TYPES, path_metrics, save
from narrow_processes import OwnedProcesses

REPO = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_footprint(actual, common):
    # Humble makeFootprintFromString parses float coordinates; padding member is float.
    f32 = lambda value: struct.unpack('f', struct.pack('f', value))[0]
    padding = f32(common['footprint_padding'])
    expected = [[f32(v)+math.copysign(padding, v) if v else 0. for v in point]
                for point in json.loads(common['footprint'])]
    if actual != expected:
        raise ValueError('Padded footprint differs from Humble baseline representation')


def validate_inputs(source, expected_sha, expected_geometry):
    raw = (source/'costmap-before-planner-1.raw.u8').read_bytes()
    cm = json.loads((source/'costmap-before-planner-1.json').read_text())
    requests = [json.loads((source/f'{name}-request.json').read_text()) for name in PLUGIN_TYPES]
    digest = hashlib.sha256(raw).hexdigest()
    if digest != expected_sha:
        raise ValueError('Original raw SHA256 differs from recorded before hash')
    if bytes(cm['data']) != raw:
        raise ValueError('Snapshot JSON data differs from original raw bytes')
    m = cm['metadata']
    keys = ('size_x', 'size_y', 'resolution', 'origin')
    if {k: m[k] for k in keys} != {k: expected_geometry[k] for k in keys}:
        raise ValueError('Snapshot geometry differs from recorded geometry')
    if (any(type(m[k]) is not int or m[k] <= 0 for k in ('size_x', 'size_y'))
            or len(raw) != m['size_x']*m['size_y']
            or not math.isfinite(m['resolution']) or m['resolution'] <= 0
            or not all(math.isfinite(m['origin']['position'][k]) for k in ('x', 'y', 'z'))):
        raise ValueError('Invalid raw size or resolution')
    if m['origin']['orientation'] != dict(x=0., y=0., z=0., w=1.):
        raise ValueError('Rotated origin cannot be represented by Costmap2D')
    if [r['planner_id'] for r in requests] != list(PLUGIN_TYPES):
        raise ValueError('Unexpected planner IDs')
    if {k: v for k, v in requests[0].items() if k != 'planner_id'} != {
            k: v for k, v in requests[1].items() if k != 'planner_id'}:
        raise ValueError('Requests differ beyond planner_id')
    if requests[0]['use_start'] is not True:
        raise ValueError('Explicit use_start=true required')
    for label in ('start', 'goal'):
        validate_pose(requests[0][label])
        if requests[0][label]['header']['frame_id'] != cm['header']['frame_id']:
            raise ValueError('Pose frame differs from frozen costmap frame')
    return dict(raw_sha256=digest, geometry={k: m[k] for k in keys},
                frame=cm['header']['frame_id'], request=requests[0], costmap=cm)


def validate_pose(pose):
    stamp = pose['header']['stamp']
    if (type(stamp['sec']) is not int or not -(2**31) <= stamp['sec'] < 2**31
            or type(stamp['nanosec']) is not int or not 0 <= stamp['nanosec'] < 10**9):
        raise ValueError('Invalid PoseStamped stamp')
    for group, keys in [('position', 'xyz'), ('orientation', 'xyzw')]:
        if not all(math.isfinite(pose['pose'][group][k]) for k in keys):
            raise ValueError('Nonfinite PoseStamped field')
    if not any(pose['pose']['orientation'][k] for k in 'xyzw'):
        raise ValueError('Zero quaternion')
    if not pose['header']['frame_id']:
        raise ValueError('Empty pose frame')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--expected-sha', required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('/tmp/b2w-frozen-planner-replay'))
    parser.add_argument('--planner', choices=['both', *PLUGIN_TYPES], default='both')
    parser.add_argument('--cost-travel-multiplier', type=float,
                        help='Smac2D-only diagnostic override; all other runtime parameters must match source')
    args = parser.parse_args(argv)
    if len(args.expected_sha) != 64 or any(c not in '0123456789abcdef' for c in args.expected_sha):
        parser.error('--expected-sha must be a lowercase SHA256')
    if args.cost_travel_multiplier is not None:
        if args.planner != 'Smac2D' or not math.isfinite(args.cost_travel_multiplier) or args.cost_travel_multiplier < 0:
            parser.error('Multiplier override requires --planner Smac2D and a finite nonnegative value')
    return args


def load_fixture(source):
    fixture = json.loads((source/'fixture.json').read_text())
    for key in ('scenario_id', 'corridor_width_m', 'seed', 'goal_relative_start_xy'):
        if key not in fixture:
            raise ValueError('Missing fixture metadata: '+key)
    width = fixture['corridor_width_m']
    if not math.isfinite(width) or width <= 0:
        raise ValueError('Invalid fixture width')
    return fixture


def validate_parameters(actual, reference, planner_id, multiplier=None):
    expected = {k: v for k, v in reference.items() if k.startswith(planner_id+'.')}
    if not expected:
        raise ValueError('Missing source runtime plugin parameters')
    if multiplier is not None:
        expected['Smac2D.cost_travel_multiplier'] = multiplier
    if actual != expected:
        raise ValueError('Runtime plugin parameters differ: '+str({
            k: [expected.get(k), actual.get(k)] for k in expected.keys() | actual.keys()
            if expected.get(k) != actual.get(k)}))


def summarize_path(path, start, width, cm):
    with path.open() as f:
        points = [[float(row['x_map']), float(row['y_map'])] for row in csv.DictReader(f)]
    q = start['pose']['orientation']
    yaw = math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))
    ox, oy = start['pose']['position']['x'], start['pose']['position']['y']
    local = [[math.cos(yaw)*(x-ox)+math.sin(yaw)*(y-oy),
              -math.sin(yaw)*(x-ox)+math.cos(yaw)*(y-oy)] for x, y in points]
    return dict(path_metrics(points, cm), chosen_route=chosen_route(local, width))


def main():
    args = parse_args()
    source = args.source.resolve()
    expected_sha = args.expected_sha
    out = args.output.resolve()
    # Keep the saved independent geometry reference and recorded hash gate.
    previous = json.loads((source/'comparison.json').read_text())
    if previous['costmap_hashes']['before-planner-1'] != expected_sha:
        raise ValueError('Previous comparison hash disagrees with frozen protocol')
    reference = json.loads((source/'costmap-between-planners.json').read_text())['metadata']
    audit = validate_inputs(source, expected_sha, reference)
    fixture = load_fixture(source)
    out.mkdir(parents=True, exist_ok=True)
    config = REPO/'course_navigation/config/nav2_dual_planner_diag.yaml'
    settings = yaml.safe_load(config.read_text())
    runtime = json.loads((source/'runtime-parameters.json').read_text())
    cp = settings['global_costmap']['global_costmap']['ros__parameters']
    pp = settings['planner_server']['ros__parameters']
    for key in ('footprint', 'footprint_padding', 'track_unknown_space', 'global_frame'):
        if cp[key] != runtime['global_costmap/global_costmap'][key]:
            raise ValueError('Costmap contract changed: '+key)
    for key in ('allow_unknown', 'tolerance'):
        if pp['NavFn'][key] != pp['Smac2D'][key]:
            raise ValueError('Common planner input differs: '+key)
    if args.cost_travel_multiplier is not None:
        settings['planner_server']['ros__parameters']['Smac2D']['cost_travel_multiplier'] = args.cost_travel_multiplier
        config = out/'parameters-overlay.yaml'
        config.write_text(yaml.safe_dump(settings))
    common = dict(audit['geometry'], frame=audit['frame'], footprint=cp['footprint'],
        footprint_padding=cp['footprint_padding'], track_unknown_space=cp['track_unknown_space'])
    save(out/'input-audit.json', {k: v for k, v in audit.items() if k != 'costmap'} | dict(
        source=str(source), common=common, allow_unknown=pp['NavFn']['allow_unknown'],
        tolerance=pp['NavFn']['tolerance'], requests_equal_except_planner_id=True,
        json_matches_raw=True, fixture=fixture, expected_sha256=expected_sha,
        cost_travel_multiplier_override=args.cost_travel_multiplier))
    records = {}
    for planner_id, plugin_type in PLUGIN_TYPES.items():
        dst = out/planner_id
        if args.planner not in ('both', planner_id):
            continue
        dst.mkdir(exist_ok=False)
        request = json.loads((source/f'{planner_id}-request.json').read_text())
        save(dst/'input.json', dict(common=common, request=request))
        token = 'frozen-pair-'+uuid.uuid4().hex
        owned = OwnedProcesses(token)
        env = dict(os.environ, B2W_NARROW_OWNER=token, ROS_DOMAIN_ID='193',
            ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(dst/'ros-log'))
        argv = [str(args.binary.resolve()), str(dst/'input.json'),
                str(source/'costmap-before-planner-1.raw.u8'), str(config), str(dst/'replay'), plugin_type]
        source_before = sha(source/'costmap-before-planner-1.raw.u8')
        if source_before != expected_sha:
            raise ValueError('Source raw changed before launch')
        save(dst/'command.json', dict(argv=argv, owner_token=token, ros_domain=193,
                                     source_before_sha256=source_before))
        proc = None
        try:
            with (dst/'runtime.log').open('w') as log:
                proc = subprocess.Popen(argv, env=env, stdout=log, stderr=subprocess.STDOUT)
                identity = Path(f'/proc/{proc.pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
                owned.seen[proc.pid] = identity
                deadline = time.monotonic()+30
                while proc.poll() is None and time.monotonic() < deadline:
                    owned.observe()
                    time.sleep(.05)
        finally:
            if proc is not None:
                cleanup = owned.cleanup(proc)
                cleanup['returncode'] = proc.wait(timeout=5)
                save(dst/'cleanup.json', cleanup)
        if proc.returncode != 0 or not cleanup['passed']:
            raise RuntimeError(f'{planner_id} replay runtime/cleanup failed; inspect {dst}')
        result = json.loads((dst/'replay.result.json').read_text())
        if result['applied_request'] != request:
            raise ValueError('C++ applied PoseStamped differs from saved request')
        if result['applied_common'] != common:
            raise ValueError('C++ costmap contract differs from frozen input')
        params = result['plugin_parameters']
        validate_parameters(params, runtime['planner_server'], planner_id, args.cost_travel_multiplier)
        validate_footprint(result['footprint_padded_xy'], common)
        hashes = dict(source_before=source_before,
            source_after=sha(source/'costmap-before-planner-1.raw.u8'))
        for stage in ('loaded', 'before-plan', 'after-plan'):
            hashes[stage] = sha(dst/f'replay.costmap-{stage}.u8')
        if any(v != expected_sha for v in hashes.values()):
            raise ValueError('Source or replay raw master changed: '+planner_id)
        result.update(summarize_path(dst/'replay.path.csv', request['start'],
            fixture['corridor_width_m'], audit['costmap']))
        if result.get('unsmoothed_available'):
            result['unsmoothed'] = summarize_path(dst/'replay.unsmoothed.path.csv',
                request['start'], fixture['corridor_width_m'], audit['costmap'])
        if planner_id == 'NavFn':
            log = (dst/'runtime.log').read_text()
            import re
            internal_start = re.findall(r'\[NavFn\] Setting start to (\d+),(\d+)', log)
            internal_goal = re.findall(r'\[NavFn\] Setting goal to (\d+),(\d+)', log)
            if not internal_start or not internal_goal:
                raise ValueError('NavFn runtime round cell evidence missing')
            # NavFn reverses endpoints for potential propagation/extraction.
            result['navfn_runtime_round_start_cell'] = list(map(int, internal_goal[0]))
            result['navfn_runtime_round_goal_cell'] = list(map(int, internal_start[0]))
            result['navfn_cell_definition'] = 'Installed-library DEBUG; internal goal=robot start, internal start=world goal'
        result.update(raw_sha256=dict(source=audit['raw_sha256'], **hashes), fixture=fixture,
            exact_start=request['start'], exact_goal=request['goal'], cleanup=cleanup,
            runtime_parameters_match_source_except_authorized_override=True,
            actual_loaded_cost_travel_multiplier=params.get('Smac2D.cost_travel_multiplier'), action_status=None)
        save(dst/'summary.json', result)
        records[planner_id] = result
        print(planner_id, result['create_plan_success'], result['chosen_route'],
              result['returned_pose_count'], result['geometric_path_length_m'], flush=True)
    save(out/'comparison.json', dict(same_input_verified=True, fixture=fixture, planners=records,
        scope='Installed plugin createPlan only; separate fresh process and FrozenCostmap per plugin'))


if __name__ == '__main__':
    main()
