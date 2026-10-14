"""Reject replacement or unequal frozen inputs before loading a ROS plugin."""
import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))


def inputs(tmp_path):
    raw = bytes([0, 10, 253, 255])
    cm = dict(header=dict(frame_id='map'), metadata=dict(size_x=2, size_y=2,
        resolution=.05, origin=dict(position=dict(x=-1., y=-2., z=0.),
        orientation=dict(x=0., y=0., z=0., w=1.))), data=list(raw))
    pose = dict(header=dict(frame_id='map', stamp=dict(sec=21, nanosec=800000000)),
        pose=dict(position=dict(x=-.95, y=-1.95, z=.625),
                  orientation=dict(x=.001, y=.002, z=.003, w=.999)))
    request = dict(start=pose, goal=copy.deepcopy(pose), use_start=True, planner_id='NavFn')
    for name, value in [('costmap-before-planner-1.json', cm), ('NavFn-request.json', request),
                        ('Smac2D-request.json', dict(request, planner_id='Smac2D'))]:
        (tmp_path/name).write_text(json.dumps(value))
    (tmp_path/'costmap-before-planner-1.raw.u8').write_bytes(raw)
    return hashlib.sha256(raw).hexdigest(), cm, request


def test_full_pose_survives_validation(tmp_path):
    from frozen_planner_replay import validate_inputs
    sha, cm, request = inputs(tmp_path)
    actual = validate_inputs(tmp_path, sha, cm['metadata'])
    assert actual['request'] == request
    assert actual['request']['start']['pose']['position']['z'] == .625
    assert actual['request']['start']['pose']['orientation']['x'] == .001
    assert actual['raw_sha256'] == sha


def test_humble_float_footprint_is_exact_and_changed_padding_is_rejected():
    from frozen_planner_replay import validate_footprint
    common = dict(footprint='[[0.70, 0.38], [0.70, -0.38], [-0.70, -0.38], [-0.70, 0.38]]',
                  footprint_padding=.05)
    actual = [[.7499999888241291, .4299999959766865],
              [.7499999888241291, -.4299999959766865],
              [-.7499999888241291, -.4299999959766865],
              [-.7499999888241291, .4299999959766865]]
    validate_footprint(actual, common)
    with pytest.raises(ValueError):
        validate_footprint(actual, dict(common, footprint_padding=.06))
    with pytest.raises(ValueError):
        validate_footprint(actual[:-1], common)


@pytest.mark.parametrize('change', ['missing_raw', 'hash', 'json_bytes', 'geometry',
                                    'start', 'goal', 'use_start', 'frame'])
def test_invalid_input_stops_before_replay(tmp_path, change):
    from frozen_planner_replay import validate_inputs
    sha, cm, request = inputs(tmp_path)
    geometry = copy.deepcopy(cm['metadata'])
    if change == 'missing_raw': (tmp_path/'costmap-before-planner-1.raw.u8').unlink()
    elif change == 'hash': sha = '0'*64
    elif change == 'json_bytes':
        cm['data'][0] = 1
        (tmp_path/'costmap-before-planner-1.json').write_text(json.dumps(cm))
    elif change == 'geometry': geometry['origin']['position']['x'] += .1
    else:
        request['planner_id'] = 'Smac2D'
        if change in ('start', 'goal'): request[change]['pose']['position']['x'] += .01
        elif change == 'use_start': request['use_start'] = False
        elif change == 'frame':
            # A shared invalid frame must also fail even when both requests match.
            request['start']['header']['frame_id'] = 'odom'
            original = dict(request, planner_id='NavFn')
            (tmp_path/'NavFn-request.json').write_text(json.dumps(original))
        (tmp_path/'Smac2D-request.json').write_text(json.dumps(request))
    with pytest.raises((ValueError, FileNotFoundError)):
        validate_inputs(tmp_path, sha, geometry)


def test_cli_requires_explicit_source_and_sha():
    from frozen_planner_replay import parse_args
    with pytest.raises(SystemExit):
        parse_args(['--binary', '/tmp/replay'])
    with pytest.raises(SystemExit):
        parse_args(['--source', '/tmp/case', '--binary', '/tmp/replay'])
    args = parse_args(['--source', '/tmp/other-case', '--expected-sha', 'a'*64,
                       '--binary', '/tmp/replay', '--output', '/tmp/output'])
    assert args.source == Path('/tmp/other-case')
    assert args.expected_sha == 'a'*64


def test_fixture_is_metadata_not_single_case_allowlist(tmp_path):
    from frozen_planner_replay import load_fixture
    fixture = dict(scenario_id='B', corridor_width_m=1.2, seed=7,
                   goal_relative_start_xy=[3, 1])
    (tmp_path/'fixture.json').write_text(json.dumps(fixture))
    assert load_fixture(tmp_path) == fixture


@pytest.mark.parametrize('change', ['nan_resolution', 'negative_size', 'nan_origin',
                                   'missing_pose_field', 'nan_pose', 'stamp', 'zero_quaternion'])
def test_malformed_geometry_and_full_pose_rejected(tmp_path, change):
    from frozen_planner_replay import validate_inputs
    sha, cm, request = inputs(tmp_path)
    if change == 'nan_resolution': cm['metadata']['resolution'] = float('nan')
    elif change == 'negative_size': cm['metadata']['size_x'] = -2
    elif change == 'nan_origin': cm['metadata']['origin']['position']['x'] = float('nan')
    else:
        if change == 'missing_pose_field': del request['goal']['pose']['orientation']['x']
        elif change == 'nan_pose': request['start']['pose']['position']['z'] = float('nan')
        elif change == 'stamp': request['start']['header']['stamp']['nanosec'] = 1000000000
        elif change == 'zero_quaternion': request['goal']['pose']['orientation'] = dict(x=0,y=0,z=0,w=0)
        for planner in ('NavFn', 'Smac2D'):
            (tmp_path/f'{planner}-request.json').write_text(json.dumps(dict(request, planner_id=planner)))
    (tmp_path/'costmap-before-planner-1.json').write_text(json.dumps(cm))
    with pytest.raises((ValueError, KeyError)):
        validate_inputs(tmp_path, sha, cm['metadata'])


def test_runtime_parameters_only_allow_requested_multiplier():
    from frozen_planner_replay import validate_parameters
    reference = {'Smac2D.cost_travel_multiplier': 1., 'Smac2D.smoother.w_data': .2}
    actual = dict(reference, **{'Smac2D.cost_travel_multiplier': 4.})
    validate_parameters(actual, reference, 'Smac2D', 4.)
    with pytest.raises(ValueError): validate_parameters(actual, reference, 'Smac2D')
    with pytest.raises(ValueError): validate_parameters(actual, reference, 'Smac2D', 2.)
    actual['Smac2D.smoother.w_data'] = .3
    with pytest.raises(ValueError): validate_parameters(actual, reference, 'Smac2D', 4.)


def test_postprocessing_regression_of_saved_pair_without_plugin_calls():
    from frozen_planner_replay import summarize_path
    root = Path('/tmp/b2w-frozen-planner-replay')
    if not (root/'comparison.json').exists(): pytest.skip('local frozen regression evidence unavailable')
    audit = json.loads((root/'input-audit.json').read_text())
    cm = json.loads((Path(audit['source'])/'costmap-before-planner-1.json').read_text())
    for planner in ('NavFn', 'Smac2D'):
        old = json.loads((root/planner/'summary.json').read_text())
        actual = summarize_path(root/planner/'replay.path.csv', old['exact_start'], 1.7, cm)
        for key, value in actual.items(): assert value == old[key], key


@pytest.mark.parametrize('override', ['-1', 'nan', 'inf'])
def test_invalid_multiplier_rejected_at_cli(override):
    from frozen_planner_replay import parse_args
    with pytest.raises(SystemExit):
        parse_args(['--source', '/tmp/case', '--expected-sha', 'a'*64, '--binary', '/tmp/bin',
                    '--planner', 'Smac2D', '--cost-travel-multiplier', override])


def test_pair_regression_uses_recorded_plugin_outputs_not_new_processes(tmp_path, monkeypatch):
    """Exercise the refactored full harness on original A fixture with saved plugin responses."""
    import os
    import shutil
    import frozen_planner_replay as replay
    old_root = Path('/tmp/b2w-frozen-planner-replay')
    if not (old_root/'input-audit.json').exists(): pytest.skip('local frozen evidence unavailable')
    audit = json.loads((old_root/'input-audit.json').read_text())
    calls = []
    class RecordedProcess:
        pid = os.getpid()
        returncode = 0
        def __init__(self, argv, **kwargs):
            dst = Path(argv[4]).parent
            planner = json.loads(Path(argv[1]).read_text())['request']['planner_id']
            calls.append(planner)
            for path in (old_root/planner).glob('replay.*'):
                if path.is_file(): shutil.copyfile(path, dst/path.name)
            kwargs['stdout'].write((old_root/planner/'runtime.log').read_text())
        def poll(self): return 0
        def wait(self, **kwargs): return 0
    class NoProcesses:
        def __init__(self, token): self.seen = {}
        def cleanup(self, proc): return dict(passed=True, remaining={}, returncode=0)
    monkeypatch.setattr(replay.subprocess, 'Popen', RecordedProcess)
    monkeypatch.setattr(replay, 'OwnedProcesses', NoProcesses)
    monkeypatch.setattr(sys, 'argv', ['replay', '--source', audit['source'], '--expected-sha',
        audit['raw_sha256'], '--binary', '/unused/recorded-responses', '--output', str(tmp_path)])
    replay.main()
    assert calls == ['NavFn', 'Smac2D']
    compared = {}
    for planner in calls:
        old = json.loads((old_root/planner/'summary.json').read_text())
        new = json.loads((tmp_path/planner/'summary.json').read_text())
        keys = old.keys() - {'cleanup', 'runtime_parameters_match_previous', 'raw_sha256'}
        for key in keys: assert new[key] == old[key], (planner, key)
        for stage, digest in old['raw_sha256'].items(): assert new['raw_sha256'][stage] == digest
        assert new['runtime_parameters_match_source_except_authorized_override'] is True
        compared[planner] = sorted(keys)
    assert new['fixture']['scenario_id'] == 'A'
    assert len(compared['Smac2D']) > 25


def test_source_hash_changed_since_audit_stops_before_process(tmp_path, monkeypatch):
    import frozen_planner_replay as replay
    old_root = Path('/tmp/b2w-frozen-planner-replay')
    if not (old_root/'input-audit.json').exists(): pytest.skip('local frozen evidence unavailable')
    audit = json.loads((old_root/'input-audit.json').read_text())
    monkeypatch.setattr(sys, 'argv', ['replay', '--source', audit['source'], '--expected-sha',
        audit['raw_sha256'], '--binary', '/unused', '--output', str(tmp_path)])
    monkeypatch.setattr(replay, 'sha', lambda path: 'b'*64)
    monkeypatch.setattr(replay.subprocess, 'Popen', lambda *a, **kw: pytest.fail('Plugin must not start'))
    with pytest.raises(ValueError, match='Source raw changed before launch'):
        replay.main()
