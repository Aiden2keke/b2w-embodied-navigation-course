"""Held-out behavior: accidental C fitting, partial acceptance and extrapolation fail."""
import hashlib
import json

import pytest

from scripts import fit_velocity_calibration as fitter
from scripts import velocity_calibration_core as core
from scripts.velocity_step_core import make_sequence

C_SPEEDS = (.055, .085, .115, .165)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def frozen_models():
    models = {}
    for kind, parameters in [('M0', {'k': 1.}), ('M1', {'k': 1.5}),
                              ('M2', {'a': 1., 'b': .06}), ('M3', {})]:
        models[kind] = {'kind': kind, 'valid': True, 'parameters': parameters,
            'knots': [[.04, .10], [.10, .16], [.20, .26]] if kind == 'M3' else [],
            'input_domain': {'zero_state': [0., 0.], 'moving': [.04, .20]},
            'output_domain': {'zero_state': [0., 0.], 'moving': [.10, .26]},
            'monotonic': True, 'extrapolation': 'forbidden'}
    return models


def rows(values=None):
    values = values or [.115, .145, .175, .225]
    return [{'u': u, 'requested_vx': u, 'v': v, 'valid_for_fit': True,
             'invalid_reasons': [], 'trial': i} for i, (u, v) in enumerate(zip(C_SPEEDS, values), 1)]


@pytest.fixture
def capture(tmp_path):
    """Native synthetic files only; these are not USER-RUN C observations."""
    b = tmp_path / 'B'; b.mkdir()
    hashes = {p: fitter.digest(fitter.ROOT / p) for p in fitter.AUTH_FILES}
    contract = {'source_hashes': hashes, 'policy_sha256': 'synthetic-policy',
                'fixture': {'generated_sha256': 'world', 'spawn': {'x': -4}}}
    bm = {**contract, 'owner_token_sha256': 'B-session'}
    (b / 'manifest.json').write_text(json.dumps(bm))
    (b / 'events.jsonl').write_text('synthetic B raw\n')
    fit = tmp_path / 'B-fit'; fit.mkdir()
    frozen = fit / 'calibration.json'
    frozen.write_text(json.dumps({'schema_version': 1, 'offline_only': True,
        'inputs': [{'directory': str(b), 'raw_sha256': sha(b/'events.jsonl'),
                    'file_hashes': {str(b/'manifest.json'): sha(b/'manifest.json'),
                                   str(b/'events.jsonl'): sha(b/'events.jsonl')}}],
        'truth': {'models': frozen_models(), 'production_selection': None}}))
    c = tmp_path / 'C'; c.mkdir()
    cm = {**contract, 'owner_token_sha256': 'C-session',
          'sequence': make_sequence(C_SPEEDS), 'experiment_start_sim_s': 0.,
          'termination': {'status': 'completed'}, 'cleanup': {'passed': True}}
    (c / 'manifest.json').write_text(json.dumps(cm))
    events = []
    for i in range(961):
        t = i * .05; block = min(int(t // 12), 3)
        moving = 3 <= t - block * 12 < 8
        u = C_SPEEDS[block] if moving else 0.
        v = u + .06 if moving else 0.
        for topic in ['/harness/requested', '/course_navigation/cmd_vel_raw',
                      '/course_navigation/cmd_vel', '/path_manager/path_manager_ros/nav_vel']:
            events.append({'topic': topic, 'sim_s': t, 'source_s': t, 'epoch': 0,
                'data': {'linear_x': u, 'linear_y': 0., 'angular_z': 0.}})
        events.append({'topic': '/network_input_debug', 'sim_s': t, 'epoch': 0,
                       'data': {'data': [0.]*6 + [u] + [0.]*50}})
        events.append({'topic': '/sim/ground_truth/odom', 'sim_s': t, 'source_s': t,
            'epoch': 0, 'child_frame_id': 'base_link',
            'data': {'linear': [v, 0., 0.], 'angular': [0., 0., 0.],
                     'position': [t*.1, 0., .6], 'quaternion_xyzw': [0., 0., 0., 1.]}})
    (c / 'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    return frozen, c


def validator():
    from scripts import validate_velocity_calibration
    return validate_velocity_calibration


def test_frozen_predictions_and_first_passing_complexity_order():
    result = validator().evaluate_models(frozen_models(), rows())
    assert result['models']['M0']['points'][0]['predicted_v'] == pytest.approx(.055)
    assert result['models']['M1']['points'][0]['predicted_v'] == pytest.approx(.0825)
    assert result['models']['M2']['points'][0]['predicted_v'] == pytest.approx(.115)
    assert result['models']['M3']['points'][0]['predicted_v'] == pytest.approx(.115)
    assert result['held_out_selection'] == 'M2'
    assert result['models']['M2']['passed'] and result['models']['M3']['passed']
    assert result['production_selection'] is None
    for kind in ('M0', 'M1', 'M2', 'M3'):
        model = result['models'][kind]
        assert model['all_predictions_supported']
        assert all(p['support_status'] == 'moving_supported' for p in model['points'])
    m0 = result['models']['M0']
    assert m0['metrics']['mae'] == pytest.approx(.06)
    assert m0['metrics']['rmse'] == pytest.approx(.06)
    assert m0['metrics']['max_abs_error'] == pytest.approx(.06)
    assert m0['points'][0]['residual'] == pytest.approx(.06)


@pytest.mark.parametrize('kind', ['M0', 'M1', 'M2', 'M3'])
@pytest.mark.parametrize('u,status', [(0., 'zero_state'), (.02, 'unsupported_zero_gap'),
    (-.01, 'outside_moving_support'), (.201, 'outside_moving_support'),
    (float('nan'), 'nonfinite_input')])
def test_explicit_zero_and_no_gap_bridge_or_extrapolation(kind, u, status):
    point = validator().predict_point(frozen_models()[kind], u)
    assert point['support_status'] == status
    assert point['predicted_v'] == (0. if u == 0 else None)


def test_c_truth_changes_errors_but_cannot_refit_any_frozen_model(capture, monkeypatch):
    frozen, c = capture
    def forbidden(*a, **k):
        pytest.fail('validation invoked a fitting/aggregation entrypoint')
    monkeypatch.setattr(core, 'fit_model', forbidden)
    monkeypatch.setattr(core, 'compare_models', forbidden)
    monkeypatch.setattr(fitter, 'compare_models', forbidden)
    monkeypatch.setattr(fitter, 'analyze_directories', forbidden)
    before = {p: p.read_bytes() for p in frozen.parent.parent.rglob('*') if p.is_file()}
    first = validator().validate(frozen, c)
    assert first['data_valid'] and first['held_out_selection'] == 'M2'
    assert first['frozen_calibration_sha256'] == sha(frozen)
    for p, data in before.items():
        assert p.read_bytes() == data
    raw = c / 'events.jsonl'
    events = [json.loads(line) for line in raw.read_text().splitlines()]
    for e in events:
        if e['topic'] == '/sim/ground_truth/odom':
            e['data']['linear'][0] += .02
    raw.write_text(''.join(json.dumps(e)+'\n' for e in events))
    second = validator().validate(frozen, c)
    for kind in frozen_models():
        assert [p['predicted_v'] for p in first['models'][kind]['points']] == [
            p['predicted_v'] for p in second['models'][kind]['points']]
    assert second['held_out_selection'] is None
    assert second['models']['M2']['metrics']['mae'] == pytest.approx(.02)
    assert frozen.read_bytes() == before[frozen]


@pytest.mark.parametrize('change', ['termination_null', 'aborted', 'cleanup', 'sequence',
    'same_session', 'same_raw', 'policy', 'source', 'fixture', 'B_manifest_changed'])
def test_invalid_or_nonindependent_capture_is_rejected(capture, change):
    frozen, c = capture
    path = c/'manifest.json'; m = json.loads(path.read_text())
    if change == 'termination_null': m['termination'] = None
    elif change == 'aborted': m['termination']['final_status'] = 'aborted'
    elif change == 'cleanup': m['cleanup'] = None
    elif change == 'sequence': m['sequence'][1]['end_s'] = 9.
    elif change == 'same_session': m['owner_token_sha256'] = 'B-session'
    elif change == 'same_raw': (c/'events.jsonl').write_text('synthetic B raw\n')
    elif change == 'policy': m['policy_sha256'] = 'different'
    elif change == 'source': m['source_hashes'][fitter.CPP] = 'different'
    elif change == 'fixture': m['fixture']['spawn']['x'] = -3
    elif change == 'B_manifest_changed': (frozen.parent.parent/'B/manifest.json').write_text('{}')
    path.write_text(json.dumps(m))
    with pytest.raises(ValueError): validator().validate(frozen, c)


def test_missing_truth_is_reported_invalid_without_subset_selection(capture):
    frozen, c = capture; raw = c/'events.jsonl'
    events = [json.loads(line) for line in raw.read_text().splitlines()]
    raw.write_text(''.join(json.dumps(e)+'\n' for e in events if not (
        e['topic'] == '/sim/ground_truth/odom' and 42 <= e['source_s'] <= 44)))
    result = validator().validate(frozen, c)
    assert not result['data_valid'] and result['held_out_selection'] is None
    assert len(result['observations']) == 4
    assert result['observations'][-1]['invalid_reasons']
    assert all(not m['passed'] for m in result['models'].values())


@pytest.mark.parametrize('offset,passes', [(.003, True), (.00301, False)])
def test_rmse_threshold_is_prospective_and_inclusive(offset, passes):
    result = validator().evaluate_models(frozen_models(), rows([v+offset for v in [.115,.145,.175,.225]]))
    assert result['models']['M2']['passed'] is passes


def test_max_error_gate_and_monotonicity_and_m3_backup():
    v = validator(); models = frozen_models()
    # RMSE .00255 < .003, but max .0051 > .005 must fail.
    result = v.evaluate_models(models, rows([.1201,.145,.175,.225]))
    assert result['models']['M2']['metrics']['rmse'] < .003
    assert not result['models']['M2']['passed']
    models['M2']['parameters']['b'] = .08
    assert v.evaluate_models(models, rows())['held_out_selection'] == 'M3'
    assert v.evaluate_models(models, rows(list(C_SPEEDS)))['held_out_selection'] == 'M0'
    assert v.evaluate_models(models, rows([.0825,.1275,.1725,.2475]))['held_out_selection'] == 'M1'
    # Never trust a stale monotonic=True flag over actual predictions/knots.
    models['M3']['knots'] = [[.04,.10],[.10,.30],[.20,.26]]
    result = v.evaluate_models(models, rows())
    assert not result['models']['M3']['predicted_monotonic']
    assert not result['models']['M3']['passed']
    result = v.evaluate_models(frozen_models(), rows([.115,.175,.145,.225]))
    assert not result['observed_monotonic'] and result['held_out_selection'] is None


def test_unsupported_or_invalid_point_blocks_whole_model():
    models = frozen_models(); models['M2']['input_domain']['moving'] = [.06,.20]
    result = validator().evaluate_models(models, rows())
    assert not result['models']['M2']['all_predictions_supported']
    assert result['models']['M2']['points'][0]['residual'] is None
    assert result['held_out_selection'] == 'M3'
    observations = rows(); observations[-1]['valid_for_fit'] = False
    assert validator().evaluate_models(frozen_models(), observations)['held_out_selection'] is None


def test_cli_outputs_only_to_fresh_external_directory(capture, tmp_path, monkeypatch, capsys):
    import builtins
    import subprocess
    v = validator(); frozen, c = capture
    real_import = builtins.__import__
    def guarded(name, *a, **k):
        if name.split('.')[0] in {'rclpy','launch','launch_ros','rosbag2_py'}:
            pytest.fail('offline validation imported ROS')
        return real_import(name, *a, **k)
    monkeypatch.setattr(builtins, '__import__', guarded)
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **k: pytest.fail('offline validation launched a process'))
    before = {p: sha(p) for p in tmp_path.rglob('*') if p.is_file()}
    out = tmp_path/'validation'
    assert v.main([str(frozen),str(c),'--output-dir',str(out)]) == 0
    report = json.loads((out/'validation.json').read_text())
    assert report['held_out_selection'] == 'M2' and report['production_selection'] is None
    assert report['inputs_unchanged'] is True
    for p, h in before.items(): assert sha(p) == h
    for bad in [out, c/'bad', frozen.parent/'bad', frozen.parent.parent/'B/bad']:
        assert v.main([str(frozen),str(c),'--output-dir',str(bad)]) == 1
    assert v.main([str(frozen),str(tmp_path/'nonexistent-C'),'--dry-run']) == 0
    assert not (tmp_path/'nonexistent-C').exists()


def test_inconsistent_pwl_domain_never_reports_supported_prediction():
    model = frozen_models()['M3']
    model['input_domain']['moving'] = [.01,.30]
    point = validator().predict_point(model,.02)
    assert point == {'predicted_v': None, 'support_status': 'unsupported_model_support'}


def test_m2_wins_even_when_m3_has_smaller_held_out_error():
    models = frozen_models()
    models['M3']['knots'] = [[.04,.101],[.10,.161],[.20,.261]]
    result = validator().evaluate_models(models, rows([.116,.146,.176,.226]))
    assert result['models']['M3']['metrics']['rmse'] < result['models']['M2']['metrics']['rmse']
    assert result['models']['M2']['passed'] and result['models']['M3']['passed']
    assert result['held_out_selection'] == 'M2'


@pytest.mark.parametrize('fault', ['frame', 'slope', 'policy', 'command', 'epoch', 'clock_rollback'])
def test_data_validity_faults_cannot_produce_model_selection(capture, fault):
    frozen, c = capture; path = c/'events.jsonl'
    events = [json.loads(line) for line in path.read_text().splitlines()]
    for e in events:
        if not 6 <= e['sim_s'] < 8: continue
        if fault == 'frame' and e['topic'] == '/sim/ground_truth/odom':
            e['child_frame_id'] = 'odom'
        elif fault == 'slope' and e['topic'] == '/sim/ground_truth/odom':
            e['data']['linear'][0] += .1*(e['source_s']-6)
        elif fault == 'policy' and e['topic'] == '/network_input_debug':
            e['data']['data'][7] = .01
        elif fault == 'command' and e['topic'] == '/path_manager/path_manager_ros/nav_vel':
            e['data']['linear_x'] += .01
        elif fault == 'epoch': e['epoch'] = 1
    if fault == 'clock_rollback':
        events.extend([{'topic': '/clock', 'kind': 'clock', 'sim_s': t, 'epoch': 0,
                        'data': {'clock_s': t}} for t in [1.,2.,1.]])
    path.write_text(''.join(json.dumps(e)+'\n' for e in events))
    result = validator().validate(frozen, c)
    assert not result['data_valid']
    assert result['held_out_selection'] is None


def test_cli_invalid_truth_exits_nonzero_with_failure_report(capture, tmp_path):
    frozen, c = capture; path = c/'events.jsonl'
    events = [json.loads(line) for line in path.read_text().splitlines()]
    path.write_text(''.join(json.dumps(e)+'\n' for e in events if e['topic'] != '/sim/ground_truth/odom'))
    output = tmp_path/'invalid-report'
    assert validator().main([str(frozen),str(c),'--output-dir',str(output)]) == 1
    report = json.loads((output/'validation.json').read_text())
    assert not report['data_valid'] and report['held_out_selection'] is None


def test_cli_expected_frozen_hash_rejects_prevalidation_model_edit(capture, tmp_path, capsys):
    frozen, c = capture; expected = sha(frozen)
    output = tmp_path/'hash-accepted'
    assert validator().main([str(frozen),str(c),'--expected-sha256',expected,
                             '--output-dir',str(output)]) == 0
    modified = json.loads(frozen.read_text())
    modified['truth']['models']['M2']['parameters']['b'] = .061
    frozen.write_text(json.dumps(modified))
    rejected = tmp_path/'hash-rejected'
    assert validator().main([str(frozen),str(c),'--expected-sha256',expected,
                             '--output-dir',str(rejected)]) == 1
    assert not rejected.exists()
    assert 'frozen calibration SHA256 mismatch' in capsys.readouterr().err
