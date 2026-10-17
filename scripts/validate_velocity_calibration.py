#!/usr/bin/env python3
"""OFFLINE frozen Protocol-B models vs one fresh native Protocol-C run.

Never fits, imports ROS, launches processes, or changes input/production files.
Retain B runs: their hash-pinned manifests in calibration.json authenticate the
independent session and unchanged policy/control/geometry/fixture contract.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# Extraction only. Deliberately do not use the fitter's aggregation entrypoint.
from scripts.fit_velocity_calibration import digest, extract_observations
from scripts.velocity_calibration_core import errors, predict
from scripts.velocity_step_core import make_sequence

C_SPEEDS = (.055, .085, .115, .165)
MODEL_ORDER = ('M0', 'M1', 'M2', 'M3')
# Newly preregistered prospective C gates, not historical calibration constants.
MAX_ABS_ERROR_MPS = .005
RMSE_MPS = .003


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def predict_point(model, u):
    """Use frozen forward support; explicit zero is never the affine intercept."""
    domain = model['input_domain']['moving']
    if not finite(u):
        status = 'nonfinite_input'
    elif u == 0.:
        status = 'zero_state'
    elif not model['valid'] or domain is None:
        status = 'invalid_model'
    elif 0 < u < domain[0]:
        status = 'unsupported_zero_gap'
    elif not domain[0] <= u <= domain[1]:
        status = 'outside_moving_support'
    else:
        status = 'moving_supported'
    value = predict(model, u) if status in ('zero_state', 'moving_supported') else None
    if value is None and status == 'moving_supported':
        status = 'unsupported_model_support'
    elif value is not None and not finite(value):
        value, status = None, 'nonfinite_prediction'
    return {'predicted_v': value, 'support_status': status}


def evaluate_models(models, observations):
    """C affects residuals and gates only; model parameters/knots stay frozen."""
    data_valid = (len(observations) == len(C_SPEEDS) and all(
        r['valid_for_fit'] is True and not r['invalid_reasons'] and finite(r['v'])
        and finite(r['u']) and abs(r['u'] - expected) <= 1e-6
        and r['requested_vx'] == expected
        for r, expected in zip(observations, C_SPEEDS)))
    ordered = sorted((r for r in observations if finite(r['u']) and finite(r['v'])),
                     key=lambda r: r['u'])
    observed_monotonic = (data_valid and all(
        b['v'] >= a['v'] for a, b in zip(ordered, ordered[1:])))
    result = {'data_valid': data_valid, 'observed_monotonic': observed_monotonic,
              'models': {}, 'held_out_selection': None, 'production_selection': None}
    for kind in MODEL_ORDER:
        model = models[kind]
        points = []
        for row in observations:
            point = {'u': row['u'], 'requested_vx': row['requested_vx'],
                     'observed_v': row['v'], **predict_point(model, row['u'])}
            point['residual'] = (row['v'] - point['predicted_v']
                if row['valid_for_fit'] and finite(row['v']) and point['predicted_v'] is not None
                else None)
            points.append(point)
        supported = (len(points) == len(C_SPEEDS) and all(
            p['support_status'] == 'moving_supported' and p['predicted_v'] is not None for p in points))
        available = [p for p in points if p['residual'] is not None]
        metrics = errors([p['residual'] for p in available], [p['observed_v'] for p in available])
        sorted_points = sorted((p for p in points if finite(p['u'])), key=lambda p: p['u'])
        branch_monotonic = (all(b[1] >= a[1] for a, b in zip(model['knots'], model['knots'][1:]))
            if kind == 'M3' else model['parameters'].get('k', model['parameters'].get('a', -1.)) >= 0.)
        predicted_monotonic = (supported and model['monotonic'] is True and branch_monotonic and all(
            b['predicted_v'] >= a['predicted_v'] for a, b in zip(sorted_points, sorted_points[1:])))
        passed = bool(data_valid and supported and observed_monotonic and predicted_monotonic
            and metrics['n'] == len(C_SPEEDS)
            # Floating arithmetic tolerance only, far below measurement precision.
            and metrics['max_abs_error'] <= MAX_ABS_ERROR_MPS + 1e-12
            and metrics['rmse'] <= RMSE_MPS + 1e-12)
        result['models'][kind] = {
            'frozen_model': copy.deepcopy(model), 'points': points, 'metrics': metrics,
            'input_domain': model['input_domain'], 'output_domain': model['output_domain'],
            'all_predictions_supported': supported, 'predicted_monotonic': predicted_monotonic,
            'passed': passed}
        if passed and result['held_out_selection'] is None:
            result['held_out_selection'] = kind
    return result


def snapshot(directory):
    return {str(p.resolve()): digest(p) for p in directory.rglob('*') if p.is_file()}


def load_frozen(path, expected_sha256=None):
    """Read the frozen JSON and its existing, hash-pinned B provenance only."""
    before = snapshot(path.parent)
    if expected_sha256 is not None and before[str(path)] != expected_sha256.lower():
        raise ValueError('frozen calibration SHA256 mismatch')
    frozen = json.loads(path.read_text())
    if frozen.get('schema_version') != 1 or not frozen.get('offline_only') or not frozen.get('inputs'):
        raise ValueError('unsupported frozen calibration schema or missing B provenance')
    models = frozen['truth']['models']
    for kind in MODEL_ORDER:
        if models[kind]['kind'] != kind:
            raise ValueError('frozen model kind mismatch')
    manifests = []
    protected = [path.parent]
    for record in frozen['inputs']:
        directory = Path(record['directory']).resolve()
        manifest_path = directory / 'manifest.json'
        hashes = record['file_hashes']
        if str(manifest_path) not in hashes:
            raise ValueError('frozen B manifest hash missing')
        for name, expected in hashes.items():
            if digest(Path(name)) != expected:
                raise ValueError(f'frozen B input hash changed: {name}')
        before.update(hashes)
        manifests.append(json.loads(manifest_path.read_text()))
        protected.append(directory)
    return frozen, manifests, protected, before


def contract(manifest):
    return {'source_hashes': manifest.get('source_hashes'), 'policy_sha256': manifest.get('policy_sha256'),
            'fixture': {k: v for k, v in (manifest.get('fixture') or {}).items()
                        if k not in ('generated_world', 'source_world')}}


def assert_unchanged(before, directories):
    after = {}
    for directory in directories:
        after.update(snapshot(directory))
    if before != after:
        raise RuntimeError('input artifacts changed during offline validation')


def validate(frozen_path, run_directory, expected_sha256=None):
    frozen_path = Path(frozen_path).resolve()
    run_directory = Path(run_directory).resolve()
    frozen, b_manifests, protected, before = load_frozen(frozen_path, expected_sha256)
    if any(run_directory == p or p in run_directory.parents or run_directory in p.parents for p in protected):
        raise ValueError('C must be a fresh separate run outside B and B-fit')
    before.update(snapshot(run_directory))
    manifest = json.loads((run_directory / 'manifest.json').read_text())
    termination = manifest.get('termination') or {}
    if (termination.get('final_status') or termination.get('status')) != 'completed':
        raise ValueError('C run not completed')
    if (manifest.get('cleanup') or {}).get('passed') is not True:
        raise ValueError('C cleanup not passed')
    if manifest.get('sequence') != make_sequence(C_SPEEDS) or not finite(manifest.get('experiment_start_sim_s')):
        raise ValueError('C must have the preregistered four unchanged 12-s blocks and valid start time')
    raw = run_directory / 'events.jsonl'
    raw_hash = digest(raw)
    if not manifest.get('owner_token_sha256') or any(
        manifest['owner_token_sha256'] == b.get('owner_token_sha256') for b in b_manifests
    ) or raw_hash in {r['raw_sha256'] for r in frozen['inputs']}:
        raise ValueError('C is not an independent session/capture')
    if any(contract(manifest) != contract(b) for b in b_manifests):
        raise ValueError('C policy/control/geometry/fixture contract differs from frozen B')
    events = [json.loads(line) for line in raw.read_text().splitlines() if line.strip()]
    observations = extract_observations(events, manifest, raw_hash)
    # Extraction already checks supported streams; also reject a supported but
    # incorrect command plateau rather than blessing it as held-out C.
    for row in observations:
        for name, stats in row['pipeline'].items():
            if stats['mean'] is not None and abs(stats['mean'] - row['requested_vx']) > .005:
                row['invalid_reasons'].append(f'{name} steady command differs from request by >.005 m/s')
                row['valid_for_fit'] = False
    result = {
        'schema_version': 1, 'offline_only': True, 'protocol': 'C_frozen_B',
        'frozen_calibration': str(frozen_path), 'frozen_calibration_sha256': before[str(frozen_path)],
        'run_directory': str(run_directory), 'raw_sha256': raw_hash,
        'preregistration': {'policy_inputs': list(C_SPEEDS), 'sequence': make_sequence(C_SPEEDS),
            'max_abs_error_mps': MAX_ABS_ERROR_MPS, 'rmse_mps': RMSE_MPS,
            'model_order': list(MODEL_ORDER), 'threshold_provenance': 'new prospective Protocol C criteria'},
        'observations': observations, **evaluate_models(frozen['truth']['models'], observations),
        'residual_sign': 'observed minus predicted', 'refit_performed': False,
        'support_note': 'explicit f(0)=0; no extrapolation or zero-to-first-moving-knot bridge',
        'production_calibration': 'deferred until USER-RUN C and ChatGPT review; no production writes',
    }
    assert_unchanged(before, protected + [run_directory])
    result['inputs_unchanged'] = True
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frozen_calibration', type=Path, help='frozen B calibration.json; retain its referenced B runs')
    parser.add_argument('run_directory', type=Path, help='one exact fresh C run directory, never a glob')
    parser.add_argument('--expected-sha256', help='reject a frozen JSON that differs from this preregistered digest')
    parser.add_argument('--output-dir', type=Path, help='fresh directory outside B/B-fit/C; required unless --dry-run')
    parser.add_argument('--dry-run', action='store_true', help='show preregistered inputs and frozen predictions only; do not read C or write files')
    args = parser.parse_args(argv)
    try:
        path = args.frozen_calibration.expanduser().resolve()
        run = args.run_directory.expanduser().resolve()
        frozen, _, protected, _ = load_frozen(path, args.expected_sha256)
        if args.dry_run:
            print(json.dumps({'dry_run': True, 'c_read': False, 'refit_performed': False,
                'frozen_calibration_sha256': digest(path), 'policy_inputs': list(C_SPEEDS),
                'predictions': {kind: [predict_point(frozen['truth']['models'][kind], u)
                    for u in C_SPEEDS] for kind in MODEL_ORDER}}, indent=2, allow_nan=False))
            return 0
        if args.output_dir is None:
            raise ValueError('--output-dir required unless --dry-run')
        output = args.output_dir.expanduser().resolve()
        if output.exists() or any(output == p or p in output.parents for p in protected + [run]):
            raise ValueError('output must be a fresh directory outside all B/B-fit/C inputs')
        result = validate(path, run, args.expected_sha256)
        output.mkdir(parents=True, exist_ok=False)
        (output / 'validation.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        print(json.dumps({'output_directory': str(output), 'data_valid': result['data_valid'],
            'held_out_selection': result['held_out_selection'], 'production_selection': None,
            'inputs_unchanged': True}))
        return 0 if result['data_valid'] else 1
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
        print(f'offline frozen validation failed: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
