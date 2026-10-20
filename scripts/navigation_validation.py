#!/usr/bin/python3
"""Deterministic offline validation. Never launches ROS, Gazebo or motion."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MATRIX = ROOT / 'course_navigation/config/navigation_validation_matrix.json'
FOCUSED = ['stage4_route', 'stage4_transport', 'coherent_localization', 'route_geometry',
           'binary_journal', 'contact_evidence', 'continuous_geometry', 'geometry_audit_states',
           'navigation_map_derivation', 'stationary_capture', 'unqualified_motion',
           'contact_intervals', 'segment_evidence', 'start_connection', 'predictive_reach',
           'stage4_safety_guard','stage4_dispatch_deployment','stage4_segment_execution',
           'corridor_portal','stage4_contact_wiring','stage4_bounds','independent_stop',
           'navigation_watchdog']


def junit_passed(path, *, expected_modules=()):
    try:
        root = ET.parse(path).getroot()
        suites = list(root.iter('testsuite'))
        present = {case.get('classname', '').split('.')[-1] for case in root.iter('testcase')}
        return (bool(suites) and sum(int(s.get('tests', '0')) for s in suites) > 0 and
                set(expected_modules) <= present and
                all(int(s.get(k, '0')) == 0 for s in suites for k in ('failures', 'errors', 'skipped')))
    except (OSError, ValueError, ET.ParseError):
        return False


def run_command(command, output, name, *, timeout):
    started = time.monotonic()
    result = dict(command=command, cwd=str(ROOT), timeout_wall_s=timeout,
                  started_unix_s=time.time(), timed_out=False, interrupted=False,
                  log=str(output / (name+'.log')))
    proc = None
    with (output / (name+'.log')).open('x') as log:
        try:
            proc = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            try:
                result['exit_code'] = proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                result['timed_out'] = True
                # Only this freshly created, still-owned process group is signalled.
                os.killpg(proc.pid, signal.SIGKILL)
                result['exit_code'] = proc.wait(timeout=5)
        except OSError as exc:
            result.update(exit_code=None, error=str(exc))
        except BaseException as exc:
            result.update(exit_code=None, interrupted=True, error=type(exc).__name__+': '+str(exc))
        finally:
            if proc is not None and proc.poll() is None:
                os.killpg(proc.pid, signal.SIGKILL)
                result['exit_code'] = proc.wait(timeout=5)
    result.update(wall_s=time.monotonic()-started,
                  status='PASS' if result['exit_code'] == 0 and not result['timed_out'] and not result['interrupted'] else 'FAIL')
    return result


def checklist(matrix, results, *, source_stable):
    checks = []
    for gate in matrix['gates']:
        suite = gate['suite']
        passed = source_stable and gate['level'] == 'offline' and results.get(suite, {}).get('status') == 'PASS'
        checks.append(dict(id=gate['id'], status='PASS' if passed else 'FAIL', level=gate['level'],
                           requires=gate['requires'], evidence=results.get(suite),
                           reason=('source_changed_during_validation' if not source_stable else
                                   'offline_regression_only' if passed else
                                   gate.get('missing', 'not_executed_or_failed'))))
    return checks


def source_identity():
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    status = subprocess.check_output(['git', 'status', '--porcelain=v1'], cwd=ROOT, text=True)
    paths = subprocess.check_output(['git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard'], cwd=ROOT).decode().split('\0')
    files = {}
    for relative in sorted(set(paths)):
        p = ROOT / relative
        if relative and p.is_file() and (relative.startswith(('scripts/', 'tests/', 'course_navigation/', 'course_bringup/', 'course_slam/', 'course_sim/')) or relative in ('AGENTS.md', 'CODEX_GUIDE.md', 'PLAN.md')):
            files[relative] = hashlib.sha256(p.read_bytes()).hexdigest()
    return dict(head=head, status=status, files=files)


def commands(output):
    return {
        'focused': [(['/usr/bin/python3', '-m', 'pytest', '-q', '-p', 'no:cacheprovider', '-o', 'addopts=',
                      '--junitxml='+str(output/'focused.xml'), *['tests/test_'+n+'.py' for n in FOCUSED]], 180)],
        'harness': [(['/usr/bin/python3', '-m', 'pytest', '-q', '-p', 'no:cacheprovider', '-o', 'addopts=',
                      '--junitxml='+str(output/'harness.xml'), 'tests/test_navigation_validation_runner.py'], 30)],
        'native': [(['cmake', '-S', str(ROOT/'scripts/contact_observer'), '-B', str(output/'native-build')], 60),
                   (['cmake', '--build', str(output/'native-build'), '-j2'], 120),
                   (['ctest', '--test-dir', str(output/'native-build'), '--output-on-failure', '--no-tests=error'], 30)],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New exclusive directory outside checkout')
    parser.add_argument('--run-offline', action='store_true', help='Execute bounded offline tests; no physical qualification')
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if output == ROOT or ROOT in output.parents:
        parser.error('Evidence must be outside the source checkout')
    matrix = json.loads(MATRIX.read_text())
    planned = commands(output)
    if not args.run_offline:
        print(json.dumps(dict(dry_run=True, motion_authorized=False, full_route_pass=False,
                              matrix=str(MATRIX), commands=planned), indent=2))
        return 0
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_LOG_DIR=str(output/'ros-log'), OPENBLAS_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1')
    os.environ.pop('PYTEST_ADDOPTS', None)
    os.environ.pop('PYTEST_PLUGINS', None)
    os.environ['PYTEST_DISABLE_PLUGIN_AUTOLOAD'] = '1'
    global_deadline = time.monotonic()+480.
    before = source_identity()
    (output/'source-before.json').write_text(json.dumps(before, indent=2))
    results = {}
    for suite, steps in planned.items():
        records = []
        for i, (command, limit) in enumerate(steps):
            remaining = global_deadline-time.monotonic()
            if remaining <= 0:break
            record = run_command(command, output, suite+'-'+str(i), timeout=min(limit, remaining))
            records.append(record)
            if record['status'] != 'PASS':
                break
        passed = len(records) == len(steps) and all(r['status'] == 'PASS' for r in records)
        if suite in ('focused', 'harness'):
            modules = ['test_'+n for n in FOCUSED] if suite == 'focused' else ['test_navigation_validation_runner']
            passed = passed and junit_passed(output/(suite+'.xml'), expected_modules=modules)
        results[suite] = dict(status='PASS' if passed else 'FAIL', steps=records)
        if any(r.get('interrupted') for r in records):break
    after = source_identity()
    (output/'source-after.json').write_text(json.dumps(after, indent=2))
    stable = before == after
    checks = checklist(matrix, results, source_stable=stable)
    report = dict(schema_version=1, motion_authorized=False, full_route_pass=False,
                  qualification='OFFLINE_ONLY', matrix_sha256=hashlib.sha256(MATRIX.read_bytes()).hexdigest(),
                  source_stable=stable, offline_pass=stable and set(results)==set(planned) and
                  all(r['status'] == 'PASS' for r in results.values()),
                  all_requirements_pass=all(c['status'] == 'PASS' for c in checks), suites=results, checklist=checks)
    (output/'matrix.json').write_text(json.dumps(matrix, indent=2))
    (output/'checklist.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(dict(checklist=str(output/'checklist.json'), offline_pass=report['offline_pass'],
                          all_requirements_pass=report['all_requirements_pass'], motion_authorized=False)))
    # Exit zero means the requested OFFLINE suite passed, not readiness for motion.
    return 0 if report['offline_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
