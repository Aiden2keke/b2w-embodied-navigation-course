"""Prevent unit-test success, skips or missing evidence becoming motion approval."""
import importlib.util
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def runner():
    path = ROOT / 'scripts/navigation_validation.py'
    assert path.exists(), 'Missing reproducible validation runner'
    spec = importlib.util.spec_from_file_location('navigation_validation', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_pytest_exit_zero_with_skips_or_no_tests_fails(tmp_path):
    m = runner()
    for attrs, expected in [('tests="2" failures="0" errors="0" skipped="0"', True),
                            ('tests="2" failures="0" errors="0" skipped="1"', False),
                            ('tests="0" failures="0" errors="0" skipped="0"', False),
                            ('tests="2" failures="1" errors="0" skipped="0"', False)]:
        p = tmp_path / 'results.xml'
        p.write_text('<testsuites><testsuite '+attrs+'/></testsuites>')
        assert m.junit_passed(p) is expected
    assert not m.junit_passed(tmp_path / 'absent.xml')


def test_filtered_junit_cannot_hide_a_required_module(tmp_path):
    m = runner();p = tmp_path/'results.xml'
    p.write_text('<testsuites><testsuite tests="1" failures="0" errors="0" skipped="0">'
                 '<testcase classname="tests.test_one" name="test_ok"/></testsuite></testsuites>')
    assert not m.junit_passed(p, expected_modules=['test_one', 'test_two'])


def test_interrupt_kills_owned_detached_test_process(tmp_path):
    import json
    import os
    import signal
    import time
    m = runner()
    child_pid = tmp_path/'child.pid'
    code = ('import importlib.util,json,pathlib,sys; '
            f's=importlib.util.spec_from_file_location("v",{str(ROOT/"scripts/navigation_validation.py")!r}); '
            'm=importlib.util.module_from_spec(s);s.loader.exec_module(m); '
            f'cmd=[sys.executable,"-c",{("import os,pathlib,time; pathlib.Path("+repr(str(child_pid))+").write_text(str(os.getpid())); time.sleep(60)")!r}]; '
            f'r=m.run_command(cmd,pathlib.Path({str(tmp_path)!r}),"interrupt",timeout=30); print(json.dumps(r))')
    p = subprocess.Popen([sys.executable, '-c', code], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic()+5
        while not child_pid.exists() and time.monotonic()<deadline:time.sleep(.01)
        assert child_pid.exists()
        pid = int(child_pid.read_text())
        p.send_signal(signal.SIGINT)
        stdout, stderr = p.communicate(timeout=5)
        result = json.loads(stdout)
        assert result['status'] == 'FAIL' and result['interrupted']
        assert not Path('/proc') .joinpath(str(pid)).exists()
    finally:
        if p.poll() is None:p.kill();p.wait()


def test_offline_success_cannot_qualify_live_motion():
    m = runner()
    matrix = {'gates': [
        {'id': 'regression', 'suite': 'focused', 'level': 'offline', 'requires': []},
        {'id': 'stop', 'suite': None, 'level': 'live', 'requires': ['regression'],
         'missing': 'Physical stop not observed'},
    ]}
    checks = m.checklist(matrix, {'focused': {'status': 'PASS'}}, source_stable=True)
    assert [c['status'] for c in checks] == ['PASS', 'FAIL']
    assert checks[1]['reason'] == 'Physical stop not observed'
    assert all(c['status'] == 'FAIL' for c in m.checklist(matrix, {'focused': {'status': 'PASS'}}, source_stable=False))


def test_timeout_is_a_failure_and_preserves_log(tmp_path):
    m = runner()
    result = m.run_command([sys.executable, '-c', 'import time; print("started", flush=True); time.sleep(60)'],
                           tmp_path, 'timeout', timeout=.2)
    assert result['status'] == 'FAIL' and result['timed_out']
    assert 'started' in (tmp_path / 'timeout.log').read_text()


def test_default_is_dry_run_and_does_not_consume_output(tmp_path):
    m = runner()
    out = tmp_path / 'evidence'
    r = subprocess.run([sys.executable, str(ROOT/'scripts/navigation_validation.py'), '--output', str(out)],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert 'motion_authorized' in r.stdout and not out.exists()
