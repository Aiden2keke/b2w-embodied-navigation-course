"""Real subprocess containment without ROS, Gazebo or motion."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import warehouse_ab_supervisor as safety


def child(token):
    return subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'],
        env=dict(os.environ, B2W_NARROW_OWNER=token))


def test_pause_proves_owned_server_stopped_and_leaves_sentinel_alive():
    target, sentinel = child('owned-test'), child('sentinel-test')
    try:
        pin = safety.identity(target.pid)
        result = safety.pause_server(pin, 'owned-test')
        assert result['contained'] and result['state'] == 'T'
        assert sentinel.poll() is None
        assert safety.identity(sentinel.pid)['state'] != 'T'
    finally:
        target.kill(); target.wait(); sentinel.kill(); sentinel.wait()


def test_wrong_token_or_start_ticks_never_signals():
    target = child('owned-test')
    try:
        pin = safety.identity(target.pid)
        assert not safety.pause_server(pin, 'foreign-test')['contained']
        pin['start_ticks'] += 1
        assert not safety.pause_server(pin, 'owned-test')['contained']
        assert safety.identity(target.pid)['state'] != 'T'
    finally:
        target.kill(); target.wait()


def test_frozen_clock_and_stale_monitor_are_wall_clock_vetoes():
    healthy = dict(wall=10., clock_advanced_wall=10., failure=None)
    assert safety.health_fault(healthy, 10.1, 1.) is None
    assert safety.health_fault(dict(healthy, clock_advanced_wall=8.), 10.1, 1.) == 'frozen_clock'
    assert safety.health_fault(healthy, 12., 1.) == 'stale_monitor'
    assert safety.health_fault(dict(healthy, failure='contact'), 10.1, 1.) == 'monitor:contact'


def test_parent_death_independent_watchdog_kills_owned_child(tmp_path):
    target, sentinel = child('orphan-test'), child('sentinel-test')
    try:
        manifest = dict(token='orphan-test', parent={'pid': 99999999, 'start_ticks': 1},
            server=safety.identity(target.pid), heartbeat=str(tmp_path/'health.json'),
            result=str(tmp_path/'result.json'),world=str(tmp_path/'observer.world'),wall_limit_s=2.,stale_s=.5)
        path = tmp_path/'watchdog.json'; path.write_text(json.dumps(manifest))
        proc = subprocess.run([sys.executable, str(Path(safety.__file__)), '--watchdog', str(path)], timeout=5)
        target.wait(timeout=2)
        result = json.loads((tmp_path/'result.json').read_text())
        assert proc.returncode == 1 and result['reason'] == 'parent_dead'
        assert not result['pause']['contained'] and result['remaining'] == {}
        assert sentinel.poll() is None
    finally:
        if target.poll() is None: target.kill(); target.wait()
        sentinel.kill(); sentinel.wait()


def test_failed_pause_cannot_report_containment():
    assert not safety.pause_server({'pid': os.getpid(), 'start_ticks': 0}, 'absent')['contained']


def test_malformed_health_always_returns_veto():
    for value in (None, [], 'bad', {}, dict(wall=True), dict(wall=float('nan')),
                  dict(wall=10., clock_advanced_wall=None), dict(wall=10., clock_advanced_wall='bad')):
        assert safety.health_fault(value, 10.1, 1.) is not None


def test_kill_race_does_not_abort_cleanup():
    proc = child('race-test')
    pin = safety.identity(proc.pid)
    proc.kill(); proc.wait()
    assert safety.terminate_owned('race-test', pin)['remaining'] == {}


def test_invalid_config_rejected_before_watching():
    for value in ({}, [], dict(token='x'), dict(token='x', wall_limit_s=float('nan'))):
        try:
            safety.validate_config(value)
        except ValueError:
            pass
        else:
            assert False


def test_truthy_complete_does_not_disarm_and_nonregular_json_rejected(tmp_path):
    assert not safety.normal_complete(dict(complete='yes'), 'unused')
    assert not safety.normal_complete(dict(complete=True, cleanup_passed=True), 'unused')
    regular = tmp_path/'health.json'; regular.write_text('{}')
    link = tmp_path/'link'; link.symlink_to(regular)
    fifo = tmp_path/'fifo'; os.mkfifo(fifo)
    for path in (link, fifo):
        try: safety.read_json(path)
        except (OSError, ValueError): pass
        else: assert False


def test_concurrent_fresh_heartbeat_after_slow_scan_reaches_wall_limit(tmp_path):
    """A separate writer can advance its timestamp during /proc discovery."""
    heartbeat=tmp_path/'health.json'; result=tmp_path/'result.json'; trace=tmp_path/'ages.jsonl'
    safety.write_json(heartbeat,dict(wall=time.monotonic(),clock_advanced_wall=time.monotonic(),phase='bootstrap',failure=None))
    writer_code="""
import sys,time
sys.path.insert(0,sys.argv[1])
from warehouse_ab_supervisor import write_json
while True:
    now=time.monotonic()
    write_json(sys.argv[2],dict(wall=now,clock_advanced_wall=now,phase='bootstrap',failure=None))
    time.sleep(.003)
"""
    writer=subprocess.Popen([sys.executable,'-c',writer_code,str(Path(safety.__file__).parent),str(heartbeat)])
    target=child('fresh-clock-race')
    config=dict(token='fresh-clock-race',parent=safety.identity(os.getpid()),server=None,
        heartbeat=str(heartbeat),result=str(result),world=str(tmp_path/'observer.world'),wall_limit_s=.3,stale_s=1.)
    path=tmp_path/'watchdog.json';path.write_text(json.dumps(config))
    code="""
import sys,time,json
sys.path.insert(0,sys.argv[1])
import warehouse_ab_supervisor as s
def slow_scan(token,*args):
    time.sleep(.05)
    return []
s.discover_servers=slow_scan
original=s.health_fault
def traced(health,now,stale):
    with open(sys.argv[3],'a') as f:
        f.write(json.dumps(dict(now=now,heartbeat_wall=health['wall'],age=now-health['wall']))+'\\n')
    return original(health,now,stale)
s.health_fault=traced
raise SystemExit(s.watchdog(json.load(open(sys.argv[2]))))
"""
    try:
        process=subprocess.run([sys.executable,'-c',code,str(Path(safety.__file__).parent),str(path),str(trace)],timeout=5)
        target.wait(timeout=2)
        evidence=dict(result=json.loads(result.read_text()),ages=[json.loads(line) for line in trace.read_text().splitlines()])
        if os.environ.get('B2W_RACE_EVIDENCE'):
            safety.write_json(os.environ['B2W_RACE_EVIDENCE'],evidence)
        assert process.returncode==1
        assert evidence['result']['reason']=='wall_limit',evidence
        assert all(row['age']>=0 for row in evidence['ages'])
        assert evidence['result']['remaining']=={}
        assert writer.poll() is None
    finally:
        writer.terminate();writer.wait(timeout=2)
        if target.poll() is None:target.kill();target.wait()


def test_actual_fortress_collapsed_title_and_native_mapping_fixture():
    fixture=Path(__file__).parent/'fixtures/fortress_server_inspection_20261023.json'
    row=json.loads(fixture.read_text())[0]
    world=row['cmd'].strip().split()[-1]
    collapsed=row['cmd'].encode()+b'\0'
    assert safety.server_signature(collapsed,row['gazebo_mappings'],world)
    assert safety.server_signature(collapsed.replace(b' ',b'\0'),row['gazebo_mappings'],world)
    assert not safety.server_signature(collapsed,row['gazebo_mappings'],world+'.foreign')
    assert not safety.server_signature(collapsed,[],world)
    assert not safety.server_signature(b'sleep\x0030\0',row['gazebo_mappings'],world)


def test_discovery_wrong_token_never_adopts_unrelated_sentinel():
    sentinel=child('discovery-sentinel')
    try:
        assert safety.discover_servers('wrong-discovery-token','/tmp/observer.world')==[]
        assert sentinel.poll() is None
    finally:
        sentinel.kill();sentinel.wait()


def test_pinned_server_loss_vetoes_even_fresh_health(tmp_path, monkeypatch):
    target = child('server-loss-test')
    pin = safety.identity(target.pid)
    target.kill(); target.wait()
    # Isolate the loop's discovery boundary; no real Fortress is started.
    monkeypatch.setattr(safety, 'discover_servers', lambda *args: [pin])
    now = time.monotonic()
    safety.write_json(tmp_path/'health.json', dict(wall=now, clock_advanced_wall=now, phase='armed'))
    config = dict(token='server-loss-test', parent=safety.identity(os.getpid()), server=pin,
        world=str(tmp_path/'observer.world'), heartbeat=str(tmp_path/'health.json'),
        result=str(tmp_path/'result.json'), wall_limit_s=.1, stale_s=1.)
    assert safety.watchdog(config) == 1
    assert json.loads((tmp_path/'result.json').read_text())['reason'] == 'server_lost'


def test_watchdog_fault_paths_write_result_without_killing_self_or_sentinel(tmp_path):
    """Real signals/cleanup; only Fortress recognition is replaced by a dummy pin."""
    for fault, expected in [('missing', 'missing_or_invalid_monitor'), ('stale', 'stale_monitor'),
                             ('freeze', 'frozen_clock'), ('wall', 'wall_limit')]:
        token = 'fault-test-' + fault
        target, sentinel = child(token), child('foreign-' + token)
        result = tmp_path/(fault+'.json')
        heartbeat = tmp_path/(fault+'-health.json')
        code = '''
import sys,time,json,os
sys.path.insert(0,sys.argv[1])
import warehouse_ab_supervisor as s
pin=json.loads(sys.argv[2]); fault=sys.argv[3]
s.discover_servers=lambda *args:[pin]
now=time.monotonic()
if fault!='missing':
    s.write_json(sys.argv[4],dict(wall=now-3 if fault=='stale' else now,
        clock_advanced_wall=now-3 if fault=='freeze' else now,phase='armed'))
config=dict(token=os.environ['B2W_NARROW_OWNER'],parent=s.identity(os.getppid()),server=None,
    world='/tmp/dummy-test.world',heartbeat=sys.argv[4],result=sys.argv[5],wall_limit_s=.1,stale_s=1.)
raise SystemExit(s.watchdog(config))
'''
        try:
            proc = subprocess.run([sys.executable, '-c', code, str(Path(safety.__file__).parent),
                json.dumps(safety.identity(target.pid)), fault, str(heartbeat), str(result)],
                env=dict(os.environ, B2W_NARROW_OWNER=token), timeout=5)
            target.wait(timeout=2)
            evidence = json.loads(result.read_text())
            assert proc.returncode == 1, evidence
            assert evidence['reason'] == expected, evidence
            assert evidence['pause']['contained'] and evidence['pause']['state'] == 'T'
            assert evidence['remaining'] == {}
            assert sentinel.poll() is None and safety.identity(sentinel.pid)['state'] != 'T'
        finally:
            if target.poll() is None: target.kill(); target.wait()
            sentinel.kill(); sentinel.wait()


def test_discovery_binds_fixture_world_hashes_token_and_start_ticks(tmp_path, monkeypatch):
    from types import SimpleNamespace
    row = json.loads((Path(__file__).parent/'fixtures/fortress_server_inspection_20261023.json').read_text())[0]
    world = row['cmd'].strip().split()[-1]
    proc = tmp_path/'proc'; proc.mkdir()
    (proc/'cmdline').write_bytes(row['cmd'].encode()+b'\0')
    (proc/'maps').write_text('\n'.join('1-2 r-xp 0 00:00 0 '+p for p in row['gazebo_mappings']))
    (proc/'environ').write_bytes(b'B2W_NARROW_OWNER=fixture-owner\0')
    fields = ['S'] + ['0']*18 + ['456']
    (proc/'stat').write_text('123 (ign) '+' '.join(fields))
    real_path = Path
    monkeypatch.setattr(safety, 'Path', lambda p: proc/str(p).split('/')[-1] if str(p).startswith('/proc/123/') else real_path(p))
    monkeypatch.setattr(safety, 'OwnedProcesses', lambda token: SimpleNamespace(scan=lambda: {123: '456'}))
    assert safety.discover_servers('fixture-owner', world) == [dict(pid=123, start_ticks=456, state='S')]
    assert safety.discover_servers('foreign-owner', world) == []
    assert safety.discover_servers('fixture-owner', world+'.wrong') == []
    (proc/'stat').write_text('123 (ign) '+' '.join(fields[:-1]+['457']))
    assert safety.discover_servers('fixture-owner', world) == []
    (proc/'stat').write_text('123 (ign) '+' '.join(fields))
    monkeypatch.setattr(safety, 'FORTRESS_SERVER_LIBRARIES', {p: '0'*64 for p in safety.FORTRESS_SERVER_LIBRARIES})
    assert safety.discover_servers('fixture-owner', world) == []


def test_termination_retains_verified_server_identity_when_postkill_scan_is_empty(monkeypatch):
    # Reproduce domain224: the pinned server disappears before the first scan.
    target = child('server-ledger-test')
    try:
        pin = safety.identity(target.pid)
        monkeypatch.setattr(safety.OwnedProcesses, 'scan', lambda self: {})
        result = safety.terminate_owned('server-ledger-test', pin)
        target.wait(timeout=2.)
        assert str(result['owned_pid_start_ticks'].get(target.pid)) == str(pin['start_ticks'])
        assert result['remaining'] == {}
    finally:
        if target.poll() is None: target.kill(); target.wait()


def test_termination_never_retains_or_kills_unowned_server_identity(monkeypatch):
    target = child('other-ledger-test')
    try:
        pin = safety.identity(target.pid)
        monkeypatch.setattr(safety.OwnedProcesses, 'scan', lambda self: {})
        result = safety.terminate_owned('wrong-ledger-token', pin)
        assert target.pid not in result['owned_pid_start_ticks']
        assert target.poll() is None
    finally:
        target.kill(); target.wait()


def test_termination_rejects_reused_pid_with_same_token_and_different_start_ticks(monkeypatch):
    target = child('reused-ledger-test')
    try:
        stale = dict(safety.identity(target.pid))
        stale['start_ticks'] += 1
        monkeypatch.setattr(safety.OwnedProcesses, 'scan', lambda self: {})
        result = safety.terminate_owned('reused-ledger-test', stale)
        assert target.pid not in result['owned_pid_start_ticks']
        assert target.poll() is None
    finally:
        target.kill(); target.wait()
