"""Sensor-only precheck admission and physical-rest evidence."""
import math
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import warehouse_ab_trial as trial


def test_unknown_and_obstacle_in_observed_pocket_reject():
    grid = dict(width=100, height=100, resolution=.1, origin=(-5., -5.), data=[0]*10000)
    assert trial.open_pocket(grid, (0., 0.), radius=1.5)
    grid['data'][50*100+50] = -1
    assert not trial.open_pocket(grid, (0., 0.), radius=1.5)
    grid['data'][50*100+50] = 100
    assert not trial.open_pocket(grid, (0., 0.), radius=1.5)
    grid['data'][50*100+50] = 0
    assert not trial.open_pocket(grid, (4.5, 4.5), radius=1.5)


def test_pocket_snapshot_distinguishes_rejected_cells_and_keeps_original_decision():
    grid = dict(width=40, height=40, resolution=.1, origin=(-2., -2.), data=[0]*1600)
    grid['data'][820] = -1
    grid['data'][821] = 100
    result = trial.pocket_snapshot(grid, (0., 0.))
    assert result['decision'] == trial.open_pocket(grid, (0., 0.)) is False
    assert result['rejected_cells']['unknown'] == 1
    assert result['rejected_cells']['occupied'] == 1
    assert result['rejected_cells']['outside'] == result['rejected_cells']['invalid'] == 0
    assert result['grid']['data'] == grid['data'] and result['radius_m'] == 1.5
    assert trial.pocket_snapshot(grid, (1.9, 1.9))['rejected_cells']['outside'] > 0
    malformed = dict(grid, data=[])
    assert trial.pocket_snapshot(malformed, (0., 0.))['rejected_cells']['invalid'] == 1


def test_map_ingestion_and_admission_snapshot_preserve_message_and_checked_tf(tmp_path):
    import json
    from types import SimpleNamespace
    from nav_msgs.msg import OccupancyGrid
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'; msg.header.stamp.sec = 12
    msg.info.width = msg.info.height = 40; msg.info.resolution = .1
    msg.info.origin.position.x = msg.info.origin.position.y = -2.
    msg.info.origin.orientation.w = 1.; msg.data = [0]*1600; msg.data[820] = -1
    monitor = object.__new__(trial.Monitor)
    monitor.latest = {}; monitor.failure = None; monitor.clock = 12.2
    monitor.tf_diagnostic = dict(stamp=12.1, simulation_age_s=.1, passed=True)
    monitor.pose = lambda: (0., 0., .25)
    monitor.receive_map(msg)
    assert monitor.grid['data'] is msg.data
    assert not trial.save_pocket_admission(monitor, tmp_path, 'initial')
    saved = json.loads((tmp_path/'pocket-admission-initial.json').read_text())
    assert saved['map_evidence']['message']['header']['stamp']['sec'] == 12
    assert saved['grid']['data'] == list(msg.data)
    assert saved['map_evidence']['data_path'] == 'grid.data'
    assert 'data' not in saved['map_evidence']['message']
    assert (tmp_path/'pocket-admission-initial.json').stat().st_size < 10000
    assert saved['sensor_pose'] == [0., 0., .25]
    assert saved['tf_diagnostic']['stamp'] == 12.1


def test_large_map_callback_converts_metadata_only(monkeypatch):
    from nav_msgs.msg import OccupancyGrid
    import rosidl_runtime_py.convert as conversion
    msg = OccupancyGrid()
    msg.info.width = msg.info.height = 800
    msg.info.resolution = .05; msg.info.origin.orientation.w = 1.
    msg.data = [0]*640000
    converted = []
    original = conversion.message_to_ordereddict
    def metadata_only(value, **kwargs):
        assert value is not msg
        converted.append(value)
        return original(value, **kwargs)
    monkeypatch.setattr(conversion, 'message_to_ordereddict', metadata_only)
    monitor = object.__new__(trial.Monitor)
    monitor.latest = {}; monitor.failure = None
    monitor.receive_map(msg)
    assert msg.header in converted and msg.info in converted
    assert monitor.grid['data'] is msg.data


@pytest.mark.parametrize('flags', [
    ['--precheck-spawn-x', '1'],
    ['--precheck-only', '--precheck-spawn-x', 'nan', '--precheck-spawn-y', '1'],
    ['--precheck-spawn-x', '1', '--precheck-spawn-y', '2'],
    ['--stationary-containment', '--precheck-spawn-x', '1', '--precheck-spawn-y', '2'],
    ['--precheck-only', '--route-mode', 'full', '--continuation-yaws', '0', '0', '0'],
    ['--stationary-containment', '--route-mode', 'full', '--continuation-yaws', '0', '0', '0'],
    ['--route-mode', 'full'],
    ['--route-mode', 'full', '--continuation-yaws', '0', 'nan', '0'],
    ['--route-mode', 'full', '--continuation-yaws', '0', '0', '0', '--execute', '--authorize-motion', 'route'],
])
def test_new_motion_options_reject_before_inputs_or_execution(flags, monkeypatch):
    monkeypatch.setattr(trial, 'check_inputs', lambda args: pytest.fail('Invalid options reached inputs'))
    with pytest.raises(SystemExit):
        trial.main(['--profile', 'A', *flags])


def test_full_route_selection_has_distinct_authority_and_explicit_yaws(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(trial, 'check_inputs', lambda args: {})
    monkeypatch.setattr(trial, 'execute', lambda args, output: calls.append(args) or 0)
    assert trial.main(['--profile', 'A', '--route-mode', 'full', '--continuation-yaws', '.1', '.2', '.3',
                       '--execute', '--authorize-motion', 'route-full', '--route-envelope','reviewed.json','--route-envelope-sha256','a'*64,'--output-root', str(tmp_path)]) == 0
    assert calls[0].route_mode == 'full' and calls[0].continuation_yaws == [.1, .2, .3]


def test_rest_requires_measured_motion_and_all_six_command_axes():
    zero = [0.]*6
    assert trial.at_rest(zero, (0., 0., 0.))
    for axis in range(6):
        command = zero.copy(); command[axis] = .001
        assert not trial.at_rest(command, (0., 0., 0.))
    assert not trial.at_rest(zero, (.03, 0., 0.))
    assert not trial.at_rest(zero, (0., 0., .03))
    assert not trial.at_rest(zero, (math.nan, 0., 0.))


def test_stop_tracks_coast_separately_and_requires_advancing_settled_window():
    stop=trial.StopEvidence()
    assert stop.observe(sim=0.,wall=0.,pose=(0.,0.,0.),stationary=False,zero=False) is None
    assert stop.observe(sim=1.,wall=1.,pose=(0.,0.,0.),stationary=False,zero=True) is None
    assert stop.observe(sim=2.,wall=2.,pose=(.225,0.,0.),stationary=True,zero=True) is None
    assert stop.observe(sim=2.,wall=10.,pose=(.225,0.,0.),stationary=True,zero=True) is None
    result=stop.observe(sim=7.,wall=11.,pose=(.226,0.,0.),stationary=True,zero=True)
    assert result['whole_stop_max_displacement_m']==.226
    assert result['max_displacement_m']<.002


def test_stop_cannot_erase_large_coast_or_reappearing_nonzero():
    for pose,zero in (((.26,0.,0.),True),((0.,0.,0.),False)):
        stop=trial.StopEvidence();stop.observe(sim=1.,wall=1.,pose=(0.,0.,0.),stationary=False,zero=True)
        try: stop.observe(sim=2.,wall=2.,pose=pose,stationary=True,zero=zero)
        except RuntimeError: pass
        else: assert False


def test_exception_latch_publishes_immediate_first_fault(tmp_path):
    from types import SimpleNamespace
    import json
    heartbeat=tmp_path/'heartbeat.json'
    monitor=SimpleNamespace(failure=None,clock_wall=12.,clock=4.,samples=5)
    trial.latch_failure(monitor,heartbeat,'excursion')
    trial.latch_failure(monitor,heartbeat,'cleanup')
    assert monitor.failure=='excursion'
    assert json.loads(heartbeat.read_text())['failure']=='excursion'


def test_stationary_acceptance_requires_watchdog_pause_zero_commands_and_cleanup():
    import copy
    pin = dict(pid=123, start_ticks=456)
    result = dict(reason='monitor:stationary_qualification_injection', emergency=True,
        pause=dict(contained=True, state='T', threads={'123': 'T'}), remaining={},
        owned_pid_start_ticks={'123': '456'})
    commands = dict(samples=100, nonzero=0)
    rest = dict(start_sim=1., end_sim=6., wall_s=3., max_displacement_m=.01,
        max_yaw_rad=.01, six_axis_final_zero=True, measured_ground_truth_rest=True)
    assert trial.stationary_passed(result, pin, commands, rest, 10)
    for key, value in [('reason', 'wall_limit'), ('remaining', {'1': '2'}),
                       ('pause', dict(contained=False)), ('owned_pid_start_ticks', {'123': '457'})]:
        bad = copy.deepcopy(result); bad[key] = value
        assert not trial.stationary_passed(bad, pin, commands, rest, 10)
    assert not trial.stationary_passed(result, pin, dict(samples=100, nonzero=1), rest, 10)
    assert not trial.stationary_passed(result, pin, dict(samples=0, nonzero=0), rest, 10)
    assert not trial.stationary_passed(result, pin, commands, dict(rest, end_sim=2.), 10)
    assert not trial.stationary_passed(result, pin, commands, rest, 0)


def test_stationary_mode_is_offline_by_default_and_gets_distinct_attempt(tmp_path, monkeypatch):
    import json
    monkeypatch.setattr(trial, 'check_inputs', lambda args: dict(profile=args.profile))
    calls = []
    monkeypatch.setattr(trial, 'execute', lambda args, output: calls.append((args, output)) or 0)
    argv = ['--profile', 'A', '--stationary-containment', '--output-root', str(tmp_path)]
    assert trial.main(argv) == 0 and not calls
    assert trial.main(argv + ['--execute']) == 0
    assert calls[0][0].stationary_containment and calls[0][0].wall_limit == 150.
    assert (tmp_path/'stationary.used').exists() and not (tmp_path/'A.used').exists()
    assert calls[0][1].name.startswith('stationary-')
    assert trial.main(argv + ['--execute']) == 2 and len(calls) == 1


def test_construction_audit_reads_captured_fortress_stack_output(tmp_path):
    import pytest
    (tmp_path/'ros-log').mkdir()
    log = tmp_path/'stack.log'
    log.write_text('[ign-1] Physics initialized\n[other-2] mesh not found\n')
    assert trial.construction_audit(tmp_path) == [str(log)]
    log.write_text('[ign-1] Failed to construct collision geometry\n')
    with pytest.raises(RuntimeError, match='Physics construction'):
        trial.construction_audit(tmp_path)
    log.write_text('[other-2] no simulator output\n')
    with pytest.raises(RuntimeError, match='unavailable'):
        trial.construction_audit(tmp_path)


def test_stationary_does_not_claim_or_require_motion_pocket():
    # Unknown/occupied motion clearance cannot veto a zero-only rest/containment test.
    for value in (-1, 100):
        grid = dict(width=40, height=40, resolution=.1, origin=(-2., -2.), data=[value]*1600)
        result = trial.motion_pocket_admission(grid, (0., 0.), stationary_containment=True)
        assert result == dict(required=False, passed=None, radius_m=1.5,
                              status='not_required_stationary')
        assert not trial.open_pocket(grid, (0., 0.))


def test_motion_still_requires_unchanged_free_disk():
    import pytest
    grid = dict(width=40, height=40, resolution=.1, origin=(-2., -2.), data=[0]*1600)
    assert trial.motion_pocket_admission(grid, (0., 0.), stationary_containment=False) == dict(
        required=True, passed=True, radius_m=1.5, status='passed')
    for value in (-1, 100):
        grid['data'][20*40+20] = value
        with pytest.raises(RuntimeError, match='Sensor-observed 1.5m open pocket absent'):
            trial.motion_pocket_admission(grid, (0., 0.), stationary_containment=False)


def test_obstacle_only_map_is_not_observed_free_space():
    from nav_msgs.msg import OccupancyGrid
    m=trial.Monitor.__new__(trial.Monitor);m.latest={};m.failure=None
    msg=OccupancyGrid();msg.info.width=4;msg.info.height=4
    msg.info.resolution=.1;msg.info.origin.orientation.w=1.
    msg.data=[-1]*15+[100]
    m.receive_map(msg)
    assert m.grid['observed_free_present'] is False
    msg.data=[0]+[-1]*14+[100]
    m.receive_map(msg)
    assert m.grid['observed_free_present'] is True
    assert not trial.open_pocket(m.grid,(0.,0.))  # One ray is not disk clearance.


def test_precheck_cli_is_offline_default_and_consumes_only_precheck_attempt(tmp_path, monkeypatch):
    import json
    monkeypatch.setattr(trial, 'check_inputs', lambda args: dict(profile=args.profile))
    calls = []
    monkeypatch.setattr(trial, 'execute', lambda args, output: calls.append((args, output)) or 0)
    argv = ['--profile', 'A', '--precheck-only', '--output-root', str(tmp_path)]
    assert trial.main(argv) == 0 and not calls
    execute = argv + ['--execute', '--authorize-motion', 'precheck-only']
    assert trial.main(execute) == 0
    assert calls[0][0].precheck_only and calls[0][0].wall_limit == 420.
    assert calls[0][1].name.startswith('precheck-')
    assert (tmp_path/'precheck.used').is_file() and not (tmp_path/'A.used').exists()
    assert json.loads((tmp_path/'precheck.used').read_text())['motion_authorization'] == 'precheck-only'
    assert trial.main(execute) == 2 and len(calls) == 1


@pytest.mark.parametrize('flags', [
    ['--precheck-only', '--execute'],
    ['--execute'],
    ['--precheck-only', '--execute', '--authorize-motion', 'route'],
    ['--execute', '--authorize-motion', 'precheck-only'],
    ['--stationary-containment', '--precheck-only'],
    ['--stationary-containment', '--execute', '--authorize-motion', 'precheck-only'],
    ['--precheck-only', '--check', '--execute', '--authorize-motion', 'precheck-only'],
])
def test_invalid_motion_authority_is_rejected_before_preflight_or_attempt(tmp_path, monkeypatch, flags):
    def forbidden(*args):
        pytest.fail('Invalid authority reached preflight or execution')
    monkeypatch.setattr(trial, 'check_inputs', forbidden)
    monkeypatch.setattr(trial, 'execute', forbidden)
    with pytest.raises(SystemExit) as error:
        trial.main(['--profile', 'A', '--output-root', str(tmp_path), *flags])
    assert error.value.code == 2 and not list(tmp_path.iterdir())


def test_failed_precheck_attempt_is_not_retried_or_charged_to_route(tmp_path, monkeypatch):
    monkeypatch.setattr(trial, 'check_inputs', lambda args: {})
    calls = []
    monkeypatch.setattr(trial, 'execute', lambda *args: calls.append(args) or 1)
    argv = ['--profile', 'A', '--precheck-only', '--execute', '--authorize-motion',
            'precheck-only', '--output-root', str(tmp_path)]
    assert trial.main(argv) == 1
    assert trial.main(argv) == 2 and len(calls) == 1
    assert not (tmp_path/'A.used').exists()


@pytest.mark.parametrize('profile', ['A', 'B'])
def test_route_authorization_remains_separate_from_precheck(tmp_path, monkeypatch, profile):
    monkeypatch.setattr(trial, 'check_inputs', lambda args: {})
    calls = []
    monkeypatch.setattr(trial, 'execute', lambda args, output: calls.append((args, output)) or 0)
    assert trial.main(['--profile', profile, '--execute', '--authorize-motion', 'route','--route-envelope','reviewed.json','--route-envelope-sha256','a'*64,
                       '--output-root', str(tmp_path)]) == 0
    assert not calls[0][0].precheck_only and calls[0][0].authorize_motion == 'route'
    assert (tmp_path/(profile+'.used')).is_file() and not (tmp_path/'precheck.used').exists()


@pytest.mark.parametrize('flags', [
    ['--planning-only','--execute'],
    ['--planning-only','--authorize-motion','route'],
    ['--planning-only','--route-mode','full','--continuation-yaws','0','0','0'],
    ['--authorize-planning'],
    ['--planning-only','--precheck-only'],
])
def test_planning_only_rejects_ambiguous_authority_and_route_options(flags,monkeypatch):
    monkeypatch.setattr(trial,'check_inputs',lambda args:pytest.fail('Invalid authority reached preflight'))
    with pytest.raises(SystemExit):trial.main(['--profile','A',*flags])


def test_planning_only_has_separate_authority_and_attempt_identity(tmp_path,monkeypatch):
    calls=[]
    monkeypatch.setattr(trial,'check_inputs',lambda args:{})
    monkeypatch.setattr(trial,'execute',lambda args,output:calls.append(args) or 0)
    flags=['--profile','A','--planning-only','--execute','--authorize-planning','--output-root',str(tmp_path)]
    assert trial.main(flags)==0
    assert calls[0].planning_only and calls[0].authorize_motion is None
    assert (tmp_path/'planning.used').exists() and not (tmp_path/'A.used').exists()
    assert trial.main(flags)==2 and len(calls)==1
