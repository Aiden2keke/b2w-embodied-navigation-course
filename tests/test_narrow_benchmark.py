import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from narrow_fixture import geometry, fixture, chosen_route, classify_failure

ROOT = Path(__file__).resolve().parents[1]

def test_widths_derive_from_current_baseline():
    g = geometry(ROOT / 'course_navigation/config/nav2.yaml')
    assert g['padded_width_m'] == pytest.approx(.86)
    assert g['padded_length_m'] == pytest.approx(1.5)
    assert g['inscribed_radius_m'] == pytest.approx(.43)
    assert g['widths_m'] == [.8, .85, .9, 1.0, 1.2, 1.6, 1.7]

def test_forced_geometry_has_no_bypass_and_exact_width(tmp_path):
    g = geometry(ROOT / 'course_navigation/config/nav2.yaml')
    b = fixture(ROOT, tmp_path, 'B', 1., g)
    boxes = {v['id']: v for v in b['boxes']}
    assert boxes['lower']['ymax'] == -.5
    assert boxes['upper']['ymin'] == .5
    assert boxes['upper']['ymax'] == b['bounds'][3]
    assert boxes['lower']['ymin'] == b['bounds'][2]
    assert ET.parse(tmp_path / 'fixture.world').find('.//world').get('name') == 'course_world'
    a = fixture(ROOT, tmp_path, 'A', 1., g)
    upper = next(v for v in a['boxes'] if v['id'] == 'upper')
    assert a['bounds'][3] - upper['ymax'] == pytest.approx(g['wide_route_width_m'])

def test_route_requires_crossing_not_endpoint_or_nearest_point():
    assert chosen_route([(0,0),(6,0)], 1.) == 'narrow'
    assert chosen_route([(0,0),(2,3),(4,3),(6,0)], 1.) == 'wide'
    assert chosen_route([(0,0),(1,0)], 1.) == 'unclassified'
    assert chosen_route([(0,0),(6,1)], 1.) == 'unclassified'

def test_failure_does_not_call_all_aborts_controller_failure():
    assert classify_failure('aborted', True, False, None) == 'navigation_abort_unattributed'
    assert classify_failure('aborted', True, True, None) == 'navigation_abort_unattributed'
    assert classify_failure('timeout', True, True, None) == 'navigation_timeout'
    assert classify_failure('succeeded', True, True, 0) == 'none'
    assert classify_failure('planner_aborted', False, False, None) == 'planning_failure'
    assert classify_failure('succeeded', True, False, 1) == 'collision'

def test_cleanup_preserves_unowned_process():
    import os
    import signal
    import subprocess
    import uuid
    from narrow_processes import OwnedProcesses
    token = uuid.uuid4().hex
    env = os.environ.copy(); env['B2W_NARROW_OWNER'] = token
    owned = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'], env=env)
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    try:
        tracker = OwnedProcesses(token)
        assert owned.pid in tracker.scan()
        report = tracker.cleanup(owned)
        assert report['passed'] and report['remaining'] == {}
        assert unrelated.poll() is None
    finally:
        for child in (owned, unrelated):
            if child.poll() is None: child.send_signal(signal.SIGTERM)
            child.wait(timeout=5)

def test_humble_ros_log_levels_are_filtered_without_byte_comparison():
    from rcl_interfaces.msg import Log
    from run_narrow_baseline import log_is_warning
    warning = Log(); warning.level = 30
    debug = Log(); debug.level = 10
    assert log_is_warning(warning)
    assert not log_is_warning(debug)

def test_raw_costmap_diagnostic_keeps_cells_and_distribution():
    from run_narrow_baseline import costmap_diagnostic
    cm = dict(metadata=dict(size_x=4, size_y=2, resolution=.5,
              origin=dict(position=dict(x=-1., y=-.5))), data=[0,10,253,254,255,20,30,40])
    result = costmap_diagnostic(cm, [-.75,-.25], [.75,-.25])
    assert result['start'] == dict(cell=[0,0], cost=0)
    assert result['goal'] == dict(cell=[3,0], cost=254)
    assert result['centerline']['costs'] == [0,10,253,254]
    assert result['centerline']['histogram'] == {'0':1,'10':1,'253':1,'254':1}
    assert result['map_counts'] == dict(unknown=1, cost_253=1, cost_254=1)
    # C++ std::round at half cells differs from floor/Python ties-to-even.
    assert result['navfn_start'] == dict(cell=[1,1], cost=20)
    assert result['navfn_goal'] == dict(cell=[4,1], cost=None)

def test_wrapper_reports_failed_workspace_source_and_does_not_launch(tmp_path):
    import subprocess
    scripts=tmp_path/'scripts'; scripts.mkdir()
    install=tmp_path/'install'; install.mkdir()
    (install/'setup.bash').write_text('false\n')
    wrapper=scripts/'run_narrow_baseline.sh'
    wrapper.write_text((ROOT/'scripts/run_narrow_baseline.sh').read_text())
    result=subprocess.run(['bash',str(wrapper),'--help'],text=True,capture_output=True)
    assert result.returncode == 1
    assert 'FAIL: workspace environment source' in result.stderr
    assert 'usage:' not in result.stdout

def test_planner_diagnostic_cannot_expand_into_scenario_width_seed_sweep(tmp_path):
    import json
    import subprocess
    subprocess.run([sys.executable,str(ROOT/'scripts/run_narrow_baseline.py'),
        '--prepare-only','--planner-diagnostic','--goal-distance','3.0',
        '--scenarios','A','B','--widths','.8','1.0','--seeds','1','2',
        '--output-dir',str(tmp_path)],check=True,capture_output=True,text=True)
    fixtures=list(tmp_path.glob('*/B*/fixture.json'))
    assert len(fixtures)==1
    design=json.loads(fixtures[0].read_text())
    assert design['scenario_id']=='B' and design['corridor_width_m']==1.7
    assert design['seed']==0 and design['goal_relative_start_xy']==[3.,0.]
    assert design['measurement_scope']=='planner_only_diagnostic'
    assert not list(tmp_path.glob('*/A*'))

def test_online_navigation_is_one_fixed_gui_case_and_records_selected_config(tmp_path):
    import json
    import subprocess
    config=ROOT/'course_navigation/config/nav2_smac2d_diag.yaml'
    result=subprocess.run([sys.executable,str(ROOT/'scripts/run_narrow_baseline.py'),
        '--prepare-only','--online-navigation','--nav-params-file',str(config),
        '--scenarios','A','B','--widths','.8','--seeds','1','2',
        '--output-dir',str(tmp_path)],capture_output=True,text=True)
    assert result.returncode==0, result.stderr
    fixtures=list(tmp_path.glob('*/B*/fixture.json'))
    assert len(fixtures)==1
    design=json.loads(fixtures[0].read_text())
    assert (design['scenario_id'],design['corridor_width_m'],design['seed'])==('B',1.7,0)
    assert design['goal_relative_start_xy']==[6.,0.]
    assert design['measurement_scope']=='fixed_online_navigation'
    manifest=json.loads(next(tmp_path.glob('*/manifest.json')).read_text())
    assert manifest['nav_params_file']==str(config)
    assert manifest['gui'] and manifest['rviz'] and manifest['hold_for_observation']

def test_observation_gate_requires_explicit_completion_text(tmp_path):
    from run_narrow_baseline import observation_complete
    gate=tmp_path/'observation-complete.txt'
    assert not observation_complete(gate)
    gate.write_text('')
    assert not observation_complete(gate)
    gate.write_text('还没观察完成')
    assert not observation_complete(gate)
    gate.write_text('观察完成\n')
    assert observation_complete(gate)
