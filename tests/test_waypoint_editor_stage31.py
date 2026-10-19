"""Stage 3.1: catch altered import identities, lossy poses, and nonmetric overlays."""
import copy
import hashlib
import math
import os
from pathlib import Path
import shutil

import pytest
import yaml
from PIL import Image
from test_waypoint_editor import editor, ROOT
from waypoint_editor.core import Editor, pixel_to_map


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    return yaml.safe_load(path.read_text())


def save(path, value):
    path.write_text(yaml.safe_dump(value, sort_keys=False))


def refresh_manifest(folder):
    m = load(folder/'manifest.yaml')
    m['outputs'] = {name: digest(folder/name) for name in m['outputs']}
    save(folder/'manifest.yaml', m)


@pytest.fixture
def v1(editor, tmp_path):
    state = copy.deepcopy(editor.initial)
    for i, goal in enumerate(state['goals']):
        goal.update(position=[.123456789012345+i, -.987654321098765-i], yaw_rad=-3.1012105780252917+i*.371)
    state['markers'] = {'entrance': [1.234, 2.345, -1.234567891234], 'exit': [3., 4., 1.75]}
    state['polygons'] = [dict(kind=k, points=[[0,0],[2,0],[2,1],[0,1]]) for k in ('restricted','corridor')]
    folder = editor.export(state, tmp_path/'exports')
    candidate = load(folder/'candidate.yaml')
    candidate['candidate_schema_version'] = 1
    for key in ('xy_tolerance', 'import_history', 'configuration_identity'):
        candidate.pop(key, None)
    save(folder/'candidate.yaml', candidate)
    manifest = load(folder/'manifest.yaml')
    for key in ('candidate_schema_version', 'xy_tolerance', 'import_history', 'configuration_identity'):
        manifest.pop(key, None)
    save(folder/'manifest.yaml', manifest)
    refresh_manifest(folder)
    return folder, candidate


def test_v1_lossless_reload_migrate_reset_baseline_and_unique_export(editor, v1, tmp_path):
    folder, original = v1
    canonical = copy.deepcopy(editor.initial)
    hashes = {p.name: digest(p) for p in folder.iterdir()}
    editor.import_candidate(folder/'candidate.yaml')
    assert editor.canonical == canonical
    assert editor.initial['goals'] == original['goals']
    assert editor.initial['markers'] == original['markers']
    assert editor.initial['polygons'] == original['polygons']
    new = editor.export(copy.deepcopy(editor.initial), tmp_path/'exports')
    c = load(new/'candidate.yaml'); m = load(new/'manifest.yaml')
    assert c['goals'] == original['goals']
    assert c['candidate_schema_version'] == m['candidate_schema_version'] == 2
    assert c['xy_tolerance'] == m['xy_tolerance'] == dict(radius_m=.25, metric='map_frame_euclidean_xy', reference='robot_base_center', region_safety_checked=False, runtime_action_success=False)
    assert c['import_history'][-1]['migration'] == dict(from_version=1, to_version=2, default_xy_radius_m=.25, poses_unchanged=True)
    assert c['import_history'][-1]['manifest_sha256'] == hashes['manifest.yaml']
    assert new != folder
    assert {p.name: digest(p) for p in folder.iterdir()} == hashes
    again = Editor(editor.map.path, editor.config_path, editor.route_path)
    again.import_candidate(new/'candidate.yaml')
    assert again.initial['goals'] == original['goals']
    assert len(again.import_history) == 2


def test_relocated_map_hash_match(editor, v1, tmp_path):
    folder, _ = v1
    relocated = tmp_path/'relocated'; relocated.mkdir()
    for p in (editor.map.path, editor.map.image_path): shutil.copy2(p, relocated/p.name)
    e = Editor(relocated/editor.map.path.name, editor.config_path, editor.route_path)
    e.import_candidate(folder/'candidate.yaml')
    assert e.initial['map_identity'] == e.map.identity
    assert e.initial['goals'] == load(folder/'candidate.yaml')['goals']


@pytest.mark.parametrize('field', ['schema','map_hash','map_metadata','route','footprint','config','quaternion','policy','costmap','deployable','tolerance','unknown_field'])
def test_fail_closed_incompatible_candidates(editor, v1, field):
    folder, c = v1
    if field=='schema': c['candidate_schema_version']=999
    if field=='map_hash': c['source_map']['image_sha256']='0'*64
    if field=='map_metadata': c['source_map']['metadata']['origin'][2]+=.1
    if field=='route': c['canonical_route_snapshot']['goals'][0]['position'][0]+=1
    if field=='footprint': c['padded_footprint'][0][0]+=.01
    if field=='config': c['source_hashes'][str(editor.config_path)]='0'*64
    if field=='quaternion': c['goals'][0]['quaternion']['z']=0
    if field=='policy': c['occupancy_policy']='free unknown'
    if field=='costmap': c['costmap_metadata']={'local':{}}
    if field=='deployable': c['deployable']=True
    if field=='tolerance': c['xy_tolerance']={'radius_m': 1}
    if field=='unknown_field': c['unexpected_coordinate_transform']={}
    save(folder/'candidate.yaml',c); refresh_manifest(folder)
    with pytest.raises(ValueError): editor.import_candidate(folder/'candidate.yaml')


@pytest.mark.parametrize('bad', ['output','map','sources','extra_output','version','radius'])
def test_manifest_tampering_rejected(editor, v1, bad):
    folder, _ = v1
    m = load(folder/'manifest.yaml')
    if bad=='output': m['outputs']['annotated.png']='0'*64
    if bad=='map': m['map_identity']['yaml_sha256']='0'*64
    if bad=='sources': m['sources'][str(editor.config_path)]='0'*64
    if bad=='extra_output': m['outputs']['../candidate.yaml']='0'*64
    if bad=='version': m['candidate_schema_version']=2
    if bad=='radius': m['xy_tolerance']={'radius_m':.5}
    save(folder/'manifest.yaml',m)
    with pytest.raises(ValueError): editor.import_candidate(folder/'candidate.yaml')


def test_missing_manifest_explicit_and_source_drift_block(editor, v1, tmp_path):
    folder, _ = v1
    (folder/'manifest.yaml').unlink()
    editor.import_candidate(folder/'candidate.yaml')
    assert editor.import_history[-1]['manifest_verified'] is False
    with (folder/'candidate.yaml').open('a') as f: f.write('\n')
    with pytest.raises(ValueError, match='changed'): editor.export(editor.initial, tmp_path/'new')


@pytest.mark.parametrize('yaw', [0,.3,math.pi/2])
def test_png_circle_has_five_native_pixel_radius_and_alpha(editor, tmp_path, yaw):
    # Pose away from labels/footprint edges. Circle pixel radius is 5 at .05 m/px.
    meta = load(editor.map.path); meta.update(resolution=.05, origin=[0,0,yaw]);save(editor.map.path, meta)
    e = Editor(editor.map.path,editor.config_path,editor.route_path)
    state = copy.deepcopy(e.initial)
    state['goals'][0].update(position=pixel_to_map(e.map.meta,20,15),yaw_rad=yaw)
    path = tmp_path/'circles.png';e.render(state,path)
    im = Image.open(path).convert('RGB')
    # 3x export: center=(61.5,46.5), radius=15. Pixels inside are alpha blended.
    inside=im.getpixel((68,48));outside=im.getpixel((80,48))
    assert inside[0] > inside[2] and inside != (254,254,254)
    assert outside == (254,254,254)
    # Left/right outline at 46 and 76 (15 native export pixels each side).
    assert im.getpixel((44,46)) == (254,254,254)
    assert im.getpixel((46,46)) != (254,254,254)
    assert im.getpixel((76,46)) != (254,254,254)


def test_tolerance_not_client_mutable(editor):
    state=copy.deepcopy(editor.initial);state['xy_tolerance']['radius_m']=.5
    with pytest.raises(ValueError, match='tolerance'): editor.validate(state)


def test_persistent_v1_exact_poses_and_identities(tmp_path):
    backup=os.environ.get('B2W_EDITOR_V1_BACKUP')
    if not backup: pytest.skip('Set B2W_EDITOR_V1_BACKUP for preserved user V1 integration')
    b=Path(backup);f=b/'candidate-16nlistv';c=load(f/'candidate.yaml')
    e=Editor(b/'warehouse.yaml',ROOT/'course_navigation/config/nav2.yaml',ROOT/'course_navigation/config/warehouse_route.yaml')
    e.import_candidate(f/'candidate.yaml')
    assert e.initial['goals']==c['goals']
    assert e.initial['goals'][0]['position']==[0.039514056696976496,6.779728935566027]
    assert e.initial['goals'][0]['yaw_rad']==-3.1012105780252917
    out=e.export(e.initial,tmp_path/'exports')
    assert load(out/'candidate.yaml')['goals']==c['goals']


@pytest.mark.parametrize('bad', ['nan_history','date_history','string_history','nan_annotation','numeric_key'])
def test_yaml_only_or_malformed_metadata_fails_before_browser(editor, tmp_path, bad):
    import datetime
    folder=editor.export(editor.initial,tmp_path/'exports');c=load(folder/'candidate.yaml')
    if bad=='nan_history': c['import_history']=[float('nan')]
    if bad=='date_history': c['import_history']=[datetime.date(2026,10,19)]
    if bad=='string_history': c['import_history']=['not an import record']
    if bad=='nan_annotation': c['goals'][0]['note']=float('nan')
    if bad=='numeric_key': c['goals'][0][1]='not a JSON object key'
    save(folder/'candidate.yaml',c);(folder/'manifest.yaml').unlink()
    with pytest.raises(ValueError): editor.import_candidate(folder/'candidate.yaml')


def test_export_stays_separate_from_runtime_and_sources(editor, v1, tmp_path, monkeypatch):
    import run_warehouse_route as runner
    folder, _=v1
    original={p:digest(p) for p in (editor.map.path,editor.map.image_path,editor.config_path,editor.route_path)}
    editor.import_candidate(folder/'candidate.yaml')
    with pytest.raises(ValueError): editor.export(editor.initial,folder/'nested')
    new=editor.export(editor.initial,tmp_path/'exports')
    monkeypatch.setattr(runner,'ROUTE',new/'candidate.yaml')
    with pytest.raises((ValueError,KeyError)): runner.load_route()
    assert {p:digest(p) for p in original} == original


def test_cli_rejects_invalid_candidate_before_binding(editor, v1, tmp_path):
    import subprocess
    folder,c=v1;c['candidate_schema_version']=99;save(folder/'candidate.yaml',c)
    result=subprocess.run(['/usr/bin/python3',str(ROOT/'scripts/edit_warehouse_waypoints.py'),
        '--map',str(editor.map.path),'--candidate',str(folder/'candidate.yaml'),'--port','0'],
        capture_output=True,text=True,timeout=5)
    assert result.returncode==2
    assert 'Unsupported candidate schema' in result.stderr
    assert 'Traceback' not in result.stderr
    assert 'Offline editor:' not in result.stdout
