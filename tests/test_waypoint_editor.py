"""Offline editor contracts; no ROS imports or running simulation."""
import copy
import hashlib
import math
from pathlib import Path
import sys

import pytest
import yaml
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from waypoint_editor.core import (Map, Editor, pixel_to_map, map_to_pixel,
                                  polygon_valid, polygons_overlap, transform_polygon)
ROOT = Path(__file__).resolve().parents[1]

@pytest.fixture
def editor(tmp_path):
    Image.new('L', (40, 30), 254).save(tmp_path/'map.pgm')
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=.1,
        origin=[-2., -1., .3], negate=0, occupied_thresh=.65, free_thresh=.25, mode='trinary')))
    return Editor(tmp_path/'map.yaml', ROOT/'course_navigation/config/nav2.yaml',
                  ROOT/'course_navigation/config/warehouse_route.yaml')

def test_pixel_center_and_inversion():
    meta=dict(width=800,height=800,resolution=.05,origin=[-20,-20,0])
    assert pixel_to_map(meta,0,0) == pytest.approx([-19.975,19.975])
    assert pixel_to_map(meta,799,799) == pytest.approx([19.975,-19.975])
    assert map_to_pixel(meta,-20,-20) == pytest.approx([-.5,799.5])

@pytest.mark.parametrize('yaw',[0,.3,math.pi/2,-2.8])
def test_rotated_origin_roundtrip(yaw):
    meta=dict(width=37,height=19,resolution=.073,origin=[1.4,-2.9,yaw])
    for u,v in [(0,0),(36,18),(9.123456789,2.987654321),(-.5,18.5)]:
        assert map_to_pixel(meta,*pixel_to_map(meta,u,v)) == pytest.approx([u,v],abs=1e-12)

def test_identity_covers_metadata_and_image(editor):
    m=editor.map
    assert m.meta['width']==40 and m.meta['height']==30
    assert m.identity['image_sha256']==hashlib.sha256(m.image_path.read_bytes()).hexdigest()
    m.image_path.write_bytes(m.image_path.read_bytes()+b'\n')
    with pytest.raises(ValueError,match='changed'): editor.assert_sources()

def test_padding_and_rotation(editor):
    for actual, expected in zip(editor.footprint, [( .75,.43),(.75,-.43),(-.75,-.43),(-.75,.43)]):
        assert actual == pytest.approx(expected)
    assert transform_polygon([[1,0],[0,1],[-1,0]],[2,3,math.pi/2])[0] == pytest.approx([2,4])

@pytest.mark.parametrize('p', [[],[[0,0],[1,1],[0,1],[1,0]],[[0,0],[1,0],[2,0]],[[0,0],[1,0],[1,float('nan')]]])
def test_invalid_polygons(p):
    assert not polygon_valid(p)

def test_polygon_overlap_includes_touch_and_containment():
    a=[[0,0],[2,0],[2,2],[0,2]]
    assert polygons_overlap(a,[[.5,.5],[1,.5],[1,1]])
    assert polygons_overlap(a,[[2,0],[3,0],[3,1],[2,1]])
    assert not polygons_overlap(a,[[3,0],[4,0],[4,1]])

def test_filled_footprint_detects_interior_unknown_lethal_and_outside(editor):
    pose=[*pixel_to_map(editor.map.meta,20,15),.3]
    assert editor.check_pose(pose)['clear']
    editor.map.cells[15][20]='unknown'
    assert editor.check_pose(pose)['unknown']>0
    editor.map.cells[15][20]='occupied'
    assert editor.check_pose(pose)['occupied']>0
    assert not editor.check_pose([100,100,0])['clear']

def test_candidate_heading_export_and_no_overwrite(editor,tmp_path):
    state=copy.deepcopy(editor.initial)
    state['goals'][0]['yaw_rad']=-1.234567891234
    folder=editor.export(state,tmp_path/'exports')
    candidate=yaml.safe_load((folder/'candidate.yaml').read_text())
    assert candidate['kind']=='offline_waypoint_candidate'
    assert candidate['goals'][0]['yaw_rad']==state['goals'][0]['yaw_rad']
    q=candidate['goals'][0]['quaternion']
    assert 2*math.atan2(q['z'],q['w'])==pytest.approx(state['goals'][0]['yaw_rad'])
    assert candidate['source_map']==editor.map.identity
    assert candidate['canonical_route_snapshot']['goals'][0]['orientation']=='corridor_heading'
    assert (folder/'annotated.png').exists()
    manifest=yaml.safe_load((folder/'manifest.yaml').read_text())
    assert manifest['outputs']['candidate.yaml']==hashlib.sha256((folder/'candidate.yaml').read_bytes()).hexdigest()
    assert editor.export(state,tmp_path/'exports')!=folder
    with pytest.raises(ValueError): editor.export(state,ROOT/'course_navigation/config')

def test_identity_mismatch_rejected(editor,tmp_path):
    s=copy.deepcopy(editor.initial); s['map_identity']['image_sha256']='0'*64
    with pytest.raises(ValueError,match='identity'): editor.export(s,tmp_path/'exports')

@pytest.mark.parametrize('mutation', ['nan','missing','polygon','marker'])
def test_bad_state_rejected(editor,mutation):
    s=copy.deepcopy(editor.initial)
    if mutation=='nan': s['goals'][0]['yaw_rad']=float('nan')
    if mutation=='missing': s['goals'].pop()
    if mutation=='polygon': s['polygons']=[dict(kind='restricted',points=[[0,0],[1,1]])]
    if mutation=='marker': s['markers']={'G1':[0,0,0]}
    with pytest.raises(ValueError): editor.validate(s)

def test_restricted_annotation_blocks_geometry(editor):
    s=copy.deepcopy(editor.initial)
    p=[*pixel_to_map(editor.map.meta,20,15),0]
    s['goals'][0].update(position=p[:2],yaw_rad=0)
    s['polygons']=[dict(kind='restricted',points=transform_polygon([[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]],p))]
    assert editor.check_state(s)['G1']['restricted']

def test_sweep_is_sampled_and_blocking(editor):
    p=[*pixel_to_map(editor.map.meta,20,15),0]
    r=editor.sweep(p)
    assert r['sample_count']==73 and r['physical_feasibility_proven'] is False

def test_reject_nontrinary_metadata(tmp_path):
    Image.new('L',(2,2),255).save(tmp_path/'m.pgm')
    (tmp_path/'m.yaml').write_text(yaml.safe_dump(dict(image='m.pgm',mode='raw',resolution=1,origin=[0,0,0],negate=0,occupied_thresh=.65,free_thresh=.25)))
    with pytest.raises(ValueError): Map(tmp_path/'m.yaml')

def test_loopback_server_and_safe_routes(editor,tmp_path):
    from waypoint_editor.server import create_server
    import threading
    import urllib.request
    import urllib.error
    import json
    server=create_server(editor,tmp_path/'exports',port=0)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    base=f'http://127.0.0.1:{server.server_port}'
    try:
        bootstrap=json.load(urllib.request.urlopen(base+'/bootstrap'))
        assert bootstrap['initial']==editor.initial
        for path in ('/','/app.mjs','/model.mjs','/map.png'):
            with urllib.request.urlopen(base+path) as response: assert response.status==200
        for path in ('/../../PLAN.md','/unknown'):
            with pytest.raises(urllib.error.HTTPError): urllib.request.urlopen(base+path)
        req=urllib.request.Request(base+'/export',data=json.dumps(dict(state=editor.initial)).encode(),headers={'Content-Type':'application/json'})
        with pytest.raises(urllib.error.HTTPError) as err: urllib.request.urlopen(req)
        assert err.value.code==403
        req.add_header('X-Editor-Token',bootstrap['token'])
        req.add_header('Origin','https://example.com')
        with pytest.raises(urllib.error.HTTPError) as err: urllib.request.urlopen(req)
        assert err.value.code==403
        req.remove_header('Origin')
        result=json.load(urllib.request.urlopen(req))
        assert Path(result['path']).is_relative_to(tmp_path/'exports')
    finally:
        server.shutdown();server.server_close();thread.join(timeout=2)

def test_costmap_import_requires_same_map_identity_and_blocks(editor,tmp_path):
    import json
    m=editor.map.meta
    record=dict(frame_id='map',source_map_image_sha256=editor.map.identity['image_sha256'],
        source_map_yaml_sha256=editor.map.identity['yaml_sha256'],
        width=m['width'],height=m['height'],resolution=m['resolution'],origin=m['origin'],
        encoding='nav2_uint8',data=[0]*(m['width']*m['height']))
    record['data'][15*m['width']+20]=255  # bottom-up: v=14, footprint still covers cell
    path=tmp_path/'local.json';path.write_text(json.dumps(record))
    editor.import_costmap(path,'local')
    pose=[*pixel_to_map(m,20,15),.3]
    check=editor.check_pose(pose)
    assert not check['clear'] and check['costmaps']['local']['unknown']>0
    record['source_map_image_sha256']='wrong';path.write_text(json.dumps(record))
    with pytest.raises(ValueError): editor.import_costmap(path,'global')

def test_rotated_origin_known_result():
    m=dict(width=2,height=3,resolution=2,origin=[10,20,math.pi/2])
    assert pixel_to_map(m,0,0)==pytest.approx([5,21])

def test_yaml_metadata_mutation_blocks_export(editor,tmp_path):
    editor.map.path.write_text(editor.map.path.read_text()+'\n# changed identity\n')
    with pytest.raises(ValueError,match='changed'): editor.export(editor.initial,tmp_path/'exports')
    assert not (tmp_path/'exports').exists()

def test_filled_footprint_checks_boundary_touch(editor):
    cell=[[0,0],[1,0],[1,1],[0,1]]
    assert polygons_overlap(cell,[[1,1],[2,1],[2,2],[1,2]])
    concave=[[0,0],[3,0],[3,1],[1,1],[1,3],[0,3]]
    assert polygon_valid(concave)
    assert not polygons_overlap(concave,[[2,2],[2.5,2],[2.5,2.5]])

def test_exports_preserve_all_sources(editor,tmp_path):
    before={p:Path(p).read_bytes() for p in editor.sources}
    result=editor.export(editor.initial,tmp_path/'exports')
    assert set(p.name for p in result.iterdir())=={'candidate.yaml','annotations.yaml','waypoints.csv','annotated.png','manifest.yaml'}
    assert before=={p:Path(p).read_bytes() for p in before}

@pytest.mark.parametrize('bad', ['yaml','frame','encoding','size','value'])
def test_bad_costmap_is_rejected(editor,tmp_path,bad):
    import json
    r=dict(frame_id='map',source_map_image_sha256=editor.map.identity['image_sha256'],
        source_map_yaml_sha256=editor.map.identity['yaml_sha256'],encoding='nav2_uint8',
        width=1,height=1,resolution=.05,origin=[0,0,0],data=[254])
    if bad=='yaml': r['source_map_yaml_sha256']='wrong'
    if bad=='frame': r['frame_id']='odom'
    if bad=='encoding': r['encoding']='occupancy'
    if bad=='size': r['data']=[]
    if bad=='value': r['data']=[256]
    path=tmp_path/'costmap.json';path.write_text(json.dumps(r))
    with pytest.raises(ValueError): editor.import_costmap(path,'local')

def test_trinary_205_never_silently_becomes_free(editor):
    im=editor.map.image.copy();im.putpixel((20,15),205);im.save(editor.map.image_path)
    loaded=Map(editor.map.path)
    assert loaded.cells[15][20]=='unknown'
    assert '205' in loaded.occupancy_policy
