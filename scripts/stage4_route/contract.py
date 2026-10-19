"""Immutable candidate selection and same-session identity-alignment contract."""
import copy
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np
import yaml
from waypoint_editor.core import Editor
from waypoint_editor.candidate import read_yaml

ROOT=Path(__file__).resolve().parents[2]
POLICY=ROOT/'course_navigation/config/stage4_selection_policy.json'
BASE=ROOT/'course_navigation/config/nav2.yaml'
RUNTIME=ROOT/'course_navigation/config/nav2_smac2d_diag.yaml'
ROUTE=ROOT/'course_navigation/config/warehouse_route.yaml'
EXECUTION=dict(planner_action='/compute_path_through_poses',controller_action='/follow_path',planner_id='GridBased',controller_id='FollowPath',goal_checker_id='general_goal_checker',replan=False,recovery=False)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def object_hash(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def write_new(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)


def verified(candidate,map_path):
    p=json.loads(POLICY.read_text());c=read_yaml(Path(candidate))
    e=Editor(map_path,BASE,ROUTE);e.import_candidate(candidate)
    if not e.import_history[-1]['manifest_verified']:
        raise ValueError('Execution requires complete verified candidate manifest')
    if (object_hash(c['goals'])!=p['goals_sha256'] or
        e.map.identity['yaml_sha256']!=p['map_yaml_sha256'] or
        e.map.identity['image_sha256']!=p['map_image_sha256'] or
        digest(BASE)!=p['baseline_sha256'] or digest(ROUTE)!=p['canonical_route_sha256'] or
        digest(RUNTIME)!=p['smac_config_sha256']):
        raise ValueError('Selected candidate/map/configuration identity mismatch')
    v=c['candidate_schema_version']
    if v==1 and digest(candidate)!=p['source_v1_sha256']:
        raise ValueError('Not the selected V1')
    if v==2 and not any(h['candidate_sha256']==p['source_v1_sha256'] and h['manifest_verified'] and
                           h['migration']==dict(from_version=1,to_version=2,default_xy_radius_m=.25,poses_unchanged=True)
                           for h in c['import_history']):
        raise ValueError('V2 lacks selected V1 verified migration provenance')
    return e,c,p


def selection_document(candidate,map_path):
    e,c,p=verified(candidate,map_path)
    if c['candidate_schema_version']!=2:raise ValueError('Execution input requires schema V2')
    return dict(kind='stage4_explicit_execution_selection',version=1,
        authorization_scope='offline_preparation_only_live_run_requires_separate_user_authorization',
        candidate=str(Path(candidate).resolve()),candidate_sha256=digest(candidate),
        manifest_sha256=digest(Path(candidate).with_name('manifest.yaml')),
        policy_sha256=digest(POLICY),source_v1_sha256=p['source_v1_sha256'],
        source_map=e.map.identity,configuration_identity=c['configuration_identity'],
        runtime_config=str(RUNTIME),runtime_config_sha256=digest(RUNTIME),
        execution=EXECUTION,source_code_sha256={str(p.relative_to(ROOT)):digest(p) for p in sorted((ROOT/'scripts/stage4_route').glob('*.py'))},frame_id='map',radius_m=.25,
        goals=copy.deepcopy(c['goals']),
        constraints=dict(narrow_center_band_m=.35,other_segment_band_m=3.0,
                         basis='acceptance restriction derived from ordered candidate centerlines; not measured physical corridor width'),
        candidate_deployable=False,integrity='SHA256 content binding; no digital signature or production approval')


def select_candidate(candidate,map_path,output):
    e,c,_=verified(candidate,map_path)
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    if c['candidate_schema_version']==1:
        candidate=e.export(e.initial,output/'v2')/'candidate.yaml'
    s=selection_document(candidate,map_path)
    path=output/'selection.json';write_new(path,s);return path


def load_selection(path):
    s=json.loads(Path(path).read_text())
    if s!=selection_document(s['candidate'],s['source_map']['yaml_path']):
        raise ValueError('Execution selection differs from authenticated source adaptation')
    return s


def registration(capture,pairs,*,alignment_p95):
    """Landmarks verify identity coordinates; never apply a guessed map transform."""
    a=np.asarray(pairs['saved'],dtype=float);b=np.asarray(pairs['live'],dtype=float)
    if (a.shape!=b.shape or a.ndim!=2 or a.shape[1]!=2 or not 4<=len(a)<=32 or
        not np.isfinite(a).all() or not np.isfinite(b).all()):raise ValueError('Require 4..32 finite landmark pairs')
    if np.linalg.svd(a-a.mean(0),compute_uv=False)[-1]<2 or np.linalg.svd(b-b.mean(0),compute_uv=False)[-1]<2:
        raise ValueError('Landmarks must span distinct non-collinear structures over several metres')
    u,_,vt=np.linalg.svd((b-b.mean(0)).T@(a-a.mean(0)))
    rot=vt.T@u.T
    if np.linalg.det(rot)<0:vt[-1]*=-1;rot=vt.T@u.T
    offset=a.mean(0)-rot@b.mean(0)
    residual=np.linalg.norm(b@rot.T+offset-a,axis=1)
    yaw=math.atan2(rot[1,0],rot[0,0])
    if (np.linalg.norm(offset)>.05 or abs(yaw)>.02 or max(residual)>.10 or
        np.sqrt(np.mean(residual**2))>.05 or not math.isfinite(alignment_p95) or alignment_p95>.10):
        raise ValueError('Map registration uncertain or non-identity: do not execute saved coordinates')
    return dict(kind='stage4_same_session_registration',capture=copy.deepcopy(capture),pairs=copy.deepcopy(pairs),
        fit=dict(xy_offset_m=offset.tolist(),yaw_rad=yaw,max_residual_m=float(max(residual))),
        occupied_alignment_p95_m=alignment_p95,
        uncertainty='Operator landmark correspondence plus occupied-grid agreement; not global relocalization. Reject non-identity; no TF is published.')


def check_registration(cert,current,*,wall,sim):
    expected=registration(cert['capture'],cert['pairs'],alignment_p95=cert['occupied_alignment_p95_m'])
    if cert!=expected:raise ValueError('Registration artifact drift')
    old=cert['capture']
    for k in ('session_id','graph','selection_sha256','live_map_sha256'):
        if old[k]!=current[k]:raise ValueError('Cross-session or changed registration identity: '+k)
    if not 0<=wall-old['wall']<=120 or not 0<=sim-old['sim']<=120:
        raise ValueError('Registration expired or clock reset')
