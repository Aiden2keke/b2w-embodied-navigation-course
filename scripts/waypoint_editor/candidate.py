"""Fail-closed saved-candidate loading; paths are provenance, never registration."""
import copy
import math
import json
import re
from pathlib import Path

import yaml

XY_TOLERANCE = dict(radius_m=.25, metric='map_frame_euclidean_xy',
                    reference='robot_base_center', region_safety_checked=False,
                    runtime_action_success=False)
V1_FIELDS = {'kind','candidate_schema_version','frame_id','deployable','occupancy_policy',
             'source_map','source_hashes','costmap_metadata','canonical_route_snapshot',
             'goals','markers','polygons','geometry','padded_footprint','limitations'}
V2_FIELDS = V1_FIELDS | {'xy_tolerance','configuration_identity','import_history'}
OUTPUTS = {'candidate.yaml','annotations.yaml','annotated.png','waypoints.csv'}


def read_yaml(path):
    if path.stat().st_size > 1_000_000:
        raise ValueError('Candidate/manifest exceeds 1 MB input bound')
    try:
        result = yaml.safe_load(path.read_text())
    except yaml.YAMLError as error:
        raise ValueError('Malformed candidate/manifest YAML') from error
    if not isinstance(result, dict):
        raise ValueError('Candidate/manifest must be a mapping')
    try:
        if json.loads(json.dumps(result, allow_nan=False)) != result:
            raise ValueError('Require string JSON object keys')
    except (TypeError, ValueError, RecursionError) as error:
        raise ValueError('Candidate/manifest metadata must contain finite JSON values and string keys') from error
    return result


def same_map(record, current):
    # Absolute paths may change; byte hashes AND complete parsed metadata must agree.
    return (isinstance(record, dict) and
            all(record.get(k) == current[k] for k in ('yaml_sha256','image_sha256','metadata')))


def load_candidate(editor, path, sha):
    path = Path(path).resolve()
    c = read_yaml(path)
    version = c.get('candidate_schema_version')
    if type(version) is not int or version not in (1, 2):
        raise ValueError('Unsupported candidate schema; require version 1 or 2')
    if set(c) != (V1_FIELDS if version == 1 else V2_FIELDS):
        raise ValueError('Incompatible candidate schema fields')
    if c['kind'] != 'offline_waypoint_candidate' or c['frame_id'] != 'map' or c['deployable'] is not False:
        raise ValueError('Require non-deployable offline map-frame candidate')
    if not same_map(c['source_map'], editor.map.identity):
        raise ValueError('Candidate map hash/metadata mismatch; no SLAM coordinate conversion is supported')
    if (c['canonical_route_snapshot'] != editor.route or c['padded_footprint'] != editor.footprint
            or c['occupancy_policy'] != editor.map.occupancy_policy):
        raise ValueError('Candidate route, footprint or occupancy policy mismatch')
    if c['costmap_metadata'] != {name: cm.meta for name, cm in editor.costmaps.items()}:
        raise ValueError('Candidate costmaps mismatch; supply the same verified costmaps at launch')
    # Compare hashes by source role, allowing relocation without trusting recorded paths.
    recorded = c['source_hashes']
    old_map = c['source_map']
    if not isinstance(recorded, dict) or any(recorded.get(old_map[k+'_path']) != old_map[k+'_sha256'] for k in ('yaml','image')):
        raise ValueError('Inconsistent candidate map source hashes')
    old_extra = {p: h for p, h in recorded.items() if p not in (old_map['yaml_path'], old_map['image_path'])}
    current_extra = {p: h for p, h in editor.sources.items() if p not in (str(editor.map.path), str(editor.map.image_path))}
    # V1 has no named configuration identity. Require an unambiguous basename/hash
    # match per role, and account for every source; never read old source paths.
    for p, h in current_extra.items():
        matches = [q for q, digest in old_extra.items() if Path(q).name == Path(p).name and digest == h]
        if len(matches) != 1:
            raise ValueError('Candidate configuration/source identity mismatch: '+Path(p).name)
        del old_extra[matches[0]]
    if old_extra:
        raise ValueError('Candidate contains unmatched source identities')
    if version == 2 and (c['xy_tolerance'] != XY_TOLERANCE or
                         c['configuration_identity'] != editor.configuration_identity or
                         not isinstance(c['import_history'], list)):
        raise ValueError('Incompatible tolerance/configuration/history metadata')
    for record in c.get('import_history', []):
        fields = {'candidate_path','candidate_sha256','original_source_map','manifest_verified','manifest_sha256','migration'}
        if (not isinstance(record, dict) or set(record) != fields or
                not isinstance(record['candidate_path'], str) or
                not isinstance(record['candidate_sha256'], str) or
                not re.fullmatch('[0-9a-f]{64}', record['candidate_sha256']) or
                not same_map(record['original_source_map'], editor.map.identity) or
                type(record['manifest_verified']) is not bool or
                (record['manifest_verified'] and (not isinstance(record['manifest_sha256'], str) or
                 not re.fullmatch('[0-9a-f]{64}', record['manifest_sha256']))) or
                (not record['manifest_verified'] and record['manifest_sha256'] is not None) or
                record['migration'] not in (None, dict(from_version=1,to_version=2,default_xy_radius_m=.25,poses_unchanged=True))):
            raise ValueError('Malformed candidate import history record')
    manifest_path = path.with_name('manifest.yaml')
    verified = {str(path): sha(path)}
    manifest_hash = None
    if manifest_path.exists() or manifest_path.is_symlink():
        m = read_yaml(manifest_path)
        if (m.get('map_identity') != old_map or m.get('sources') != recorded or
                m.get('occupancy_policy') != c['occupancy_policy'] or m.get('limitations') != c['limitations']):
            raise ValueError('Candidate export manifest identity mismatch')
        expected_fields = {'occupancy_policy','sources','map_identity','outputs','limitations'}
        if version == 2:
            expected_fields |= {'candidate_schema_version','xy_tolerance','configuration_identity','import_history'}
            if any(m.get(k) != c[k] for k in ('candidate_schema_version','xy_tolerance','configuration_identity','import_history')):
                raise ValueError('Candidate export manifest schema/tolerance mismatch')
        if set(m) != expected_fields or not isinstance(m['outputs'], dict) or set(m['outputs']) != OUTPUTS:
            raise ValueError('Incompatible export manifest schema/outputs')
        for name, digest in m['outputs'].items():
            output = path.parent/name
            if not output.is_file() or sha(output) != digest:
                raise ValueError('Candidate export manifest output hash mismatch: '+name)
            verified[str(output.resolve())] = digest
        # The manifest's candidate hash must cover the selected file, even if renamed.
        if m['outputs']['candidate.yaml'] != sha(path):
            raise ValueError('Selected candidate does not match export manifest')
        manifest_hash = sha(manifest_path)
        verified[str(manifest_path)] = manifest_hash
    state = dict(map_identity=copy.deepcopy(editor.map.identity),
                 goals=copy.deepcopy(c['goals']), markers=copy.deepcopy(c['markers']),
                 polygons=copy.deepcopy(c['polygons']), xy_tolerance=copy.deepcopy(XY_TOLERANCE))
    editor.validate(state)
    for g in state['goals']:
        q = g.get('quaternion')
        a = g['yaw_rad']
        expected = dict(x=0., y=0., z=math.sin(a/2), w=math.cos(a/2))
        if not isinstance(q, dict) or set(q) != set(expected) or any(
                type(q[k]) not in (int,float) or not math.isfinite(q[k]) or abs(q[k]-v)>1e-12
                for k,v in expected.items()):
            raise ValueError('Candidate quaternion/yaw mismatch')
    history = copy.deepcopy(c.get('import_history', []))
    history.append(dict(candidate_path=str(path), candidate_sha256=sha(path),
                        original_source_map=copy.deepcopy(old_map),
                        manifest_verified=manifest_hash is not None, manifest_sha256=manifest_hash,
                        migration=(dict(from_version=1,to_version=2,default_xy_radius_m=.25,
                                        poses_unchanged=True) if version == 1 else None)))
    return state, history, verified
