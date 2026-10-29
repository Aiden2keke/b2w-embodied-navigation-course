"""Immutable unknown-encoding correction; never a geometry or motion certificate."""
import json
from pathlib import Path
import shutil

import numpy as np
import yaml

from .contract import POLICY, digest, object_hash, write_new
from waypoint_editor.core import Map


def _document(source, candidate, directory):
    source, candidate, directory = Path(source).resolve(), Path(candidate).resolve(), Path(directory).resolve()
    policy = json.loads(POLICY.read_text())
    m = Map(source)
    c = yaml.safe_load(candidate.read_text())
    if (digest(candidate) != policy['source_v1_sha256'] or
            digest(source) != policy['map_yaml_sha256'] or
            digest(m.image_path) != policy['map_image_sha256'] or
            object_hash(c['goals']) != policy['goals_sha256']):
        raise ValueError('Unverified parent candidate/map identity')
    meta = dict(m.raw, image='map.pgm', free_thresh=.19)
    values = np.asarray(m.image)
    occ = (255.-values)/255.
    classification = np.where(occ > meta['occupied_thresh'], 100,
                              np.where(occ < meta['free_thresh'], 0, -1))
    expected = np.array([[100 if x=='occupied' else 0 if x=='free' else -1 for x in row] for row in m.cells])
    if meta['negate'] != 0 or not np.array_equal(expected, classification):
        raise ValueError('Correction must preserve every conservative source cell class')
    native_before = np.where(occ > m.raw['occupied_thresh'], 100,
                              np.where(occ < m.raw['free_thresh'], 0, -1))
    changed = np.flatnonzero(np.flipud(native_before != classification).reshape(-1))
    runs = []
    for index in changed.tolist():
        if runs and index == runs[-1][0] + runs[-1][1]:
            runs[-1][1] += 1
        else:
            runs.append([index, 1, 0, -1])
    return meta, dict(kind='navigation_map_derivation', version=1,
        derivation='unknown_encoding_correction', motion_ready=False, sensor_geometry_changed=False,
        frame_id='map', goals_sha256=object_hash(c['goals']), goals=c['goals'],
        parent=dict(map_yaml=str(source), map_yaml_sha256=digest(source), image=str(m.image_path),
                    image_sha256=digest(m.image_path), candidate=str(candidate), candidate_sha256=digest(candidate)),
        output=dict(map_yaml=str(directory/'map.yaml'), map_yaml_sha256=None,
                    image=str(directory/'map.pgm'), image_sha256=digest(m.image_path)),
        changed_native_cells_rle=runs,
        classification_counts=dict(free=int(np.sum(classification==0)), unknown=int(np.sum(classification==-1)),
                                   occupied=int(np.sum(classification==100))),
        sensor_sources=[], registration=None,
        limitations=['Only saved-map encoding corrected; occupied geometry and old raster obstruction unchanged',
                     'No same-session sensor support, path, dynamic envelope or motion qualification'])


def derive_unknown_encoding(source, candidate, directory):
    directory = Path(directory).resolve()
    meta, doc = _document(source, candidate, directory)
    directory.mkdir(parents=True, exist_ok=False)
    shutil.copy2(doc['parent']['image'], directory/'map.pgm')
    (directory/'map.yaml').write_text(yaml.safe_dump(meta, sort_keys=False))
    doc['output']['map_yaml_sha256'] = digest(directory/'map.yaml')
    manifest = directory/'derivation.json'
    write_new(manifest, doc)
    verify_derivation(manifest)
    return manifest


def verify_derivation(path):
    path = Path(path).resolve()
    try:
        actual = json.loads(path.read_text())
        meta, expected = _document(actual['parent']['map_yaml'], actual['parent']['candidate'], path.parent)
        expected['output']['map_yaml_sha256'] = digest(path.parent/'map.yaml')
        if (actual != expected or yaml.safe_load((path.parent/'map.yaml').read_text()) != meta or
                digest(path.parent/'map.pgm') != expected['output']['image_sha256']):
            raise ValueError('Map derivation identity or bytes changed')
        return actual
    except (KeyError, TypeError, OSError) as exc:
        raise ValueError('Invalid map derivation') from exc
