"""Derived metadata fixes unknown encoding while preserving every geometry cell."""
import json
from pathlib import Path
import sys

from PIL import Image
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
MAP = Path('/home/zyd/b2w-course-evidence/waypoints/20261019-v1/warehouse.yaml')
CANDIDATE = MAP.parent / 'candidate-16nlistv/candidate.yaml'
pytestmark = pytest.mark.skipif(not MAP.is_file() or not CANDIDATE.is_file(),
                                reason='Preserved, authenticated campaign V1 fixture unavailable')


def test_unknown_encoding_correction_preserves_image_and_goals(tmp_path):
    from stage4_route.map_derivation import derive_unknown_encoding, verify_derivation
    result = derive_unknown_encoding(MAP, CANDIDATE, tmp_path / 'derived')
    manifest = verify_derivation(result)
    assert manifest['motion_ready'] is False
    assert manifest['sensor_geometry_changed'] is False
    assert manifest['classification_counts'] == {'free': 98551, 'unknown': 525643, 'occupied': 15806}
    assert manifest['output']['image_sha256'] == manifest['parent']['image_sha256']
    cfg = yaml.safe_load((result.parent / 'map.yaml').read_text())
    assert cfg['free_thresh'] == .19
    assert cfg['origin'] == [-20, -20, 0]
    assert cfg['resolution'] == .05
    assert sum(r[1] for r in manifest['changed_native_cells_rle']) == 525643


def test_changed_geometry_or_goal_provenance_rejected(tmp_path):
    from stage4_route.map_derivation import derive_unknown_encoding, verify_derivation
    result = derive_unknown_encoding(MAP, CANDIDATE, tmp_path / 'derived')
    im = Image.open(result.parent / 'map.pgm')
    im.putpixel((400, 400), 254 if im.getpixel((400, 400)) == 0 else 0)
    im.save(result.parent / 'map.pgm')
    with pytest.raises(ValueError, match='derivation'):
        verify_derivation(result)
    changed = tmp_path / 'candidate.yaml'
    cfg = yaml.safe_load(CANDIDATE.read_text()); cfg['goals'][0]['position'][0] += .01
    changed.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError, match='parent'):
        derive_unknown_encoding(MAP, changed, tmp_path / 'other')


def test_derived_geometry_manifest_cannot_be_loaded_as_execution_selection(tmp_path):
    from stage4_route.map_derivation import derive_unknown_encoding
    from stage4_route.contract import load_selection
    result = derive_unknown_encoding(MAP, CANDIDATE, tmp_path / 'derived')
    with pytest.raises(ValueError, match='execution selection'):
        load_selection(result)
