import sys
from pathlib import Path
from PIL import Image
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import warehouse_manual_logger as logger
import warehouse_truth_floorplan as floor


def test_fixed_map_uses_canonical_world_registration(tmp_path):
    meta, base = floor.load_floorplan()
    output = tmp_path / 'trajectory.png'
    logger.render_fixed_track(output, [], meta, base)
    rendered = np.asarray(Image.open(output).convert('RGB'))
    assert rendered.shape[0] > meta['height'] and rendered.shape[1] > meta['width']
    assert np.count_nonzero(np.all(rendered == (91, 111, 132), axis=2)) > 100
    row = dict(x=0., y=0., timestamp_ns=1, segment=0, epoch=0)
    logger.render_fixed_track(output, [row], meta, base)
    col, line = floor.world_to_pixel([[0., 0.]], meta)[0].round().astype(int)
    assert not np.array_equal(np.asarray(Image.open(output).convert('RGB'))[line, col], base[line, col])


def test_fixed_map_rejects_wrong_world(tmp_path):
    world = tmp_path / 'world.sdf'
    world.write_text('<sdf><world name="other"/></sdf>')
    with pytest.raises(ValueError, match='world'):
        logger.load_fixed_map(world)
