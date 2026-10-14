"""Offline installed-library contract tests. No rclcpp init/node/DDS."""
from functools import lru_cache
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))


@lru_cache(None)
def binary():
    source=ROOT/'scripts/frozen_obstacle_critic'
    assert (source/'critic.cpp').is_file(), 'missing installed-library critic harness'
    build=Path(tempfile.mkdtemp(prefix='b2w-critic-test-'))
    subprocess.run(['cmake','-S',str(source),'-B',str(build),
                    '-DCMAKE_PREFIX_PATH=/opt/ros/humble'],check=True,capture_output=True)
    subprocess.run(['cmake','--build',str(build),'-j2'],check=True,capture_output=True)
    return build/'frozen_obstacle_critic'


def evaluate(tmp_path, cells, pose=(4.5,4.5,0.), poly=((-2,-2),(2,-2),(2,2),(-2,2))):
    raw=bytearray(100)
    for x,y,cost in cells: raw[y*10+x]=cost
    path=tmp_path/'raw.bin';path.write_bytes(raw)
    values=[10,10,1.,0.,0.,*pose,len(poly),*(v for p in poly for v in p)]
    result=subprocess.run([str(binary()),str(path)],input=' '.join(map(str,values)),
                          text=True,capture_output=True,check=True)
    return json.loads(result.stdout)


@pytest.mark.parametrize('cost,base,foot',[(0,True,True),(252,True,True),
                                         (253,False,True),(254,False,True),(255,False,True)])
def test_center_rejection_and_perimeter_only_interior_semantics(tmp_path,cost,base,foot):
    # An interior lethal/unknown cell is invisible to the perimeter critic.
    r=evaluate(tmp_path,[(4,4,cost)])
    assert r['base_legal'] is base
    assert r['footprint_legal'] is foot


@pytest.mark.parametrize('cost,legal',[(253,True),(254,False),(255,False)])
def test_perimeter_distinguishes_inscribed_lethal_unknown_and_reports_first(tmp_path,cost,legal):
    r=evaluate(tmp_path,[(3,2,cost),(5,2,254)])
    assert r['base_legal'] is True
    assert r['footprint_legal'] is False
    assert r['first_illegal_cell']==([5,2,254] if legal else [3,2,cost])


def test_closing_edge_is_checked_and_outside_vertices_reject(tmp_path):
    r=evaluate(tmp_path,[(2,4,254)])
    assert r['first_illegal_cell']==[2,4,254]
    r=evaluate(tmp_path,[],pose=(.5,4.5,0.))
    assert r['base_legal'] is True
    assert r['footprint_legal'] is False
    assert r['first_illegal_cell'] is None


def test_short_raw_input_is_rejected(tmp_path):
    path=tmp_path/'short.bin';path.write_bytes(b'\0')
    result=subprocess.run([str(binary()),str(path)],input='10 10 1 0 0 4 4 0 3 0 0 1 0 0 1',
                          text=True,capture_output=True)
    assert result.returncode!=0


@pytest.mark.parametrize('cost,legal',[(253,True),(254,False),(255,False)])
def test_single_perimeter_cell_uses_installed_point_cost(tmp_path,cost,legal):
    r=evaluate(tmp_path,[(3,2,cost)])
    assert r['footprint_legal'] is legal
    assert r['first_illegal_cell']==(None if legal else [3,2,cost])


def test_yaw_transform_and_center_outside_are_applied(tmp_path):
    import math
    r=evaluate(tmp_path,[(5,3,254)],pose=(4.5,4.5,math.pi/2),
               poly=((-2,-1),(2,-1),(2,1),(-2,1)))
    assert r['base_legal'] is True
    assert r['first_illegal_cell']==[5,3,254]
    r=evaluate(tmp_path,[],pose=(-.1,4.5,0))
    assert r['base_legal'] is False
    assert r['footprint_legal'] is False


def test_replay_refuses_output_inside_original_evidence(tmp_path):
    import diagnose_frozen_warehouse as diagnostic
    source=tmp_path/'source'; source.mkdir()
    with pytest.raises(ValueError,match='original evidence'):
        diagnostic.analyze(source,binary(),source/'new-analysis')
    assert not (source/'new-analysis').exists()
