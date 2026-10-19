"""Run installed Smac Node2D and smoother, without rclcpp::init or ROS/DDS."""
import json
from pathlib import Path
import subprocess
import os
import pytest

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture(scope='module')
def probe(tmp_path_factory):
    existing=os.environ.get('B2W_STAGE4_PROBE')
    if existing:return Path(existing)
    build=tmp_path_factory.mktemp('stage4-smac-build')
    subprocess.run(['cmake','-S',str(ROOT/'scripts/stage4_smac_probe'),'-B',str(build),'-DCMAKE_PREFIX_PATH=/opt/ros/humble','-DPython3_EXECUTABLE=/usr/bin/python3'],check=True,capture_output=True,timeout=30)
    subprocess.run(['cmake','--build',str(build),'-j2'],check=True,capture_output=True,timeout=30)
    return build/'stage4_smac_probe'


@pytest.mark.parametrize('barrier',[None,254,255])
def test_real_search_and_unknown_contract(probe,tmp_path,barrier):
    w=80;grid=bytearray(w*w)
    if barrier is not None:
        for row in range(w):grid[row*w+40]=barrier
    path=tmp_path/'grid.bin';path.write_bytes(grid)
    r=subprocess.run([str(probe),str(path),str(w),str(w),'.05','0','0','.5','2','3.5','2','.75'],capture_output=True,text=True,check=True,timeout=5)
    report=json.loads(r.stdout.splitlines()[-1])
    assert report['found'] is (barrier is None)
    if barrier is None:
        assert len(report['path'])>2 and abs(report['path'][-1][2]-.75)<1e-12
