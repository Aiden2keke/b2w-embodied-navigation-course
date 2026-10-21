"""Regression checks: a filled cylinder, shell gap or unsafe spawn must fail."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from course_sim.pipe_environment import PipeConfig, make_pipe_world, clearance


def inside_box(point, collision):
    pose = list(map(float, collision.findtext('pose').split()))
    size = list(map(float, collision.findtext('geometry/box/size').split()))
    x, y, z = [point[i] - pose[i] for i in range(3)]
    a = pose[3]
    local = (x, math.cos(a)*y+math.sin(a)*z, -math.sin(a)*y+math.cos(a)*z)
    return all(abs(v) <= s/2+1e-8 for v, s in zip(local, size))


def test_hollow_shell_has_no_gaps_and_invert_is_continuous():
    cfg = PipeConfig()
    root = ET.fromstring(make_pipe_world(cfg))
    walls = root.findall(".//model[@name='pipe_shell']/link/collision")
    assert len(walls) == cfg.segments
    # Empty bore and complete wall around the whole circumference.
    for i in range(720):
        a = i*math.tau/720
        def point(r): return (0, r*math.cos(a), cfg.axis_z_m+r*math.sin(a))
        assert not any(inside_box(point(cfg.inner_radius_m-.01), c) for c in walls)
        assert any(inside_box(point(cfg.inner_radius_m+.02), c) for c in walls)
    floor = root.find(".//model[@name='invert']/link/collision")
    for x in [-7.99, -4, 0, 4, 7.99]:
        for y in [-.95, 0, .95]:
            assert inside_box((x,y,-.01), floor)
    assert not root.findall('.//uri') and not root.findall('.//include')
    assert root.findtext('.//scene/ambient') == '0.025 0.025 0.03 1'
    assert make_pipe_world(cfg) == make_pipe_world(cfg)


@pytest.mark.parametrize('kwargs', [dict(inner_radius_m=-1), dict(segments=8),
    dict(axis_z_m=2), dict(length_m=0), dict(segments=96.5), dict(ambient=float('nan'))])
def test_invalid_parameters(kwargs):
    with pytest.raises(ValueError): PipeConfig(**kwargs)


def test_real_urdf_spawn_clearance_and_rejection():
    root = Path(__file__).resolve().parents[1]
    urdf = root/'b2w_sim/b2w_description_ros2/urdf/b2w_description_rsl.urdf'
    result = clearance(PipeConfig(), urdf)
    assert result['passed']
    assert result['poses']['zero']['floor_clearance_m'] > .18
    assert result['poses']['zero']['aabb_dimensions_m'][1] > .45
    assert not clearance(PipeConfig(spawn_y_m=1.4), urdf)['passed']
    assert not clearance(PipeConfig(spawn_z_m=.1), urdf)['passed']


def test_cleanup_finds_orphan_and_preserves_unrelated_process(tmp_path):
    import importlib.util
    import os
    import subprocess
    import sys
    import uuid
    spec=importlib.util.spec_from_file_location('pipe_smoke',Path(__file__).resolve().parents[1]/'scripts/smoke_pipe.py')
    smoke=importlib.util.module_from_spec(spec);spec.loader.exec_module(smoke)
    token=uuid.uuid4().hex
    env=os.environ.copy();env['B2W_PIPE_SMOKE_TOKEN']=token
    sentinel=subprocess.Popen([sys.executable,'-c','import time; time.sleep(25)'])
    parent=subprocess.Popen([sys.executable,'-c',
        'import subprocess,sys; subprocess.Popen([sys.executable,"-c","import time; time.sleep(25)"])'],env=env)
    try:
        parent.wait(timeout=3)
        assert smoke.owned_processes(token), 'parent exit must not hide surviving child'
        result=smoke.cleanup(token,parent)
        assert not result['remaining']
        assert sentinel.poll() is None
    finally:
        smoke.cleanup(token,parent)
        sentinel.terminate();sentinel.wait(timeout=3)


def test_plugin_resolution_supports_configured_prefix(tmp_path,monkeypatch):
    import importlib.util
    spec=importlib.util.spec_from_file_location('prepare_pipe',Path(__file__).resolve().parents[1]/'scripts/prepare_pipe.py')
    prepare=importlib.util.module_from_spec(spec);spec.loader.exec_module(prepare)
    # Resource discovery, not a loadability claim. Runtime smoke tests actual loading.
    lib=tmp_path/'libignition-gazebo6-test-fixture-system.so';lib.touch()
    monkeypatch.setenv('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',str(tmp_path))
    assert prepare.resolve_plugin('ignition-gazebo-test-fixture-system')==str(lib)
    assert prepare.resolve_plugin('definitely-absent-test-library') is None
