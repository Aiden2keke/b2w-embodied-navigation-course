from pathlib import Path
import sys
import math
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from warehouse_operational_footprint import CollisionProjection


def test_forward_kinematics_rotates_collision_with_real_joint(tmp_path):
    file=tmp_path/'robot.urdf'
    file.write_text('<robot><link name="base_link"/><link name="leg"><collision><origin xyz="1 0 0"/><geometry><box size="0.4 0.2 0.1"/></geometry></collision></link><joint name="hinge" type="revolute"><parent link="base_link"/><child link="leg"/><origin xyz="0 0 0"/><axis xyz="0 0 1"/><limit lower="-3.2" upper="3.2" velocity="1"/></joint></robot>')
    model=CollisionProjection(file)
    assert model.bounds({'hinge':0.})==pytest.approx([.8,1.2,-.1,.1])
    assert model.bounds({'hinge':math.pi/2})==pytest.approx([-.1,.1,.8,1.2])
    with pytest.raises(ValueError):model.bounds({})
    with pytest.raises(ValueError):model.bounds({'hinge':4.})


def test_missing_wheel_phase_bounds_all_explicit_wheel_angles():
    import yaml
    root=Path(__file__).resolve().parents[1]
    config=yaml.safe_load((root/'course_sim/config/rl_sar.yaml').read_text())['b2w_rl_sar_controller']['ros__parameters']
    q=dict(zip(config['joint_names'][:12],config['default_dof_pos'][:12]))
    model=CollisionProjection(root/'artifacts/recovery/robot.urdf');bound=model.bounds(q)
    for angle in (0.,.7,math.pi/2,math.pi,4.6):
        explicit=dict(q,**{name:angle for name in config['joint_names'][12:]});b=model.bounds(explicit)
        assert b[0]>=bound[0]-1e-10 and b[1]<=bound[1]+1e-10
        assert b[2]>=bound[2]-1e-10 and b[3]<=bound[3]+1e-10
