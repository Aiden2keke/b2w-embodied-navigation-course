"""The physical pose-to-Path connection must be in the certificate."""
from pathlib import Path
import math
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from stage4_route.spatial import assess_path


def grid():
    return dict(width=160, height=160, resolution=.025, origin=[-2., -2., 0.], data=[0]*25600)


def test_obstacle_between_actual_start_and_clear_first_path_pose_blocks():
    g = grid();g['data'][80*160+80] = 254
    goals = [dict(name='G1', position=[1., 0.], yaw_rad=0.)]
    r = assess_path([[1., 0., 0.]], goals, [-1., 0.], [g], start_pose=[-1., 0., 0.])
    assert not r['passed'] and not r['continuous_clearance_proven']


def test_initial_rotation_is_checked_and_clear_actual_connection_is_recorded():
    g = grid();g['data'][104*160+104] = 254
    goals = [dict(name='G1', position=[0., 0.], yaw_rad=0.)]
    r = assess_path([[0., 0., 0.]], goals, [0., 0.], [g], start_pose=[0., 0., math.pi/2])
    assert not r['passed']
    r = assess_path([[0., 0., 0.]], goals, [0., 0.], [grid()], start_pose=[0., 0., math.pi/2])
    assert r['passed'] and r['initial_connection_checked']
