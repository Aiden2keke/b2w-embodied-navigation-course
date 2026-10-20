"""Independent segments retain original identities and can never be full passes."""
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from stage4_route.evidence import RouteEvidence


def goals():
    return [dict(name=f'G{i+1}', position=[2.*i, 0.], yaw_rad=0.) for i in range(5)]


def sample(t, x):
    return dict(sim=t, wall=t, x=x, y=0., yaw=0., tf_stamp=t, odom_stamp=t, map_odom_stamp=t,
                slam_map_age=.1, slam_map_wall_age=.1, slam_map_wall_budget=7.5,
                clock_age=.01, pose_age=.01, tf_receipt_age=.01, speed=0., wz=0.,
                command_speed=0., command_wz=0., safety='clear')


@pytest.mark.parametrize('phase', range(5))
def test_each_segment_can_be_assessed_without_qualifying_full_route(phase):
    start = -1. if phase == 0 else 2.*(phase-1)
    target = 2.*phase
    e = RouteEvidence(goals(), start=[start, 0.], count=1, start_index=phase)
    for i in range(41):
        e.observe(sample(1.+i*.1, start+(target-start)*i/40))
    e.observe(sample(5.1, target));e.terminal(4, sim=5.2)
    for i in range(62):e.observe(sample(5.2+i*.1, target))
    result = e.finish(cleanup=True, safety_complete=True)
    assert result['segment_pass']
    assert result['result'] == 'SEGMENT_PASS'
    assert [x['goal'] for x in result['region_events']] == [f'G{phase+1}']
    assert not result['full_route_pass']
    assert not e.finish(cleanup=True, safety_complete=False)['segment_pass']


def test_start_to_g1_still_rejects_future_g2_even_outside_selected_segment():
    e = RouteEvidence(goals(), start=[-1., 0.], count=1)
    e.observe(sample(1., 2.))
    assert e.failure == 'out_of_order_G2'


def test_g1_to_g2_preserves_mandatory_band_even_with_one_selected_goal():
    e = RouteEvidence(goals(), start=[0., 0.], count=1, start_index=1)
    s = sample(1., 1.);s['y'] = .350001
    e.observe(s)
    assert e.failure == 'disallowed_detour'


def test_future_goal_exclusion_remains_active_after_selected_goal_confirmation():
    route = goals();route[1]['position'] = [.6, 0.]
    e = RouteEvidence(route, start=[-1., 0.], count=1)
    e.observe(sample(1., 0.));e.observe(sample(1.1, 0.))
    assert len(e.events) == 1
    for i in range(1, 5):e.observe(sample(1.1+i*.1, i*.1))
    assert e.failure == 'out_of_order_G2'


def test_segment_cannot_start_at_target_or_replace_measured_start():
    with pytest.raises(ValueError, match='preceding'):
        RouteEvidence(goals(), start=[2., 0.], count=1, start_index=1)
    e = RouteEvidence(goals(), start=[0., 0.], count=1, start_index=1)
    e.observe(sample(1., 2.))
    assert e.failure == 'segment_start_pose_mismatch'


def test_nonfinite_start_and_measured_pose_outside_preceding_region_rejected():
    for start in [[float('nan'), 0.], [0., float('inf')], [0.], ['x', 0.]]:
        with pytest.raises(ValueError):RouteEvidence(goals(), start=start, count=1, start_index=1)
    e = RouteEvidence(goals(), start=[.25, 0.], count=1, start_index=1)
    e.observe(sample(1., .275))
    assert e.failure == 'segment_start_outside_preceding_region'
