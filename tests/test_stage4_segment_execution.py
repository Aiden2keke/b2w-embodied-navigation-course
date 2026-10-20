"""Execution selection and shared route constraints; no live transport construction."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from test_stage4_route import goals,sample


@pytest.mark.parametrize('phase',range(5))
def test_runner_submits_only_requested_original_segment(phase):
    from stage4_route.runner import run_route
    class Fake:
        audit={};failure=None
        def ready(self,*a):return sample(1.,-1. if phase==0 else 2.*(phase-1))
        def send(self,g,e):self.sent=g
        def tick(self):return dict(status=6)
        def sim(self):return 1.
        def close(self):return True
    t=Fake();r=run_route(dict(goals=goals(),execution={}),dict(session_id='x'),t,count=1,start_index=phase)
    assert t.sent==[goals()[phase]] and t.segment_start_index==phase
    assert not r['full_route_pass'] and r['original_goal_start_index']==phase


def test_shared_segment_policy_keeps_corridor_and_changes_only_g3_g4():
    from stage4_route.constraints import segment_band,deadline_for
    assert [segment_band(i) for i in range(5)]==[3.,.35,3.,5.25,3.]
    assert deadline_for(3.,measured_speed=.1,rotation_rad=0.,measured_yaw_rate=.1,mode='segment')==dict(sim_s=65.,wall_s=117.5)
    with pytest.raises(ValueError):deadline_for(20.,measured_speed=.01,rotation_rad=0.,measured_yaw_rate=.1,mode='segment')
    with pytest.raises(ValueError):deadline_for(1.,measured_speed=0.,rotation_rad=0.,measured_yaw_rate=.1,mode='segment')


def test_preflight_uses_original_phase_and_excludes_future_unselected_goals():
    from stage4_route.spatial import assess_path
    g=goals();grid=dict(width=300,height=200,resolution=.1,origin=[-10.,-10.,0.],data=[0]*60000)
    # G3->G4 ordinary detour remains permitted within its reviewed topological band.
    path=[[4.,0.,0.],[4.,4.,0.],[6.,4.,0.],[6.,0.,0.]]
    result=assess_path(path,[g[3]],[4.,0.],[grid],start_index=3,all_goals=g)
    assert result['passed']
    g[4]['position']=[5.,4.]
    assert not assess_path(path,[g[3]],[4.,0.],[grid],start_index=3,all_goals=g)['passed']
