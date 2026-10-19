"""Slow structural map is distinct from fast localization and obstacle safety."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from stage4_route.evidence import RouteEvidence
from test_stage4_route import sample, goals


def test_nominal_five_second_structural_stream_survives_thirty_seconds():
    e=RouteEvidence(goals(),start=[0,0])
    for i in range(1,301):
        t=i*.1;s=sample(t,0.)
        s.update(slam_map_age=t-5*int(t/5),slam_map_wall_age=t-5*int(t/5),slam_map_wall_budget=7.5)
        e.observe(s)
    assert e.failure is None


def test_structural_wall_timeout_does_not_wait_for_sim_clock():
    e=RouteEvidence(goals(),start=[0,0]);s=sample(10.,0.)
    s.update(slam_map_age=1.,slam_map_wall_age=7.6,slam_map_wall_budget=7.5)
    e.observe(s);assert e.failure=='stale_or_missing_localization'


def test_structural_budget_is_calibrated_from_three_publications():
    from stage4_route.freshness import StructuralMapCadence
    c=StructuralMapCadence()
    c.observe(0.,0.,0.);c.observe(5.,5.,6.)
    with pytest.raises(ValueError):c.ages(6.,7.)
    c.observe(10.,10.,12.)
    assert c.ages(12.,14.)==dict(slam_map_age=2.,slam_map_wall_age=2.,slam_map_wall_budget=9.)
    with pytest.raises(ValueError):c.ages(17.6,18.)
    with pytest.raises(ValueError):c.ages(12.,21.1)


def test_duplicate_map_does_not_renew_cadence_and_regression_latches():
    from stage4_route.freshness import StructuralMapCadence
    c=StructuralMapCadence()
    for t in (0.,5.,10.):c.observe(t,t,t)
    c.observe(10.,11.,11.)
    assert c.ages(11.,11.)['slam_map_wall_age']==1.
    with pytest.raises(ValueError):c.observe(9.,11.1,11.1)
    with pytest.raises(ValueError):c.ages(11.2,11.2)


def test_structural_producer_exceeding_hard_wall_limit_rejected():
    from stage4_route.freshness import StructuralMapCadence
    c=StructuralMapCadence();c.observe(0.,0.,0.);c.observe(5.,5.,15.)
    with pytest.raises(ValueError):c.observe(10.,10.,30.)


def test_missing_source_publication_rejected_even_if_wall_stream_is_fast():
    from stage4_route.freshness import StructuralMapCadence
    c=StructuralMapCadence();c.observe(0.,0.,0.)
    with pytest.raises(ValueError):c.observe(10.,10.,5.)
