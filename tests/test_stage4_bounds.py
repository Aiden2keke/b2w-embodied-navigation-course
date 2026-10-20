"""Explicit conservative component accounting; schema checks are not physics."""
from pathlib import Path
import sys
import pytest
import math
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))


def evidence():
    return dict(kind='stage4_conservative_bounds',version=1,session_id='s',
        components=dict(registration_translation_m=.01,registration_yaw_rad=.01,registration_radius_m=2.,
            map_boundary_m=.01,tracking_translation_m=.01,tracking_yaw_rad=.01,body_extent_m=.01),
        measured=dict(stop_translation_m=.1,stop_yaw_rad=.02,abort_latency_s=.3,max_speed=.2,max_wz=.3),
        sensor_only_registration=True,worst_case_bound=True,scope=dict(valid_from_sim=1.,valid_until_sim=20.,
            region=[-2.,-2.,2.,2.]),raw_evidence=[dict(path='/tmp/measurement.sqlite3',sha256='a'*64)],
        unresolved=['engine coverage','external review'])


def test_uncertainty_combines_translation_yaw_map_body_tracking():
    from course_navigation.stage4_bounds import validate_bounds
    e=evidence();config=dict(session_id='s',stop_translation_m=.12,stop_yaw_rad=.03,abort_latency_s=.6,
        max_speed=.2,max_wz=.3,uncertainty_m=.1)
    result=validate_bounds(config,e)
    assert result['minimum_uncertainty_m']==pytest.approx(.06+math.hypot(.75,.43)*.01)
    assert not result['motion_qualified']
    config['uncertainty_m']=.06
    with pytest.raises(ValueError):validate_bounds(config,e)


@pytest.mark.parametrize('fault',['p95','missing','stop','clock'])
def test_p95_or_missing_bounds_cannot_become_conservative_allowance(fault):
    from course_navigation.stage4_bounds import validate_bounds
    e=evidence();c=dict(session_id='s',stop_translation_m=.12,stop_yaw_rad=.03,abort_latency_s=.6,
        max_speed=.2,max_wz=.3,uncertainty_m=.1)
    if fault=='p95':e['worst_case_bound']=False
    if fault=='missing':del e['components']['body_extent_m']
    if fault=='stop':c['stop_translation_m']=.09
    if fault=='clock':e['scope']['valid_until_sim']=0.
    with pytest.raises((ValueError,KeyError)):validate_bounds(c,e)
