"""Portal crossing needs sensor identity and full body tail, not two samples."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))


def cert():
    return dict(kind='stage4_sensor_corridor_portal',version=1,session_id='s',sensor_sha256='a'*64,
        registration_sha256='b'*64,map_sha256='c'*64,origin=[0.,0.],tangent=[1.,0.],
        entry_m=0.,exit_m=3.,lateral_bounds_m=[-.6,.6],uncertainty_m=.02,max_yaw_rad=.1)


def test_stationary_and_center_exit_do_not_prove_tail():
    from stage4_route.portal import CorridorPortal
    p=CorridorPortal(cert())
    for i in range(2):p.observe(dict(x=-1.,y=0.,yaw=0.,sim=1.+i*.1,wall=1.+i*.1))
    assert not p.passed
    for i in range(1,42):p.observe(dict(x=-1.+i*.1,y=0.,yaw=0.,sim=1.1+i*.1,wall=1.1+i*.1))
    assert not p.passed  # center beyond exit, tail still inside
    for i in range(42,50):p.observe(dict(x=-1.+i*.1,y=0.,yaw=0.,sim=1.1+i*.1,wall=1.1+i*.1))
    assert p.passed and p.events['tail_exit']['x']>3.75


@pytest.mark.parametrize('fault',['inside_start','turn','boundary','gap','reverse'])
def test_bad_traversal_is_never_certified(fault):
    from stage4_route.portal import CorridorPortal
    p=CorridorPortal(cert());p.observe(dict(x=-1.,y=0.,yaw=0.,sim=1.,wall=1.))
    if fault=='inside_start':p=CorridorPortal(cert());s=dict(x=1.,y=0.,yaw=0.,sim=1.1,wall=1.1)
    else:
        for i in range(1,12):p.observe(dict(x=-1.+i*.1,y=0.,yaw=0.,sim=1.+i*.1,wall=1.+i*.1))
        s=dict(x=.2,y=0.,yaw=0.,sim=2.2,wall=2.2)
        if fault=='turn':s['yaw']=.2
        if fault=='boundary':s['y']=.2
        if fault=='gap':s['sim']=3.
        if fault=='reverse':s['x']=-.1
    p.observe(s);assert p.failure and not p.passed


def test_missing_sensor_bound_certificate_rejected():
    from stage4_route.portal import CorridorPortal
    c=cert();del c['sensor_sha256']
    with pytest.raises((ValueError,KeyError)):CorridorPortal(c)
