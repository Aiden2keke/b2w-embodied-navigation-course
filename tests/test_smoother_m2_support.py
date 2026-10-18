"""Structural command support, independent of any route motion capture."""
import copy
import hashlib
from pathlib import Path

import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]


def api():
    from scripts import smoother_m2_support
    return smoother_m2_support


def test_structural_audit_exists():
    assert (ROOT/'scripts/smoother_m2_support.py').is_file()


def test_current_yaml_derived_delta_and_all_ramp_buckets():
    m=api();r=m.audit()
    actual=yaml.safe_load(m.NAV_CONFIG.read_text())['velocity_smoother']['ros__parameters']
    assert r['configuration']['smoothing_frequency']==actual['smoothing_frequency']
    assert r['delta_v_up']==actual['max_accel'][0]/actual['smoothing_frequency']
    assert r['delta_v_down']==abs(actual['max_decel'][0])/actual['smoothing_frequency']
    assert [p['vx'] for p in r['startup']]==[.025,.05,.075,.1,.125]
    assert [p['bucket'] for p in r['startup']]==['LOW']*4+['SUPPORTED']
    assert [p['vx'] for p in r['deceleration']]==[.125,.1,.075,.05,.025,0.]
    assert [p['bucket'] for p in r['deceleration']]==['SUPPORTED']+['LOW']*4+['ZERO']
    assert r['classification']=='LOW_SPEED_SUPPORT_BLOCKER'
    assert r['NORMAL_NAV2_M2_READY'] is False
    assert r['physical_motion_data'] is False
    assert r['source_sha256']==hashlib.sha256(m.NAV_CONFIG.read_bytes()).hexdigest()
    assert r['high_speed_risk']['configured_smoother_max_vx']==.4
    assert r['high_speed_risk']['frozen_route_fraction'] is None


def test_modified_synthetic_yaml_changes_nominal_ramp(tmp_path):
    m=api();cfg=yaml.safe_load(m.NAV_CONFIG.read_text())
    cfg['velocity_smoother']['ros__parameters']['smoothing_frequency']=10.
    cfg['velocity_smoother']['ros__parameters']['max_accel'][0]=.4
    cfg['velocity_smoother']['ros__parameters']['max_decel'][0]=-.3
    p=tmp_path/'nav2.yaml';p.write_text(yaml.safe_dump(cfg));r=m.audit(p)
    assert r['delta_v_up']==.04 and r['delta_v_down']==.03
    assert [v['vx'] for v in r['startup']]==[.04,.08,.12]
    assert [v['vx'] for v in r['deceleration']]==[.12,.09,.06,.03,0.]


@pytest.mark.parametrize('field,value',[('feedback','CLOSED_LOOP'),('smoothing_frequency',0.),('max_accel',[0.,0.,0.]),('deadband_velocity',[.1,0.,0.]),('scale_velocities',True)])
def test_unsupported_configuration_stops_proof(tmp_path,field,value):
    m=api();cfg=yaml.safe_load(m.NAV_CONFIG.read_text());cfg['velocity_smoother']['ros__parameters'][field]=value
    p=tmp_path/'nav.yaml';p.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError):m.audit(p)


def test_historical_missing_evidence_does_not_control_blocker():
    from scripts.audit_route_command_support import audit
    r=audit(paths=[])
    assert r['classification']=='LOW_SPEED_SUPPORT_BLOCKER'
    assert r['NORMAL_NAV2_M2_READY'] is False
    assert r['frozen_route_G1_G2']['raw'] is None
    assert r['frozen_route_G1_G2']['shared'] is None
    assert r['frozen_route_G1_G2']['final'] is None
    assert 'LOW-SPEED TRANSITION' in r['smallest_next_check']


def test_pure_forward_scope_identity_and_deferred_report():
    from course_navigation import m2_inverse
    from scripts.m2_open_space import plan
    assert m2_inverse.identity()['validated_condition']=='pure_forward_vy0_wz0'
    assert m2_inverse.identity()['turning_lateral_invariance_validated'] is False
    assert plan()['validated_condition']=='pure_forward_vy0_wz0'
    assert plan()['deferred_due_to']=='LOW_SPEED_SUPPORT_BLOCKER'
    assert m2_inverse.transform((.15,.1,.2),enabled=True)==(m2_inverse.inverse(.15),.1,.2)
