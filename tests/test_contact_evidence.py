"""Full contact proof needs inventory, fresh empty heartbeats and exact support pairs."""
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
BODY = 'base_link::base_link_collision'
FOOT = 'FR_foot::FR_foot_collision'
GROUND = 'course_world::aws_robomaker_warehouse_GroundB_01_001::link::collision'


def test_empty_contact_heartbeat_and_complete_expected_inventory_are_required():
    from stage4_route.contact_evidence import ContactEvidence
    e = ContactEvidence([BODY, FOOT])
    e.inventory([BODY, FOOT], stamp=1., wall=10.)
    assert not e.complete(sim=1., wall=10.)
    e.contacts([], stamp=1., wall=10.)
    assert e.complete(sim=1.1, wall=10.1)
    assert not e.complete(sim=1.6, wall=10.6)


def test_every_body_contact_fails_except_declared_wheel_ground_support():
    from stage4_route.contact_evidence import ContactEvidence
    for body in [BODY, FOOT]:
        e = ContactEvidence([BODY, FOOT]);e.inventory([BODY, FOOT], stamp=1., wall=10.)
        e.contacts([['b2w::'+body, 'rack::link::collision']], stamp=1., wall=10.)
        assert e.failure == 'forbidden_robot_environment_contact'
    e = ContactEvidence([BODY, FOOT]);e.inventory([BODY, FOOT], stamp=1., wall=10.)
    e.contacts([['b2w::'+FOOT, GROUND]], stamp=1., wall=10.)
    assert e.complete(sim=1.1, wall=10.1)
    e.contacts([['b2w::'+BODY, GROUND]], stamp=1.1, wall=10.1)
    assert e.failure == 'forbidden_robot_environment_contact'


def test_missing_collision_or_duplicate_heartbeat_cannot_renew_coverage():
    from stage4_route.contact_evidence import ContactEvidence
    e = ContactEvidence([BODY, FOOT]);e.inventory([FOOT], stamp=1., wall=10.)
    e.contacts([], stamp=1., wall=10.)
    assert not e.complete(sim=1., wall=10.)
    e = ContactEvidence([BODY, FOOT]);e.inventory([BODY, FOOT], stamp=1., wall=10.)
    e.contacts([], stamp=1., wall=10.)
    e.inventory([BODY, FOOT], stamp=1., wall=10.4)
    e.contacts([], stamp=1., wall=10.4)
    assert not e.complete(sim=1.6, wall=10.6)
