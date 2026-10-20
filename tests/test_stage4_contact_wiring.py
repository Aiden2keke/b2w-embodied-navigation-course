"""Physical reports veto; reporting completeness alone never proves engine coverage."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from types import SimpleNamespace as NS
from test_stage4_transport import transport


def test_transport_forbidden_unpaired_contact_requests_independent_abort(tmp_path):
    from course_navigation.stage4_control import read_control
    from course_navigation.stage4_contact_stream import ContactStream
    t=transport();t.contacts=ContactStream(['body::collision'],first_sequence=1)
    t.sim=lambda:10.
    t.guard_config=dict(session_id='s',inhibit_path=str(tmp_path/'stop'))
    msg=NS(header=NS(frame_id='b2w_evaluation_seq_1',stamp=NS(sec=10,nanosec=0)),
        contacts=[NS(collision1=NS(name='b2w::body::collision'),collision2=NS(name='wall'))])
    t.observe_contact('/b2w/evaluation/full_contacts',msg,20.)
    assert t.failure and read_control(tmp_path/'stop')['inhibited']


def test_contact_closed_is_reporting_only_never_physical_coverage():
    from course_navigation.stage4_contact_stream import ContactStream
    c=ContactStream(['body::collision'],first_sequence=1)
    c.inventory(dict(sequence=1,interval_start=9.,stamp=10.,sampled_steps=200,
        collisions=['body::collision'],ever_forbidden_contact=False,evaluation_only=True),wall=20.)
    c.contact_message(NS(header=NS(frame_id='b2w_evaluation_seq_1',stamp=NS(sec=10,nanosec=0)),contacts=[]),wall=20.)
    assert c.intervals.closed(start=9.,end=10.,sim=10.,wall=20.)
    assert not c.physical_coverage_qualified
    assert not c.intervals.closed(start=9.,end=10.1,sim=10.1,wall=20.1)
