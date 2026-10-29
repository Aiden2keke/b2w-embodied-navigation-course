from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from test_stage4_route import sample
from stage4_route.stopping import IndependentStop


def test_observed_stop_does_not_require_action_terminal():
    stop=IndependentStop(dict(sim=10.,wall=10.))
    for i in range(62):stop.observe(sample(10.+i*.1,0.))
    assert stop.report()['confirmed'] and stop.report()['action_independent']
    s=sample(16.2,.04);stop.observe(s)
    assert not stop.report()['confirmed']


def test_pre_inhibit_pose_or_duplicates_cannot_establish_stop():
    stop=IndependentStop(dict(sim=10.,wall=10.))
    for i in range(62):
        s=sample(10.+i*.1,0.);s['tf_stamp']=9.9;stop.observe(s)
    assert not stop.confirmed
