"""Offline selection equivalence and per-callback freshness guard regressions."""
from collections import deque
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import warehouse_ab_trial as trial


def reference(history, clock, wall, sim_limit, wall_limit):
    valid=[x for x in history if clock is not None
        and all(type(v) in (int,float) and math.isfinite(v) for v in (clock,x[0],x[1],wall))
        and 0 <= clock-x[0] <= sim_limit
        and 0 <= wall-x[1] and (wall_limit is None or wall-x[1] <= wall_limit)]
    return max(valid,key=lambda x:(x[0],x[1]),default=None)


def test_selection_keeps_unordered_ties_future_and_exact_boundaries():
    rng=random.Random(42)
    for _ in range(150):
        rows=[(rng.choice([17.,17.5,18.,18.01,float('nan'),float('inf'),None]),
               rng.choice([99.,99.5,100.,100.01,float('nan'),None]),i) for i in range(128)]
        m=trial.Monitor.__new__(trial.Monitor);m.clock=18.;m.stamp_history={'test':deque(rows,maxlen=128)}
        for limit in (.5,None):
            expected=reference(rows,m.clock,100.,.5,limit)
            status,selected=m.stream_status('test',100.,.5,limit)
            assert selected is expected and status['passed']==(expected is not None)
            assert status['buffered_samples']==128


def test_shared_decision_clock_and_wall_validated_once(monkeypatch):
    m=trial.Monitor.__new__(trial.Monitor);m.clock=18.123456
    m.stamp_history={'test':deque((17.8+i*.0001,99.9,i) for i in range(128))}
    seen=[];original=math.isfinite
    def finite(value):
        seen.append(value)
        return original(value)
    monkeypatch.setattr(trial.math,'isfinite',finite)
    assert m.stream_status('test',100.123456,.5,.5)[0]['passed']
    assert seen.count(m.clock)==1 and seen.count(100.123456)==1


@pytest.mark.parametrize('clock,wall', [(None,100.),(float('nan'),100.),(18.,float('inf')),(True,100.)])
def test_invalid_decision_time_fails_closed(clock,wall):
    m=trial.Monitor.__new__(trial.Monitor);m.clock=clock;m.stamp_history={'test':[(18.,100.,None)]}
    assert not m.stream_status('test',wall,.5,.5)[0]['passed']
