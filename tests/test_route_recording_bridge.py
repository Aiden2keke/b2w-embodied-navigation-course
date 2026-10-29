from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))

from run_warehouse_route import load_route, planned_goals


def test_recording_heading_contract_overrides_only_g3_through_g5():
    config = load_route()
    goals = planned_goals(config, 'full', [0.0, 0.5, -0.5])
    assert [goal['name'] for goal in goals] == ['G1', 'G2', 'G3', 'G4', 'G5']
    assert [goal['yaw_rad'] for goal in goals[:2]] == pytest.approx([
        planned_goals(config, 'critical')[0]['yaw_rad'],
        planned_goals(config, 'critical')[1]['yaw_rad'],
    ])
    assert [goal['yaw_rad'] for goal in goals[2:]] == [0.0, 0.5, -0.5]
    assert [goal['pose']['pose']['position']['x'] for goal in goals] == [
        point['position'][0] for point in config['goals']
    ]


def test_recording_heading_contract_rejects_unreviewed_or_partial_headings():
    config = load_route()
    with pytest.raises(ValueError):
        planned_goals(config, 'full')
    with pytest.raises(ValueError):
        planned_goals(config, 'full', [0.0, float('nan'), 0.0])
