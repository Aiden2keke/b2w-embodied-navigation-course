"""Approved footprint-gate config contract; no ROS node or DDS."""
from copy import deepcopy
from hashlib import sha256
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'course_navigation/config'
SOURCE = CONFIG / 'nav2_smac2d_diag.yaml'
CANDIDATE = CONFIG / 'nav2_smac2d_footprint_gate_diag.yaml'
HASHES = {
    'nav2.yaml': 'd63852ab2eed9c6cc1c258aaf2bdc2b11753ff9d9969e1885785d3cab2a87046',
    'nav2_smac2d_diag.yaml': '97e4eb2bcf58faa3f3a19728b4d17437b9dfe615d07964902e24bb5057a98e9e',
}

def load(path):
    return yaml.safe_load(path.read_bytes())

def follow(doc):
    return doc['controller_server']['ros__parameters']['FollowPath']

@pytest.mark.parametrize('name,expected', HASHES.items())
def test_protected_config_hash(name, expected):
    assert sha256((CONFIG / name).read_bytes()).hexdigest() == expected

def test_exact_two_line_insertion_and_all_other_settings():
    original = SOURCE.read_bytes()
    assert original.count(b'      - BaseObstacle\n') == 1
    assert original.count(b'      BaseObstacle.scale: 0.02\n') == 1
    expected = original.replace(
        b'      - BaseObstacle\n',
        b'      - BaseObstacle\n      - ObstacleFootprint\n', 1).replace(
        b'      BaseObstacle.scale: 0.02\n',
        b'      BaseObstacle.scale: 0.02\n      ObstacleFootprint.scale: 0.000001\n', 1)
    assert CANDIDATE.read_bytes() == expected
    source, candidate = load(SOURCE), load(CANDIDATE)
    before, after = follow(source), follow(candidate)
    assert after['critics'].count('BaseObstacle') == 1
    assert after['critics'].count('ObstacleFootprint') == 1
    idx = before['critics'].index('BaseObstacle') + 1
    assert after['critics'] == before['critics'][:idx] + ['ObstacleFootprint'] + before['critics'][idx:]
    restored = deepcopy(candidate)
    follow(restored)['critics'].remove('ObstacleFootprint')
    assert follow(restored).pop('ObstacleFootprint.scale') == 1e-6
    assert restored == source
    # Exact bytes plus restored-tree equality preserve all old scales/defaults,
    # vx/vy/reverse/accel/decel/frequency/tolerance, both costmaps and smoother.
    for key in ('local_costmap', 'global_costmap', 'planner_server', 'velocity_smoother'):
        assert candidate[key] == source[key]
    planner = source['planner_server']['ros__parameters']['GridBased']
    assert planner['plugin'] == 'nav2_smac_planner/SmacPlanner2D'
    assert 'cost_travel_multiplier' not in planner  # Keep installed default1.0.

def test_legal_score_bound():
    doc = load(CANDIDATE)
    params = follow(doc)
    resolution = doc['local_costmap']['local_costmap']['ros__parameters']['resolution']
    scale = params['ObstacleFootprint.scale']
    assert resolution == 0.05
    assert params['BaseObstacle.scale'] == 0.02
    assert not params.get('BaseObstacle.sum_scores', False)
    assert not params.get('ObstacleFootprint.sum_scores', False)
    assert math.isfinite(scale) and 0 < scale <= (1e-5 * 252 * 0.02) / (253 * resolution)
    contribution = resolution * scale * 253
    assert contribution == pytest.approx(1.265e-5)
    assert contribution / (252 * params['BaseObstacle.scale']) <= 1e-5

def test_installed_plugin_xml_resolution():
    from ament_index_python.packages import get_package_share_directory
    xml = Path(get_package_share_directory('dwb_critics')) / 'default_critics.xml'
    tree = ET.parse(xml)
    classes = tree.findall('.//class')
    matches = [c for c in classes if c.attrib['type'] == 'dwb_critics::ObstacleFootprintCritic']
    assert len(matches) == 1
    assert matches[0].attrib['base_class_type'] == 'dwb_core::TrajectoryCritic'
    assert tree.find('.//library').attrib['path'] == 'dwb_critics'
