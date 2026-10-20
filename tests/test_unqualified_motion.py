"""The known-unqualified Stage4 stack must reject before ROS initialization."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))


def test_cli_motion_flag_is_not_a_safety_qualification(tmp_path):
    result = subprocess.run([sys.executable, str(ROOT/'scripts/run_candidate_route.py'), 'run',
                             '--selection', str(tmp_path/'missing.json'), '--session', str(tmp_path/'session'),
                             '--mode', 'full', '--execute-user-run'], capture_output=True, text=True)
    assert result.returncode == 2
    assert 'UNQUALIFIED_FOR_MOTION' in result.stderr
    assert not (tmp_path/'session').exists()


def test_direct_transport_send_cannot_bypass_cli_interlock():
    from stage4_route.ros_transport import RosTransport
    import pytest
    transport = RosTransport.__new__(RosTransport)
    transport.failure = None
    from types import SimpleNamespace
    transport.journal = SimpleNamespace(failure=None)
    with pytest.raises(ValueError, match='UNQUALIFIED_FOR_MOTION'):
        transport.send([], {})
