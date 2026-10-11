import os
import subprocess
import sys
import pytest
from course_sim.session import SessionLease


def test_same_domain_is_exclusive_and_reusable_after_close(tmp_path):
    first = SessionLease(93, directory=tmp_path)
    with pytest.raises(RuntimeError, match='already running'):
        SessionLease(93, directory=tmp_path)
    env = os.environ.copy()
    env['PYTHONPATH'] = str(__import__('pathlib').Path(__file__).resolve().parents[1] / 'course_sim')
    child = subprocess.run([sys.executable, '-c',
        'from course_sim.session import SessionLease; import sys; SessionLease(93,directory=sys.argv[1])',
        str(tmp_path)], env=env, capture_output=True, text=True)
    assert child.returncode != 0 and 'already running' in child.stderr
    other = SessionLease(94, directory=tmp_path)
    other.close()
    first.close()
    replacement = SessionLease(93, directory=tmp_path)
    replacement.close()
