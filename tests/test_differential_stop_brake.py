"""Compile and execute the production C++ brake state machine; no ROS/Gazebo."""
from pathlib import Path
import subprocess

def test_cpp_stop_brake_lifecycle_and_limits(tmp_path):
    root=Path(__file__).resolve().parents[1]
    output=tmp_path/'brake-test'
    subprocess.run(['/usr/bin/c++','-std=c++17','-Wall','-Wextra','-pedantic',
        '-I',str(root/'b2w_sim/b2w_controllers/src'),
        str(root/'tests/cpp/test_differential_stop_brake.cpp'),'-o',str(output)],check=True)
    subprocess.run([str(output)],check=True)
