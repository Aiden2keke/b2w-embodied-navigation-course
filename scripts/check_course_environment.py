#!/usr/bin/python3
"""Report prerequisites without installing or changing the machine."""
import argparse
import importlib.util
import json
import platform
import os
import subprocess
import importlib
import shutil
import sys
from pathlib import Path


def check():
    from ament_index_python.packages import get_package_share_directory
    checks = {'python_3_10': sys.version_info[:2] == (3, 10), 'ros_humble': os.environ.get('ROS_DISTRO') == 'humble'}
    for name in ['ros2', 'colcon', 'ign']:
        checks['cli:' + name] = shutil.which(name) is not None
    for name in ['rclpy', 'numpy', 'scipy', 'yaml', 'onnxruntime', 'rosbag2_py']:
        try:
            importlib.import_module(name)
            checks['python:' + name] = True
        except (ImportError, OSError):
            checks['python:' + name] = False
    for name in ['ros_gz_sim', 'ros_gz_bridge', 'nav2_bringup', 'tf2_ros',
                 'robot_state_publisher', 'lio_sam']:
        try:
            get_package_share_directory(name)
            checks['ros:' + name] = True
        except LookupError:
            checks['ros:' + name] = False
    root = Path(__file__).resolve().parents[1]
    for path in ['b2w_sim/b2w_controllers/src/policy/policy_force_new.onnx',
                 'rl_nav_controller/deployment_policies/nav_policy.onnx',
                 'third_party/b2w_rl_sar/policy/b2w/robot_lab/policy.onnx',
                 'third_party/b2w_rl_sar/robot_assets/b2w_description/urdf/b2w_description.urdf']:
        checks['file:' + path] = (root / path).is_file()
    for binary in [root/'install/b2w_controllers/lib/b2w_controllers/b2w_controllers',
                   root/'install/b2w_controllers/lib/b2w_controllers/b2w_rl_sar_controller',
                   root/'install/lio_sam/lib/lio_sam/lio_sam_mapOptimization']:
        result = subprocess.run(['ldd', str(binary)], capture_output=True, text=True) if binary.is_file() else None
        checks['linked:' + binary.name] = result is not None and result.returncode == 0 and 'not found' not in result.stdout
    return {'display': os.environ.get('DISPLAY'), 'headless_gpu_cli': shutil.which('nvidia-smi') is not None, 'platform': platform.platform(), 'python': sys.executable,
            'checks': checks, 'ready': all(checks.values())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()
    result = check()
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(f"Python: {result['python']}")
        for name, passed in result['checks'].items():
            print(f"{'OK' if passed else 'MISSING':7} {name}")
    return 0 if result['ready'] else 1


if __name__ == '__main__':
    sys.exit(main())
