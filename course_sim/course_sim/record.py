"""Record a new rosbag plus sensor and source metadata without overwriting runs."""
import argparse
import datetime
import json
import subprocess
import hashlib
import signal
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default='course_results')
    parser.add_argument('--fault-profile', choices=['normal', 'mild', 'medium', 'severe'], default='normal')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--horizontal-samples', type=int, default=2000)
    parser.add_argument('--scene', default='confined')
    parser.add_argument('--world', default='')
    parser.add_argument('--locomotion-backend', choices=['legacy', 'rl_sar'], default='rl_sar')
    parser.add_argument('--pretrained-directory', default='third_party/b2w_rl_sar')
    args = parser.parse_args()
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    run = Path(args.output_dir) / stamp
    run.mkdir(parents=True, exist_ok=False)
    share = Path(get_package_share_directory('course_sim'))
    profiles = yaml.safe_load((share / 'config/fault_profiles.yaml').read_text())
    metadata = vars(args) | {'fault_values': profiles[args.fault_profile],
                            'sensors': yaml.safe_load((share / 'config/sensors.yaml').read_text()),
                            'scan_model': 'instantaneous_snapshot', 'truth_usage': 'evaluation_only',
                            'lidar_self_returns': 'excluded_by_visibility_mask',
                            'lidar_self_occlusion_simulated': False,
                            'body_state_usage': 'low_level_locomotion_only'}
    metadata['sensors']['lidar']['horizontal_samples'] = args.horizontal_samples
    if args.locomotion_backend == 'rl_sar':
        bundle = Path(args.pretrained_directory).resolve()
        metadata['locomotion_source'] = json.loads((bundle / 'POLICY_SOURCE.json').read_text())
        metadata['policy_sha256'] = hashlib.sha256((bundle / 'policy/b2w/robot_lab/policy.onnx').read_bytes()).hexdigest()
        metadata['body_state_usage'] = 'rl_sar uses private raw IMU and joint feedback; navigation uses SLAM'
    metadata['sensors']['lidar']['actual_nominal_points_per_second'] = args.horizontal_samples * 32 * 10
    world = Path(args.world) if args.world else share / 'worlds/confined.world'
    if world.is_file():
        metadata['world_sha256'] = hashlib.sha256(world.read_bytes()).hexdigest()
    adaptation = world.parent / 'adaptation.json'
    if adaptation.is_file():
        metadata['scene_source'] = json.loads(adaptation.read_text())
    revision = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True)
    metadata['source_revision'] = revision.stdout.strip() if revision.returncode == 0 else None
    metadata['config_sha256'] = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                                 for path in (share / 'config').glob('*') if path.is_file()}
    status = subprocess.run(['git', 'status', '--porcelain'], capture_output=True, text=True)
    metadata['source_dirty'] = bool(status.stdout.strip()) if status.returncode == 0 else None
    (run / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    topics = ['/pointcloud', '/imu', '/sim/ground_truth/odom', '/tf', '/tf_static', '/clock',
              '/map',
              '/slam/odom', '/slam/map_points', '/goal_pose', '/collision_plate/force_torque',
              '/joint_states', '/path_manager/path_manager_ros/nav_vel']
    if args.locomotion_backend == 'rl_sar':
        topics += ['/sim/raw/imu', '/sim/raw/joint_states', '/network_input_debug', '/joint_commands', '/joint_torques']
    command = ['ros2', 'bag', 'record', '--use-sim-time', '-o', str(run / 'bag')] + topics
    process = subprocess.Popen(command)
    try:
        result = process.wait()
    except KeyboardInterrupt:
        process.send_signal(signal.SIGINT)
        result = process.wait(timeout=20)
    raise SystemExit(result)
