"""Independent, video-free preflight for the qualified warehouse route."""
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PINNED_SOURCES = [
    'scripts/run_warehouse_navigation.py', 'scripts/warehouse_navigation_qualification.py',
    'scripts/warehouse_track_artifacts.py', 'scripts/warehouse_ab_trial.py',
    'scripts/run_warehouse_route.py', 'scripts/warehouse_route_evidence.py',
    'scripts/warehouse_route_admission.py', 'scripts/warehouse_ab_supervisor.py',
    'course_sim/course_sim/session.py',
]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_review(path):
    """Require reviewed control/deployment evidence; no presentation artifact is required."""
    path = Path(path).resolve(); side = path.with_suffix(path.suffix+'.sha256')
    if not path.is_file() or not side.is_file():
        raise RuntimeError(f'Missing independently reviewed {path} and {side}; retained heading/Profile-B envelope remain unqualified')
    expected = side.read_text().split()[0]
    if digest(path) != expected:
        raise RuntimeError('Reviewed run configuration SHA256 changed')
    cfg = json.loads(path.read_text())
    for key in ('deployment_manifest', 'route_envelope', 'control_review'):
        if key not in cfg:
            raise RuntimeError('Missing independently reviewed '+key)
        item = cfg[key]
        if digest(item['path']) != item['sha256']:
            raise RuntimeError(key+' pin mismatch')
    pins = cfg.get('source_pins', {})
    if set(pins) != set(PINNED_SOURCES) or any(digest(ROOT/name) != pins[name] for name in PINNED_SOURCES):
        raise RuntimeError('Independently reviewed automation source pins changed/incomplete')
    control = json.loads(Path(cfg['control_review']['path']).read_text())
    required = ('retained_heading', 'bidirectional_turning', 'small_command_tracking',
                'profile_B_directional_stopping', 'balance_and_stop', 'independent_review')
    if not all(control.get(k) is True for k in required):
        raise RuntimeError('Controller/route qualification not accepted')
    if not control.get('reviewer') or control.get('reviewer') == control.get('implementer'):
        raise RuntimeError('Independent scientific review absent')
    if control.get('deployment_manifest_sha256') != cfg['deployment_manifest']['sha256']:
        raise RuntimeError('Control qualification belongs to another deployment')
    for evidence in control.get('evidence', []):
        if digest(evidence['path']) != evidence['sha256']:
            raise RuntimeError('Control raw evidence changed')
    if not control.get('evidence'):
        raise RuntimeError('No empirical controller qualification')
    yaws = cfg.get('continuation_yaws')
    if not isinstance(yaws, list) or len(yaws) != 3 or not all(
            type(v) in (int, float) and math.isfinite(v) for v in yaws):
        raise RuntimeError('G3-G5 reviewed arrival headings absent')
    from warehouse_route_admission import load_envelope
    cfg['envelope'] = load_envelope(cfg['route_envelope']['path'], cfg['route_envelope']['sha256'],
        digest(ROOT/'artifacts/recovery/robot.urdf'),
        digest(ROOT/'course_navigation/config/nav2_smac2d_dwb_slow_diag.yaml'), [.2, .6],
        cfg['deployment_manifest']['sha256'])
    cfg['_review_path'] = str(path); cfg['_review_sha256'] = expected
    return cfg


def ros_env(domain):
    script = 'source /opt/ros/humble/setup.bash && source install/setup.bash && env -0'
    raw = subprocess.check_output(['/bin/bash', '-c', script], cwd=ROOT)
    env = dict(item.split('=', 1) for item in raw.decode().split('\0') if '=' in item)
    env.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'course_sim/config/fastdds_local.xml'),
        QT_QPA_PLATFORM='xcb')
    env['LD_LIBRARY_PATH'] = str(ROOT/'.deps/lib')+':' + str(ROOT/'b2w_sim/b2w_controllers/third_party/onnxruntime/lib64')+':'+env.get('LD_LIBRARY_PATH', '')
    return env


def _course_stack(argv):
    try:
        parts = shlex.split(' '.join(argv))
    except ValueError:
        return any('ign gazebo' in item for item in argv)
    names = {Path(item).name for item in parts[:2]}
    return bool(names & {'b2w_rl_sar_controller', 'lio_sam_mapOptimization', 'warehouse_ab_trial.py'} or
                any(Path(item).name in ('ign', 'gz') and parts[i+1:i+2] == ['gazebo']
                    for i, item in enumerate(parts)))


def host_admission():
    gpu = subprocess.run(['nvidia-smi', '--query-compute-apps=pid,process_name,used_memory',
                          '--format=csv,noheader'], check=True, capture_output=True, text=True).stdout.strip()
    if gpu:
        raise RuntimeError('Insufficient GPU isolation: existing compute process; no job will be modified')
    competing = []
    for command in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            argv = [item.decode(errors='replace') for item in command.read_bytes().split(b'\0') if item]
            if argv and _course_stack(argv):
                competing.append(str(command.parent))
        except OSError:
            continue
    if competing:
        raise RuntimeError('Existing simulator/controller ownership: '+','.join(competing))
    return dict(gpu_compute_empty=True, competing=[])


def fresh_domain():
    from course_sim.session import SessionLease
    for domain in range(120, 233):
        lock = ROOT/'course_results'/f'.b2w-course-{os.getuid()}-domain-{domain}.lock'
        if lock.exists():
            continue
        occupied = False
        for environ in Path('/proc').glob('[0-9]*/environ'):
            try:
                if f'ROS_DOMAIN_ID={domain}'.encode() in environ.read_bytes().split(b'\0'):
                    occupied = True; break
            except OSError:
                continue
        if not occupied:
            lease = SessionLease(domain); lease.close()
            return domain
    raise RuntimeError('No fresh unused ROS domain')
