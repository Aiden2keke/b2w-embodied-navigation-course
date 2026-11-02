#!/usr/bin/python3
"""One-command, video-free, safety-gated Warehouse Start -> G1 -> G5 run."""
import argparse
import fcntl
import json
import os
from datetime import datetime, timezone
from pathlib import Path
import signal
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))


def wait_for_owner(process, request_stop=False):
    if request_stop and process.poll() is None:
        process.send_signal(signal.SIGINT)
    while True:
        try:
            return process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            continue  # Keep coordinator identity and exclusive lease until owner containment finishes.
        except KeyboardInterrupt:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', action='store_true', required=True,
                   help='Explicitly start Warehouse and the qualified autonomous route')
    p.add_argument('--review-config', type=Path,
                   default=ROOT/'course_results/tracks/reviewed_run.json')
    p.add_argument('--output-root', type=Path, default=ROOT/'course_results/tracks')
    return p


def run(argv=None):
    args = parser().parse_args(argv)
    output = args.output_root.resolve() / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    output.mkdir(parents=True, exist_ok=False)
    owner = None; report = {'success': False, 'stop_reason': 'not_started'}
    route_dir = None
    lease = None
    try:
        from warehouse_navigation_qualification import fresh_domain, host_admission, read_review, ros_env
        from warehouse_ab_supervisor import identity
        from warehouse_track_artifacts import write_track_artifacts
        from run_warehouse_route import load_route

        cfg = read_review(args.review_config.resolve())
        lock_path = ROOT/'course_results'/f'.b2w-g1-g5-batch-{os.getuid()}.lock'
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        lease = lock_path.open('a+')
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as lock_error:
            raise RuntimeError('Another qualified warehouse route workflow owns the run lock') from lock_error
        bootstrap_env = ros_env(0)
        for import_path in bootstrap_env.get('PYTHONPATH', '').split(':'):
            if import_path and import_path not in sys.path:
                sys.path.append(import_path)
        host = host_admission()
        domain = fresh_domain()
        manifest = cfg['deployment_manifest']
        caller = identity(os.getpid())
        session_root = output/'owner'; session_root.mkdir()
        command = ['/usr/bin/python3', str(ROOT/'scripts/warehouse_ab_trial.py'),
                   '--profile', 'B', '--recording-parent-pid', str(caller['pid']),
                   '--recording-parent-start-ticks', str(caller['start_ticks']), '--gui',
                   '--deployment-manifest', manifest['path'], '--output-root', str(session_root)]
        deployment = json.loads(Path(manifest['path']).read_text())
        if 'controller_revision' in deployment:
            command += ['--controller-revision-sha256', manifest['sha256']]
        command += ['--route-mode', 'full', '--authorize-motion', 'route-full',
                    '--continuation-yaws', *map(str, cfg['continuation_yaws']),
                    '--route-envelope', cfg['route_envelope']['path'],
                    '--route-envelope-sha256', cfg['route_envelope']['sha256']]
        env = ros_env(domain)
        report['host_admission'] = host
        report['ros_domain_id'] = domain
        with (output/'launch.log').open('x') as log:
            subprocess.run(command+['--check'], cwd=ROOT, env=env, stdout=log,
                           stderr=subprocess.STDOUT, check=True, timeout=30)
            owner = subprocess.Popen(command+['--execute'], cwd=ROOT, env=env,
                                     stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
            try:
                code = owner.wait()
            except KeyboardInterrupt:
                report['stop_reason'] = 'interrupted'
                code = wait_for_owner(owner, request_stop=True)
        sessions = sorted(session_root.glob('*-*/session-report.json'))
        session_report = json.loads(sessions[-1].read_text()) if sessions else {}
        route_dir = sessions[-1].parent/'route' if sessions else None
        metrics_root = sessions[-1].parent/'goal-manager' if sessions else None
        report = session_report
        report['host_admission'] = host
        report['ros_domain_id'] = domain
        report['process_exit_code'] = code
        report['stop_reason'] = report.get('route', {}).get('stop_reason') or report.get('error') or report.get('stop_reason')
        report['success'] = bool(code == 0 and report.get('success') is True)
        if route_dir and (route_dir/'evidence'/'index.json').is_file():
            evidence = json.loads((route_dir/'evidence'/'index.json').read_text())
        else:
            evidence = {'segments': [], 'errors': ['No route localization evidence was captured']}
        track = write_track_artifacts(output, load_route(), report, evidence,
                                      evidence_root=route_dir, metrics_root=metrics_root)
        report['track_directory'] = str(output)
        report['navigation_success'] = track['navigation_success']
        report['success'] = track['success']
        if report['navigation_success'] and not report['success']:
            report['stop_reason'] = track['failure_reason']
        track.update(track_directory=str(output), process_exit_code=code,
                     ros_domain_id=domain, host_admission=host,
                     video_recording='disabled; user records the screen manually')
        (output/'result.json').write_text(json.dumps(track, indent=2)+'\n')
    except (Exception, KeyboardInterrupt) as error:
        if owner is not None and owner.poll() is None:
            try:
                wait_for_owner(owner, request_stop=True)  # Preserve owner identity until watchdog cleanup ends.
            except ProcessLookupError:
                pass
        report = {'success': False,
                  'stop_reason': 'interrupted' if isinstance(error, KeyboardInterrupt) else 'prerequisite_or_runtime_failure',
                  'failure_reason': repr(error), 'process_exit_code': owner.returncode if owner else None}
        try:
            from run_warehouse_route import load_route
            from warehouse_track_artifacts import write_track_artifacts
            track = write_track_artifacts(output, load_route(), report,
                                          {'segments': [], 'errors': [repr(error)]}, evidence_root=route_dir)
            track.update(outcome='interrupted' if report['stop_reason'] == 'interrupted' else 'failure',
                         failure_reason=report['failure_reason'],
                         track_directory=str(output))
            (output/'result.json').write_text(json.dumps(track, indent=2)+'\n')
        except Exception as artifact_error:
            report['artifact_error'] = repr(artifact_error)
        (output/'failure.json').write_text(json.dumps(report, indent=2)+'\n')
    finally:
        if lease is not None:
            lease.close()
    print(str(output))
    return 0 if report.get('success') else 1


if __name__ == '__main__':
    raise SystemExit(run())
