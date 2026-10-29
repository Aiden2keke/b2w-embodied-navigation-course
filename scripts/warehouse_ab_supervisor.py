#!/usr/bin/python3
"""Independent wall-clock containment for one explicitly owned simulator.

SIGSTOP containment is not a measured physical stop. No ROS or velocity output.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import shlex
import stat
import time
from narrow_processes import OwnedProcesses

FORTRESS_SERVER_LIBRARIES={
    '/usr/lib/x86_64-linux-gnu/libignition-gazebo6-ign.so.6.18.0':
        '78e7226957ca35217ade7e358d7ea7ec8c8de31ead36c7c462cb4898d7a9085a',
    '/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libignition-gazebo6-physics-system.so.6.18.0':
        'f1ccf096669cfc6f9cdfb2d153e1e8a6101571f8abcb22a24ca9040b4a86842b',
}


def identity(pid):
    fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    return dict(pid=int(pid), start_ticks=int(fields[19]), state=fields[0])


def same_process(pin, token=None):
    try:
        current = identity(pin['pid'])
        if current['start_ticks'] != pin['start_ticks']:
            return False
        if token is not None:
            marker = ('B2W_NARROW_OWNER=' + token).encode()
            if marker not in Path(f'/proc/{pin["pid"]}/environ').read_bytes().split(b'\0'):
                return False
        return current['state'] != 'Z'
    except (OSError, KeyError, ValueError):
        return False


def write_json(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix + f'.{os.getpid()}.tmp')
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    os.chmod(temp, 0o600)
    temp.replace(path)


def read_json(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or not 0 < info.st_size <= 65536:
            raise ValueError('Not a bounded owned regular JSON file')
        data = os.read(fd, 65537)
        if len(data) > 65536:
            raise ValueError('JSON grew beyond limit')
        return json.loads(data)
    finally:
        os.close(fd)


def pause_server(pin, token, timeout=.5):
    if not same_process(pin, token):
        return dict(contained=False, reason='server_identity_or_owner_mismatch')
    try:
        os.kill(pin['pid'], signal.SIGSTOP)
    except OSError as error:
        return dict(contained=False, reason=repr(error))
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if not same_process(pin, token):
            return dict(contained=True, state='dead', physical_stop_measured=False)
        try:
            states = {int(p.parent.name): p.read_text().rsplit(')', 1)[1].split()[0]
                      for p in Path(f'/proc/{pin["pid"]}/task').glob('*/stat')}
        except OSError:
            continue
        if states and all(state in ('T', 't', 'Z') for state in states.values()):
            return dict(contained=True, state='T', threads=states, physical_stop_measured=False)
        time.sleep(.01)
    return dict(contained=False, reason='SIGSTOP_not_confirmed', physical_stop_measured=False)


def health_fault(health, now, stale):
    numeric = lambda value: type(value) in (float, int) and math.isfinite(value)
    if not isinstance(health, dict) or not numeric(health.get('wall')):
        return 'invalid_monitor'
    if not 0 <= now - health['wall'] <= stale:
        return 'stale_monitor'
    if health.get('failure'):
        return 'monitor:' + str(health['failure'])
    if not numeric(health.get('clock_advanced_wall')):
        return 'invalid_clock_monitor'
    if not 0 <= now - health['clock_advanced_wall'] <= stale:
        return 'frozen_clock'
    return None


def terminate_owned(token, server):
    owner = OwnedProcesses(token)
    # A stopped simulator is killed without resuming physics. Other owned
    # processes receive TERM first; no process-name/group based signaling.
    if server and same_process(server, token):
        # Retain the verified identity before SIGKILL: a post-kill /proc scan
        # may no longer see it, but stationary acceptance needs this proof.
        owner.seen[server['pid']] = str(server['start_ticks'])
        try:
            os.kill(server['pid'], signal.SIGKILL)
        except ProcessLookupError:
            pass
    for sig, duration in ((signal.SIGTERM, .8), (signal.SIGKILL, .8)):
        for pid, ticks in owner.scan().items():
            if pid == os.getpid():
                continue
            try:
                if same_process(dict(pid=pid, start_ticks=int(ticks)), token):
                    os.kill(pid, sig)
            except ProcessLookupError:
                pass
        until = time.monotonic() + duration
        while time.monotonic() < until and set(owner.scan()) - {os.getpid()}:
            time.sleep(.02)
        if not set(owner.scan()) - {os.getpid()}:
            break
    return dict(owned_pid_start_ticks=owner.seen, remaining={p:t for p,t in owner.scan().items() if p != os.getpid()})


def validate_config(config):
    try:
        if not isinstance(config, dict) or not isinstance(config['token'], str) or not config['token']:
            raise ValueError()
        for field, ceiling in (('wall_limit_s', 1200.), ('stale_s', 2.)):
            value = config[field]
            if type(value) not in (float, int) or not math.isfinite(value) or not 0 < value <= ceiling:
                raise ValueError()
        for key in ('parent',):
            if any(type(config[key][v]) is not int or config[key][v] <= 0 for v in ('pid', 'start_ticks')):
                raise ValueError()
        for field in ('heartbeat', 'result', 'world'):
            if not Path(config[field]).is_absolute():
                raise ValueError()
        if config.get('server') is not None and any(type(config['server'][v]) is not int or config['server'][v] <= 0 for v in ('pid', 'start_ticks')):
            raise ValueError()
    except (KeyError, TypeError, ValueError):
        raise ValueError('Invalid bounded watchdog configuration') from None
    return config


def server_signature(cmdline, mappings, world):
    """Match actual Fortress native server plus its exact private world.

    Ruby can replace argv with a single display title, so NUL boundaries are
    not a reliable command tokenization boundary on this installed release.
    """
    if not world or not Path(world).is_absolute():return False
    try:
        args=shlex.split(cmdline.replace(b'\0',b' ').decode())
    except (ValueError,UnicodeError):return False
    invocation=any(Path(part).name=='ign' and args[i+1:i+2]==['gazebo'] for i,part in enumerate(args))
    return bool(invocation and args[-1:]==[str(world)] and '-s' in args and '-r' in args
                and set(FORTRESS_SERVER_LIBRARIES)<=set(mappings))


def discover_servers(token, world=None):
    found = []
    if not world:return found
    for pid,ticks in OwnedProcesses(token).scan().items():
        try:
            # Fortress CLI loads the native server into its Ruby process.
            # The invocation is pinned as well; no matching process names alone.
            cmd = Path(f'/proc/{pid}/cmdline').read_bytes()
            mappings={line.split(None,5)[5] for line in Path(f'/proc/{pid}/maps').read_text().splitlines() if len(line.split(None,5))==6}
            if (server_signature(cmd,mappings,world) and
                    all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha for path,sha in FORTRESS_SERVER_LIBRARIES.items())):
                candidate=identity(pid)
                if candidate['start_ticks']==int(ticks) and same_process(candidate,token):
                    found.append(candidate)
        except (OSError, ValueError):
            continue
    return found


def normal_complete(health, token):
    """Disarm only after measured normal rest AND no owned live children."""
    if not isinstance(health, dict) or health.get('complete') is not True:
        return False
    rest = health.get('normal_rest')
    if not isinstance(rest, dict) or health.get('cleanup_passed') is not True:
        return False
    for field in ('start_sim', 'end_sim', 'wall_s', 'max_displacement_m', 'max_yaw_rad'):
        if type(rest.get(field)) not in (float, int) or not math.isfinite(rest[field]):
            return False
    return (rest_proof(rest)
        and not (set(OwnedProcesses(token).scan()) - {os.getpid()}))


def rest_proof(rest):
    if not isinstance(rest, dict): return False
    for field in ('start_sim', 'end_sim', 'wall_s', 'max_displacement_m', 'max_yaw_rad'):
        if type(rest.get(field)) not in (float, int) or not math.isfinite(rest[field]): return False
    return (rest.get('six_axis_final_zero') is True and rest.get('measured_ground_truth_rest') is True
        and rest['end_sim']-rest['start_sim'] >= 5. and rest['wall_s'] >= 3.
        and 0 <= rest['max_displacement_m'] <= .02 and 0 <= rest['max_yaw_rad'] <= .03)


def watchdog(config):
    """Independent child survives owner death; bounded even if ROS/clock freezes."""
    validate_config(config)
    started = time.monotonic()
    server = config.get('server')
    if server and not any(p['pid'] == server['pid'] and p['start_ticks'] == server['start_ticks'] for p in discover_servers(config['token'],config['world'])):
        server = None  # An arbitrary caller-supplied PID is never a physics identity.
    reason = None
    shutdown_started = None
    while reason is None:
        now = time.monotonic()
        try:
            if server is None:
                servers = discover_servers(config['token'],config['world'])
                if len(servers) == 1:
                    server = servers[0]
                    write_json(Path(config['result']).with_name('server-pin.json'), server)
                elif len(servers) > 1:
                    reason = 'multiple_owned_servers'
                    break
        except Exception:
            reason = 'server_discovery_or_pin_failure'
            break
        if not same_process(config['parent']):
            reason = 'parent_dead'
        elif now - started >= config['wall_limit_s']:
            reason = 'wall_limit'
        else:
            try:
                health = read_json(config['heartbeat'])
                # The owner can publish during discovery or file I/O. Compare
                # its stamp with a clock sampled AFTER acquiring that snapshot.
                now = time.monotonic()
                reason = health_fault(health, now, config['stale_s'])
                if (reason is None and server is not None and
                        health.get('phase') != 'shutdown' and
                        not same_process(server, config['token'])):
                    reason = 'server_lost'
                # Bootstrap can wait for the FIRST clock for at most 90s;
                # receipt heartbeat remains mandatory and cannot renew grace.
                if (reason in ('invalid_clock_monitor', 'frozen_clock') and
                        health.get('phase') == 'bootstrap' and now-started < 90.):
                    reason = None
                if (health.get('phase') == 'shutdown' and rest_proof(health.get('normal_rest'))
                        and server is not None and not same_process(server, config['token'])):
                    if shutdown_started is None: shutdown_started = now
                    if reason in ('invalid_clock_monitor', 'frozen_clock') and now-shutdown_started <= 10.:
                        reason = None
                if normal_complete(health, config['token']) and reason in (None, 'frozen_clock', 'invalid_clock_monitor'):
                    write_json(config['result'], dict(reason='normal_shutdown', emergency=False))
                    return 0
            except Exception:
                reason = 'missing_or_invalid_monitor'
        if reason is None:
            time.sleep(.02)
    try:
        paused = pause_server(server, config['token']) if server else dict(contained=False, reason='server_not_discovered')
    except Exception as error:
        paused = dict(contained=False, reason=repr(error))
    finally:
        # Physics is already stopped. Keep the existing adapter/observer alive
        # briefly for an in-flight owner zero; never delay containment itself.
        if paused.get('contained') is True:
            time.sleep(.1)
        cleanup = terminate_owned(config['token'], server)
    try:
        write_json(config['result'], dict(reason=reason, emergency=True, pause=paused, **cleanup))
    except OSError:
        return 2  # Containment already attempted even when evidence write fails.
    return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--watchdog', type=Path, required=True)
    args = parser.parse_args()
    return watchdog(json.loads(args.watchdog.read_text()))


if __name__ == '__main__':
    raise SystemExit(main())
