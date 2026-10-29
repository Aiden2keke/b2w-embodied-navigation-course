"""Private same-host control lease/inhibit. No ROS actions, velocity or truth input.

Independent watchdog revokes permission; only the adapter publishes final Twist.
Process/lease loss forces zero at the next publication. Adapter death relies on
the unchanged low-level timeout and STILL requires measured physical qualification.
"""
import json
import math
import os
from pathlib import Path
import tempfile
import stat
import time


def process_identity(pid):
    p=Path('/proc')/str(pid)
    fields=(p/'stat').read_text().rsplit(')',1)[1].split()
    if fields[0]=='Z':raise ValueError('dead_process')
    return dict(pid=int(pid),start_ticks=fields[19],pgid=int(fields[2]),uid=p.stat().st_uid)


def process_matches(identity):
    try:return process_identity(identity['pid'])==identity
    except (OSError,ValueError,KeyError,TypeError):return False


def read_control(path):
    fd=os.open(path,os.O_RDONLY|os.O_NONBLOCK|os.O_NOFOLLOW|os.O_CLOEXEC)
    try:
        mode=os.fstat(fd)
        if not stat.S_ISREG(mode.st_mode) or mode.st_uid!=os.getuid() or mode.st_mode&0o022:
            raise ValueError('Control record must be an owned, non-writable-by-others regular file')
        data=os.read(fd,8193)
    finally:os.close(fd)
    if len(data)>8192:raise ValueError('oversized_control_record')
    return json.loads(data)


def write_control(path,value):
    p=Path(path);data=json.dumps(value,sort_keys=True,allow_nan=False).encode()
    if len(data)>8192:raise ValueError('oversized_control_record')
    with tempfile.NamedTemporaryFile(dir=p.parent,prefix='.control-',delete=False) as f:
        name=f.name;f.write(data);f.flush();os.fsync(f.fileno())
    try:os.replace(name,p)
    finally:
        if Path(name).exists():Path(name).unlink()


def write_adapter_health(path,value):
    """Atomically publish a transient heartbeat, without disk durability waits.

    Readers require a live adapter identity and the original callback wall time
    within 150ms. A heartbeat is never a restart/recovery record. Keep durable
    owner/config/lease writes on write_control; no background writer or queue.
    """
    p=Path(path);data=json.dumps(value,sort_keys=True,allow_nan=False).encode()
    if len(data)>8192:raise ValueError('oversized_control_record')
    name=None
    try:
        with tempfile.NamedTemporaryFile(dir=p.parent,prefix='.control-',delete=False) as f:
            name=f.name;f.write(data);f.flush()
        os.replace(name,p)  # Closed, complete 0600 file; readers see old OR new JSON.
    finally:
        if name is not None and Path(name).exists():Path(name).unlink()


def latch_inhibit(path,reason):
    try:
        fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_CLOEXEC,0o600)
        with os.fdopen(fd,'w') as f:json.dump(dict(inhibited=True,reason=str(reason)),f)
    except FileExistsError:pass  # A later actor cannot clear or weaken the first fault.


def distinct_control_paths(paths):
    resolved=[];inodes=[]
    for name in paths:
        p=Path(name)
        if not p.is_absolute() or p.is_symlink():raise ValueError('Absolute non-symlink control paths required')
        resolved.append(p.resolve())
        if p.exists():
            st=p.stat();inodes.append((st.st_dev,st.st_ino))
    if len(set(resolved))!=len(resolved) or len(set(inodes))!=len(inodes):
        raise ValueError('Control paths and existing inodes must be distinct')


class Watchdog:
    def __init__(self,config):
        self.config=dict(config);self.sequence=0;self.last_heartbeat=None;self.last_health=None;self.failure=None;self.first_fault=None
        distinct_control_paths([config[k] for k in ('heartbeat_path','lease_path','inhibit_path','adapter_heartbeat_path')])

    def step(self,*,wall=None):
        c=self.config;health=None;h=None;failed_predicate='control.inhibit'
        try:
            if self.failure or os.path.lexists(c['inhibit_path']):raise ValueError(self.failure or 'inhibit_latched')
            failed_predicate='process.identity'
            if not all(process_matches(c[k]) for k in ('owner','adapter')):raise ValueError('owned_process_identity_lost')
            failed_predicate='health.read'
            health=read_control(c['adapter_heartbeat_path'])
            failed_predicate='owner.read'
            h=read_control(c['heartbeat_path'])
            # Validate immutable read snapshots against time sampled AFTER both reads.
            # Sampling in the caller can make a concurrent publication appear future.
            if wall is None:wall=time.monotonic()
            failed_predicate='health.validation'
            if health['session_id']!=c['session_id']:failed_predicate='health.session_id'
            elif health['adapter']!=c['adapter']:failed_predicate='health.adapter'
            elif type(health['sequence']) is not int or health['sequence']<=0:failed_predicate='health.sequence'
            elif type(wall) not in (int,float) or not math.isfinite(wall):failed_predicate='sampled_wall_nonfinite'
            elif type(health['wall']) not in (int,float) or not math.isfinite(health['wall']):failed_predicate='health.wall_nonfinite'
            elif wall-health['wall']<0:failed_predicate='health.age_negative'
            elif wall-health['wall']>.15:failed_predicate='health.age_gt_150ms'
            else:failed_predicate=None
            if failed_predicate:raise ValueError('adapter_publication_stalled')
            failed_predicate='health.regression_or_conflict'
            if self.last_health and (health['sequence']<self.last_health['sequence'] or
                    health['wall']<self.last_health['wall'] or
                    health['sequence']==self.last_health['sequence'] and health!=self.last_health):
                raise ValueError('adapter_publication_regression_or_conflict')
            self.last_health=health
            failed_predicate='owner.validation'
            if (h['session_id']!=c['session_id'] or type(h['sequence']) is not int or h['sequence']<=0 or
                    not all(type(v) in (int,float) and math.isfinite(v) for v in (wall,h['wall'])) or
                    not 0<=wall-h['wall']<=.25):raise ValueError('owner_heartbeat_stale_or_invalid')
            failed_predicate='owner.regression_or_conflict'
            if self.last_heartbeat and (h['sequence']<self.last_heartbeat['sequence'] or h['wall']<self.last_heartbeat['wall']):
                raise ValueError('owner_heartbeat_regression')
            if self.last_heartbeat and h['sequence']==self.last_heartbeat['sequence'] and h!=self.last_heartbeat:
                raise ValueError('conflicting_owner_heartbeat_duplicate')
            self.last_heartbeat=h;self.sequence+=1
            if c.get('zero_only') is True:
                failed_predicate='zero_only.unexpected_lease'
                if os.path.lexists(c['lease_path']):raise ValueError('unexpected_lease_in_zero_only_session')
                return True  # Monitor health and inhibit faults; NEVER arm the adapter.
            failed_predicate='lease.write'
            write_control(c['lease_path'],dict(session_id=c['session_id'],sequence=self.sequence,wall=wall,
                                              owner=c['owner'],adapter=c['adapter']))
            return True
        except (OSError,ValueError,KeyError,TypeError) as exc:
            self.failure=self.failure or str(exc)
            latch_inhibit(c['inhibit_path'],self.failure)
            Path(c['lease_path']).unlink(missing_ok=True)
            if self.first_fault is None:
                if wall is None:wall=time.monotonic()
                # One bounded in-memory snapshot only AFTER permission is revoked.
                def safe(value):
                    if type(value) is float and not math.isfinite(value):return repr(value)
                    if isinstance(value,dict):return {k:safe(v) for k,v in value.items()}
                    if isinstance(value,list):return [safe(v) for v in value]
                    return value
                age=None
                if (isinstance(health,dict) and all(type(v) in (int,float) and math.isfinite(v)
                        for v in (wall,health.get('wall')))):age=wall-health['wall']
                self.first_fault=safe(dict(reason=self.failure,failed_predicate=failed_predicate,
                    sampled_wall=wall,health_age_s=age,
                    health={k:health.get(k) for k in ('session_id','adapter','sequence','wall')} if isinstance(health,dict) else None,
                    owner={k:h.get(k) for k in ('session_id','sequence','wall')} if isinstance(h,dict) else None))
            return False
