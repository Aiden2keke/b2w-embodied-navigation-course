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
        self.config=dict(config);self.sequence=0;self.last_heartbeat=None;self.last_health=None;self.failure=None
        distinct_control_paths([config[k] for k in ('heartbeat_path','lease_path','inhibit_path','adapter_heartbeat_path')])

    def step(self,*,wall):
        c=self.config
        try:
            if self.failure or os.path.lexists(c['inhibit_path']):raise ValueError(self.failure or 'inhibit_latched')
            if not all(process_matches(c[k]) for k in ('owner','adapter')):raise ValueError('owned_process_identity_lost')
            health=read_control(c['adapter_heartbeat_path'])
            if (health['session_id']!=c['session_id'] or health['adapter']!=c['adapter'] or
                    type(health['sequence']) is not int or health['sequence']<=0 or
                    not all(type(v) in (int,float) and math.isfinite(v) for v in (wall,health['wall'])) or
                    not 0<=wall-health['wall']<=.15):raise ValueError('adapter_publication_stalled')
            if self.last_health and (health['sequence']<self.last_health['sequence'] or
                    health['wall']<self.last_health['wall'] or
                    health['sequence']==self.last_health['sequence'] and health!=self.last_health):
                raise ValueError('adapter_publication_regression_or_conflict')
            self.last_health=health
            h=read_control(c['heartbeat_path'])
            if (h['session_id']!=c['session_id'] or type(h['sequence']) is not int or h['sequence']<=0 or
                    not all(type(v) in (int,float) and math.isfinite(v) for v in (wall,h['wall'])) or
                    not 0<=wall-h['wall']<=.25):raise ValueError('owner_heartbeat_stale_or_invalid')
            if self.last_heartbeat and (h['sequence']<self.last_heartbeat['sequence'] or h['wall']<self.last_heartbeat['wall']):
                raise ValueError('owner_heartbeat_regression')
            if self.last_heartbeat and h['sequence']==self.last_heartbeat['sequence'] and h!=self.last_heartbeat:
                raise ValueError('conflicting_owner_heartbeat_duplicate')
            self.last_heartbeat=h;self.sequence+=1
            if c.get('zero_only') is True:
                if os.path.lexists(c['lease_path']):raise ValueError('unexpected_lease_in_zero_only_session')
                return True  # Monitor health and inhibit faults; NEVER arm the adapter.
            write_control(c['lease_path'],dict(session_id=c['session_id'],sequence=self.sequence,wall=wall,
                                              owner=c['owner'],adapter=c['adapter']))
            return True
        except (OSError,ValueError,KeyError,TypeError) as exc:
            self.failure=self.failure or str(exc)
            latch_inhibit(c['inhibit_path'],self.failure)
            Path(c['lease_path']).unlink(missing_ok=True)
            return False
