"""Private owned session bootstrap. Zero observation cannot issue or accept a motion lease."""
import copy
import math
import os
from pathlib import Path
from uuid import uuid4
from course_navigation.stage4_control import process_identity,process_matches,write_control,latch_inhibit
from course_navigation.stage4_guard import FinalCommandGuard


class SessionOwner:
    def __init__(self,directory,guard):
        self.directory=Path(directory).absolute();self.token=uuid4().hex;self.sequence=0
        self.config=copy.deepcopy(guard);self.config['owner']=process_identity(os.getpid())
        for key,name in (('heartbeat_path','owner.json'),('adapter_heartbeat_path','adapter.json'),
                         ('lease_path','lease.json'),('inhibit_path','inhibit.json')):
            self.config[key]=str(self.directory/name)
        FinalCommandGuard(self.config)  # validate before consuming the directory
        self.directory.mkdir(mode=0o700,parents=False,exist_ok=False)
        write_control(self.directory/'guard.json',self.config)

    def heartbeat(self,wall):
        self.sequence+=1
        write_control(self.config['heartbeat_path'],dict(session_id=self.config['session_id'],sequence=self.sequence,wall=wall))

    def accept_adapter(self,health,*,wall):
        if self.config.get('zero_only') is True and health.get('context_state')!='ready':
            raise ValueError('Bootstrap awaiting coherent source-time context')
        required={'live','local','global'} if self.config.get('zero_only') is True else {'saved','live','local','global'}
        prepared=health['prepared'];adapter=health['adapter'];budget=prepared['structural_wall_budget'];source_budget=prepared['structural_source_wall_budget']
        if (health['session_id']!=self.config['session_id'] or type(health['sequence']) is not int or health['sequence']<=0 or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in (wall,health['wall'])) or
                not 0<=wall-health['wall']<=.15 or prepared['failure'] or
                set(prepared['grids'])!=required or
                type(budget) not in (int,float) or not 7.5<=budget<=20. or
                type(source_budget) not in (int,float) or not 7.5<=source_budget<=20. or not process_matches(adapter)):
            raise ValueError('Bootstrap needs fresh fault-free prepared adapter health and measured structural cadence')
        marker=('B2W_NARROW_OWNER='+self.token).encode()
        if marker not in Path(f"/proc/{adapter['pid']}/environ").read_bytes().split(b'\0'):
            raise ValueError('Adapter does not belong to this owned session')
        return dict(self.config,adapter=adapter,zero_only=True)

    def stop(self,reason):
        latch_inhibit(self.config['inhibit_path'],reason)
