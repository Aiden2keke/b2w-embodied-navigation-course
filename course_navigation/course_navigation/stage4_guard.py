"""Opt-in effective final-command veto, with no implied motion qualification.

Bounds cover the *entire* worst measured stop excursion, plus source/abort latency.
No conversion from requested command to physical braking capability is invented.
Faults latch until a new session. Grid preparation can run off the ROS callback.
"""
import copy
import math
from pathlib import Path
import threading
import time
from .stage4_geometry import ClearanceGrid,grid_revision
from .structural_freshness import StructuralMapCadence,structural_ages
from .stage4_control import process_matches,process_identity,read_control,latch_inhibit,distinct_control_paths
import os

REQUIRED={'saved':'map','live':'map','local':'odom','global':'map'}


class FinalCommandGuard:
    def __init__(self,config):
        self.config=copy.deepcopy(config);self.grids={};self.failure=None
        self.last_sim=self.last_wall=None;self.lease_sequence=0;self.armed=False;self.last_lease=None
        self.contacts=None
        self.adapter_identity=process_identity(os.getpid());self.sources={}
        self.structural=StructuralMapCadence();self.live_cadence=StructuralMapCadence()
        self.lock=threading.Lock();self.last_report={}
        c=self.config
        if 'zero_only' in c and type(c['zero_only']) is not bool:raise ValueError('Invalid zero-only mode')
        self.zero_only=c.get('zero_only') is True
        positive=('stop_translation_m','stop_yaw_rad','uncertainty_m','max_speed','max_wz',
                  'max_command','max_command_wz','abort_latency_s','max_sim_rate')
        if not self.zero_only and (any(type(c[k]) not in (int,float) or not math.isfinite(c[k]) or c[k]<=0 for k in positive) or
                not c.get('session_id') or len(c['bounds_evidence_sha256'])!=64):
            raise ValueError('Explicit finite stop/latency/uncertainty evidence bounds required')
        if not self.zero_only and c['abort_latency_s']<.6:raise ValueError('Abort allowance must cover owner/lease/timer/compute detection budgets')
        paths=[Path(c[k]) for k in ('lease_path','inhibit_path')]
        if any(not p.is_absolute() for p in paths) or paths[0]==paths[1]:raise ValueError('Distinct absolute control paths required')
        distinct_control_paths(paths)

    def check_zero_control(self):
        """An immutable observation manifest has no transition to armed operation."""
        if not self.zero_only:return
        if self.failure:raise ValueError(self.failure)
        if os.path.lexists(self.config['lease_path']):raise ValueError('unexpected_lease_in_zero_only_session')
        if os.path.lexists(self.config['inhibit_path']):raise ValueError('independent_inhibit')
        if not process_matches(self.config['owner']):raise ValueError('owner_identity_lost')

    def inhibit(self,reason):
        self.failure=self.failure or str(reason)
        try:latch_inhibit(self.config['inhibit_path'],self.failure)
        except OSError:pass  # Still return zero; lease remains bound to this exact adapter PID.

    def invalidate(self,name):
        with self.lock:self.grids.pop(name,None)

    def install(self,name,grid,*,frame,source,receipt,encoding,epoch):
        entry=self.prepare(name,grid,frame=frame,source=source,receipt=receipt,encoding=encoding,epoch=epoch)
        if entry is not None:
            with self.lock:self.grids[name]=entry

    def prepare(self,name,grid,*,frame,source,receipt,encoding,epoch):
        started=time.monotonic()
        if name not in REQUIRED or frame!=REQUIRED[name] or epoch!=self.config['session_id']:
            self.inhibit('grid_frame_or_session_mismatch');return
        cells=grid['data']
        if (encoding not in ('occupancy','costmap') or len(cells)>1000000 or
                len(cells)!=grid['width']*grid['height'] or
                any(type(v) is not int or not (-1<=v<=100 if encoding=='occupancy' else 0<=v<=255) for v in cells) or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in (source,receipt))):
            self.inhibit('invalid_grid');return
        frozen,revision=grid_revision(grid,encoding)
        with self.lock:
            old=self.sources.get(name)
            if old and source<old[1]:self.inhibit('grid_source_clock_regression');return
            if old and source==old[1]:
                if revision!=old[3]:self.inhibit('conflicting_grid_source_revision');return
                return old
        checker=old[0] if old and revision==old[3] else ClearanceGrid(frozen)
        checker.data.flags.writeable=False;checker.points.flags.writeable=False
        with self.lock:
            if name=='live':self.live_cadence.observe(source,source,receipt)
            entry=(checker,source,receipt,revision);self.sources[name]=entry
        if time.monotonic()-started>.5:self.inhibit('grid_preparation_deadline');return
        return entry

    def refresh(self,name,*,frame,source,receipt,encoding,epoch):
        """Only GridPreparation calls this after exact accepted-payload equality."""
        if (frame!=REQUIRED[name] or epoch!=self.config['session_id'] or
                encoding not in ('occupancy','costmap') or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in (source,receipt))):
            self.inhibit('invalid_grid_metadata');return False
        with self.lock:
            old=self.grids.get(name)
            if old is None:return False
            if source<old[1] or receipt<old[2]:
                self.inhibit('grid_clock_regression');return False
            if source==old[1]:return True  # duplicate cannot renew receipt
            if name=='live':self.live_cadence.observe(source,source,receipt)
            self.grids[name]=self.sources[name]=(old[0],source,receipt,old[3])
            return True

    def observe_structure(self,source,*,sim,wall):
        try:
            with self.lock:self.structural.observe(source,sim,wall)
        except (TypeError,ValueError) as exc:self.inhibit('structural_source:'+str(exc))

    def prepared_status(self):
        with self.lock:
            return dict(failure=self.failure,structural_wall_budget=self.live_cadence.wall_budget,
                        structural_source_wall_budget=self.structural.wall_budget,grids={k:dict(sha256=v[3],source=v[1],receipt=v[2]) for k,v in self.grids.items()})

    def output(self,*,command,poses,pose_stamp,pose_receipt,sim,wall,speed,wz):
        started=time.monotonic();c=self.config;result=(0.,0.,0.)
        try:
            if self.zero_only:
                self.check_zero_control()
                return result
            values=[*command,pose_stamp,pose_receipt,sim,wall,speed,wz]
            if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):raise ValueError('nonfinite_guard_input')
            if self.failure:raise ValueError(self.failure)
            if os.path.lexists(c['inhibit_path']):raise ValueError('independent_inhibit')
            if not process_matches(c['owner']):raise ValueError('owner_identity_lost')
            if ((self.last_sim is not None and sim<self.last_sim) or
                    (self.last_wall is not None and wall<self.last_wall)):raise ValueError('guard_clock_regression')
            if (self.last_wall is not None and wall>self.last_wall and
                    sim-self.last_sim>c['max_sim_rate']*(wall-self.last_wall)+1e-6):
                raise ValueError('declared_sim_rate_exceeded')
            self.last_sim,self.last_wall=sim,wall
            try:lease=read_control(c['lease_path'])
            except FileNotFoundError:
                if self.armed:raise ValueError('lease_lost')
                return result
            if (lease['session_id']!=c['session_id'] or type(lease['sequence']) is not int or
                    lease['sequence']<max(1,self.lease_sequence) or
                    type(lease['wall']) not in (int,float) or not math.isfinite(lease['wall']) or
                    not 0<=wall-lease['wall']<=.15):raise ValueError('lease_stale_or_invalid')
            if lease.get('adapter')!=self.adapter_identity or lease.get('owner')!=c['owner']:
                raise ValueError('lease_process_identity_mismatch')
            if self.last_lease and lease['sequence']==self.last_lease['sequence'] and lease!=self.last_lease:
                raise ValueError('conflicting_lease_duplicate')
            self.last_lease=lease
            self.lease_sequence=lease['sequence'];self.armed=True
            if self.contacts is None or not self.contacts.ready(sim=sim,wall=wall):
                raise ValueError('contact_reporting_missing_stale_or_forbidden')
            if not 0<=sim-pose_stamp<=.5 or not 0<=wall-pose_receipt<=.5:raise ValueError('guard_pose_stale')
            if speed<0 or speed>c['max_speed'] or abs(wz)>c['max_wz']:raise ValueError('physical_envelope_exceeded')
            if math.hypot(*command[:2])>c['max_command'] or abs(command[2])>c['max_command_wz'] or abs(command[1])>1e-9:
                raise ValueError('unsupported_guard_command')
            with self.lock:grids=dict(self.grids)
            if set(grids)!=set(REQUIRED) or self.structural.wall_budget is None or self.live_cadence.wall_budget is None:return result  # no output while preparing a new grid
            with self.lock:self.structural.ages(sim,wall)
            radius=math.hypot(.75,.43)
            # Adapter death additionally relies on the pinned unchanged policy's
            # .5 simulation-second command timeout, which still needs physical tests.
            delay=sim-pose_stamp+max(.5,c['abort_latency_s']*c['max_sim_rate'])
            reach=c['stop_translation_m']+c['max_speed']*delay+radius*(c['stop_yaw_rad']+c['max_wz']*delay)+c['uncertainty_m']
            reserves={}
            for name,(checker,source,receipt,revision) in grids.items():
                if name=='live':
                    with self.lock:structural_ages((source,receipt),self.live_cadence.wall_budget,sim=sim,wall=wall)
                elif name!='saved' and (not 0<=sim-source<=2. or not 0<=wall-receipt<=2.):raise ValueError('guard_grid_stale')
                pose=poses[REQUIRED[name]]
                if len(pose)!=3 or not all(math.isfinite(v) for v in pose):raise ValueError('invalid_guard_pose')
                reserves[name]=checker.clearance(pose,cap=max(.5,reach+.01))-reach
                if reserves[name]<=1e-9:raise ValueError('predictive_filled_body_or_braking_veto')
            if time.monotonic()-started>=.05:raise ValueError('guard_cycle_deadline')
            self.last_report=dict(reserves_m=reserves,reach_m=reach,wall_s=time.monotonic()-started,
                                  revisions={k:v[3] for k,v in grids.items()},motion_qualified=False)
            result=tuple(command)
        except (OSError,ValueError,TypeError,KeyError,OverflowError) as exc:self.inhibit(exc)
        return result


class GridPreparation:
    """At most four pending immutable copies; rebuilding a revision denies output."""
    def __init__(self,guard):
        self.guard=guard;self.accepted={};self.pending={};self.generations={};self.received={};self.condition=threading.Condition();self.closed=False
        self.worker=threading.Thread(target=self.run,daemon=True,name='stage4-grid-preparation');self.worker.start()

    def submit(self,name,grid,**metadata):
        if name not in REQUIRED:self.guard.inhibit('unknown_grid_source');return
        if len(grid['data'])>1000000 or len(grid['data'])!=grid['width']*grid['height']:
            self.guard.inhibit('oversized_or_malformed_grid');return
        frozen=dict(grid,data=tuple(grid['data']),origin=tuple(grid['origin']))
        with self.condition:
            source=metadata['source'];old=self.received.get(name)
            if old and source<old[0]:self.guard.inhibit('received_grid_clock_regression');return
            if old and source==old[0]:metadata=dict(metadata,receipt=old[1])
            else:self.received[name]=(source,metadata['receipt'])
            signature=(frozen,metadata['frame'],metadata['encoding'],metadata['epoch'])
            try:
                if self.accepted.get(name)==signature and self.guard.refresh(name,**metadata):return
            except (KeyError,TypeError,ValueError) as exc:
                self.guard.inhibit('grid_refresh:'+repr(exc));return
            self.guard.invalidate(name)
            generation=self.generations.get(name,0)+1;self.generations[name]=generation
            self.pending[name]=(generation,frozen,metadata);self.condition.notify()

    def run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda:self.closed or self.pending)
                if self.closed:return
                name=next(iter(self.pending));generation,grid,metadata=self.pending.pop(name)
            try:
                entry=self.guard.prepare(name,grid,**metadata)
                with self.condition:
                    if generation==self.generations[name] and entry is not None:
                        with self.guard.lock:self.guard.grids[name]=entry
                        self.accepted[name]=(grid,metadata['frame'],metadata['encoding'],metadata['epoch'])
            except Exception as exc:self.guard.inhibit('grid_preparation:'+repr(exc))

    def close(self):
        self.guard.inhibit('grid_worker_closed')
        with self.condition:self.closed=True;self.pending.clear();self.condition.notify()
        self.worker.join(timeout=2.)
        return not self.worker.is_alive()
