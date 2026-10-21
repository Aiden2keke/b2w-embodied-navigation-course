#!/usr/bin/python3
"""Offline synthetic grid/update benchmark; never imports ROS or qualifies motion."""
import argparse
import json
import os
from pathlib import Path
import statistics
import sys
import tempfile
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'course_navigation'))
from course_navigation.stage4_guard import FinalCommandGuard,GridPreparation
from course_navigation.stage4_control import process_identity,write_control


def benchmark(duration=3.,size=800):
    class TimedGuard(FinalCommandGuard):
        def prepare(self,*a,**kw):
            t=time.perf_counter()
            try:return super().prepare(*a,**kw)
            finally:preparation.append(time.perf_counter()-t)
    class SyntheticContacts:
        def ready(self,**kw):return True
    preparation=[];cycles=[];submit_times=[];available=zero=total=0
    with tempfile.TemporaryDirectory(prefix='stage4-grid-benchmark-') as directory:
        p=Path(directory);ident=process_identity(os.getpid())
        cfg=dict(session_id='synthetic',owner=ident,lease_path=str(p/'lease'),inhibit_path=str(p/'inhibit'),
            stop_translation_m=.2,stop_yaw_rad=.05,uncertainty_m=.05,max_speed=.3,max_wz=.3,
            max_command=.1,max_command_wz=.2,abort_latency_s=.6,max_sim_rate=2.,bounds_evidence_sha256='a'*64)
        g=TimedGuard(cfg);g.contacts=SyntheticContacts()
        grid=dict(width=size,height=size,resolution=.05,origin=[-size*.025,-size*.025,0.],data=[0]*(size*size))
        # Three advancing structural publications, synthetic same-rate clocks.
        wall=time.monotonic()
        for source in (0.,5.,10.):
            g.observe_structure(source,sim=source,wall=wall-10.+source)
            g.install('live',grid,frame='map',source=source,receipt=wall-10.+source,encoding='occupancy',epoch='synthetic')
        for name in ('saved','local','global'):
            g.install(name,grid,frame='odom' if name=='local' else 'map',source=10.,receipt=wall,encoding='costmap',epoch='synthetic')
        worker=GridPreparation(g);started=time.monotonic();next_update=0.;next_cycle=0.;next_structure=5.;sequence=0
        try:
            while time.monotonic()-started<duration:
                now=time.monotonic();elapsed=now-started;sim=10.+elapsed
                if elapsed>=next_structure:
                    g.observe_structure(sim,sim=sim,wall=now)
                    worker.submit('live',grid,frame='map',source=sim,receipt=now,encoding='occupancy',epoch='synthetic')
                    next_structure+=5.
                if elapsed>=next_update:
                    for name in ('local','global'):
                        t=time.perf_counter();worker.submit(name,grid,frame='odom' if name=='local' else 'map',source=sim,receipt=now,encoding='costmap',epoch='synthetic')
                        submit_times.append(time.perf_counter()-t)
                    next_update+=.1
                if elapsed>=next_cycle:
                    sequence+=1;write_control(cfg['lease_path'],dict(session_id='synthetic',sequence=sequence,wall=now,owner=ident,adapter=ident))
                    with g.lock:available+=int(set(g.grids)=={'saved','live','local','global'})
                    t=time.perf_counter();out=g.output(command=(.05,0.,0.),poses={'map':[0.,0.,0.],'odom':[0.,0.,0.]},pose_stamp=sim,pose_receipt=now,sim=sim,wall=now,speed=.1,wz=0.)
                    cycles.append(time.perf_counter()-t);zero+=int(out==(0.,0.,0.));total+=1;next_cycle+=.05
                time.sleep(.001)
            failure=g.failure
        finally:joined=worker.close()
    def stats(values):
        ordered=sorted(values)
        return dict(count=len(values),p99_ms=1000*ordered[min(len(ordered)-1,int(.99*len(ordered)))],max_ms=1000*max(values),mean_ms=1000*statistics.mean(values))
    return dict(kind='synthetic_offline_grid_benchmark',size=[size,size],duration_s=duration,
        requested_output_hz=20,requested_updates_hz=10,cycles=total,available_fraction=available/total,
        zero_duty=zero/total,cycle=stats(cycles),submit=stats(submit_times),preparation=stats(preparation),
        failure=failure,worker_joined=joined,motion_authorized=False,
        limitation='Synthetic clocks/contact/bounds; no ROS, DDS, physical timing or safety qualification')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float,default=3.)
    a=p.parse_args()
    if not .1<=a.duration<=30:p.error('duration must be in [.1,30] seconds')
    result=benchmark(a.duration)
    with a.output.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2));return 0

if __name__=='__main__':raise SystemExit(main())
