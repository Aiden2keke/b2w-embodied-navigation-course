#!/usr/bin/python3
"""Passive zero-motion readiness probe. Never sends goals, actions or velocity."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'course_navigation'))
from stage4_route.contract import ROOT,write_new,digest


def check_capacity(transport,*,wall,last_capacity):
    first=getattr(transport,'recording_first_wall',None)
    if first is not None and wall-first>=5. and wall-last_capacity>=1.:
        transport.verify_recording_capacity(90.)
        return wall
    return last_capacity


def probe(directory,deployment):
    from stage4_route.ros_transport import RosTransport
    transport=None;error=None;passed=False;clean=False;first=None;end=None;closure=None
    started=time.monotonic();verified=False;last_graph=0.;last_capacity=0.;last_wait=None;previous=None
    try:
        from stage4_route.motion_qualification import require_motion_qualification
        try:require_motion_qualification()
        except ValueError as exc:
            if 'UNQUALIFIED_FOR_MOTION' not in str(exc):raise
        else:raise ValueError('Zero-only probe requires the production motion interlock')
        selection=dict(runtime_config=str(ROOT/'course_navigation/config/nav2_smac2d_diag.yaml'))
        transport=RosTransport(directory,deployment=deployment,stationary_only=True)
        while time.monotonic()-started<90.:
            transport.spin();wall=time.monotonic()
            if transport.failure or transport.journal.failure:raise ValueError(transport.failure or transport.journal.failure)
            if not verified:
                try:transport.verify_runtime(selection);verified=True
                except (KeyError,ValueError,LookupError) as exc:last_wait=str(exc);continue
            try:
                s=transport.sample();transport.check_live_map()
                if wall-last_graph>=1.:
                    current=transport.graph()
                    if first and current!=graph:raise ValueError('Session graph changed during zero observation')
                    graph=current;last_graph=wall
                if not transport.contacts.ready(sim=s['sim'],wall=wall):raise ValueError('Contact reports missing, stale or forbidden')
                if (s['safety']!='clear' or s['speed']>.03 or abs(s['wz'])>.03 or
                        s['command_speed']!=0. or s['command_wz']!=0.):raise ValueError('Zero-motion stationary evidence failed')
            except (KeyError,ValueError,LookupError) as exc:
                if first:raise
                last_wait=str(exc);continue
            if previous and s['tf_stamp']>previous['tf_stamp'] and (
                    s['tf_stamp']-previous['tf_stamp']>.3 or s['wall']-previous['wall']>.5):
                raise ValueError('Stationary source/receipt sampling gap')
            if previous is None or s['tf_stamp']>previous['tf_stamp']:previous=dict(s)
            if first is None:first=dict(s)
            if math.hypot(s['x']-first['x'],s['y']-first['y'])>.03:raise ValueError('Stationary observation excursion exceeded .03m')
            last_capacity=check_capacity(transport,wall=wall,last_capacity=last_capacity)
            if s['sim']-first['sim']>=30. and wall-first['wall']>=5.:
                if end is None:end=s['sim'];closure_deadline=wall+2.
                closure=transport.contact_closure(first['sim'],end)
                if closure['reporting_closed_through_end']:passed=True;break
                if wall>closure_deadline:raise ValueError('Contact reporting does not close the zero-observation endpoint')
        if not passed:raise ValueError('Zero-observation wall timeout: '+str(last_wait))
    except (Exception,KeyboardInterrupt) as exc:error=repr(exc)
    finally:
        if transport:
            try:clean=transport.close()
            except Exception as exc:error=(error or '')+' cleanup: '+repr(exc)
    report=dict(kind='stage4_zero_motion_readiness',zero_motion_smoke_pass=bool(passed and clean and not error),
        cleanup_passed=clean,error=error,first_sample=first,accepted_end_sim=end,contact_closure=closure,
        wall_duration_s=time.monotonic()-started,runtime=transport.audit if transport else {},
        motion_authorized=False,safety_complete=False,full_route_pass=False,segment_pass=False,
        limitation='Passive operational evidence only; physical bounds/coverage/stop chain and independent review remain required')
    if transport and transport.raw_path.exists():report['journal_sha256']=digest(transport.raw_path)
    write_new(directory/'zero-result.json',report)
    return 0 if report['zero_motion_smoke_pass'] else 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--deployment',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(mode=0o700,exist_ok=False)
    return probe(a.output,a.deployment)

if __name__=='__main__':raise SystemExit(main())
