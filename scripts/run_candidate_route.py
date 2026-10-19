#!/usr/bin/python3
"""Explicit candidate route preparation, registration and USER-RUN execution.

Default commands are offline. --capture-live is read-only; --execute-user-run
is the only motion boundary, never invoked by offline tests or automatically.
"""
import argparse
import json
from pathlib import Path
import sys
from uuid import uuid4
from stage4_route.contract import select_candidate,load_selection,digest,object_hash,write_new,registration


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    s=sub.add_parser('select');s.add_argument('--candidate',type=Path,required=True);s.add_argument('--map',type=Path,required=True);s.add_argument('--output',type=Path,required=True)
    c=sub.add_parser('capture');c.add_argument('--selection',type=Path,required=True);c.add_argument('--session',type=Path,required=True);c.add_argument('--capture-live',action='store_true',required=True)
    r=sub.add_parser('register');r.add_argument('--session',type=Path,required=True);r.add_argument('--landmarks',type=Path,required=True)
    plan=sub.add_parser('plan');plan.add_argument('--selection',type=Path,required=True);plan.add_argument('--session',type=Path,required=True);plan.add_argument('--mode',choices=['critical','full'],default='critical');plan.add_argument('--capture-live',action='store_true',required=True)
    e=sub.add_parser('run');e.add_argument('--selection',type=Path,required=True);e.add_argument('--session',type=Path,required=True);e.add_argument('--mode',choices=['critical','full'],default='critical');e.add_argument('--execute-user-run',action='store_true',required=True)
    a=p.parse_args(argv)
    try:
        if a.command=='select':
            path=select_candidate(a.candidate,a.map,a.output);print(path);return 0
        if a.command=='register':
            c=json.loads((a.session/'capture.json').read_text());pairs=json.loads(a.landmarks.read_text())
            if pairs.get('live_map_sha256')!=c['live_map_sha256']:raise ValueError('Landmarks refer to another live map')
            cert=registration(c,{k:pairs[k] for k in ('saved','live')},alignment_p95=c['alignment_p95_m'])
            write_new(a.session/'registration.json',cert);print(a.session/'registration.json');return 0
        selection=load_selection(a.selection)
        from stage4_route.ros_transport import RosTransport
        if a.command=='capture':
            a.session.mkdir(parents=True,exist_ok=False);transport=RosTransport(a.session);capture=None
            try:capture=transport.idle_capture(a.selection)
            finally:
                clean=transport.close();write_new(a.session/'capture-cleanup.json',transport.audit)
            if not clean:raise ValueError('Read-only capture cleanup/stationary evidence failed')
            write_new(a.session/'capture.json',capture);print(a.session/'capture.json');return 0
        from stage4_route.runner import run_route
        c=json.loads((a.session/'capture.json').read_text())
        if c['selection_sha256']!=digest(a.selection):raise ValueError('Selection differs from registered session')
        if a.command=='plan':
            transport=RosTransport(a.session)
            try:
                transport.ready(selection,c);transport.preflight(selection['goals'][:2 if a.mode=='critical' else 5])
            finally:
                clean=transport.close();write_new(a.session/('plan-audit-'+uuid4().hex+'.json'),dict(cleanup_passed=clean,session_id=c['session_id'],runtime=transport.audit))
            if not clean:raise ValueError('Planner-only cleanup failed')
            return 0
        # Persistent consumption marker forbids retries/reuse even after interruption.
        write_new(a.session/'execution-lock.json',dict(session_id=c['session_id'],mode=a.mode,selection_sha256=digest(a.selection)))
        transport=RosTransport(a.session)
        report=run_route(selection,c,transport,count=2 if a.mode=='critical' else 5,
                         wall_limit=120 if a.mode=='critical' else 360,sim_limit=90 if a.mode=='critical' else 240)
        report['registration_sha256']=digest(a.session/'registration.json')
        write_new(a.session/'result.json',report)
        write_new(a.session/'evidence-manifest.json',dict(session_id=c['session_id'],files={f.name:digest(f) for f in a.session.iterdir() if f.is_file()}))
        print(a.session/'result.json');return 0 if report['practical_route_pass'] else 1
    except (ValueError,KeyError,OSError,RuntimeError) as exc:
        if getattr(a,'session',None) and a.session.is_dir():
            write_new(a.session/('rejection-'+uuid4().hex+'.json'),dict(command=a.command,error=str(exc)))
        print('Stage 4 rejected: '+str(exc),file=sys.stderr);return 2


if __name__=='__main__':sys.exit(main())
