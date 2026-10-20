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
# This source runner must use its matching private Stage4 package, not an old underlay.
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'course_navigation'))
from stage4_route.contract import select_candidate,load_selection,digest,object_hash,write_new,registration


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    s=sub.add_parser('select');s.add_argument('--candidate',type=Path,required=True);s.add_argument('--map',type=Path,required=True);s.add_argument('--output',type=Path,required=True)
    c=sub.add_parser('capture');c.add_argument('--selection',type=Path,required=True);c.add_argument('--session',type=Path,required=True);c.add_argument('--capture-live',action='store_true',required=True)
    r=sub.add_parser('register');r.add_argument('--session',type=Path,required=True);r.add_argument('--landmarks',type=Path,required=True)
    plan=sub.add_parser('plan');plan.add_argument('--selection',type=Path,required=True);plan.add_argument('--session',type=Path,required=True);plan.add_argument('--mode',choices=['segment','critical','full'],default='critical');plan.add_argument('--capture-live',action='store_true',required=True)
    e=sub.add_parser('run');e.add_argument('--selection',type=Path,required=True);e.add_argument('--session',type=Path,required=True);e.add_argument('--mode',choices=['segment','critical','full'],default='critical');e.add_argument('--execute-user-run',action='store_true',required=True)
    for parser in (plan,e):parser.add_argument('--segment',choices=['start-g1','g1-g2','g2-g3','g3-g4','g4-g5'])
    e.add_argument('--deadline-evidence',type=Path,help='Same-profile measured achieved-rate and certified path metrics JSON')
    for parser in (c,plan,e):parser.add_argument('--deployment',type=Path,help='Explicit permitted external session deployment JSON')
    a=p.parse_args(argv)
    try:
        if a.command=='run':
            from stage4_route.motion_qualification import require_motion_qualification
            require_motion_qualification()
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
            a.session.mkdir(parents=True,exist_ok=False);transport=RosTransport(a.session,deployment=a.deployment);capture=None
            try:capture=transport.idle_capture(a.selection)
            finally:
                clean=transport.close();write_new(a.session/'capture-cleanup.json',transport.audit)
            if not clean:raise ValueError('Read-only capture cleanup/stationary evidence failed')
            write_new(a.session/'capture.json',capture);print(a.session/'capture.json');return 0
        from stage4_route.runner import run_route
        c=json.loads((a.session/'capture.json').read_text())
        if c['selection_sha256']!=digest(a.selection):raise ValueError('Selection differs from registered session')
        from stage4_route.constraints import scope,deadline_for
        start_index,count=scope(a.mode,a.segment)
        if a.command=='plan':
            transport=RosTransport(a.session,deployment=a.deployment)
            try:
                transport.segment_start_index=start_index
                transport.ready(selection,c);transport.preflight(selection['goals'][start_index:start_index+count])
            finally:
                clean=transport.close();write_new(a.session/('plan-audit-'+uuid4().hex+'.json'),dict(cleanup_passed=clean,session_id=c['session_id'],runtime=transport.audit))
            if not clean:raise ValueError('Planner-only cleanup failed')
            return 0
        # Persistent consumption marker forbids retries/reuse even after interruption.
        write_new(a.session/'execution-lock.json',dict(session_id=c['session_id'],mode=a.mode,segment=a.segment,selection_sha256=digest(a.selection)))
        metrics=json.loads(a.deadline_evidence.read_text())
        if metrics['session_id']!=c['session_id'] or metrics['selection_sha256']!=digest(a.selection):raise ValueError('Deadline evidence differs from session/profile')
        limits=deadline_for(metrics['length_m'],measured_speed=metrics['achieved_speed_lower_bound_m_s'],
            rotation_rad=metrics['rotation_rad'],measured_yaw_rate=metrics['achieved_yaw_rate_lower_bound_rad_s'],mode=a.mode)
        transport=RosTransport(a.session,deployment=a.deployment)
        report=run_route(selection,c,transport,count=count,start_index=start_index,wall_limit=limits['wall_s'],sim_limit=limits['sim_s'])
        report['registration_sha256']=digest(a.session/'registration.json')
        write_new(a.session/'result.json',report)
        write_new(a.session/'evidence-manifest.json',dict(session_id=c['session_id'],files={f.name:digest(f) for f in a.session.iterdir() if f.is_file()}))
        print(a.session/'result.json');return 0 if report['practical_route_pass'] else 1
    except (ValueError,KeyError,OSError,RuntimeError) as exc:
        if getattr(a,'session',None) and a.session.is_dir():
            write_new(a.session/('rejection-'+uuid4().hex+'.json'),dict(command=a.command,error=str(exc)))
        print('Stage 4 rejected: '+str(exc),file=sys.stderr);return 2


if __name__=='__main__':sys.exit(main())
