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


def execution_exit_code(report,mode):
    """0 formal acceptance; 3 practical diagnostic only; 1 failed execution."""
    key='full_route_pass' if mode=='full' else 'segment_pass'
    result='FULL_ROUTE_PASS' if mode=='full' else 'SEGMENT_PASS'
    if (report.get(key) is True and report.get('result')==result and
            report.get('cleanup_passed') is True and not report.get('error')):return 0
    return 3 if report.get('practical_route_pass') is True else 1


def execution_prerequisites(args,capture):
    """Read-only pre-attempt validation; live ready/dispatch rechecks still apply."""
    from stage4_route.constraints import deadline_for
    if args.deadline_evidence is None:raise ValueError('--deadline-evidence is required for execution')
    metrics=json.loads(args.deadline_evidence.read_text())
    if metrics['session_id']!=capture['session_id'] or metrics['selection_sha256']!=digest(args.selection):
        raise ValueError('Deadline evidence differs from session/profile')
    limits=deadline_for(metrics['length_m'],measured_speed=metrics['achieved_speed_lower_bound_m_s'],
        rotation_rad=metrics['rotation_rad'],measured_yaw_rate=metrics['achieved_yaw_rate_lower_bound_rad_s'],mode=args.mode)
    from stage4_route.contract import check_registration
    import time
    cert=json.loads((args.session/'registration.json').read_text())
    check_registration(cert,capture,wall=time.time(),sim=capture['sim'])
    from stage4_route.deployment import load_deployment,verify_controller
    from course_navigation.stage4_control import read_control
    deployment=load_deployment(args.deployment or args.session/'deployment.json')
    guard=read_control(deployment['guard_manifest'])
    if (deployment['session_id']!=capture['session_id'] or guard['session_id']!=capture['session_id'] or
            digest(deployment['guard_manifest'])!=deployment['guard_manifest_sha256']):
        raise ValueError('Deployment differs from session/guard')
    policy=json.loads((Path(__file__).resolve().parents[1]/'course_navigation/config/stage4_runtime_identity.json').read_text())
    verify_controller(deployment,policy)
    return limits


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
        limits=execution_prerequisites(a,c)
        # Atomic persistent marker only after read-only prerequisites pass.
        # Any interruption from this boundary still consumes the attempt.
        write_new(a.session/'execution-lock.json',dict(session_id=c['session_id'],mode=a.mode,segment=a.segment,selection_sha256=digest(a.selection)))
        transport=RosTransport(a.session,deployment=a.deployment)
        report=run_route(selection,c,transport,count=count,start_index=start_index,wall_limit=limits['wall_s'],sim_limit=limits['sim_s'])
        report['registration_sha256']=digest(a.session/'registration.json')
        write_new(a.session/'result.json',report)
        write_new(a.session/'evidence-manifest.json',dict(session_id=c['session_id'],files={f.name:digest(f) for f in a.session.iterdir() if f.is_file()}))
        print(a.session/'result.json');return execution_exit_code(report,a.mode)
    except (ValueError,KeyError,TypeError,OSError,RuntimeError) as exc:
        if getattr(a,'session',None) and a.session.is_dir():
            write_new(a.session/('rejection-'+uuid4().hex+'.json'),dict(command=a.command,error=str(exc)))
        print('Stage 4 rejected: '+str(exc),file=sys.stderr);return 2


if __name__=='__main__':sys.exit(main())
