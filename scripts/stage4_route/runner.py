"""Bounded single-action orchestration with unconditional owned-action cleanup."""
import time
from .evidence import RouteEvidence


def run_route(selection,session,transport,*,count,wall_limit=120.,sim_limit=90.):
    evidence=None;cleanup=False;error=None;terminal=False;started=time.monotonic()
    try:
        transport.run_wall_limit=wall_limit
        start=transport.ready(selection,session)
        evidence=RouteEvidence(selection['goals'],start=[start['x'],start['y']],count=count)
        first_sim=start['sim'];terminal_wall=None
        transport.send(selection['goals'][:count],selection['execution'])
        while time.monotonic()-started<wall_limit:
            item=transport.tick()
            if item.get('failure'):evidence.fail(item['failure'])
            if item.get('sample'):
                evidence.observe(item['sample'])
                if item['sample']['sim']-first_sim>sim_limit:evidence.fail('sim_timeout')
            if item.get('status') is not None and not terminal:
                evidence.terminal(item['status'],sim=transport.sim());terminal=True;terminal_wall=time.monotonic()
            if evidence.failure:break
            if terminal:
                if evidence.finish(cleanup=True,safety_complete=False)['practical_route_pass']:break
                if time.monotonic()-terminal_wall>30.:evidence.fail('terminal_stop_or_regions_missing');break
        else:evidence.fail('wall_timeout')
    except KeyboardInterrupt:
        error='interrupted'
        if evidence:evidence.fail(error)
    except Exception as exc:
        error=repr(exc)
        if evidence:evidence.fail(error)
    finally:
        if evidence:transport.cleanup_observer=evidence.observe
        try:cleanup=transport.close()
        except BaseException as exc:error=(error or '')+' cleanup: '+repr(exc)
    if evidence and getattr(transport,'failure',None):evidence.fail(transport.failure)
    report=(evidence.finish(cleanup=cleanup,safety_complete=False) if evidence else
            dict(result='ROUTE_FAIL',full_route_pass=False,practical_route_pass=False,cleanup_passed=cleanup))
    report.update(session_id=session['session_id'],error=error,wall_duration_s=time.monotonic()-started,
                  simulated_duration_s=(evidence.last['sim']-first_sim if evidence and evidence.last else None),
                  selection=selection,runtime=transport.audit)
    return report
