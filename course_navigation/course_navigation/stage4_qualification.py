"""Fail closed while the experimental control/abort path is unqualified.

There is deliberately no flag, environment variable or JSON boolean override.
Replace this interlock only with a reviewed implementation that enforces the
per-run physical gates; unit tests or an operator motion flag cannot do that.
"""


def require_motion_qualification():
    raise ValueError(
        'UNQUALIFIED_FOR_MOTION: predictive adapter veto, action-independent '
        'owned abort, sensor-only bounded registration and all-body/end-watermarked '
        'physical evaluation are not qualified; see docs/LUNA_VALIDATION_HANDOFF.md')


def validate_e1_pulse_request(request,*,session_id=None):
    """Read-only request accounting, never qualification or approval ingestion.

    Hashes bind supporting bytes; they do not establish conservative physical
    bounds, freshness, independent containment or acceptance by a reviewer.
    The unconditional motion veto above remains the only execution decision.
    """
    import hashlib
    import json
    import math
    from pathlib import Path

    def finite(value):
        try:return type(value) in (int,float) and math.isfinite(value)
        except OverflowError:return False

    def pinned(reference):
        if type(reference) is not dict or set(reference)!={'path','sha256'}:
            raise ValueError('Explicit evidence path and SHA256 required')
        path=reference['path'];sha=reference['sha256']
        if (type(path) is not str or not Path(path).is_absolute() or
                type(sha) is not str or len(sha)!=64 or any(c not in '0123456789abcdef' for c in sha)):
            raise ValueError('Absolute evidence path and lowercase SHA256 required')
        file=Path(path)
        if file.is_symlink() or not file.is_file():raise ValueError('Regular nonsymlink evidence file required')
        data=file.read_bytes()
        if hashlib.sha256(data).hexdigest()!=sha:raise ValueError('E1 evidence content identity changed')
        return data

    keys={'kind','version','mode','session_id','attempt_id','command','pulse_source_s','rest_source_s',
          'rest_wall_s','attempt_source_s','attempt_wall_s','scope','zero_only_result','evidence'}
    if type(request) is not dict or set(request)!=keys:raise ValueError('Exact offline E1 request fields required')
    r=request
    if (r['kind']!='stage4_e1_pulse_request' or type(r['version']) is not int or r['version']!=1 or
            r['mode']!='e1_forward_pulse' or
            any(type(r[k]) is not str or not r[k].strip() for k in ('session_id','attempt_id')) or
            (session_id is not None and r['session_id']!=session_id)):
        raise ValueError('New session-bound single forward pulse request required')
    command=r['command'];expected=dict(linear_x=.05,linear_y=0.,linear_z=0.,angular_x=0.,angular_y=0.,angular_z=0.)
    if (type(command) is not dict or set(command)!=set(expected) or
            any(not finite(command[k]) or command[k]!=v for k,v in expected.items())):
        raise ValueError('Only the exact six-axis .05 forward pulse may be described')
    times=[r[k] for k in ('pulse_source_s','rest_source_s','rest_wall_s','attempt_source_s','attempt_wall_s')]
    if (not all(finite(v) for v in times) or not 0<r['pulse_source_s']<=2. or
            r['rest_source_s']<5. or r['rest_wall_s']<3. or
            not r['pulse_source_s']+r['rest_source_s']<=r['attempt_source_s']<=90. or
            not r['rest_wall_s']<=r['attempt_wall_s']<=180.):
        raise ValueError('Bounded pulse, measured-rest windows and whole-attempt ceilings required')
    scope=r['scope']
    if type(scope) is not dict or set(scope)!={'valid_from_sim','valid_until_sim','region'}:
        raise ValueError('Explicit local spatial and source-time scope required')
    region=scope['region'];start=scope['valid_from_sim'];end=scope['valid_until_sim']
    if (type(region) is not list or len(region)!=4 or not all(finite(v) for v in [start,end,*region]) or
            not 0<=start<end or end-start<r['attempt_source_s'] or
            region[0]>=region[2] or region[1]>=region[3]):
        raise ValueError('Finite nonempty pocket and scope covering the whole attempt required')
    evidence=r['evidence']
    if type(evidence) is not dict or set(evidence)!={'bootstrap_envelope','independent_abort','local_clearance'}:
        raise ValueError('Pinned bootstrap, independent abort and local-clearance references required')
    for reference in evidence.values():pinned(reference)
    zero=json.loads(pinned(r['zero_only_result']))
    if (type(zero) is not dict or zero.get('kind')!='stage4_zero_session' or
            zero.get('zero_motion_smoke_pass') is not True or zero.get('motion_authorized') is not False or
            type(zero.get('cleanup')) is not dict or zero['cleanup'].get('passed') is not True):
        raise ValueError('Full ZeroOnly probe and owned cleanup PASS required')
    return dict(request_valid=True,motion_qualified=False,motion_authorized=False,
        limitation='Request shape/content identities only; evidence semantics, freshness, containment, '
                   'conservative local bounds, deployment identity and single-use enforcement remain unqualified')
