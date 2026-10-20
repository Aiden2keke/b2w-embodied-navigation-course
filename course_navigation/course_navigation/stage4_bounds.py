"""Conservative reserve accounting. Structural validity is not qualification.

The current external review/physical interlock remains unconditional. A p95 or
landmark residual is never accepted as a physical worst-case error bound.
"""
import math

COMPONENTS=('registration_translation_m','registration_yaw_rad','registration_radius_m',
            'map_boundary_m','tracking_translation_m','tracking_yaw_rad','body_extent_m')


def validate_bounds(config,evidence):
    e=evidence;c=config
    if (e['kind']!='stage4_conservative_bounds' or e['version']!=1 or e['session_id']!=c['session_id'] or
            e['sensor_only_registration'] is not True or e['worst_case_bound'] is not True):
        raise ValueError('Explicit sensor-supported conservative bounds required; residual/p95 is diagnostic only')
    b=e['components'];measured=e['measured'];scope=e['scope']
    def positive(v):return type(v) in (int,float) and math.isfinite(v) and v>0
    if any(not positive(b[k]) for k in COMPONENTS):raise ValueError('All error components need independently justified positive bounds')
    if (not all(type(v) in (int,float) and math.isfinite(v) for v in
            [scope['valid_from_sim'],scope['valid_until_sim'],*scope['region']]) or
            not 0<=scope['valid_from_sim']<scope['valid_until_sim'] or len(scope['region'])!=4 or
            scope['region'][0]>=scope['region'][2] or scope['region'][1]>=scope['region'][3]):raise ValueError('Finite spatial and temporal evidence scope required')
    for name in ('stop_translation_m','stop_yaw_rad','abort_latency_s','max_speed','max_wz'):
        if not positive(measured[name]) or not positive(c[name]):raise ValueError('Complete measured stopping envelope required')
        if name.startswith('max_'):
            if c[name]>measured[name]:raise ValueError('Guard envelope exceeds calibrated range')
        elif c[name]<measured[name]:raise ValueError('Declared bound is below measured excursion/latency')
    radius=math.hypot(.75,.43)
    minimum=(b['registration_translation_m']+b['registration_radius_m']*b['registration_yaw_rad']+
        b['map_boundary_m']+b['tracking_translation_m']+radius*b['tracking_yaw_rad']+b['body_extent_m'])
    if not positive(c['uncertainty_m']) or c['uncertainty_m']<minimum:raise ValueError('Combined uncertainty exceeds declared reserve')
    if not e['raw_evidence']:raise ValueError('Hashed raw measurement references required')
    for record in e['raw_evidence']:
        if not record['path'].startswith('/') or len(record['sha256'])!=64 or any(x not in '0123456789abcdef' for x in record['sha256']):raise ValueError('Raw evidence identity missing')
    return dict(minimum_uncertainty_m=minimum,scope=scope,motion_qualified=False,
                limitation='Schema/accounting only; independent raw replay, physical coverage and external review still required')
