"""Frozen M2 strict diagnostic inverse. No validated low-speed moving state."""
import hashlib
import json
import math
from pathlib import Path

A = 1.0245445280935277
B = 0.0659258536489146
U_MIN = 0.03999999910593033
U_MAX = 0.20000000298023224
V_MIN = 0.10690763385664151
V_MAX = 0.27052833048110847
FROZEN_B_SHA256 = '15b371aa80afe5cef8461d077fae0bc8bb0a180ae92ea5ff6c669c9b04d352bc'
DIAGNOSTIC_TOPIC = '/course_navigation/m2_inverse_diagnostic'


class UnsupportedVelocity(ValueError):
    """Design boundary: no policy command may be substituted for this request."""


def inverse(v):
    if not math.isfinite(v):
        raise UnsupportedVelocity(f'unsupported desired vx: {v}')
    if v == 0.:
        return 0.
    if not V_MIN <= v <= V_MAX:
        raise UnsupportedVelocity(f'unsupported desired vx {v!r}; zero or [{V_MIN!r}, {V_MAX!r}] only')
    # Exact frozen lower endpoint: binary64 subtraction/division is one ulp
    # below U_MIN. Preserve that authenticated endpoint identity explicitly;
    # values even one ulp outside V_MIN are still rejected above.
    if v == V_MIN:
        return U_MIN
    u = (v - B) / A
    if not U_MIN <= u <= U_MAX:
        raise UnsupportedVelocity(f'unsupported generated policy u {u!r}')
    return u


def transform(command, enabled=False):
    if not enabled:
        return command
    x, y, wz = command
    return inverse(x), y, wz


def identity():
    payload = {'version': 'm2-strict-v1', 'a': A, 'b': B,
               'validated_condition': 'pure_forward_vy0_wz0',
               'turning_lateral_invariance_validated': False,
               'moving_u_support': [U_MIN, U_MAX], 'desired_v_domain': [V_MIN, V_MAX],
               'zero': 'exact separate state', 'lower_endpoint': 'V_MIN maps exactly to U_MIN (one binary64 ulp correction only at equality)', 'unsupported': 'raise; no substituted command',
               'frozen_b_sha256': FROZEN_B_SHA256,
               'protocol_c': '/tmp/b2w-user-calibration-C/20261017T104226682472Z',
               'validation_sha256': 'c882bba3889a5cd21a30fdbec9f46acf559518d1b0b700f4abcf44b488df3617',
               'implementation_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    return {**payload, 'sha256': hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()}
