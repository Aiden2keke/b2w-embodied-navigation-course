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
