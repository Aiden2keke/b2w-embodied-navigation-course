# E1 offline request preparation — 2026-10-23

**NONZERO NO-GO. No Gazebo launch, goal, velocity command, lease or motion authority.**
Checkpoint `cff87a8`; changes remain uncommitted. Original shared checkout and Pipe V0 untouched.

## Implemented bounded prerequisite

Existing `stage4_qualification.py` gains `validate_e1_pulse_request`: read-only request
accounting with exact schema, version/mode/session/attempt identifier, all six axes
fixed to .05/0/0 and zero remaining axes, pulse<=2 advancing-source seconds,
rest>=5source/3wall seconds, attempt<=90source/180wall seconds, finite nonempty local
region and source scope covering the entire attempt. It rejects unknown authorization
fields, booleans/nonfinite/overflowing numeric inputs and missing or changed evidence.
Absolute nonsymlink regular-file evidence references are bound by SHA256; bytes are read
once for digest and parsing. The referenced existing `stage4_zero_session` result must
claim full probe PASS, owned cleanup PASS, and motion_authorized=false.

Supporting references named `bootstrap_envelope`, `independent_abort`, `local_clearance`
are required and content-bound only. Their contents are **not** accepted as qualified
physical evidence. No bootstrap envelope is invented; no measured future stopping
number is inserted into existing `stage4_bounds.validate_bounds`. A scope rectangle
alone does not prove body containment or clearance. SHA256 is content identity, not
approval, immutability or reviewer authentication.

Existing owner CLI gains the read-only spelling:

```bash
/usr/bin/python3 scripts/stage4_session.py --inputs /absolute/request.json --check-e1-request
```

Request is a secure `read_control` input, but no output directory, session, lease,
heartbeat or inhibit is created. No ROS domain environment is needed for this check.
Success explicitly prints `NONZERO remains NO-GO`. The proposal's exact future spelling
`--qualification ... --output ... --execute-e1` is reserved and **always rejects**
through unchanged `require_motion_qualification`, before artifact reads or owner launch.
It is not an executable qualified motion command. No changes to SessionOwner acceptance,
watchdog/lease, IndependentStop, command publication, grids, or abort path.

## Tests and reproducibility

TDD: initial26 failures showed missing validator/reserved CLI; next26+existing cases
passed56/56. A new oversized numeric-input case then failed with OverflowError;
minimal finite-number handling now rejects it with ValueError. Final focused tests:
**60 passed in1.80s**, `git diff --check` PASS. Raw full outputs retained under `raw/e1*`.
All successful ZeroOnly/evidence request objects in tests are synthetic fixtures; no
real reviewed E1 request or qualification file was manufactured.

```bash
source experiments/g1_g5/evidence/zero-policy-20261022/candidate-env.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -q -p no:cacheprovider \
  --basetemp=/tmp/b2w-e1-final-20261023 \
  tests/test_stage4_e1_offline.py tests/test_unqualified_motion.py \
  tests/test_stage4_zero_boundary.py tests/test_stage4_bounds.py tests/test_stage4_session.py
```

Tests exercise real file/hash replacement, malformed/missing evidence, direct-symlink
rejection, wrong mode/session, exact six-axis ceilings, scope and stop-window arithmetic,
failed ZeroOnly/cleanup, non-authorizing successful return, environment-free check CLI,
and unconditional execution rejection with existing or missing inputs and purported
approval booleans, with no output session creation. Existing tests retain zero-only
no-lease/inhibit and transport/final-guard authority checks. AST comparison verifies the
unconditional veto body is unchanged from HEAD. Source/log SHA256 are in `e1-summary.json`.

## Remaining prerequisites and limits

This prepares request integrity and the exact rejecting future CLI only. It does not
check an expired current live scope, actual pose/body containment, copied source/install
identity, a duplicated runtime attempt, live lease loss, actual abort timing or stopping.
Those checks require the unimplemented and separately reviewed per-run owner boundary.
Fresh ZeroOnly PASS is still absent. Independently accepted conservative local bootstrap
bounds, source-supported saved/live/local/global clearance, independent wall-clock
containment with expressly authorized zero-only fault demonstrations, and complete
contacts/normal-rest measurements remain blockers. No boundary review or live qualification
is claimed. Existing installation has not been rebuilt by this offline subtask; strict
copy-identity preflight must reject it until a matching build is prepared and checked.
