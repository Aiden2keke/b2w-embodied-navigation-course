# ZeroOnly qualification attempt — permission rejection, 2026-10-23

## Outcome

The authorized one-shot wrapper was invoked, but the Gazebo session did not start. The harness returned exit code 2 in 0.719s during input validation. This is a **failed ZeroOnly qualification attempt**, not a full probe run or a watchdog failure. No retry was performed.

The first and decisive blocker was:

```text
Stage4 zero-only session rejected: Control record must be an owned, non-writable-by-others regular file
```

`scripts/stage4_session.py` reads the input with `read_control()`; [stage4_control.py](/home/zyd/program/b2w-embodied-navigation-course-stage4/course_navigation/course_navigation/stage4_control.py) rejects control files whose mode has group/world write bits (`mode & 0o022`). The new input was created by `Path.write_text()` while the invoking shell had `umask 0002`, yielding mode `0664` (owner 1000, group 1000). The pre-existing pinned input is mode `0600`. This permission check correctly stopped launch before any Gazebo or robot process.

## What ran

- Pushed checkpoint: `0c83ab4a513143907ab8ba964cfe5c07a5454153` on `experiment/unattended-g1-g5-20261019`; remote SHA was verified equal.
- Fresh session ID: `zero-wd-2a9ad1bea4064830b502b353e79fb172`.
- Exclusive one-shot attempt ID: `5f0e87afa2b142838a7d743ca5155573`.
- Preflight passed on commit `0c83ab4`: domain197 idle, no previous owner/runtime process, no ROS nodes other than the preflight node, pinned world/controller/LIO/GTSAM/contact assets matched, 35 source/config/launch copy identities matched, hard motion qualification still returned `UNQUALIFIED_FOR_MOTION`.
- Patched installed `stage4_control.py` SHA256 `4c9e32b169dbe8d0ced5ea83cbcc8bed1ca99c96c139cac5c20c74d53e2525be` matched source. Standalone watchdog script SHA256 `eef423424643bfb39ae4898ddcf14e6ff3c048892133417601be0605b3380475` matched the reviewed checkout version; it was **not launched**, because input validation rejected first.
- Harness exit `2`; passive observer exit `0` with no observer error or topics captured.
- No session directory, stack, watchdog or probe was created. No final Twist or contact sample was produced. No nonzero command, goal or lease was sent.
- Owned cleanup is PASS: no owned Gazebo/session process was launched, observer exited normally, and the targeted process check found no session processes.

## Full-probe acceptance

**NOT PASSED.** The watchdog and probe never started, so stationary observation, stop evidence, LIO/TF/Nav2 freshness, exact-zero commands, paired contact closure and probe cleanup are all unobserved. Do not infer them from the preflight.

## Smallest next action

Stop for independent review. If a new attempt is later authorized, create a **new** input record using owner-only mode `0600` (for example the existing `stage4_control.write_control()` atomic writer, or an explicit `os.open(..., 0o600)`), assign a fresh session and attempt ID, then rerun the same bounded harness once. Do not reuse attempt `5f0e87afa2b142838a7d743ca5155573`. No threshold, source interlock or watchdog change is justified by this failure.

## Evidence

Raw scripts, exact attempt/input, preflight result, harness output, passive observer output and wrapper result are indexed under `experiments/g1_g5/evidence/zero-policy-20261023/raw/`; `SHA256SUMS` covers the captured files. The previous October22 live watchdog failure remains distinct: its historical health predicate was not recorded.

## Checkout

Branch `experiment/unattended-g1-g5-20261019`; commit `0c83ab4a513143907ab8ba964cfe5c07a5454153`, pushed normally and verified at origin. This report and this attempt's evidence are new, uncommitted local review records. The original checkout and Pipe V0 were untouched. The existing E1 plan remains in `PLAN.md`; E1 and Start→G1 motion remain NO-GO.
