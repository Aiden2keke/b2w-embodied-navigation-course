# APRIL Review Bundle — B2-W ZeroOnly Attempt (2026-10-23)

## Outcome

The one-shot ZeroOnly harness attempt **failed before Gazebo session launch**. It is not a full ZeroOnly probe pass. The harness returned 2 in 0.719s on its input-record permission check. No retry was made.

First blocker: `Control record must be an owned, non-writable-by-others regular file`. The new per-run input was created mode `0664` because the shell umask was `0002`; group-write violates `read_control()`'s existing `mode & 0o022` check. This is a fail-closed rejection. The previous pinned input is mode 0600.

```python
# course_navigation/course_navigation/stage4_control.py:31-33
mode = os.fstat(fd)
if not stat.S_ISREG(mode.st_mode) or mode.st_uid != os.getuid() or mode.st_mode & 0o022:
    raise ValueError("Control record must be an owned, non-writable-by-others regular file")
```

```python
# scripts/stage4_session.py:35-36
value = read_control(path)
if value['kind'] != 'stage4_zero_session_inputs' or value['version'] != 2: ...
```

The exact one-shot attempt is `5f0e87afa2b142838a7d743ca5155573` with session `zero-wd-2a9ad1bea4064830b502b353e79fb172`. Input, wrapper, observer, harness output, preflight and empty observation are preserved under `experiments/g1_g5/evidence/zero-policy-20261023/raw/`.

## Preflight and execution evidence

- Commit `0c83ab4a513143907ab8ba964cfe5c07a5454153` was pushed normally to `experiment/unattended-g1-g5-20261019`; `git ls-remote` returned the same SHA.
- Preflight passed: ROS domain 197 idle; no previous owner/runtime process; only its temporary preflight ROS node; 35 source/config/launch copy hashes matched; world, LIO-SAM, GTSAM, contact bridge and controller pins matched; hard motion qualification still rejected nonzero motion.
- Installed `stage4_control.py` SHA matched source (`4c9e32b169dbe8d0ced5ea83cbcc8bed1ca99c96c139cac5c20c74d53e2525be`). Reviewed watchdog script SHA also matched (`eef423424643bfb39ae4898ddcf14e6ff3c048892133417601be0605b3380475`), but it was not launched.
- Harness exit 2; passive observer exit 0 with no topics. No session directory, Gazebo, watchdog, probe, final Twist, contact sample, lease, goal or command publisher was created. Owned cleanup PASS; targeted process check found no session processes.
- Full stationary, stop, localization/TF/Nav2, zero-command, contact interval/closure and probe cleanup criteria are **unobserved**.

Raw evidence manifest SHA256: `3add313967c4dca9abea307e06788c3f3915a7e794340f7091229757a09a09df`. Compact evidence index: `experiments/g1_g5/evidence/zero-policy-20261023/index.json`.

## Next step and motion status

Stop for independent review. If a later attempt is authorized, create a **new** 0600 input via the existing atomic control writer, use fresh session/attempt IDs, and then run the same bounded harness once. Do not reuse this attempt. No source, threshold or interlock change is justified. E1 and Start→G1 remain NO-GO; the existing E1 plan is unchanged.

## Checkout

Branch `experiment/unattended-g1-g5-20261019`; local and remote HEAD `0c83ab4a513143907ab8ba964cfe5c07a5454153`. The failed-attempt report, evidence index and this report's supporting evidence are uncommitted local records. Current worktree:

```text
## experiment/unattended-g1-g5-20261019...origin/experiment/unattended-g1-g5-20261019
 M PLAN.md
?? docs/reviews/B2W_ZeroOnly_Trial_Permission_Block_20261023.md
?? experiments/g1_g5/evidence/zero-policy-20261023/
```
