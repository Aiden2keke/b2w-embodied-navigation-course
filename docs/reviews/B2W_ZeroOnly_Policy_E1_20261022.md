# Reviewed ZeroOnly deadline policy + ONE Warehouse trial — 2026-10-22

**Full live probe FAILED. NONZERO NO-GO. No fix/retry after failure.**
Repository `/home/zyd/program/b2w-embodied-navigation-course-stage4`, branch
`experiment/unattended-g1-g5-20261019`, unchanged HEAD
`4ae85f4148a89a76b1ad25c5b059c2140be41b85`. Working changes remain uncommitted.
User communicated external architecture **CONDITIONAL PASS** for the immutable
ZeroOnly-only policy and expressly authorized one bounded trial after preflight.
This records that communicated decision, not a fabricated external approval.

## Exact implementation and offline evidence

Only runtime source changed: `course_navigation/course_navigation/velocity_adapter.py`.
Prepublish>=50ms still captures first overrun wall/thread CPU timing and forces an
exact-zero Twist; sticky `adapter_publication_deadline` is conditional on
`guard.zero_only is not True` (:339–342). Post-health>=50ms retains timing;
`adapter_health_publication_deadline` plus its immediate zero publish remains
identical for motion mode, conditional on that same fixed flag (:363–364).
No threshold, timer, owner, lease, freshness, contacts, guard geometry,
qualification function, controller or harness changed. No runtime parameter or
`armed=False` exemption added. Existing ctor-captured ZeroOnly mode never arms or
leases; its existing output path returns zero independently of input command.
Unexpected nonzero inputs and leases still permanently inhibit.

Tests changed only in `tests/test_stage4_startup_repair.py` and
`tests/test_stage4_health_io.py`. Tests distinguish fixed ZeroOnly from unarmed
motion mode, delay/simulated CPU load, retained first timing after recovery,
health-write delay versus actual write/rename errors, and owner/lease/nonzero
faults after an overrun. Existing zero-boundary, watchdog, malformed localization,
stale-health and hard nonzero interlock regressions also ran.

TDD RED: **7 failed/21 passed** against old source, all expected ZeroOnly policy
failures. GREEN: **71 passed in1.56s**, no ROS node/context or Gazebo in this set.
Exact selected suite (sourced health-io candidate env, system Python):

```bash
/usr/bin/python3 -m pytest -q -p no:cacheprovider \
  --basetemp=/tmp/b2w-zero-policy-green \
  tests/test_stage4_startup_repair.py tests/test_stage4_health_io.py \
  tests/test_stage4_zero_boundary.py tests/test_unqualified_motion.py \
  tests/test_navigation_watchdog.py
```

Independent static code review found no blocking defect before the live trial.
Health-stage first-overrun timing becomes visible on the next successful health
callback, because the current JSON was already serialized when that delay is
measured. Immediate shutdown may lose that final diagnostic; no same-callback
persistence claim. Review also noted waiting-TF/localization+delay combinations
are not newly tested; their unchanged zero/fault paths have existing coverage.

Regular isolated `course_navigation` copy build PASS in0.985s, under
`experiments/g1_g5/evidence/zero-policy-20261022/artifacts/install/`.
Preflight checked35 regular copied Python/launch/config files (including existing
bringup underlay), fresh import resolution to the new install, pinned native LIO,
GTSAM, contact observer/world, policy and reviewed controller. No fallback links,
downloads or dependency changes. Domain197 idle; display/GPU preflight PASS.
Actual runtime adapter argv names this new install; actual LIO executable SHA
and GTSAM mappings match preflight. Copy identities are in `raw/preflight-host.json`.

## Exact single execution and useful progress

From Bash, repository root, using existing limits:

```bash
source experiments/g1_g5/evidence/zero-policy-20261022/candidate-env.bash
/usr/bin/python3 experiments/g1_g5/evidence/zero-policy-20261022/raw/run_once.py
```

That retained one-shot wrapper starts the existing passive recorder, then exactly:

```bash
/usr/bin/python3 scripts/stage4_session.py \
  --inputs experiments/g1_g5/evidence/startup-repair-20261021/zero-inputs-candidate.json \
  --output experiments/g1_g5/evidence/zero-policy-20261022/raw/session \
  --execute-zero-smoke
```

Authoritative absolute argv, owner identities and token are in `raw/session/invocation.json`;
exclusive `raw/attempt.json` proves one attempt. No retry. Zero-only guard true,
no lease/goal/nonzero publication by the task, hard production motion interlock
still raises. Original Warehouse observer world retained; no GT localization.

| Evidence | Observed result |
| --- | --- |
| Old50ms barrier | Recovered after prepublish **82.903ms wall /1.337ms thread CPU**; ready health retained and no inhibit for that reason. |
| Bootstrap | Three prepared live/local/global grids; structural budgets7.5s/7.511243s. Ready cadence observed monotonic347220.833078; watchdog and probe created. This is new progress beyond all preceding trials. |
| Full probe | Started, **FAILED**; `first_sample:null`, `accepted_end_sim:null`, no stationary accepted interval or endpoint closure. |
| Final Twist | **716/716 received samples all six axes exact zero**; raw command1 received zero; smoothed command not observed. Best-effort recorder is not lossless proof. |
| LIO-SAM | **162 positive imu_step updates**, deltaTij approximately0.1s. No observed update_failed/b0 exception; historical b0 root cause still unresolved. |
| SLAM/map |115 odometry messages,11.4 source seconds;3 structural point maps;12 occupancy maps;19 local and6 global raw costmaps. Sustained runtime parameter/graph verification was not completed by probe. |
| Contacts |592 contacts+592 inventory messages; paired delivered sequences1–592 contiguous; four foot/ground pairs; ever_forbidden=false. Full-body engine coverage remains unqualified. |
| Stationary estimate | Max SLAM distance from first0.000704m over11.4s. This is limited localization evidence, not whole-trial physical stationarity or full probe PASS. |
| Time/cleanup | Live stack47.665s, wrapper48.381s; existing220s supervisor/90s bootstrap/150s watchdog/90+35s probe/10+5+3 cleanup bounds unchanged. Owned cleanup PASS. |

## First remaining real blocker — preserved watchdog health boundary

First inhibit file reason is **`adapter_publication_stalled`**, observed monotonic
**347222.359472**, latest received sim30.2575. This comes from the unchanged
`course_navigation/course_navigation/stage4_control.py:97–100` health predicate,
after watchdog/probe started. Watchdog exited; supervisor saw
`Owned watchdog/stack/evaluation bridge exited` and stopped the owned session.
This is **not** another50ms sticky callback fault and **not** a probe pass.

The predicate jointly checks session/adapter identity, positive sequence, finite
clock values and **0<=sampled_wall-health_callback_start<=.15**. Watchdog did not
save the exact record and wall value it rejected, so do not claim the exact failed
subcondition or prove stale age>150ms from the error string alone. The nearby
final command receiver gap is **159.635ms** (347222.130601→347222.290236), consistent
with a real scheduling/delivery interruption; it cannot identify callback versus
DDS/receiver scheduling. Latest observer health before fault was sequence713,
callback start347222.129119, observed347222.153931. Next sampled health716 is
already faulted. Intermediate714/715 and watchdog's read are not captured.
Also consider the clock-before-read ordering when reviewing possible negative
ages; no cause is established. No threshold increase, watchdog removal, new
framework, source repair or live repeat was made after this failure.

Probe was interrupted during effective-parameter checks (only planner parameters
recorded); it later reports invalid rcl context and failed stationary cleanup.
Its journal seals100/100 entries, but `cleanup_passed:false`. These are subordinate
to the earlier watchdog inhibit; owned OS-process cleanup PASS does not turn
probe cleanup into PASS. Adapter and map_projector also log invalid ROS context
on shutdown. LIO/controller were not the first fatal marker.

**Next review decision:** resolve this specific health predicate/read-order versus
actual publication interruption using preserved samples and a small offline
reproduction if authorized; retain150/250ms limits. Do not blindly repeat Gazebo
or infer that removing50ms failure alone qualified the control stack. E1 remains
blocked additionally on independent stop/abort and accepted local bounds below.

## Minimal E1 and fastest Start→G1 path

Concrete code-entry-point plan is in the **single PLAN.md NOW**, subsection
“Minimal E1 implementation plan”; no second active plan or motion implementation.

1. Review a conservative bootstrap envelope for one open Warehouse pocket. The
   present `stage4_bounds.validate_bounds` requires measured stops: historical
   forward-stop .079872m is provisional, not an accepted first-pulse bound.
2. Extend existing `stage4_session.py` / `stage4_route/session.py` for a separate
   immutable E1 session; replace unconditional qualification only with scoped,
   validated per-run evidence at all existing nonzero boundaries. No flag bypass
   or mutation of a running ZeroOnly session.
3. Extend existing watchdog/supervisor for identity-pinned wall-time Gazebo
   containment if adapter/controller dies or sim clock freezes. Current inhibit
   and sim-time rl_sar timeout alone cannot prove a stop then. Containment is
   emergency FAIL, never accepted stationary success; review and zero-only fault
   demonstration precede E1. No alternate velocity publisher.
4. Same-session sensor-only local registration, all required local/saved/live/global
   grid support, unknown blocking, conservative translation/yaw/body/tracking/
   latency/stop reserve, physical evaluation coverage and complete contact intervals.
   No G2–G5 corridor prerequisite; no GT registration.
5. Existing adapter input → final guard → sole final Twist → unchanged rl_sar.
   One proposed .05/0/0 pulse<=2sim s, then measured rest>=5source/3wall s,
   one attempt<=90sim/180wall s. No goal/turn/speed escalation. Actual speed is
   measured, not assumed equal to policy command.

The proposed extension would eventually accept this copy-ready command form,
**NOT implemented or authorized today**:

```bash
/usr/bin/python3 scripts/stage4_session.py \
  --inputs "$reviewed_e1_inputs" --qualification "$reviewed_e1_qualification" \
  --output "$fresh_e1_evidence" --execute-e1
```

PLAN.md specifies artifact contents, measurement/abort prerequisites and entry
points. Numerical safe region/clearance/bounds must come from accepted evidence;
no honest presently executable qualified E1 command exists. After fresh ZeroOnly
PASS and qualified E1, use measured bounds plus path-specific turning/registration/
clearance and a separately authorized existing Start→G1 segment command. Preserve
original G1 and .25m region. Then fresh G1 corridor observations and separate
G1→G2→G3→G4→G5 segments before a continuous run.

## Preservation and artifacts

Evidence root: `experiments/g1_g5/evidence/zero-policy-20261022/`.
`raw/` and `artifacts/` are Git-ignored local evidence, not remotely backed up.
`summary.json`, build/test/review records and checksum index are compact review data.
Raw contains the existing wrapper/observer, JSONL, invocation, all stack/watchdog/
probe logs and SQLite journal. One concise APRIL bundle is copied to the temporary
exchange directory, with a durable identical repository copy.

Independent cleanup check: **27 PID/start identities and both owner tokens absent**,
domain197 idle.525 frozen repository files unchanged during live trial. Original
checkout397 audited files and both Pipe worktrees342 each unchanged; inherited
trial58 checksum entries verified. No foreign process signaled. No commit/push,
nonzero motion, goal, lease, system upgrade or destructive Git operation.
