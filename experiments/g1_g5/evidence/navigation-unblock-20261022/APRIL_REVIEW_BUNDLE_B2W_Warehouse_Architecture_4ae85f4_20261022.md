# APRIL Review Bundle — Warehouse architecture decision

Independent review requested. No runtime patch or live authorization.
# Warehouse navigation: architecture decision for independent review — 2026-10-22

Repository: `/home/zyd/program/b2w-embodied-navigation-course-stage4`.
Branch: `experiment/unattended-g1-g5-20261019`.
Audited HEAD: `4ae85f4148a89a76b1ad25c5b059c2140be41b85`.
Disposition: **reviewable architecture proposal; runtime unchanged; NONZERO NO-GO**.
No live experiment, goals, motion, commit, push or dependency changes in this task.
The sole active milestone is PLAN.md NOW. This report supplies its evidence and runbook.

## Decision

Reuse the existing Warehouse simulation/sensor/LIO-SAM/Nav2/adapter/rl_sar chain.
Do not build a second navigation stack or pursue another general timing framework.
The shortest justified implementation is a **reviewed ZeroOnly-specific treatment
of callback overruns**, followed by one bounded zero-start trial. Changing today's
sticky deadline semantics is a safety-boundary decision, so **no runtime patch is
applied here**, as requested. A zero-start pass is necessary evidence, not motion
qualification. The next engineering work must explicitly address the still-hard
motion qualification boundary, with a local E1/Start→G1 scope rather than requiring
completion of G2–G5 before the first motion.

Three alternatives considered:

| Path | Decision / reason |
| --- | --- |
| Continue optimising callbacks until no observed 50 ms miss | Not the preferred next step: no demonstrated worst-case guarantee on this host; fixes have exposed another startup gate without reaching the probe. |
| Start ordinary baseline with empty guard, direct Twist, or old physics script | Reject as a shortcut: it avoids current Stage4 motion qualification and owned abort checks. Historical working motion is evidence for reuse, not present authorization. |
| Retain baseline components and final guarded boundary; review a ZeroOnly-only overrun policy | Recommend: smallest change at the observed blocker, no alternate command producer, no change to motion-mode deadlines or qualifications. |

## Real capability and evidence limits

* `docs/validation/2026-10-10-rl-sar-navigation.json`, `warehouse.results[0]`:
  rl_sar Warehouse normal/seed0 map goal `(1,-1)`, success, SLAM path
  **1.567540 m**, **5.66 simulation s**, final error **0.191669 m**, zero reported
  collision-plate events. This is the relevant rl_sar result; the 1.5336 m older
  example in COURSE_VALIDATION is a different historical baseline result.
  Collision plate results do not prove all-body coverage. Initial independent
  planning measurement was aborted; do not count its latency as successful planning.
* `docs/validation/2026-10-11-init-physics.json`: Warehouse rl_sar stand/forward/
  stop/turn/stop all passed. Stand 15 s: <0.001 m displacement; forward command
  .2 for8 s:2.138 m; stop after forward max excursion .079872 m. Turn .25 for8 s:
  2.151 rad and .432 m translation; turn is not rotation in place. These are
  historical physical measurements, not certified current abort/uncertainty bounds.
* `course_navigation/config/warehouse_route.yaml:23–29`: historical Start→G1
  succeeded at original manager tolerance; G1→G2 was first known failure and later
  replaced; G3–G5 were issued after that failure. This provenance does **not** prove
  strict .25 m ordered passage, independent later segments or continuous success.
* Latest report `docs/reviews/B2W_Stage4_Health_IO_ZeroOnly_4ae85f4_20261022.md`
  and `experiments/g1_g5/evidence/health-io-zeroonly-20261022/summary.json`:
  sensors received, 51 positive LIO integration steps at .1 s, seven SLAM poses
  spanning .6 s; no observed b0 abort. Contact bridge delivered356 paired reports.
  Final received Twist502/502 exact zero. Global raw costmap not observed;
  complete probe and independent live watchdog did not start. Whole-trial physical
  stationary state, sustained localization/Nav2 readiness and stop/abort remain
  unqualified. This is a startup failure, not evidence of broken locomotion.

## Actual code chain: baseline and Stage4 delta

| Layer | Exact source | Function / observed status |
| --- | --- | --- |
| Gazebo physics → bridge | `course_sim/launch/simulation.launch.py:24–95`, `course_sim/config/bridge_rl_sar.yaml` | Original Warehouse model, torque physics, clock, sensor bridge and private feedback. Current sensors delivered. Simulation lock and launch exit handlers prevent overlapping owned sessions. |
| Public sensor processing | `course_sim/course_sim/sensor_disturbance.py`, `course_sim/setup.py:20` (cloud entrypoint uses the same sensor_disturbance module) | `/pointcloud`, `/imu`; existing fault/seed model. No truth localization. |
| LIO-SAM → public localization | `course_slam/launch/slam.launch.py:10–28`, `course_slam/course_slam/pose_adapter.py` | Four native LIO executables; internal TF isolated; public `/slam/odom`, `/slam/map_points`, map→odom→base_link. Latest integration works briefly; historical b0 remains unresolved. |
| Bringup → Nav2 | `course_bringup/launch/course.launch.py:30–48`; `course_navigation/launch/navigation.launch.py:82–146` | 20 s delayed SLAM, map projector, lifecycle manager; controller raw velocity → smoother → `/course_navigation/cmd_vel`. Historical autonomous goal works. Current trial stopped before sustained global-map readiness. |
| Final adapter baseline | `course_navigation/course_navigation/velocity_adapter.py:19–40,56–79,209–256,278–340` | 20 Hz steady timer; finite limits and .5 s command/state/clock/TF expiry, clock rollback latch. Sole final `/path_manager/path_manager_ros/nav_vel` producer. No guard manifest means baseline path: that is not an approved Stage4 bypass. |
| rl_sar → B2-W joints | `b2w_sim/b2w_controllers/src/rl_sar_controller.cpp:43–45,61–108,125–170`; `course_sim/config/rl_sar.yaml:144–145` | Raw joint/IMU feedback; 57D policy,16 actions; 50 Hz inference/200 Hz torque; feedback expiry .15 s, command expiry .5 s (ROS/simulation time), idle wheel braking, yaw gain1.6. Missing feedback outputs zero torque, not a proven physical stop. |
| Stage4 guard additions | `velocity_adapter.py:81–124,292–360`; `stage4_guard.py:41–52,113–170` | Contact stream, cached grids/cadence, coherent pose context, identity/inhibit, predictive body/stop veto, lease and callback deadlines. Added to adapter, not a replacement navigation engine. |
| Session additions | `scripts/stage4_session.py:77–135`; `scripts/stage4_route/session.py:11–45` | Owned processes and identity-pinned launch; Smac2D diagnostic profile rather than baseline NavFn; goal manager disabled; all three prepared grids/cadence required before watchdog/probe. No startup motion lease. |

Code references are at the audited HEAD; no shared topics, TF or interfaces changed.
The live LIO diagnostics overlay is another experimental delta, not evidence that
baseline LIO is fundamentally unusable. Ground Truth remains evaluation-only;
private raw IMU feedback for rl_sar is not a localization substitution.

## Architectural blocker and 50 ms interpretation

`velocity_adapter.py:335–339` permanently inhibits on callback wall elapsed≥.05 s,
even with `guard.zero_only=True` and no lease. `:356–360` does the same after
health I/O. `stage4_guard.py:49–52` retains the first failure and creates the
independent inhibit file. `stage4_session.py:102` sees that file and stops
bootstrap before watchdog launch at:121 and probe at:125.

Latest prepublish88.607 ms wall /1.111 ms thread CPU; localization mark88.549 ms /
1.055 ms. Health write .449 ms. Approximately87.5 ms was not counted as executing
this thread. This is consistent with waiting or descheduling, **not identification
of the cause**: cumulative range includes check_zero_control (filesystem/process
identity), localization_valid (clock/TF), exception handling and scheduling.
No evidence distinguishes a TF lock, I/O, GIL, host load or preemption.

50 ms is a reasonable nominal20 Hz work budget and may be a conservative
motion-mode veto. It is **not an established hard schedulability guarantee** for
this Python/ROS2/Gazebo workstation, nor justified as an unconditional lifetime
failure criterion for a mode that cannot arm. ROS2's official real-time design
requires deterministic scheduling and avoiding page faults/blocking/allocation;
normal timer periods alone do not supply that guarantee:
https://design.ros2.org/articles/realtime_background.html and
https://design.ros2.org/articles/realtime_proposal.html.

The current callback stopwatch also excludes queueing **before callback start**;
a delayed timer can pass its execution budget. Conversely, it includes health
I/O **after final Twist publication**. It is therefore neither end-to-end command
age nor physical stop latency. Replacing wall time with thread CPU would conceal
real delay during motion and is rejected. One88 ms delay below150 ms does not
prove end-to-end freshness or stop safety either.

## Boundary classification

| Check / mechanism | Necessary role; treatment |
| --- | --- |
| Sole final publisher, exclusive owner, no unexpected goals/actions | Retain to prevent competing commands. Baseline reuse must keep these controls. |
| Exact-zero immutable ZeroOnly mode, unexpected lease/nonzero-input rejection, hard qualification interlock | Retain: directly prevents unqualified commanded motion. Zero command itself does not prove physical stationarity. |
| Finite/clamped commands, advancing sensor/TF/clock freshness, rollback, sticky malformed state, contact faults | Retain: control integrity and actual stop permission. Missing localization before arming may wait at zero, as already implemented. |
| Motion lease150 ms, owner heartbeat250 ms, adapter health150 ms, independent watchdog/inhibit | Retain unchanged. Watchdog revokes leases; it cannot publish a stop if adapter is dead. Low-level timeout uses sim time; frozen clock requires separately measured abort handling. |
| Filled body, unknown/offmap blocking, tracking/stop/uncertainty reserves and sensor-supported map registration | Retain for motion. These are not prerequisites to commanding exact zero. All-body contact coverage is a real unclosed safety assurance, not disposable logging. |
| 50 ms single-callback sticky latch in immutable ZeroOnly | Startup diagnostic coupled to permanent failure; proposed narrow review below. Keep current motion-mode latch unchanged. |
| Three grids/structural cadence, recording capacity, end-watermarked reports | Readiness/evidence acceptance checks, not torque generation. Keep them for a navigation qualification pass; do not claim a probe pass without them. They need not define low-level startup health, but no restructuring is necessary for the first narrow proposal. |
| Saved registration, full corridor/portal or G5 proof before startup | Not needed for ZeroOnly; already excluded from its v2 inputs. Start→G1 needs its own sensor-supported approach/turn/stop, not later corridor visibility. Preserve strict later-segment requirements. |

## Proposed narrow patch — NOT APPLIED, independent review required

Scope only `velocity_adapter.py:335–360` and affected focused tests. For immutable
ZeroOnly, retain overrun timings and exact-zero publication, but treat a50 ms
callback overrun as a diagnostic rather than calling sticky inhibit **for that
reason alone**. Every other error retains present latching behavior. This must
never be inferred from `armed=False`: only the immutable `guard.zero_only` flag
qualifies. No runtime switch, environment override or threshold change.

Keep150/250 ms freshness and watchdog unchanged. Old health remains dated at the
callback start, not rewritten with a fresh completion time. Fresh health required
for bootstrap is unchanged. Existing watchdog still starts at prepared readiness;
no watchdog removal or claim of bootstrap watchdog qualification. Probe must
still fail any violated recorded stream/freshness/capacity/stationarity criterion.
50 ms overruns remain visible in review; a resulting zero-only pass does not
certify20 Hz motion scheduling. If diagnostics alone are unacceptable to the
independent reviewer, retain current behavior rather than inserting a larger
constant. Do not add a timer service, queue, second writer or alternate publisher.

Required focused regressions for that future patch: ZeroOnly waiting/ready with
65 ms prepublish and postpublish delay remains exact zero and unarmed; timings
retained;150 ms stale health still rejected; nonzero/lease/owner/contact/invalid
localization faults still latch; **motion-mode**50 ms prepublish/health faults
still latch; direct adapter/transport/CLI qualification remains non-bypassable.
Current tests intentionally assert today's sticky ZeroOnly overrun, and must be
changed only after that policy is independently accepted.

## Next progression and bounded local commands

**Now executable offline only**, from Bash (not sourced into zsh):

```bash
cd /home/zyd/program/b2w-embodied-navigation-course-stage4
source experiments/g1_g5/evidence/health-io-repair-20261022/candidate-env.bash
/usr/bin/python3 scripts/stage4_session.py \
  --inputs experiments/g1_g5/evidence/startup-repair-20261021/zero-inputs-candidate.json \
  --output /tmp/b2w-warehouse-review-input-check-20261022
```

This command validates identities only, creates no session and launches nothing.
If artifacts are absent/stale, stop; do not download, fallback to an old install,
or alter hashes. The currently installed candidate can validate because runtime
source is unchanged. A future patch needs a fresh regular copy build and reviewed
source/install identities before the live command below is eligible.

**A: one stable zero-start trial, AFTER independent review and explicit live
approval**. Preconditions: accepted narrow policy, matching copied install,
unchanged source world/observer/policy/controller identities, display/GPU ready,
ROS domain197 empty, no existing owner. Use the reviewed environment (current
candidate-env above only remains suitable if source has not changed). From that
Bash environment, the exact current harness interface is:

```bash
stage4_trial="$PWD/experiments/g1_g5/evidence/navigation-unblock-20261022/raw/zero-$(date -u +%Y%m%dT%H%M%SZ)"
/usr/bin/python3 scripts/stage4_session.py \
  --inputs experiments/g1_g5/evidence/startup-repair-20261021/zero-inputs-candidate.json \
  --output "$stage4_trial" --execute-zero-smoke
```

Do not run unchanged just to reproduce the known gate. No automatic retry.
Existing bounds: bootstrap90 wall s; watchdog150 wall s after it starts;
supervisor220 wall s; probe90+35; owned cleanup10+5+3 plus child reaping.
Success: exit0, `session-result.json.zero_motion_smoke_pass=true`, probe result
PASS for≥30 simulation s and≥5 wall s stationary observation, three live/local/
global grids and cadence ready, unique graph, valid localization, observed final
six-axis zeros, paired end-contact closure, owned cleanup PASS. Require actual
result files; absent watchdog/probe is FAIL. Abort first nonzero, unexpected
lease/source/goal, forbidden contact, identity/freshness/capacity fault, b0/native
crash, timeout or body instability. Ctrl+C the exact owned harness terminal;
it inhibits then cleans token/PID-start-matched children. Inspect results and
verify owned identities absent; never broad-kill processes or reset a shared world.

**B: stop/motion qualification and short controlled simulation.** There is
currently **no qualified E1 command**. `require_motion_qualification()` in
`course_navigation/course_navigation/stage4_qualification.py:9–13` unconditionally
raises; `run_candidate_route.py:64–66`, `ros_transport.py:584–585` and
`velocity_adapter.py:322` enforce it. SessionOwner returns `zero_only=True` at
`scripts/stage4_route/session.py:41`; it is not a motion bootstrap. A review of A
alone cannot make B executable. Do not advertise a command that cannot work.

The next implementation after A must use these existing boundaries to enforce
reviewed per-run local physical gates before replacing the unconditional raise:
owned action-independent stop including adapter/controller loss, conservative
initial open-pocket clearance/abort allowance, reliable physical evaluator,
sensor-only local registration uncertainty (no world truth), measured command
tracking/whole-stop excursion and same-session evidence. This is a circular
calibration dependency unless the first .05 command has a separately reviewed
conservative bootstrap envelope. Historical .0799 m stopping is a hint, not that
bound. Freeze that envelope and exact command/owner/abort invocation first.
Proposed first pulse: forward command .05 for≤2 simulation s, then zero until
≥5 source s and≥3 wall s rest;≤90 sim/180 wall s ceiling, one attempt, no corridor,
no speed escalation. Actual distance/speed must be measured, not assumed from
command. Pass only inside the predeclared physical envelope, zero forbidden
contacts, valid localization and bounded measured abort/stop on every exercised
fault. Numerical tracking/stop/uncertainty allowances must come from accepted
bounds, not invented defaults. Missing bounds/abort implementation means STOP.

`check_locomotion_physics.py:47,75–83` directly publishes the final topic and runs
.2 forward/.25 turn; it bypasses Stage4. Do not use it as the new E1 qualification
command. Likewise ordinary `run_warehouse.sh` plus RViz goals is not an authorized
shortcut. No honest executable B command exists before the reviewed implementation.

**C: Start→G1, after B and separate goal authorization.** Preserve exact selected
G1 `[-0.18571805953979492,7.189985752105713]` in registered SLAM map. Do not substitute
short historical `(1,-1)` or Gazebo world coordinates. Use existing selected
candidate, one same-session local sensor registration, exact padded swept path
and measured Start/heading; forbid G2–G5 entry. The existing executor interface
for the future qualified deployment is below, deliberately **NOT READY TO RUN**:

```bash
# Set all four absolute paths to the independently reviewed, same-session artifacts.
# No such qualified deployment/deadline artifact exists in this task.
/usr/bin/python3 scripts/run_candidate_route.py run \
  --selection "$reviewed_selection" --session "$reviewed_session" \
  --deployment "$reviewed_deployment" --deadline-evidence "$reviewed_deadline" \
  --mode segment --segment start-g1 --execute-user-run
```

Today it returns UNQUALIFIED_FOR_MOTION before ROS construction. The frozen future
command needs real artifact paths and a qualified motion owner; do not remove the
raise to make this example work. Ceiling180 sim/300 wall s, tighter duration via
existing `constraints.deadline_for` using measured achieved rates and path metrics.
Success: exact ordered G1 region .25 m for≥.1 source s, no future region, physical
stop≥5 source/3 wall s, clean contact interval through stop, localization/geometry/
recording intact, current executor qualification and independent approval. Abort
all B faults plus path/registration change, .15 m/20 s progress violation, rejected/
unresolved goal/cancel, ABORTED/CANCELED or deadline. Owned inhibit/zero precedes
cancel; process absence alone never means stopped.

At G1 obtain fresh sensor-supported corridor boundaries; then separately qualify
G1→G2 (mandatory narrow corridor), G2→G3, G3→G4 (reviewed detour), G4→G5. Preserve
original goal identities, .25 m regions and unknown blocking. Only after independent
segment passes attempt a continuous Start→G5 run; stitched segments are not full
success. Existing formal repeated-run acceptance remains separate from this first
practical navigation milestone.

## Offline verification and preservation

Fresh bounded checks (no ROS node/context or Gazebo):
`tests/test_stage4_startup_repair.py`, `test_stage4_zero_boundary.py`,
`test_unqualified_motion.py`, `test_navigation_watchdog.py`: **48 passed in0.78 s**.
They verify present behavior, including the deadline fault; they do not validate
the proposed policy or physical safety. No new regression tests needed for this
document-only change. Input identity validation and final preservation hashes are
in the compact evidence index. All inherited reports/raw evidence preserved;
original checkout and both Pipe worktrees unchanged. No owned live processes were
created; the zero-boundary unit test's own short Python child is explicitly reaped.

## Exact documentation diff

```diff
diff --git a/PLAN.md b/PLAN.md
index 56ea94b..3c091da 100644
--- a/PLAN.md
+++ b/PLAN.md
@@ -24,7 +24,35 @@
 - 旧实施计划归档至 `docs/plans/archive/2026-10-10-local-course-system-plan.md`。历史 design/validation 保留为证据，不是另一个当前 roadmap。无新增 HANDOFF。
 - 历史轮次只读与 timing 交付已结束；本轮范围以最新用户请求和下方 NOW 为准。旧实验/交付记录只作历史证据。
 
-## NOW — Adapter transient health I/O repair (October 22)
+## NOW — Unblock real Warehouse navigation: architecture review (October 22)
+
+Worktree `/home/zyd/program/b2w-embodied-navigation-course-stage4`, branch
+`experiment/unattended-g1-g5-20261019`, HEAD `4ae85f4148a89a76b1ad25c5b059c2140be41b85`.
+Latest trial: prepublish88.61 ms wall/1.11 ms CPU, health I/O0.45 ms;
+51 positive LIO steps,356 contact pairs,502/502 final commands zero.
+Complete ZeroOnly probe and live independent watchdog NOT STARTED. NONZERO NO-GO.
+
+- [x] Preserve inherited reports/evidence; compare baseline chain with Stage4.
+- [x] Confirm historical rl_sar Warehouse motion/navigation and original G1 result;
+  separate these from current readiness and physical safety qualification.
+- [x] Identify permanent50 ms ZeroOnly callback latch and unconditional motion
+  qualification as distinct blockers; prepare narrow boundary proposal, no runtime edit.
+- [x] Prepare exact existing zero-start interface and conditional segment interface;
+  explicitly identify missing qualified E1/motion bootstrap rather than invent a command.
+- [x] Small relevant offline regressions48 PASS; validate copy-input identities,
+  preserve original/Pipe worktrees and inherited evidence; deliver ONE APRIL bundle.
+- [ ] Independent review of the ZeroOnly-only deadline policy; if accepted, implement
+  focused regressions and matching copy build, then separately authorize ONE bounded
+  zero trial. No motion, commit/push or live retry under this task.
+
+Decision and bounded progression:
+[Warehouse architecture review](docs/reviews/B2W_Warehouse_Navigation_Architecture_20261022.md).
+Compact evidence: `experiments/g1_g5/evidence/navigation-unblock-20261022/`.
+First practical target remains stable zero startup → local stop/motion qualification
+→ short controlled motion → Start→G1. Later segments do not gate initial zero startup.
+Runtime thresholds, watchdog, sticky failures and hard motion interlocks unchanged.
+
+## Historical — Adapter transient health I/O repair (October 22)
 
 Experimental worktree `/home/zyd/program/b2w-embodied-navigation-course-stage4`;
 branch `experiment/unattended-g1-g5-20261019`, checkpoint `5178993636287a22bf7cd847038d4904828fd898`.
diff --git a/docs/STAGE4_RUNTIME_READINESS.md b/docs/STAGE4_RUNTIME_READINESS.md
index 0b60437..f30c348 100644
--- a/docs/STAGE4_RUNTIME_READINESS.md
+++ b/docs/STAGE4_RUNTIME_READINESS.md
@@ -1,3 +1,11 @@
+# Current entry — Warehouse navigation architecture review (2026-10-22)
+
+Audited HEAD `4ae85f4148a89a76b1ad25c5b059c2140be41b85`; runtime unchanged.
+Latest ZeroOnly attempt failed before watchdog/probe. No live/motion authorization.
+Use PLAN.md NOW and [the current architecture decision and bounded commands](reviews/B2W_Warehouse_Navigation_Architecture_20261022.md).
+Earlier commands below are historical. A ZeroOnly deadline-policy proposal awaits
+independent review; no qualified E1/motion bootstrap exists yet.
+
 # Current status: ZeroOnly runtime FAILED; offline startup repair only
 
 Persistent worktree: `/home/zyd/program/b2w-embodied-navigation-course-stage4`.
```

## experiments/g1_g5/evidence/navigation-unblock-20261022/offline-tests.txt

```text
Command: bash -c source candidate-env.bash && /usr/bin/python3 -m pytest -q -p no:cacheprovider --basetemp=/tmp/b2w-architecture-tests-20261022 tests/test_stage4_startup_repair.py tests/test_stage4_zero_boundary.py tests/test_unqualified_motion.py tests/test_navigation_watchdog.py
Tool-observed result: 48 passed in 0.78s; exit 0.
No Gazebo or ROS node/context. Existing tests, runtime source unchanged.

```

## experiments/g1_g5/evidence/navigation-unblock-20261022/input-validation.txt

```text
Exact command is in current architecture report, offline command block.
Fresh tool result: Inputs verified; no session launched
Exit code: 0
No output session directory created.

```

## experiments/g1_g5/evidence/navigation-unblock-20261022/preservation.json

```text
{
  "/home/zyd/program/b2w-embodied-navigation-course-stage4": {
    "preexisting_files_checked": 514,
    "changed": [
      "PLAN.md",
      "docs/STAGE4_RUNTIME_READINESS.md"
    ],
    "expected_changes": [
      "PLAN.md",
      "docs/STAGE4_RUNTIME_READINESS.md"
    ]
  },
  "/home/zyd/program/b2w-embodied-navigation-course": {
    "preexisting_files_checked": 397,
    "changed": [],
    "expected_changes": []
  },
  "/tmp/b2w-pipe-v0": {
    "preexisting_files_checked": 342,
    "changed": [],
    "expected_changes": []
  },
  "/tmp/b2w-pipe-integration-20261021": {
    "preexisting_files_checked": 342,
    "changed": [],
    "expected_changes": []
  },
  "inherited_trial_manifest_entries_verified": 58,
  "inherited_trial_owned_processes_remaining": [],
  "runtime_source_changes": [],
  "large_video_files": "Excluded from hash audit; never opened or modified by this task"
}

```

## experiments/g1_g5/evidence/navigation-unblock-20261022/SHA256SUMS

```text
88d27714ff4d068cc79ca1d3a443b384b9adae9af1ec96240300a8b61a65d741  PLAN.md
733db7968d54f287b25e4f3040717239ef8d60a2dd392c166be3c7365f189cb1  docs/STAGE4_RUNTIME_READINESS.md
ed1565938afbbdccaa78209f8f2300208034a4f58f94983f2fb5bb617496f868  docs/reviews/B2W_Warehouse_Navigation_Architecture_20261022.md
6739fce366508e5977a1509cbe7160c2cbc8faa490746eebc0434d7c564649e0  experiments/g1_g5/evidence/navigation-unblock-20261022/input-validation.txt
ff0ddafd9b9dd3000183d7ed7df3f46f0e17c2ed2ef114e9bb4c1ccad1034055  experiments/g1_g5/evidence/navigation-unblock-20261022/offline-tests.txt
4b3b3fb95a69cb8cd7ddae3216615ce90771ec3bdeaddcde092d15d65a0992f4  experiments/g1_g5/evidence/navigation-unblock-20261022/index.json
049235209eb7008ad150d14e193244d5aa01ea249921e329643566c67ce3aa8a  experiments/g1_g5/evidence/navigation-unblock-20261022/preservation.json

```

## course_navigation/course_navigation/stage4_qualification.py

```text
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

```

## Independent review decisions requested

1. Accept or reject the immutable-ZeroOnly-only deadline diagnostic policy; motion-mode50ms behavior and all150/250ms checks stay unchanged.
2. Confirm local E1/Start→G1 qualification scope and which physical gates must precede the first bounded pulse. No substitution of a ready boolean for enforcement.
3. Reject live repetition of unchanged candidate and direct baseline/physics command bypasses. No commit/push under this handoff.
