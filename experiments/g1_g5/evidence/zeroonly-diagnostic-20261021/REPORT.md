# Stage4 reviewed checkpoint and ONE ZeroOnly diagnostic trial — 2026-10-21

**Live result: FAIL. NONZERO NO-GO. Stop for external review; no repair or retry.**

## Checkpoint and review boundary

Committed and normally pushed ONLY `experiment/unattended-g1-g5-20261019`:
`5178993636287a22bf7cd847038d4904828fd898`, parent
`1f33365757de7fd3d372ccf91b203f1461cd862f`. Origin's server SHA, local HEAD and tracking
SHA agreed; ahead/behind0/0. No pre-existing unpushed commits. Exact38-file list is
`committed-files.txt`. No shared branch, original source checkout or Pipe changes.

Verified review bundle SHA256
`eb3e81c362a6fce17a872991d253d91529e5e59e303b40b058e8b73a9eff5b11` from the preserved
Trash copy and exact embedded source patch. All492 frozen source/record hashes matched,
including every staged byte. Audited text sizes, file scope and credential patterns;
no raw/build/binary payload staged. Default staged `git diff --check` reported five
upstream context whitespace lines inside the new diagnostic patch. These exact reviewed
bytes were retained: all other staged files and every actual added C++ line passed.
Initial commit attempt stopped on that check; initial push was a no-op. The subsequent
single commit and normal fast-forward push succeeded. No implementation changed here.
Previously reviewed83 Python and4 native tests were verified against their indexed
logs, not rerun or presented as fresh runtime qualification.

## Preflight and one invocation

`raw/preflight-host.json`: source clean;61 regular copy-installed module/config/launch
identities match; v2 world/inventory/policy/controller checks pass. Backend selection
was checked separately from v2: diagnostic IMU SHA256
`3bd8109f580ceac90670f5d12e57267192731e2bfd2af46eb1740d08672e50f5`.
Runtime PID3184438 `/proc/exe` SHA and GTSAM maps confirm this executable and shared
`.deps/lib/libgtsam.so.4.1.1`, preflight SHA
`e04567b0a6346ed8c4a05a21b42b337ee2ce429030f8df87e3149b76a6bfb379`.
Diagnostic flag1 present. Independent production motion interlock rejected motion;
zero-only guard true, no lease, no goal/action producer. Domain197 initially idle.

Executed exactly once via existing `scripts/stage4_session.py --execute-zero-smoke`,
candidate-env.bash and zero-inputs-candidate.json under startup-repair-20261021.
Exact argv/inputs/owner token in raw/session/invocation.json. Exclusive raw/attempt.json
and raw/run_once.py prevent reuse. Existing limits preserved:90s bootstrap,220s
supervisor bound,150s watchdog,90s probe plus35s transport,10+5+3s owned cleanup.
Live-stack wall36.342s including cleanup; wrapper37.184s. Harness1, recorder0.
No goals, motion lease, nonzero command, safety bypass, threshold change or live retry.
Passive recorder subscribes only; records all received IMU vectors, timing controls
and token-scoped process identities/maps. Recorder32MiB/260s caps were not reached.
Its /proc scan at0.1s and first-seen executable hashing add uncalibrated host load;
this evidence does not exclude recorder contribution to scheduling/I/O contention.

## First trustworthy failure

First inhibit: `adapter_health_publication_deadline`, observed monotonic292697.978907569,
received clock17.18s. Raw adapter snapshot sequence493 contains the corresponding
pre-health timing; first saved sticky timing is sequence495. Context was `ready`
before the inhibit; first sticky snapshot context `fault`, prepared grids empty.

| Cumulative section / isolated I/O | wall ms | thread CPU ms |
| --- | ---: | ---: |
| localization |27.183|1.529|
| context |27.526|1.867|
| guard |27.976|2.317|
| prepublish |28.004|2.345|
| published |28.216|2.556|
| current health write only |41.062|0.974|
| published + health write |69.278|3.531|
| previous health write |7.102|0.757|

The unchanged50ms total-health check fired after synchronous write_control. Its
implementation serializes JSON, creates a temporary file, writes/flushes/fsyncs and
replaces the health path. A wall-dominant overrun is demonstrated; exact syscall
latency versus runnable/descheduled time is NOT_OBSERVED. This is not evidence of
CPU-heavy guard math, nor proof that fsync alone caused it. Do not raise deadlines.

## Backend and contact results

LIO exception: **NOT_OBSERVED**, not repaired/qualified.51 diagnostic graph records:
one initial priors-only update at correction11.9, queue56 covering11.9–12.175,
last IMU11.895, integrated_dt0 (expected for the priors-only graph, not an IMU factor).
Then50 imu_step updates, key1–50, correction12.0–16.9; queue52–62, integrated_dt
0.09999999999999787–0.10000000000000142s. First step queue12–12.285, last IMU11.995;
last step queue16.9–17.165, last IMU16.895. All3 factors/3 values. No update_failed
or b0 abort in this observation. Original b0 cause remains unresolved; the second
no-argument optimizer.update() remains explicitly uninstrumented.

Owned one-way evaluation bridge actually ran and logged both GZ→ROS mappings.
335 Contacts and335 inventory messages received, sequences1–335 contiguous,
stamps0.0025–17.2075s;331 Contacts messages nonempty. Four unique foot/warehouse-ground
pairs observed. Inventory last sampled21 steps, last ground support17.2075,
ever_forbidden_contact=false. **ROS delivery PASS** for this bounded observation;
full-body physical collision detection coverage and end-watermarked interval
qualification remain NOT_QUALIFIED. No diagnostic body contact was induced.

## ROS freshness, zeros and limited pose evidence

| Stream | received | received-clock minus header seconds |
| --- | ---: | --- |
| IMU |3410|−0.0225 to0.0150|
| pointcloud |172|0.0075 to0.0800|
| SLAM odometry |4|0.2700 to0.2850|
| SLAM map points |1|0.3725|
| occupancy map800×800 |1|0.0625|
| local raw costmap100×100 |1|0.0025|
| global raw costmap |0|NOT_OBSERVED|
| map→odom TF |4|0.2725 to0.2875|
| odom→base_link TF |4|0.2700 to0.2850|

6891 clock callbacks received. These are receiver-side age estimates using the most
recent clock callback; slight negative values reflect inter-topic reception order,
not source timestamp verification. No sustained readiness/freshness PASS is claimed.
The four odometry poses span only0.3s; max distance from first0.000338m. Whole-trial
physical stationarity is **NOT_OBSERVED**; zero commands alone do not prove it.

496/496 observed final `/path_manager/path_manager_ros/nav_vel` Twists were exactly
zero on all six axes, including one received after first observed inhibit. One raw
Nav2 Twist during shutdown was also zero. `/course_navigation/cmd_vel` NOT_OBSERVED.
BEST_EFFORT recorder counts are observations, not a proof of lossless delivery.
No independent watchdog/probe/deployment was created before bootstrap inhibition:
watchdog/stop physical response **NOT_OBSERVED**. Owner heartbeat, adapter inhibit,
all observed final zeros and supervisor termination are the demonstrated stop path.

## Secondary shutdown evidence and cleanup

After launch sent SIGINT, adapter rclpy subscription take_message raised RuntimeError;
stop_stage4's final Twist publish raised RCLError because publisher context was invalid;
adapter exited1. Lifecycle manager required launch SIGTERM after5s. These are retained
secondary shutdown failures, not substituted for the earlier health deadline.
No repair was made. Existing ownership cleanup completed despite these exceptions.

Harness cleanup PASS; independent post-check verifies all25 PID/start identities
absent, no owner token or passive recorder survivor, domain197 idle. nvidia-smi lists
only unrelated desktop GPU processes. No broad process kill or unrelated cleanup.
All492 reviewed files and407 original shared source files still match preserved
hashes. Shared source unchanged. Raw evidence is ignored LOCAL ONLY, not remotely
backed up. Compact new diagnostic/review records remain uncommitted for review.
PLAN.md retained exactly as committed; this execution record supplies the reviewed
continuation without rewriting its historical offline-only milestone text.

## Remaining blockers and exactly one next action

ZeroOnly qualification failed on wall-dominant synchronous adapter health publication;
first backend b0 mechanism remains unproved, startup/global readiness and watchdog
qualification incomplete, whole-trial stationarity/full-body physical coverage absent.
NONZERO remains NO-GO, including Start→G1 and sequential G1–G5.

**Next action: external review of this packet**, focusing on a minimal follow-up
measurement that separates write_control syscall durations from scheduler/off-CPU
time under the same strict deadlines. No further code change, commit/push, Gazebo
repeat or navigation motion before that review.
