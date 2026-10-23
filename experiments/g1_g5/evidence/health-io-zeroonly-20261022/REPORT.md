# Stage4 Health I/O checkpoint + ONE ZeroOnly trial — 2026-10-22

**Live result: FAIL. Complete ZeroOnly probe: NOT EXECUTED. NONZERO NO-GO.**
One authorized trial only; no repair or retry. Stop for independent review.

## Reviewed checkpoint

Repository `/home/zyd/program/b2w-embodied-navigation-course-stage4`, branch
`experiment/unattended-g1-g5-20261019`. Committed and normally pushed ONLY this branch:
`4ae85f4148a89a76b1ad25c5b059c2140be41b85`, parent
`5178993636287a22bf7cd847038d4904828fd898`. Origin server, tracking and local HEAD
matched; ahead/behind0/0, no pre-existing unpushed commits, tracked source clean.

Verified reviewed bundle SHA256
`e6996e405fc58fabb969de7bbfcc94c3c461817ce5a215a3d07d65250331fe8a` from its preserved
Trash copy. Exact repair patch embedded; all22 pending source/test/compact-record
files matched reviewed hashes. Both prior evidence manifests passed (45 and55
checksum entries). No raw/build data, binary, credential-pattern or Pipe payload
staged. Added the explicitly requested external-decision transcription under
`docs/reviews/B2W_Stage4_Health_IO_External_Decision_20261022.md`, for23 committed files.
It records the user's communicated external CONDITIONAL PASS, not agent self-review
or a newly invented reviewer approval. Full committed list and audit hashes retained.
The reviewed66-test evidence was hash-verified; no redundant tests/benchmarks rerun.

## Preflight and exact single execution

Used `experiments/g1_g5/evidence/health-io-repair-20261022/candidate-env.bash`.
Fresh-process imports resolved velocity_adapter and stage4_control to its isolated
regular install, not an older underlay. Installed bytes match current source; the
adapter calls write_adapter_health and that function has no fsync.61 current
module/config/launch copy identities match. v2 validator pins world/inventory,
source SDF/policy/controller identities; no fields, limits or thresholds changed.

Backend checked separately from v2 and the new overlay: actual diagnostic LIO PID
3564542 /proc/exe SHA256
`3bd8109f580ceac90670f5d12e57267192731e2bfd2af46eb1740d08672e50f5`.
B2W_STAGE4_LIO_DIAGNOSTICS=1 present. Actual /proc maps confirm
`/home/zyd/program/b2w-embodied-navigation-course/.deps/lib/libgtsam.so.4.1.1`,
whose preflight SHA256 is
`e04567b0a6346ed8c4a05a21b42b337ee2ce429030f8df87e3149b76a6bfb379`.
The running adapter's command/traceback also names the repaired install prefix.
Zero-only guard true, hard production motion interlock rejects, no lease, idle
ROS domain197, localhost-only communication, no previous owner token/runtime.
Host/display/GPU preflight PASS. No Ground Truth entered navigation; evaluation
contact data and unchanged low-level body-state feedback retain their existing scope.

Executed existing `scripts/stage4_session.py --execute-zero-smoke` exactly once,
with the existing startup-repair zero-inputs-candidate.json. The exclusive
raw/attempt.json identifies the new checkpoint; raw/session/invocation.json retains
exact argv/inputs/owner and unique token815d8266b49f46c3a8d9deab684dc302.
Passive observer started first, no command/goal/action/service publisher. Wrapper
also watches explicit backend fatal markers to interrupt only its exact harness
PID if needed; no such backend marker occurred. Existing limits unchanged:
90s bootstrap,220s supervisor,150s watchdog,90s probe plus35s transport,10+5+3s
owned cleanup. Live-stack wall36.671s including cleanup; wrapper37.350s.
Harness exit1, passive recorder exit0. No goals, leases, nonzero output, motion
authorization, interlock bypass, threshold changes, new source changes or retry.

## First trustworthy blocker: prepublish wall-time stall

First inhibit `adapter_publication_deadline`, observed at monotonic338074.325527161,
latest received simulation clock18.20s. First saved sticky fault is health sequence499,
callback start338074.230364054; sequence501 retains the same sections and includes
the completed health-I/O timing. Immediately preceding recorded health498 was ready
and fault-free; first fault prepared grids are empty and cadence budgets unset.

| Fault callback section (cumulative except I/O) | wall ms | thread CPU ms |
| --- | ---: | ---: |
| localization mark |88.549|1.055|
| prepublish |88.607|1.111|
| published |88.858|1.362|
| current health write only |0.449|0.449|
| previous health write |0.847|0.848|

The unchanged50ms PREPUBLISH check fired, unlike the previous health-stage deadline.
Almost all measured wall time accrued before the localization mark, whose range
includes guard.check_zero_control(), localization_valid(), and the armed-state
check. These include filesystem/process identity checks, clock access and direct
TF lookups. This cumulative mark does not isolate a particular function, lock,
syscall or scheduler event. No context/guard-completion mark exists for that
callback. **Exact wait versus descheduling cause: NOT_OBSERVED.** Do not label TF,
GIL, /proc, disk or the recorder as proven root cause, or raise the50ms threshold.

Transient-health repair actual runtime evidence:243 sampled previous-I/O durations
have median0.528ms, max2.773ms wall (median0.517/max2.343ms CPU). Fault-callback health
write0.449ms. No health-stage first fault was observed. This demonstrates the repaired
installed heartbeat path writing promptly in these samples, not a hard latency
bound, lossless sampling guarantee or complete ZeroOnly qualification.

## Zero-only probe/watchdog outcome

Bootstrap observed inhibit and stopped. No watchdog manifest/log, deployment,
probe directory or lease exists. **The complete ZeroOnly probe never executed**;
its30-simulation-second accepted stationary interval, endpoint contact closure,
graph/runtime checks and independent live watchdog behavior are NOT_OBSERVED.
The demonstrated stop path is adapter sticky inhibit, continued observed zeros,
owner/supervisor detection and bounded owned-process cleanup, not a full physical
watchdog/abort qualification. Guard thresholds and hard motion interlocks remain.

## Actual ROS, command and stationary evidence

502/502 observed final `/path_manager/path_manager_ros/nav_vel` Twists have ALL SIX
axes exactly zero, including2 received after first observed inhibit. Both raw
`/course_navigation/cmd_vel_raw` and `/course_navigation/cmd_vel` NOT_OBSERVED.
Passive subscriptions are BEST_EFFORT; counts are received samples, not a proof
of lossless command recording. No navigation action or motion lease was issued.

| Stream | received | received-clock minus header seconds |
| --- | ---: | --- |
| clock |7307|n/a|
| IMU |3617|−0.0250 to0.0125|
| pointcloud |183|0.0075 to0.0575|
| SLAM odometry |7|0.2825 to0.3025|
| SLAM map points |1|0.3350|
| occupancy map |1|0.0525|
| local raw costmap |1|0.0050|
| global raw costmap |0|NOT_OBSERVED|
| map→odom TF |7|0.2825 to0.3050|
| odom→base_link TF |7|0.2825 to0.3025|

These are receiver-side estimates against the latest clock callback, not source
clock ordering qualification. Small negative sensor ages reflect inter-topic
receipt order. No sustained readiness/freshness PASS. Seven SLAM poses span only
0.6s, max distance from first0.000222m. **Whole-trial physical stationarity remains
NOT_OBSERVED**; zero commands and a short stable SLAM segment do not establish it.

## LIO and evaluation delivery

LIO exception/b0: NOT_OBSERVED.52 graph records: one priors-only initial update
correction12.8, queue53 spanning12.8–13.06, last IMU12.795, integrated_dt0 (normal
for initial priors, not an IMU integration factor). Then51 imu_step updates,
key1–51, corrections12.9–17.9, queue54–62, deltaTij
0.09999999999999787–0.10000000000000142s. First step queue12.9–13.185,
last IMU12.895; last step queue17.9–18.185,last IMU17.895;3 factors/3 values.
No diagnostic update_failed or GTSAM abort. Historical b0 cause remains unresolved;
second no-argument optimizer.update() remains explicitly uninstrumented.

Owned one-way evaluation bridge actually ran and logged GZ→ROS Contacts/String
mappings.356 Contacts and356 inventory messages, contiguous sequences1–356,
stamps0.0025–18.2725s;352 nonempty. Four unique foot/warehouse-ground pairs only;
reported ever_forbidden_contact=false. Last inventory sampled20 steps, last ground
support18.2725. **ROS contact/inventory delivery PASS** in this bounded observation.
Full-body physical collision coverage and a probe end-watermarked interval remain
NOT_QUALIFIED/NOT_OBSERVED. No deliberate collision or physical bound test performed.

## Process lifecycle, preservation and remaining blockers

Harness cleanup PASS. Independent post-check finds all25 PID/start identities,
owner token and passive recorder absent; ROS domain197 idle. nvidia-smi shows only
unrelated desktop GPU processes, none killed. No broad process cleanup. The512
checkpoint file hashes and407 shared-checkout source hashes are unchanged after
trial. No implementation was patched, staged or committed after the failure.
Only the new result/review records remain uncommitted. Raw evidence is ignored,
LOCAL ONLY, not remotely backed up. Complete SHA256 indexes retained here.

Secondary shutdown failures occurred AFTER launch SIGINT: controller_server warns
about unloading a class-loader library with live objects, then terminates without
an active exception (exit−6). Adapter rclpy take_message RuntimeError and invalid
publisher-context RCLError recur (exit1). These do not replace the earlier sticky
prepublish deadline as first blocker. Cleanup still removed all owned processes;
clean process absence does not imply clean shutdown of each application.

Remaining: wall-dominant pre-localization-mark delay unlocalized; complete startup/
global-grid/cadence readiness and ZeroOnly probe absent; historical b0 unresolved;
physical stationary/coverage/abort qualification incomplete. **NONZERO NO-GO**,
including Start→G1 and sequential G1–G5. PLAN.md retained at the reviewed checkpoint;
this durable execution record records the authorized continuation and failed gate.

**Exactly one next action: independent review of this result packet**, focusing
on the smallest measurement to distinguish early callback identity/clock/TF work
from scheduler/off-CPU delay before any implementation change or another trial.
