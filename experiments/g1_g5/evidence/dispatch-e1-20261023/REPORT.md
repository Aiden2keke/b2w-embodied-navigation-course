# Warehouse dispatch attribution and offline E1 — 2026-10-23

Historical181.573ms gap **UNRESOLVED**; no production dispatch patch and **0 new Gazebo trials**. Latest full ZeroOnly result remains **FAIL**; E1/NONZERO remains **NO-GO**. Completed non-authorizing E1 request accounting,237 focused regressions, fresh isolated copy build and actual installed zero-only adapter smoke. This is concrete offline progress, not motion qualification.

## A. Exact historical cause and scheduling path

Sequence696 starts at88017.609832271;697 starts88017.791405108:181.572837ms. Sequence696 published after27.314479ms;697's previous_health_io records29.911981ms. Thus approximately**124.346377ms after health I/O** remains until the next callback starts. This decomposition uses adjacent original timing records, not observer receipt times. The rejected record was191.620133ms old when Watchdog.step checked it. The150ms limit remains correct and unchanged.

Original evidence lacks callback entry/exit events, timer-ready times, worker stacks and native-thread scheduling samples within that124ms interval. A process-level S state does not distinguish GIL, executor waiting, filesystem wait or another thread. Map stamp27 occurred nearby; no causal attribution follows. No retrospective trace can honestly recover the missing stack.

Actual path, verified against installed Humble code and current source:

1. `rclpy.spin(node)` uses SingleThreadedExecutor. Command, odom, contacts, structure, all three grid subscriptions and the steady50ms timer use the default MutuallyExclusiveCallbackGroup. TF listener subscriptions have a ReentrantCallbackGroup but the same single executor thread. Callback-group changes alone do not create CPU independence.
2. The executor's ready snapshot yields timers before subscriptions, but does not preempt an active handler or reconsider a newly ready timer between every subscription in the existing snapshot. Taking/deserializing a message occurs before user callback entry.
3. Grid callback validates dimensions/quaternion, copies `msg.data` into a tuple and calls GridPreparation.submit. Its condition-protected exchange preserves at most four immutable pending generations, immediate invalidation of changed geometry, accepted-payload refresh and stale-generation exclusion. It is a Python thread in the same process, not GIL isolation.
4. Worker prepare validates every cell, converts occupancy, canonical-JSON hashes, converts tuple to NumPy and builds ClearanceGrid/cKDTree. These stages share the interpreter GIL. Locks protect short metadata/install sections; CPU work is outside the guard lock. No long held guard lock was established by these probes.
5. Publish checks ownership/inhibit/localization/TF/guard, publishes through the sole final publisher, then synchronously writes original callback-wall health using transient atomic write_adapter_health (no fsync). Releasing/reacquiring the GIL around filesystem operations can extend several short I/O operations while worker CPU runs. This differs from proving storage hardware was blocked.

## B. Offline reproduction and correction decision

Five bounded probes used the real VelocityAdapter/GridPreparation and real ROS serialized800x800 OccupancyGrid plus800x800 Costmap ingress; local grids120x120, or actual100x100 in the dynamic run. Separate producer process avoided injecting producer GIL cost into the adapter. Sparse and unknown-filled Warehouse-shaped synthetic cells were used because the original cell payload was not recorded. Ready variants supplied synthetic coherent /clock, SLAM odom and TF; no ground truth, simulator, controller, goal or motion input. They do not represent real localization qualification.

Observations are bounded in-memory enter/leave wall/thread-CPU events, timer lateness, worker validation/hash/array/tree stages, Python frame snapshots and a separate process sampling native-thread schedstat/wchan. Data flushes after shutdown; no new per-cycle disk tracing or production instrumentation. The12000-event ring may retain only a tail in the32s dynamic run; counts below describe retained observations.

| Probe | Retained publish starts | Max start gap ms | p95 gap ms | Max publish wall ms |
|---|---:|---:|---:|---:|
| ingress-baseline-1 | 311 | 143.195 | 83.827 | 142.663 |
| ingress-dynamic-baseline-1 | 554 | 131.854 | 82.931 | 114.844 |
| ingress-parked-control-1 | 320 | 64.906 | 50.728 | 12.328 |
| ingress-ready-baseline-1 | 287 | 143.076 | 104.881 | 121.927 |
| ingress-warehouse-1 | 314 | 95.671 | 77.204 | 91.222 |

In the ready probe's maximum gap, publication took121.927ms wall but3.017ms thread CPU. Worker validation,33.163ms CPU JSON encoding and51.436ms wall/30.316ms CPU NumPy conversion overlap.47/59 external samples show the main thread in futex wait while the worker runs in50/59; main runnable scheduling delay was7.498ms. In the simpler unknown-grid maximum, main runnable delay was0.007197ms and41/43 samples were futex waits. Together with frame locations, this supports **same-process worker/GIL interference in these probes**, rather than long map subscription CPU work or host runnable starvation alone.

The paired ready parked-worker attribution control kept subscriptions, data copies and invalidation, but parked prepare and accepted no grid. It reduced max publish121.927→12.328ms, p95 start gap104.881→50.728ms and max start gap143.076→64.906ms. This control is deliberately unable to bootstrap; it is not a production candidate or a disabled safety check in a live session. Single paired trials establish a contribution, not a latency guarantee.

None reproduced181.573ms or the original124.346ms post-health interval. The matching ready symptom occurs mostly *inside* publication. JSON chunking, separate callback groups, a process worker or threshold changes therefore cannot be selected as the actual historical correction from current evidence. **No dispatch correction was invented.** Adapter/guard/geometry/control/watchdog bytes remain unchanged. Independent read-only reviewer agrees. Live work stopped under mission step4.

## C. Full ZeroOnly result and cleanup

No new Gazebo trial:0/1 conditional allowance used. Latest full result remainsFAIL: original699/699 exact-zero six-axis Twists,534 paired contact candidates/inventories, bootstrap/all grids/watchdog/probe started; watchdog correctly inhibited sequence696. Stationary acceptance, full runtime graph and contact endpoint closure were not accepted; probe and probe cleanup FAIL. Prior owned OS cleanup PASS. No new full-probe PASS is claimed.

Fresh **offline installed adapter smoke** observed20/20 exact-zero six-axis Twists, one final publisher,50ms steady timer and expected subscription/group interfaces. Worker joined; fresh post-destruction domain209 graph contained only the cleanup checker. All five producer children reaped0, workers joined, recorded probe-owner PID/start identities absent, no probe survivors and no lease files. No broad process signals, Pipe/original-checkout edits or simulator containment claims.

## D. E1 offline implementation completed

Existing qualification module now checks request shape/accounting: exact six-axis .05-forward command, pulse<=2 source seconds, rest>=5source/3wall seconds, attempt<=90source/180wall, finite local region/source scope, new session/attempt identifiers, and SHA-bound references for ZeroOnly result, bootstrap envelope, independent abort and local clearance. A referenced summary must claim full ZeroOnly and owned cleanup PASS. File hashes and summary claims **do not authenticate evidence or accept its semantics**; tests deliberately use synthetic fixtures.

Existing owner offers read-only `--check-e1-request`, which creates no owner/output/lease. The exact future spelling `--qualification ... --output ... --execute-e1` is reserved and always invokes unchanged unconditional require_motion_qualification before input reads or session creation. An approval boolean, well-formed request or forged PASS cannot execute. No SessionOwner/Watchdog/IndependentStop behavior or physical qualification boundary changed.

TDD records:26 initial failures for missing feature, then a focused overflow rejection failure, then60 focused E1/boundary tests PASS; independent reviewer reproduced60PASS. Final root regression suite: **237 passed in22.21s**. Scope includes Stage4 grid/health/session/bounds/transport/final-guard and unconditional motion tests. `git diff --check` PASS; original veto AST identical.

## E. Remaining blockers to controlled motion

1. Attribute historical residual dispatch interval and establish/test/review a supported correction; then fresh full ZeroOnly PASS including stationary, grids, graph, contacts, watchdog/probe cleanup and owned cleanup.
2. Independently accepted conservative local bootstrap/stopping envelope. Proposed future pulse measurements cannot qualify their own prior safety bounds.
3. Identity-pinned independent wall-clock abort/owned simulator containment with expressly authorized zero-only fault demonstrations (dead adapter/controller, frozen clock, failed pause acknowledgement, ownership sentinel). Current inhibit/zero publication alone is insufficient on adapter death or frozen simulation. Emergency freeze/termination is not measured normal rest.
4. Sensor-only local registration and saved/live/local/global unknown/offmap-vetoed filled-body clearance, uncertainty and stopping/abort reachable sets; complete body contact/stability and advancing-source rest qualification.
5. Reviewed existing-owner extension enforcing per-run deployment/process/producer/scope/evidence identities, single attempt, current scope, live lease/abort and measured pulse/stop acceptance, with explicit E1 authority. E1 forward calibration does not authorize Start→G1 route/turn execution.

**F. Shortest next action:** extend the bounded offline ingress probe to the remaining original contact/structure/clock traffic and observed scheduling cadence, capturing the post-health→timer-dispatch interval. Change only a mechanism that reproduces that interval, then reconsider the one conditional ZeroOnly trial. No identical Gazebo retry. After qualified E1, use the existing Start→G1 segment execution and local path/turn qualification from the preserved E1 plan; no new framework.

## Verification, identities and Git

Fresh regular-copy navigation+bringup build:2 packages PASS in1.43s. All35 source/config/launch copies match actual installed bytes under `artifacts/install`; fresh imports come from that prefix. Installed graph/smoke and independently pinned LIO/GTSAM/world/contact artifacts checked. Direct read-only policy/model/ONNX/nav2-controller/actual RL-SAR/watchdog hashes PASS; see runtime-identities.json and additional-pins.json. Build emitted inherited incomplete-LIO-prefix and intentionally disabled-bytecompile warnings; neither is counted as physical validation.

Automatic approval review rejected an owner CLI validation invocation as potentially launching Gazebo; it did not execute. Completed the needed pins with direct read-only hashes, without owner invocation. No permission-blocked required verification remains, and no workaround launched a session.

HEAD remains`cff87a8fef98d5d58b1b7bf2ed05a6e6c2b94263` on experiment branch. Source/test diff is`source.diff` SHA256`4a4571b658d4d0a2ba8b21c7fad6586f171dbc25ef20a6534da6583235999365`. Changes are audited and verified but **uncommitted/unpushed**: applicable AGENTS.md says “No commit, push, full Gazebo repeat, motion goal or nonzero authorization before external review.” User authorizes commits only when normal approvals permit; local code review does not replace that external gate. The sole PLAN.md/NOW is updated and predecessor/E1 design preserved. All56 predecessor raw hashes reverified; prior bundle/checksum remain untouched.

Raw evidence is local-only/ignored, indexed by RAW_SHA256SUMS. Compact report, summaries, reviews, source diff and identities are repository records. The one final APRIL bundle/checksum is copied to the exchange directory with Python-only adaptation of the existing delivery tool and verified by SHA256/readback; historical files are not overwritten.
