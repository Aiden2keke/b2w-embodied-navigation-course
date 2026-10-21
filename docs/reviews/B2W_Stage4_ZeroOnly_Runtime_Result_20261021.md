# B2-W Stage4 — single real ZeroOnly trial: FAIL (2026-10-21)

Checkpoint `1f33365757de7fd3d372ccf91b203f1461cd862f`; experimental branch
`experiment/unattended-g1-g5-20261019`. Started 2026-10-21T09:30:20.360530+00:00.
**Exactly one Gazebo start; no retry. NONZERO NO-GO remains.**

The stack ran for **28.190766 wall seconds including its cleanup**.
It failed during bootstrap; the qualification probe and independent watchdog never
started. The first fatal subsystem error in the launch log was LIO-SAM IMU
preintegration aborting on `gtsam::IndeterminantLinearSystemException`, variable
`b0`. Later the adapter latched `adapter_publication_deadline`; that inhibit
caused the session owner to stop the stack. These are distinct observations:
there is no evidence that the GTSAM exception caused the adapter scheduling miss.

## Review and prerequisites

The [external CONDITIONAL PASS](B2W_Stage4_ZeroOnly_Independent_Review_20261021.md)
and its checksum were read and verified at SHA256 `a256e457006a8ad0bc03df23a8e86e083b3e390308172d0454618d7316ca7549`. It explicitly
supersedes the missing-review blocker for this one trial only. The same bytes and
checksum are now copied here; the original checkout was read only.
The referenced APRIL snapshot SHA256
`ede14f4eb77f3aaa38aee011e890cb8b46ec34ae614bd8d04d99ef1c433c78f5`
was read from its preserved Trash copy and verified. No historical bundle moved.

Before launch: correct HEAD/branch and clean source; all35 regular copied module,
launch/config identities matched current source and sealed hashes. The existing
v2 validator passed for pinned observer-world, collision SDF/inventory, RL-SAR
policy bytes and controller executable/library. Existing hard motion interlock
was called and rejected with `UNQUALIFIED_FOR_MOTION`. A two-second host discovery
found only the preflight node in local-only domain197; no existing B2W owner or
runtime processes. NVIDIA RTX A4000 and DISPLAY=:1 were accessible.
No offline regression matrix was rerun.

Executed once from the documented sourced Bash environment:

```text
/usr/bin/python3 scripts/stage4_session.py --inputs /tmp/b2w-zero-unblock-sol-20261020/zero-inputs.json --output /tmp/b2w-zero-live-20261021/session --execute-zero-smoke
```

The validation-only invocation preceded this execution and returned0. Existing
90s bootstrap /220s supervisor /150s watchdog /90s probe plus cleanup limits were
unchanged. A separate passive recorder subscribed before launch, recorded six-axis
commands, headers, relevant TF and pose plus control/process snapshots, and had a
260s/32MiB cap. It created no goal, action client, velocity publisher or lease.
Its recorder error is null; it exited normally. The actual qualification probe
was never reached. No production source was instrumented or changed.

## Actual runtime evidence

| Item | Observation | Classification |
|---|---|---|
| Final command |499 observed `/path_manager/path_manager_ros/nav_vel` Twist messages, all six axes exactly0; first arrived before clock, final at observed sim19.1775 |PASS for those499 observations; no claim of lossless transport |
| Upstream commands |No messages on `/course_navigation/cmd_vel_raw` or `/course_navigation/cmd_vel` |NOT_OBSERVED, not independently qualified zero streams |
| Clock/sensors |7690 clock callbacks,192 public pointcloud callbacks,3807 IMU callbacks; stored clock/IMU samples near20Hz |Observed, not a long-run freshness PASS |
| LiDAR source age |192 header samples:0.020–0.0625s relative to most recently received clock |Observed; callback ordering applies |
| Localization |4 `/slam/odom` samples, stamps18.6/18.7/18.8/18.9, `odom→base_link`;8 corresponding public TF edge records,4 each for `map→odom` and `odom→base_link` |Only0.3 source seconds; freshness ages0.2575–0.2775s at receipt |
| Map |One `/slam/map_points` message, map frame stamp18.5, age0.325s; one800×800 `/map`, stamp19.0, age0.055s |Cadence NOT_QUALIFIED: fewer than3 structural publications |
| Dynamic costmaps |No raw local/global costmap message recorded; final prepared grid set empty |NOT_OBSERVED |
| Pose |4 sensor-SLAM poses over0.3 source seconds; max XY excursion0.000225612m, planar speed0.002187914m/s, yaw rate0.000120603rad/s |Insufficient for30sim/5wall stationary acceptance or physical stop |
| Contact reports |0 full-contact and0 inventory ROS messages |FAIL for required observation; no collision-free claim |
| Owner/adapter health |Sampled240 waiting_localization,2 ready,1 fault; last health sequence498, fault adapter_publication_deadline |Startup health observed; bootstrap not ready |
| Watchdog/lease |No watchdog.json, deployment.json, probe directory or lease.json created |Runtime watchdog NOT_OBSERVED; no motion lease issued |
| Stop/cleanup |Owner inhibited; launch interrupted; cleanup passed, remaining{}, all22 launch-started PIDs plus observer/owner absent |PASS for process cleanup; fault-to-physical-stop NOT_QUALIFIED |
| GPU |Final full nvidia-smi shows only unrelated pre-existing desktop/apps; no Gazebo PID; all owned identities absent |PASS for owned GPU process cleanup |

IMU source-age samples span −0.0125 to+0.0125s against the last delivered clock;
separate callback ordering can explain this sign and no strict synchronized IMU
freshness certificate is claimed. No Ground Truth input was used by the passive
recorder or navigation. Existing low-level simulation body feedback is unchanged.

## First credible failure and exact boundaries

`session/stack.log:253–275` records IMU process start, the uncaught GTSAM exception
near bias variable `b0`, and process exit−6. This precedes first public localization
and the later adapter inhibit. The launch file starts the four backend nodes
independently (`course_slam/launch/slam.launch.py:15–17`); the remaining mapping
process still produced the short localization/map interval after IMU process loss.

```text
[lio_sam_imuPreintegration-18] terminate called after throwing an instance of 'gtsam::IndeterminantLinearSystemException'
[lio_sam_imuPreintegration-18] Indeterminant linear system detected while working near variable
[lio_sam_imuPreintegration-18] 7061644215716937728 (Symbol: b0).
[ERROR] [lio_sam_imuPreintegration-18]: process has died [pid 3029437, exit code -6, ...].
```

The available backend source initializes `B(0)` and calls the optimizer at
`third_party/lio_sam/src/imuPreintegration.cpp:335–344`:

```cpp
prevBias_ = gtsam::imuBias::ConstantBias();
gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
graphFactors.add(priorBias);
graphValues.insert(X(0), prevPose_);
graphValues.insert(V(0), prevVel_);
graphValues.insert(B(0), prevBias_);
optimizer.update(graphFactors, graphValues);
```

This is a diagnostic source pointer, not a proven throwing instruction: the run
contains no backend backtrace or factor graph dump, and multiple update sites
exist. An underconstrained bias, numerical/library issue, or input/configuration
cause has NOT been distinguished. No parameter or dependency was changed.

The final adapter fault is separately established by the first inhibit file and
last health record. `velocity_adapter.py:324–326` is exact:

```python
if guard is not None and time.perf_counter()-publication_started>=.05:
    guard.inhibit('adapter_publication_deadline');out=Twist()
self.pub.publish(out)
```

This branch records a callback crossing the existing50ms wall threshold; no direct
per-section profiling establishes compute time versus scheduling delay. It is not
the anticipated source-time TF ValueError: that reason was not latched. The owner
then sees inhibit at `stage4_session.py:92` and enters existing owned cleanup.
Shutdown-only invalid-context errors in several nodes occur after SIGINT and are
not promoted ahead of the earlier backend failure.

There is also a concrete contact-delivery gap: the observer plugin advertises
Ignition transport topics (`scripts/contact_observer/observer.cpp:14–15`), while the
actually used `bridge_rl_sar.yaml` has neither evaluation-topic mapping and the
session launcher starts no separate evaluation bridge. This accounts for an
unwired ROS observation boundary; native reporter/engine coverage itself was not
measured. It remains unresolved, with no patch or retry here.

## Preserved evidence and decision

The [machine evidence record](B2W_Stage4_ZeroOnly_Runtime_Evidence_20261021.json)
contains the actual result, observation summary, per-file raw SHA256 values, exact
paths and cleanup record. Full startup JSONL and stack/control logs remain raw
intermediate evidence under `/tmp/b2w-zero-live-20261021/`; key excerpts and hashes
are included in the permanent APRIL bundle. They are not the final report location.
The unchanged463 tracked files were hash-verified before adding only these review
records. No source fix, PLAN change, commit or push occurred. One existing NOW is
retained; this execution record supplies the current runtime outcome.
Campaign live-stack accounting:211.308248+28.190766270=239.499014270 wall seconds,
zero motion attempts. Overall ZeroOnly_RUNTIME_FAIL; safety_complete:false,
segment_pass:false, full_route_pass:false. No motion readiness is self-approved.

**Exactly one next action:** submit this failed-trial packet to ChatGPT review,
prioritizing the LIO-SAM `b0` abort before any patch or additional trial.
