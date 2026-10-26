# B2-W yaw contract and live normal cleanup — 2026-10-26

Base: `25043f96ee63807b04ab5f1a36d2dc9bd5a1faab`; single agent,
`navigation/g1-g5-recovery-20261025`. Evidence root:
`artifacts/recovery/yaw-contract-cleanup-20261026/` (included in APRIL).
One new stationary Gazebo session; zero motion sessions, goals or retries.

## Actual results and remaining boundary

**Normal cleanup PASS**, using new domain 211, unused output, exclusive launch
lease, unique owner token/IGN partition, unchanged watchdog and deployment manifest
`first-motion-control-risks-20261025/reviewed-deployment.json`.
All 46 source/install pins, native pins and 26 collision shapes passed fresh
preflight; actual server/native mappings, robot description and construction logs
passed the existing live audit. Read `stationary-offline.json`, `preflight.json`,
`pre-motion-snapshot.json` and `session-report.json` in the evidence directory.

The only session is `stationary/stationary-a9e7f5fcb9ec45bbb21df5b1b7eb0fe1`.
Initial/final measured rest: sim 19.08–24.1625 (5.0825 s), wall 5.0809 s,
maximum translation .000255879 m, yaw 2.739e-7 rad. LIO and truth rest,
six-axis exact zero and closed contact coverage passed. 628 final-command samples,
zero nonzero samples. 472 armed contact samples; truth/readiness/contact first
faults null; existing TF/odom .5 s freshness and physical gates unchanged.
All 19 registered owned identities exited within the existing 8 s normal bound;
normal cleanup had no survivors and required no fallback. Watchdog result:
`normal_shutdown`, `emergency=false`. Localization ingress/thread close passed.
Host audit rechecked PID/start ticks and token: no remaining owner/server,
and domain 211 lease was reacquired and released.

Fault containment was **armed but not newly fault-injected live**. The old
stationary fault-injection result remains historical evidence; the current
focused suite verifies failure handling. A normal exit and deliberate destructive
containment of the same server cannot both be live exit outcomes in one session.
No extra session was spent to manufacture that claim.

**Yaw diagnostic NOT RUN**. Existing precheck executes `('forward','turn')`;
setting pulse duration does not select yaw. More importantly, its clock-sampled
loop has no simulator-side expiry: it sends zero only after dispatch crosses the
requested duration, followed by asynchronous Twist/adapter/controller delivery.
The historical 1 s forward pulse lasted 1.0025 s. The adapter's .5 wall-second
staleness timeout and 50 ms steady timer do not enforce a .25 simulated-second
expiry. A yaw-only tuple change would remove forward but would not establish the
strict requested bound. No yaw flag was added, no lower nominal duration was
presented as a guaranteed cap, and no nonzero publication was attempted. A shared
adapter/controller expiry change would require a separately reviewed contract;
this session stops before motion as instructed. The current observer also omits
`/joint_commands`, `/joint_torques` and named `/joint_states`, so a future mode
must record these existing outputs before claiming the requested instrumentation.

## Pinned deployment versus training contract

Pinned rl_sar revision `376d42c9b128f963ab08579762d5a216a976ce39`, zoo assets
`7dd30bdc7806898950b354260655d5a7f0ce844e`, ONNX SHA256
`da31c0c1820b893608e2d02746bbada16d0abb5de6116277e37ccfb02024c564`.
`source-identities.json` hashes exact inspected local sources and compares all
16 movable joint origins/axes. Hip parent link is renamed trunk→base_link;
this is not a literal parent-name match or an axis reversal.

| Contract | Pinned upstream deployment | Actual Fortress deployment |
| --- | --- | --- |
| 0–2 | body angular velocity × .25 (ROS2 branch) | aligned raw IMU body gyro × .25 |
| 3–5 | inverse-quaternion rotated world gravity | normalized quaternion Rᵀ(0,0,−1) |
| 6–8 | body vx, vy, yaw; scales [1,1,1] | same order; yaw additionally ×1.6, clipped ±1 |
| 9–24 | q−default, wheel positions masked to zero | identical, leg defaults [0,.8,−1.5] ×4 |
| 25–40 | joint dq × .05 | identical, feedback mapped by joint name |
| 41–56 | previous 16 actions | previous actions after local idle wheel override |
| Joint order | FR/FL/RR/RL hip-thigh-calf, then FR/FL/RR/RL wheels; identity map | identical names for observations and named force topics |
| Action | leg position offsets [.125,.25,.25] ×4; wheel velocity ×5 | same targets, not direct wheel torque or measured velocity |
| Gains/limits | legs kp160/kd5, wheels kp0/kd1; torque 200/200/320 ×4, wheels20 | same RL gains and torque clamps |
| Timing | dt .005, decimation4 | sim timer .005, every fourth tick inference (50 Hz) |
| Zero transition | policy's unmodified actions | infer legs normally, override wheels to zero when each command abs<.01 or stale |

Concise exact source excerpts (see paths in `source-identities.json`):

```cpp
// pinned rl_sdk.cpp, ComputeObservation/ComputeOutput/RLControl
obs_list.push_back(this->obs.commands * this->params.Get<std::vector<float>>("commands_scale"));
// Wheel relative positions are masked; separate outputs are sent as motor q/dq.
output_dof_vel = vel_actions_scaled;
fsm_command->motor_command.dq[i] = _output_dof_vel[i];
fsm_command->motor_command.tau[i] = 0;
```

The upstream SDK also computes `output_dof_tau`, but RLControl uses q/dq with
kp/kd and tau=0. Its standalone tau expression must not be mistaken for the
wheel velocity actuator actually selected by that path.

```cpp
// local rl_sar_controller.cpp
obs[8]=std::clamp(obs[8]*float(yaw_gain_),-1.f,1.f);
obs[25+i]=dq_[i]*.05;
obs[41+i]=last_[i];
if (idle && i>=12) last_[i]=0;
velocity_target_[i]=i>=12?scale_[i]*last_[i]:0;
tau[i]=std::clamp(kp*(target_[i]-q_[i])+kd*(velocity_target_[i]-dq_[i]),-limits_[i],limits_[i]);
```

All wheel axes are `0 1 0` with matching joint origins. Positive wheel rotation
at nominal stance corresponds to forward rolling: faster left wheels are
consistent with negative yaw. No command/feedback sign flip or left/right swap
was found. Gazebo uses named ApplyJointForce plugins, rather than ideal wheel
velocity tracking. Wheel target−measured dq is the kd1 torque demand, bounded at
±20 Nm; slip, body/leg motion and contact impulses affect actual dq and yaw.
The generator substitutes course inertias/dynamics and Fortress systems, so
matching axes and gains does not establish training-physics equivalence.

**Original training yaw range: unavailable from the pinned artifacts.**
`POLICY_SOURCE.json` pins deployment code/model/zoo assets, but contains no
robot_lab training commit, run configuration, seed or USD/URDF checksum for the
training run. Exported `commands_scale=[1,1,1]` is a scale, not a sampled range.
The local ±1 yaw clamp is likewise not evidence of a training range.
Current upstream [B2W configuration](https://raw.githubusercontent.com/fan-ziqi/robot_lab/main/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/unitree_b2w/rough_env_cfg.py)
and [base configuration](https://raw.githubusercontent.com/fan-ziqi/robot_lab/main/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/velocity_env_cfg.py)
show the same joint groups/scales and inherited ang_vel_z ±1 with heading-command
sampling; the B2W ±1.5 override is commented. These are mutable 2026 sources,
**not provenance for this exported weight**. No original range, sign convention
or small-command training coverage is inferred from them.

Historical local settings in `docs/LOCOMOTION_STATUS.md`: unmodified zero-command
policy drift ~.066 m/s led to wheel-only idle braking; fixed-leg idle caused
pitch/falls and was reverted. Gain1.6 was a local calibration for weak turning,
not an upstream training parameter. Historical .25 rad/s for8 s reported positive
2.268 rad on a different test; it does not qualify this .05 rad/s transient,
prove all-turn sign reversal, or authorize replacing current strict admission.

## Command → model → wheels → truth chronology

Old raw recording: `live-qualification-short-motion-20261025/B/precheck-610cfc41a0104ddb821ed77e0fea2085`.
No new physical yaw result is implied. `contract-analysis.json` and its offline
script use receive-clock alignment for unstamped network arrays; torque estimates
below are reconstructed at policy samples, **not measured 200 Hz actuator torque**.
Previous 95-pair ONNX replay parity (≤1.01328e-6) remains the independent output check.

1. Before turn, sim32.0–32.2: exact zero and idle wheel targets0; wheel dq
   right−left≈.0001 rad/s, gyro wz≈0.
2. Owner requests +.05 yaw at32.2; adapter final is +.05 and model receives +.08.
   During first .1 s, inferred applied wheel target right−left≈−2.855 rad/s,
   measured dq difference≈−.769, gyro≈−.0807 rad/s. By next .1 s gyro≈−.1905.
3. Across the old one-second turn, raw target means in FR/FL/RR/RL order are
   [.0375,1.5719,−2.2389,2.7375] rad/s; measured dq means
   [.1202,.7372,−.4778,1.0730]. Target and feedback both favor left forward speed.
   Independent quaternion yaw delta−.154184 rad and raw gyro mean−.154735 rad/s
   confirm physical negative yaw, not a TF heading or quaternion wrap error.
4. At33.2 owner sends exact zero. First .1 s contains transition samples.
   At33.3–33.5 wheel targets are all0, measured right−left≈+1.698 rad/s,
   body gyro≈+.1651 rad/s. Legs remain policy controlled; wheel torque law now
   brakes dq. Coast net yaw+ .151055 rad; the transient does not disappear because
   final net yaw is small. Whole response takes several seconds to measured rest.
5. Old turn contact stream:21 received samples, each includes all four named
   wheel/ground contacts. All1728 contact entries lack wrenches. Contact presence
   proves neither equal left/right load nor no slip; ground torque contribution
   cannot be reconstructed from this stream. Actual per-wheel torque was not
   recorded by the old observer. Network dq is measured joint velocity, not target.

Additional offline intervention holds every input except obs[8] fixed, without
sending commands or rolling physics. Across10 pre-turn idle observations, raw
right−left wheel targets for yaw −.08/0/+.08 are +.204/−1.503/−2.751 rad/s.
Across48 active-turn observations they are −.522/−1.263/−3.274. The zero case
here is raw inference **before** idle override. This shows command-dependent
negative wheel differential already in this model at these states, including
pre-turn rest. It excludes an explanation consisting only of wheel-servo lag or
an upstream adapter sign error. It does not prove that those wheel differences
alone determine body yaw: coupled leg actions/contact and state feedback remain.

**Narrowest demonstrated cause:** positive requested yaw produces negative
wheel differential at policy output in the present deployment, followed by actual
negative dq/gyro/yaw. Exact zero switches the wheel target contract while learned
leg stabilization continues, and opposite physical wheel/yaw motion follows.
A globally reversed training convention, a particular mapping bug, asymmetric
contact torque or idle override as the sole cause of rebound is **not proven**.
No yaw negation, vx scaling, policy replacement or gain/idle change is justified.

One discriminating next test, requiring a new session/authorization: run the
already requested single pure+.05 yaw≤.25 simulated seconds from verified rest,
with an enforced simulation-time expiry and existing safety contract, recording
named wheel targets/dq/torques and body/leg/contact transition through zero.
It distinguishes a model-origin reversed initial differential from a
contact/actuation-only reversal and locates the rebound relative to the zero
transition. This is a transient diagnostic, not steady-state control evidence.
Do not substitute a longer run or request a speculative suite.

## Minimal change, review and route blocker

Only production change: explicit `--stationary-normal-cleanup` valid exclusively
with `--stationary-containment`. After existing zero/rest/audits it takes the
committed owned SIGINT normal-cleanup path, keeping watchdog/failure/fallback and
8 s survivor checks. Default stationary fault-injection and forward/turn precheck
behavior stay unchanged. Seven new mode/failure cases plus one CLI rejection
are covered; initial new normal-mode test failed on the old injection branch,
then passed. Final focused suite174 PASS (33 precheck tests); no control build
needed because controller/model/interfaces were untouched. Single-agent scoped
review PASS; no independent reviewer claimed, per requested single-agent scope.

Exact Start→G2 blocker: controllable yaw and directional stopping/route envelope
remain unqualified, and the strict bounded pure-yaw diagnostic is not available
in the current command lifecycle.145-pose7.213m Start→G1 planning evidence remains;
actual Start→G2 and continuous Start→G5 video NOT DONE. Navigation .3m tolerance
and staging .05m remain distinct. Main reconciliation branch/local files and
Stage4 are preserved. No further runtime action after APRIL delivery.
