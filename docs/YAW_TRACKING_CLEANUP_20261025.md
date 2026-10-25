# Yaw tracking and normal cleanup diagnosis — 2026-10-25

Scope: offline only, single agent. Base `557b856cb4c6240392a1e5e21ad653a6febb5fcd`,
branch `navigation/g1-g5-recovery-20261025`. No Gazebo, motion, route goal,
hardware, policy/config tuning or live retry. Cleanup fix passes offline review;
yaw/velocity tracking and route qualification remain unresolved.

Evidence root: `artifacts/recovery/live-qualification-short-motion-20261025/`.
New replay scripts/results: `artifacts/recovery/yaw-cleanup-20261025/` in the
APRIL bundle. Historical raw evidence is unchanged.

## Command, model and measurement trace

The Profile-B session report specifies pure `(vx,wz)=(.05,0)` and `(0,.05)`
pulses at simulation times 24.845–25.8475 and 32.2–33.2. The first sampled
loop exceeded its requested 1s duration by .0025s; this remains a limitation.
`monitor-events.jsonl` records the adapter's final command segments with the
same positive signs. No active TF/odom freshness fault preceded shutdown.

Exact source snippets, unchanged by this fix:

```python
# course_navigation/course_navigation/velocity_adapter.py, VelocityGate
self.command = x * scale, y * scale, max(-self.limits[1], min(self.limits[1], yaw))
# VelocityAdapter.command
self.gate.set_command((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic())
# VelocityAdapter.publish
out.linear.x, out.linear.y, out.angular.z = self.gate.output(time.monotonic())
```

```cpp
// b2w_sim/b2w_controllers/src/rl_sar_controller.cpp
command_={std::clamp(m->linear.x,-1.,1.),std::clamp(m->linear.y,-.5,.5),std::clamp(m->angular.z,-1.,1.)};
gravity_=rot.normalized().toRotationMatrix().transpose()*Eigen::Vector3d(0,0,-1);
gyro_={v.x,v.y,v.z}; body_time_=stamp; have_body_=true;
obs[i]=gyro_[i]*.25;
obs[3+i]=gravity_[i];
obs[6+i]=command_valid?command_[i]:0;
obs[8]=std::clamp(obs[8]*float(yaw_gain_),-1.f,1.f);
velocity_target_[i]=i>=12?scale_[i]*last_[i]:0;
tau[i]=std::clamp(kp*(target_[i]-q_[i])+kd*(velocity_target_[i]-dq_[i]),-limits_[i],limits_[i]);
```

`course_sim/config/rl_sar.yaml` sets `yaw_command_gain: 1.6`, `idle_hold: true`,
wheel action scale 5, joint order FR/FL/RR/RL followed by those four wheels.
The source uses name-based feedback mapping and matching named force topics.
Generated trial `artifacts/recovery/robot.urdf` and the pinned zoo URDF have
identical movable-joint origins/axes (all wheel axes `0 1 0`); the IMU fixed
joint has rpy `0 0 0`. Generator `course_sim/course_sim/description.py` changes
inertias/dynamics and installs named ApplyJointForce plugins; it does not reverse
joint axes or command signs. This rules out an explicit sign flip in the traced
adapter/controller chain and a generated-vs-zoo joint-axis mismatch. It does not
prove the policy's original training convention.

Pinned upstream `third_party/b2w_rl_sar/src/rl_sar/library/core/rl_sdk/rl_sdk.cpp`
uses `obs.commands * commands_scale`, body gyro scaled by `.25`, inverse-rotated
gravity, masked wheel positions and wheel velocity outputs. Its selected YAML
specifies command scale `[1,1,1]`, the same 57D ordering and identity joint mapping.
The local positive yaw gain and idle wheel override are explicit deployment
extensions, not upstream behavior.

The recorded `/network_input_debug` removes uncertainty about what reached the
model: 47/50 forward-window inputs have vx `.050000000745`; 49/50 turn-window
inputs have wz `.080000005662`. Remaining samples are transition zeros.
Debug arrays are unstamped: these windows use observer receive simulation clock,
not exact actuator timestamps. Positive yaw reached the model; adapter sign/scaling
is not the cause of the negative response.

Offline ONNX replay of all 95 adjacent active-input pairs agrees with the next
recorded previous-action vector to maximum absolute error `1.01328e-6`.
Policy ONNX SHA256 is `da31c0c1820b893608e2d02746bbada16d0abb5de6116277e37ccfb02024c564`.
There is no evidence of an incorrect ONNX conversion or different inference
output in this recording. The existing source/install/native identity audit
also matched before the trial.

During the turn, mean measured wheel rates in FR/FL/RR/RL order are
`[.120235, .737237, -.477843, 1.072980] rad/s`; raw replay wheel velocity
outputs average `[.037513, 1.571896, -2.238897, 2.737459] rad/s`.
With the traced axes, the left wheels' greater forward speed is consistent with
negative yaw. This is a low-level policy/deployment response mismatch, already
present in wheel outputs/feedback, not just a displayed navigation heading.
These quantities alone cannot separate learned leg motion, wheel contact/slip,
feedback dynamics or a mismatch with the original training joint convention.

| Independent measurement | During +yaw pulse | After pulse to measured rest |
|---|---:|---:|
| Gazebo quaternion yaw change | −.154183956 rad | +.151054890 rad |
| Gazebo twist mean wz | −.151656221 rad/s | +.019335660 rad/s |
| Raw body gyro inferred from debug (`obs[2]*4`) | −.154735277 rad/s | +.019290466 rad/s |

Yaw stays between −.215 and −.056 rad, far from wrapping. Quaternion differences
and raw body gyro independently confirm the sign and exclude a yaw-unwrapping
or OdometryPublisher-only measurement artifact. After zero, yaw first continues
to −.214355 rad then reverses. This is braking/relaxation dynamics; the final
near-zero net yaw must not erase the transient negative turn.

The historical forward mean/peak `.097012/.141762 m/s` uses an assumed twist
frame rotation in `measure.py`. Do not rely solely on that assumption: direct
position differences rotated into the body frame independently yield mean
`.099100`, peak `.141945 m/s` over the recorded .99s pose interval. Excessive
forward tracking is real. The replay confirms vx remains `.05` at model input;
there is no demonstrated missing factor that authorizes scaling vx.

**Unresolved causal boundary:** the same pinned model, correct traced command
sign and positive input produce a negative short-turn response in this deployment.
No steady-state turn, opposite-sign trial, or training environment action/frame
contract is available in this trial. A one-second pulse plus opposite recovery
cannot prove a globally reversed yaw convention, nor prove that reversing input
would provide stable feedback. No yaw negation, vx scaling, model replacement,
or gain/idle tuning is justified by these data. No yaw fix is claimed.

## First cleanup failure and minimal correction

After both measured stops passed, Owner enters shutdown. The original code was:

```python
for process in reversed(children):
    if process.poll() is None: process.send_signal(signal.SIGINT)
# eight wall seconds, spinning Owner and polling children
remaining=owner.scan()
if remaining: raise RuntimeError('Normal owned cleanup incomplete')
```

Every launch uses `start_new_session=True`. A PID-only signal is not a terminal
process-group signal. Installed Humble `/opt/ros/humble/lib/python3.10/site-packages/ros2run/api/__init__.py`
contains:

```python
process = subprocess.Popen(cmd)
while process.returncode is None:
    try:
        process.communicate()
    except KeyboardInterrupt:
        # the subprocess will also receive the signal and should shut down
        # therefore we continue here until the process has finished
        pass
```

That assumption fails for Owner's PID-only signaling. It signals wrapper roots,
not the helper nodes they spawn. The observer handles SIGINT; ros2 launch
explicitly forwards SIGINT and its stack exits, as `stack.log` shows.

First failing predicate: nonempty `owner.scan()` after the **8s normal cleanup
bound**, reported as `RuntimeError('Normal owned cleanup incomplete')`.
The failed normal map was overwritten by fallback in the historical report;
there is no exact saved normal-bound per-thread snapshot. The watchdog's next
ownership scan identifies these six surviving process identities (PID/start ticks):

- `3099131/29241203`: contact-bridge ros2-run root.
- `3099133/29241207`: precheck-map ros2-run root.
- `3099134/29241209`: precheck-adapter ros2-run root.
- `3099135/29241215`, `3099139/29241222`, `3099140/29241222`: their helper descendants.
  The bridge's earlier start and native identity identify the first; the record
  does not persist a definitive map-vs-adapter child PID assignment for the last two.

This is not a localization-thread timeout: ingress closes successfully later.
Gazebo server `3099161/29241233` is already absent at watchdog intervention;
its pause result is `server_identity_or_owner_mismatch`, not containment PASS.
Last clock receipt is wall `292463.680733`; failure heartbeat is `292471.111402`.
The readiness/truth stale snapshots occur after streams stopped, not in motion.
Contact-bridge logs its signal handler at wall UTC `1792939936.197456`, roughly
8s after the stack shutdown handlers around `1792939928.15`.

Watchdog fallback uses token/identity-checked TERM then KILL, and empties its
six-process map. Owner fallback subsequently reports no remaining processes;
final host evidence confirms this. Fallback succeeds because it reaches the
actual helper PIDs, not because TF was repaired or a wait was extended.

The fix replaces only root SIGINT delivery with existing
`owner.send(signal.SIGINT)`, whose scan requires the inherited unique token and
whose send rechecks PID start ticks. The unchanged 8s deadline, nonempty check,
watchdog disarm predicates, fallback and localization close remain. Watchdog is
outside the token-owned set. No group/name signaling or simulator resume is added.
`normal_cleanup` preserves the map and owned identity snapshot before fallback
can replace `cleanup`, so failure remains visible and success remains false.

## Verification, review and next boundary

Two minimal regression tests exercise real trial control flow with simulated
ROS/process boundaries: wrapper descendants must receive normal shutdown, and a
stuck owned child must retain its failed normal snapshot after fallback succeeds.
Both fail on the base implementation and pass with the fix. The existing precheck
suite passes 25 tests. Related ownership, containment, zero/stop, physical veto,
contact, freshness/readmission and localization suites pass **185 tests** in total.

A separate process-only reproduction invokes the **installed ros2run wrapper**
with a harmless sleeping Python child. PID-only SIGINT leaves both alive;
`OwnedProcesses.send(SIGINT)` closes both with wrapper exit 0, while an unrelated
process stays alive. No ROS node, DDS, Gazebo or motion is used in that reproduction.
The wider suite includes its existing isolated ROS ingress test, without simulation.
First test invocation lacked install setup and a writable ROS log directory:
181 passed/4 environment failures. After sourcing this checkout's install and
using `/tmp` for ROS logs: 185 passed. Raw outputs are retained; no checks removed.

Single-agent diff review: PASS for the cleanup correction and evidence claims,
**not** for motion or route qualification. Unchanged: `.5s` TF/odom freshness,
physical/contact/collision vetoes, zero delivery, stop proof, watchdog containment,
route admission, policy/config/native code and interfaces. No wholesale merge.
Main remains `51870e1` with all 24 local paths; Stage4 checkout untouched.

Smallest useful next live boundary, requiring a new session's authorization:
first stationary measured rest followed by normal cleanup to confirm the helper
fix on the real stack. For motion, resolve/verify the original training joint and
frame contract offline before selecting a controller change. If a bounded physical
diagnostic is then authorized, use one pure +`.05 rad/s` pulse capped at `.25s`,
zero through the existing adapter, and full measured rest plus normal cleanup.
Keep the existing observed-free disk, `.5s` freshness, physical/contact gates,
wall deadline and independent watchdog; stop on any veto and do not retry.
Capture existing debug inputs plus named joint feedback/targets/torques and truth
for transition timing. This is a diagnostic proposal, not a tracking PASS or a
reason to repeat an unchanged one-second test. Do not extend motion or attempt
Start→G2 until tracking/stopping and the route envelope actually pass.

The existing 145-pose Start→G1 path remains valid planning evidence. Real
Start→G2 and continuous Start→G5 video remain NOT DONE. Navigation tolerance
`.3m` and staging tolerance `.05m` remain distinct. Stop after APRIL delivery.
