# B2-W low-speed calibration: offline audit and USER-RUN protocols

Scope: 2026-10-16, branch `improve/narrow-navigation`, committed HEAD
`460111a9a37105874df694235b0b561906779cf5`. Measurement/audit/tooling only.
No ROS/Gazebo session, nonzero command, parameter/model change, commit or push
was performed in this stage. Root `PLAN.md` is the sole current plan.

## AGAINST / FOR / neutral verdict, before selecting a model

**AGAINST command calibration as explanation/remedy:** a learned feedback policy
maps observations and history as well as command into actions; one ascending run
cannot establish a state-independent function. Per-wheel tracking error, coupled
contact/leg motion, unknown training actuator equivalence and session variability
can contribute. An input inverse will not remove braking distance or yaw drift.
The radius-based approximation is not a no-slip proof.

**FOR:** final command and actual ONNX command match request; the mean wheel
target already encodes nearly all observed positive physical speed bias.
Actual mean wheel rate and truth agree closely with the target mean in this run.
This is evidence for a stable mapping candidate worth testing in fresh sessions.

**Neutral verdict:** the leading hypothesis moves toward **positive bias already
present in the policy's average command→wheel-target mapping (A)**. A plant-only
explanation in which an appropriate average target becomes an oversized average
wheel rate (B) is weakened. A is not unique causality: large individual target
errors remain; this is a closed-loop policy/plant equilibrium. Fresh repeats,
low-speed transition data and independent validation are required. No production
calibration model is selected.

The hypothesis is `actual physical vx = f(policy vx command)`, with explicit
`f(0)=0`, monotonic positive moving branch only if observations support it,
no extrapolation, and the lowest complexity surviving fresh validation. A moving
affine relation is provisional and is not a global line through zero.

## Source authentication and geometry

The existing reviewed capture is
`/tmp/b2w-velocity-steps-sol/20261015T103630976957Z`.
`events.jsonl` SHA256 is
`b21194c17e73985f7cd24f78fd5a6e965e91bf1c5c9979261a4465edf9099e98`.
Its `manifest.json` contains `source_hashes`; the active controller, configuration,
URDF and generator hashes match current committed files. This stage reconstructed
the launch-generated robot offline and converted it with `ign sdf -p` (no server).
The historical capture did not save the live robot-description payload; the
reconstruction authenticates its hash-matched source chain, not a historical
in-memory dump. Installed launch and URDF resolve to these same source files.

| Geometry source | Radius / dimensions | Evidence: path + key snippet + interpretation |
|---|---|---|
| Manufacturer nominal | 225 mm diameter → 0.1125 m radius | [Unitree B2-W](https://www.unitree.com/b2-w/): `Wheel Diameter 225mm`; external hardware specification only. |
| Deployed four collision cylinders | radius 0.1125 m; axial length 0.02 m; centers y=±0.04 m | `b2w_sim/b2w_description_ros2/urdf/b2w_description_rsl.urdf`, each `*_foot/collision`: `<cylinder length="0.02" radius="0.1125"/>`, `rpy="1.5708 0 0"`; collision rolling radius is locally explicit. |
| Reconstructed launch URDF and converted SDF | same four radii 0.1125 m, length .02 m | `/tmp/b2w-calibration-audit-20261016/deployed-torque.urdf` and `.sdf`: `<radius>0.1125</radius>`; Fortress conversion retains cylinders. |
| Deployed visual meshes | max radial distance in joint x-z plane: FR/RR .113707585 m, FL/RL .114018126 m; each axial width .050000009 m | `b2w_sim/b2w_description_ros2/meshes/{FR,FL,RR,RL}_foot.dae`: `<unit ... meter="1"/>`, scene matrix spatial scale `.001`, geometry POSITION vertices. Apply scene matrix to vertices, then `max(hypot(x,z))`; these are noncircular mesh bounds, not an analytic radius. |
| Vendored rl_sar_zoo collision AND visual | mesh-based; same transformed bounds as deployed visuals | `third_party/b2w_rl_sar/robot_assets/b2w_description/urdf/b2w_description.urdf`: `<mesh filename="package://b2w_description/meshes/FR_foot.dae"/>` (all four collision/visuals). Parsed transformed positions match local visuals although DAE file hashes differ. |
| WheelSlip plugin | .1125 m all four; normal profile compliance=0 | `course_sim/course_sim/description.py`: `radius=float(collision.get('radius')) ...`; `sub(wheel,'wheel_radius',radius)`. The `.1125` fallback is not used here. Zero compliance is a plugin parameter, not measured absence of sliding. |
| Pinned candidate robot_lab reference | numerical effective radius unresolved | `/tmp/b2w-control-chain-plan/official/robot_lab_unitree.py`, `UNITREE_B2W_CFG`: `asset_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/unitree/b2w_description/urdf/b2w_description.urdf"`, `replace_cylinders_with_capsules=False`. [Pinned 500399e source](https://github.com/fan-ziqi/robot_lab/blob/500399ed75f510aeaff28705a8ce736c514dbec3/source/robot_lab/robot_lab/assets/unitree.py) names an external Isaac Lab asset, without binding its bytes to this checkpoint. It does not prove a training rolling radius. |

Mismatch: deployment collision is an ideal .1125 m cylinder, narrower than the
~.05 m mesh; visual/upstream maximum radial extents exceed it by 1.208 mm on
right wheels and 1.518 mm on left wheels. These are geometric differences, not
an estimated dynamic effective radius. `description.py` copies upstream inertias
and optional joint dynamics while retaining the course collision geometry:
`link.insert(0, copy.deepcopy(source))` for **inertial**, and same-name
`joint/dynamics` copy. It does not copy upstream collision meshes.
`scripts/prepare_rl_sar.py` downloads/authenticates the zoo bundle; its
`POLICY_SOURCE.json` pins zoo `7dd30bdc7806898950b354260655d5a7f0ce844e`, version
1.0.2. No local conversion overrides these cylinder dimensions.

Only **.1125 m deployed collision radius** is used below, and only for approximate
wheel-spin predictions. Relative joint dq omits parent calf/thigh and body angular
rates, wheel-center velocity, load/contact allocation and possible slip. A matching
radius is insufficient for a no-slip claim. Mesh maximum bounds are unsuitable
as a replacement rolling radius.

## Wheel signs, order, units, and time windows

Evidence: `course_sim/config/rl_sar.yaml`, `joint_names` is FR,FL,RR,RL
hip/thigh/calf followed by `FR_foot_joint, FL_foot_joint, RR_foot_joint,
RL_foot_joint`; `action_scale` ends `[5,5,5,5]`, wheel `rl_kp=0`, `rl_kd=1`,
wheel `torque_limits=20`. In
`b2w_sim/b2w_controllers/src/rl_sar_controller.cpp`:

```cpp
obs[6+i]=command_valid?command_[i]:0;
obs[25+i]=dq_[i]*.05;
auto tensor=Ort::Value::CreateTensor<float>(mem,obs.data(),obs.size(),shape.data(),2);
// the exact inference input is published, not a reconstructed desired command:
m.data.assign(obs.begin(),obs.end()); debug_->publish(m);
velocity_target_[i]=i>=12?scale_[i]*last_[i]:0;
m.data=target_; for (size_t i=12;i<16;++i) m.data[i]=velocity_target_[i]; actions_pub_->publish(m);
tau[i]=std::clamp(kp*(target_[i]-q_[i])+kd*(velocity_target_[i]-dq_[i]),-limits_[i],limits_[i]);
```

Interpretation: `/network_input_debug.data[6]` is actual consumed ONNX vx after
float conversion; `/joint_commands.data[12:16]` is already scaled wheel velocity
target in rad/s. **Do not multiply those targets by five again.** Wheel dq is
looked up by `JointState.name`, not assumed sensor array order. The recorder
`velocity_step_recorder.py` stores `name`, `velocity` and header stamps intact.

Geometry evidence: all `*_foot_joint` have `axis xyz="0 1 0"`, chain static
origins `rpy="0 0 0"`; hips rotate about x, thigh/calf about y. At bottom contact
`omega × (-r z)` points -x for +y spin, so forward hub motion is +x.
Define `omega_forward[j]=sign(axis_body_y)*dq[j]`; all four signs are +1 in
this chain, including right wheels. Runtime hip |q|<pi/2 is checked for supported
windows. This defines a forward sign, not a full pose-corrected body model.
Synthetic tests deliberately permute joint order and reverse left axes to detect
signed-average mistakes.

All statistics below use final fixed two-second steady windows, half-open:
[20.33,22.33), [32.33,34.33), [44.33,46.33), [56.33,58.33) sim seconds.
Commands, policy inputs and targets use causal receive-time ZOH; actual dq and
body twists use supported native-source segments with the existing core's
half-open boundary rule. Targets have 100% coverage; actual/error coverage is
99.50–99.875%. A max-gap guard and ≥95% coverage apply; missing data is null.
Tracking errors align actual source time with the latest received target ≤50 ms
old; receipt times are not actuator application timestamps. Tiny differences
between difference-of-means and mean aligned error reflect sampling/alignment.

The following tables are generated from the NEW offline output
`/tmp/b2w-calibration-audit-20261016/reviewed-four-speed/calibration.json`.
Existing raw and derived results were not rewritten.

| request | raw/shared/final | ONNX obs[6] | mean target ω | mean actual dq | r target | r actual | truth vx | SLAM vx | truth−r actual | SLAM−r actual |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.100000 | 0.100000/0.100000/0.100000 | 0.100000001490 | 1.493986 | 1.500083 | 0.168073 | 0.168759 | 0.168775 | 0.170325 | 0.000016 | 0.001565 |
| 0.200000 | 0.200000/0.200000/0.200000 | 0.200000002980 | 2.403414 | 2.407043 | 0.270384 | 0.270792 | 0.270770 | 0.270678 | -0.000023 | -0.000114 |
| 0.300000 | 0.300000/0.300000/0.300000 | 0.300000011921 | 3.301412 | 3.300566 | 0.371409 | 0.371314 | 0.371280 | 0.371800 | -0.000034 | 0.000486 |
| 0.400000 | 0.400000/0.400000/0.400000 | 0.400000005960 | 4.083375 | 4.089076 | 0.459380 | 0.460021 | 0.459378 | 0.459179 | -0.000643 | -0.000842 |

| request | truth−r target | SLAM−r target |
|---:|---:|---:|
| 0.10 | 0.000702 | 0.002251 |
| 0.20 | 0.000386 | 0.000294 |
| 0.30 | -0.000129 | 0.000391 |
| 0.40 | -0.000001 | -0.000200 |

Wheel vectors are **FR, FL, RR, RL**, rad/s; all are forward-equivalent (+ signs here).

| request | each target ω | each actual dq | each aligned target−actual | mean signed error | aggregate MAE | aggregate RMSE |
|---|---|---|---|---|---|---|
| 0.100000 | 1.575610, 1.255309, 0.610440, 2.534586 | 1.499903, 1.500511, 1.499881, 1.500037 | 0.075711, -0.245195, -0.889442, 1.034561 | -0.006091 | 0.561227 | 0.694195 |
| 0.200000 | 2.319138, 2.170015, 1.399012, 3.725491 | 2.406590, 2.407724, 2.405718, 2.408139 | -0.087478, -0.237731, -1.006724, 1.317334 | -0.003650 | 0.662317 | 0.838675 |
| 0.300000 | 2.851890, 3.444044, 1.611439, 5.298274 | 3.293916, 3.312428, 3.285046, 3.310875 | -0.442005, 0.131523, -1.673499, 1.987281 | 0.000825 | 1.058577 | 1.319535 |
| 0.400000 | 3.351977, 5.100169, 1.065900, 6.815453 | 4.061861, 4.139119, 4.060072, 4.095250 | -0.709810, 0.961039, -2.994093, 2.720209 | -0.005664 | 1.846288 | 2.114465 |

| request | target max−min | actual max−min | target left−right | actual left−right | target lateral / yaw | stop lateral / yaw | peak steady |τ|/limit (all joints) | wheel peak |τ|/20 |
|---|---|---|---|---|---|---|---|---|
| 0.100000 | 1.924146 | 0.000630 | 0.801923 | 0.000382 | -0.005809/-0.011252 | -0.002205/0.001339 | 0.140974 | 0.004716, 0.012788, 0.044860, 0.052607 |
| 0.200000 | 2.326479 | 0.002421 | 1.088678 | 0.001777 | -0.030493/-0.031598 | -0.003551/0.015051 | 0.136851 | 0.005096, 0.012520, 0.051034, 0.066592 |
| 0.300000 | 3.686835 | 0.027382 | 2.139495 | 0.022170 | -0.111521/-0.091562 | -0.005312/0.059894 | 0.144022 | 0.023235, 0.008096, 0.085520, 0.100869 |
| 0.400000 | 5.749552 | 0.079048 | 3.748872 | 0.056218 | -0.179595/-0.128456 | -0.006959/0.087593 | 0.200454 | 0.040881, 0.059481, 0.161613, 0.140535 |

Units: lateral m, yaw rad. The coupling columns cover the whole target/stop
phases, not just steady windows. Torque ratios here cover steady windows; no
sample reaches saturation. Raw peaks alone cannot prove every interval was observed.

Interpretation with evidence: the first table's `r target` from authenticated
`/joint_commands.data[12:16]` is already above policy input by roughly
.059–.071 m/s. The actual mean differs from target mean by <.0061 rad/s
(<.00069 m/s), whereas per-wheel mean absolute errors are .561–1.846 rad/s.
Thus the mean supports A while cancellation prevents claiming uniformly good
actuator tracking. The torque law above permits different positive/negative
wheel torques at nearly common contact-constrained rates; no unique contact
mechanism is proven. Target asymmetry and the recorded yaw/lateral coupling
remain relevant and are not repaired by a scalar command inverse.

## Candidate models: analysis only

The offline tool fits the actual consumed policy input, not desired corrected
physical velocity. Truth and SLAM are fitted separately. Each run-window is one
observation; there is **one independent run and four distinct speeds**, with one
observation at each speed. Per-speed repeat std is null, not evidence of zero
variance. M3 interpolates knot means and its zero training error is expected.
The report includes MAE/RMSE/max error, signed residuals, R², monotonicity,
LOSO, repeatability, measured/fitted domains, and diagnostic inverses. No
polynomial, neural model or cubic fallback is implemented.

| truth model | parameters | MAE | RMSE | max error | R² | LOSO MAE (interior only) |
|---|---|---|---|---|---|---|
| M0 | {"k": 1.0} | 0.067551 | 0.067722 | 0.071280 | 0.612359 | 0.071025 |
| M1 | {"k": 1.220555748988301} | 0.026834 | 0.030625 | 0.046719 | 0.920728 | 0.019032 |
| M2 | {"a": 0.972320206285006, "b": 0.07447066447553755} | 0.003474 | 0.003683 | 0.005113 | 0.998854 | 0.004963 |
| M3 | [[0.10000000149011612, 0.16877498209195232], [0.20000000298023224, 0.27076961131634103], [0.30000001192092896, 0.3712800463437134], [0.4000000059604645, 0.4593782461681995]] | 0.000000 | 0.000000 | 0.000000 | 1.000000 | 0.003474 |

M2 is fitted **only to nonzero moving observations**. Its positive intercept
must not be evaluated at zero or in the unmeasured low-speed gap. All four
candidate moving branches are monotonic in this dataset. Their measured input
support is approximately [.10,.40] m/s; measured truth support is
[.168775,.459378] m/s. Diagnostic inverse support additionally intersects each
candidate's fitted output range with that measured output range. `f(0)=0` is
an isolated explicit physical state, without pretending the unmeasured gap has
been characterized. Endpoint leave-one-speed-out predictions are null because
extrapolation is forbidden; only two interior speeds are identifiable. LOSO
within the same ascending run is not independent fresh validation.

M1's systematic residuals (+.046719,+.026658,+.005113,−.028844 m/s) make a
single proportional scale a weak description here. M2's residuals
(−.002928,+.001835,+.005113,−.004021) support testing an affine moving branch,
without concluding it is correct. M3 has more flexibility and awaits evidence
that simpler models fail fresh validation. None is approved for production.

## USER-RUN Protocol A: repeatability first

**DO NOT RUN UNTIL CHATGPT REVIEW PASSES. These commands are prepared, not
executed or currently authorized for motion.** Run sequentially, after reviewing
this change. Each invocation owns a fresh simulation session; wait for its
completed/cleanup result before starting the next. No manual direct `nav_vel`
publisher is needed. Domain 194 must be free; the existing harness checks leases
and command ownership rather than killing other sessions.

```zsh
# DO NOT RUN UNTIL CHATGPT REVIEW PASSES
cd /home/zyd/program/b2w-embodied-navigation-course
source /opt/ros/humble/setup.zsh
source install/setup.zsh
/usr/bin/python3 scripts/run_velocity_steps.py --speeds 0.20 --ros-domain-id 194 --gui --output-root /tmp/b2w-user-calibration-A1
/usr/bin/python3 scripts/run_velocity_steps.py --speeds 0.20 --ros-domain-id 194 --gui --output-root /tmp/b2w-user-calibration-A2
/usr/bin/python3 scripts/run_velocity_steps.py --speeds 0.20 --ros-domain-id 194 --gui --output-root /tmp/b2w-user-calibration-A3
```

Each is exactly one **12-s simulation-time block**: 3 s zero → 5 s .20 → 4 s
zero, excluding readiness and final zero flush; timer completion may occur at
the next existing 20 Hz callback. Policy-command inputs are .20, not calibrated
desired physical speeds. vy=wz=0; 20 Hz raw publisher, unchanged smoother and
adapter, truth evaluation only. Every session resets the simulation via a fresh
launch, not an in-place reset. Keep all native artifacts and partial failures.

After all three successful cleaned sessions, offline aggregation (safe without
ROS/Gazebo; fresh output path required):

```zsh
/usr/bin/python3 scripts/fit_velocity_calibration.py /tmp/b2w-user-calibration-A1/20* /tmp/b2w-user-calibration-A2/20* /tmp/b2w-user-calibration-A3/20* --output-dir /tmp/b2w-user-calibration-A-fit-01
```

Review policy target mean **and all four targets**, actual wheel rates, truth and
SLAM steady vx, yaw/lateral target and stopping drift, requested/final-zero stop
delays and stopping path. `metric_repeatability` reports mean/sample std/range
and supported run counts for these metrics. Stable targets with varying actual
rates/body response suggest session/plant variation; variable targets at matched
policy input show closed-loop state/history sensitivity. Three runs describe
repeatability only coarsely and do not identify a universal inverse. A failure or
invalid window remains explicit; do not treat it as zero or silently replace it.

## USER-RUN Protocol B: low-speed mapping, AFTER Protocol A review

**DO NOT RUN UNTIL CHATGPT REVIEW PASSES. Protocol A data review must pass
before this separate sweep is authorized. Not executed in this stage.**

```zsh
# DO NOT RUN UNTIL CHATGPT REVIEW PASSES; Protocol A review must pass first
cd /home/zyd/program/b2w-embodied-navigation-course
source /opt/ros/humble/setup.zsh
source install/setup.zsh
/usr/bin/python3 scripts/run_velocity_steps.py --speeds 0.04,0.07,0.10,0.13,0.20 --ros-domain-id 194 --gui --output-root /tmp/b2w-user-calibration-B
```

Exactly five 12-s blocks = **60 simulation seconds**, excluding readiness/flush.
These values are policy-command experiment inputs, **not desired corrected
physical velocities**. No .04/.07/.13 behavior has been validated. The goal is
to locate transition/dead-zone behavior and test whether the positive moving
branch remains approximately affine. Preserve monotonicity violations and
coverage/slope failures; do not force an inverse through them.

Offline after the user sweep:

```zsh
/usr/bin/python3 scripts/fit_velocity_calibration.py /tmp/b2w-user-calibration-B/20* --output-dir /tmp/b2w-user-calibration-B-fit-01
```

Compare separately before pooling with compatible repeats; the tool rejects
mismatched policy/control/geometry/fixture contracts and duplicate raw sessions.
A saved summary does not replace native events: the tool recomputes the existing
steady-state validity from `manifest.json` + `events.jsonl`. Outputs go only to a
fresh directory outside all input runs, including hashes of every input file.
It never starts ROS/Gazebo and never writes inside an input run directory.

## Harness change and validation boundaries

Existing runner only: `--speeds` accepts a distinct finite comma-list within
**[.04,.40] m/s**, the experiment envelope spanning the requested low-speed
protocol and unchanged defaults. This bound is not measured support. Invalid,
duplicate, nonfinite, zero, negative and outside-envelope values fail explicitly;
no requested point is clamped. Defaults remain [.10,.20,.30,.40] and 48 s.
All manifest/export/live completion times derive from 12×number of points.
High-precision custom points get distinct filenames; default filenames stay
unchanged. The existing 240 wall-second motion watchdog is retained even for
custom schedules; a list too long for that bound will abort, not extend it.

Validation: native synthetic fixtures cover order/sign conversion, per-wheel
tracking/error cancellation, zero-state/nonzero fits, monotonic inverse and
unsupported domains, independent repeats, missing/authentication failures,
raw-file preservation, CLI errors, custom timing/boundaries and actual recorder
Twist field semantics. Existing cleanup/publisher/readiness regressions run with
no ROS nodes. Static `ign sdf -p` converts URDF only. The two live-ROS tests in
the existing suite are excluded for this offline stage; no smoke is necessary.
The default plan and all eight derived files from the reviewed capture are
compared byte-for-byte to committed behavior into a NEW temporary directory.

## Reversal evidence and limitations

Reverse or weaken A if matched fresh sessions lose the mean target bias, if
policy targets are unstable with command held fixed, if provenance/frame/sign/
timing authentication fails, or if appropriate target means plus contact/leg
kinematics account for the body bias. Gain numbers alone cannot settle this.
The run has ascending order, no randomized order, no repeat confidence, no
low-speed transition evidence, no measured contact forces/slip/full-body rolling
kinematics, no original resolved training run and no contemporaneous ONNX-memory
hash. No result here establishes narrow-space navigation safety or a production
inverse. Fresh validation should test prediction errors and repeatability against
an agreed physical-error tolerance before selecting the simplest model. The
physical-error acceptance tolerance is not yet specified, so no numeric
production acceptance threshold is invented.


Independent review and final checks: **78 bounded tests passed; 2 live-ROS tests
excluded**. Initial broader collection without sourced message definitions failed
on missing `sensor_msgs`/`lifecycle_msgs`; the sourced environment rerun passes
without starting ROS nodes. The independent reviewer identified and rechecked
three fixes: exact captured Float32 constant plateaus now define fit/repeat input
keys (time-weighted means retained separately); full captured source/fixture
contracts are compared before pooling; truth and SLAM validity are independent.
Jittered repeat and incompatible-config fixtures reproduce the pre-fix failures.
Final read-only recheck reports no remaining critical/important issues.

The default dry-run JSON is byte-identical to committed behavior; 48,003 time
probes agree, and all eight original derived result files are byte-identical when
recomputed into a new directory. Recorder bytes, ownership/cleanup/executor,
readiness/publisher predicates and world fixture generator are unchanged.
Production source/config/models and every existing reviewed run file match the
initial hashes. Live read-only `git ls-remote` confirms origin at the same HEAD.
No staging, commit, push, live smoke, nonzero run or unrelated process signalling.
