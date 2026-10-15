# B2-W deployed control-chain audit

## Artifact and provenance

The selected deployment artifact is
`third_party/b2w_rl_sar/policy/b2w/robot_lab/policy.onnx`, selected by
`course_sim/launch/simulation.launch.py` (`policy = pretrained / 'policy/b2w/robot_lab/policy.onnx'`)
and passed as `policy_path` to `b2w_rl_sar_controller`. Its SHA256 is
`da31c0c1820b893608e2d02746bbada16d0abb5de6116277e37ccfb02024c564`.
`third_party/b2w_rl_sar/POLICY_SOURCE.json` records the source project
`fan-ziqi/rl_sar`, pinned revision `376d42c9b128f963ab08579762d5a216a976ce39`,
robot `b2w`, policy `robot_lab`, and the PT source SHA256
`38155076408e8eccb22690c6c5be14bd1dcb9149245ca5e493308a9f6ff93b34`.
The ONNX was derived from that PT with PyTorch 2.5.1+cpu / ONNX opset 17;
the recorded parity maximum absolute error is `1.239776611328125e-05`.

The accompanying pinned `robot_lab/config.yaml` SHA256 is
`d44bb3c36043f7216f7699a23d904fd01b5fc29ab7c35aabe56dc024d3f1cc74`.
That file documents an rl_sar deployment contract. It does **not** identify
the resolved Isaac Lab run that produced these weights. The inspected
robot_lab B2-W actuator/task reference is commit
`500399ed75f510aeaff28705a8ce736c514dbec3` (Flat and Rough task
registrations are present); it is a source reference, not a checkpoint
binding. The corresponding historical training task, resolved run config,
actuator randomization draw, simulator version, and training asset remain
**provenance unresolved**. Neither current upstream main, the rl_sar revision,
nor policy filenames establish those historical facts.

The inspected reference is
[`UNITREE_B2W_CFG` at the pinned robot_lab commit](https://github.com/fan-ziqi/robot_lab/blob/500399ed75f510aeaff28705a8ce736c514dbec3/source/robot_lab/robot_lab/assets/unitree.py)
and its B2-W Flat/Rough registrations are in the
[`unitree_b2w` task package](https://github.com/fan-ziqi/robot_lab/tree/500399ed75f510aeaff28705a8ce736c514dbec3/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/unitree_b2w).
The inspected parent velocity environment defines a reset event that scales
stiffness and damping uniformly by 0.5–2.0. Whether the actual checkpoint's
resolved task enabled or overrode that event, and what values were drawn, is
unknown. Therefore 160/5 is the nominal inspected reference, not a proven
historical training value.

## Runtime chain

```text
Nav2 FollowPath request (Twist, m/s and rad/s)
  → /course_navigation/cmd_vel_raw
  → nav2_velocity_smoother (unchanged, OPEN_LOOP, 20 Hz)
  → /course_navigation/cmd_vel
  → course_navigation/velocity_adapter (50 ms steady timer)
  → /path_manager/path_manager_ros/nav_vel
  → b2w_rl_sar_controller command callback
  → 57-element observation → ONNX → 16 actions
  → 12 leg position targets + 4 wheel angular-velocity targets
  → explicit C++ torque law → per-joint ApplyJointForce bridge
  → Fortress physics → base motion
```

Evidence and runtime interpretation:

| Stage | Source and key snippet | Units / scale / cadence | Origin and implication |
|---|---|---|---|
| Request and smoother | `course_navigation/launch/navigation.launch.py`: ControllerServer `cmd_vel` remaps to `/course_navigation/cmd_vel_raw`; smoother input/output remap raw → `/course_navigation/cmd_vel`. `course_navigation/config/nav2.yaml`: `controller_frequency: 20.0`; smoother `smoothing_frequency: 20.0`, `feedback: OPEN_LOOP`. | Twist: m/s, m/s, rad/s. Smoother max `[0.4,0,0.6]`, min `[-0.2,0,-0.6]`; x accel/decel `+0.5/-0.5 m/s²`. | The step runner publishes 20 Hz to the raw interface, matching unchanged controller cadence. Smoother behavior is deployment-side shaping, not policy behavior. |
| Adapter | `course_navigation/course_navigation/velocity_adapter.py`: `scale=min(1,max_linear/hypot(x,y))`; yaw clipped to `max_angular`; steady timer `.05`; lookup `map` → `base_link`; publish final topic. | Default linear norm limit 0.5 m/s, yaw ±0.6 rad/s, timeout 0.5 s; 20 Hz wall timer. No independent vx/vy scaling below the norm limit. | Deployment-specific freshness/localization gate. It outputs zero if command, SLAM odometry, or TF is stale/unavailable. |
| Controller command and observation | `b2w_sim/b2w_controllers/src/rl_sar_controller.cpp`: callback clamps `[vx,vy,wz]` to `[-1,1],[-.5,.5],[-1,1]`; observation `obs[6+i]=command_[i]`; `obs[8]=clamp(obs[8]*yaw_gain,-1,1)`. | m/s, m/s, rad/s; vx/vy scale 1, yaw scale `yaw_command_gain=1.6`; commands timeout after 0.5 s. Policy runs every fourth 5 ms timer tick = 50 Hz; torque update nominal 200 Hz. | Command vector is body-relative policy input. Yaw gain is deployment-specific and is applied only to wz. Observation values are clipped to ±100 before inference. |
| ONNX and actions | Same C++ file: one input width 57, one output width 16; actions clipped to ±100. `target_[i]=ref_[i]+scale_[i]*last_[i]` for leg indices; wheel `velocity_target_[i]=scale_[i]*last_[i]`. | 12 leg position offsets in rad with scales `[.125,.25,.25]` per hip/thigh/calf; four wheel target rates in rad/s with scale 5. | Action layout/order follows `rl_sar.yaml`. Action clipping and scales match the pinned deployment config; historical training equivalence is unresolved. |
| Joint feedback and torque | C++ subscribes `/sim/raw/joint_states` and `/sim/raw/imu`; finds each configured joint by name; requires finite q/dq and freshness ≤0.15 s. Torque law: `kp*(target-q)+kd*(velocity_target-dq)`, clamped by joint limit. | q rad, dq rad/s, torque Nm. Policy target torque every 5 ms; action/observation update every 20 ms. | This is the sole active joint feedback law in the rl_sar path. Standing uses `fixed_kp/fixed_kd` during first 2 s; normal policy tracking then uses `rl_kp/rl_kd`. |
| Gazebo | `course_sim/course_sim/description.py`: torque mode removes `JointPositionController`/`JointController` plugins and adds `ignition-gazebo-apply-joint-force-system` for each movable joint. `course_sim/config/bridge_rl_sar.yaml` maps each `/model/b2w/joint/<name>/cmd_force` to Fortress. | Force command Nm, physics step 0.0025 s = nominal 400 Hz (`course_sim/worlds/confined.world`). URDF effort/velocity attributes are declarations; effective enforcement of velocity limits by Fortress was not established. | ApplyJointForce is the plant input, not a second PD layer. Passive dynamics and engine enforcement are not equivalent evidence for a training actuator model. |

## Training-reference versus deployment values

“Inspected reference” below means the pinned files distributed with rl_sar or
the explicitly inspected robot_lab commit above. It does not convert an
unknown historical training run into a known configuration.

| Quantity | Inspected training/deployment reference | Course deployment | Match / risk | Evidence |
|---|---|---|---|---|
| Hip Kp/Kd | 160 / 5 nominal | 160 / 5 | Numeric reference match; run unresolved | `robot_lab/config.yaml` `rl_kp/rl_kd`; `course_sim/config/rl_sar.yaml`; C++ torque law |
| Thigh Kp/Kd | 160 / 5 nominal | 160 / 5 | Numeric reference match; run unresolved | Same arrays, C++ law |
| Calf Kp/Kd | 160 / 5 nominal | 160 / 5 | Numeric reference match; run unresolved | Same arrays, C++ law |
| Wheel stiffness/damping | 0 / 1 nominal | Kp 0 / Kd 1 in velocity-error torque | Numeric reference match; semantics differ from inspected implicit actuator | Pinned `robot_lab/config.yaml`; deployed `rl_kp/rl_kd`, C++ wheel target law |
| Startup gains | fixed legs 200/200/300, Kd 3; wheels 0/1 | same for 2 s stand transition | Deployment contract match; not a policy-motion interval | Pinned `base.yaml`, deployment YAML, C++ `standing` branch |
| Effort limits | hip/thigh 200, calf 320, wheels 20 Nm | same torque clamps; URDF declares the same limits | Numeric declaration match; simulator enforcement unresolved | Config `torque_limits`, C++ `std::clamp`, generated URDF copied by `description.py` |
| Velocity limits | 23, 23, 14, 50 rad/s by joint type | same URDF declarations; C++ does not clamp dq target | Declaration match; effective engine enforcement unresolved | PT asset URDF `<limit velocity=...>`; generated description; C++ action target |
| Action scale | hip .125, thigh/calf .25 rad; wheels 5 rad/s | identical | Pinned config match; actual training task unresolved | `robot_lab/config.yaml`, `rl_sar.yaml`, C++ target conversion |
| Observation/action clip | observations ±100; actions ±100 | both clipped ±100 | Value match; action-manager ordering/semantics not proven for historical run | Config `clip_obs`, `clip_actions_*`; C++ clamps |
| Policy frequency | dt .005 × decimation 4 = 50 Hz | 50 Hz | Reference match; actual run unresolved | Pinned `base.yaml`; C++ 5 ms timer and `tick_%4` |
| Low-level torque rate | 200 Hz implied by dt .005 | 200 Hz nominal | Reference match | `base.yaml`; C++ 5 ms timer |
| Physics rate | inspected candidate `base.yaml dt=.005` = 200 Hz; actual run unresolved | Fortress 400 Hz nominal | Candidate source differs; historical match unresolved | `base.yaml`; `confined.world` max step .0025; no historical run artifact |
| vx/vy/wz scale | `[1,1,1]` in policy config | `[1,1,1.6]` before observation clamp | vx/vy match; yaw deployment extension | Pinned `commands_scale`; controller C++ and `yaw_command_gain: 1.6` |
| Yaw command gain | No historical run-specific value recovered | 1.6 | Training equivalence unresolved | `course_sim/config/rl_sar.yaml`; C++ applies it to `obs[8]` |
| Joint order | FR, FL, RR, RL hip-thigh-calf, then FR/FL/RR/RL wheels | same 16 names | Reference match | Pinned `base.yaml`, deployment `joint_names`; C++ name lookup |
| Default pose | each leg `[0,.8,-1.5]` rad; wheels 0 | identical | Reference match | `base.yaml`, policy config, deployment YAML; C++ reference offset |
| Passive damping/friction | inspected reference actuator friction 0; historical asset unresolved | generated movable joints omit explicit `<dynamics>`; Fortress defaults unverified | Equivalence not established | `course_sim/course_sim/description.py` copies training URDF `<dynamics>` when present; generated deployment asset inspection |
| Contact/slip/material | training material/randomization unresolved | normal profile slip ratio 0; course floor box 20×16×0.2 m, diffuse `[.6,.65,.7,1]`; original course collision geometry retained | Equivalence unresolved | `fault_profiles.yaml`, `confined.world`, `description.py` retains collision and copies inertias |
| Actuator model semantics | inspected robot_lab legs use `DCMotorCfg`; wheels use `ImplicitActuatorCfg`; historical run unresolved | explicit fixed symmetric torque clamp in C++, wheel velocity-error torque, then force plugin | Not equivalent by evidence; same nominal numbers do not establish same motor/plant model | inspected robot_lab B2W actuator source; C++ law; description generator |
| Idle hold | historical policy override unresolved | `idle_hold: true`; below 0.01 on all command axes, wheel actions forced to zero while learned leg outputs remain | Deployment-specific behavior | YAML and C++ `if (idle && i>=12) last_[i]=0` |

## Control-law conclusions

- **Leg outputs are position targets:** policy action is scaled and added to
  the configured default joint pose; the C++ torque law tracks that position.
- **Wheel outputs are angular-velocity targets:** scaled action is compared
  with measured wheel dq in the damping term, yielding a torque request. They
  are not direct wheel torque actions.
- **Kp/Kd are applied in the custom C++ controller**, in the explicit torque
  equation, followed by per-joint torque saturation. Startup uses a second
  configured gain set for the stand transition; after it, the policy tracking
  gains are active.
- **There is no active integral term** in the examined runtime path. The law
  contains proportional position error and derivative/velocity error only.
- **There are not two active PD controllers on each joint** in the rl_sar
  model generated by `description.py`: legacy position controller plugins are
  removed; Gazebo receives force commands. Physics/contact/passive dynamics
  remain downstream.
- **“PID tuning” is not an accurate abstraction.** This runtime is RL body
  command tracking, leg position PD, and wheel velocity-error torque. No proven
  numerical hip/thigh/calf gain mismatch exists against the inspected
  reference. Historical training dynamics are unresolved, actuator-model
  equivalence is not proven, and no gain change is justified before the
  proposed step-response characterization.

## Verdict

**Provenance is insufficient; characterize first and avoid gain changes.**
Numerical deployment gains match the inspected pinned reference, while the
actual checkpoint's resolved training actuator configuration and simulator
semantics are not authenticated. The current issue could still be command
shaping, learned steady-state bias, transient policy/plant response, coupling,
or a combination; the open-space harness measures these layers without
claiming a single metric identifies a unique cause.
