# B2-W open-space velocity step response

This harness measures the normal downstream command path before anyone changes
Nav2 speed limits or actuator gains. The harness supports a zero-only live readiness smoke and a 48 simulation-second
characterization, with GUI or headless rendering. Formal/long nonzero tests
are user-run locally. Agents may implement/test the harness and perform bounded
zero-motion or minimal safe smoke checks; an exception requires explicit user
authorization in that turn.

The independent Sol review executed the full 48-s nonzero characterization
despite this separation rule, without the required explicit user exception.
That existing run is preserved as agent-run diagnostic evidence, not a user-run
formal experiment. Its data and conclusions remain unchanged; no repeat is
needed merely to correct provenance.

## What the experiment tests

The runner owns a fresh course session and publishes deterministic `Twist`
requests at **20 Hz**, derived from the unchanged
`course_navigation/config/nav2.yaml` `controller_frequency` and checked against
the unchanged smoother frequency. The tested chain is:

```text
deterministic requested Twist
→ /course_navigation/cmd_vel_raw
→ unchanged Nav2 velocity_smoother
→ /course_navigation/cmd_vel
→ unchanged velocity_adapter
→ /path_manager/path_manager_ros/nav_vel
→ rl_sar controller → policy → actuator torque → Gazebo physics
```

This is not a direct policy-input test: the existing smoother and adapter stay
in the path, and their received commands are recorded separately. It is not a
Nav2 planner/controller test: only the upstream planner-side request source is
replaced. The runner does not start planner_server, controller_server,
behavior_server, DWB, NavigateToPose, goal management, or recovery behavior.
The recorder owns the sole raw command publisher. The harness publishes no
shared or final command.

## Open fixture and SLAM observability

Each run writes a temporary world into its fresh output directory. It retains
the source confined world's physics step, world systems, floor collision,
floor dimensions, and visual material. The robot spawns at `(-4, 0, 1) m` with
heading `+x`. A reserved clear strip is recorded in `manifest.json`; no
landmark enters the strip, and the original internal obstacle/corridor boxes
are removed. Four outer boundary walls enclose the floor.

Six fixed, unequal boxes are placed outside the clear strip at these
center/size values in metres:

| Landmark | Center `(x,y,z)` | Size `(x,y,z)` |
|---|---|---|
| west north | `(-6.3, 3.25, 0.85)` | `(0.35, 0.55, 1.7)` |
| mid north | `(-1.8, 4.35, 0.65)` | `(0.85, 0.45, 1.3)` |
| east north | `(4.7, 3.15, 0.95)` | `(0.4, 0.7, 1.9)` |
| west south | `(-5.1, -4.2, 0.75)` | `(0.7, 0.35, 1.5)` |
| mid south | `(0.65, -4.75, 0.9)` | `(0.45, 0.8, 1.8)` |
| east south | `(6.2, -3.45, 0.6)` | `(0.95, 0.4, 1.2)` |

The landmarks provide asymmetric LiDAR geometry for SLAM observability only;
they are not control inputs and are never navigated around. Their presence
does not guarantee good localization. The manifest stores source/generated
world hashes, exact poses, dimensions, spawn and strip bounds. Ground truth is
evaluation-only and never steers, advances, selects, resets, corrects, or
safety-terminates the motion.

## Readiness, sequence, and safety

The runner starts the existing course in `mode:=slam control_source:=none
locomotion_backend:=rl_sar fault_profile:=normal`, plus only the existing
velocity smoother, its lifecycle manager, and the existing adapter. It uses
a unique ROS domain, Ignition partition and owner token. The owner records
PID/start-time identities and cleans only its child processes with bounded
INT → wait → TERM → wait → KILL handling.

Before motion it requires advancing simulation clock, stable clock epoch,
live rl_sar controller and adapter, completed 2 s stand/policy warm-up, fresh
finite private joint feedback and IMU, fresh finite SLAM odometry with
`base_link` child frame, fresh ground-truth instrumentation with known
`base_link` body-twist frame, valid `map→base_link` TF, active smoother, fresh
zero final command, expected command publisher ownership, and no competing
publisher. Ground truth is checked only for instrumentation/frame metadata.
Readiness times out after 120 monotonic wall seconds; it uses live evidence,
not a fixed startup sleep.

One explicit rclpy `SingleThreadedExecutor` services both runner and recorder
through readiness, motion and zero flush. `/clock` uses `rosgraph_msgs/Clock`;
clock/private sensors/dynamic TF use best-effort subscriptions compatible with
reliable and best-effort publishers. Static TF uses reliable transient local.
Readiness also requires real recorder callbacks on every required stream,
exact command/force publisher names and namespaces, one clock publisher, and
absence of navigation command nodes. Diagnostic dictionaries are outside the
explicit readiness gate keys. Zeros publish at 20 Hz while waiting for readiness.

`--readiness-smoke` launches this same stack, never starts the motion sequence,
and maintains every readiness gate for 3 wall seconds. Every command is zero;
real events, fixture hashes and normal cleanup evidence are saved. No per-speed
physical results are generated because no experiment start is defined.

The phases use simulation-clock time. Each level has 3 s at zero, 5 s at the
target, then 4 s at zero. The complete schedule is 48 simulation seconds:

| Level | Settle | Target hold | Stop observation |
|---:|---:|---:|---:|
| 0.10 m/s | 3 s | 5 s | 4 s |
| 0.20 m/s | 3 s | 5 s | 4 s |
| 0.30 m/s | 3 s | 5 s | 4 s |
| 0.40 m/s | 3 s | 5 s | 4 s |

`vy` and `wz` remain zero. There is no automatic repositioning, reverse,
yaw-correction, or truth-based motion correction. During the run, a frozen or
backward clock, lost controller/smoother/adapter, command conflict, stale or
invalid required SLAM/TF/private feedback, changed odometry body frame, or
raw-IMU tilt over 25° causes repeated zero and an aborted partial run. The
240 s wall watchdog bounds the motion phase. Before shutdown the sequence enters zero-only mode; zeros publish repeatedly
for at least 1.2 wall seconds while both nodes receive callbacks. The executor
is stopped and both nodes are removed/destroyed before rclpy shutdown. The user can press
Ctrl+C in the runner terminal at any point; partial artifacts are retained.
Visually inspect the open strip and stop if anything appears unsafe.

## Captures and analysis

`events.jsonl` stores native receipts, including topic/type, source header
seconds/nanoseconds when present, source stamp, simulation receive time and
epoch, monotonic wall receipt, frame/child frame, source order, finite payload
and validity. Unstamped `Twist` samples use receive-time proxies; receipt
differences are not transport latency or actuator application timestamps.
The capture includes requested/raw/shared/final commands, `/clock`, truth and
SLAM odometry, raw IMU, private joint q/dq and wheel dq by joint name,
`/network_input_debug`, joint commands/torques, and TF. Private joint samples
are decimated to at most 100 Hz and counts are recorded. Point clouds,
costmaps, DWB evaluation and video are not recorded.

The runner writes `manifest.json`, `events.jsonl`, `timeseries.csv`, one JSON
file per requested speed under `per-speed/`, and `summary.json`, `summary.csv`,
and `summary.md`. The presentation time series is a 20 Hz grid; it does not
create synthetic observations for the metric calculations. Offline
`--analyze RUN_DIR` reads saved native events, regenerates derived files, and
checks that the raw capture hash is unchanged.

For truth and SLAM separately, the analysis reports:

- final fixed 2 s target-window, time-weighted vx mean, coverage, slope-based
  validity, gain/bias, SLAM comparison, and truth/final-command gain;
- requested-relative 10–90% rise, target peak, requested and tail-relative
  overshoot, settling or censoring;
- requested-zero, final-nav-zero and truth effective-stop times; both
  requested/final zero stopping durations; stop path and net displacement;
  censored 4 s path/net displacement if no effective stop occurs;
- target and stopping phase lateral displacement, yaw drift, peak `|vy|` and
  peak `|wz|`;
- requested/raw/shared/final receive timing, plateaus, receive rates and
  coverage, plus freshness-guarded command integrals.

Truth and SLAM never substitute for one another. Truth metrics use body twist
only after the recorder verifies the declared child frame; path metrics use
supported pose segments with interpolated boundaries and shortest-angle yaw.
Unstamped commands are integrated as receive-causal zero-order holds. Samples
are not interpolated across configured source gaps. Missing data, unsupported
windows, invalid values, rollback, and censored metrics remain null with
quality reasons rather than becoming zero.

## Interpretation boundaries

These patterns are candidates to investigate together; none identifies a
unique cause from one trial:

- **A:** truth steady vx consistently exceeds requested vx → command scaling
  or policy steady-state bias candidate.
- **B:** steady vx is accurate but peak/settling transient is large →
  actuator, policy, or transient-dynamics candidate.
- **C:** vx is accurate but vy/wz/yaw drift is substantial → locomotion
  coupling or asymmetry candidate.
- **D:** final `nav_vel` substantially lags requested/raw → upper command
  pipeline or smoother candidate.
- **E:** final command tracks while truth lags, overshoots, or drifts →
  downstream policy/actuator/plant candidate.

Do not choose a gain change or Nav2 change from one number. Compare command
ordering, coverage/gaps, torque saturation, wheel symmetry, pose/twist
agreement, and independent SLAM/truth trends. The test does not prove an
unresolved historical training configuration, actuator equivalence,
repeatability, or narrow-space safety. No gain is tuned by this harness.

## Run and replay commands

From a built, prepared course workspace with ROS 2 Humble and the pinned model
available, source `/opt/ros/humble/setup.zsh` and `install/setup.zsh`, then run
the bounded characterization command:

```zsh
/usr/bin/python3 scripts/run_velocity_steps.py --ros-domain-id 194 --gui --output-root /tmp/b2w-velocity-steps
```

Before motion, run the same architecture in zero-only mode and inspect
`manifest.json` readiness, recorder counters and cleanup:

```zsh
/usr/bin/python3 scripts/run_velocity_steps.py --readiness-smoke --ros-domain-id 194 --no-gui --output-root /tmp/b2w-velocity-steps-smoke
```

`--no-gui` retains GPU LiDAR rendering through the existing launch contract;
headless server rendering must work on the host. Offline analysis shifts the
relative sequence by the recorded `experiment_start_sim_s`; native event
stamps and raw capture bytes stay unchanged. Pre-clock startup receipts may
have null simulation stamps; requested events start only after recorder clock
receipt, and every motion-window command must have a simulation timestamp.

The non-motion dry-run is:

```zsh
/usr/bin/python3 scripts/run_velocity_steps.py --dry-run
```

Re-analyze an existing run without ROS or child processes:

```zsh
/usr/bin/python3 scripts/run_velocity_steps.py --analyze /absolute/path/to/run-directory
```

## Current offline calibration stage (2026-10-16)

The default invocation still requests `[0.10,0.20,0.30,0.40]` over exactly 48
simulation seconds. The existing runner now also accepts `--speeds 0.20` or
`--speeds 0.04,0.07,0.10,0.13,0.20`; every point retains 3 s zero → 5 s target
→ 4 s zero. Inputs must be distinct finite values in [.04,.40] m/s, with
explicit errors and no clamp. This experiment envelope does not assert measured
support or authorize motion. All command/readiness/cleanup paths stay unchanged.

[The calibration audit and USER-RUN protocols](VELOCITY_CALIBRATION_AUDIT.md)
authenticate geometry and wheel signs, split policy targets from actual wheels
and body velocity, and give exact commands labelled **DO NOT RUN UNTIL CHATGPT
REVIEW PASSES**. Protocol A repeats come before Protocol B review/authorization.
Neither was executed in this stage. Pure `scripts/fit_velocity_calibration.py`
reads completed native run directories and writes JSON/CSV/Markdown into a fresh
external output directory; it never starts ROS/Gazebo or changes input results.
Production inverse calibration remains deferred.
