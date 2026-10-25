# Smac2D + RPP + geometry adaptive speed: one practical warehouse route

The user ran an adaptive attempt on Oct 18; it never entered OPEN and stopped
near G1. See [the retained-run diagnosis](ADAPTIVE_RUN_2026-10-18.md) and
[the state-machine repair](ADAPTIVE_STATE_REPAIR_2026-10-18.md) before
repeating it. The repair awaits independent review; only offline verification
is authorized in this milestone. The intended user run is
ONE Start→G1→G2→G3→G4→G5 attempt with `--advancement practical`, no retry.
Physical .50 m/s and successful G1→G2 traversal remain unmeasured.

## Operating points and reference width

| Mode | Physical target | Initial direct ROS/policy command ceiling | Evidence |
|---|---|---|---|
| OPEN | maximum target .50 m/s | .43 | Recorded .40 gave truth .459378 m/s; .43 is a provisional 7.5% command increase, not an achieved physical speed. |
| NARROW | .18 m/s, approximately .15–.20 | .11 | Held-out .085/.115 commands gave .152952/.184238 m/s; .11 is between these observed operating points. |

Existing records: [four-speed audit](VELOCITY_CALIBRATION_AUDIT.md), table of
steady two-second windows; [historical B/C values](../PLAN.md), including the
recorded source paths and artifact hash. Original B/C raw data are not retained
in this machine's `/tmp`; these are existing documented measurements, not a new
calibration. The points describe pure forward motion. Turning/acceleration may
differ. M2 stays disabled and unchanged; no low-speed fit is extrapolated to OPEN.

**`w_ref` is required** and is the approximate measured obstacle-to-obstacle free-space width
in metres, including the space occupied by the robot, without subtracting the
footprint. It is not the sum of the two residual side clearances. The user has
supplied **W_ref=1.0 m** as an approximate measurement; keep this explicit input.
The historical run also loaded 1.0 m. Independent metric authentication and its
uncertainty are unavailable; this patch does not request another calibration.
The WORLD canonical floorplan is an all-height collision projection, includes
overhead objects and has no authenticated G1→G2 registration, so it cannot
replace the supplied measurement.

If refining the measurement later, without driving the robot:

1. Obtain a saved OccupancyGrid/costmap from the prior G1→G2 attempt, including
   resolution, origin, frame and the same run's goal/path evidence. Confirm both
   sides of the corridor are observed. If using authored scene geometry, identify
   the actual two obstacle objects and traversed height; do not infer registration
   by equating WORLD and SLAM coordinates.
2. In a viewer with metric coordinates (or a pixel ruler using the saved map's
   metres-per-cell), measure across the corridor **perpendicular** to its local
   direction at entry, middle, exit and each visible pinch point. Use lethal
   obstacle cell edges, excluding inflated costs; unknown cells are not walls
   with known locations and invalidate that cross-section.
3. Retain the smallest observed free width, the two endpoints of each section,
   source file/scene-object identities and resolution. For an image ruler,
   width = pixel distance × resolution; report grid uncertainty of roughly two
   cells. This is obstacle-to-obstacle width; do not subtract the robot footprint.
4. Record the measurement provenance and uncertainty before revising `W_REF`.
   The current repair retains the user's approximate 1.0 m value.

## Behavior and interfaces

`adaptive_speed:=true` derives SmacPlanner2D + RPP from unchanged `nav2.yaml`.
Its expanded planner is `nav2_smac_planner/SmacPlanner2D`; NavFn's `use_astar`
switch is removed only from this opt-in profile. Planner tolerance .3 and
allow_unknown=false are preserved. The default remains NavFn + DWB.
Custom diagnostic Nav2 profiles are rejected with the
adaptive option, preventing accidental DWB SpeedLimit angular coupling.
The RPP plugin, message fields and parameter names were checked against installed
Humble 1.1.20 headers, plugin XML and binary strings. Source semantics:
[RPP](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp),
[controller subscription](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp),
[DWB angular scaling](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_plugins/src/kinematic_parameters.cpp).

- Governor inputs: `/local_costmap/costmap_raw` (Nav2 Costmap),
  `/received_global_plan` (RPP's pruned path in base_link, published every control
  tick), `/slam/odom`, timestamped TF, plus `/pointcloud` header/size for obstacle
  source freshness. Each path is transformed into the costmap
  frame at its message timestamp. Ground truth is used only by the recorder.
- The existing signed padded footprint is read from the original costmap YAML.
  Samples cover the current footprint and upcoming path at ≤.10 m intervals.
  Cell areas, unknown cells and the map boundary constrain clearance; inflation
  costs are not interpreted as physical walls. Conservative longitudinal strips
  give minimum left/right clearance from the footprint edges. Width is their
  sum plus footprint width; truncated free regions give a lower bound.
- OPEN is normal travel after fresh valid inputs and geometric safety have
  been confirmed. NARROW enters immediately for width ≤ `w_ref + .10` or either
  footprint-relative side ≤.15 m. Footprint-envelope overlap also forces NARROW;
  RPP remains responsible for collision rejection. Heading error and curvature
  are diagnostics only; ordinary turning does not latch NARROW. The obsolete
  governor `curvature_threshold` / `heading_threshold` settings are removed.
- Lookahead is at least 1 m, increased to
  `max(0, v_measured² − .18²)/(2*.20) + v_measured*.80 + .20` metres.
  The .80 s latency allowance includes costmap publication, governor tick,
  smoothing and actuation. Deceleration .20 m/s² is an initial conservative
  assumption; it is not a measured guarantee or the smoother's command slope.
  Once geometry triggers NARROW, its detection horizon is retained through
  braking/data interruptions until confirmed OPEN release. Slowing cannot hide
  a still-upcoming choke by shrinking the checking distance. The current path
  and footprint are rechecked each tick; a clear replanned path can release.
  A path shorter than the checking horizon cannot confirm safety and selects
  NARROW (`invalid_geometry`), including near a goal. RPP's goal acceptance and
  approach behavior remain configured as before.
- OPEN release requires .5 s sustained width > `w_ref + .10 + .03` and both side
  clearances > `.15 + .03`. The actual entire padded footprint, including its
  rear, and upcoming path samples must meet these thresholds. The intervening
  band retains the current mode. Diagnostics distinguish `release_margin`
  (timer blocked) from `release_hold` (eligible and timing); `tight`,
  `heading_error`, `curvature`, `release_eligible` and `clear_duration_s` remain
  available. Missing/stale/invalid data and clock rewinds reset
  release and select NARROW. Freshness checks source and steady receipt time;
  cached TF is checked by source timestamp. Humble's raw-costmap topic leaves
  metadata.update_time unset (zero), so header and receipt times are used there;
  a supplied nonzero update_time is also checked. These timestamps cannot detect
  a publisher that continuously re-stamps old underlying sensor data. The separate
  scan heartbeat prevents OPEN when the obstacle observation source stops,
  even if costmap or map publishers continue refreshing their headers.
  It establishes scan receipt, not proof that a particular scan was incorporated
  into the costmap; Nav2's observation/collision checks remain enabled.
  See the installed version's
  [publisher implementation](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_publisher.cpp).
- Output `/course_navigation/speed_limit` is an absolute **command** limit with
  `percentage=false`, always positive. In Humble zero restores default speed;
  it is never sent as a stop. `/course_navigation/speed_status` records mode,
  reason, width, side clearance, braking horizon and measured SLAM speed.
- RPP keeps curvature regulation, cost regulation, heading rotation and footprint
  collision checking enabled. Its yaw setting and downstream yaw ceiling remain
  .6 rad/s in both modes. RPP `setSpeedLimit` affects linear desired speed.
  Ordinary RPP curvature behavior can still change the commanded yaw.
- RPP/smoother/adapter share the .43 forward ceiling. An opt-in adapter watchdog
  caps planar commands after smoothing, including recovery translations, while
  scaling yaw by the same factor when moving translation is clipped, so the
  command's curvature is preserved through OPEN→NARROW transitions. Intentional
  in-place rotation (planar speed ≤1e-6 m/s) retains yaw. Unsaturated commands
  pass through and existing angular safety bounds remain in force.
  Missing/invalid/zero/percentage/stale (> .5 s) limit heartbeat
  produces a real zero Twist. Default DWB has no governor/watchdog subscription.
  A governor scheduling gap over .5 s also resets its clearance hold on recovery.
- A downward-only SLAM speed feedback guard reduces the command ceiling above
  each physical target. It never raises a command or applies M2. It cannot
  certify a strict physical maximum or eliminate transient overshoot; ground truth
  in this user run measures actual behavior independently.

Tunable values are in `course_navigation/config/adaptive_speed.yaml`. Supply
`adaptive_params_file:=/absolute/path.yaml` to use a copy. The initial OPEN
ceiling may not exceed .44 in this implementation. Launch passes the chosen
ceiling consistently to RPP, smoother and adapter. ROS parameters are read at
startup; restart to apply settings. No footprint, collision-layer,
SLAM, policy, M2, canonical waypoint or arrival tolerance changes are made.

## Exact user-run commands (zsh)

**Run these commands only after independent bundle review accepts this milestone.**
Close an existing course session normally with Ctrl+C before T1. Use the same
repository and domain in both terminals. The two changed Python packages were
already built with `scripts/build_course.sh --packages-select course_navigation
course_bringup --symlink-install`; rerun that build if using a fresh checkout.

**T1 — after review only: fresh warehouse, approximate user W_ref:**

```zsh
cd /home/zyd/program/b2w-embodied-navigation-course
W_REF=1.0  # User's approximate obstacle-to-obstacle measurement, metres.
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/b2w-adaptive-state-route/ros-log
mkdir -p /tmp/b2w-adaptive-state-route/metrics
scripts/run_warehouse.sh adaptive_speed:=true w_ref:="$W_REF" \
  results_directory:=/tmp/b2w-adaptive-state-route/metrics timeout_s:=240.0
```

Do not send RViz/manual goals during T2. The runner waits for an idle, localized,
active stack and sends each canonical waypoint once. No separate calibration or
motion experiment is requested.

**T2 — run the one complete route and automatically record speed:**

```zsh
cd /home/zyd/program/b2w-embodied-navigation-course
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/course_sim/config/fastdds_local.xml"
export PYTHONPATH="$PWD/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$PWD/.deps/lib:$PWD/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR=/tmp/b2w-adaptive-state-route/ros-log
/usr/bin/python3 scripts/run_warehouse_route.py --mode full --advancement practical \
  --execute --ready-timeout 120 --output-root /tmp/b2w-adaptive-state-route/route
```

The timestamped route directory contains `report.json`, maps/trajectories and
overlays, plus:

- `speed-trace.jsonl`: source-stamped truth/SLAM planar speeds, received
  raw/smoothed/final commands, limits, mode/geometry decisions and goal events.
- `speed-summary.json`: separate truth/SLAM counts, median, p95 and max for
  OPEN/NARROW/UNKNOWN. Mode attribution requires causal status ≤.5 s old.
  These distributions include acceleration, turns and stops. Inspect the trace
  for steady segments; a maximum alone does not prove sustained .50 m/s travel.
- `speed-runtime-parameters.json`: read-only snapshots of loaded Smac2D, RPP, smoother,
  adapter and governor settings, with unavailable responses explicitly marked.

Missing evidence stays unavailable. Evaluate whether NARROW physical speeds
are genuinely lower and around .15–.20 in the corridor, and report the measured
OPEN speed. Preserve the first failed leg, trace and T1 ROS logs before changing
anything. Diagnose only failures actually observed in this attempt.

**Rollback:** stop T1 normally, restart with `scripts/run_warehouse.sh
adaptive_speed:=false` (or omit the option). This restores unchanged NavFn + DWB
profile and .40 smoother ceiling and disables the adapter governor watchdog.
Do not run a second test as part of this delivery.

## Offline checks

The current Oct 18 state-machine repair and its fresh evidence are documented
in [ADAPTIVE_STATE_REPAIR_2026-10-18.md](ADAPTIVE_STATE_REPAIR_2026-10-18.md).
The following counts describe earlier milestones.

Focused tests cover synthetic raw-costmap geometry, rotated grids and TF, cell
areas, unknown/off-map space, lookahead, physical braking distance, curvature,
hysteresis, stale map/odom/path/localization, metadata update time, clock reset,
nonfinite inputs, positive SpeedLimit semantics, adapter heartbeat/yaw handling,
launch expansion, rollback and speed evidence. Existing practical route, route
ownership/cleanup, M2, footprint and adapter regression tests are included.
No test initializes a ROS node or launches a simulator. Build success and offline
checks do not establish live readiness or physical traversal.

The preceding RPP-only iteration passed 194 checks. This Smac2D iteration adds
actual launch-expanded planner/limit-chain assertions and command recorder
integration coverage. Fresh logs, expanded adaptive/default parameter snapshots,
geometry assessment and owned-process cleanup evidence are stored in
`/tmp/b2w-smac-adaptive-review/` and attached to the review bundle. The new bundle
requires independent review before a live user run.

Previous Smac2D bundle results: **196 tests passed (2.53s)**, both packages built (1.62s),
practical dry-run and `git diff --check` passed. Five owned validation process
groups were checked after exit with zero remaining members. Launch-expanded
snapshots confirm Smac2D+RPP only when opted in, and NavFn+DWB on rollback.

Curvature review fix: **206 focused tests passed (3.03s)**, both packages built,
dry-run, launch/rollback expansion and diff checks passed. Clipped moving twists
now scale vx/vy/yaw uniformly; deliberate in-place yaw and stale-data stops are
preserved. All five owned validation process groups exited with no remaining
members. Fresh evidence is in `/tmp/b2w-curvature-review/`; this replacement
bundle requires independent review before any live user run.

Changed files for this delivery (the inherited practical test is untouched):

- `course_navigation/course_navigation/adaptive_speed.py`
- `course_navigation/course_navigation/speed_governor.py`
- `course_navigation/course_navigation/adaptive_profile.py`
- `course_navigation/config/adaptive_speed.yaml`
- `course_navigation/course_navigation/velocity_adapter.py`
- `course_navigation/launch/navigation.launch.py`
- `course_navigation/setup.py`, `course_navigation/package.xml`
- `course_bringup/launch/course.launch.py`
- `scripts/route_speed_recording.py`, `scripts/run_warehouse_route.py`
- `tests/test_adaptive_speed.py`, `tests/test_adaptive_node.py`
- `tests/test_adaptive_launch.py`, `tests/test_route_speed_recording.py`
- `docs/ADAPTIVE_SPEED.md`, `docs/INTERFACE_ALIGNMENT.md`
- `CODEX_GUIDE.md`, `PLAN.md`
