# Stage 4 candidate route execution and evidence

This is an offline implementation and review candidate. **The selected route is
not cleared for motion.** Real installed Smac search finds center paths, but the
saved-map padded-footprint checks reject parts of G1→G2 and G3→G4. There was no
Gazebo, live ROS session, motion, commit or production-route approval in this turn.
The sole current roadmap is [PLAN.md](../PLAN.md).

## Selection and architecture

The immutable selected source is
`/home/zyd/b2w-course-evidence/waypoints/20261019-v1/candidate-16nlistv/candidate.yaml`.
`stage4_selection_policy.json` pins its SHA256, exact complete goals object hash,
map YAML/PGM, original NavFn config, canonical route and selected Smac config.
No coordinates or individual yaw values are retyped into navigation code.
`run_candidate_route.py select` verifies the complete V1 export manifest and all
four output hashes using the reviewed editor loader, makes a separate lossless
V2 export and creates `selection.json`. Direct V2 input must have the selected
V1's verified migration history and exactly matching goals. Missing manifests,
map/config drift, changed order/yaw/quaternion/tolerance or altered selection fail.
Editor artifacts retain `deployable: false`; the distinct selection declares
only offline preparation authority. A future explicitly authorized user chooses
`--execute-user-run`; selection alone starts nothing and approves no production
route. SHA256 authenticates content identity/integrity against reviewed pins;
these artifacts are not digitally signed and do not authenticate a human signer.

Runtime modules live in `scripts/stage4_route/`; no new ROS topic or TF is
published. The launch argument `goal_manager_enabled` defaults to `true` for
backward compatibility. Stage 4 requires `false`, `adaptive_speed:=false` and
Smac2D+DWB. The canonical runner and 0.05 m benchmark remain untouched by Stage 4.

Installed Humble 1.1.20 alternatives were evaluated:

| API | Decision |
| --- | --- |
| NavigateThroughPoses | Available; default installed BT uses RemovePassedGoals radius 0.7 m, periodic replanning and recoveries. Its action success is no physical waypoint proof. A separate preflight plus BT recomputation could execute a different path. Not used. |
| ComputePathThroughPoses | Selected for one complete concatenated global path, preserving all exact candidate poses as planning inputs. Planning success alone accepts no physical region. |
| FollowPath | Selected to execute the **identical Path object** checked and journaled by preflight. One controller action; no replan, retry, clear, backup, spin or intermediate goal removal. Its genuine SUCCEEDED result, guarded endpoint transform and fresh final pose/stop are mandatory. |
| Sequential NavigateToPose | Existing canonical baseline retained; separate endpoint goal checks can cause intermediate deceleration/turns. Not used by candidate executor. |

Smac2D uses radius/inscribed grid collision checking and a holonomic smoother;
it does not enforce the rectangular footprint's continuous kinematics or every
intermediate yaw. Exact yaws remain planning inputs and journaled provenance;
only final yaw is an arrival condition. DWB follows the concatenated path and
need not turn in place at G1–G4. It may still slow/stop for safety or fail. With
no replanning/recovery this version deliberately fails when the fixed global path
cannot be followed safely. A success claim cannot depend on automatic retry.

## Exact preserved working profile

Selected file: `course_navigation/config/nav2_smac2d_diag.yaml`, differing from
`nav2.yaml` only by planner plugin. NavFn+DWB remains the default rollback.

| Component | Effective source contract |
| --- | --- |
| Planner | `nav2_smac_planner/SmacPlanner2D`, GridBased, tolerance 0.3 m, allow_unknown=false; installed defaults downsample=false, cost_travel_multiplier=1, max_iterations=1000000, max_on_approach_iterations=1000, max_planning_time=2 s, use_final_approach_orientation=false. `use_astar` is a retained unused NavFn key. |
| Controller | `dwb_core::DWBLocalPlanner`, 20 Hz; vx 0…0.4 m/s, vy=0, wz ±0.6 rad/s; x accel/decel ±0.5, theta ±0.6; 20×5×20 samples, sim_time=1.7, linear/angular granularity .05/.025. |
| Critics | Original ordered RotateToGoal, Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist. Scales 32, default Oscillation, .02, 24, 32, 32, 24 respectively. No critics added/disabled. |
| Goal checker | Original stateful SimpleGoalChecker: XY .25 m, yaw .3 rad. Independent evaluator adds measured stop; it never substitutes for controller action success. |
| Footprint | Original ±.70×±.38 m rectangle, signed padding .05 m → full 1.50×.86 m. No shrink or obstacle-threshold changes. |
| Costmaps | Global map frame, local rolling odom frame, resolution .05 m; static+ObstacleLayer+inflation .8 m/factor3; global track_unknown_space=true; planner refuses unknown. |
| Smoother/interface | Original OPEN_LOOP velocity smoother .4/0/.6 limits, .5/0/.6 accel/decel → `/course_navigation/cmd_vel` → original adapter → RL-SAR `/path_manager/path_manager_ros/nav_vel`. M2 inverse and governor disabled. |
| Localization | Existing LiDAR/IMU LIO-SAM, `/slam/odom` in odom→base_link and map→odom TF. Truth never drives goals or localization. |
| Endpoint guard | Existing reviewed process-scoped Nav2 controller overlay is mandatory. Exact executable/mapped library hashes pinned in `stage4_runtime_identity.json`; underlay and duplicate controller processes rejected. |

Before sending anything capable of motion, the runner queries effective runtime
parameters, ACTIVE lifecycle states, publisher/client graph and actual controller
executable/mapped library. Defaults alone are not runtime proof. This turn verified
installed packages/libraries, launch expansion and overlay resolution offline;
no live parameter dump was available. Optional inherited DWB observability overlay
must remain off for this exact-parameter gate. Existing raw evidence is retained.

## Registration before motion

`map` as a frame name is insufficient. A bounded read-only capture freezes live
OccupancyGrid, fresh TF/SLAM/clock evidence, exact source selection hash and unique
clock/SLAM publisher GIDs. The robot must be stationary, the profile must match,
and competing producers must be absent. Saved/live occupied grids must contain
at least 200 points with structure spanning ≥4 m in each axis. Nearest saved
occupied-cell distance p95 must be ≤0.10 m.

An operator identifies 4…32 distinctive corresponding fixed structural landmarks
in the saved map and that captured live map: different rack/wall corners, spread
through the mapped area, not repeated ambiguous rack bays. `landmarks.json` is:

```json
{"live_map_sha256":"<copy capture.json value>",
 "saved":[["saved x1","saved y1"],["x2","y2"],["x3","y3"],["x4","y4"]],
 "live":[["live x1","live y1"],["x2","y2"],["x3","y3"],["x4","y4"]]}
```

Replace the quoted coordinate placeholders with actual numeric metres measured
from the two maps; these are registration landmarks, never retyped runtime goals.
The live grid's cell-center map coordinate is origin + rotated
`[(column+.5)*resolution,(row+.5)*resolution]`, ROS row 0 at bottom. The entire
numeric grid and origin/resolution are in `capture.json`; the saved map can be
viewed with the existing offline editor or a map viewer. Retain screenshots or
annotated corner selection alongside landmarks for human verification.

The rigid SE(2) fit requires non-collinear spread (smallest centered singular
value ≥2 m), translation norm ≤.05 m, rotation ≤.02 rad, RMS residual ≤.05 m,
maximum residual ≤.10 m. This only accepts coordinates already aligned with the
saved map. A larger fitted transform rejects; it is never silently applied to
goals or published as TF. Resolve localization/registration separately if rejected.
The numeric thresholds bound this acceptance check, not SLAM accuracy guarantees.

The certificate binds the capture/session, selection, map hash, graph and pairs.
At execution entry it expires after 120 wall seconds or 120 simulated seconds,
and clock rollback rejects. New SLAM/clock publisher GIDs reject cross-session
reuse. Saved/live map agreement and graph identity are checked repeatedly during
execution. Fresh per-link TF/SLAM remains mandatory. Correspondence ambiguity,
common map errors, unobserved clock resets between separate processes, drift below
the stated checks and full-body contact are residual uncertainties; this is not
certified global relocalization. Re-capture/register after interruption or restart.
If timely correspondence selection is impractical, do not extend/disable the gate.

## Independent acceptance contract

- Exactly G1,G2,G3,G4,G5 first accepted events, in order. For the short run only
  G1,G2. Later unvisited region entry fails. A missed region cannot be skipped.
- Each inclusive Euclidean XY radius .25 m uses fresh measured map→base_link,
  separately fresh map→odom and SLAM odom. Two distinct increasing TF/odom samples
  must remain inside for ≥.1 simulated seconds. Duplicate samples cannot confirm.
- Pose/TF/clock receipt ages and per-link TF/odom stamp ages ≤.5 s; SLAM map input
  age ≤2 s. Accepted samples have gaps ≤.3 simulated/.5 wall seconds and position
  increments ≤.2 m. Missing/stale/NaN/backward evidence fails rather than advancing.
- G1→G2 measured centers must stay within .35 m of the exact endpoint centerline
  capsule, with ≥2 corridor samples. This is a **restricted acceptance band**,
  not a surveyed corridor width or footprint clearance claim. Other segment bands
  are 3 m, allowing observed static-map detours but bounding them. G1→G2 cannot
  detour through the open area. Gaps are not bridged by inferred physical passage.
- Every measured sample checks the filled padded footprint against the saved map
  and fresh public local/global master costmaps; unknown, lethal or outside cells
  block; the center also rejects inscribed cost. Preflight separately samples the
  whole planned path at ≤.025 m/rad, checks ordered regions and corridor bands.
  Endpoint-only checks never authorize motion. Sampled public costmaps are not
  locked controller-private state or a continuous physical collision guarantee.
- G5 requires FollowPath status SUCCEEDED (4), all ordered region evidence,
  final XY ≤.25 and yaw error ≤.3, and ≥1 simulated second of fresh stationary
  evidence **after action success**. SLAM twist and pose differences must give
  speed ≤.03 m/s and |wz|≤.03 rad/s; fresh final nav_vel must be ≤.01 in each
  speed/angular measure. G2 is the stopping endpoint in the short run.
- Failure/rejection/abort/cancel/clock reset/concurrent action or goal/manual
  replacement/interruption cancels only the exact owned action. No retries.
  `execution-lock.json` consumes that session. Cleanup must confirm owned terminal
  status and fresh stopping; cleanup samples/faults remain part of acceptance.
- `PRACTICAL_ROUTE_PASS_SAFETY_UNCERTAIN` requires all observed contracts including
  cleanup. The current simulator provides only partial collision-plate coverage,
  so this implementation **cannot emit FULL_ROUTE_PASS in live transport**.
  Unknown full-body contact coverage is explicit, never zero-filled or inferred.
  Full-body safety evidence and independent review remain future work. The pure
  evaluator tests the complete-safety case, but no user flag fabricates it.
- Historical .05 m benchmark stays `not_run_unmodified`; neither practical route
  acceptance nor candidate geometry rewrites historical results.

Each session stores selection and registration hashes, source/config/code pins,
UUID/action outcomes, raw timestamped public CDR journal (256 MiB bound), graph,
parameter snapshots, measured trajectory, per-goal minimum XY error, terminal
error, corridor evidence, clock/wall times and cleanup. Pointcloud payloads are
hashed with header/size rather than fully retained; other selected messages retain
CDR. Publisher GIDs are graph identities, not per-message authentication in this
Humble rclpy version. Raw transport drops cannot be proven absent. Resource-limit
stops fail closed. Hard-killed runs may lack result.json and are never PASS.

## Offline planning and diagnosis results

The isolated `stage4_smac_probe` calls the **installed** AStarAlgorithm<Node2D>,
GridCollisionChecker and Smac smoother without ROS init, node, DDS, goals or
simulation. Input is the verified map reconstructed with conservative unknown
policy, lethal cells and .8 m/3.0 inflation using inscribed radius .43 m.
It reproduces the real 1.1.20 search contract and final individual goal yaw.
It does not reproduce the live layered master costmap/ObstacleLayer. Smoothing
per leg also cannot certify the concatenated controller's corner behavior.

| Segment | Center path | Padded footprint samples blocked | Conservative clearance lower bound |
| --- | --- | --- | --- |
| Start→G1, hypothetical (0,0) | found | 0/385 | .534 m |
| G1→G2 | found, center deviation ≤.236 m | 128/309, occupied | overlaps occupied cells |
| G2→G3 | found | 0/211 | .289 m |
| G3→G4 | found, detour ≤2.498 m | 127/439, occupied | overlaps occupied cells |
| G4→G5 | found, detour ≤1.907 m | 0/368 | .216 m |

All five endpoint filled footprints and all 73 discrete 5° yaw samples at each
endpoint are clear on the saved map. This supports inspecting turn/stopping
locations there; it proves neither braking distance nor continuous approach
clearance. Straight candidate-to-candidate headings also collide on G1→G2,
G3→G4 and G4→G5. No footprint, inflation or threshold was changed to force a pass.
The hypothetical Start→G1 plan comes within .204 m of G4 **before G1**; full-mode
preflight rejects this early future region too. Actual Start is unknown offline.

Missing: registered current Start, live local/global layered costmaps, dynamic
obstacles, full-body contact coverage and physical tracking/braking evidence.
There is **no runtime path-planning PASS or autonomous-route PASS**. Review the
route geometry/path shaping before attempting motion; current preflight is
expected to reject. The annotated plot and full search/check data are in the
review evidence directory.

Historical first DWB window: 190 retained records with CDR hashes verified; one
all-invalid evaluation, 419/419 BaseObstacle rejections; 29 native obstacle lines.
Nearest earlier public grid: every trajectory starts at cost 220 and subsequently
hits 253. Thus the observed mechanism is predicted center motion into inscribed
inflation, rather than proof of an already lethal robot-center cell. Grid age is
.465 s, nearest retained odom age .355 s. Map→odom is absent from this window;
controller-private inputs are not captured. Zero-velocity sample is absent even
though vx=0 and angular ranges ±.6 appear in the candidate set; an already moving
rollout can still translate. Lateral samples remain zero as configured. There is
no retained near-zero-command interval sufficient to quantify residual stopping.
Turn-too-late near G2, actual footprint overlap, braking residual and stale TF are
hypotheses, not established causes. Historical G2 goal/headings differ from this
candidate. The second retained window has zero messages and no diagnostic value.

## Bounded USER-RUN procedure: Start→G1→G2

**Deferred until this bundle's independent review and separate live-run
authorization. Current geometry is blocked: stop at the planner-only step if it
rejects, and do not issue the motion command below with this known failed plan.**
Do not let an agent execute these commands automatically. Use one idle warehouse
session, no RViz goal publisher, manual control, other runner, replay or competing
ROS action client. Do not run with `dwb_observability:=true` in this exact profile.

Offline preparation (safe now; choose a new output directory):

```bash
/usr/bin/python3 scripts/run_candidate_route.py select \
  --candidate /home/zyd/b2w-course-evidence/waypoints/20261019-v1/candidate-16nlistv/candidate.yaml \
  --map /home/zyd/b2w-course-evidence/waypoints/20261019-v1/warehouse.yaml \
  --output /tmp/stage4-prepared-UNIQUE
```

Future user owning terminal, after normal shutdown of any previous stack:

```bash
ROS_LOG_DIR=/tmp/stage4-user-logs-UNIQUE \
 scripts/with_nav2_controller_overlay.sh ros2 launch course_bringup course.launch.py \
 mode:=navigation scene:=warehouse gui:=true rviz:=false \
 map_mode:=online adaptive_speed:=false goal_manager_enabled:=false \
 nav_params_file:="$PWD/course_navigation/config/nav2_smac2d_diag.yaml"
```

This launches sensors/SLAM/Nav2 without automatic goals. It does not re-register
SLAM to the saved map. Wait for stationary stable SLAM. In another terminal at the
repo root, with the same ROS_DOMAIN_ID/DDS settings (wrapper defaults domain91):

```bash
scripts/with_nav2_controller_overlay.sh /usr/bin/python3 scripts/run_candidate_route.py capture \
 --selection /tmp/stage4-prepared-UNIQUE/selection.json \
 --session /tmp/stage4-short-UNIQUE --capture-live
# Inspect capture.json and independently choose the numeric landmarks described above.
/usr/bin/python3 scripts/run_candidate_route.py register \
 --session /tmp/stage4-short-UNIQUE --landmarks /tmp/landmarks-UNIQUE.json
scripts/with_nav2_controller_overlay.sh /usr/bin/python3 scripts/run_candidate_route.py plan \
 --selection /tmp/stage4-prepared-UNIQUE/selection.json \
 --session /tmp/stage4-short-UNIQUE --mode critical --capture-live
```

Capture is read-only (15 s discovery loop plus bounded service checks); plan is
read-only with an 8 s planning bound and 5 s cleanup. Both require stationary
fresh data and valid runtime identity. Capture's entire grid can be rendered
using origin/resolution for landmark selection; retain annotations. If selection
takes over 120 s, capture/register again into a new session rather than reuse an
expired certificate. Missing data or uncertain correspondence means stop.
Retain all `plan-audit-*`, `rejection-*`, capture and raw files for review.

Only after a reviewed safe plan, resolved geometry/safety limitations and explicit
user authorization, the short-run motion entry is:

```bash
scripts/with_nav2_controller_overlay.sh /usr/bin/python3 scripts/run_candidate_route.py run \
 --selection /tmp/stage4-prepared-UNIQUE/selection.json \
 --session /tmp/stage4-short-UNIQUE --mode critical --execute-user-run
```

It rechecks registration/config/graph and a newly computed **whole path before
motion**, then follows that exact checked path. Maximum 120 wall/90 simulated
seconds plus bounded preflight/cleanup. Stop manually with Ctrl+C on runner if
needed; it cancels its exact action and attempts a fresh stop confirmation. If
cleanup is unconfirmed, use the owning launch's normal Ctrl+C shutdown and retain
logs; do not issue replacement goals to hide failure. The runner does not own or
kill the user's launch. The owning terminal must shut it down normally after the
run; launch shutdown evidence is separate from action cleanup in this attach-only
runner. Inspect result.json/evidence-manifest.json and native logs independently.
A practical result still carries full-body collision uncertainty.

## Subsequent full route, explicitly deferred

Do not continue from G2 by sending G3–G5. Review the complete short-run evidence
first; resolve footprint conflicts, early out-of-order regions, map registration
and safety coverage. Only after that review and a new explicit authorization,
start a fresh original-Start session, repeat capture/landmark registration and
planner-only validation with `--mode full`, then use the same `run` entry with
`--mode full`. This is one G1–G5 planned Path and one FollowPath, bounded at
360 wall/240 simulated seconds plus preflight/cleanup. Each session is single-use;
there is no automatic retry, resume, goal skip or relabeling of the 0.05 m baseline.
This procedure is not currently cleared for execution.

## Review gate

One april-review-bundle contains the source, scoped report, tests/build, installed
identity, actual selected V2/selection, native search, failure analysis and
integrity evidence. Stop after generation. Independent ChatGPT Review PASS is
required before requesting authorization for one narrowly scoped local Stage 4
commit in a later turn. No commit/push/merge is authorized now.

Version-specific source cross-checks: [Smac2D 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/smac_planner_2d.cpp),
[planner server](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_planner/src/planner_server.cpp),
[RemovePassedGoals](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_behavior_tree/plugins/action/remove_passed_goals_action.cpp).
Installed actions, headers, libraries and package versions were checked locally;
no assumption is taken from current Nav2 main.

Final offline verification: 52 targeted Stage 4 tests passed; the broader final
suite passed 293 tests with one loopback HTTP test deselected. That HTTP test
passed separately (1 test) with a narrowly approved loopback socket exemption;
the original sandbox failure is retained. Course navigation/bringup built in
2.10 s; native Smac probe built and tests enforce lethal/unknown barriers.
Overlay ELF/header/ABI resolution checks passed. These are offline results only.
