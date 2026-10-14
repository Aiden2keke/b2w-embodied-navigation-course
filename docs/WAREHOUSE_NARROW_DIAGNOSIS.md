# Warehouse narrow-aisle execution failure diagnosis

> Current review (2026-10-14): the exact user-run LIGHT evidence below now reproduces the aisle stall. Healthy pre-failure receipts are compatible with20Hz. The installed1.1.20 frozen critic comparison shows four center-legal/perimeter-illegal local poses. First rejecting critic and physical contact remain unresolved. Earlier smoke/transparency statements below are historical; the captured-failure review is authoritative. The separate footprint-gate diagnostic is now implemented and statically validated; no live experiment was run by Codex.

Historical captured evidence scope: branch `improve/narrow-navigation`, HEAD `0c31f08e19ce51849f2c06b107e8a89b2ff55ef7`, current source/config, supplied GUI recordings and the completed user-run 2026-10-14 smoke. No warehouse experiment, goal, cancellation, navigation parameter change, commit or push was performed. Root `PLAN.md` is the only current plan. The first causal layer of the stall is **unresolved**. Apparent physical contact remains a hypothesis.

## Video observations

Both `NavFn.mp4` and `Smac2D.mp4` were decoded locally using OpenCV 4.5.4, with overviews every five seconds and denser frames around transitions. The corresponding `.webm` recordings are present; MP4 was used for visual inspection. No OCR. Frames stay in `/tmp/b2w-warehouse-diagnosis/`, never Git. `ffprobe` is unavailable. OpenCV's FPS-based random seeking gives mismatched times on these variable-rate recordings; the observations below use **sequentially decoded presentation timestamps**. Final buffered frames report zero PTS, so approximate endpoints also use file metadata: NavFn about 168.4 s, Smac about 117.3 s. These are recording seconds, not simulation time. Event windows intentionally remain broad where robot motion or route lines are obscured by the point cloud or camera movement.

| Event | NavFn recording | Smac2D recording | Visual limits |
|---|---|---|---|
| Initial goal interaction / route appearance | approximately 6–12 s | approximately 4–10 s | Goal arrow/path and subsequent motion, not a measured ROS publication time. |
| Initial global route choice | approximately 10–30 s: detour around obstructing pallet/shelf region | approximately 8–25 s: more direct narrow-aisle approach | Different observed live maps; no same-input algorithm comparison. |
| First successful-direction passage | Detour brings robot around the obstacle region by approximately 40–50 s | Narrow passage/continuing progress approximately 20–50 s; robot emerges at the near end | Visual progress, not action-result or full-body clearance certification. |
| Later goal / route change toward return passage | approximately 50–60 s | approximately 53–59 s (2D Goal Pose tool visible near 55 s) | Cannot distinguish every replan from a new goal without `/goal_pose` and `/plan`. |
| Narrow aisle entry in failing traversal | approximately 62–75 s | approximately 65–71 s, following turn at near end | Approximate center entry, not whole-body crossing. |
| Slowing / loss of continuing progress | approximately 78–90 s | approximately 70–77 s | No velocity traces; braking, contact and commanded zero cannot be ordered visually. |
| Final broadly stationary period | approximately 90–168 s, including later camera changes | approximately 77–117 s | Small body/joint motion and apparent RViz shifts remain; not an odometry estimate. |
| Defensible contact observation | No confirmed visible contact | approximately 70–77 s: body approaches pallet/shelf side during return turn/passage; projected near-contact is plausible | Gazebo's oblique view and RViz map cannot establish contact depth, force, or which collision surface touched. A wall/shelf collision is not asserted as fact. |

The strongest video fact is **execution asymmetry after a route exists**. One Smac direction progresses through the aisle while the return stalls near obstacles. The videos do not show raw commands, DWB scores, plate wrench or controller exceptions. They cannot rank H1 against H2 as proven causes. Earlier frozen NavFn extraction failure in `NARROW_BASELINE.md` concerns global planning on another input and does not explain a mid-execution stall after a usable path appears.

## Current source/config architecture and snippets

Repository snippets below are current, not inferred from a video filename. Installed `dwb-core`, `dwb-critics` and `nav2-controller` report Humble **1.1.20**. Actual running-session parameters are captured by the observer; no running warehouse parameter dump was taken in this task.

| Item | Path + snippet | Interpretation |
|---|---|---|
| 1. Global planners | `course_navigation/config/nav2.yaml:218`: `GridBased.plugin: nav2_navfn_planner/NavfnPlanner`, `use_astar: true`, `tolerance: 0.3`, `allow_unknown: false`; `course_navigation/config/nav2_smac2d_diag.yaml:225`: `plugin: nav2_smac_planner/SmacPlanner2D` | The diagnostic YAML differs from baseline by exactly the plugin line. NavFn uses A*; Smac2D does not consume NavFn's retained `use_astar` key. Smac 1.1.20 defaults `cost_travel_multiplier` to 1.0; no new multiplier is selected. Video labels identify intended configurations, not runtime plugin proof. [Smac source](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/smac_planner_2d.cpp). |
| 2. DWB generator / limits | `course_navigation/config/nav2.yaml:87`: `plugin: dwb_core::DWBLocalPlanner`, `min_vel_x: 0.0`, `max_vel_x: 0.4`, `max_vel_theta: 0.6`; `acc_lim_x: 0.5`, `decel_lim_x: -0.5`, theta acceleration/deceleration ±0.6; `vx_samples: 20`, `vy_samples: 5`, `vtheta_samples: 20`, `sim_time: 1.7`, granularities 0.05 m / 0.025 rad | Generator is **StandardTrajectoryGenerator**, the versioned default when `trajectory_generator_name` is absent. Sampling/rollout uses kinematic limits and current `/slam/odom` twist; a normal DWB rollout cannot select reverse vx. Recovery backup is a separate behavior. Controller runs at 20 Hz. [DWB initialization](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp), [generator](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_plugins/src/standard_traj_generator.cpp). |
| 3. Active critics | `course_navigation/config/nav2.yaml:115`: `critics: [RotateToGoal, Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist]`; scales respectively **32, 1(default), 0.02, 24, 32, 32, 24** | `Oscillation.scale` is not set; the base critic declares 1.0. Path/Goal alignment use forward point 0.1 m; RotateToGoal slowing factor 5, lookahead −1. These are configured scales, not comparable physical weights. MapGrid-derived critics multiply scale by resolution/2 (0.025 here): PathDist/PathAlign normally 0.8, GoalDist/GoalAlign 0.6. PathAlign can set its effective scale to zero near goal. Runtime evaluation includes actual scales; positive short-circuited scores may be partial. [Critic default](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_core/include/dwb_core/trajectory_critic.hpp), [MapGrid scale](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/include/dwb_critics/map_grid.hpp), [PathAlign](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/path_align.cpp). |
| 4. Obstacle critic semantics | Official 1.1.20 `base_obstacle.cpp:83`: `costmap_->getCost(cell_x, cell_y)`; rejects 253/254/255 and off-grid centers. `obstacle_footprint.cpp:96`: rasterizes oriented footprint edges; `pointCost` rejects 254/255 | **BaseObstacle** checks every sampled trajectory center, with inflation representing an inscribed-radius approximation. It does not explicitly sweep a rectangle. Its default `sum_scores=false` returns the last pose's obstacle cost after checking all poses. **ObstacleFootprint** reads the padded robot footprint and checks its perimeter at each pose; 253 is a legal scored edge cost in this version. It does not fill the polygon interior or provide 3D physical contact detection. It is absent from both original configs; the new independent footprint-gate diagnostic enables it alongside BaseObstacle. [BaseObstacle](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/base_obstacle.cpp), [ObstacleFootprint](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp). |
| 5. Local/global footprint | `course_navigation/config/nav2.yaml:159,215`: `[[0.70,0.38],[0.70,-0.38],[-0.70,-0.38],[-0.70,0.38]]`, `footprint_padding: 0.05` | Both use original 1.40×0.76 m, padded **1.50×0.86 m**, half-length .75 / half-width .43; inscribed radius .43, circumscribed radius ≈.8645. Nav2 pads signed x/y components. At yaw θ the modeled lateral span is `1.50*abs(sin θ)+.86*abs(cos θ)`; worst span ≈1.729 m. This model is not the moving legs' measured envelope. [Padding](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/footprint.cpp). |
| 6. Inflation / observation | `course_navigation/config/nav2.yaml:150,210`: `inflation_radius: 0.8`, `cost_scaling_factor: 3.0`; both resolution .05; static+obstacle+inflation; lidar marking/clearing, heights .15–2 m, minimum marking/raytrace range .6 m | Inscribed band is 253; outside .43 m the exponential cost decreases toward the .8 m inflation extent. .8 m is not a required rectangular-body clearance. Local costmap is rolling 5×5 m in `odom`, update/publish 5/2 Hz; global is in `map`, update/publish 1/1 Hz, warehouse map 40×40 m. Projection, footprint clearing, sensor blind ranges and missing low/high surfaces affect geometric evidence. [Inflation](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp). |
| 7. Progress checker | `course_navigation/config/nav2.yaml:78`: `SimpleProgressChecker`, `required_movement_radius: 0.15`, `movement_time_allowance: 20.0` | Resets its baseline when planar displacement is **greater than .15 m**. Pure rotation does not reset it. Fails after more than 20 simulation seconds from its last baseline; this may precede/follow the visual stop depending on earlier slow movement. A later progress error is a consequence candidate, not proof of the initial cause. [Checker](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/plugins/simple_progress_checker.cpp). |
| 8. Controller failure tolerance | `course_navigation/config/nav2.yaml:72`: `failure_tolerance: 0.3` | In 1.1.20 a compute-command PlannerException produces zeros during tolerance; after >.3 s since last valid command it raises controller patience exceeded. The progress check occurs before that compute-command catch. This is distinct from the 20 s progress allowance. [Controller server](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp). |
| 9. Smoother | `course_navigation/config/nav2.yaml:278`: `feedback: OPEN_LOOP`, `smoothing_frequency: 20.0`; max `[.4,0,.6]`, min `[-.2,0,-.6]`, accel `[.5,0,.6]`, decel `[-.5,0,-.6]`, `velocity_timeout: .5`, zero deadband | Smooths command transitions and times out stale input. SLAM odom topic is configured, but feedback mode is open loop. Its reverse limit does not enable reverse DWB trajectories. |
| 10. vy disabled twice | `course_navigation/config/nav2.yaml:91,93,99,102`: min/max vy, y acceleration/deceleration all zero; smoother's y limits also all zero | Five requested vy samples cannot create nonzero lateral velocity under zero limits. This is a high-level constraint; it does not prove that lateral recovery would succeed. |
| 11. Adapter and rl_sar accept vy | `course_navigation/course_navigation/velocity_adapter.py:27,69,76`: `(x,y,yaw)` retained with planar-norm clamp; `b2w_sim/b2w_controllers/src/rl_sar_controller.cpp:97`: subscribes nav_vel and clamps `linear.y` to ±.5; `:148`: `obs[6+i]=command_valid?command_[i]:0` | Interface retains lateral input. Adapter max planar speed .5 / yaw .6, timeout .5 s, 20 Hz steady timer; stale command or qualifying SLAM state, backward odom stamp, or missing map→base TF can zero final nav_vel. RL accepts vx ±1, vy ±.5, yaw ±1; policy runs 50 Hz, torque 200 Hz. `course_sim/config/rl_sar.yaml:141`: command timeout .5, feedback timeout .15, idle_hold true, yaw_command_gain 1.6; requested yaw is rescaled before inference. Interface support is not tracking performance evidence. |
| 12–13. Physical contact coverage | `b2w_sim/b2w_description_ros2/urdf/b2w_description_rsl.urdf:1585`: plate collision `box size="0.95 0.45 0.02"`; fixed joint offset `[0,0,.1]`; joint force-torque sensor 100 Hz, child frame / child_to_parent. `course_sim/config/bridge_rl_sar.yaml:31`: `WrenchStamped` bridge | Plate is thin and narrower than the navigation footprint. Its joint wrench includes dynamic/inertial loading; it is not a contact pair or full-body bumper. Base, head/tail, lidar, legs and wheels have separate collision shapes. The active URDF has no full-body contact sensor. Foot-contact xacro belongs to a different description path and is not generated here. `course_sim/course_sim/description.py:23,40` replaces selected inertial/dynamics fields from training URDF and removes old sensor plugins; plate sensor remains and world ForceTorque system supplies it. Silent plate cannot exclude contacts elsewhere. Existing goal-manager >50 N threshold is only an evaluation heuristic. |
| 14. Existing DWB outputs | `nav2_dwb_controller/dwb_core/src/publisher.cpp:70`: publish_evaluation/global/transformed/local/trajectories default true; cost_grid_pc false; `:97`: creates `evaluation`, `received_global_plan`, `transformed_global_plan`, `local_plan`, `marker`, `cost_cloud` | With current root namespace these are `/evaluation`, `/received_global_plan`, `/transformed_global_plan`, `/local_plan`, `/marker`, `/cost_cloud`. `debug_trajectory_details: false` controls detailed exception logs, not evaluation availability. LIGHT omits all four evaluation/debug-plan subscriptions; explicit `--dwb-debug` restores them after discovery, without changing any parameter. All-invalid evaluations and failed-score critic names can support H1 even without verbose logs. [Publisher](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_core/src/publisher.cpp). |

The command chain is `controller_server → /course_navigation/cmd_vel_raw → velocity_smoother → /course_navigation/cmd_vel → velocity_adapter → /path_manager/path_manager_ros/nav_vel → rl_sar`. Names/remaps are exact in `course_navigation/launch/navigation.launch.py:24`. Recovery behaviors publish to the intermediate smoothed-command topic, so a raw-controller-only comparison during recovery is insufficient. `/slam/odom` feeds controller/localization; `/sim/ground_truth/odom` remains evaluation-only. RViz uses `/goal_pose`; `/plan` is its global-path display topic. NavigateToPose/FollowPath UUIDs are retained without asserting a one-to-one action association.

## Evidence-ranked hypothesis matrix

Ranking means priority supported by **current static/video evidence**, not estimated cause probability. Several hypotheses can be sequentially true.

| Priority | Hypothesis | Actual support | Missing/disconfirming evidence / discriminator |
|---|---|---|---|
| 1, mechanism to test | H3: center/inscribed approximation admits rectangular corner collision | Active BaseObstacle is center-cell based; padded rectangle's corner radius exceeds inscribed radius. Return turn occurs close to obstacles. | No recorded center-safe/footprint-lethal-overlap pose or measured contact yet. Full-grid overlap with 254 plus command/truth/contact timing would strengthen it. A center-safe pose alone proves nothing about physical contact. |
| 2, tied causal candidates | H1: useful DWB trajectories invalid, controller zeros | Active hard-rejecting critics and narrow geometry can invalidate candidates; stall follows an existing path. | No zero-before-motion trace or DWB invalid evaluation from the videos. All-invalid evaluation or exact No valid trajectories log preceding raw zero supports H1. Legal trajectories with nonzero output weaken it. |
| 2, tied causal candidates | H2: commanded motion continues, contact blocks motion | Smac return appears near a shelf/pallet while forward passage worked. | Apparent contact is projection evidence only. Sustained final nonzero + stationary truth supports B; contemporaneous plate rise strengthens contact, but silence cannot reject it. |
| 3, established constraint, unproven cause | H5: vy=0 prevents centering/recovery | DWB and smoother y channels are demonstrably zero; a return turn changes rectangular clearance. | No evidence that a lateral maneuver is needed, feasible or tracked. DWB candidates, footprint pose and local path determine whether centering is the limiting mechanism. |
| 4, plausible unmeasured alternative | H6: RL fails to track vx/wz under contact/tight turn | Robot is driven by an RL policy with timeout/feedback gates and yaw scaling; interfaces do not guarantee tracking. | No measured tracking residual. Nonzero final command and policy-input debug with near-zero truth, weak contact evidence and feasible costmap support tracking investigation; cannot separate uninstrumented contact from RL solely by this observer. |
| Downstream-only candidate | H4: progress checker aborts after prior stall | 20 s displacement-based checker is structurally downstream of lost progress. | No progress log from videos. Simulation occurrence stamp after the measured stationary onset supports E. Receive-only logs or an ABORTED status cannot establish ordering. |

Also inspect the **smoother/adapter boundary**: raw motion + smoothed zero suggests smoother/command-source behavior; smoothed motion + final zero suggests adapter command/SLAM freshness or TF gate. This is a separate measurable possibility not covered by H1–H6. TF/SLAM coverage failures must remain visible.

## Observer artifacts and interpretation

Added `scripts/observe_warehouse_narrow_failure.py`, pure `scripts/warehouse_observer_helpers.py`, and `tests/test_warehouse_observer.py`. The script attaches only. It creates subscriptions and discovered List/GetParameters clients, no command publishers, action clients, navigation parameter setters, subprocesses, launches or kill operations. rclpy's normal node construction creates its own parameter-event housekeeping publisher; this does not write any existing node's parameters. It disables its own rosout publisher and parameter services. Parameter client future cancellation is local cleanup, never navigation cancellation.

Each invocation creates `/tmp/b2w-warehouse-observer/<UTC timestamp>-<label>/` (or chosen `--output-root`). `--duration` is wall seconds; default 300; zero waits for observer-terminal Ctrl+C. Simulator pauses do not end the wall-time deadline. Detach leaves GUI/navigation running.

- `manifest.json`: domain, observer PID, explicit `light` / `dwb-debug` mode, selected/omitted topic contract, thresholds, and final `measurement_health`.
- `events.jsonl`: per-event sequence, simulation receipt time, source stamp, monotonic and UTC receipt time, epoch, full commands/odom/TF/paths/status UUIDs/wrench, relevant rosout/BT logs and evaluation score summaries. With `--dwb-debug`, DWB records every candidate velocity/total/critic score and rejection critic; only the best valid predicted pose sequence is retained. Positive scores may have short-circuited, so they do not establish every critic accepted.
- `samples.jsonl`: causal alignment at about 10 simulation Hz; command age limit .3 s, stamped odom/wrench .5 s; source sequence IDs/ages, map→base and odom→base TF, current plan/goal/footprint references, statuses and local/global master references. Backward clock starts a new epoch. Unknown/stale commands remain null, never zero. Activity becomes unknown when its status publisher vanishes or changes generation. Action status is transition evidence, not a periodic pose sensor.
- `runtime-parameters.json`: actual read-only node parameter values, including generator/critics/footprint/planner defaults. Missing nodes are explicit. It is a configuration snapshot, not used as an assumed time-varying footprint.
- `costmaps/*.bin` + per-message JSON: actual raw uint8 **master** costs from `/local_costmap/costmap_raw` and `/global_costmap/costmap_raw`, content SHA256, dimensions/resolution/origin/frame/source stamp. Binaries deduplicate by bytes; metadata preserves rolling origins and timestamps. No OccupancyGrid cost conversion. These are published snapshots, not atomic controller-memory traces.
- `context/*.json`: complete current paths, goal and published footprint messages.
- `stall-*.json`: pre-onset, onset, about 1 s after onset, and end samples, with raw local/global masters and current path/robot-pose references. Exact-stamp TF enrichment is deferred up to 15 wall seconds because SLAM TF lags current clock; retries are 10 Hz and only new metadata is written.
- `summary.json`: explicit mode, final `measurement_health`, candidate stationary intervals, evidence cases/reasons, missing-topic/type/no-message health, errors, available parameter nodes and cleanup. `first_causal_layer` stays null until evidence is reviewed; no automatic root-cause verdict.

Stationary candidates require observed active navigation, a continuous same-goal/same-epoch/same-frame segment of ≥3 simulation seconds, position extent ≤.04 m and yaw excursion ≤.1 rad. Prefer truth; SLAM-only stationarity is not certified physical immobility. Missing poses, source changes, gaps >.35 s, rotations or goal replacements break the segment. Intentional BT wait, goal transitions, slow creep and oscillation can need manual inspection even when a candidate exists. No candidates is not a successful-traversal verdict.

| Evidence case | Required support | What remains uncertain |
|---|---|---|
| A | Continuous fresh raw-zero suffix with measurable subsequent displacement before the stationary interval | Timing uses unstamped Twist receipt; inspect full event order. Zero only after a stop does not get A. |
| B | Materially nonzero final vx/vy/wz throughout interval with stationary evaluation truth | Contact vs low-level tracking failure remains unresolved; SLAM disagreement is explicit. |
| C | Observed quiet plate baseline followed by >50 N near onset (within ~1 s) | Wrench is real simulated joint-force measurement, not a full-body contact pair; loading can spike without shelf contact. Missing baseline does not get C. |
| D | Exact feasibility/exception log around stall, plus available DWB evaluation/rejection critics | Exception timing/cause and trajectory scores must be inspected; action ABORTED alone is insufficient. |
| E | Valid simulation occurrence stamp for progress failure after ≥3 s of prior stationarity | Receipt-only/mismatched timestamps remain ordering-unknown; a later abort does not identify the first cause. |

Offline geometry uses a **filled convex padded footprint** versus closed raw cell areas. It recovers the already padded shape from the temporally matched published footprint, with exact TF at footprint and costmap stamps. No second padding is applied. Footprint header is Nav2's robot-pose stamp, often lagging costmap publication; allow and report ≤.75 s skew, assuming shape remains constant over that skew. Missing matches/TF/frame agreement leave geometry unknown. This is intentionally more conservative than Nav2's perimeter-only ObstacleFootprintCritic, not a simulated replacement critic. [Nav2 footprint publication](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_ros.cpp).

Outputs distinguish center-cell cost, lethal(254) overlap, inscribed(253) overlap, unknown(255) overlap and out-of-map. `center_safe_footprint_colliding` means center cost <253 **and padded polygon overlaps a represented lethal cell**. Inscribed overlap alone is inflated-zone overlap and cannot prove body collision. Minimum distances to lethal and to lethal-or-inscribed **cell areas** are 2D finite-grid estimates; null means no such cells, not infinite physical clearance. Unknown/out-of-map forbids a clear verdict. Grid phase/resolution, clearing under the robot, map errors and 3D robot articulation limit inference. Physical contact measurement and costmap geometric test remain separate fields.

## LIGHT / DWB-debug and official subscriber gates

Default LIGHT retains `/clock`, `/goal_pose`, `/plan`, raw/smoothed/final commands, `/slam/odom`, evaluation-only truth odom, plate wrench, both action status topics, BT/relevant rosout, TF/static TF, both raw costmaps and published footprints, and `/network_input_debug`. It creates no subscriptions to `/evaluation`, `/received_global_plan`, `/transformed_global_plan`, `/local_plan`. `--dwb-debug` adds exactly those four and retains the previous per-candidate score/rejection/best-pose recording semantics. Graph discovery may list omitted topics; discovery does not subscribe. No Nav2 parameter is changed.

Official [Nav2 Humble 1.1.20 publisher.cpp](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_core/src/publisher.cpp), path `nav2_dwb_controller/dwb_core/src/publisher.cpp`, lines 158–168:

```cpp
if (publish_evaluation_ && eval_pub_->get_subscription_count() > 0) {
  auto msg = std::make_unique<dwb_msgs::msg::LocalPlanEvaluation>(*results);
  eval_pub_->publish(std::move(msg));
}
```

The subscriber gate encloses the full evaluation copy and publication. Adding this observer can activate that publisher-side work; DDS transport, Python deserialization, trajectory-score traversal and JSON writes add measurement work. This is a supported perturbation mechanism, not proof of the smoke's cause.

Same source, `publishGenericPlan`, lines 359–367:

```cpp
if (pub.get_subscription_count() < 1) {return;}
```

This gate precedes conversion/publication for received and transformed plans and the Path2D local-plan overload. The trajectory local-plan overload at lines 234–249 constructs its Path before checking subscription count, but still gates publication. LIGHT does not eliminate every internal plan computation/copy. Other subscribers may continue activating debug publications. [publisher.hpp](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_core/include/dwb_core/publisher.hpp), `shouldRecordEvaluation`, uses publication flags in `shouldRecordEvaluation`, not subscription counts; removing our subscription does not promise to disable internal score accumulation. Neither mode is assumed transparent before timing validation.

## Observer self-health definitions

Both final manifest and summary contain `measurement_health`:

- Wall duration: monotonic attach interval, ending before ROS teardown and offline stall summarization. Simulation duration: sum of received clock advances within epochs, excluding reset jumps; absent clock or no within-epoch pair is null. Sim/wall ratio uses those durations.
- Per selected topic: received callback count and count / attach wall duration in `observed_receive_hz_wall`. Count includes all rosout callbacks, although only relevant logs are saved. No receipt means count 0 and rate unknown. This average includes idle time; it is **not automatically a publisher/control rate**.
- Aligned sample count / wall duration and / simulation duration. Clock pauses/reset gaps and missing clocks remain explicit; these are observed sample rates, not a configured timer guarantee.
- TF age p50/p95/max separately for map→base and odom→base, using aligned sample simulation clock minus observer-local TF stamp. Missing, negative and nonfinite ages are excluded and statistics retain valid count; no valid ages gives null percentiles/max.
- Raw/smooth/final command age p50/p95/max, including stale causal history in `observed_ages`; linear-interpolated percentiles. Unstamped Twist age uses simulation receipt time, so transport/callback backlog latency remains unknown. These ages do not certify controller freshness.
- Fresh aligned sample count/fraction for raw/smooth/final/SLAM/truth/wrench. Fresh requires a finite value and nonnegative bounded age (.3 command / .5 odom/wrench simulation seconds); stale/missing values are null in causal samples and never become zero commands. Fraction denominator is all aligned samples; no samples gives null fraction. Zero fresh coverage does not mean zero physical motion.
- Final output directory bytes, including manifest/summary and every data artifact, measured after closing streams and completing offline summaries. Explicit mode is `light` or `dwb-debug`.

## Completed user-run smoke: functional attach, transparency unresolved

Inspected `/tmp/b2w-warehouse-observer/20261014T060802.404851Z-smac2d-smoke` without modifying it. Summary: 60.059389878 wall seconds, 388 aligned samples, no observer errors, no stationary candidate, all 24 expected topics recorded, all seven runtime parameter nodes captured, no missing parameter node. Runtime `controller_server.controller_frequency` = 20 Hz. This predates mode selection and used all four debug subscriptions.

User observation: short forward goal succeeded; second goal traversed the narrow aisle more slowly but succeeded; observer ended; return through the same aisle after observer exit also succeeded. **Live attach is functionally validated and topic/runtime coverage is validated. No stall reproduced. Previous return-direction failure is therefore not deterministic.** Successful traversals come from the user's observations, not merely absence of candidates. This success does not confirm or reject H1–H7 and does not authorize tuning.

421 `/evaluation` and 421 raw command receipts correspond to ~7.01 receipts / attach wall second over the whole run, including idle periods. This is not proof of a controller running at ~8 Hz or any other measured loop frequency. Subscriber/callback backlog or drops are possible explanations. Observer-local TF ages over 387 valid samples:

| TF | p50 (s) | p95 (s) | max (s) |
|---|---:|---:|---:|
| map→base | 3.6025 | 6.9630 | 8.7375 |
| odom→base | 2.8250 | 4.2945 | 5.5500 |

These timing observations motivate LIGHT plus health measurements. They do not establish controller-side TF ages, slowdown or causality. Current full observer transparency is **not validated**. Audit evidence: `/tmp/b2w-observer-light-review/completed-smoke-audit.json`.

## Historical protocol — superseded by the captured LIGHT failure below

Attach to the existing intended GUI session; do not launch another warehouse or change planner/configuration. In a zsh terminal, one exact command block (domain 91 matches the completed smoke):

```zsh
cd /home/zyd/program/b2w-embodied-navigation-course
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/course_sim/config/fastdds_local.xml"
export PYTHONPATH="$PWD/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$PWD/.deps/lib:$PWD/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=2
/usr/bin/python3 scripts/observe_warehouse_narrow_failure.py --label smac2d-light-smoke --duration 60 --output-root /tmp/b2w-warehouse-observer
```

Use the same short/aisle goals manually in RViz for this timing smoke, leaving navigation configuration unchanged. Review `mode=light`, omitted debug subscriptions, topic/runtime coverage, sim/wall ratio, sample rate, TF-age distributions and fresh signal coverage. Rates during this mixed idle/active smoke need event-window context; do not compare whole-run raw receipt Hz directly to 20 Hz. Healthy timing should show sustained fresh causal coverage during navigation and no recurring multi-second observer-local TF backlog; thresholds come from documented alignment limits and existing TF sources, not a fabricated control-rate requirement. If timing remains unhealthy, investigate observer work/transport first; do not proceed to failure attribution or tuning. LIGHT timing is still user-run and unvalidated by Codex.

Only after healthy LIGHT timing, later reproduction uses `--duration 0` on the same observer command (retain LIGHT), label `smac2d-light-repro`. Repeat the same outbound/return narrow route, **maximum three round trips**, stopping repetitions at the first failure. If a stall occurs, leave it unchanged for **at least 25 simulation seconds**, then Ctrl+C **only the observer**. Do not send a replacement goal, cancel navigation or terminate the GUI during that evidence window. If all three round trips succeed, record **intermittent / not reproduced** and do not tune the controller yet. Optional `--dwb-debug` is a separate explicit measurement choice, never an automatic next experiment.

## Verification, cleanup and decision boundary

Only observer unit and compile/help/static checks were run in this revision. Red evidence: `/tmp/b2w-observer-light-review/tests-red.log` (9 new failures, 21 old passes); green evidence and static results are saved alongside it. Existing stall classification and the detailed evaluation callback remain unchanged. Python compile/help are checked without creating a ROS node; 30 observer tests passed. Static audit checks no launch/action/publisher/set-parameter/process-control API in the observer. **User-run live attachment/coverage is functionally validated; observer transparency and a real stall remain unvalidated**. The next measurement is the user-run LIGHT timing smoke, not an agent-run experiment. Missing subscriptions/runtime matches are visible and must not be interpreted as absent collision or zero command.

No Gazebo/Nav2/observer session was launched by the agent; only short-lived extraction, source-download and verification tools ran and finished. No session process was signalled. Review agent performed pure tests/read-only review only. At that historical turn, user videos were untracked and unmodified; they are now committed through Git LFS at the current HEAD described below; extracted frames and source copies remain in `/tmp`. All 270 protected tracked files match their initial hashes; HEAD remains unchanged. Static AST comparison preserves every old helper and the detailed evaluation block. No commit/push.

No behavior change is recommended yet. Critic replacement, weight/inflation/footprint/vy/Smac-cost/planner/SLAM/policy changes remain deferred. A later proposal must first argue **against** the change (alternate explanation, regression risk and evidence gaps), then **for** it (identified causal layer and expected discriminator), then give a **neutral verdict and reversal conditions**. Nothing in a visually plausible collision authorizes tuning here.

After all documentation/checks, final operation is the requested April bundle. Original `~/bin/april-review-bundle` contains prohibited automatic-abort/exit lines; use a controlled `/tmp` copy removing only those lines, leaving bundle logic and the original unchanged. Final output **must** remain `/home/zyd/APRIL_REVIEW_BUNDLES`; temporary evidence/logs stay under `/tmp`. Verify final absolute path and SHA256, then stop without edits, commit or push.


---

# Captured LIGHT warehouse failure: frozen causal review

Evidence: `/tmp/b2w-warehouse-observer/20261014T065029.802588Z-smac2d-light-smoke`. Exact directory exists; no substitute. User observation: “The robot got stuck in the narrow aisle.” Branch `improve/narrow-navigation`, HEAD `0c31f08e19ce51849f2c06b107e8a89b2ff55ef7`. Offline analysis only. No Gazebo, live observer/navigation/action, parameter mutation, behavior YAML, commit or push. Preserve inherited observer/docs/video work. Output `/tmp/b2w-light-causal-review`.

## 1. Observer transparency

Recorded mode **light**, 60.00087861897191 wall seconds, 59.89999999995254 simulation seconds, ratio **0.998318714303169**, one clock epoch, 590 aligned samples, errors `[]`, all seven runtime parameter nodes captured. Manifest omits `/evaluation`, `/received_global_plan`, `/transformed_global_plan`, `/local_plan`; no corresponding subscription or saved data events. Graph presence alone is not a subscription. Runtime controller frequency=20 Hz, failure_tolerance=0.3 s, active critics include BaseObstacle, planner=SmacPlanner2D. These runtime facts agree with the existing Smac diagnostic YAML, which differs from baseline only in planner plugin.

Whole-run TF ages (count/p50/p95/max, seconds): map→base **589 / .3525 / .405 / .5**; odom→base **589 / .35 / .405 / .5**. Fresh whole-run coverage: raw **217/590 (36.7797%)**, smooth **239/590 (40.5085%)**, final/SLAM/truth/wrench each **589/590 (99.8305%)**. Whole-run raw/smooth age p95=35.104/17.559 s is stale history after command cessation, not transport delay. Commands are unstamped; receipt-clock ages cannot prove transport latency or DDS losslessness.

The actual new-goal healthy interval is bounded by controller-start log seq6126 at **83.640** and first feasibility log seq14958 at **95.035**, end exclusive. It spans **11.399472971039359 wall / 11.395 simulation seconds**. Each of raw/smooth/final has **228 receipts**, hence **20.000924654959015 Hz wall** (20.00877577885038 Hz sim). Median wall interarrival ~.050 s; p95 raw/smooth/final=.052697/.053655/.052744 s; maxima=.060004/.059684/.060703 s. These are interval receipt rates compatible with 20 Hz, not exact publisher loop frequency. All six signals are fresh in **112/112 active aligned samples**. Active map and odom TF ages both p50=.36375, p95=.4025, max=.4275 s.

**Verdict:** observed pre-failure timing is healthy within the recorded measurements; no sustained callback backlog is indicated there. Absolute non-perturbation, individual DDS drops and command transport latency are not measurable from this recording. Later raw absence aligns with terminal FollowPath and smooth output ends when stopped, with separate recovery pulses; it must not be diagnosed as callback backlog from whole-run averages. A single desired-loop-rate warning at95.405 occurs during the failed retry, after the first controller abort. Early attach contains historical wall-stamped rosout replay; it is excluded from the new-goal interval.

## 2. Exact failure chronology and clock boundary

All table times are **simulation RECEIVE times** except the explicitly marked goal header and costmap/TF source stamps. Rosout headers are Unix wall time (e.g.1791960649.186666596), not simulation occurrence time. BT outer timestamps use simulation time; BT inner transition timestamps use wall time. Exact integer sec/nanosec fields remain in `chronology.json` and `bt-events.jsonl`; decimal wall values are rounded IEEE doubles. No fabricated wall→simulation conversion. Sequence numbers identify original `events.jsonl` records; samples have no seq, so their original line is given.

| Event | Receive time s | Original evidence |
|---|---:|---|
| `/goal_pose`, map goal(-4.1030473709,5.3277192116) | 83.590 | seq6060; header exactly83.590 |
| Old Navigate goal CANCELING then CANCELED | 83.5925 /83.610 | seq6064/6085; old UUID e49c97d6422543c987b3942c2d9dce73 |
| New NavigateToPose EXECUTING | 83.630 | seq6105; UUID ec4062a3fa4d444dbee8dac8d667639e, acceptance stamp83.630 |
| Controller begins new goal | 83.640 | seq6126; wall header1791960637.7820654 |
| First No valid trajectories out of419 | 95.035 | seq14958; wall header1791960649.1866665 |
| Raw first exact zero | 95.035 | seq14959; actual recorded zero, not imputation |
| FollowPath status ABORTED | 95.3375 | seq15195; UUID1e0ef65a5e89aea3ea008dc74812da50 |
| First Controller patience exceeded | 95.340 | seq15201; wall header1791960649.4871788 |
| FollowPath abort log | 95.3425 | seq15203; wall header1791960649.4873705 |
| Retry EXECUTING / controller-start log | 95.3525 /95.355 | seq15211/15214; new UUID5dca6ccb2f1c4ef42f507cf7cf3390dd |
| BT FollowPath FAILURE → local clear SUCCESS → RUNNING | 95.355 | seq15213; inner wall timestamps .501023898/.502690942/.502707547 after1791960649 |
| Retry still invalid, then resumes nonzero raw | 95.405–95.5025 /95.5575 | seq15253/15256/15303/15333; nonzero rawseq15375 |
| Next sustained invalid burst | 97.2575 | seq16697; wall header1791960651.4083370; rawzero seq16698 |
| Last raw receipt (exact zero); FollowPath status ABORTED | 97.555 | seq16931/16932; UUID0c6121428648deebe71aa6df3f747a8c |
| Second Controller patience exceeded | 97.5575 | seq16935; wall header1791960651.7082705 |
| FollowPath abort log | 97.560 | seq16937; wall header1791960651.7084280 |
| BT FollowPath failure; local/global recovery clear | 97.5675 | seq16945; wall1791960651.720926700/.721266203/.722246382 |
| Smooth exact zero / final nav_vel exact zero | 97.7875 /97.8225 | seq17110/17140 |
| First Starting point in lethal space | 98.0175 | seq17289; wall header1791960652.1732890; global snapshot98.015 seq17284 |
| Recovery Spin RUNNING → FAILURE; Wait RUNNING | 99.0375 →99.1675 | BTseq18037/18145 |
| Recovery angular pulse final nonzero → zero | 99.070 →99.1725 | seq18066/18150; smooth99.040→99.140 |
| Threshold-defined truth stationarity begins | 99.470 | samples.jsonl line234; truth eventseq18386 atstamp99.460; fresh truth age.010s |
| Further final recovery angular pulses | 107.065→107.165;115.0275→115.1275 | seq24439→24523;30791→30874; final remains zero after115.1275 through135.670 |
| Navigate recovery BT FAILURE | 120.9875 | seq35665 |
| NavigateToPose final ABORTED / abort log | 120.9975 | statusseq35674; logseq35675 wall1791960675.2009110; Goal failedseq35677 |

Raw cessation is bounded by the final received sample97.555 and no further receipts through observation end135.7, not a directly observed publisher destruction timestamp. Last raw motion precedes the second zero burst. Smooth includes behavior-server outputs during recoveries, so its later pulses are not renewed DWB raw commands. Cross-topic delivery reverses a few millisecond-scale status/log orders; do not reorder the source evidence or claim that ABORTED caused the preceding exception. Path preemption also terminates earlier FollowPath UUIDs during normal replanning, so an ABORTED status alone is not a controller failure.

Truth stationarity uses existing ≥3s/.04m/.1rad segmentation; it is a detected interval start, not an instantaneous measured contact/stop time. Intervals99.470–110.0325 and110.135–120.900 have truth extents .026667m/.099648rad and .025336m/.089582rad. Small rotation/jitter splits intervals. SLAM is also stationary in each; absolute truth versus map/odom coordinates are not interchangeable. There is **no recorded “Failed to make progress” log**.

Official [controller_server.cpp1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp), lines382–435 and465–506: each new execution resets progress checker and initializes last-valid time. Progress is checked before computing controller velocity, outside the tolerated controller-exception catch. A valid command refreshes last-valid time. A controller PlannerException with positive failure_tolerance produces a zero command during tolerance; elapsed time strictly greater than tolerance raises patience exceeded. The outer catch publishes zero, terminates the current action and returns. −1 tolerates indefinitely; zero rethrows immediately. Progress failure does not use this tolerance catch. Thus these are feasibility/patience aborts followed by retries/planner failures; the later stationary robot need not produce a progress exception after FollowPath has returned. This describes versioned source behavior, not unrecorded internal execution timestamps.

## 3. Frozen local/global geometry timeline

Each row uses its saved raw master bytes, exact-costmap-stamp recorded TF pose in the same frame, and the saved padded shape recovered from a published footprint with matched TF. No padding added. Local resolution=.05000000074505806m,100×100, frameodom. Global framemap; its full metadata remains in original source and raw hashes below. All selected footprint skew values are≤.32s. Shape is assumed constant within reported skew; exact recorded floating-point vertices, poses, source seq/stamp/skew, full hashes and filled geometry are in `geometry.json`. This is exact critic evaluation **conditional on those saved inputs**, not an instrumented copy of DWB's internal map/pose at each control tick.

BaseObstacle rejects center253/254/255 or off-grid. Filled polygon overlap is reported separately: 253 is an inflated inscribed zone, 254 a represented obstacle, neither a Gazebo contact measurement. All11 rows have **inscribed overlap=yes, unknown overlap=no, footprint outside-map=no**. The lethal-overlap column is raw254 filled-cell-area overlap.

| Map/header time | Center cell/cost | Base legal | Filled254 | Exact perimeter legal | First rejected perimeter cell(x,y,cost) |
|---|---|---|---|---|---|
| map 94.0200 | [386, 526, 113] | True | False | True | None |
| map 96.0175 | [375, 531, 220] | True | True | False | [366, 545, 254] |
| map 98.0150 | [364, 531, 253] | False | True | True | None |
| odom 93.5975 | [48, 53, 98] | True | False | True | None |
| odom 94.1975 | [47, 52, 119] | True | False | True | None |
| odom 94.7975 | [45, 51, 185] | True | False | True | None |
| odom 95.3975 | [47, 51, 239] | True | True | False | [38, 65, 254] |
| odom 95.9975 | [48, 51, 220] | True | True | False | [39, 65, 254] |
| odom 96.5975 | [46, 51, 204] | True | True | False | [35, 63, 254] |
| odom 97.1975 | [45, 49, 207] | True | True | False | [31, 39, 254] |
| odom 97.7925 | [47, 49, 253] | False | True | True | None |

| Map/header time | Pose x,y,yaw (rad; displayed6 decimals) | Footprint source seq/stamp/skew(s) |
|---|---|---|
| map 94.0200 | (-0.677866,6.344785,2.300560) | 14169 /93.7000 /0.3200 |
| map 96.0175 | (-1.204550,6.552581,2.610696) | 15738 /95.7000 /0.3175 |
| map 98.0150 | (-1.767604,6.570334,-3.047451) | 17282 /97.7000 /0.3150 |
| odom 93.5975 | (-0.573977,6.216462,2.062929) | 13990 /93.4000 /0.1975 |
| odom 94.1975 | (-0.726875,6.378444,2.424919) | 14457 /94.0000 /0.1975 |
| odom 94.7975 | (-0.976157,6.456366,2.729744) | 14926 /94.6000 /0.1975 |
| odom 95.3975 | (-1.176018,6.520304,2.675483) | 15405 /95.2000 /0.1975 |
| odom 95.9975 | (-1.201544,6.550187,2.610367) | 15875 /95.8000 /0.1975 |
| odom 96.5975 | (-1.362951,6.626794,2.787261) | 16335 /96.4000 /0.1975 |
| odom 97.1975 | (-1.606954,6.557539,-3.089106) | 16804 /97.0000 /0.1975 |
| odom 97.7925 | (-1.766079,6.567107,-3.035224) | 17265 /97.6000 /0.1925 |

Padded shape is approximately[(+.75,+.43),(+.75,−.43),(−.75,−.43),(−.75,+.43)]m, with per-record float32-publication recovery differences retained **without rounding** in JSON. Displayed pose rounding is not used by either geometry calculation or C++.

| Map/header time | Original metadata source | Raw SHA256 |
|---|---|---|
| map 94.0200 | `costmaps/0014172.json` | `56ea08bc43562a11e56d3b48de53ec4125c5c50ac7e97562f6e8d12223b7a79b` |
| map 96.0175 | `costmaps/0015737.json` | `2c65891847cd4b7c1c65a6764a4793af7221a3937812952d09a8bc1762bfe363` |
| map 98.0150 | `costmaps/0017284.json` | `c974c3909f1b8c55a3cfe4e7bb97b04e522410ffc41a7b896212fc6fae2f4f1d` |
| odom 93.5975 | `costmaps/0013835.json` | `67e431e6f20ba8822eff6f78e60dfe6cc8449647aad15b58beb40db045ddf592` |
| odom 94.1975 | `costmaps/0014306.json` | `0c1777973e7a84ed204d4a4fc936662c8b7fabd32c57bab671a57e41d0d096b1` |
| odom 94.7975 | `costmaps/0014765.json` | `7ceb7181d338f11664f51cd80d445b84d73b58dc5af586cc9d1e8e7239c75ead` |
| odom 95.3975 | `costmaps/0015245.json` | `0a9466d3bdc4b9a8fad9d03c6a9f14159fd149aab257f1efee01ca8f0cebc580` |
| odom 95.9975 | `costmaps/0015717.json` | `df76ccf5554b3b8b1f1499de536ef24ac38e8e99e5859f1cd08aa35b72876406` |
| odom 96.5975 | `costmaps/0016182.json` | `8288f35f1f7ecfbfb6f8c1eb665c2a9067b364fc64ba390dacd3dba08d66e3ba` |
| odom 97.1975 | `costmaps/0016649.json` | `10b1ddba85d13c941786759dd2ed1b8803df92e3e2d452a3c056a7bb8bc1a83a` |
| odom 97.7925 | `costmaps/0017116.json` | `5876204f095ca224d1172831a2c6355bdda89da35d6c8e08fd551539330d1a82` |

At global98.015, center(364,531)=**253**, while filled footprint overlaps raw254. This is consistent with Smac's subsequent lethal-start wording; the wording need not mean center254. [Smac2D1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/smac_planner_2d.cpp) configures radius checking; [GridCollisionChecker](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/collision_checker.cpp) rejects cost≥INSCRIBED(253), including unknown when disallowed; [AStar](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/a_star.cpp) reports a rejected starting node as lethal space. The map header is98.015; its receipt is98.0125(seq17284), and the log receipt is98.0175(seq17289): **5ms receipt gap**, versus2.5ms between map header and log receipt. Planner's actual start request/internal map at failure was not captured, so this is strong map-state consistency, not proof of byte-identical planner input. Local/global clears are recorded just before these snapshots; later costmap states must not be treated as static continuations of the first failure map.

## 4. Exact BaseObstacle versus ObstacleFootprint result

Tiny C++ harness links the installed `/opt/ros/humble/lib/libdwb_critics.so`, version**1.1.20**, SHA256`19286a7f26ccd47ba6da2690698a5b9d58a4169b5575853071f410df7b3a3ebe`. It constructs only an in-memory Costmap2D and critic subclasses exposing input setters. No rclcpp init, node, costmap ROS instance, plugin lifecycle, DDS or navigation process. It invokes installed BaseObstacle::scorePose and ObstacleFootprint::scorePose directly. The latter uses installed transform/worldToMap and perimeter edge order, closing edge included. An additional trace uses installed LineIterator and pointCost to report the first rejected cell; it does not determine the main legality verdict. Binary linkage is saved in `linked-libraries.log`.

The [official ObstacleFootprint1.1.20 source](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp), lines80–174, checks center bounds, then rasterizes the perimeter, rejects254/255 and accepts253. Interior cells are not inspected. [BaseObstacle1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/base_obstacle.cpp), lines76–96, checks center and rejects253/254/255. Tests exercise these actual installed calls, including interior lethal/unknown acceptance by perimeter, closing edge, yaw, off-grid and first-cell order. No filled-polygon result is substituted for critic legality.

The26 saved local maps between goal and99.47 were replayed; the four center-legal/perimeter-illegal observations occur at95.3975,95.9975,96.5975,97.1975. Each first rejected perimeter cell is254. Thus the pre-stall window contains four direct observations across1.8s; no unobserved continuous interval is interpolated.

**This failure contains direct map-space evidence of a footprint-validity mismatch between the active BaseObstacle approximation and exact ObstacleFootprint semantics.**

At94.7975 perimeter is still legal, and the first recorded feasibility failure is95.035, before the earliest saved mismatch95.3975. No critic rejection scores were subscribed in LIGHT. The first failing critic and its exact hypothetical rollout rejection are unresolved. At97.7925(and global98.015), center253 is invalid for BaseObstacle but the perimeter is legal despite interior lethal overlap. The swap is therefore not a universally more conservative collision predicate. This replay tests current poses only, not DWB rollout search, weighted selection, alternate trajectories, stopping dynamics, contact or action success.

## 5. Physical-contact evidence boundary

The user observed a stuck robot; thresholded truth confirms near stationarity after controller feasibility failures and command reductions. That does not measure a shelf contact pair or force. All5986 raw plate wrench samples have maxnorm**1.7125642478477905N**, far below50N; aligned589 samples max1.303778N. This is a small fixed-joint plate with inertial loads and limited coverage, so it neither proves nor excludes full-body contact. 253 overlap is inflation geometry;254 overlap is padded footprint versus mapped lethal cells; neither is physical contact. No body/leg collision-pair telemetry, physical aisle clearance or articulated swept envelope was measured. Map projection, TF/SLAM error, cell discretization and the configured footprint remain distinct from the actual moving body.

## 6. Why automated summary missed the precursor; fix

Original stationary candidates exist, but both have cases=[]. The first begins99.47; the old log search starts98.47. It excludes all feasibility logs(up to97.555), both patience logs95.340/97.5575, and first planner lethal-start98.0175. Raw is missing during stationarity; it is not a recorded zero. Mixed/mostly-zero final signals also do not satisfy existing A/B/C/D/E conditions. Original summary therefore failed to expose the precursor, rather than demonstrating absence of controller failure.

`precursor_chronology` now adds a separate six-second receipt timeline ending at stationary onset, same epoch, capped at first observed current Navigate UUID activation. Six seconds covers the observed4.435s failure-to-stationarity gap plus retry/braking, with explicit bounded horizon; it is a diagnostic bound, not a causal rule. It preserves exact eventseq, integer source sec/nanosec headers, decimal source_stamp_s, receipt clock, action acceptance stamps/transitions and offsets. At99.47: first feasibility−4.435s; first patience−4.130s; last raw(received zero)−1.915s; second patience−1.9125s; first planner lethal-start−1.4525s. Last raw was fresh when received; age1.915s makes it stale at stationary onset. No sample is filled with zero. If no valid raw is within the window, last_raw_command stays null.

Observer reads only the selected saved event topics **after detaching**, adding no live subscriptions/callback work. Existing `classify_interval`, stationary detector and A/B/C/D/E semantics are unchanged. Cases may still be[], now with explicit precursor evidence; first_causal_layer remains null. `extended-summary.json` demonstrates the extension on this exact original input, without overwriting summary.json. The second stationary segment's bounded precursor has later planner failures; it does not pull the original controller failure forward across an unbounded window.

## 7. Tests, reproducibility and integrity

RED: two new observer timing tests failed,30 old passed(`tests-red.log`); ten installed-harness tests failed before the harness existed(`critic-tests-red.log`). Added output-location guard and integer-header/goal-acceptance retention each have saved RED logs. Read-only review independently reran47 tests and found no critical/important issues; its two minor timestamp/header findings are corrected, with the new header regression included in the final48. GREEN/final:**48 relevant tests passed**, including actual compiled-library tests; no live ROS node. CMake C++17 compile with-Wall/-Wextra/-Wpedantic passes. Python in-memory compile, observer API/static scope and git diff whitespace pass. Offline diagnostic emits11 selected maps plus26 local-sweep snapshots. `static-integrity.json`: all270 protected tracked files unchanged, all2324 original evidence files unchanged, branch/HEAD unchanged. The three existing navigation YAML hashes are preserved. No policy/SLAM/planner/DWB limits/footprint/inflation/interface edits.

Commands(requires standard sourced Humble environment for installed DSOs):

```zsh
source /opt/ros/humble/setup.zsh
cmake -S scripts/frozen_obstacle_critic -B /tmp/b2w-light-causal-review/critic-build -DCMAKE_PREFIX_PATH=/opt/ros/humble
cmake --build /tmp/b2w-light-causal-review/critic-build -j2
/usr/bin/python3 -m pytest tests/test_warehouse_observer.py tests/test_frozen_obstacle_critic.py -q
/usr/bin/python3 scripts/diagnose_frozen_warehouse.py --source /tmp/b2w-warehouse-observer/20261014T065029.802588Z-smac2d-light-smoke --critic-binary /tmp/b2w-light-causal-review/critic-build/frozen_obstacle_critic --output /tmp/b2w-light-causal-review
```

Raw evidence is never regenerated or modified. The CLI rejects an output directory inside the original evidence. The build must be sourced; an initial unsourced attempt lacked ament's Python path, and an unsourced replay lacked dependency DSO paths. Both are offline environment failures, resolved by the documented source command; neither is a navigation result. No live process was started or signalled; synchronous compiler/test/replay subprocesses finished.

## 8. Implemented diagnostic: BaseObstacle plus footprint gate

Implementation provenance: branch `improve/narrow-navigation`, HEAD `945afc9e8af2d9782fe0e901dfed5624431e44f3`, equal to cached and live origin branch HEAD before implementation. The approved 155445 bundle was plan-only at `b30055eba0f576a6ad170baf9979ff60b12f5c18`; the user subsequently committed/pushed all four videos through Git LFS. This turn does not modify video content, LFS pointers or `.gitattributes`. Earlier HEADs remain historical evidence provenance.

`course_navigation/config/nav2_smac2d_footprint_gate_diag.yaml` now exists. It is byte-identical to `nav2_smac2d_diag.yaml` except these two inserted lines under FollowPath:

```diff
       - BaseObstacle
+      - ObstacleFootprint
       - GoalAlign
 ...
       BaseObstacle.scale: 0.02
+      ObstacleFootprint.scale: 0.000001
```

BaseObstacle remains at 0.02. Every old critic/order/scale, planner and Smac setting (including absent `cost_travel_multiplier`), motion/reverse/acceleration limits, both costmaps, footprint/padding/inflation and smoother are unchanged. Baseline and source SHA256 remain respectively `d63852ab2eed9c6cc1c258aaf2bdc2b11753ff9d9969e1885785d3cab2a87046` and `97e4eb2bcf58faa3f3a19728b4d17437b9dfe615d07964902e24bb5057a98e9e`.

Configured scale **s=1e-6 > 0** is required: DWB skips scoreTrajectory entirely when effective scale is zero, also skipping its legality exceptions. Installed `obstacle_footprint.hpp::getScale()` returns `resolution * scale_`; local resolution 0.05 gives effective scale **5e-8**. With unchanged default `sum_scores=false`, all sampled poses are checked and the last pose supplies the legal score. Maximum legal footprint score is 253, so maximum added legal contribution is **1.265e-5**. BaseObstacle maximum legal contribution is `252 * 0.02 = 5.04`; ratio **2.509920634920635e-6 <= 1e-5** (0.0002509921%). The allowed configured-scale ceiling is `3.984189723320158e-6`. This bound uses YAML resolution; a user trial must confirm actual runtime resolution. It is relative to maximum Base contribution, not the current trajectory's Base score, which may be zero. Near-ties and short-circuit effects remain possible; this is not a pure hard-only gate or tuned production weight.

Validation: approved new config regression first gave RED (2 missing-candidate failures / 3 passes), then the required three suites gave **53 passed in 4.59s**. The new test checks protected hashes, exact byte delta, restored-tree equality, order/scales, unchanged planner/costmaps/smoother, positive bounded scale and installed XML type resolution. Existing frozen installed-library semantic tests remain unchanged. A fresh temporary pluginlib probe under `/tmp/b2w-footprint-gate-implementation` compiled and created both `dwb_critics::BaseObstacleCritic` and `dwb_critics::ObstacleFootprintCritic` from `/opt/ros/humble/lib/libdwb_critics.so` with no rclcpp init, node, DDS or controller initialization. YAML/compile/integrity results are saved alongside the final bundle evidence. No Gazebo/Nav2 launch, goal or live parameter change; **no live result yet**. Exact user T1/T2 commands are in sole root `PLAN.md` section E, using LIGHT without `--dwb-debug`.

**AGAINST:** the added gate can refuse the aisle or turns; the four frozen mismatches do not demonstrate an alternate rollout. The first95.035 feasibility failure precedes the first saved mismatch. Added perimeter rasterization can affect timing; tiny positive scores can affect near-ties. Earlier critics and short-circuit scoring can prevent the gate from running on already discarded candidates.

**FOR:** retaining BaseObstacle preserves center253 rejection while adding the exact installed perimeter254/255 rejection demonstrated on four frozen inputs. The diagnostic isolates this approximation gap with a bounded legal-score contribution and unchanged planner, motion, geometry, mapping and tracking chain.

**NEUTRAL VERDICT:** implemented for a user-run diagnostic comparison only. No root cause, improved navigation, feasible alternative, physical contact or full-body safety is established. Perimeter checking does not fill the interior or measure moving-leg clearance. Reverse/abandon if aisle feasibility or timing degrades, the failure remains, or other critics/tracking/map/TF dominate; do not compensate with simultaneous tuning. Even a future successful passage alone cannot certify physical safety.

## 9. Final delivery

All changes and evidence are prepared before the final April bundle operation. Use the controlled copy removing only forbidden automatic-abort/exit lines from the user's bundler; preserve its collection logic and output exactly`/home/zyd/APRIL_REVIEW_BUNDLES`. Final absolute path/SHA256 are supplied by the final tool output and final response; then STOP, with no post-bundle edits, commit or push.
