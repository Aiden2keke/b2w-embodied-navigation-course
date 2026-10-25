# Oct 18 adaptive-v2 false navigation success

Investigation only; no production code/configuration change. Starting HEAD
`7eb8c685d13cc53af0a2987fa625b55a8949da00`; inherited dirty checkout preserved.
Evidence: `/tmp/b2w-adaptive-v2/route/20261018T135709.017672Z/` and sibling
`metrics/`, `ros-log/`. Investigation evidence and offline probe:
`/tmp/b2w-false-success-investigation/` (included explicitly in review bundle).

## Finding and confidence

A concrete upstream Humble defect is present in Nav2 1.1.20 and the currently
installed controller library: `ControllerServer::isGoalReached()` ignores failure
of the map-path-endpoint to odom transform. The failed output remains the default
pose, position (0,0,0), orientation (0,0,0,1). `SimpleGoalChecker` can accept this
as arrival near the odom origin, even while the real map goal is metres away.
This mechanism is reproduced against installed libraries without ROS init,
nodes, executor, DDS, simulation or movement.

This is the strongest explanation for this incident, not a complete replay of
its internal state. The decisive log pair at wall time 1792331837 is:

```text
.604420109 tf_help: Transform data too old when converting from map to odom
.604450659 tf_help: Data time: 57s 707500000ns, Transform time: 57s 100000000ns
.604480640 controller_server: Reached the goal!
```

The age is .6075 s against configured .5 s transform tolerance. About 30 us after
the second error, the controller declares success; BT reports success at
1792331837.663609031. Current library disassembly confirms the transform call
at 0x94446 flows to the checker call at 0x944a6 without testing its boolean
return. No binary identity from the incident itself was retained; current apt
packages report 1.1.20. All 11 source hashes in the run manifest match the
current repository files, including goal_manager, profile, launch and recorder.

Not observed: the exact odom-frame robot pose at this goal check, current
check_xy_ state, and incoming path endpoints. A bad endpoint or localization
jump cannot be excluded from the missing data. The stale-TF condition is proven;
its producer-side cause (SLAM latency, scheduling or another issue) is not.

## Exact action and path chain

1. Runner sent G1 in `map`: (-0.18571805953979492, 7.189985752105713),
   yaw -2.8966601449198057; quaternion z=-.9925103760697063,
   w=.12216035933955931; stamp 55.0275 s. The report contains only this sent goal.
   BT logs navigating from (-.34,-.00) to (-.19,7.19).
2. `goal_manager.py:110` assigns that message unchanged to NavigateToPose.pose.
   It also sends a separate ComputePathToPose request (GridBased, use_start=false)
   for latency measurement. That result's path is discarded; it is not the BT's
   path and its success/29.14 ms latency does not prove the followed endpoint.
3. NavigateToPose stores the incoming pose in the goal blackboard. Repository
   configuration does not override the default BT XML; installed
   `navigate_to_pose_w_replanning_and_recovery.xml` plans at 1 Hz and passes
   `{path}` to FollowPath. There is no SmoothPath action in this tree.
   The historical BT parameter/XML were not captured at runtime.
4. Runtime parameter evidence and planner startup log confirm
   SmacPlanner2D, tolerance .30 m, allow_unknown=false. Smac uses the global
   costmap frame (`map`), converts search cells back to world coordinates,
   applies its **internal** smoother, then restores the requested final
   orientation by default. A tolerance endpoint may differ from the exact
   requested XY; the actual endpoints/lengths were not saved.
5. Smac's holonomic smoother updates interior points only; endpoints remain
   fixed. Its fallback returns the previous path on time/iteration/collision
   failure. A launched standalone `simple_smoother` server is not proof of use.
   Velocity_smoother processes commands and does not smooth geometric paths.
6. FollowPath uses RPP. ControllerServer copies the full input path's last pose
   into end_pose_, assigns the path header frame, and resets the goal checker
   on each new path. New paths are logged at 1833.45, 1834.51, 1835.56,
   1836.60 and 1837.60. Its local costmap frame is `odom`.
7. RPP transforms/prunes a local portion into `base_link` for tracking and
   publishes `received_global_plan` (a local transformed/pruned path despite
   its name). That does not replace ControllerServer's stored full endpoint.
   RPP only reads checker tolerances; the server performs terminal validation.
8. SimpleGoalChecker uses XY .25 m and yaw .30 rad in the on-disk profile;
   these particular runtime values were not dumped. It ignores velocity and Z.
   With stateful=true, passing XY latches check_xy_=false until reset. Every
   replacement path resets it, so cross-path state retention alone is not an
   explanation. A failed transform can itself establish an incorrect XY latch.

## Decisive source excerpts

References are pinned to the installed package's release, not current main.
Full downloaded files and hashes are supplied with the review evidence.

[ControllerServer](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp#L543-L560):

```cpp
nav_2d_utils::transformPose(...);  // boolean return ignored
return goal_checkers_[current_goal_checker_]->isGoalReached(
  pose.pose, transformed_end_pose.pose, velocity);
```

The ellipsis/comment abbreviates the source arguments; it is not a proposed edit.
The same source assigns `end_pose_ = path.poses.back();` at line 419 (one-based).

[tf_help](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/nav_2d_utils/src/tf_help.cpp#L55-L83):

```cpp
return false;
```

This executes after logging the age violation and before the fallback
`tf2::doTransform(in_pose, out_pose, transform);`. Installed geometry message
headers initialize Quaternion.w to 1.0; this is not an invalid all-zero quaternion.

[SimpleGoalChecker](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/plugins/simple_goal_checker.cpp#L91-L111):

```cpp
if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
  return false;
}
```

It then latches XY when stateful, and tests yaw. The offline probe verifies both
fresh-state acceptance at (.1,0), yaw=0 against the failed default endpoint,
and a false XY latch persisting to a later yaw-only success until reset.
It also verifies rejection of the true distant G1, an outside-radius query,
and the proposed fail-closed boolean guard. This is an isolated library defect
reproduction, not an exact replay or a deployed controller fix.

[Smac planner](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/smac_planner_2d.cpp#L254-L298):

```cpp
_smoother->smooth(plan, costmap, time_remaining);
plan.poses.back().pose.orientation = goal.pose.orientation;
```

[Smac smoother](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/smoother.cpp#L128-L158):

```cpp
for (unsigned int i = 1; i != path_size - 1; i++) {
```

That loop excludes both endpoints; no truncation is demonstrated by this code.

## What the retained measurements mean

Nav2 status=4 is recorded, but goal_manager.success=false and practical
advancement stopped at G1; no goal validation was bypassed. The goal-manager
metric requires action success AND distance within .30 m. End TF is null because
its .5 s freshness check failed. `final_goal_distance_m=7.185598121916984` uses
the last available measured map position when the end snapshot is unavailable;
it is not a fresh terminal position measurement.

`path_length_m=.1189416942051028` accumulates localized robot displacement in
RunMetrics.pose; it is **not planned path length**. The evidence index explicitly
identifies `00-Start-G1.path.json` as sampled map→base_link TF. Its 15 poses span
54.7–56.7 s, from (-.344096,-.000256) to (-.268316,.006152).
The world-frame truth trajectory has 28 poses, 55.03–57.73 s. Neither file
contains planner output. Never compare world XY directly with the map goal
without registration. Start TF age .33 s was fresh; that does not guarantee
freshness throughout the action. Action/controller elapsed time is ~5.2 wall
seconds; the metric's elapsed simulation time is 2.71 s.

## Review decision and smallest missing observations

No runtime change is shipped. The defect is in the installed external Nav2
controller, which this repository does not vendor/build. Changing goal tolerances,
stateful mode, frames or TF tolerances would mask the failure rather than repair
that unchecked return. A focused upstream/overlay repair is to return false
before invoking the checker when transformPose fails. The probe demonstrates
that guard, but a deployable overlay and its integration are outside this
qualified incident attribution; review those separately before another live run.

For a next independently authorized run, retain only the missing boundaries:

- Record `/unsmoothed_plan` and `/plan` with original headers and all poses,
  ComputePathToPose/FollowPath UUIDs, plus loaded BT XML/hash and checker/frame
  parameters. Keep goal_manager's extra latency request distinguishable from
  BT requests. `/plan` alone does not identify the FollowPath action input.
- Add bounded, diagnostic-only controller records on setPlannerPath and on
  failed/successful goal checks: active UUID/path sequence, path and last-pose
  headers, first/last XY/yaw, pose count, computed planned arc length; odom-frame
  query pose/time; transform boolean and output; latest map↔odom TF stamp and
  transform, checker ID/check_xy_ before/after and result. Capture atomically
  inside the controller; external TF snapshots cannot prove these arguments.
- Preserve `/tf`, `/tf_static`, `/clock`, `/slam/odom` over the short attempt
  with receipt times, so localization evolution and scheduling can be checked.
  Record binary SHA256/build IDs and package versions before starting.

No retries, goal sends, extra action requests, forced transforms or validation
bypasses are part of this telemetry proposal. Canonical route/headings, footprint,
collision safety, practical advancement and adaptive speed behavior are untouched.

## Offline validation and cleanup

- Installed-library C++ probe: CMake build and execution pass within 90/90/10 s
  limits, including all assertions described above. Its prepopulated Buffer emits
  a dedicated-thread warning on the first extrapolation lookup (no listener is
  running), then reproduces the exact too-old data/transform timestamps. No
  threaded listener is needed for this deliberately immutable synthetic input.
- Final focused suite: **125 passed in 1.44 s**, bounded by 90 s, using
  `/usr/bin/python3 -m pytest` with ROS and `install/setup.bash` sourced and
  ROS_LOG_DIR under the investigation directory. Covers metrics/math, readiness,
  practical route, adaptive geometry/node/launch/state and recording tests.
- `scripts/build_course.sh --packages-select course_navigation course_bringup
  --symlink-install`: **2 packages finished in 1.93 s**, bounded by 120 s.
- Initial `scripts/test_course.sh` invocation also collected the entire tests
  directory because that script always supplies it: 692 passed, 3 skipped,
  13 failed, 6 errors. ROS logging attempted the sandbox-read-only ~/.ros/log;
  context-init errors followed. A focused retry with writable logging but only
  the ROS underlay had 119 pass / 6 launch failures (course_sim package missing
  from ament index). The final correctly sourced focused run passed. Those
  initial logs are retained; the full suite is not claimed passing.
- All 44 retained raw files hash identically before/after; only PLAN.md and this
  new document changed during this investigation. All inherited production/test
  files match their pre-documentation hashes. There is one NOW in PLAN.md.
- All owned foreground probe/compiler/test/build commands have ended. The broad
  initial test collection attempted isolated ROS context initialization; this
  was not a navigation or simulation run. No Gazebo or live goals were started;
  no persistent process was created to clean up. No unrelated process is killed.
- Final delivery is one normal april-review-bundle including original logs,
  offline probe, downloaded source, binary disassembly and integrity manifest.
  STOP for independent review; no commit, push or live navigation.
