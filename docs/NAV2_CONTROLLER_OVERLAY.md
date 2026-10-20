# Nav2 Humble 1.1.20 endpoint-transform guard

Review candidate only. No simulation or live goals were run. Independent review
is required before a separately authorized live run. This repairs the confirmed
unchecked-transform defect; it does not establish complete incident attribution.
See [the retained investigation](ADAPTIVE_V2_FALSE_SUCCESS_2026-10-18.md).

## Source and minimal change

Source: [Bloom release/humble/nav2_controller/1.1.20-1](https://github.com/ros2-gbp/navigation2-release/tree/release/humble/nav2_controller/1.1.20-1).
The 17,446-byte release archive SHA256 is
`bac776b1b5e5cc1dd6365aec941d71d19e1a06fa5ad5d5ee84f299b05d0a6235`.
Its controller source matches the reviewed upstream 1.1.20 file byte for byte;
all six public headers, package.xml and plugins.xml match the installed package
`1.1.20-1jammy.20260915.232401`. The preparation script rejects other versions
and archive hashes. The archive is cached in the ignored overlay workspace.

Only production change, in `ControllerServer::isGoalReached()`:

```cpp
  if (!nav_2d_utils::transformPose(
    costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),
    end_pose_, transformed_end_pose, tolerance)) {
    return false;
  }

  return goal_checkers_[current_goal_checker_]->isGoalReached(
    pose.pose, transformed_end_pose.pose,
    velocity);
```

The full original package is built against the existing Humble underlay with its
original CMakeLists, headers, plugins and dependencies. No apt install, upgrade,
/opt edit, system library or shell configuration change is involved. The tracked
patch is `patches/nav2_controller_1.1.20/endpoint-transform.patch`.
Smac2D, RPP, adaptive logic, footprint, route positions/headings, goal_manager
acceptance and all tolerances are untouched. No diagnostic code was added.

## Reproduce offline

From the repository root in Bash (scripts also run directly from zsh):

```bash
scripts/prepare_nav2_controller_overlay.sh
# Or supply the pinned archive as the single argument for an offline prepare.
tests/nav2_controller_guard/run.sh /tmp/b2w-nav2-guard-recheck
ROS_LOG_DIR=/tmp/b2w-nav2-guard-logs \
  scripts/with_nav2_controller_overlay.sh \
  /usr/bin/python3 scripts/check_nav2_controller_overlay.py
```

Preparation requires a fresh `src/nav2_controller` tree and refuses to overwrite
an existing one. To rebuild already prepared source without downloading:

```bash
(
  source /opt/ros/humble/setup.bash
  cd .deps/nav2-controller-1.1.20
  CMAKE_BUILD_PARALLEL_LEVEL=2 /usr/bin/python3 /usr/bin/colcon build \
    --base-paths src --packages-select nav2_controller \
    --allow-overriding nav2_controller --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DPython3_EXECUTABLE=/usr/bin/python3
)
```

The regression extracts and compiles the complete production method without
rewriting its body. A test-only class supplies robot pose, costmap getters and
zero odometry; transformPose and SimpleGoalChecker are real ROS libraries.
There is no ROS init, node, executor, listener, DDS, lifecycle activation or motion.
This is a method-level boundary test, not a full controller lifecycle test.

Expected RED against the original release: exactly three failures: false goal
success near odom origin, checker invocation after transform failure, and false
XY latch. Expected GREEN: all 11 assertions pass. Successful nonidentity transform
to a distant goal rejects; legitimate transformed arrival succeeds; wrong yaw,
same-frame arrival and missing robot pose retain their original behavior. The
synthetic stale transform emits expected tf_help errors at 57.7075 versus 57.1 s
with unchanged .5 s tolerance. Tests use explicit checks, unaffected by NDEBUG.

## Activation and rollback

Activation is process-scoped, via `scripts/with_nav2_controller_overlay.sh`.
It loads the underlay, existing course install, then the overlay **last**, and
checks the selected ament prefix. The parent shell remains unchanged.
The wrapper provides the same course/DDS/library settings as run_course.sh.
For a future independently authorized launch, the exact entry is:

```bash
scripts/with_nav2_controller_overlay.sh ros2 launch course_bringup course.launch.py \
  mode:=navigation scene:=warehouse gui:=true rviz:=true
```

Supply the previously reviewed adaptive arguments unchanged when applicable.
Do not wrap `run_course.sh`/`run_warehouse.sh` or source their setup afterward:
they reload prefixes. The command above bypasses warehouse asset preparation;
existing assets must already be present. It was **not executed** in this task.

Rollback: stop that opt-in launch normally, then use the ordinary
`scripts/run_warehouse.sh`/`scripts/run_course.sh` in the unchanged parent shell.
No uninstall or environment editing is necessary. Optional artifact removal,
after all overlay processes have stopped, is limited to the owned
`.deps/nav2-controller-1.1.20` directory. Keep it for review/reproduction for now.
A fresh baseline shell sourcing only `/opt/ros/humble/setup.bash` and
`install/setup.bash` resolves the original package and core library in
`/opt/ros/humble`; this was checked offline.

## Validation evidence and limits

Evidence directory: `/tmp/b2w-nav2-guard-review/` (explicit logs in review bundle).

- `red-green-reproduction.log`: RED 3 expected failures, GREEN 11 passing checks.
- `reproducible-build.log`: fresh preparation from the pinned archive and isolated
  package build; `course-build.log`: course_navigation/course_bringup build.
- `offline-tests.log`: 125 focused project tests passed (math, readiness, practical
  route, adaptive node/launch/geometry/state and recording).
- `runtime-resolution.json`: the final opt-in wrapper resolves ROS launch's
  ExecutableInPackage to `.deps/nav2-controller-1.1.20/install/nav2_controller/lib/nav2_controller/controller_server`.
  The executable's ELF interpreter `--list` resolves its core to the overlay
  `lib/libcontroller_server_core.so`. RTLD_NOW maps that same library in a process
  and does not map the original core. No controller main() is executed.
- ABI checks: all six public headers and package/plugin metadata are unchanged;
  all 67 existing strong package function/data and RTTI/vtable exports across six
  libraries are preserved. Each library adds a harmless RTTI type-name export.
  SONAME/NEEDED sets match; `ldd -r` finds no unresolved libraries/symbols.
  This is compatibility evidence for the installed underlay, not a claim that
  rebuilt binaries are byte-identical to apt binaries or live-tested.
- `patched-method-disassembly.log`: built method tests transformPose's boolean
  before reaching the checker. `integrity.json`: inherited files and retained raw
  evidence unchanged; apt controller verification and owned-process check logged.

The initial whole-Nav2 source download timed out and was discarded as an input;
the complete pinned Bloom package was used instead. Initial verification attempts
found a missing CMake library search hint, duplicate source discovery during
manual staging, an incorrect colcon setup path, and read-only default ROS logging.
These were corrected in the final scripts; retained logs are not live-run evidence.

## Separate unresolved issue: map→odom staleness

The incident includes map→odom data .6075 s old against .5 s tolerance. Its producer
cause remains unresolved (SLAM timing, scheduling and localization evolution were
not sufficiently recorded). This guard prevents failed endpoint transforms from
being treated as arrival; it does not repair TF freshness, guarantee navigation
progress, or change existing progress/cancellation behavior. Retain the original
investigation's bounded TF/path telemetry proposal for a separately authorized
run. No tolerance increase, forced transform, retry or acceptance bypass is used.


## Explicit external Stage4 deployment (NEEDS FIX continuation)

The experimental worktree does not restore `.deps` or `.venv` links. The opt-in
wrapper/checker accepts absolute `B2W_CONTROLLER_PREFIX` (the copied installed
nav2_controller package), `B2W_COURSE_UNDERLAY`, optional `B2W_NATIVE_LIB_DIR` and
`B2W_PYTHON_SITE`. Historical local `.deps` fallback remains only for an explicitly
prepared local build. Missing prefixes fail; no download/install occurs. Runtime
Stage4 checks use the explicit `--deployment` JSON, not that fallback: exact pinned
executable/library hashes, PID/start ticks/PGID/UID/token, executable target and
mapped core path/inode/device are required. Guard manifest digest and loaded adapter
parameters must match. Offline loader proof does not qualify a running controller.
