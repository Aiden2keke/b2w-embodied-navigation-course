# Manual Nav2 admission audit

This session did not enable or authorize manual navigation. The only user
launcher remains `scripts/run_warehouse_manual.sh`. It still launches the
zero-motion viewer; changing it to navigation without the protected execution
owner would bypass the current safety boundary.

## Exact blockers

- The launcher selects `mode:=slam control_source:=none`; Nav2 and the goal
  manager are absent.
- `goal_manager.py` sends NavigateToPose and ComputePathToPose independently.
  Its computed path is only a planning metric. It does not authorize the path
  actually executed or subsequent replans. `require_bound_execution()` in
  `scripts/warehouse_route_admission.py` rejects this gap.
- The Stage4 manifest launch requires `goal_manager_enabled:=false`. There is
  no qualified owner binding RViz goals to that protected execution chain.
- `velocity_adapter.py` calls `require_motion_qualification()` for a nonzero
  guarded output. `stage4_qualification.py` unconditionally raises
  UNQUALIFIED_FOR_MOTION: predictive veto, action-independent owned abort,
  sensor-only bounded registration, and all-body/end-watermarked evaluation
  remain unqualified. Identity checks and zero-motion observations do not
  satisfy this contract. See `STAGE4_RUNTIME_READINESS.md`.

Enabling execution requires a qualified owner and either exact checked-path
execution or reviewed continuous clearance coverage of the actual changing
Nav2 execution, plus fresh deployment, localization, contact, stopping and abort
evidence. No flags, substituted paths or generated approval documents were used.

## Fresh identity and build checks

The existing pinned Nav2 controller overlay was rebuilt successfully (30.3 s).
Extracting the release archive, applying the repository endpoint-transform patch
with zero fuzz, and comparing the entire resulting source tree to the deployed
overlay source returned diff exit 0. Rebuilt executable/library SHA256 values
remain identical to `course_navigation/config/stage4_runtime_identity.json`:

```
40653e793f7707537e41940fd280d2abf6054a61ffcbb4a085e07ba591d137ee
388ba725baa46fc07699de44c1945e7056a9c0a04dc0a8fdad55055798ad4ad6
```

The overlay wrapper resolves nav2_controller to this repository's
`.deps/nav2-controller-1.1.20/install/nav2_controller`. No safety identity pins
were rewritten. This is disk/build verification, not live mapped-library
verification: no protected navigation controller was running.

Endpoint-transform regression reproduced three failures in upstream and zero
in the patched source. The five scoped admission/guard/deployment test files
passed: 104 tests in 3.12 s using the installed overlay and writable ROS logs.
Earlier sandbox/environment failures are retained and are not motion evidence.

## Host observation limits

The unchanged launcher ran on the Gazebo host, then received Ctrl+C. Session
`course_results/tracks/20261101T160222Z` exported trajectory.csv, trajectory.png
and result.json with motion_enabled=false, 324 TF samples and a received map.
Localization net displacement was 0.00196891 m; accumulated TF variation was
0.04501645 m. These are stationary localization variations, not traveled
distance. No navigation goal was dispatched and no physical motion measurement
was established.

Two independent initialization probes received no ROS samples and failed.
The desktop capture did not establish warehouse/camera visibility. Therefore
warehouse visibility, RViz goal acceptance, physical travel and a traveled-path
PNG remain unvalidated. Raw logs and test/source comparison outputs are retained
under `course_results/manual-nav2-audit/`. No camera changes, automatic routes,
recording, new worktree or GitHub push were made.
