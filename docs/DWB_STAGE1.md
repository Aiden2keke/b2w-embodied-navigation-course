# Warehouse Stage 1: original DWB profiles and bounded failure evidence

Offline preparation only. No Gazebo, live observer, ROS goals or route experiments
were run. This does not establish navigation progress, physical safety, overhead
transparency or a repaired corridor passage. The sole active milestone is
[PLAN.md](../PLAN.md). Prior adaptive/RPP and endpoint-transform-guard work is
preserved independently; this feature neither enables either nor alters Nav2.

## Explicit selection and rollback

The existing `nav_params_file` selector remains the profile selector:

| File | Global planner | FollowPath | Footprint diagnostic |
|---|---|---|---|
| `nav2.yaml` (default) | `nav2_navfn_planner/NavfnPlanner` | `dwb_core::DWBLocalPlanner` | Original BaseObstacle chain |
| `nav2_smac2d_diag.yaml` | `nav2_smac_planner/SmacPlanner2D` | Identical original DWB | Same original chain |
| `nav2_smac2d_footprint_gate_diag.yaml` | SmacPlanner2D | DWB | Retains BaseObstacle and adds positive-scale ObstacleFootprint |

The first two differ only in the planner plugin; their DWB, costmaps, padded
footprint, limits, smoother and goal checks are identical. The Smac file retains
the historical NavFn-only `use_astar` key; Smac2D does not consume it. Installed
Smac `cost_travel_multiplier` default remains 1.0, with no new tuning.

`dwb_observability:=true` appends `nav2_dwb_observability.yaml` **only** to the
controller_server parameters, after the selected file. Its entire delta is:

```yaml
FollowPath.debug_trajectory_details: true
FollowPath.publish_evaluation: true
FollowPath.publish_trajectories: false
```

ROS flattens nested parameter dictionaries and the last file overrides matching
parameter names. The overlay does not replace the FollowPath configuration.
Critics/order/scales, sampling, acceleration limits, collision predicates,
local/global costmaps and command routing stay unchanged. The option rejects
RPP/custom non-DWB controllers and `adaptive_speed:=true`. No governor or adapter
SpeedLimit subscription is added.

Future USER-RUN selection, **after independent review and separate live-run
authorization**, using the existing prepared warehouse environment:

```bash
scripts/run_warehouse.sh adaptive_speed:=false dwb_observability:=true \
  nav_params_file:="$PWD/course_navigation/config/nav2_smac2d_diag.yaml"
```

For the independent safety diagnostic, select
`nav2_smac2d_footprint_gate_diag.yaml` instead. Do not treat its stricter gate as
part of original DWB or remove it to claim a guarded pass. Roll back by normally
stopping the owning launch, then omitting `nav_params_file`, observability and
adaptive options. That restores default NavFn+DWB. No file restoration or package
uninstall is required. This stage does not change the original YAML files.

## Installed API and offline verification

Checks target installed Humble **1.1.20**, not current Nav2 main:

- [DWB local planner](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp)
  declares `.debug_trajectory_details`; on all-invalid failure it logs each
  first-throwing critic/reason and its reported fraction. Failed evaluation
  entries retain the full rollout and the throwing critic with raw score -1.
  Earlier scores are discarded by that exception path. Best/worst indices do
  not establish a winner when every trajectory is invalid.
- [DWB publisher](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_core/src/publisher.cpp)
  declares `.publish_evaluation` and `.publish_trajectories`. The installed
  `dwb_core/publisher.hpp::shouldRecordEvaluation()` returns their logical OR.
  Evaluation publication needs a subscriber; disabling markers alone does not
  disable evaluation construction. No marker subscription is added here.
- Installed library strings contain all three parameter names. The offline C++
  probe uses pluginlib to instantiate NavFn, Smac2D, DWB and the independent
  footprint critic, then destroys them without ROS init, nodes or DDS. It also
  exercises the installed IllegalTrajectoryTracker reason/fraction API.
- Tests call the real launch setup and Node parameter expansion, then parse its
  ordered parameter files with installed rclcpp/rcl_yaml_param_parser. They do
  **not** execute Node actions or any launch service.

Reproduce only offline checks from a Bash shell at the repository root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/b2w-stage1-check-logs /usr/bin/python3 -m pytest -q \
  tests/test_dwb_stage1_launch.py tests/test_dwb_failure_window.py
```

The tests build `scripts/nav2_stage1_probe` into a temporary directory and load
only installed libraries. No policy or system package is rebuilt. The standalone
probe also accepts `--plugins` and `--parameters NODE YAML [YAML ...]`.

## Bounded attach-only recording

Future USER-RUN, from the same sourced domain/DDS environment as the independently
authorized DWB session; this command sends no goals and changes no parameters:

```bash
/usr/bin/python3 scripts/observe_warehouse_narrow_failure.py \
  --dwb-failure-window --label canonical-G1-G2 --duration 120 \
  --window-before 3 --window-after 2 \
  --window-memory-mib 64 --window-disk-mib 256
```

The first received nonempty evaluation whose candidate totals are all negative
triggers one capture. Later failures do not extend it. Native log arrival order
can differ from evaluation order. If no such evaluation arrives, the final rolling
prehistory is retained with `trigger=null`; no failure is inferred from missing
data. LIGHT remains the default; legacy `--dwb-debug` remains separate and still
omits nonwinning predicted poses. The two debug modes are mutually exclusive.

Defaults and hard bounds:

| Resource | Default / behavior | Maximum |
|---|---|---|
| Receipt window | 3 s before / 2 s after first observed failure | 10 s each |
| Observer duration | Smaller of requested duration and 120 wall s; zero also means 120 | 120 s, checked between callbacks |
| Encoded retained records | 64 MiB, including conservatively double-charged context references | 256 MiB |
| Retained record count | 4096, including context anchors | 8192 |
| Raw ROS message | 4 MiB, checked before deserialization | Fixed in CLI |
| Encoded record | 16 MiB | Fixed in CLI |
| Capture directory | 256 MiB, including 1 MiB reserved for manifest | 1024 MiB |
| DDS history | BEST_EFFORT, KEEP_LAST depth 1 per selected topic | Drops are possible |

Count/byte pressure evicts old records or stops capture; oversize messages are
explicit gaps. A disk-limit stop never silently continues with a summary-only
capture. All locally detected drops and incomplete prehistory invalidate
`retention_complete`. Transport completeness stays **unknown** even if no local
drop is detected. Unusually large final manifest detail is omitted with an
explicit `manifest_detail_limit` gap; original raw JSONL remains intact.
The byte limit bounds retained encoded data, not total process
RSS; deserialization, JSON conversion, ROS middleware and Python have extra
working memory. The manifest records process CPU time, max RSS and wall duration.
No live timing/non-perturbation guarantee is made. Only the observer's resources
are destroyed; no external processes are signalled.
The disk budget covers this capture's two files. Native Nav2/ROS logs follow the
existing launch lifetime and are outside that budget; roll back the diagnostic
profile when the run ends.

Two new files are created in a unique run directory, never overwriting evidence:

- `failure-window.jsonl`: complete serialized ROS **CDR** as base64, its SHA256,
  complete decoded source message, topic/type, sequence,
  monotonic/UTC nanoseconds, exact latest `/clock` stamp and epoch. Original ROS
  integer stamps stay in each source message. Nonfinite JSON floats use explicit
  tags; the authoritative CDR preserves the original value.
- `failure-window-manifest.json`: limits, trigger, received counts, drops,
  retained older-context sequences, integrity hash, read-only parameter snapshots,
  selected publisher graph, resource measurements and observer cleanup state.

Complete LocalPlanEvaluation records include every candidate velocity, pose,
time offset and score plus best/worst indices. Native rosout retains per-critic
rejection reasons. A parsed reason is a convenience field on that **log record**;
it is not fabricated inside the evaluation or attributed to individual candidates.
Invalid-entry scale zero can be a default field in the native exception message;
consult the parameter snapshot for the configured critic scale.

The same window retains published local/global costmaps, footprints, raw TF,
global/debug plans, three command stages, action states, SLAM, evaluation-only
truth, wrench and available policy debug records. Last-seen static TF, plans,
goals and costmap/footprint messages may be retained as bounded older context,
with their original time. Clock resets retain explicit epochs;
they do not justify cross-epoch geometric reconstruction.

These are **publicly observed** messages. No controller-private costmap lock,
input pose, thresholded twist, critic state or TF buffer is captured. There is
no exact synchronized controller-state reconstruction or proof that a published
costmap equals the bytes used for a particular evaluation. Installed Humble rclpy
discards MessageInfo before invoking subscriptions: per-message publisher GID is
therefore null, while graph endpoint identities are recorded separately. Shared
smoother/behavior-topic source attribution remains unavailable. Missing publishers,
stale context, dropped messages and absent rejection reasons remain limitations.

## Acceptance wording for later stages

Practical arrival and optional precision are separate results. Require genuine
Nav2 terminal success, goal_manager validation, usable fresh endpoint/TF evidence,
the frozen intended waypoint geometry/headings and retained endpoint error.
Evaluate the optional 0.05 m target separately. Keep the historical strict
benchmark and its pass/fail fields unchanged; do not relabel practical completion
as a strict pass or treat missing endpoint evidence as acceptable.

Retain the endpoint-transform false-success protection and review its opt-in
environment before a later live run. This Stage 1 activates no controller overlay.
Collision safeguards remain intact; zero plate detections alone cannot certify
absence of full-body/leg contact. No new G1/G2 route acceptance is claimed here.

## Stage 1 verification evidence

`/tmp/b2w-stage1-review/` contains the before-state hashes, effective launch
configurations/library identities, native plugin probe results, stage-only diff,
RED/GREEN and final regression logs, build log and integrity checks.

- Final bounded offline regression: **251 passed in 17.09 s**, covering isolation,
  actual launch and installed parameter parsing, CDR roundtrip, 419-candidate raw
  retention, native executor dispatch, integer timestamps/epochs, pre/post bounds,
  context retention without GID, size/count/disk limits and manifest pressure.
- `course_navigation` and `course_bringup` build: both finished in **1.93 s**.
- Six baseline/Smac/gated launch combinations with observability off/on pass.
  Installed 1.1.20 NavFn, Smac2D, DWB and footprint critic instantiate offline.
- HEAD/branch remain unchanged. All 565 fingerprinted original evidence files
  and nine protected external policy/library artifacts match their before-state;
  inherited source changes outside the four intentional edit sites are unchanged.
- Initial unsourced-overlay checks failed to locate course packages; sourcing
  the existing install resolves that environment failure. Raw logs are retained.
- Internal review found and closed the Humble callback and manifest-sizing
  defects. Independent review of the final bundle is still required. No live
  observer attachment, Gazebo, goals or route experiments were performed.
