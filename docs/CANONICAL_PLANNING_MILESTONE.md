# Canonical planning evidence milestone — 2026-10-25

Status: **canonical path evidence DONE; Owner execution FAILED; navigation and video uncompleted**.
This document closes the inherited implementation/evidence milestone. It does not
qualify motion or repair TF scheduling.

## Verified result and limits

One authorized, corrected-source planning-only Gazebo session on domain198 produced
one real `/planner_server` `/plan`:145 poses,7.212946685m, frame `map`, stamp24.5.
Attempt: `artifacts/trials/canonical-plan-deferred-20261025/planning-0de114ab06a44ac9a57fd28747b72f2d`.
SmacPlanner2D was ACTIVE with `GridBased.allow_unknown=false`. Owner FAILED at
clock24.5025, common TF24.0, age0.5025>0.5s; observer already had paired TF24.2.
The published path exists, but terminal ComputePathToPose result/status was not
retained. Do not manufacture action success or replace the raw Owner failure.

The initial rotation and0.5m path prefix clear the recorded map geometrically.
No physical stopping/tracking/localization envelope is qualified. Requested G1
padded footprint intersects184 unknown `/map` cells; bracketing global masters
contain no unknown/lethal footprint cells but four inscribed cells. Arrival and
rotation coverage remain unresolved. Historical0.064844853m G1 navigation arrival
passed Goal Manager0.3m tolerance and fails strict0.05m staging. Neither threshold
changed. G1→G2 and continuous G1→G5/video are not completed.

All642 Owner/613 observer final commands and1125 policy command triplets were zero.
Watchdog contained36 server threads and cleaned23 owned PID/start-time identities;
postflight domain, lease and GPU identities restored. No retries.62 original
capture files rehashed successfully when sealing the evidence.

Evidence is external and preserved at
`artifacts/recovery/final-canonical-planning-20261025/` and
`/home/zyd/APRIL_REVIEW_BUNDLES/APRIL_REVIEW_BUNDLE_b2w-embodied-navigation-course_20261025_final-canonical-planning.md`.
Bundle SHA256: `cee66344302175de133cd1f15ad9e90e4be2515bfc647e162e06606e94e991a7`.
It embeds selected original path/map/TF/costmap/config/fault records; full streams
remain local and are covered by its raw hash manifest. No raw evidence is added
to Git in this closure.

## Coherent source/config/test set

- `warehouse_ab_trial.py` and `run_warehouse_ab.sh`: bounded owner, explicit planning-only
  branch, precheck/route separation, identity checks and one-shot accounting.
- `warehouse_ab_supervisor.py`, `owner_tf_delivery.py`, existing `narrow_processes.py`:
  owned containment and callback diagnostics. No new watchdog or TF framework.
- `warehouse_route_admission.py`, `warehouse_operational_footprint.py`: fail-closed
  current/swept footprint and reviewed directional-bound contracts. No qualified
  default envelope is shipped.
- `run_warehouse_route.py`, `record_warehouse_video.py`: existing sequential runner
  extended to explicit G3–G5 headings and recording readiness; no validated G3–G5
  incoming paths or successful uncut route video exist yet.
- Profile B changes only DWB `max_vel_x`0.4→0.2 versus profile A; Smac+DWB+rl_sar
  preserved. Inherited velocity-adapter explicit-zero response and batched pose TF
  changes are retained with their tests; public frames and estimates are unchanged.
- Three inherited `stage4_contact_*` Python modules and `scripts/contact_observer/`
  are actual imports/build dependencies of the owner, despite their legacy names.
  Preserve them; no new Stage4 infrastructure was created. `engine_probe.cpp` is an
  exact source copy of the existing ignored construction probe, SHA256
  `3af8b83ed625606d4f4eb602321eb4e908f7b78463369f7f314effa73a18d358`.
- Focused tests include admission/rejection, planning isolation, callback ordering,
  map bootstrap, contact pairing, one-shot authority, ownership/cleanup and route
  staging. The small Fortress command-line inspection fixture is source test data.

## Tests and review

On this prepared workspace, ROS Humble sourced with the existing isolated install:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR="$PWD/artifacts/recovery/canonical-planning-close-20261025/test-ros-log"
/usr/bin/python3 -m pytest -q \
  tests/test_warehouse_ab_trial.py tests/test_warehouse_ab_supervisor.py \
  tests/test_warehouse_precheck_only.py tests/test_warehouse_truth_admission.py \
  tests/test_warehouse_readiness.py tests/test_owner_dispatch.py tests/test_owner_tf_delivery.py \
  tests/test_map_bootstrap.py tests/test_motion_readmission.py tests/test_first_motion_control_risks.py \
  tests/test_warehouse_route_admission.py tests/test_warehouse_operational_footprint.py \
  tests/test_warehouse_route.py tests/test_warehouse_video.py tests/test_warehouse_observer.py \
  tests/test_warehouse_route_overlays.py tests/test_velocity_steps.py tests/test_velocity_calibration.py \
  -k 'not test_real_ros_executor_services_runner_and_recorder_zero_only'
ctest --test-dir artifacts/recovery/contact-build --output-on-failure
```

Result: **367 passed,1 deselected**; existing native observer test **1/1 PASS**.
Fresh offline preflight also verified46 source/install pairs and26 collision
handles, with `no_world_or_motion_started=true`.
The initial broader selection had367 passes and one environment failure opening
read-only `~/.ros/log` in the DDS executor integration test. It is explicitly
excluded above; no claim of full-suite or live scheduling validation is made.
No Gazebo, physical motion, new build or feature development occurred at closure.

Self-review (single agent, no independent reviewer): **PASS for milestone archival
and handoff**, not for navigation readiness. Reviewed tracked diffs and every
selected dependency group, planning-only call graph, missing-envelope fail-closed
behavior, unchanged precheck disk/pulses, owned fault/cleanup paths and staged file
inventory. No unresolved finding prevents this evidence/source checkpoint;
known runtime TF failure and external reproducibility limits remain explicit.
Owner SHA256 remains `749a41692b21ff843e77a96003404f54a2fd8ac59b3132a6e1e641e701432fed`;
watchdog remains `ccbf17372337f8c8f511031d4e1eb7ca844a038c326b3771b96a9e528cf4613e`.

## Reproduction inputs and next work

Git carries source, configuration, tests and concise documentation, not generated
assets or recorded trials. A clean clone alone cannot run every evidence-dependent
test or this identity-pinned owner. Follow CODEX_GUIDE for ROS/assets/install;
recover/review generated `robot.urdf`, `robot.sdf`, `collision-inventory.json`,
contact plugin, construction probe/result, `root-installed-manifest.json` and the
reviewed deployment manifest under local `artifacts/`. Native/source paths in the
manifests bind the existing checkout; never edit hashes merely to pass preflight.
Tests also use original `.before` source snapshots in
`first-motion-control-risks-20261025/`, `first-motion-validation-20261025/`, and
raw `owner-dispatch-precheck-20261025/` and `motion-continuous-20261025-1/` records.
These remain external. Keep this prepared workspace for the next session.

Offline input check (no simulator, ROS node, goal or velocity):

```bash
ROS_DOMAIN_ID=198 scripts/run_warehouse_ab.sh --profile B --planning-only --check \
  --deployment-manifest artifacts/recovery/first-motion-control-risks-20261025/reviewed-deployment.json
```

NEXT: diagnose Owner TF/Odom ingestion/executor scheduling using captured trace;
distinguish ingress from callback delay, service coherent localization snapshots
without changing0.5s freshness, and prove fail-closed behavior/owned cleanup with
focused tests. Do not resume startup-order patches, use observer truth as control,
relabel unknown, or invent stopping bounds. Any later live validation requires new
explicit authorization, fresh domain/lease/output, identity verification and the
existing <=420s watchdog. Motion calibration, route trials, hardware and video
execution require their own scope/authorization; none is inherited from closure.
