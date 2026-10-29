# Recording → navigation checkpoint, 2026-10-28

User acceptance of the Gazebo overview and RViz top-down LiDAR/map appearance
is accepted session input. `radar.webm` and `warehouse.webm` are actual rendered
footage. No further camera design or stationary recording prerequisite is needed
to investigate controller qualification. This does not assert synchronized,
complete, decodable ten-trial recordings. No approval artifact was generated.

## Saved RViz follow

`artifacts/video/manual_views/g1_g5.rviz` now uses Fixed Frame `map`, Current
View `TopDownOrtho`, Target Frame `base_link`, Angle 0, Scale
42.2995730952694. Scale/angle come from the actual latest retained working
`20261028T103930Z-ac6d8126/attempt_01/radar.rviz`. X/Y offsets are zero to center
the target rather than keep the previous map-center offset (.55679,3.59499).
All other original display settings are preserved. The actual presentation
generator was exercised and preserves these values. Three Gazebo camera input
files have identical before/after SHA256 values.

This moves the viewing origin, while map data remains transformed in fixed
frame `map`; it does not publish/change TF or re-register the map. Live robot
centering and map registration during motion are **pending**: the read-only host
check found no running Gazebo/RViz. No simulator was launched merely for a view
check. Verify centering and static rack/wall alignment during the next admitted
diagnostic, using fresh map→odom→base_link and paired truth for evaluation only.

## Actual admission chain and blockers

| Boundary | Concrete requirement / present state | Treatment |
| --- | --- | --- |
| `run_g1_g5_record.py:read_review` | Missing externally reviewed `reviewed_run.json` + digest, deployment/envelope/control/recording artifacts, seven automation source pins | Collector gate. Do not fabricate; first control diagnostic does not use collector. |
| Same, control review | Retained heading, bidirectional turning, small-command tracking, Profile-B directional stopping, balance/stop and independent review tied to raw data/deployment | Real controller evidence missing; retained heading actually FAILS. |
| Same, full-route inputs | Three finite G3–G5 arrival headings | Needed before full route only; unnecessarily early for Start→G1 or G1→G2. Derive/check against incoming path, do not invent. |
| Same, camera/recording review | Exact saved camera/RViz hashes, separate implementer/reviewer identity, actual synchronized video and re-decode | Recording-specific. User has already accepted appearance. Do not require new aesthetics approval for control work. Changed RViz hash cannot inherit old integrity pins silently. |
| `warehouse_ab_trial.py:main` | Reviewed changed controller permitted only in normal stationary or historical-yaw mode, fixed spawn (0,-1); historical-yaw ≤.5 sim s | Explicit motion veto. Blocks sustained test and **all routes** with this revision. No alternative entry or old manifest used to evade it. |
| `validate_controller_revision`, `check_inputs` | Exact binary/native/source/install/config pins, pinned same-deployment stationary/rest evidence, engine/contact identity | Safety/integrity requirements; retain. Old qualification cannot silently authorize modified harness/controller. |
| `load_envelope` | Profile-B [.2 m/s,.6 rad/s] command caps, deployment/model identity, footprint ≥(.7,.38), padding ≥.05, nonzero localization allowance, measured forward/reverse/lateral/yaw/rotation-drift bounds and raw evidence | Real missing operational envelope. Single .02694 m coast is not a conservative bound. Include smoother/recovery reachable commands or constrain them through reviewed configuration. |
| `route_start_admission` | Fresh idle unique source, valid live planner result and ≤.5 m initial prefix, whole-cell swept rectangle/full initial yaw sweep and directional stop margin, observed-free map | Real geometry/readiness check. Historical G1 footprint has 184 unknown map cells and four inscribed master cells; historical planning success is not clearance. Fresh canonical Start/map correspondence and arrival/sweep coverage required. |
| Runtime owner | Fresh TF/odom/clock/truth, full collision observer identity/coverage, support/balance, command/physical caps, independent watchdog, measured rest and exact owned cleanup | Preserve all vetoes. Diagnostic runtime PASS is not useful turning PASS. |
| Progressive execution | Owner has only `critical` (G1,G2) and `full` (G1…G5); collector always full. G1 success immediately proceeds to G2; measured owner rest is after route | Missing safe Start→G1-only checkpoint and explicit per-waypoint settled-stop proof. Existing arrival snapshot/action success is insufficient for requested stopping evidence. Add bounded prefix selection and settled-stop evidence before progression, only after controller qualification. |

The inherited Stage-4 candidate implementation in the original workspace is a
different path/selection contract. Its geometry results are not admission for
this canonical route. Direct invocation of the generic route runner would omit
the current owner gates and is not an acceptable workaround.

Fresh same-controller `.5 s --check` with the original motion deployment manifest
also rejects: `Controller revision source/config pins changed or incomplete`.
All 46 source/install pairs and nine native files still match; four of five
revision source pins match. The changed file is `scripts/warehouse_ab_trial.py`:
expected `170bcb3ca6d391f76d2fdc2c0983fe6cb5f89db14dd7ed487162e7b5264f1d5f`,
actual `569813eedc023859bf8905ea93cb4479d515af414d24633a05875cd67c1bacc7`.
This pre-existed this continuation (no harness code was edited here). Review
the inherited harness changes together with the proposed duration admission;
do not merely rewrite the old manifest and call it reviewed. Exact audit is
`course_results/imported_evidence/navigation-progress-20261028/deployment-identity.json`.

## Fresh offline measurements from retained raw data

Recomputed from domain225 `observer/.../events.jsonl`, SHA256
`aa7f3822712d3ed9666954b3fa3313f9b6b932bc0f75141daaef23e0e4d3daf2`.
Script and complete results: `course_results/imported_evidence/navigation-progress-20261028/`.

| Measurement | Result |
| --- | ---: |
| External command / internal policy yaw command | +.25 / +.400000006 rad/s |
| Owner pulse / received final-command interval | .500 / .455 sim s |
| Received external command integral | .113750 rad |
| Truth turn in owner interval / at received zero | .081080756 / .083010780 rad |
| Truth yaw at final rest relative to start | .012269420 rad |
| Owner-window retention / delivery-aligned retention | 15.1323% / 14.7805% |
| Received-zero→final-rest rebound | −.070741361 rad |
| Received-zero→qualified-rest start | 1.350 sim s |
| Whole-stop translation / rest-window translation | .0269383 / .0035140 m |
| SLAM/truth relative heading discrepancy, 68 timestamp-matched samples | max .000319600 rad |

Rest 27.1725…32.395 sim s includes measured LIO/truth rest and six-axis zero.
The retained runtime report has no contact/readiness/truth fault, normal watchdog
shutdown and successful empty owned cleanup. Its `success:true` is runtime safety
success; the predeclared ≥.04 rad and ≥50% retained-yaw criterion FAILS.

Prior actual IMU/torque analysis additionally shows one valid heading reference,
IMU/truth difference ≤.001149 rad, late heading error .070741 rad and restoring
torque .565942 Nm/wheel, matching K=8, D=.5. Added torque does not reach .75 Nm;
no 20 Nm wheel clamp saturation. Thus observed failure is physical body-heading
rebound with insufficient closed-loop heading retention, not display rotation,
missing command, lost reference, reversed feedback or localization-only motion.

Idle handling zeros learned wheel velocity targets while learned leg balance
remains active; the added feedback acts on wheels. Stance/contact coupling or
short-pulse elastic deformation is plausible, but contact load/lateral slip and
a matched sustained response are missing. These data do **not** identify a unique
plant cause, prove wheel saturation, or justify a sign/gain/cap increase. The
previous frozen-state PI replay is not physical evidence; do not deploy it.

## Smallest discriminating next experiment

ONE same-controller +.25 rad/s sustained turn, nominal **1.0 sim s**, then exact
six-axis zero until original qualified rest and normal cleanup. No forward
pulse, negative pulse, route goal, retries, gain/cap/policy change or hardware.
Use the same (0,-1) spawn and fresh known-free 1.5 m pocket. Doubling duration
extends beyond the .5 s transient while expected .16–.23 rad yaw remains below
the original .35 rad excursion veto; this is a design estimate, not a safety
guarantee. Original physical yaw ≤.4 rad/s, speed ≤.25 m/s, displacement ≤.25 m,
tilt/height/contact/freshness/watchdog and whole-stop limits remain authoritative.
Keep the existing 10 wall-second pulse deadline and original owner/watchdog bound.
If a veto fires, contain and classify failed/blocked; do not retry shorter.

Record owner, adapter and model onset/zero separately; commanded yaw integral;
truth/IMU/SLAM unwrapped relative yaw; last-half turn rate; peak, at-zero and
settled yaw; both retention definitions; zero-to-rest time and full stopping
translation/yaw; wheel positions/torques, leg stance and contact evidence.
Retain ≥.04 rad / ≥50% functional floor; do not lower it after observing results.
A one-sign pass only admits further qualification, never the route envelope.
Missing steady rate makes the test inconclusive, not PASS.

If sustained yaw also mostly rebounds, target coordinated stance/body-heading
control during active→stop transition, with measured wheel/contact authority;
wheel-only proportional hold is demonstrably inadequate in the tested state.
If sustained yaw retains while short pulses rebound, target transient/stop
coordination and characterize minimum useful turn duration/small commands before
Nav2. Neither outcome justifies blindly adding integral or raising torque caps.

**Execution status: BLOCKED / NOT RUN.** An actual offline 1.0 s invocation was
rejected by the unchanged parser: `historical-yaw requires an explicit
--precheck-pulse-s at most .5`. User conditionally authorized one safe diagnostic,
but also prohibited bypassing motion vetoes. No override or alternate mode was
used. Next executable checkpoint is review of a narrowly scoped sustained-test
duration admission with all original physical/watchdog limits unchanged and
matching harness/deployment pins; then run that one test. No new permission to
redesign cameras or collect stationary videos is needed.

After independent controller and route PASS, implement/verify G1-only arrival
and settled-stop checkpoint, then G1→G2, then G5, retaining actual waypoint
telemetry. Only after those pass use the existing deterministic ten-result
collector and success/failed/blocked classification. Route and batch attempts
this continuation: **zero**. No GitHub push, approvals, new locomotion patch or
recording-framework development.

## Verification boundary

Saved-config YAML and the actual `rviz_config()` output were checked for exact
frame, class, scale, angle and offsets; original displays and all Gazebo camera
hashes are preserved. Relevant recording, geometry, readmission and precheck
tests: **155 passed in 13.46 s** under the sourced ROS environment. Initial
unsourced invocation failed on missing `rclpy`; both logs are retained. Offline
duration rejection and source-pin rejection are preserved as failures. These
checks grant no physical PASS. Live centering, sustained turning and all routes
remain unverified in this continuation.
