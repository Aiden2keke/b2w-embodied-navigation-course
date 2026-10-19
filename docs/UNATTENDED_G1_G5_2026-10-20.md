# Unattended G1–G5 campaign: blocked before motion

Outcome: **BLOCKED_GEOMETRY**, with additional unresolved registration and physical
safety/evidence gates. **No FULL_ROUTE_PASS; no Gazebo stack or motion trial was
started.** No measured waypoint passage, physical path length, speed, contacts or
final rest can be reported. Live-stack budget used: **0 / 21,600 wall seconds**;
motion experiments: **0 / 10**; controller families tried live: **0 / 3**.

The user authorized unattended bounded Gazebo execution and provisional local
checkpoints. Motion was conditional on every applicable gate. The mandatory
corridor has no justified uncertainty envelope under the current saved map;
Task 2 explicitly permits this preliminary geometry stop before E1. Controller
tuning, route shaping and alternate controllers cannot resolve that missing
geometric safety certificate. There was no live infrastructure retry.

Worktree: `/tmp/b2w-unattended-worktree-20261019`, experimental branch
`experiment/unattended-g1-g5-20261019`. The original `improve/narrow-navigation`
checkout and all its inherited dirty files are preserved. `PLAN.md` in the
experimental branch is the current roadmap. Evidence root:
`~/b2w-course-evidence/unattended/runs/20261019T154246Z/`.

## Preservation and checkpoints

The pre-edit baseline contains all 388 tracked/nonignored files, including the
tracked historical videos, and the complete dirty patch. Every archived file
was compared with its SHA256 manifest before changes. The source archive SHA256:
`bf39a5d42926b301c3e0a6ff17a2c481cd842301f91a53c2f7f95c82f043c979`.
The policy/model, source map, selected V1, original Nav2 profiles, canonical route,
LIO-SAM source and prior Stage 4 evidence remain protected. Existing temporary
Stage 4 and Astra evidence were archived durably before further probes.

Local commits are **provisional pending later independent review**:

- `f9591a1`: exact inherited adaptive, Stage 4, endpoint-guard and diagnostic
  dependencies, separately checkpointed; 208 baseline regressions and two package
  builds passed. Its inherited whitespace warnings were recorded and retained
  to preserve exact bytes; this is not a claim that inherited Stage 4 was safe.
- `8c1c454`: source/receipt/clock watchdog and structural-map cadence repairs.
- `8457e4e`: bounded geometric interval audit and reproducible map report.
- The handoff commit is enumerated in the evidence root's `checkpoints.json`;
  no push, merge, reset or system-wide change occurred.

## Independent audit of Astra's findings

| Finding | Fresh evidence / disposition |
| --- | --- |
| F1: 2 s structural-map freshness vs 0.2 Hz producer | Confirmed in LIO-SAM and evaluator. Fixed the production transport/evaluator contract: source age 7.5 s; wall budget max(7.5, 1.5 × maximum of two startup periods), at most 20 s, after three distinct publications. Missed publications, duplicates and rollback reject. Fast pose/TF limits remain .5 s. A 30 s nominal 5 s publication regression now passes. No live-startup qualification claimed. |
| F2: stale cached TF and duplicate odometry renewal | Fifteen original failing real-callback regressions reproduced the faults. Fixed final adapter checks for both map→odom and odom→base_link, normalized source stamps, finite pose/twist/quaternion, frames, distinct odometry, monotonic receipt time, clock suspension and latched rollback. Preserved the existing unlocalized sim_world body-state mode. Additional review regressions caught malformed nanoseconds and stale-source rollback before their fixes. No policy change. |
| F3: full-body map conflicts | Newly compiled installed Smac 1.1.20 probe exactly reproduced 128 G1→G2 and 127 G3→G4 blocked samples. New interval certificates exclude the entire mandatory cross-section for an additional 7.5 mm rectangular half-extent envelope. Nominal connected route remains unresolved. |
| F4: physical acceptance unavailable | Independently generated current torque-mode robot description has 22 collision links and no contact sensors. Plate box is .95×.45×.02 m. This is source-generated inventory, not verification of an unlaunched spawned model. Existing transport lacks independent truth and full contact coverage; runner still supplies safety_complete=False. Unresolved, no motion. |
| F5: near-identity verification and weak local error bound | Source confirms manual landmarks, translation ≤.05 m/yaw ≤.02 rad and one-way occupied p95 ≤.10 m. No automatic same-session registration or sub-centimetre local uncertainty proof exists. No justified source-map projection defect or independent map/world evaluation transform was established. Unresolved. |
| F6: early future-region entry and missing connections | Fresh hypothetical (0,0) full-plan evaluation reports 12 early future-region samples, despite all five planned regions being reached. Measured Start is unavailable. Initial yaw/connection and final braking safety remain unqualified. Existing phase restrictions and exact goals preserved. |
| F7: predictive filled-body safety / stopping | Pinned DWB profile contains BaseObstacle, without a predictive filled-interior/swept footprint critic. Source preflight and retrospective checks are insufficient. No measured campaign braking/creep bound exists. Unresolved; no controller tuning attempted. |
| F8: evidence capacity | Real ROS serialization of two 800×800 grids gives 640104 and 640120 CDR bytes; base64 alone is 1,706,968 bytes/s at 1 Hz each. 256 MiB lasts 157.2586 s before other traffic/metadata. Production callback explicitly reproduced raw_evidence_limit. Chunked recorder not implemented after the geometry stop. |
| F9: asynchronous samples and cleanup | Production evaluator rejects fresh interleaved TF/old-but-still-fresh odom as clock_reset_or_duplicate. Source confirms separately queried latest TFs, prior-sample overwrite and one shared 5 s cleanup budget without continued graph checks. Runtime occurrence rate unknown. Coherent assembler/cleanup repair remains unimplemented. |

The synthetic serialization benchmark's pair encoding median was approximately
3.85 ms; its observed maximum was 35.2 ms. These are offline empty-grid encoding
measurements, **not** runtime observer or 20 Hz controller deadline validation.

## Geometry result and its limits

The unchanged padded rectangle is **1.50×.86 m**. Source map image SHA256:
`e90f2dd81c89d41828ddb9f60732bdcae5cc7526dcf4a8919638a335d4a585f3`.
The exact selected G1/G2 positions put every continuous ordered traversal across
map **x=-2.5 m**. The full perpendicular .35 m capsule at that x permits
**y=[6.1359044672, 6.8405017072] m**. This fixes the audit's slightly undersized
vertical strip. The endpoints' entire .25 m acceptance disks lie on opposite
sides of the slice. Future-region exclusions can only reduce this searched set.

Closed-cell rectangle/cell SAT checks filled interiors, edges, unknowns and map
bounds. A conservative interval method covers all y and all distinct rectangle
yaws (π symmetry), using sin/cos Lipschitz bounds. Each excluded interval records
its collision-cell index and numeric bounds. For the 7.5 mm envelope, all **72
leaf boxes** have occupied-cell witnesses (cost 254), with complete domain area
and independent polygon checks at each box's corners/center. No unknown cell
was converted to free in the final audit. The 10 mm envelope is also excluded.

**Astra's zero sampled poses at 5 mm is not continuous nonexistence.** Refinement
found a source-grid-clear 5 mm pose at approximately
`(-2.5, 6.683790748827164, 0.04391020005322648 rad)`; westward equivalent yaw is
that value minus π. This is one cross-section pose, with **no connected,
forward-trackable route, initial connection or braking proof**. It must not
be used as a motion instruction.

Consequently, this audit bounds a possible additional rectangular envelope below
7.5 mm; it does not prove the physical warehouse impossible or nominate 7.5 mm as
an adequate safety allowance. Map boundary, registration, tracking XY/yaw,
latency, body variation and braking terms are all unmeasured for this campaign.
The entire existing .05 m padding is retained; no term silently spends it again.
Without a justified combined bound, the G2 gate is unresolved and motion stops.
The generated plot was visually inspected for map orientation and gate placement.

## Planning metrics and all experiment outcomes

These are newly reproduced **offline** planning metrics for hypothetical Start=(0,0):

| Segment | Center path found | Blocked padded samples / checked |
| --- | --- | --- |
| Start→G1 | yes | 0 / 385 |
| G1→G2 | yes | 128 / 309 |
| G2→G3 | yes | 0 / 211 |
| G3→G4 | yes | 127 / 439 |
| G4→G5 | yes | 0 / 368 |

Combined gate: 255 blocked samples, 12 early future-region samples; **FAIL**.
Runtime path/waypoint/stop metrics: unavailable, since no motion was authorized
by the gates. No video or trajectory demonstration is claimed.

| Planned live experiment | Outcome |
| --- | --- |
| E0 two passive fresh starts | Not started: preliminary geometry and physical/evidence gates unresolved |
| E1 open-pocket calibration | Not started: plan requires stopping before E1 when preliminary geometry cannot certify a plausible envelope |
| E2 Start→G1→G2 | Blocked by G2/G3/G4/G5 |
| E3 isolated G3→G4 | Not started |
| E4 three fresh full routes | Not started; 0 / 3 accepted |
| E5 fallback comparison | Not justified: no certified feasible path/tracking failure |

Offline failures are retained, including: initial prefix-mismatch regressions
(184 pass/4 fail); missing-underlay build; watchdog RED (15 fail/3 pass);
ROS_LOG_DIR sandbox error (62 pass/6 fail); structural RED (5 fail); geometry RED
(6 fail, absent module); simulation-mode compatibility RED (1 fail); final review
RED (4 fail/26 pass); coarse unresolved 5 mm search; broad loopback sandbox denial.
These are development/probe attempts, not hidden motion experiments.

Final validation: **441 distinct offline tests passed across the selected suite
and two scoped rechecks**. The broad restricted-sandbox run had 439 passes, one
loopback socket denial and one optional preserved-V1 skip. The localhost-only
test subsequently passed outside that restriction; the skipped V1 integration
passed with its actual backup path supplied. The repaired-contract subset had
129 passes. Six geometry tests pass. The native endpoint-transform guard's
expected RED/GREEN regression passed. Two isolated course packages rebuilt.
No Gazebo/ROS node, sensor freshness, physical clearance, real callback timing or
runtime library activation is certified by these offline checks.

## Reproduction and minimum next action

From the experimental worktree, use a **new** output directory each time:

```bash
cd /tmp/b2w-unattended-worktree-20261019
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/b2w-review-mpl \
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 scripts/audit_unattended_geometry.py \
  --candidate /home/zyd/b2w-course-evidence/waypoints/20261019-v1/candidate-16nlistv/candidate.yaml \
  --map /home/zyd/b2w-course-evidence/waypoints/20261019-v1/warehouse.yaml \
  --output /tmp/b2w-review-geometry-new
# Expected exit 2: BLOCKED_GEOMETRY, gate_G2_passed=false.

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/b2w-review-ros
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest \
  tests/test_navigation_watchdog.py tests/test_structural_map_cadence.py \
  tests/test_route_geometry.py tests/test_stage4_route.py \
  tests/test_stage4_transport.py -q -p no:cacheprovider
```

This is an offline reproducer, not a completed live demonstration entry point.
The `.deps` symlink in the experimental worktree references the original pinned
build dependencies and is not committed. Its original-prefix overlay wrappers
are not qualified for live use from this worktree. The durable review bundle
contains the branch changes, commits, inputs, reports, test/build/failed-probe
logs and manifests; the complete baseline archive is separately hash-referenced.

Minimum unblock: obtain independently sensor-supported local corridor/map
registration and boundary uncertainty, and establish whether the narrow raster
conflicts are faithful physical geometry. If a projection defect is established,
correct it in a derived, provenance-bound map and repeat the unchanged geometry
checks. Otherwise a connected path plus a defensible combined error/braking
budget below the certified upper bound is required. Existing coarse identity
limits cannot establish that. After geometry is plausible, finish F8/F9,
automatic registration, predictive filled-body control, full contact/evaluation
coverage and five-second final-hold acceptance before E0/E1 and route motion.
A changed physical passage, smaller footprint, changed goals or policy is outside
this campaign; no such change was made or assumed.
