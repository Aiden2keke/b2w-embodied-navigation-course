# Oct 18 adaptive Start→G1 diagnosis — STOP for review

Historical diagnosis-only milestone. The later user-authorized control repair
is documented in [ADAPTIVE_STATE_REPAIR_2026-10-18.md](ADAPTIVE_STATE_REPAIR_2026-10-18.md).
Release rules and next-run commands below describe the former implementation;
use [ADAPTIVE_SPEED.md](ADAPTIVE_SPEED.md) for current behavior and review gating.
The raw-run findings, including the separate G1 residual-motion/heading issue,
remain evidence of that run.

This diagnosis uses only the retained route/metrics, ROS logs, current source
and PLAN.md. It does **not** establish a repaired physical run. No Gazebo,
calibration, goals, commits or pushes were executed. The motion controller and
configuration were not tuned: the retained evidence does not establish a safe
motion/configuration defect fix. Confirmed misleading governor diagnostics were
fixed, with control-output equivalence checked offline.

## Evidence identity and preservation

Primary run: `/tmp/b2w-adaptive-route/route/20261018T080730.741736Z/`;
metrics: `/tmp/b2w-adaptive-route/metrics/20261018T080732.191832Z.json`;
controller log: `/tmp/b2w-adaptive-route/ros-log/controller_server_190243_1792310798810.log`.
The earlier 08:00Z run is preserved separately; it is not pooled into statistics.
Raw files were read only. All derived analysis/validation is under
`/tmp/b2w-oct18-review/`; `raw-sha256.json` fingerprints **all** retained route,
metrics and ROS log files, and `preservation.json` verifies unchanged bytes.
`analyze.py` reproduces the numerical analysis; `analysis.json` has full results.

The route ran approximately 16:07:32–16:09:10 local, sim 45.6825–123.0075.
G1 alone was sent. Report end_reason is **shutdown**, Nav2 terminal status is
null, and the navigator log says its goal was still executing at deactivation.
There were repeated controller sub-action aborts, not a demonstrated terminal
NavigateToPose failure before shutdown. The report's `navigation_failed` is the
runner classification, not a Nav2 error code. Cleanup was unconfirmed in the
original run; this diagnosis does not rewrite it as successful cleanup.

## Why OPEN never appeared

All **988** speed_status samples say NARROW: OPEN 0%, NARROW 100% by count and
covered simulation time. Sample reasons: curvature 417 (42.21%), release_hold
399 (40.38%), geometry 88 (8.91%), stale_or_missing 84 (8.50%). Left-continuous
simulation-time integration within the navigation interval covers 77.260 s of
77.325 s (last status precedes shutdown); respectively 32.805 s (42.46%),
31.180 s (40.36%), 6.775 s (8.77%), 6.500 s (8.41%). These are decision-time
fractions, not time spent physically inside geometrically defined corridors.

Loaded governor `w_ref=1.0`; its provenance as a measured corridor width is
**unavailable**. Do not assume that parameter is a verified physical measurement.
The source's release rule is:

```python
release = (not turning and width > cfg.w_ref+cfg.width_margin_m+cfg.release_margin_m
           and min(left, right) > cfg.min_side_clearance_m+cfg.release_margin_m)
# Here: width > 1.30 m AND both footprint-relative sides > .35 m for 2 sim seconds.
```

Start yaw was essentially 0, while G1 lies predominantly along +map y: the
initial bearing from the recorded start is about +1.55 rad. NARROW during
initial alignment is expected. The first healthy status (45.805 s) has width
3.108 m, left .256 m, right 1.909 m, curvature .597/m but reason `curvature`.
Because curvature is below .8/m, the **heading-error** branch fired. Even after
alignment, a large total width does not establish bilateral clearance.

Only **15 of 399** release_hold records actually satisfy release thresholds;
all occur at 54.3925–55.4475 s, with the longest eligible held interval **1.125 s**
including the final held sample to the next status. This is shorter than 2 s.
The other **384** are inside the hysteresis margin and never start/continue the
release timer. Mid-route the small side frequently remains .15–.35 m, despite
reported total widths around 2.5–3.2 m. There is no retained sustained, aligned,
fresh, bilaterally clear interval that the implementation incorrectly refused
to release. Lowering release clearance or deleting the hold would be tuning a
safety choice, not fixing a demonstrated bug.

The 904 geometry-valid statuses have width median/p95 2.682/3.259 m,
left .209/.401 m, right 1.463/2.000 m; minimum width/clearances are zero.
Curvature median/p95/max .502/1.403/2.602 per metre. Every horizon is 1.0 m:
recorded speed never makes the braking formula exceed its minimum lookahead.
Geometry samples include current pose and future path footprint envelopes;
these are not measurements of robot-center lateral wall distance alone.

904/988 statuses are healthy. The 84 unhealthy statuses expose missing/old path
in 79 and old odom/localization TF in 5 (same 5 also costmap TF). There are no
cloud-age or map-age threshold violations, and no >.5 s tick stalls. During
navigation, source-age median / p95 / max (simulation seconds) is:

| Source | Median | p95 | Maximum | Existing limit |
|---|---:|---:|---:|---:|
| Raw costmap | .283 | .560 | .640 | 1.5 |
| RPP path | .433 | 1.360 | 5.513 | .75 |
| SLAM odom | .405 | .500 | .728 | .6 |
| Localization TF | .408 | .500 | .728 | .6 |
| Costmap TF | .408 | .500 | .728 | .6 |
| Cloud | .105 | .155 | .190 | .75 |

Receipt ages were not retained, so historical dual-clock freshness cannot be
independently reconstructed; healthy is the node's retained decision. Most
late path staleness follows controller rejection/recovery, so it cannot explain
the original lack of OPEN. No stale protection was relaxed.

## Command chain and actual motion

The governor limit ranges **.08155–.11000 m/s**, median/p95 .11; overspeed
protection only reduces it. Raw, smoothed and final vx each have median .06,
p95/max .11. Raw vx .06 occurs in 514/1786 navigation samples, zero in 411;
final vx zero occurs in 582/1956, and .06 in 510. RPP's configured regulated
minimum .06, cost/curvature/approach regulation and rotation/stops account for
commands below the governor ceiling; a SpeedLimit is a ceiling, not a request
to drive at that velocity. The records do not identify which RPP regulator
bound every tick. Raw abs(yaw) max .996 rad/s is clipped by the smoother/final
chain to .6; this is not a measured physical yaw rate.

Retained speed-summary.json: truth NARROW median/p95/max
**.11574/.17745/.27565 m/s** (7749 samples); SLAM
**.11768/.18201/.24280 m/s** (774 samples). These include start, turning and
stopping. No OPEN physical samples exist and neither .50 m/s OPEN nor sustained
.18 m/s NARROW tracking is demonstrated. Navigation-window figures, excluding
pre-goal samples, are in analysis.json. The chain is visibly commanding low
speed already; this is not evidence that .43 commands were lost downstream.

## G1 collision rejection and turn feasibility

There are **68** RPP collision-ahead warnings, first epoch 1792310937.9303358
(16:08:57.930), last 1792310948.7219732 (16:09:08.722), and eight controller
patience-exceeded events. Local costmap clearing and re-sending FollowPath do
not remove the repeated rejection. Five earlier `Transform data too old` errors
are present; the final pair at 16:08:49.283/.333 reports path time 105.9025 s vs
TF 105.4 s, exceeding RPP's configured .5 s tolerance by .0025 s. They are real
intermittent control interruptions, not proof of a persistent localization outage.

Aligning the log's system epoch to the trace's monotonic receipt clock through
goal_start_wall_time_s and the recorded /goal_pose gives the first collision at
approximately **112.54 sim s** (alignment has transport latency uncertainty).
Nearest status is healthy: map/path/odom/cloud/localization TF ages
.540/.440/.440/.140/.440 s. Left clearance is already **.0444 m**, curvature 0,
and reason still says curvature. Near 112.96 s its footprint-envelope width and
both sides are **zero**. Freshness does not guarantee spatial accuracy, but
continuous source staleness is not the demonstrated first collision trigger.

Last positive raw vx: 107.7925 s; last positive final vx: 107.9025 s (.00601).
Translation is then zero while small nonzero yaw commands continue until RPP
rejects them. Independent truth poses move **.302 m net from 107.9 to 114 s**.
Truth speed at 110/111/112/113 s is .0433/.0603/.0397/.0409 m/s. Integrated
truth travel during 109–112.54 s is .175 m, and during 112.54–114 s another
.0386 m. Thus the residual physical movement is real, not merely stale TF or a
smoother vx tail. This does not isolate momentum from policy response to small
rotation commands or contact; new calibration is outside scope.

Frozen G1 is (-.185718, 7.189986), yaw **-2.896660 rad** (G1→G2 corridor
heading). Incoming robot yaw near 110 s is +1.298385, requiring a shortest
**+2.08814 rad / 119.64°** turn. Endpoint is (-.355302, 7.332431), .22147 m
from G1, yaw approximately +1.432: position is within the configured .25 m Nav2
XY tolerance, but yaw is about **1.95 rad** away, far outside .3 rad tolerance.
The unchanged stateful goal checker permits position acceptance then heading
alignment. The frozen route consequently asks for a substantial turn at G1;
being close in XY cannot satisfy the existing arrival contract.

Footprint source: nav2.yaml rectangle 1.40 × .76 m, padding .05 m on each
side; effective envelope **1.50 × .86 m**, corner radius .8645 m. A pivot can
sweep a diameter of 1.729 m; passage width sufficient when aligned does not
establish pivot clearance. Smac2D planning does not provide a proved full-body
orientation-sweep contract for this final goal turn.

A limited offline geometric check uses the retained **122 s /map**, occupied
cells >=65, and exact separating-axis intersection of each cell square with
the padded rectangle, including its interior. This is a **static-map proxy**,
not a replay of RPP's missing runtime local costmap/collision arc:

| Pose / sweep | Proxy evidence |
|---|---|
| Recorded 110 and 112 s poses | 0 occupied overlaps |
| Approximate first-collision pose | 1 occupied cell at current yaw; 26/181 sampled shortest-turn orientations overlap |
| Recorded 113/114 s poses | 2 / 3 occupied cells overlap |
| Recorded 122 s pose | 4 occupied cells; 84/181 shortest-turn samples overlap |
| Exact canonical G1 center, incoming yaw → frozen heading | 0 occupied and 0 unknown cells at all 181 sampled orientations |

The sampled sweep is not a continuous collision-free proof. The later static
map could differ from the collision-time obstacle layer; footprint checker
rasterization also differs. Still, it supports the combined explanation of
**a large required heading change plus real residual translation into poor
turning clearance**. It contradicts a blanket claim that the frozen G1 center
is intrinsically impossible to turn in. The first warning could be a current
footprint collision or projected rotation collision; absence of local costmap,
RPP carrot/arc and input path snapshots prevents distinguishing those branches.
collision_plate count is zero, which does not prove the entire robot never
contacted anything. The record shows collision *rejection*, not verified impact.

## Minimal fix and offline verification

Confirmed diagnostic defects in adaptive_speed.py:

```python
# Before: labels heading misalignment as curvature and masks simultaneous overlap.
reason = 'curvature' if turning else 'geometry' if tight else 'release_hold'
# Before: release_hold also reported when release is false (384 observed ticks).
```

The fix reports geometry first, heading separately, and `release_margin` when
the hold cannot start. It adds heading_error, tight, release_eligible and
clear_duration_s to the existing decision JSON:

```python
reason = ('geometry' if tight else 'heading' if heading_error >= cfg.heading_threshold
          else 'curvature' if turning else 'release_hold' if release else 'release_margin')
```

Mode/limit/hysteresis decisions are unchanged: heading is included in the same
immediate NARROW reset that previously used the combined curvature label.
No threshold, lookahead, collision checker, footprint, costmap, SLAM, locomotion,
smoother, waypoint or advancement configuration was changed. OPEN .43 ceiling
and anticipatory obstacle sampling/braking remain available under the existing
clearance contract. This patch improves diagnosis; it does **not** claim to
make this failed route faster or successful.

Three focused regressions failed before the fix, then the relevant geometry,
ROS callback, opt-in launch/default rollback and recorder checks passed:
**57 passed**. `course_navigation` symlink build passed. A separate 400-update
comparison with the previous diagnostic selection restored confirms identical
mode, command_limit and clear_since state, including open travel, obstacle
entry, bends and stale inputs. Evidence: red-tests.log, tests.log/tests.xml,
build.log and equivalence.py/equivalence.json under the derived root.
Initial test attempts lacked writable ROS_LOG_DIR and then the workspace
setup; those environment failures are retained separately. Correctly sourced
checks passed; no product change was made for those environment mistakes.
No runtime processes were launched by these tests; ownership/cleanup audit is
in cleanup.json. No existing ROS/Gazebo processes were signalled.

## Review decisions and next user commands

**No frozen route-contract change has been made.** The evidence does not yet
prove one is necessary. If G1 is intended as a transit waypoint, accepting XY
arrival before its frozen outgoing heading or assigning a Start→G1 arrival
heading is an explicit **user approval** decision: it changes headings or
advancement semantics. Alternatively, requiring a more centered stop before
the existing turn changes the arrival/position contract and also needs review.
Do not silently reduce XY tolerance, skip yaw acceptance, change G1/G2
coordinates or manufacture a narrower footprint.

Remaining uncertainties: whether curvature spikes are real or path-tangent
jitter (RPP paths not retained); exact rejected collision arc/cell; localization
accuracy despite fresh timestamps; source of physical movement during nominal
rotation; authentication of the supplied 1.0 m W_ref; unavailable loaded
planner/controller parameter snapshots. Recorder dispatches one asynchronous
parameter read without discovery gating: a discovery race is a hypothesis for
those missing snapshots, not established runtime plugin misconfiguration.
Neither changing curvature thresholds nor disabling rotation is justified by
the retained evidence alone.

Next commands, **after review**, for a deliberate unchanged-contract repeat:

```zsh
cd /home/zyd/program/b2w-embodied-navigation-course
scripts/build_course.sh --packages-select course_navigation --symlink-install
# T1: first close the prior session normally; enter only an authenticated W_ref.
read -r 'W_REF?Measured G1→G2 obstacle-to-obstacle width (m): '
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/b2w-adaptive-oct18-reviewed/ros-log
mkdir -p /tmp/b2w-adaptive-oct18-reviewed/metrics
scripts/run_warehouse.sh adaptive_speed:=true w_ref:="$W_REF" \
  results_directory:=/tmp/b2w-adaptive-oct18-reviewed/metrics timeout_s:=240.0
```

Separate T2, while T1 remains running (one route, no manual goals):

```zsh
cd /home/zyd/program/b2w-embodied-navigation-course
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/course_sim/config/fastdds_local.xml"
export PYTHONPATH="$PWD/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$PWD/.deps/lib:$PWD/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR=/tmp/b2w-adaptive-oct18-reviewed/ros-log
/usr/bin/python3 scripts/run_warehouse_route.py --mode full --advancement practical \
  --execute --ready-timeout 120 --output-root /tmp/b2w-adaptive-oct18-reviewed/route
```

This is a diagnostic repeat with improved reason visibility, not a repaired-G1
acceptance test. Do not expect OPEN where the existing clearance rule cannot
release. Original `/tmp/b2w-adaptive-route` remains the raw evidence location.
