# Oct 18 adaptive state-machine repair — STOP for independent review

The user authorized this control repair after the
[retained-run diagnosis](ADAPTIVE_RUN_2026-10-18.md). PLAN.md's sole NOW milestone
was updated before changing control semantics. This milestone is offline only:
no Gazebo, goals, calibration, commit or push. Physical improvement and full
route success remain unverified.

## Behavior implemented

| Condition | Decision |
|---|---|
| Fresh valid perception/localization/path and confirmed safe geometry | OPEN, normal travel |
| Upcoming width <= W_ref + .10 m, or either footprint-side clearance <= .15 m | Immediate NARROW |
| Footprint overlap, unknown cells, off-map boundary | Conservative geometry restriction; existing RPP collision checks remain authoritative |
| Ordinary curvature or heading misalignment with safe footprint geometry | No state trigger; RPP retains curvature, cost, approach and heading regulation |
| Current entire footprint and forward samples have width > W_ref + .13 m and both sides > .18 m for .5 s | Release OPEN |
| Between entry and release thresholds | Retain mode; blocked release timer resets |
| Missing/stale/invalid required data, clock rewind, governor scheduling gap | NARROW and renewed confirmation; adapter heartbeat failure still produces zero Twist |

W_ref stays **1.0 m**, the user's approximate obstacle-to-obstacle measurement.
The original .10 m entry width margin and .15 m side minimum are unchanged.
The former release required both sides > .35 m for 2 s; release now uses .03 m
additional clearance and .5 s confirmation. This is an authorized safety-margin
choice supporting ordinary open travel, not a claim that the earlier code
violated its former thresholds. The retained start-side clearance .2556 m can
now release; .1442 m near-side clearance and .0444 m near G1 still restrict.
The 1.50 x .86 m padded footprint and cell-area/unknown/boundary checks are
unchanged. Including the actual current footprint prevents release while its
rear remains in the restriction. Path-aligned footprints are also checked ahead.

The forward checking distance is at least 1 m and uses measured SLAM speed:

```text
d = max(1.0, max(0, v_measured² - .18²)/(2*.20) + v_measured*.80 + .20)
```

At .50 m/s this is 1.144 m from the projected path position, plus the sampled
footprint's front reach. Deceleration .20 m/s² and latency .80 s remain
conservative assumptions, without new physical calibration. Once geometry
triggers NARROW, its detection horizon is retained until confirmed OPEN. This
prevents slowing from hiding the same upcoming choke outside a shorter
low-speed horizon and releasing OPEN before passage. Invalid-data interruptions
reset confirmation and preserve this reach. Every valid update checks the
current path again; a safe replan away from the restriction can release.
The path must cover this entire checking horizon. A shorter fresh path cannot
establish OPEN and reports `invalid_geometry`, including during final approach.
This may retain NARROW near the goal; goal acceptance remains RPP/Nav2's existing
contract. A later complete safe path can confirm OPEN without restarting.

Physical targets remain **OPEN .50 / NARROW .18 m/s**. Command ceilings remain
.43 / .11 m/s with the same downward-only measured-speed overshoot guard.
These targets and provisional commands are not calibrated guarantees. A limit
does not request that RPP drive at that speed; it can command less during a
turn, approach, stop or collision rejection. No inverse policy fit was added.

Curvature and heading_error remain diagnostic fields. Their obsolete governor
threshold parameters were removed. Entry no longer depends on a reason string;
geometry's `tight` result selects NARROW. Reasons are `geometry`, `release_hold`,
`release_margin`, `open`, `stale_or_missing` and `invalid_geometry`.

## Regression evidence and its limits

Source trace: `/tmp/b2w-adaptive-route/route/20261018T080730.741736Z/speed-trace.jsonl`,
SHA256 `d7009656872d29a5f04e70e9fd5810d2cdb74cbda73fdcf0bd66c8a27daf1440`.
The historical run had 988/988 NARROW statuses, including 417 formerly labelled
curvature and only 15 truly eligible for the old 2 s release. Its longest eligible
interval was about 1.125 s. These observations motivated separate tests for
turning, small safe bilateral margins and release timing.

`tests/fixtures/oct18_adaptive_status.json` retains exact selected statuses,
the initial safe interval at 45.805–46.4 s, the formerly eligible interval at
54.3925–55.4475 s, source identity and SHA256. Tests reconstruct conservative
straight strips from the recorded side minima and exercise real CostGrid and
Governor. The reported width and two sides are independent minima across
different poses, so they are not assumed to sum to one physical cross-section.
Original runtime local costmaps, input paths and heading errors were not
recorded; reconstructed strips and synthetic turns are **not** a runtime replay
or a prediction of the new physical trajectory. Retained healthy flags also
cannot reconstruct missing historical receipt timestamps.

Coverage includes:

- Start-side clearances approximately .25–.39 m and both retained safe intervals
  complete confirmation; ordinary heading changes and path bends do not latch.
- Near-side .1442 m, near-G1 .0444 m and overlapping geometry still restrict.
- Measured-speed braking reaches an upcoming choke before the current front;
  lower deceleration or higher latency extends anticipation. Unknown is unsafe.
- Slowing to zero before the choke and temporary stale data cannot erase the
  restriction; the rear must clear before the release timer starts. A truncated
  fresh path cannot hide the restriction or confirm OPEN at startup.
- Release-band jitter resets confirmation, while an established OPEN remains
  OPEN inside the band. Invalid path/pose/footprint/speed and rewind reset OPEN.
- Actual launch-expanded YAML releases the safe start strip. Planner remains
  SmacPlanner2D; RPP regulation/collision flags and goal-checker settings remain
  intact. Node freshness, adapter watchdog, recorder and frozen route/cleanup
  regressions pass.

Fresh evidence is under `/tmp/b2w-adaptive-state-review/`:

- `red-tests.log/xml`: **12 failed, 33 passed** against the prior control logic;
  failures expose the former release/turn contract. Existing safety cases pass.
- `red-braking-memory.log`: **2 failed**, demonstrating premature OPEN when
  deceleration or stale data hid the upcoming restriction before adding memory.
- `red-path-coverage.log`: **2 failed**, demonstrating that a short fresh path
  could previously hide the restriction or confirm OPEN without full coverage.
- `tests.log/xml`: **218 passed** in the focused geometry/node/launch/recording/
  route/footprint/session suite, with ROS environment sourced but no nodes started.
- `build.log`: course_navigation and course_bringup symlink build, exit 0.
- `tests-owner.json`, `build-owner.json`, `cleanup.json`: owned process groups
  exited with no members; no signals to pre-existing processes.
- `raw-sha256.json`, `preservation.json`: all retained run files remain unchanged.
- `incremental.diff`: changes versus this task's starting copies where available;
  the review bundle also contains the complete inherited worktree contents.

## Separate unresolved issue: G1 residual motion and large heading turn

This state-machine repair does not resolve G1. Independent truth in the retained
run shows about **.302 m net displacement after final vx stops** (107.9–114 sim s),
while small nonzero yaw commands continue. The frozen G1 heading requires
**119.64 degrees** from the recorded incoming yaw. The robot ends about .221 m
from G1, within the existing .25 m XY tolerance, but about 1.95 rad from target
yaw. Position acceptance alone does not satisfy its existing heading contract.

There were 68 RPP collision-ahead rejections. At the first warning (~112.54 sim s)
required data were fresh and retained side clearance was only .0444 m. A later
static-map proxy supports occupied overlap at the displaced pose, whereas
181 sampled turn orientations at exact canonical G1 center were clear. This
does not prove a continuous safe rotation or identify RPP's exact rejected arc.
Momentum, policy rotation response and contact remain unresolved possibilities.

Waypoint coordinates/headings, practical advancement, Nav2 goal acceptance,
footprint/padding, collision checking, SLAM, RL policy and Smac2D/RPP profile
are unchanged by this task. Any future heading, stopping/arrival or collision
change is a separate decision. The historical run's cleanup remains unconfirmed;
this task's clean offline process audit does not rewrite that history.

## Independent review gate

Review the complete bundle, this report and PLAN.md before any live run. Check
the .03 m/.5 s release choice, horizon retention, real footprint rear coverage,
the unchanged freshness/watchdog behavior and the separation of G1's unresolved
motion/turn issue. No successful speed target, physical clearance or full-route
pass is claimed. After generating the requested bundle the agent stops here.
