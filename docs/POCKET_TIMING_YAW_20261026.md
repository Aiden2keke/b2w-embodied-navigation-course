# Pocket admission timing and negative-yaw preflight — 2026-10-26

Base `b2b6260c73d61300a4f65d4e836da143fb095a8a`, branch
`navigation/g1-g5-recovery-20261025`, single agent. Evidence directory:
`artifacts/recovery/pocket-timing-yaw-20261026/` in the APRIL payload.

## Result

Timing correction and offline review PASS. **Live diagnostic BLOCKED before
simulator launch**: host GPU preflight found unrelated compute PID 3640423,
`/home/zyd/miniconda3/envs/raylab/bin/python`. Domain213 lease probe succeeded;
no competing simulator/owner/controller was found. The GPU assertion ran before
DDS domain discovery, so domain node discovery was NOT completed. The ordinary
sandbox cannot access the GPU; this blocking observation came from the actual
host with sandbox escalation. No automatic approval rejection occurred.

Per the user's stop-on-any-failure condition, no waiting/recheck/retry occurred.
Zero simulator attempts, zero pulses, zero navigation goals. The authorized
negative-yaw live attempt was not started. Nonzero delivered duration is 0 by
non-execution; there is no measured delivery interval or duration PASS. No new
model command, raw wheel target, wheel dq, torque, gyro, truth yaw, stopping
rebound or qualified rest was captured. No new cleanup lifecycle was exercised;
the transient host lease was closed before the GPU assertion. Existing live
normal-cleanup evidence remains historical, with offline cleanup regressions PASS.

## Exact cause and correction

The original bootstrap waits up to85 wall seconds for ready streams, server pin
and **some** genuinely observed-free map coverage. This does not establish a
full 1.5m disk. At the reviewed domain212 early snapshot (clock18.595, map18.0),
cell(376,425) was UNKNOWN, nearest cell point1.487999m from sensor pose, among2944
intersecting cells; occupied/outside/invalid counts were zero. `execute()` called
`motion_pocket_admission()` here and threw **before post-audit readmission and
mandatory initial measured rest**. This is the exact premature timing cause.
The geometry rejection itself was correct. A previous independent startup had
that cell FREE; it does not prove the rejected session would later clear it.

`MapProjector` processes real transformed scans and publishes `/map` once per
simulated second after receiving a SLAM map. `Monitor.receive_map()` replaces
its grid and saves the actual message stamp and monotonic receipt. Required rest
already dispatches callbacks while publishing exact zero and retains unchanged
TF/odom/contact/physical/watchdog checks. It can therefore receive additional
coverage without introducing another waiting framework or moving the robot.

The corrected precheck order is:

```python
early_free = save_pocket_admission(monitor, output, 'initial')
# early_free is diagnostic only; no motion admitted
# existing post-audit callback readmission and physical veto
rest_entry_sim, rest_entry_wall = monitor.clock, time.monotonic()
report['initial_rest'] = rest()
# for precheck_only:
save_pocket_admission(monitor, output, 'after-rest')
report['final_motion_map'] = require_motion_map(monitor, rest_entry_sim, rest_entry_wall)
report['final_motion_pocket'] = motion_pocket_admission(
    monitor.grid, monitor.pose()[:2], stationary_containment=False)
# existing per-pulse snapshot, then callback dispatch and active guards
require_motion_map(monitor, rest_entry_sim, rest_entry_wall)
if not open_pocket(monitor.grid, monitor.pose()[:2]):
    raise RuntimeError('Sensor open pocket absent before '+kind)
# only now may the existing pulse loop publish nonzero
```

The snippet elides report fields and unchanged surrounding code; the complete
reviewed diff is in the evidence. `require_motion_map` requires both actual map
stamp and receipt after initial-rest entry, rejects future stamps, and checks
age against existing map-input bounds6sim/20wall seconds. This is a new
conservative admission guard: previous `ready()` explicitly had no `/map` age
threshold. It runs after rest and again immediately before pulse publication.
It does not rewrite stamps or alter TF/odom0.5s freshness. No extra waiting was
needed or added; absent updates or persistent unknown/occupied cells fail closed.

Unchanged: `open_pocket`, map/bootstrap implementation, 1.5m radius, unknown/
occupied/outside/invalid rejection, rest thresholds/window/deadline, contact and
physical vetoes, watchdog ownership/deadlines, pulse command/loop and normal
cleanup. The .35rad/.25m excursion and .4rad/s/.25m/s speed vetoes remain.
Default forward+turn order and +.05 commands are covered by existing behavior
regressions. Stationary and route admission paths retain their existing logic.
No production sign/gain, policy, topic, TF or module-boundary change.

## Offline evidence

- Three initial timing regressions failed against original code: later-free
  map was rejected before rest; persistent unknown/occupied cases never reached
  initial_rest. Two freshness regressions then failed before adding the new guard.
- Final focused run: **165 passed in14.84s**, with ROS and this checkout's install
  sourced. Earlier environment-only failures omitted the install overlay; later
  fixture failures exposed missing map messages in the existing handoff fixture.
  That fixture now feeds actual `Monitor.receive_map(OccupancyGrid)` callbacks.
- Saved20261025 and20261026 snapshots replay through unchanged production
  geometry/final-admission predicates: respectively PASS and FAIL. Cell(376,425)
  is respectively0 and-1. Persistent-1/100 reject; a synthetic0 variant passes
  geometry only. No synthetic mutation is represented as real observed coverage.
- In-memory execute regression supplies later-free coverage during mandatory
  zero rest and admits only afterwards; persistent unknown/occupied and absent/
  stale publication cases send no nonzero command. Existing real Monitor
  callback regressions retain TF/odom/contact/physical vetoes and immediate
  pre-publication dispatch; existing watchdog/rest/normal-cleanup tests pass.
- Offline deployment check PASS: all46 source/install pairs,26 constructed
  collision handles, unchanged native/policy/config identities. Owner source
  SHA256 is recorded in `offline.json`; no source changed after that check.
- Single-agent scoped source/test/diff review PASS; no independent reviewer.

**Did the unknown cell become free?** Historical free vs unknown is reproduced;
no new physical map was collected, so later clearing of that cell is UNKNOWN.
Incomplete early coverage is established; coverage completion during rest is a
plausible hypothesis supported by offline sequencing, not a live-proven cause.

## Signed response and next action

Existing positive request+.05rad/s gave model+.08 after unchanged gain1.6,
negative wheel differential/gyro and owner-window truth yaw−.154183956rad.
Zero-to-qualified-rest rebound was+.151055087rad, net−.003128869rad. Whole
pulse+stop maxima were.056074m displacement,.157630rad yaw,.058336m/s speed
and.220313rad/s yaw speed. These values and raw positive events are retained
from prior `negative-yaw-20261026/comparison.json`; no historical values changed.
Old actual torque/contact-wrench evidence was unavailable. A matched-rest
negative response and whole-stop comparison are still unavailable.

Yaw controllability remains BLOCKED; no physical basis for production sign flip
or gain tuning was added. Actual Start→G2 and continuous Start→G5 video remain
NOT DONE. Prior145-pose Start→G1 planning evidence is retained.

Exact next action in a NEW session: verify host GPU availability and all existing
admissions once, obtain renewed explicit authority for ONE corrected-source pure
negative-yaw diagnostic at existing(0,-1) spawn, using nominal.25sim seconds,
then exact zero and qualified rest. Capture actual post-rest map/cell and
receive-clock delivered interval (target<=.30s), signed wheel/body response and
whole stop. If that clears yaw/stopping uncertainty, finish the reviewed
Profile-B directional stopping/tracking envelope and G1 arrival/rotation
coverage before one Start→G2 attempt. Do not repeat stationary/planning
qualification or expand Stage4. This session ends after reviewed commit/push
and ONE APRIL+SHA256 delivery; no additional live action is authorized here.
