# Handoff — Negative pure-yaw attempt rejected; no pulse delivered

Single agent, `/tmp/b2w-navigation-ab-20261023`, branch
`navigation/g1-g5-recovery-20261025`, session base exactly
`8df7df454f03af8dc6a568732b37966907e93d36`.
Commit/push/ref/checksum: see sealed APRIL Git and delivery evidence.
Main reconciliation/local files and Stage4 preserved.

Minimal opt-in `--precheck-axis negative-yaw` selects one pure -.05rad/s pulse
using the existing precheck owner and clock loop; default forward/turn unchanged.
Existing observer now captures joint targets, controller torque demands and
named joint feedback. 175 focused offline tests PASS; new tests RED→GREEN.
All admission/physical/contact/zero/rest/watchdog/cleanup paths unchanged.

Exactly ONE simulator attempt, domain212, requested spawn(0,-1), nominal.25sim s.
REJECTED BEFORE MOTION: initial1.5m observed-free disk includes one unknown cell
at nearest distance1.487999m; no occupied/outside/invalid cells. TF/odom/contact
and readiness passed at exception. No pulse, retry, goal, hardware or tuning.
Actual nonzero delivery duration0sim s, no interval; diagnostic NOT ADMITTED,
not a successful duration or yaw qualification. No separate stationary session.

483 final Twists and837 model command triplets all zero;837 target arrays,
3747 torque demands and1874 named joint samples. Last observed five seconds
truth drift.309mm/yaw2.113e-7rad; owner-qualified initial rest not reached.
362 contact samples/no contact failure. Watchdog contained36 server threads;
owned fallback cleanup cleared all19 identities. Host/token/server/lease checks
PASS; localization cleanup PASS. Normal cleanup NOT exercised this attempt;
prior stationary628-zero/19-process normal PASS remains historical evidence.
No postfault physical rest certification from pausing physics.

Old positive pulse remains+.05→model+.08→negative wheel differential/dq/gyro,
yaw-.154184rad then+.151055rad rebound. New OFFLINE fixed-state interventions
also show negative yaw sensitivity with state offset; no live negative response
exists and no matched-rest signed comparison is possible. Policy-origin local
response plus idle transition is the narrow likely mechanism; plant/contact and
state bias remain unresolved. No global sign flip or other control patch justified.

Read `docs/NEGATIVE_YAW_DIAGNOSTIC_20261026.md` and its evidence root
`artifacts/recovery/negative-yaw-20261026/` in APRIL. Next session: investigate
saved observed-free admission coverage/timing without weakening the predicate,
then separately authorize one matched-rest negative diagnostic. No live retry
in this session. Start→G2 remains blocked by controllable yaw/stopping and route
envelope;145-pose Start→G1 planning evidence retained. Actual Start→G2 and
continuous G1→G5 video NOT DONE. Navigation.3m/staging.05m remain distinct.

Single-agent scoped review PASS for diagnostic code/evidence, not yaw control.
Commit/push coherent changes; deliver ONE APRIL+SHA256, STOP after delivery.
