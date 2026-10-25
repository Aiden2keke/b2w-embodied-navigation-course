# Live TF qualification / short motion — stopped at admission (2026-10-25)

Base and verified remote: `fdfd64cb0d692726c83b625c0b15155a991aa441`.
Branch: `experiment/canonical-planning-20261025`. Single agent.

LIVE TF QUALIFICATION: FAIL (not performed; admission blocked).
FORWARD/TURN/STOP: FAIL (not performed; admission blocked).
These are unmet qualification gates, not an observed live TF or motion failure.
Zero simulator sessions started; no nonzero commands, goals, DWB or retry.

## First blocking condition and decision

The user permits one precheck-only attempt only if the changed Owner can complete
its required zero-motion qualification inside that attempt, and explicitly says
to stop if the contract requires a separate zero-only session.
The existing contract does require a separate session:

- `scripts/warehouse_ab_trial.py` declares `--stationary-containment` and
  `--precheck-only` in the same mutually exclusive argument group.
- Motion authority explicitly requires prior stationary qualification; stationary
  mode rejects motion authority. Its success requires initial rest, exact-zero
  command evidence, injected watchdog containment, owned cleanup and then returns.
  That final containment/cleanup cannot certify a session that then moves.
- The committed TF scheduling review, HANDOFF and active NOW all require separate
  changed-Owner zero-only qualification before the micro-check. Historical
  stationary evidence does not qualify the changed executable.

STOP before simulator launch. Do not reinterpret precheck initial rest or startup
pause/resume as the full stationary watchdog qualification. No mode/contract edits.
The authorized precheck attempt was not consumed. A separate stationary launch is
outside this session's precheck-only authorization; none was substituted.

## Preflight evidence and limits

Initial branch, exact expected HEAD and clean worktree verified. Read AGENTS,
CODEX_GUIDE, HANDOFF, only active PLAN/NOW body and committed scheduling review.
Host read-only inspection: NVIDIA RTX A4000 driver570.211.01 available; no matching
Gazebo/ROS launch/bridge/LIO-SAM/Owner processes. Initial sandbox GPU denial was
resolved by host inspection, not treated as GPU failure.

`artifacts/recovery/live-tf-qualification-20261025/identity-audit.json` freshly
checks all46 reviewed source/install pairs and9 native pins: no mismatches;
source/install scope and native ledger equal historical manifest scope. Historical
manifest preserved. Separate current committed script identities:

- Owner: `2edb1003316603796546933f6e0b7ae06a7036c444f9593dbf5218d7734e964c`
- TF delivery: `139d78365605f5d083acff472729dd1c9116671871f761e05e0d1bb73009af4b`

Both match Git HEAD and prior final-verification ledger. The reviewed deployment
manifest covers package/native identities, not these two source-run scripts;
this explicit supplementary audit reconciles them without rewriting pins or
claiming a newly reviewed deployment certificate.

Fresh offline `--check` passed with the reviewed manifest, Profile B,1s pulses,
spawn(0,-1), domain231:46 pairs and26 constructed collision handles; no world,
ROS node or command. Domain231 was only an offline parameter: live emptiness,
exclusive lease and graph isolation are NOT certified. No trial directory or
`.used` marker created. Do not reuse this as a live domain clearance certificate.

Static graph accounting: empty-domain check runs before Monitor constructs
`warehouse_owner_localization`; thus the new node cannot trip that initial check.
It has rosout and parameter services disabled, one owned executor and only TF/odom
subscriptions. Later checks enforce sole command publishers and absent goal
producer/manager rather than a fixed total node count. Live graph remains untested.

## Measurements, safety and review

Owner/observer TF/odom ages, ingress/callback timing, command delivery, actual
forward/yaw speed, excursion, stopping coast, measured rest and contact validity:
N/A; no live samples. Do not report zero speed/coast as a measurement. Whether the
dedicated executor eliminates the historical0.5025s failure is UNDETERMINED.

Safety: stop-before-launch respected. No owned simulator processes were created;
runtime watchdog/contact/rest/cleanup performance is untested. Existing independent
watchdog,0.5s freshness, physical vetoes, exact-zero and owned cleanup unchanged.
Single-agent actual-diff review: documentation only; no runtime or deployment pin
changes. Validation: fresh identity audit, offline engine/input check and
`git diff --check`; no regression test rerun warranted for documentation only.

## Exact next experiment and Start→G2 blocker

New session must separately authorize ONE changed-Owner zero-only
`--stationary-containment` qualification, with fresh domain<=231, exclusive lease,
unused output, reviewed identities and graph, advancing TF/odom, valid contacts,
initial rest, zero-only delivery, independent watchdog containment and cleanup.
Use its existing mode contract; alternate spawn options currently belong only to
precheck mode. No motion authorization is implied by this report.

Only after PASS and new precheck authorization, run canonical Warehouse at(0,-1),
recheck1.5m observed clearance,1s vx.05/rest/1s wz.05/rest, preserving all original
limits. Short-motion milestone remains OPEN. Then resolve G1 body/rotation
coverage and Profile-B stopping/tracking/localization bounds (micro-check cannot
qualify0.2m/s or mixed motion), reviewed route envelope and explicit0.3m navigation
versus0.05m staging tolerance before separately authorized actual Start→G2.
Existing145-pose7.213m Start→G1 path stays valid historical planning evidence;
no repeated planning capture required. Start→G2, Start→G5 and video remain NOT DONE.
