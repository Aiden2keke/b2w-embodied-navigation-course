# Handoff — Qualification attempt CLOSED: A PASS / B FAIL

Workspace `/tmp/b2w-navigation-ab-20261023`; branch
`experiment/canonical-planning-20261025`; remote `project`:
https://github.com/Aiden2keke/b2w-embodied-navigation-course.git.
Initial clean expected HEAD:270241453b45d791a3ff10fc0f8fff535f233174.
Use git rev-parse HEAD / git ls-remote project for the final documentation commit.
Single agent. No runtime, deployment-pin, startup/TF/Stage4 changes.

User authorized exactly one stationary session, then one separate precheck only
if A passed. Both consumed; no retry or ongoing live authorization.
A PASS on fresh domain213: advancing localization, all existing sensors/contact/
rest gates,619 exact-zero final samples,5.255sim/5.521691wall rest,
0.000270749m/2.36389e-7rad drift; injected watchdog paused all server threads,
owned processes removed, localization executor closed and lease released.

B FAIL on separate fresh domain212, canonical Warehouse spawn(0,-1).
Initial/forward/turn observed1.5m disks each passed2944 cells. Both1s requested
pulses and measured stops completed. First recorded failure:
`RuntimeError('Normal owned cleanup incomplete')` at normal shutdown after the
8wall-second cleanup bound. Watchdog subsequently found server identity absent;
its pause did NOT certify containment in B. Fallback cleanup reports PASS;
independent host check confirms no owned PID/token processes, both leases released,
and both localization threads closed. No automatic retry or runtime repair.

Measured forward body-vx mean/peak0.097012/0.141762m/s (planar peak0.141922).
Command+wz0.05 produced mean/min wz−0.151656/−0.200533rad/s; coast reversed yaw,
ending near its initial heading. This is wrong-sign tracking, not qualification.
Forward whole-coast max0.036475618m/0.005350422rad; turn whole-coast max
0.018353265m/0.151054692rad. Rest windows5.2875sim/5.796611wall and
5.4125sim/5.878406wall seconds; LIO+truth rest and six-axis exact zero passed.
Pulse+stop excursion forward0.135668m/0.007438rad, turn0.055833m/0.157627rad;
final net translation0.167270m and yaw−0.003306rad. All physical vetoes retained;
no forbidden contact, goal, NavigateToPose, DWB, mixed command or hardware.
Owner sampled forward loop elapsed1.0025sim seconds despite requested1s cap;
turn1s. Final-topic receive-clock nonzero→zero0.9425s/0.9075s; commands unstamped,
so this does not prove exact actuator timing or erase the sampled-loop overrun.

Observer active TF/odom age max A0.420s/B0.440s. Owner active freshness gates
passed; healthy per-decision maxima are not persisted. B's first readiness snapshot
was captured AFTER shutdown failure: TF sim age0.3425s; odom wall age8.055928s,
reflecting stopped streams.233 callbacks per localization edge,1051 TF callbacks;
last received/inserted stamps40.6s match, insertion completed0.000110354wall seconds
following callback entry. Owner max between spins0.175366s; dedicated ingress
continued receiving during the attempt. No DDS transport-latency or general TF
reliability claim. A has no persisted healthy ingress timing ledger.

Fresh46 source/install pairs,9 native identities and committed Owner/TF hashes
match. Evidence and measurement ledger:
`artifacts/recovery/live-qualification-short-motion-20261025/`.
APRIL archive includes raw session/observer evidence, measurement script, review,
documentation diff and Git verification; delivered with SHA256 sidecar to
`/home/zyd/APRIL_REVIEW_BUNDLES/` and destination checksum verified.

This attempt milestone is CLOSED with failure; short motion remains unqualified.
First Start→G2 blocker is B's failed normal shutdown gate, alongside wrong-sign yaw
and excessive forward tracking. NEXT NEW SESSION: resolve these, establish
Profile-B route-relevant stopping/tracking/localization/envelope bounds and G1
body/rotation coverage, keeping explicit0.3m navigation vs0.05m staging tolerance;
then genuine Start→G1→G2 and first continuous Start→G5 video. Existing canonical
145-pose7.213m Start→G1 capture retained; no recapture.0.2m/s/mixed-motion Profile B
and Start→G2/G5/video NOT qualified/done. Preserve original checkout/Stage4.
STOP after APRIL delivery; no infrastructure or test-count expansion.
