# Handoff — Owner TF/odom offline correction

Prepared workspace: `/tmp/b2w-navigation-ab-20261023` (keep generated inputs).
Branch: `experiment/canonical-planning-20261025`; project remote alias `project`:
https://github.com/Aiden2keke/b2w-embodied-navigation-course.git.
Base:523dd0f65fa8807b3ab3e4220907f04726b3a2b5. Fix commit contains this file;
use `git rev-parse HEAD` and verify the same branch with `git ls-remote project`.
Original local `origin`, shared checkout and Stage4 edits are preserved.

DONE: independent localization node/executor; locked TF message/decision reads;
bounded TF-chain history and odom handoff retaining callback receipts; failure
latching and explicit bounded thread cleanup. Freshness remains0.5s. All existing
contact, sole command owner, stop, watchdog and cleanup safeguards remain.
Single-agent actual-diff review PASS for OFFLINE correction.379 focused tests
PASS (including real DDS runner/recorder), latest ingestion subset10/10 PASS;
46 source/install pairs,26 collision handles and62 original raw hashes verified.
No Gazebo or motion. Historical DDS arrival is unavailable: replay establishes a
supported dispatch failure mode, not the sole historical transport cause or a
live latency guarantee. Report: `docs/reviews/B2W_Owner_TF_Odom_Scheduling_20261025.md`.
Evidence: `artifacts/recovery/owner-tf-scheduling-20261025/` (ignored local logs).

NEXT, separate authorization: changed-Owner zero-only qualification, then existing
Profile-B precheck1s vx.05/rest/1s wz.05/rest in a verified1.5m sensor-free pocket;
retain original phase/watchdog limits and all physical vetoes. Exact proposed
invocation and acceptance are in the report; it has NOT been run. Reverify this
commit's Owner+owner_tf_delivery hashes along with deployment/native pins. Do not
reuse historical749a4169 Owner pin as if unchanged or alter pins to bypass review.
Micro-check cannot qualify0.2m/s Profile-B stopping/tracking/mixed-motion bounds.

Real145-pose7.213m Start→G1 path already exists. Do not reopen Smac or startup work
or require a new path capture to establish that fact. Historical Owner session
FAILED at TF age.5025s; action terminal status unavailable. Actual Start→G2 and
continuous Start→G5/video NOT DONE. G1 body/rotation coverage and Profile-B bounds
remain blockers;0.3m navigation tolerance vs0.05m staging unchanged. No observer
truth for control, grace, unknown-as-free, policy change, broad kills or force push.
No authorization for new Gazebo/motion is inherited from this delivery.
