# Sustained-turn milestone — completed; retained heading FAIL

## Actual authorized single-run result

**Runtime safety/rest/cleanup PASS; useful retained-heading qualification FAIL.**
One consumed attempt and one positive pulse exist in
`course_results/imported_evidence/sustained-turn-20261028/live/precheck-9c8597f5add2480885484fad569e9c5d`.
No route, retry, tuning, batch or GitHub push occurred.

Invocation provenance: the assistant's read-only preflight returned
`Another sustained experiment invocation is active`. Host inspection identified
PID2192674 holding FD9, already running the exact reviewed command/manifest in
the fixed output directory. No second invocation was launched. The assistant
observed this matching invocation through normal completion and independently
audited its evidence/cleanup. This report does not attribute its launch to an
assistant tool call. The failed competing preflight is preserved, not relabeled
as a pre-motion PASS. The existing owner passed its recorded source/native,
engine, initial-rest, fresh-map, TF/contact and first-motion admissions.

| Measured quantity | Actual result |
| --- | ---: |
| External yaw request / model yaw input | +.25 / +.400000006 rad/s |
| Owner pulse / received final-command interval | 1.000 / .940 sim s |
| Model nonzero interval | .9375 sim s |
| External received-command integral | .235 rad |
| Truth transient turn | +.181280940 rad |
| Received-zero→final-rest rebound | −.169579126 rad |
| Retained truth heading | +.011701814 rad |
| Retention | **6.4551%** |
| Qualification floor | ≥.04 rad AND ≥50% — **FAIL** |
| Max cumulative yaw / translation | .184380 rad / .055536 m |
| Max translation speed / active yaw rate | .067990 m/s / .215462 rad/s |
| Max rebound yaw rate (already zero commanded) | .305893 rad/s |
| Max base tilt | .605891 degrees |
| Zero→qualified rest start | 1.7325 sim s |
| Qualified rest interval | 27.0525…32.1900 sim s |
| Whole-stop translation / rest-window drift | .017992 / .003394 m |
| Rest-window yaw drift | .004638 rad |
| Last-second truth speed / yaw rate max | .000765 m/s / .00000720 rad/s |

The limits remained unchanged and no guard fault was recorded. The larger
rebound rate occurred after zero was commanded. Normal watchdog shutdown,
six-axis final zero, measured LIO/truth rest and empty owned cleanup all pass.
Independent host audit found zero surviving registered/owned processes and
successfully reacquired domain226's lease. Nineteen registered identities were
removed. The attempt marker is consumed; do not rerun the prepared command.

IMU/truth heading discrepancy max .001535 rad; relative SLAM/truth yaw discrepancy
max .000371 rad over76 paired samples. Relative registered XY discrepancy max
.006266 m. All158 contact messages in the pulse/stop window contain all four
feet, with zero forbidden-contact inventory flags; full session contact count631.
Contact wrenches are absent, so friction/load claims would be unsupported.

The heading reference was captured once. At rest its error is .168504 rad and
the reconstructed added wheel scalar is .749999925 Nm, matching the **.75 Nm
hold cap**. This sustained trial does saturate that cap; the short trial did not.
No20 Nm motor clamp saturation occurred. Asynchronous reconstruction has a
transient peak estimate .772 Nm; this is not a direct clamp-output measurement
or proof of a cap violation. Late settled reconstruction/formula agree.

Wheel differential changed+.510582 rad; peak wheel speeds were1.33…1.89rad/s.
Leg motion is substantial (largest hip change .114112 rad). During stopping,
fitted stance yaw moves from−.103247 to+.022589rad: +.125835rad, about74% of body
recoil magnitude. This is geometric correlation, not a force decomposition.
Net integrated longitudinal rolling residual magnitudes are ≤1.83mm active and
≤1.31mm stopping; these are not instantaneous slip maxima and cannot exclude
cancelling longitudinal slip, lateral slip or unmeasured contact loads.

**Control decision:** the unchanged controller fails retained heading even with
the longer turn and active capped wheel hold. A leading supported mechanism is
stance/leg reconfiguration during active→stop combined with inadequate wheel-only
heading authority. Localization is an unlikely explanation. A unique dominant
physical cause cannot be proved without contact loads/lateral slip evidence.
Recommend ONE next approach: design coordinated stance/body-heading stopping
offline, preserving balanced support and a common body-yaw objective through the
transition. No torque-cap increase, blind gain series or further motion follows
from this result. The controller remains unqualified for navigation.

Independent Sol Medium interpretation reproduced the raw yaw, wheel differential,
settled torque and cleanup values and confirmed the above causal limitations.
Analysis, raw hashes and audit: `course_results/imported_evidence/sustained-turn-20261028/analysis/`.
Observer events SHA256:
`8483f1c3e47ffe2bc2b1d5c42b3f390ecf6877ff0743f7336babe043fc264a9d`.
Session report SHA256:
`caa19c611e761bc987fec0355bc4b7aadc28c5feadd6b89c6faa210cf57eff79`.

The remainder records the preparation/review checkpoint preceding this result.

Scope: one Gazebo-only +.25 rad/s pulse, nominal up to 1.0 simulated second,
then exact six-axis zero and measured rest. No route, batch, GUI changes,
controller tuning, rebuild or APRIL bundle. Existing views/worktrees preserved.

## Exact inherited source review

The original motion manifest (not present in the imported evidence set) had
SHA256 `48d09d89b3528367d8a6ecd2a793f7e9562265cbb28be25272b55886447b57b5`.
Its trial source was recovered from git commit
`1c3e0220e627325f663dc9c37fd253d47d21bdfb` and hashes exactly to
`170bcb3ca6d391f76d2fdc2c0983fe6cb5f89db14dd7ed487162e7b5264f1d5f`.
Inherited HEAD trial hash was
`569813eedc023859bf8905ea93cb4479d515af414d24633a05875cd67c1bacc7`.
The exact 13-hunk diff and both complete sources are retained in
`course_results/imported_evidence/sustained-turn-20261028/`; none is inferred from commit descriptions.

Classification of every inherited diff hunk (line numbers refer to the original
reviewed source; mixed hunks are classified by executable effect):

| Old line | Difference | Classification / effect on this experiment |
| --- | --- | --- |
| 781 | `gui_client_command` helper | Diagnostic instrumentation/GUI process construction; new mode forbids GUI. |
| 804 | `stationary_view_only` report field | Diagnostic instrumentation; no actuation. |
| 814 | Fail on changed/dead recording coordinator PID identity | Safety critical additional liveness veto; unused without coordinator, forbidden here. |
| 894 | Always start server without embedded GUI | Controller/motion execution environment and safety ownership; same physics server, separate GUI below, no physics/policy parameter change. |
| 916 | Attach GUI only after exact server pin/token check | Safety critical process identity plus diagnostic display; forbidden here. |
| 1082 | Bounded stationary viewer, exact zeros, stability veto, renewed rest | Controller/motion lifecycle plus safety critical additional vetoes; stationary branch not entered here. |
| 1162 | Journal exact sent waypoint and measured arrival/truth/readiness | Diagnostic instrumentation, synchronous IO/timing in route branch; no route branch entered here. |
| 1181 | Optional bounded zero-command recording end hold with rest veto | Controller/motion lifecycle plus safety critical rest checks; forbidden here. |
| 1311 | Reject viewer evidence as controller qualification | Safety critical admission strengthening; retained. |
| 1420 | New CLI options, changed GUI help | Executable diagnostic/controller-lifecycle configuration; help text documentation/nonfunctional. All recording/view options forbidden here. |
| 1430 | PID/start identity pair and finite end-hold validation | Safety critical argument checks; retained. |
| 1439 | GUI config and zero-only viewer scope/duration checks | Safety critical argument checks; retained. |
| 1460 | Canonical viewer spawn exception; diagnostic spawn stays (0,-1) | Safety critical qualification scope; sustained mode forbids viewer and still requires (0,-1). |

No inherited controller law, policy, wheel torque cap, pulse physical threshold,
collision guard or independent watchdog code changes were found in this drift.
It was executable drift, so the old manifest was correctly rejected. The old
manifest and historical evidence remain untouched.

## Minimal reviewed candidate

`scripts/warehouse_ab_trial.py` adds a separate `sustained-yaw` axis. It permits
only explicit finite `0 < duration <= 1.0`, Profile B, fixed (0,-1), revised
deployment digest, precheck-only mode and no GUI/recording flags. The historical
axis still rejects duration above .5. The source revision remains forbidden from
route execution. Runtime command is one +.25 rad/s yaw pulse, followed by the
original exact-zero/rest/owned cleanup path; no pulse loop retries.

Existing .25 m displacement, .35 rad yaw excursion, .25 m/s translation and
.4 rad/s yaw-rate values are unchanged. A sustained-only guard additionally
applies cumulative yaw from first-motion origin and the speed/rate limits through
stopping, rejecting nonfinite velocities. Historically these yaw/rate checks were
pulse-only, and StopEvidence used a separate zero-command origin. Collision,
freshness, support, body attitude/height, 1.5 m observed-free pocket, 10 wall-second
pulse deadline, 420 wall-second owner/watchdog bound and five-sim-second qualified
rest remain in force. Guard faults latch, publish zero and invoke the original
containment/cleanup path. Sampled checks and allowed truth age are not continuous
hard guarantees; realized duration/yaw must be measured.

Candidate source SHA256:
`0b32db40e619d3aaab28a86d329cf506e15c54554cac83be43938ad7f5ba8c50`.
Candidate deployment SHA256:
`06fa316157fd3bce2314c2799a77c99a5424e53077c55b7b494582c537540c30`.
The candidate deployment file is
`course_results/imported_evidence/sustained-turn-20261028/candidate-deployment.json`.
Only the harness source pin and explicit `sustained_turn` scope differ from the
old manifest. Validation loads the digest-anchored original, restores the old
harness pin in a comparison copy, removes the new scope and requires complete
object equality. All 46 source/install records, nine native identities, other
four source pins and stationary evidence must remain exact. The controller binary
SHA256 remains `aa2e03e68e777cea84aea8e5a4e4f6d82477f5b9d3265df8a657bc7eb1ccacc5`.
This is source identity review, not a fabricated empirical controller PASS.

## Offline evidence and run admission

The new tests were first run against the old implementation: 12 failed as
expected (unsupported axis, absent cumulative guard and missing scope checks).
After the extension, 43 focused sustained tests pass. Full focused suite: **233 passed**. Independent Sol Medium final source/offline
safety review: **PASS**, with **131 independently rerun tests passing**. The exact
reviewed source and manifest hashes above were independently verified. Review
record: `course_results/imported_evidence/sustained-turn-20261028/independent-safety-review.json`; it
explicitly grants no live-motion or empirical qualification. The real
engine-construction/native/source/stationary-evidence `--check` passes without
ROS node initialization, simulation or motion. The prepared host command was
also executed with **only `--check`**, successfully.

Read-only host inventory: systemd PID1, accessible NVIDIA0/control/UVM devices,
expected RTX A4000 UUID, no existing GPU compute job and no competing simulator.
Desktop graphics remain active (about 30% utilization/6143 MiB at check time);
this is not exclusive GPU hardware ownership. Runtime must repeat the inventory,
require full host visibility, exact prepared hashes and an empty ROS domain226.
Independent owner/watchdog retain exact token/server identity. Fresh runtime
map/contact/rest/TF admissions cannot be claimed PASS before launch.

Historical one-shot command and source workspace are intentionally not retained
as runnable scripts. The preserved actual session and analysis are under
`course_results/imported_evidence/sustained-turn-20261028/` and fingerprinted by
its `MIGRATION_SHA256.json`. The original hash-pinned `motion-deployment.json`
referenced by this historical experiment is absent from the two source
repositories; its test remains skipped and it cannot authorize current motion.
There is no sustained-turn rerun command in this checkout.

## Measurement and decision contract

Retain observer raw truth pose/twist/base orientation, private IMU, SLAM,
policy input, delivered final commands, all 16 joint positions/velocities,
targets/torques, plus owner's complete contact/inventory stream. Compare named
wheel and leg kinematics, longitudinal rolling residual and support transitions;
rolling residual is an approximate slip indicator, not a contact-load/slip proof.
Missing signals or ambiguous timing makes causal attribution inconclusive.
Internal brake target/correction is not directly logged; it may only be inferred
from the unchanged formula and aligned joint/IMU data with that limitation stated.

Measure owner and delivered onset/zero intervals separately, external/model yaw
command integrals, transient/at-zero/settled truth and IMU yaw, rebound, both
owner-window and delivery-aligned retention, late turn rate, stop time, full-stop
translation/yaw, settled drift, localization consistency and watchdog/cleanup.
Use the existing ≥.04 rad and ≥50% retained-yaw floor; a runtime safety PASS is
not functional turning PASS. A one-sign pass cannot qualify bidirectional or
Profile-B operating range. Never infer body yaw from `.25 × duration`.

At preparation, latest measurements were .08108 rad transient/.01227 rad retained
(15.13%); the completed sustained result is now recorded above. Source review
alone supplies no physical root-cause claim. If sustained turn also rebounds, correlate wheel
rolling/stance evidence before choosing one correction; the supported current
candidate is coordinated stance/body-heading control across active→stop, rather
than blind wheel torque/gain increments. If sustained yaw retains, characterize
small-command/bidirectional behavior and Profile-B directional stop bounds before
fresh G1 swept-clearance and a G1-only arrival/stop checkpoint. Neither route nor
batch is authorized now.

Next action: offline coordinated stance/body-heading stop design from this failed
trial. The single live authorization is consumed; no route or repeat is authorized.
