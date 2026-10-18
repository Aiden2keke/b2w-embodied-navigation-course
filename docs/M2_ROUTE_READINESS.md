# M2 route readiness evidence (offline, corrected 2026-10-18)

Current roadmap: [PLAN.md](../PLAN.md). This report records evidence; it is not another plan.

**B. LOW_SPEED_SUPPORT_BLOCKER**. **NORMAL_NAV2_M2_READY=false**. FULL_ROUTE_PASS remains the system goal. The previous C readiness decision was rejected and is superseded. No production calibration is selected. No C2/C3 or further fit campaign.

## Structural smoother evidence and separate high-side risk

SSoT: `course_navigation/config/nav2.yaml`, `velocity_smoother.ros__parameters`:

```yaml
smoothing_frequency: 20.0
feedback: OPEN_LOOP
scale_velocities: false
max_velocity: [0.4, 0.0, 0.6]
min_velocity: [-0.2, 0.0, -0.6]
max_accel: [0.5, 0.0, 0.6]
max_decel: [-0.5, 0.0, -0.6]
deadband_velocity: [0.0, 0.0, 0.0]
```

`scripts/smoother_m2_support.py` reads this current YAML, hashes it, checks the supported OPEN_LOOP/unscaled/zero-x-deadband assumptions, then derives `delta_v_up=max_accel_x/frequency=.025` and `delta_v_down=abs(max_decel_x)/frequency=.025`. The calculation stops if the YAML cannot justify it. Synthetic YAML tests change frequency and acceleration and require changed ramps.

| Nominal startup tick toward .15 | vx | M2 desired-output bucket |
|---:|---:|---|
| 1 | .025 | LOW / unsupported |
| 2 | .050 | LOW / unsupported |
| 3 | .075 | LOW / unsupported |
| 4 | .100 | LOW / unsupported |
| 5 | .125 | SUPPORTED; first V_MIN crossing |

From the supported .125 state: `.125 SUPPORTED -> .100 LOW -> .075 LOW -> .050 LOW -> .025 LOW -> 0 ZERO`. These are nominal command recurrence values, not physical motion observations. The normal post-smoother strict adapter rejects the first positive startup command before legacy limiting. Exact G1/G2 logs are unnecessary to establish this structural integration blocker.

**Separate HIGH-SPEED RISK:** both DWB max_vel_x and smoother max_velocity_x permit .4 m/s, above M2 V_MAX=.27052833048110847. Analog runs contain high commands. Exact frozen-route high-side fractions remain unavailable; no claim that every route reaches .4.

## C closure

Formal run: `/tmp/b2w-user-calibration-C/20261017T104226682472Z`. Frozen validation: `/tmp/b2w-user-calibration-C-validation-01/validation.json` (SHA256 `c882bba3889a5cd21a30fdbec9f46acf559518d1b0b700f4abcf44b488df3617`).

C DATA VALID PASS; observed_monotonic=true. M2 VALIDATION PASS; M3 also passes as backup. Complexity order M0 -> M1 -> M2 -> M3 selects M2. production_selection=NONE; refit_performed=false.

| u | Held-out truth v (m/s) |
|---:|---:|
| .055 | .12206408203827651 |
| .085 | .15295192621044565 |
| .115 | .18423769301428736 |
| .165 | .2354430135573286 |

M2 MAE=.0003071140364180498; RMSE=.0003557242162431472; max_abs_error=.0004892164972506163; R2=.9999278975002264. All predictions supported and monotonic. Frozen B SHA256 `15b371aa80afe5cef8461d077fae0bc8bb0a180ae92ea5ff6c669c9b04d352bc`.

## Candidate boundary and semantics

`controller_server -> /course_navigation/cmd_vel_raw -> unchanged velocity_smoother -> /course_navigation/cmd_vel -> VelocityAdapter.command -> legacy VelocityGate -> VelocityAdapter.publish -> /path_manager/path_manager_ros/nav_vel -> rl_sar command_ -> obs[6:9]`.

Adapter is the existing command-to-policy boundary. The candidate checks desired vx before legacy limiting and converts gated vx immediately before publication. Legacy limit/localization/timeout logic is unchanged. Parameter `m2_inverse_enabled` defaults to false; no production launch or YAML enables it. `vy` and `wz` are unchanged by the inverse (legacy gate limits still apply). No policy, controller, Nav2 YAML, smoother parameters, SLAM or footprint edits. The diagnostic runner has an explicit opt-in transition range; its ordinary .04 minimum remains unchanged.

```python
if v == 0.: return 0.
if not V_MIN <= v <= V_MAX: raise UnsupportedVelocity(...)
if v == V_MIN: return U_MIN  # exact frozen endpoint; binary64 one-ULP issue
u = (v - 0.0659258536489146) / 1.0245445280935277
```

Moving u support `[0.03999999910593033,0.20000000298023224]`; desired physical v domain `[0.10690763385664151,0.27052833048110847]`. Exact lower endpoint maps to U_MIN explicitly because direct binary64 inversion is one ULP below it; adjacent values outside the physical domain still reject. Upper endpoint inverse is within u support and is not forced to U_MAX (the supported physical upper domain is an intersection). No extrapolation, bridge, interpolation or clamping.

Unsupported positive vx, negative vx and nonfinite vx raise a named exception, publish an unsupported diagnostic and log FATAL. The candidate node terminates without publishing a substitute Twist. This is a strict diagnostic boundary, not an authorized dynamic route controller. Existing downstream watchdogs are not redefined by this inverse.

**Experimental operating domain:** `validated_condition=pure_forward_vy0_wz0`. B/C used vy=wz=0. The implemented transform preserves vy/wz, but this is not experimental proof of vx-response invariance during turning/lateral commands. Candidate identity, diagnostic JSON identity and reports state this scope; no new nonzero-wz rejection is added in this repair. Dynamic routing remains unauthorized.

Only opt-in creates `/course_navigation/m2_inverse_diagnostic` (std_msgs/String JSON). Every accepted/rejected event includes identity, desired triple, policy triple or null, and reason. The startup log exposes identity. Package declares the std_msgs dependency.

Candidate version `m2-strict-v1`; current scoped identity SHA256 `96465ddad7a707b7e21cd69f97e6761f2c2ea0eb099aed61e4264fc2139cc103`; inverse source SHA256 `dd7b299acdf69406ce3ea2115d48787f94e967550f2627edab5503c379b30f7b`. Only operating-condition metadata was added in this repair; inverse/zero/endpoints/default-off semantics are preserved. Identity includes pure-forward scope, frozen B hash, C validation hash, coefficients, domains, endpoint rule and source hash. Harness additionally pins adapter/harness source hashes.

## Frozen route evidence

Exact G1: `course_results/navigation/20261016T083151.972156Z.json`: succeeded at original .3 m manager tolerance; final error .06484485334757964 m. This is not the current .05 m full-route advancement PASS.

Exact G2: `course_results/navigation/20261016T083236.246776Z.json`: replaced, duration field 26.015 sim s, path 2.12618673516426 m, final error 2.5863396397371887 m. Neither file contains velocity samples or simulation start/end timestamps. Filename wall-clock time plus duration cannot establish a simulation interval in another capture.

For frozen Start->G1 and G1->G2: raw/shared/final counts, time fractions in all four requested support categories, positive min/median/p90/max, and braking-versus-traversal attribution are **unavailable (null), not zero**. No frozen-route recovery/braking boundaries can be recovered from these summaries.

The missing Oct16 command timeline limits exact frozen-route support fractions, high-speed demand, braking versus traversal, recovery attribution and simultaneous angular-command distribution. It does not control the proven low-speed-blocker decision. An exact inverse-OFF G1/G2 capture may later answer those questions; it is not the next prerequisite.

Next USER-RUN experiment is the preregistered [low-speed transition diagnostic](LOW_SPEED_TRANSITION.md), not an M2 in-domain tracking test or fit campaign.

## Preserved command inventory

All native inputs below are read-only and hash-pinned in `/tmp/b2w-m2-route-readiness/route-audit.json`. Duplicate /tmp Oct13 trace has the same bytes as the repository copy; only the repository copy is counted. Manager summaries and unexecuted mock/dry-run route artifacts are not velocity evidence.

Time fractions use the entire stated simulation receipt window; gaps >.5 s are omitted, never extrapolated. Equal-time count samples are all retained; time integration uses the last received sample. Remaining coverage is missing. Quantiles use linear interpolation of count ranks. Raw is the observed raw topic, which may also contain behavior commands. These runs use different goals/configurations and cannot authenticate the frozen route.

| Run | Stream | N | zero / low / supported / high counts | Positive min / median / p90 / max | Coverage |
|---|---|---:|---|---|---:|
| warehouse-baseline-20261013 | final | 437 | 334 / 11 / 25 / 67 | 0.017105263 / 0.315789474 / 0.400000000 / 0.400000000 | 98.315% |
| warehouse-baseline-20261013 | raw | 104 | 1 / 8 / 18 / 77 | 0.021052632 / 0.400000000 / 0.400000000 / 0.400000000 | 22.767% |
| warehouse-baseline-20261013 | shared | 107 | 4 / 11 / 25 / 67 | 0.017105263 / 0.315789474 / 0.400000000 / 0.400000000 | 23.633% |
| 20261014T060802.404851Z-smac2d-smoke | final | 618 | 309 / 29 / 52 / 228 | 0.013157895 / 0.378947368 / 0.400000000 / 0.400000000 | 100.000% |
| 20261014T060802.404851Z-smac2d-smoke | raw | 420 | 55 / 38 / 11 / 316 | 0.021052632 / 0.400000000 / 0.400000000 / 0.400000000 | 90.726% |
| 20261014T060802.404851Z-smac2d-smoke | shared | 427 | 47 / 29 / 63 / 288 | 0.013157895 / 0.378947368 / 0.400000000 / 0.400000000 | 91.035% |
| 20261014T065029.802588Z-smac2d-light-smoke | final | 1199 | 788 / 51 / 45 / 315 | 0.003947368 / 0.378947368 / 0.400000000 / 0.400000000 | 99.937% |
| 20261014T065029.802588Z-smac2d-light-smoke | raw | 436 | 50 / 38 / 34 / 314 | 0.021052632 / 0.400000000 / 0.400000000 / 0.400000000 | 36.306% |
| 20261014T065029.802588Z-smac2d-light-smoke | shared | 453 | 43 / 51 / 45 / 314 | 0.003947368 / 0.378947368 / 0.400000000 / 0.400000000 | 37.609% |
| 20261014T090829.047739Z-smac2d-footprint-gate-light | final | 1884 | 1317 / 83 / 103 / 381 | 0.003947368 / 0.378947368 / 0.400000000 / 0.400000000 | 99.960% |
| 20261014T090829.047739Z-smac2d-footprint-gate-light | raw | 917 | 375 / 75 / 75 / 392 | 0.021052632 / 0.378947368 / 0.400000000 / 0.400000000 | 49.890% |
| 20261014T090829.047739Z-smac2d-footprint-gate-light | shared | 1043 | 480 / 79 / 103 / 381 | 0.003947368 / 0.378947368 / 0.400000000 / 0.400000000 | 54.102% |
| B-w1.7000-seed0 | final | 286 | 1 / 5 / 19 / 261 | 0.025000000 / 0.400000000 / 0.400000000 / 0.400000000 | 1.513% |
| B-w1.7000-seed0 | raw | 286 | 1 / 1 / 14 / 270 | 0.042105263 / 0.400000000 / 0.400000000 / 0.400000000 | 1.507% |
| B-w1.7000-seed0 | shared | 285 | 0 / 5 / 19 / 261 | 0.025000000 / 0.400000000 / 0.400000000 / 0.400000000 | 1.507% |

Each run has separate observed-goal windows, source lines/sequences, receipt and available source stamps, terminal/censor state and recovery transitions in the JSON. Missing initial /goal_pose is represented as unknown exact XY, never reconstructed from rounded log text. A cancellation of an old action between a new request and its Begin message is not assigned to the new goal. Before-first-recovery and from-first-recovery windows are separate. Terminal braking is not established by log markers alone.

## Closest noncanonical corridor example

Oct14 `20261014T090829.047739Z-smac2d-footprint-gate-light/events.jsonl`: initial navigation log at sim 82.8925, succeeds at 104.7075; initial exact goal event was not captured. Next observed goal is `[-4.510785102844238,5.606146812438965]`, received at 105.125, Begin at 105.21, failure at 141.2675. Source lines 18364 / 18429 / 47481. **These are not frozen G1/G2 coordinates.**

| Stream | N | zero / low / supported / high count fraction | zero / low / supported / high time fraction | Positive min / median / p90 / max |
|---|---:|---|---|---|
| final | 738 | 583 (78.997%) / 33 (4.472%) / 54 (7.317%) / 68 (9.214%) | 79.076% / 4.648% / 7.076% / 9.200% | 0.003947368 / 0.232894737 / 0.400000000 / 0.400000000 |
| raw | 397 | 259 (65.239%) / 24 (6.045%) / 44 (11.083%) / 70 (17.632%) | 37.387% / 3.299% / 5.886% / 9.566% | 0.021052632 / 0.273684211 / 0.400000000 / 0.400000000 |
| shared | 455 | 300 (65.934%) / 33 (7.253%) / 54 (11.868%) / 68 (14.945%) | 39.303% / 4.600% / 7.104% / 9.248% | 0.003947368 / 0.232894737 / 0.400000000 / 0.400000000 |

Raw/shared coverage is 56.139% / 60.255%; final coverage 100%. Missing raw/shared time is not zero motion. Before first observed Spin at 121.73, raw has 24 low positive commands totaling 1.1925 s and 70 high commands totaling 3.4575 s; final has 33 low totaling 1.6800 s and 68 high totaling 3.3250 s. From 121.73 through failure, all received forward commands are zero. Unsupported positives therefore precede recovery in this analog, but terminal-braking versus corridor traversal and the frozen route demand remain unestablished.

## Deferred future M2 open-space check — DO NOT RUN YET

The harness directly supplies supported physical steps to the adapter shared input, with no Nav2/smoother process. This avoids inventing an inverse for smoother acceleration/deceleration ramps. It validates adapter -> policy -> truth only; an open-space PASS cannot clear the smoother startup/deceleration blocker.

| Desired physical vx | Inverse policy u |
|---:|---:|
| 0.12 | 0.05277871763339224 |
| 0.15 | 0.08206002183968573 |
| 0.2 | 0.13086219551684158 |
| 0.25 | 0.1796643691939974 |

Each point is 3 s zero -> 5 s target -> 4 s zero. Default script mode prints the plan with no ROS imports or launch. No output directory for a future live run has been created. Retained future command, deferred until low-speed transition evidence and route-compatible command semantics have been selected/validated (fresh isolated domain):

```zsh
cd /home/zyd/program/b2w-embodied-navigation-course
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export OPENBLAS_NUM_THREADS=1
/usr/bin/python3 scripts/m2_open_space.py --execute --ros-domain-id 195 --output-root /tmp/b2w-user-m2-open-space-01
```

Exactly one invocation. The harness launches its owned sim/SLAM stack and workspace adapter itself; it owns cleanup. No warehouse startup is part of this command. Keep its printed UTC directory, startup logs, manifest, native events and `m2-report.json`, including failure. To read the plan offline, omit --execute.

Report separates desired physical vx, inverse u, actual final-nav plateau, Float32 ONNX obs[6] plateau, truth steady vx, truth-desired error, lateral/yaw drift, final-zero-to-stop time/path, and candidate identity/hashes. Steady truth uses the fixed final 2 s, >=95% coverage and |slope|<=.02 m/s²; policy plateau requires authenticated 57-element layout. Invalid/missing data never earns PASS. Full-sequence clock/command coverage (>=95%) and corruption, exact schedule/plan, source stamps, normalized truth pose, frame, explicit readiness, active candidate and owned cleanup are checked.

Prospective diagnostic gates, fixed before data:

- DATA: every point valid/authenticated, candidate active, no unsupported event, cleanup passes.
- TRACKING: truth max_abs_error <=.010 m/s; RMSE <=.007 m/s; response monotonic.
- BASIC STABILITY: no sign/axis corruption; the prospective operational definition of obvious truth reversal is target vx < -.01 m/s; abs lateral <=.03 m and abs yaw drift <=.05 rad per 5 s target; final-zero-to-truth-stop <=.8 s and path <=.05 m.
- Stop definition remains native supported .5 s planar/yaw quiet; gaps do not establish a stop. A same-stamp zero-length stop path is 0 only with exact valid truth pose evidence.

No live validation result exists for this candidate. Production selection remains NONE. A full warehouse attempt remains blocked by the structural low-speed transition gap and unvalidated dynamic operating conditions, even if this direct-shared check passes. Sequence: LOW-SPEED TRANSITION -> choose/validate route-compatible command semantics -> ONE candidate open-space end-to-end check -> normal Nav2 integration check -> FULL warehouse route. No more fit-oriented experiments inside the validated domain.

## Previous-stage offline verification and integrity (historical)

Final guarded regression: **317 passed, 1 known live ROS test deselected**. Both rclpy.init and Context.init are forbidden in that test process; pytest plugin autoload disabled. RED/GREEN and independent review records are under `/tmp/b2w-m2-route-readiness/`. Audit repeat is byte-identical; git diff --check passes.

A preliminary broad test selection mistakenly included the existing ROS executor test. It attempted initialization and failed at read-only ROS log creation before node creation. No Gazebo, navigation or motion was started. The failed log is preserved and is not counted as validation. All subsequent regression runs explicitly exclude it and forbid ROS initialization.

310 protected tracked files are unchanged. 585 of 588 initial snapshot files are unchanged; the three intentional existing-file edits are PLAN, adapter and package dependency XML. Another 124 Protocol A capture files match the earlier pre-task snapshot. This proves protected route coordinates/full-route harness, A/B/C tools/captures/frozen results, ONNX/PT/controller/Nav2/SLAM/footprint integrity. No production configuration enables the candidate. Candidate live/physical validation is still pending; no commit/push.

## Corrective repair validation (2026-10-18)

The previous classification C is superseded by structural classification **B**, NORMAL_NAV2_M2_READY=false. C DATA VALID PASS and M2 held-out PASS remain unchanged; pure-forward scope and production_selection=NONE apply. See the current sole PLAN and LOW_SPEED_TRANSITION.md for the explicit deferred order.

Final guarded offline regression: **360 passed, 1 known live ROS executor test deselected**. Both rclpy.init and Context.init were forbidden; no ROS initialization occurred. Independent repair review found no remaining important findings after RED/GREEN fixes. Structural and historical audits repeat byte-identically.

**712 protected existing files** match the repair-start snapshot. Nav2 YAML equals committed HEAD; existing adapter source equals repair input. Ordinary range guard, default sequences and native analysis remain equivalent to HEAD. A/B/C raw/derived/frozen results, full-route harness/canonical coordinates, ONNX/PT/controller, Nav2/smoother YAML, SLAM and footprint remain unchanged. No production launch enables M2. Both future experiment roots remain absent. No commit or push.
