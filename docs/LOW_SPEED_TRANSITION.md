# Route-relevant low-speed transition diagnostic — preregistered, NOT RUN

The sole current roadmap is [PLAN.md](../PLAN.md). System success remains FULL_ROUTE_PASS. Current integration verdict is **B. LOW_SPEED_SUPPORT_BLOCKER**, **NORMAL_NAV2_M2_READY=false**. C DATA VALID PASS and M2 held-out PASS remain closed; production_selection=NONE. This document prepares one diagnostic session and defines its interpretation before any observations exist.

## Question and operating condition

What physical forward response exists between exact zero and the current validated moving branch under the unchanged downstream chain? All requests are pure forward, vy=wz=0. They are **policy input u**, not corrected desired physical vx. No M2 inverse is active. No affine fit through zero, bridge, clamp, pass-through rule, interpolation or hysteresis is selected for production.

Current `nav2.yaml` OPEN_LOOP smoother derives .025 m/s per nominal tick from .5/20. Startup toward .15 traverses .025/.050/.075/.100 LOW values before .125 SUPPORTED. Deceleration from .125 traverses the same gap. This structural proof is independent of frozen-route historical timing. See [M2_ROUTE_READINESS.md](M2_ROUTE_READINESS.md) and `scripts/smoother_m2_support.py`.

## Runner range, idle_hold and measurement audit

- `velocity_step_core.make_sequence` and the ordinary runner CLI retain the existing [.04,.40] experiment range and duplicate-point rejection. Historical source/docs establish this as an **experiment-range guard, not a safety threshold or proven minimum attainable speed**. Only the dedicated transition diagnostic opts into [0,.04] and repeated anchors. No controller or production range is changed.
- `rl_sar_controller.cpp` evaluates `idle_hold_ && (... abs(command_[0])<.01 && abs(command_[1])<.01 && abs(command_[2])<.01)` using the consumed **double** command, before creating Float32 `obs[6:9]`. `rl_sar.yaml` enables idle_hold. Idle zeroes wheel actions while retaining learned leg balance; it does not prove exact physical zero.
- `.005` is below the strict threshold and should expose zero wheel-action targets. Its Float32 debug value is `.004999999888241291`, distinguishable from exact zero. `.010` equals the double threshold and does not satisfy `<.01`, even though its Float32 observation is `.009999999776482582`. Debug rounding must not be used to infer which branch executed.
- Existing controller timing is 200 Hz torque / nominal 50 Hz policy; requests/smoother are 20 Hz. The fixed final 2 s steady window nominally contains 40 command and 100 policy observations. Native source timestamps, coverage and sample counts determine actual validity; no rounded plot is evidence.
- Truth/SLAM fixed-window means can distinguish command-level signals but physical detectability remains unmeasured. Prospective zero/moving measurement resolution is **.005 m/s**. Responses inside that zero band remain unresolved.
- The legacy .02 m/s truth-stop definition would be too coarse for this diagnostic. Its explicit manifest sets `truth_stop_speed_mps=.005`; the same supported .5 s planar/yaw quiet window and |yaw_rate|<=.05 rad/s apply. Ordinary experiments and the retained M2 open-space harness keep their .02 default. No controller stop behavior changes.

The plan CLI derives and hashes idle_hold/timing facts from current controller/YAML and stops if the expected layout is unavailable.

## Frozen sequence

| Trial | Target policy u | Purpose |
|---:|---:|---|
| 1 | 0 | Separate exact-zero reference |
| 2 | .005 | Sub-threshold command, instrumentally distinct from zero |
| 3 | .010 | Strict idle_hold boundary |
| 4 | .015 | Just above boundary |
| 5 | .020 | Intermediate transition |
| 6 | .030 | Approach B input endpoint |
| 7 | .040 | Current B lower endpoint |
| 8 | .010 | Same-session boundary repeat after moving branch |
| 9 | .040 | Same-session moving-endpoint repeat |

Every block is 3 s zero -> 5 s target -> 4 s zero, including the all-zero reference block. Nominal sequence duration is **108 simulation seconds**; readiness and cleanup add time. Exactly one fresh owned session, no retries, exclusions, tuning or adaptive points. Two repeats test same-session consistency, not independent-session repeatability.

Chain: requested u -> raw -> unchanged smoother -> shared -> default-off adapter -> final nav_vel -> unchanged rl_sar/ONNX -> wheels/physics. Maximum target .04 has a nominal .025 startup intermediate, preserved in native data; measurements use authenticated final plateaus, not an assumed perfect step.

## Prospective DATA and stability gates

The report records requested u, raw/shared/final command plateaus, actual Float32 ONNX command, truth and SLAM steady vx, slope/coverage, named wheel targets/actual/error/asymmetry, wheel/total torque peak ratios, target lateral/yaw drift and stopping metrics. It preserves every trial and both repeat anchors in distinct files.

DATA requires all nine trials with no exclusions: exact preregistered plan/sequence and source hashes; current unchanged controller/geometry, Nav2 YAML and ONNX policy identity; inverse disabled; completed readiness/runtime/owned cleanup; native clock without epoch rollback; finite, source-stamped, normalized base_link truth/SLAM poses; pure-forward plateau authentication; raw/shared/final plateaus equal requested u within 1e-6; actual ONNX plateau equal requested input within 1e-6; >=95% supported coverage. Whole-sequence command/policy/clock/torque streams must meet coverage, with existing native gap limits. Named wheel/torque instrumentation must be available for every steady window. Idle wheel-action zero override must be observed for sub-threshold points.

Fixed final 2 s steady windows require >=95% supported coverage and |vx slope|<=.02 m/s² for truth and SLAM. Missing/invalid data gives C_UNSUITABLE_OR_INSUFFICIENT and cannot select a branch. Capture raw SHA256 is checked on replay; replay preserves raw/manifest bytes. Execution returns nonzero when evidence is invalid, even if the motion runner completed.

Per 5 s target, abs lateral displacement <=.03 m and abs yaw drift <=.05 rad. Final-zero-to-truth-stop <=.8 s and path <=.05 m, using the diagnostic .005 m/s quiet threshold. Incomplete stopping/pose coverage cannot establish stability. Zero reference abs steady vx <=.005 m/s. Repeated .010 and .040 truth means differ by <=.010 m/s. Both .040 observations must agree with frozen B's `.1068173727577565 m/s` reference within .010 m/s; otherwise changed conditions/non-repeatability remain unresolved. These are prospective diagnostic gates, not newly claimed historical constants.

## Interpretation before data

| Outcome | Required observed evidence | Permitted conclusion |
|---|---|---|
| A_CONTINUOUS_BRANCH_COMPATIBLE_ON_TESTED_GRID | DATA/stability/reference/repeat gates pass; response monotonic within .005 m/s resolution; at least one resolved moving response >.005 and <=.020 m/s; largest adjacent observed physical jump <=.040 m/s | Compatible with a controllable near-zero branch on this sampled grid. Arbitrary continuity is not proven; further command-semantics validation is required. |
| B_DEAD_ZONE_MINIMUM_MOVING_SPEED_EVIDENCE | Same validity/stability gates; an initial sampled zero/dead-zone prefix followed by resolved stable motion; minimum observed moving response >=.020 m/s; A criteria do not hold | Report dead-zone and **minimum observed stable moving speed**, not a proven global attainable minimum. A discrete stop/move policy may be needed, requiring separate validation. |
| C_UNSUITABLE_OR_INSUFFICIENT | Missing/invalid evidence, unstable/drifting response, reversal/nonmonotonicity beyond .005 m/s tolerance, repeat/reference mismatch, or neither sampled interpretation is resolved | No suitable route-control transition has been established. Preserve every failure; no production bridge or inverse is selected. |

The zero-reference mean is used to decide whether a response exceeds the .005 m/s resolution band. `minimum_observed_moving_vx` remains the actual truth mean; its increment relative to the zero reference is reported separately, without silently subtracting the baseline from physical speed.

Exact observed monotonicity is reported separately from the .005 m/s resolution test. Finite samples cannot prove a discontinuity between every untested pair or a global minimum attainable speed. Every outcome retains production_authorized=false, bridge_selected=false, continuity_proven=false, production_selection=NONE and NORMAL_NAV2_M2_READY=false. No M2 accuracy refit occurs.

## USER-RUN command — prepared only

Use one fresh session and isolated domain. The harness owns startup and cleanup; do not start another simulator for this command.

```zsh
cd /home/zyd/program/b2w-embodied-navigation-course
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export OPENBLAS_NUM_THREADS=1
/usr/bin/python3 scripts/low_speed_transition.py --execute --ros-domain-id 196 --output-root /tmp/b2w-user-low-speed-transition-01
```

Omit `--execute` to print the offline plan. Do not invent the future UTC directory. Preserve the printed directory, launch logs, manifest, events.jsonl, all per-trial artifacts and low-speed-transition-report.json, including failures. Offline replay after that USER-RUN capture:

```text
/usr/bin/python3 scripts/low_speed_transition.py --analyze ACTUAL_PRINTED_RUN_DIRECTORY
```

Next sequence is **LOW-SPEED TRANSITION -> choose/validate route-compatible command semantics -> ONE candidate open-space end-to-end check -> normal Nav2 integration check -> FULL warehouse route**. Exact G1/G2 inverse-OFF capture is not needed to establish the low-speed blocker. The retained `.12/.15/.20/.25` direct-shared M2 check remains deferred and cannot clear the smoother blocker. No C2/C3 and no in-domain RMSE campaign.
