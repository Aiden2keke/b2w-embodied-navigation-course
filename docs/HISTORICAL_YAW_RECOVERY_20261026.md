# Historical yaw recovery candidate — 2026-10-26

Functional yaw remains **unqualified**. This change prepares one bounded
historical-amplitude reproduction, with the existing controller/configuration.
The live preflight was BLOCKED before launch by unrelated GPU compute-list
PID3946592 (`mpv`). No simulator, pulse, retry, stationary session or goal.
Domain217 lease was available/closed, no competing simulator; DDS discovery
and live physical admissions were not reached. No other job was disturbed.

## Historical baseline: summary verified, exact run not fully attributable

The original tracked evidence is
`docs/validation/2026-10-10-rl-sar-fortress.json`, first committed in
`e0537d879459fd84fe666e7918bb906224391779` (2026-10-11). It records:

| Phase | External yaw | Sim duration | Truth yaw delta | Maximum tilt |
|---|---:|---:|---:|---:|
| turn | +.25 rad/s | 8s | +2.2675922676rad | 3.5689528deg |
| stop_after_turn | 0 | 10s | -.1910896515rad | 5.8206890deg |

The summary therefore reports **+2.0765026161rad retained**, about91.57% of the
turn delta. This is not the near-total rebound seen at small yaw commands.
The report contains phase aggregates and final joints, not the raw sampled
truth trajectory. Its harness used `/sim/body_state` quaternion unwrapping.

Its recorded ONNX hash `da31c0c1820b893608e2d02746bbada16d0abb5de6116277e37ccfb02024c564`
and configuration hash `af815d98bc2cc2c0ca61771e4ccb680843537f2aa2e84b61cf4fd9badadf3fff`
match current files. Controller source, description generator, confined world,
fault profiles and physics-check harness in that first evidence commit are
byte-identical to current. Controller source SHA256:
`0f88178ffd3959a8a1cce449a450cd69bc784d08814848358d651b617fa0d339`.

The config uses idle_hold=true, yaw gain1.6, named FR/FL/RR/RL joints, leg PD and
wheel velocity-error torque (kp0,kd1,20Nm clamp). Historical mode=sim published
commands directly; today's precheck uses the adapter with immediate explicit
zero forwarding. Controller, policy and gain need no restoration/rebuild.

Historical world was confined, default spawn(-6,0,1), normal slip0, a flat box
floor with no explicit friction coefficients. Current candidate keeps the
required Warehouse and observed-free pocket at(0,-1), not that old spawn/world.
The runtime log identifies a different host (`xavier`) and policy path but no
runtime Git HEAD/controller binary/engine hash or material/load/slip proof.
No raw trajectory or attributable October10 video was found in searched local
validation/results/video evidence, including the preserved main workspace.
Its remaining uncommitted files were read only. Thus the historical result is
source-consistent summary evidence, not a fully pinned/recomputable live trial.
The old8s turn also reached .251844m excursion, above today's .25m veto.

## Current finite-state operating scan

Deterministic ONNX command interventions used138 recorded states in eight
initial-rest, active, early-post-zero and final-rest windows from positive and
negative trials. Model yaw grid spans±.60, including historical±.40 and failed
±.08, with denser points near sign changes. External yaw=model yaw/1.6.
Physical sensors and previous actions stay fixed. Raw wheel targets, idle-applied
wheel targets, absolute and relative-zero right-minus-left differential,
per-wheel targets, leg actions/offsets and sampled finite-difference sensitivity
are saved in `yaw-operating-region.json` in the new APRIL evidence.

At every selected state, both absolute and relative-zero differential signs
are correct at tested negative points[-.60,-.50,-.40,-.36,-.32,-.28,-.24,-.20]
and positive+.60. These are **discrete sensor-clamped observations**, not a
continuous or physically qualified operating interval. At+.40 the rest states
are correct, but some previous positive-turn/post-zero states remain biased
negative. Small±.08 are locally reversed. Raw zero-command differential is
biased about-1.5rad/s at rest, while applied idle wheel targets are exactly0.
Leg targets change with yaw and restore at zero. Neither global inversion nor
an unconditional minimum-yaw rescale is supported.

## Minimal candidate and validation

`--precheck-axis historical-yaw` selects exactly one **+.25rad/s** pulse,
explicit nominal duration at most **.5sim s**, then exact zero and full measured
rest. It cannot be used outside precheck; omitted/default2s or longer durations
reject. The other modes/defaults and controller/native/policy/config are intact.
No hidden nonzero stop command or route use is added.

This is option1, reuse the historical command/configuration, before inventing
wheel-yaw control. Historical retained-yaw summary plus current rest-state
positive+.40 differential makes it a credible bounded physical test; offline
inference cannot predict whether .5s will overcome transient posture/traction.
All existing1.5m pocket, fresh actual map, initial rest, TF/odom/contact gates,
.25m/.35rad excursion, .25m/s/.4rad/s speed vetoes, watchdog and cleanup remain.
No separate stationary session is required: no controller/config/native changed;
mandatory initial measured rest still applies.

Two new regressions failed on the original default two-pulse behavior/CLI.
Final **66 focused precheck tests PASS**, including duration rejection and12
admission/physical/stop/watchdog faults for the new mode. Original deployment
check PASS46 source/install pins, native pins,26 collision handles. No broad
suite, model changes or system upgrades. Single-agent scoped source review PASS;
this is not an independent physics review or functional-yaw PASS.

Next concrete action, after resource availability and explicit resumption:
check a fresh host/domain/lease/output once, then use `scripts/run_warehouse_ab.sh`
with profileB, `--execute --precheck-only --authorize-motion precheck-only`,
`--precheck-axis historical-yaw --precheck-pulse-s .5`, spawn0,-1 and the existing
reviewed deployment manifest. Capture actual interval, signed body rotation and
retention after rest. No automatic retry, longer pulse, route or hardware.
Only after physical yaw evidence: signed Profile-B directional stopping/tracking
and G1 arrival/rotation envelope, independent review and Start→G2 authority.
Start→G2/Start→G5 video remain NOT DONE. Existing PLAN/HANDOFF edits are preserved.
