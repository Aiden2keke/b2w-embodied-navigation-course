# Handoff — Actual normal cleanup PASS; bounded yaw NOT RUN

Single agent, `/tmp/b2w-navigation-ab-20261023`, recovery branch
`navigation/g1-g5-recovery-20261025`, session base exactly
`25043f96ee63807b04ab5f1a36d2dc9bd5a1faab`.
Final commit/ref/checksum: delivered bundle Git verification and git rev-parse HEAD.
Main reconciliation `51870e1` and local files preserved; Stage4 untouched.

Exactly ONE new stationary Gazebo session: domain211, unused output and launch
lease;46 source/install pins, native/server identities and26 shapes checked.
Actual normal cleanup PASS:628 exact-zero command samples,5.0825sim-second
measured LIO/truth rest, max .000255879m drift and2.739e-7rad yaw drift.
All19 registered identities/helpers/server exited within unchanged8s bound,
no fallback; localization ingress/thread cleanup PASS. Watchdog accepted normal
completion; fault handling remained armed, no new live fault injection.
Host PID/start-tick/token audit empty; domain lease reacquired/released.

Minimal explicit `--stationary-normal-cleanup` requires stationary containment
and uses the existing normal exit after zero/rest. Default fault-injection mode
and normal cleanup fix unchanged. New test RED→GREEN;174 focused tests PASS,
including33 precheck tests. Single-agent scoped review PASS, no independent
reviewer or yaw-control PASS claimed.

Actual yaw diagnostic NOT RUN: existing precheck runs forward+turn; .25 pulse
setting is not yaw-only. Its sampled clock loop/asynchronous Twist chain cannot
strictly enforce≤.25 simulated seconds (old1s pulse overshot2.5ms). No yaw-only
flag or alternative nonzero pulse, retries, route goal or hardware used. One
stationary session total, zero motion sessions. Original training commit/range
is not pinned in POLICY_SOURCE; current upstream training config is not proof
of this model's original range or convention.

Frozen-observation ONNX intervention: positive yaw changes wheel differential
in negative direction even at recorded pre-turn rest. Old +.05 request reaches
model+.08, then negative target/dq/gyro and truth yaw−.154184rad. At zero,
idle_hold overrides wheel targets while learned legs continue; positive wheel
feedback/gyro and+.151055rad rebound follow. Actual torque was not recorded;
all old contact entries lack wrenches. No mapping bug, global sign reversal or
sole rebound mechanism proven. No negation, vx scaling, gain/idle tuning or
policy replacement justified/applied.

Full concise snippets, chronology, limitations, commands and source identities:
`docs/YAW_CONTRACT_CLEANUP_20261026.md`.
Evidence: `artifacts/recovery/yaw-contract-cleanup-20261026/` in APRIL bundle,
plus old `live-qualification-short-motion-20261025/` and `yaw-cleanup-20261025/`.

Next session: establish a reviewed simulation-time command expiry and capture
existing named wheel targets/feedback/torques before the one bounded pure+.05yaw
≤.25s diagnostic from verified rest, with all original gates/watchdog/cleanup.
This is one transient diagnostic, not steady-state qualification. Controllable
yaw, directional stopping and route envelope remain the Start→G2 blockers.
145-pose7.213m Start→G1 path retained; actual Start→G2 and continuous Start→G5
video NOT DONE; .3m navigation and .05m staging tolerances distinct.
Commit/push reviewed completed scope; deliver ONE APRIL bundle+SHA256 and STOP.
