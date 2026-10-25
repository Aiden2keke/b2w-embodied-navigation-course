# Handoff — Cleanup fixed offline; tracking still blocks navigation

Single agent; recovery branch `navigation/g1-g5-recovery-20261025` in
`/tmp/b2w-navigation-ab-20261023`, based exactly on clean `557b856cb4c6240392a1e5e21ad653a6febb5fcd`.
GitHub `project` experimental ref matched base; remote HEAD `66db5152`.
Final commit/ref/checksum: use delivered bundle Git verification and git rev-parse HEAD.
Main `51870e1` and 24 local paths preserved; no main/Stage4 wholesale merge.

Normal shutdown defect proven: ros2-run catches wrapper-only SIGINT and waits
for unsignaled helper. Recorded six survivors after stack exit;8s bound fails.
Use existing token/start-tick checked `owner.send(SIGINT)` for normal shutdown;
keep8s deadline, survivor failure, watchdog and fallback. Preserve normal_cleanup
snapshot separately from fallback cleanup. Real installed-wrapper reproduction
confirms PID-only signal failure and owned-signal success, unrelated PID preserved.
Two regression tests RED→GREEN;25 precheck and185 related safety tests PASS.
Single-agent review PASS applies to cleanup code/evidence, not live qualification.

Yaw NOT FIXED: model recorded+.08 from+.05 request; body gyro mean−.154735rad/s,
truth quaternion delta−.154184rad.95 replay pairs agree within1.02e-6.
Negative wheel response already exists below the positive model command.
Coast reverses+.151055rad. Forward pose-derived mean/peak .099100/.141945m/s.
Exact model training convention vs transient/contact behavior unresolved; no
sign/gain/vx/policy tuning justified or applied. `.5s` TF/odom freshness and all
physical/contact/collision/stop/watchdog checks retained; no active freshness fault.

Details/snippets: docs/YAW_TRACKING_CLEANUP_20261025.md.
Old raw evidence: artifacts/recovery/live-qualification-short-motion-20261025/.
New replay/reproduction/RED/GREEN/safety results: artifacts/recovery/yaw-cleanup-20261025/.
APRIL bundle includes both; commit pushed to project recovery branch, no merge.
No Gazebo, motion, route goal, retry or hardware this session. STOP after delivery.

NEXT SESSION requires new authorization: stationary normal-cleanup check first;
resolve training contract offline before choosing a controller change. Proposed
smallest motion diagnostic is one pure+.05rad/s pulse≤.25s then zero, measured
rest and normal cleanup with all existing gates; no retry. It is not a tracking
qualification. Keep145-pose7.213m Start→G1 path. Actual Start→G2 and continuous
Start→G5 video remain NOT DONE; .3m navigation vs .05m staging remains explicit.
