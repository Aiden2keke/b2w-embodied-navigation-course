# Handoff — canonical planning evidence closed

Project remote: https://github.com/Aiden2keke/b2w-embodied-navigation-course.git
Remote branch: `experiment/canonical-planning-20261025` (local remote alias `project`).
Verified source milestone commit: `48b7b8fd302f684c88263871027ba1ee04abca7f`.
The branch tip additionally contains this documentation-only handoff commit; obtain
its exact SHA with `git rev-parse HEAD` and verify against `git ls-remote project refs/heads/experiment/canonical-planning-20261025`.
Existing `origin` is the local staging clone and was preserved.

**DONE:** real145-pose,7.213m canonical Start→G1 Smac path captured.
**FAILED:** Owner session at TF age0.5025s>0.5s; terminal planner action status unavailable.
Initial rotation/0.5m prefix geometrically clear; motion admission remains blocked.
G1 body/rotation map coverage and Profile-B stopping/tracking/localization bounds
remain unresolved. Historical0.0648m G1 arrival passes0.3m navigation tolerance,
fails0.05m staging; neither changed. Start→G2 and continuous Start→G5/video NOT DONE.

Closure checks:367 offline tests PASS,1 DDS integration test deselected after an
initial read-only ROS log-path failure; native observer1/1 PASS;46 source/install
pairs and26 collision handles verified offline. Single-agent self-review PASS
for archival/handoff, not navigation readiness. No Gazebo/motion at closure.

Key files: `scripts/warehouse_ab_trial.py`, `scripts/owner_tf_delivery.py`,
`scripts/warehouse_route_admission.py`, `scripts/warehouse_ab_supervisor.py`,
`scripts/run_warehouse_route.py`, `scripts/record_warehouse_video.py`.
Read sole `PLAN.md` NOW and `docs/CANONICAL_PLANNING_MILESTONE.md` for exact tests,
external input dependencies and evidence hashes. Keep the prepared workspace:
`/tmp/b2w-navigation-ab-20261023`. Git excludes raw/generated `artifacts/`; a clean
clone alone cannot run the evidence-dependent tests or identity-pinned owner.

Read-only/offline input command (no ROS node or simulation):
```sh
ROS_DOMAIN_ID=198 scripts/run_warehouse_ab.sh --profile B --planning-only --check \
  --deployment-manifest artifacts/recovery/first-motion-control-risks-20261025/reviewed-deployment.json
```
Evidence: `artifacts/recovery/final-canonical-planning-20261025/` and the matching
`20261025_final-canonical-planning.md` APRIL bundle in `/home/zyd/APRIL_REVIEW_BUNDLES/`.
Owner source remains SHA256 `749a41692b21ff843e77a96003404f54a2fd8ac59b3132a6e1e641e701432fed`.

**NEXT exact task:** focused Owner TF/Odom ingestion/executor scheduling diagnosis
using the recorded trace; distinguish middleware ingress from callback delay.
Acceptance: coherent localization snapshots served within unchanged0.5s limits,
focused replay/order/fail-closed tests pass, planning-only still cannot issue
nonzero commands or NavigateToPose, and owned cleanup/watchdog remain intact.
Do not continue startup-order patches or require another path-capture session
merely to establish that a real path exists. No TF repair was begun at closure.

Safety: no unknown-as-free, invented envelope, collision bypass, freshness grace,
observer truth as control, policy change, broad kills or force push. Preserve
Smac+DWB+rl_sar, canonical world/goals and precheck-only1.5m gate. Prior one-shot
Gazebo authorizations are consumed. Every new Gazebo session needs explicit fresh
authorization plus unused domain/lease/output and deployment/watchdog checks;
calibration, route motion and hardware require separately authorized scope.
