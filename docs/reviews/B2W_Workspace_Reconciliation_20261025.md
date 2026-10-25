# B2-W workspace reconciliation — 2026-10-25

This report supersedes the old working-tree PLAN.md NOW for workspace status.
This session reviewed and organized existing content; no runtime qualification,
simulation, navigation trial, merge, refactoring or policy tuning occurred.

## Git relationship and the 71 changes

Main is a normal checkout with its common Git directory at
`/home/zyd/program/b2w-embodied-navigation-course/.git`. It initially used
`improve/narrow-navigation` at `c629881c5c85da24eebfc09c2aa871a4e926825a`, equal
to its verified origin branch. That original branch pointer remains unchanged.
Main now uses `reconcile/preserve-main-20261025`, based directly on c629881.
Origin is `https://github.com/Aiden2keke/b2w-embodied-navigation-course.git`.
Its other registered worktrees are the Stage 4 checkout and two stale/prunable
/tmp registrations; none was changed or pruned.

`/tmp/b2w-navigation-ab-20261023` is a separate clone with its own `.git`, index
and sole registered worktree, without object alternates. Its branch and upstream
are `experiment/canonical-planning-20261025` and
`project/experiment/canonical-planning-20261025`; verified HEAD is
`557b856cb4c6240392a1e5e21ad653a6febb5fcd`. Its tracked/untracked status remains
clean. Its origin is the local Stage 4 checkout; project is the GitHub URL above.
Do not push that experiment branch to its local origin.

Antigravity's 71 paths were exactly 10 tracked modifications + 61 untracked,
with zero staged paths or tracked deletions. Ignored files were not counted.
Separate clones have independent working files and indexes; cleanliness does
not propagate. Main and canonical experiment diverge at e7f1eec (initially
8 main-only / 5 experiment-only commits). Main vs Stage 4 HEAD 438ab50 is
1 main-only / 18 Stage 4-only commits. Automatic integration would lose context.

64/71 dirty paths exactly match historical Stage 4 committed content. At its
current HEAD 438ab50, 44 still match, 20 have evolved and 7 are absent. All 61
untracked paths are absent from canonical experiment HEAD. Seven paths have no
Stage 4 historical match: one independent review report and six recordings.

## Closed offline work

- `672ba6435898500b20b522c4b6078c4be531bffd`: ten paths for the pinned
  endpoint-transform guard, method regression, activation and investigation.
- `d19a3f8411326901a4e6588f1ed0c8ae7384782b`: 32 paths for optional adaptive
  profiles, watchdog, speed recording, practical advancement and DWB observation.
  NavFn/DWB and strict advancement defaults remain. Practical completion does
  not assert strict FULL_ROUTE_PASS. These are offline-complete components;
  physical speed, fresh-map reproducibility and route success remain unqualified.
- `9dbdb08698f1dc07b504c5e8cd9868d9f8c91f5a`: five historical review archive
  paths, retaining their original limitations and verified sidecar checksum.
  These reports concern other revisions, not certification of this checkout.

Validation: 236 selected Python tests passed. Additional preservation checks:
165 passed / 1 skipped; their one socket-restricted editor test passed separately
outside the sandbox (1 passed). Native guard: original three expected failures,
patched eleven checks passed. Patch dry-run with fuzz=0 and offline loader,
header and ABI resolution passed. No course build or live stack was run.
One initial broad Python invocation was interrupted; default ROS log writes were
read-only. Corrected explicit tests used ROS_LOG_DIR under /tmp and the course
install environment. No full repository suite PASS is claimed. Default diff
whitespace checking flags two required blank context lines in the patch and two
Markdown hard breaks in an archived report; original bytes were retained.

## Preserved incomplete work

Twenty-four original paths remain dirty/untracked in place:

- `PLAN.md`
- `course_navigation/config/stage4_runtime_identity.json`
- `course_navigation/config/stage4_selection_policy.json`
- `docs/STAGE4_CANDIDATE_ROUTE.md`
- `scripts/analyze_candidate_route.py`
- `scripts/diagnose_stage4_window.py`
- `scripts/run_candidate_route.py`
- `scripts/stage4_route/__init__.py`
- `scripts/stage4_route/contract.py`
- `scripts/stage4_route/evidence.py`
- `scripts/stage4_route/ros_transport.py`
- `scripts/stage4_route/runner.py`
- `scripts/stage4_route/spatial.py`
- `scripts/stage4_smac_probe/CMakeLists.txt`
- `scripts/stage4_smac_probe/probe.cpp`
- `tests/test_stage4_route.py`
- `tests/test_stage4_smac.py`
- `tests/test_stage4_transport.py`
- `video/Screencast from 2026年10月18日 12时10分32秒.mp4`
- `video/Screencast from 2026年10月18日 12时10分32秒.webm`
- `video/Screencast from 2026年10月18日 16时07分33秒.mp4`
- `video/Screencast from 2026年10月18日 16时07分33秒.webm`
- `video/Screencast from 2026年10月19日 16时24分49秒-7fb302f99c8e2c3ac445c2357618eb5906d0a6dd.mp4`
- `video/Screencast from 2026年10月19日 16时24分49秒-7fb302f99c8e2c3ac445c2357618eb5906d0a6dd.webm`

Dependencies outstanding: exact candidate V1/export and saved-map identities
in the selection policy; fresh same-session registration and source uncertainty;
dynamic costmap/path revision binding; measured stopping/braking, complete body
contact coverage and terminal evidence. Runtime identity is machine/build-specific.
The old runner always passes safety_complete=False; CLI/segment/transport
contracts need comparison with later hardened Stage 4 revisions. Do not execute
these older prototypes based on this review. PLAN is preserved history, not a
new authorization. Videos are primary historical evidence, not build waste.

Ignored build/install/log/cache remain in place. `.deps`, `.venv` and downloaded
third_party assets are runtime/model dependencies. `course_results/` holds
historical main metrics; experiment `artifacts/` holds separate evidence. None
was deleted or staged. Existing install trees do not prove current source loading.

## Historical capability and next session

Main retains legacy NavFn/DWB and Smac diagnostics, the canonical route/strict
harness, M2 calibration and held-out validation, transition diagnostics, waypoint
editor, Pipe V0, SLAM/policy assets and historical metrics. Several of these are
absent from canonical experiment HEAD; wholesale replacement is inappropriate.
No verified canonical G1→G2 successful baseline was found. The retained Oct 16
Start→G1 succeeded at .064844853 m under .3 m navigation tolerance; G1→G2 was
false/replaced. Older Oct 14 successes used different goals. This cannot prove
that G1→G2 never succeeded elsewhere. The .05 m staging gate remains distinct.

Safest next branch: `navigation/g1-g5-recovery-20261025`, created from the final
committed reconciliation HEAD in a fresh checkout/worktree. Keep the 24 inherited
paths here. Compare and selectively port required canonical-planning/TF fixes;
do not merge entire experiment or Stage 4 branches. Authenticate the intended
G1→G2 baseline before claiming recovery or full G1→G5 delivery.

Audit evidence is in `/tmp/b2w-reconcile-20261025/`: initial/final status,
per-path hashes, ignored inventory, exact staged patches, tests and an untracked
source snapshot. The single delivered APRIL bundle under
`/home/zyd/APRIL_REVIEW_BUNDLES/` records the complete 71-path classification,
all final commit/push SHAs and this handoff's commit. Stop after delivery;
further development begins in a fresh Codex session.
