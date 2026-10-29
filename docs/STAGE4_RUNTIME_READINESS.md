# Stage4 runtime readiness — consolidated record

This file replaces the copied worktree-specific runbook. Historical source
commands depended on deleted or separate `/tmp` worktrees and must not be used.
Current integration status and the only G1–G5 entry point are in [HANDOFF.md](../HANDOFF.md).

## Current decision

Stage4's navigation and final-command guard are integrated for continued offline
evaluation. The guard remains fail-closed: `stage4_qualification.require_motion_qualification()`
raises `UNQUALIFIED_FOR_MOTION` until deployment, map, controller, stopping and
route evidence satisfy the qualification contract. No live motion was authorized
or started during consolidation.

The selected candidate and source map live at
`experiments/g1_g5/waypoints/20261019-v1/`. Its filled padded-footprint audit
rejects 128/309 samples on G1→G2 and 127/439 on G3→G4. The offline center path
does not override those failures. See [the candidate report](STAGE4_CANDIDATE_ROUTE.md)
and [the experiment artifact policy](EXPERIMENT_ARTIFACT_POLICY.md).

## Revalidation

Use the build and test setup in [CODEX_GUIDE.md](../CODEX_GUIDE.md), from this
repository root. Keep generated logs and evidence under `course_results/`.
Run the offline Stage4 safety and route tests before considering any live
session. Zero-motion observation, where separately authorized, does not qualify
nonzero motion. A new live session needs fresh review of the exact installed
identities, localization and TF timing, contact coverage, stop/abort bounds and
route geometry. Retain failed evidence and do not automatically retry.

Old experiment records that were carried over are archived under
`course_results/imported_evidence/`; their original metrics are not current
qualification evidence. Imported recovery artifacts are kept under ignored
`artifacts/recovery/` and do not authorize execution against current source.
