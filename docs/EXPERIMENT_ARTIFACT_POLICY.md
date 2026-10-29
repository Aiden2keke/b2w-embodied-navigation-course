# G1–G5 repository artifact policy

User policy received 2026-10-20. Do not create symlinks or move existing evidence
during development. Preserve original sessions and evidence unchanged.

Only after genuine, independently evidenced G1–G5 navigation succeeds, curate
useful successful artifacts as copies under `experiments/g1_g5/<run-id>/`:

- Verified occupancy map image and metadata.
- Executed waypoint/route and Nav2 configuration snapshots.
- Representative complete ROS 2 bags, including metadata and every referenced
  recording file; retain complete original recordings separately.
- Navigation metrics, trajectories, plots and demonstration media.
- A concise README and SHA256 manifest connecting every artifact to its original
  session, configuration and commit. Record actual acceptance scope, repeat count,
  source paths/hashes, and any derivation commands.

Verify copied bytes against the original evidence and review provenance before
staging. Use Git LFS for large bags/videos where available and verify the staged
objects are LFS pointers. Never silently commit oversized binaries or generated
caches; if large artifact storage is unavailable, leave those files out of Git
and report the limitation explicitly.

Review the staged artifact scope and reproducibility, then create a scoped local
commit on the experimental branch only. No automatic push or merge.

Failed, partial, synthetic or inconclusive runs must not be archived as successful.
If the campaign stops before success, retain diagnostic evidence separately and
report the blocker. Do not create a successful-run archive merely to satisfy the
directory convention.

Current campaign: **BLOCKED_GEOMETRY**, zero live trials, no verified G1–G5 run.
There is no qualifying successful artifact set to curate. Diagnostic evidence
remains under `~/b2w-course-evidence/unattended/runs/20261019T154246Z/`.
