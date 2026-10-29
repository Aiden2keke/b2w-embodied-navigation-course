# Handoff — repository consolidation and G1–G5 admission

Repository root: this directory. Integration branch: `integration/consolidate-stage4-g1-g5-20261029`. Stage4 and G1–G5 source directories remain untouched. No GitHub push.

## Integrated

- Stage4: fail-closed final velocity guard, localization freshness and grid/contact worker; offline route selection, journal, registration, footprint/corridor and stop-evidence checks; their focused tests and candidate map/waypoint source. Existing launch defaults remain unchanged; `stage4_guard_manifest` is opt-in.
- G1–G5: owned recorder/admission harness, camera restoration and media verification; Gazebo GUI, saved pose and RViz views; verified stationary source captures and bounded-failure reports. Experimental continuous-route BT, demo Nav2 profile, coordinated-stop source and old `/tmp` output defaults are not wired into launch.
- SLAM, topic and TF sources match between the Stage4 snapshot and primary HEAD; no public interface or localization source substitution was made. Gazebo truth remains evaluation-only.

## Current result

There is no genuine complete Start→G1→G2→G3→G4→G5 run. Stage4's installed planner can find center paths, but its saved-map padded-footprint check rejects 128/309 G1→G2 and 127/439 G3→G4 samples. Its motion qualification function still raises `UNQUALIFIED_FOR_MOTION`; the route candidate is not deployable. The separate G1–G5 recording source has zero route attempts and no success video. In its one bounded rolling diagnostic, +0.05 m/s and +0.05 rad/s delivered for 0.9925 simulated seconds produced −0.146038 rad of body yaw before stop. There is no reviewed safe signed-steering mapping.

Do not start a route, retry this point, weaken a footprint/collision/stop threshold, or load the unadmitted presentation BT/config to bypass these results. The recorder's external review, exact deployment, envelope and control pins are absent from `course_results/g1_g5/reviewed_run.json`; the entry point stops before simulator launch.

## One route entry point

From the repository root, after a reviewed run manifest and matching SHA256 sidecar are supplied, the gated command is:

```bash
/usr/bin/python3 scripts/run_g1_g5_record.py --run --max-attempts 12 --max-total-minutes 120
```

At the current state it exits before launching the simulator because motion/controller admission is unresolved. It does not perform a genuine run today.

## Evidence and saved views

- Candidate source/map: `experiments/g1_g5/waypoints/20261019-v1/`; source coordinates and map pixels are preserved, paths and SHA256 bindings are repository-relative.
- Gazebo/RViz settings: `artifacts/video/manual_views/`; the imported RViz view is `TopDownOrtho`, target `base_link`, angle 0, scale 42.2995730952694. The imported camera values are preserved; local file hashes are in `artifacts/video/manual_views/SHA256SUMS`, and original source hashes are retained in the imported raw evidence.
- G1–G5 stationary captures are in `video/g1-g5-stationary-check-20261028/`; they are not route-success videos. Stage4 NavFn/Smac2D videos already present in `video/0c31f08e19ce51849f2c06b107e8a89b2ff55ef7/` are byte-identical to the Stage4 source copies. Existing October screencasts remain untracked.
- Imported raw measurements are retained under ignored `course_results/imported_evidence/`; `MIGRATION_SHA256.json` covers copied records. Do not commit files under `video/` or `course_results/`.

## Verification

- Offline focused regression suites: **589 passed, 10 skipped**. Skips are the sustained-turn manifest mutation tests, because the source checkout does not contain the exact baseline manifest pinned by those tests; this missing artifact also blocks that historical motion admission.
- The G1–G5 recorder dry run completed with zero attempts and wrote its report under `course_results/integration-dry-run/`. It launched no simulator and confirmed the missing reviewed run manifest, controller review and route envelope.
- `git diff --check` is clean. No Gazebo build or live simulation was run during this migration.
