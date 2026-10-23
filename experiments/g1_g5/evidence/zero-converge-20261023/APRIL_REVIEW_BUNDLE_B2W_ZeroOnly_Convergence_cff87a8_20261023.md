# APRIL Review Bundle — Warehouse ZeroOnly convergence

Record commit / verified origin: `cff87a8fef98d5d58b1b7bf2ed05a6e6c2b94263` on `experiment/unattended-g1-g5-20261019`.
Runtime source checkpoint: `0c83ab4a513143907ab8ba964cfe5c07a5454153` (unchanged production bytes).

# Warehouse ZeroOnly convergence — 2026-10-23

Full ZeroOnly **FAIL** after **one actual Gazebo launch** (of at most three authorized). The run reached coherent bootstrap with all three prepared grids and started the reviewed watchdog and probe. The watchdog correctly inhibited on a genuine stale health record; its underlying callback/host scheduling cause remains unproven. Work stops under the user's unexplained safety-critical fault rule. No second launch or speculative source change was made.

The secure-input setup issue is resolved for this attempt: a fresh session input was written using the existing `write_control()`, verified regular/non-symlink, UID1000, mode0600, read back by `read_control()`, and passed v2 validation on that exact file before launch. This task had no prelaunch validation failure. Prior attempts and evidence are preserved.

## Exact first fault

- `Watchdog.step()` at `course_navigation/course_navigation/stage4_control.py:112` rejected `health.age_gt_150ms`.
- Sampled monotonic wall `88017.801452404005` minus original callback wall `88017.609832271002` = **191.620133 ms**, limit150ms.
- Rejected sequence696; session `zero-converge-44d7085ccef54ab1a88d3d32bad4a25e`; adapter PID792493/start8798090. Owner heartbeat timestamp `88017.788914663` remained recent. Exact JSON is in `attempt-1/raw/session/watchdog.log`.
- Sequence697 next started at88017.791405108, 181.573ms after sequence696 started; its record was not available to the watchdog at the fault. The original callback timestamps were not refreshed or tolerated.

The live map was800×800; map stamp27 arrived near the dispatch gap. This is correlation only. Existing publication timing does not capture the full between-callback wait or an executor/OS stack at the stall. The offline same-size serializer check measured48.794ms total and27.719ms maximum scheduler gap, insufficient to reproduce or attribute the191.620ms violation. A streaming candidate preserved the hash but cost130.399ms; it was not applied. No deadline/threshold/guard bypass is justified.

## Evidence and acceptance

- Input, installed adapter/control, standalone watchdog, 35 copy identities and pinned LIO/GTSAM/controller/world/contact configuration passed preflight; domain197 idle, owner isolation valid, hard nonzero qualification still rejected motion.
- Final six-axis Twist: **699 observed, 0 nonzero**. No motion lease, command goal or nonzero publisher was issued.
- LIO: 144 positive integration steps; no recorded LIO fatal event. SLAM odometry104 messages over10.300s, maximum localization displacement0.000522m. These observations do not constitute full stationary acceptance.
- Contact delivery: 534 paired-message candidates and534 inventory records, sequences1..534, contiguous=True, forbidden flag=False. Probe endpoint closure was not accepted.
- Probe **FAIL**; first accepted sample/end/contact closure absent, probe cleanup **FAIL** after supervisor shutdown. Only initial planner runtime parameters were verified; no full LIO/TF/Nav2/graph acceptance is claimed.
- Session owned cleanup **PASS**; postcheck verified all27 recorded PID/start identities gone, no token survivor, domain197 empty apart from cleanup node, NVIDIA compute-process query empty. Passive observer exited0. Harness live stack47.806s, wrapper48.611s.

## E1 and next action

E1 offline implementation did not start because the requested full ZeroOnly PASS condition was not met. The existing E1 plan remains unchanged: scoped bootstrap/motion qualification, independent owned abort/containment, and local sensor-only clearance/registration/stopping evidence remain necessary; nonzero interlocks stay closed.

The next useful work is to reproduce/attribute the adapter's between-callback dispatch delay through the actual grid ingress/worker and executor path, or capture a bounded first-stall scheduling snapshot. Apply only an evidence-supported correction, then targeted tests and matching copy rebuild before a fresh authorized launch. Do not repeat the identical trial or relax150ms.

## Records

Base HEAD `0c83ab4a513143907ab8ba964cfe5c07a5454153`, experiment branch only. No production source changed in this task. Secure input/preflight/runner copies, observer stream, exact first fault, probe SQLite/seal, ROS logs, benchmark and cleanup proof are under `experiments/g1_g5/evidence/zero-converge-20261023/attempt-1/raw/`. `summary.json` is the compact machine-readable record; raw data are Git-ignored and indexed. Original checkout/Pipe untouched.


## Exact first fault

```json
{
  "event": "stage4_watchdog_first_fault",
  "failed_predicate": "health.age_gt_150ms",
  "health": {
    "adapter": {
      "pgid": 792418,
      "pid": 792493,
      "start_ticks": "8798090",
      "uid": 1000
    },
    "sequence": 696,
    "session_id": "zero-converge-44d7085ccef54ab1a88d3d32bad4a25e",
    "wall": 88017.609832271
  },
  "health_age_s": 0.19162013300228864,
  "owner": {
    "sequence": 1290,
    "session_id": "zero-converge-44d7085ccef54ab1a88d3d32bad4a25e",
    "wall": 88017.788914663
  },
  "reason": "adapter_publication_stalled",
  "sampled_wall": 88017.801452404
}
```

## Runtime hashes and validation

The exact new input was securely atomically written at mode0600 and passed read_control and v2 validation before launch. All35 current copied Python/config/launch files matched. Standalone watchdog argv and adapter argv were captured; LIO executable SHA and GTSAM mappings were captured from the actual owned process. Hard nonzero qualification still rejected motion. No source edits, threshold changes, additional unit suite or rebuild was needed because deployed bytes remained current. Historical focused82-pass tests remain historical; this task's new verification consists of exact-input/pinned preflight, live evidence, cleanup and the negative serializer benchmark.

```json
{
  "course_navigation.velocity_adapter": {
    "path": "/home/zyd/program/b2w-embodied-navigation-course-stage4/experiments/g1_g5/evidence/watchdog-firstfault-20261023/artifacts/install/course_navigation/lib/python3.10/site-packages/course_navigation/velocity_adapter.py",
    "sha256": "dc97d0445ec790fc0b5c2d171070f38cb9ad5e5a7d91e01a5fb9cdf86f6136c8"
  },
  "course_navigation.stage4_control": {
    "path": "/home/zyd/program/b2w-embodied-navigation-course-stage4/experiments/g1_g5/evidence/watchdog-firstfault-20261023/artifacts/install/course_navigation/lib/python3.10/site-packages/course_navigation/stage4_control.py",
    "sha256": "4c9e32b169dbe8d0ced5ea83cbcc8bed1ca99c96c139cac5c20c74d53e2525be"
  }
}
```

```json
{
  "watchdog": {
    "path": "/home/zyd/program/b2w-embodied-navigation-course-stage4/scripts/stage4_safety_watchdog.py",
    "sha256": "eef423424643bfb39ae4898ddcf14e6ff3c048892133417601be0605b3380475",
    "launched_by": "scripts/stage4_session.py"
  },
  "pinned_artifacts": {
    "experiments/g1_g5/evidence/startup-repair-20261021/artifacts/lio-build/lio_sam_imuPreintegration": "3bd8109f580ceac90670f5d12e57267192731e2bfd2af46eb1740d08672e50f5",
    "experiments/g1_g5/evidence/startup-repair-20261021/artifacts/lio-diagnostic-prefix/lib/lio_sam/lio_sam_imuPreintegration": "3bd8109f580ceac90670f5d12e57267192731e2bfd2af46eb1740d08672e50f5",
    "experiments/g1_g5/evidence/startup-repair-20261021/artifacts/native-build/libstage4_contact_observer.so": "365101bd07e82db095111b2519466005617efcb186ba08e28dfb0947135f15fa",
    "experiments/g1_g5/evidence/startup-repair-20261021/artifacts/native-build/gtsam_probe": "f9c3460c5154c2d29d89c2236a1b01e2dec669baca107a9683e5bb965ed7953a",
    "experiments/g1_g5/evidence/startup-repair-20261021/artifacts/native-build/bridge_contract": "1f1df749b2a4f049597f40af6169dc422d9c8108c3c92abbe741a7dcd938d70a",
    "experiments/g1_g5/evidence/startup-repair-20261021/artifacts/candidate-observer-world.sdf": "341247a069f5c42eb7257b26bd88504377851d9e772975117cc8f04ce7a6e7fc"
  },
  "gtsam": {
    "path": "/home/zyd/program/b2w-embodied-navigation-course/.deps/lib/libgtsam.so.4.1.1",
    "sha256": "e04567b0a6346ed8c4a05a21b42b337ee2ce429030f8df87e3149b76a6bfb379"
  }
}
```

## Attempt and evidence index

- Actual Gazebo launches this task: **1/3**; prelaunch failures this task: **0**.
- Attempt `23e60b2d2ac14995a0dde9f0da5422b4`; session `zero-converge-44d7085ccef54ab1a88d3d32bad4a25e`.
- Final six-axis commands:699/699 exactly zero; no lease. Contacts534/534 matched sequences1..534; accepted endpoint absent.
- Probe FAIL and probe cleanup FAIL; owned OS cleanup PASS. No false claim of complete stationary or runtime qualification.
- E1: no offline implementation begun because the full ZeroOnly PASS condition was unmet. Existing PLAN.md E1 section preserved; motion remains NO-GO.
- Raw files:56; raw manifest `experiments/g1_g5/evidence/zero-converge-20261023/RAW_SHA256SUMS`, SHA256 `c67d6ffb35a7d223e93d6034639cfc35111172728ee4f338b3e2925276710259`. Raw files are ignored/local only, not remotely backed up.
- Compact result `summary.json`; pinned identities `identities.json`; engineering report `docs/reviews/B2W_ZeroOnly_Convergence_20261023.md`. All compact records committed; final bundle/checksum are durable untracked copies for this handoff.

## Current Git status

```text
## experiment/unattended-g1-g5-20261019...origin/experiment/unattended-g1-g5-20261019
?? experiments/g1_g5/evidence/zero-converge-20261023/APRIL_REVIEW_BUNDLE_B2W_ZeroOnly_Convergence_cff87a8_20261023.md
?? experiments/g1_g5/evidence/zero-converge-20261023/APRIL_REVIEW_BUNDLE_B2W_ZeroOnly_Convergence_cff87a8_20261023.md.sha256
```

Only this final handoff's durable bundle/checksum are untracked; production source and staged index are clean. Normal push and origin SHA verification succeeded. No original/Pipe edits or uncontrolled process termination.
