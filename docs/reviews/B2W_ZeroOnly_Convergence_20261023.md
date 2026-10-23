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
