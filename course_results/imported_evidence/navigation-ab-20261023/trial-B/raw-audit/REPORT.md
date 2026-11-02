# B raw evidence audit

Audit scope: read-only review of this pre-motion B capture. No goals, pulses, or route were issued or presumed. The capture ended on the server-library identity mismatch; it is not a completed trial or A/B result.

## Run disposition

- `session-report.json`: `success=false`, `precheck=[]`, `contact_failure=null`, `error=KeyboardInterrupt()`, cleanup passed with `remaining={}`. It records only stack, contact bridge, observer, precheck-map, and precheck-adapter invocations. There is no navigation launch or goal action.
- `monitor-events.jsonl`: 1,118 `/final` command samples; all six linear/angular axes were exactly zero in every sample. Raw and smoothed navigation commands were absent. Goal pose, plan, NavigateToPose/FollowPath statuses, and behavior-tree log were absent in observer topic health. A zero command stream is not proof the body was at rest: the body moved during startup settling (below).
- `/clock`: one epoch; 34.73 simulated seconds over 56.09 wall seconds before interruption. This was a partial capture, not the configured 420 s trial.
- `root-server-inspection.json` shows the live server mapped `/usr/lib/x86_64-linux-gnu/libignition-gazebo6-ign.so.6.18.0`; the launch identity predicate expected the `-gz.so` variant. `root-emergency-containment.json` records the mismatch reason, all 36 server threads stopped (`T`), `motion_authorized=false`, and `physical_stop_measured=false`. `root-emergency-cleanup.json` and `session-report.json` record no remaining owned processes.

## Contacts and collision inventory

- `monitor-events.jsonl`: 680 full-contact messages, 675 nonempty. The only four unique pairs were each of `FL_foot`, `FR_foot`, `RL_foot`, and `RR_foot` against `course_world::aws_robomaker_warehouse_GroundB_01_001::link::collision`. No self, non-ground environment, or other robot collision pair was observed in this capture.
- The same file contains 679 inventory records, sequence 1–679, each with the identical 26-name collision inventory; 13,884 physics steps sampled. `ever_forbidden_contact` was false for all 679 records; a ground-support stamp was present in 675/679.
- This establishes what the observer/plugin reported during a quiet, pre-motion capture. It is not deliberate collision coverage, and the inventory list alone does not prove that all 26 shapes had valid live physics-engine mappings. The independently observed Gazebo library identity mismatch prevents treating the live backend mapping as verified. Empty/quiet samples cannot prove the absence of unobserved or unregistered contacts.

## Startup body motion; command stream was zero

`observer/<run>/samples.jsonl` reports truth as `[x, y, yaw]` in `sim_world`. The first valid sample is at 0.105 sim seconds: `[0.00042, approximately 0, 0.000014]`. During the first 2 simulated seconds x reaches `-0.19336`; by 3–6 s it has returned to `[-0.06633, -0.05301]`. It is comparatively settled by 8 s: over 8–15 s, x is `[-0.05354, -0.05310]`, y about `-0.01066`, yaw about `-0.05634`; over 15–34.7 s x is `[-0.05456, -0.05354]`, y `[-0.01065, -0.01060]`, yaw about `-0.056346`. These data show startup translation/rotation with zero reported `/final` commands; they do not establish cause or physical rest, and cannot be described as commanded navigation.

## LIO/TF limits

`observer/<run>/summary.json` records 156 `/slam/odom` messages and 152 samples for each map-base and odom-base TF. TF age is p50 0.3175 s, p95 0.37 s, max 0.38 s. In aligned samples map-base and odom-base agree within 0.00025 m in x, 0.000017 m in y, and 0.000027 rad yaw. SLAM reports frame `odom`; ground truth is frame `sim_world`. Their origins are not registered: settled truth x differs from odom/map x by about 0.289 m. No registered GT-to-LIO transform or ATE was computed, so do not claim localization accuracy from this comparison. SLAM samples appear only late in the capture (around 19 sim seconds).

## Logged diagnostics

`observer/<run>/summary.json` has `errors=[]`; the observer recorded two `/rosout` entries, both INFO. `stack.log` contains SDF warnings for `pose` elements with an undefined `frame` attribute in warehouse models and `libEGL` warnings that DRI2 screen creation failed. No native physics/backend error was found in the captured logs; the server-library identity mismatch remains the material backend-validation limit.

## Evidence files

All paths below are relative to this B trial directory (`/tmp/b2w-navigation-ab-20261023/artifacts/trials/B-5d8101be0c2045b48738c3b42dd611be/`):

- `session-report.json`
- `monitor-events.jsonl`
- `root-server-inspection.json`
- `root-emergency-containment.json`
- `root-emergency-cleanup.json`
- `stack.log`
- `observer/20261023T105047.016859Z-B/summary.json`
- `observer/20261023T105047.016859Z-B/topic-health.json`
- `observer/20261023T105047.016859Z-B/samples.jsonl`
- `observer/20261023T105047.016859Z-B/events.jsonl`
