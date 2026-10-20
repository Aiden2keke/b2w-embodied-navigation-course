# Luna validation handoff — external NEEDS FIX continuation, 2026-10-20

This is the validation contract for the current PLAN NOW. The JSON matrix is
`course_navigation/config/navigation_validation_matrix.json`; source YAML/C++/Python
remain SSoT. This handoff adds no motion authority. External ChatGPT verdict is NEEDS FIX / NO-GO. No nonzero experiment may run before new external review and unresolved physical gates. Luna cannot approve motion or edit safety code.

## Execute exactly

Run from `/tmp/b2w-unattended-worktree-20261019`. These commands launch no ROS nodes,
simulator, command publishers or actions. They source existing read-only underlays.
The wrapper must be executed with Bash, even from a zsh terminal.

```bash
# Dry-run: print exact command arrays and limits; no output directory created.
bash scripts/run_navigation_validation.sh --output /tmp/b2w-luna-check-UNIQUE
# Execute once using a NEW, nonexistent output directory.
bash scripts/run_navigation_validation.sh --run-offline --output /tmp/b2w-luna-check-UNIQUE
```

Use an actual unique timestamp in place of UNIQUE; output reuse is rejected. No
automatic retry. If source changes while running, the entire report fails. Underlay
paths can be set explicitly with B2W_COURSE_UNDERLAY and B2W_EXPERIMENT_UNDERLAY; save
resolved values. Missing underlay is a blocker, never a reason to install/upgrade.

Outputs: `checklist.json`, copied `matrix.json`, source-before/after SHA256 manifests,
command/exit/timing metadata, complete `focused-0.log`, `harness-0.log`, JUnit XML and
`native-{0,1,2}.log`, owned native build directory. Offline expected result:
exit 0, `offline_pass:true`, strict nonempty JUnit with no skips/failures/errors,
native CTest passes; `motion_authorized:false`, `full_route_pass:false`, and live
requirements remain FAIL until independently qualified. Exit 0 alone never qualifies
motion. Dry-run always exits 0 and is not a test result.

Limits: focused 180 wall s; runner tests 30; native configure/build/test 60/120/30;
whole invocation budget 480 wall s. Timeout kills only the freshly created test
process group and retains its log. Luna stops after this invocation and reports
exact output path, HEAD, source drift, counts, failures and elapsed time. Parent
examines the evidence and decides next work. No test weakening, changing skips,
command substitutions, extra launches, retries, motion approval or implementation.

## Physical-stage commands and stop rules

There is currently **no qualified physical-stage command**. Existing
`run_candidate_route.py run --execute-user-run` is unqualified; do not invoke it.
The runtime veto/lease/abort/segment/portal code is an offline candidate. Automatic conservative registration, measured fault-chain stop/latency, operational supervisor bootstrap and all-shape physical coverage remain dependencies; unit tests cannot waive them. The CLI, direct send and final guarded adapter share an unconditional interlock. A missing stage must be reported FAIL with its source and expected
result below; do not invent commands or fill a readiness boolean.

For any future physical stage, the parent must first freeze a manifest with exact
invocation, session/epoch/GIDs/UUID, PID/start-ticks/PGID/token, source/binary/config,
registered map/grid revisions and uncertainty, exact post-smoothing Path and Start
connection, envelope/latency/stop bounds, observer coverage, disk/rate allowance,
sim+wall deadline, motion-slot counter and abort path. Luna may execute only that
reviewed manifest and must stop on its first veto. A changed observation or diagnosed
fix plus affected regression rerun is required before another attempt. Never
terminate foreign processes or equate process exit with physical stopping.

Start→G1 requires only its own local approach/turn/braking/future-region gates. Do
not demand full-corridor visibility for that safe experiment. E1 calibration may
precede it where needed. At G1, stop and obtain new sensor-supported coverage before
G1→G2. Preserve all five original identities, ordered .25 m regions, the mandatory
corridor, full padding, unknown blocking and unchanged RL-SAR policy. The proposed version2 shared G3→G4 topological band is5.25m (saved alternative needs4.937525m); selection/planning/evidence use the same file. G1→G2 stays.35m. This revision needs external review and does not provide physical clearance.

Budgets inherited: 211.308247903 live-stack wall s, 0/10 motion attempts, 5 stationary
starts; total 21600 s, 3 families, 2 critical trials per family. E1 ≤90 sim/180 wall
s and one slot; critical segments ≤180/300, full runs ≤600/900. Every stage freezes
a tighter measured duration if possible. Three consecutive fresh full accepts at
seeds0/1/2 with identical hashes are required; a change resets this count. Individual
segments and transition passes are diagnostic and never FULL_ROUTE_PASS.

Abort: forbidden/unknown/self contacts; uncertain/stale registration, localization,
perception or grids; pose/clock/frame/map discontinuity; stopping/envelope violation;
future goal entry; foreign commands/goals/actions; recorder/disk/queue/flush failure;
controller deadline or .15 m/20 s progress violation; ABORTED/CANCELED, unresolved
handle/cancel; budget/deadline. Independent inhibit/zero must precede owned cancel,
then measured stop and complete end-watermarked collision interval, seal, replay and
owned process cleanup. Do not move until that mechanism has been qualified.

## Complete requirement matrix

Offline PASS applies only to the named regression group. Every live row requires
current-session evidence; the historical source notes are diagnostic. Dependencies
are IDs, not authorization switches. Exact offline command arrays/timeouts are in
the dry-run output. For missing physical commands, inspect the current checklist and
source at the row below; the parent implements and reviews before Luna execution.

| ID | Level / dependency | Requirement and expected result | Source / missing evidence |
| --- | --- | --- | --- |
| O01 | offline; none | Source identities, immutable selected V1→V2 goals/map/profile and .25 m radius. Exact hash-bound selection; tampering and unverified migration rejected | test_stage4_route.py; contract.py; stage4_selection_policy.json |
| O02 | offline; none | Coherent source-time localization and separate advancing receipt freshness. Finite normalized quaternions; map→odom→base at odom time; source/receipt ≤.5 s; duplicates cannot renew | test_coherent_localization.py; test_stage4_transport.py |
| O03 | offline; none | Continuous filled padded-body geometry, initial/terminal turns, unknown and off-map. All closed nonfree cells and between-pose rotation/translation checked; bounded unresolved search fails | test_continuous_geometry.py; test_route_geometry.py |
| O04 | offline; none | Full serialized recording, capacity, flush and replay failure behavior. No data loss/truncation/overflow can pass; measured capacity >=5 wall s and 2×max(mean,peak)×(trial+60 s) | test_binary_journal.py; test_stage4_transport.py |
| O05 | offline; none | Every-step contact reporting paths and sticky transient veto. Native reporting tests pass all inventory paths; does not prove engine construction | scripts/contact_observer; tests/test_contact_evidence.py |
| O06 | offline; none | Ordered .25 m passage and terminal stop evidence regressions. ≥.1 sim s region confirmation; no future region entry; ≥5 source s AND ≥3 wall s rest; creep fails | test_stage4_route.py; evidence.py |
| O07 | offline; none | Validation cannot convert skips, missing data, source drift or timeout into qualification. Exclusive outputs; strict JUnit; timeout log retained; motion_authorized=false | tests/test_navigation_validation_runner.py |
| O08 | offline; O01 | Unqualified motion dispatch interlock at CLI and direct transport. No flag/environment/JSON override; reject before ROS construction and FollowPath | tests/test_unqualified_motion.py; motion_qualification.py |
| O09 | offline; O05 | Paired contact intervals and final watermark regression. Bounded integer buffer; contiguous source intervals; immediate sticky veto; duplicates cannot renew wall freshness; last-report-before-end fails | tests/test_contact_intervals.py; contact_intervals.py |
| O10 | offline; O06 | Independent segment evaluation and original future-goal exclusions. Previous-goal region and first measured Start required; original goal names/order and narrow phase retained; even a passed segment can never be FULL_ROUTE_PASS | tests/test_segment_evidence.py; evidence.py |
| O11 | offline; O03 | Measured initial translation/yaw connection in preflight geometry. Certificate includes actual measured Start and yaw through first checked Path pose; initial connection obstacle/rotation collision fails | tests/test_start_connection.py; spatial.py; ros_transport.py:preflight |
| O12 | offline; O03 | Prepared immutable grid reach kernel and expiry regressions. Filled body plus caller-supplied whole-stop reach/uncertainty; explicit occupancy unknown normalization and frame/epoch/time/deadline rejection; NOT runtime veto/stop qualification | tests/test_predictive_reach.py; predictive_reach.py |
| R01 | live; O01, O07 | Current session environment, source/binary/model/profile identities. Environment ready:true; fixed unchanged policy; loaded hashes and exact effective parameters recorded | setup_course_humble.sh --json; prepare_rl_sar.py; actual resolved copy install |
| R02 | live; R01 | Single owned Gazebo/ROS domain and no competing goals/actions/commands. Unique clock/SLAM/adapter; exact expected producers; owned UUID; foreign producer abort | ros_transport.py:graph; launch lease and owned PID/starttime/PGID |
| R03 | live; R01, O02 | Live sensors/clock/body stability and coherent localization. Fast source/receipt ≤.5 s, monotonic clock, no >.3 source/.5 wall sample gaps, no >.2 m jumps | check_initialization.py; coherent_localization; session journal |
| R04 | live; R03 | Structural SLAM cadence and >=30 sim s passive observation. Source age ≤7.5 s; measured wall allowance in [7.5,20] s; >=3 advancing maps; retained raw points/TF | StructuralMapCadence; stationary.py |
| L01 | live; R04, O01 | Automatic sensor-only saved/live registration, independent holdout and ambiguity rejection. Noncollinear fit/holdout geometry; conservative local uncertainty fits reserve; no truth/world substitution; two fresh passive starts; ≤30 wall s / ≤2 new-observation attempts | contract.py currently operator pairs only; sensor-fit-complete.json historical diagnostic |
| L02 | live; L01 | Same-session frame/map/epoch/graph binding, expiry and moving drift monitor. Exact hashes/GIDs and observation times; age ≤120 s; rollback/map/frame/uncertainty drift abort | check_registration; missing automatic local bound monitor |
| P01 | live; R02, R04, O03 | Zero-action Nav2 readiness: effective Smac2D+DWB, costmaps and footprint. No FollowPath; recorded active lifecycle/plugins, controller 20 Hz, padded 1.50×.86 m, unknown blocked, local odom/global map frames coherent | verify_runtime; nav2_smac2d_diag.yaml; current live costmaps |
| P02 | live; P01, L02 | Exact segment path certificate including actual Start/heading and stop pockets. Sensor-supported filled body plus registration/tracking/latency/braking reserve; no future disk entry; post-smoothing actual FollowPath bytes checked; ≤30 wall s | preflight; assess_path; continuous.certify_path |
| C01 | live; O03, P01 | Predictive filled-body/uncertainty/braking veto in effective adapter path. Cached immutable saved/live/local/global grids; own frames/source times; actual motion+command latency+conservative stop; unknown/offmap/frame/stale/deadline/no progress fail closed | Opt-in final adapter reach veto implemented; measured physical bounds and timing remain missing |
| C02 | live; C01 | Replay deadline qualification and independent owned-stack abort. Cycle p99 <50 ms at20 Hz; freeze worst abort latency; unresolved/late handle, rejected cancel, dead controller, localization and recorder faults produce zero and measured physical stop | Lease/inhibit/watchdog and adapter heartbeat candidate; operational startup/runner heartbeat and measured physical fault chain missing |
| K01 | live; O05, R03 | All-body physical contact coverage or independent geometry evaluator. Positive all non-foot engine construction/mapping or independently bounded collision evaluator; source inventory hashes; preserve self-collision settings; unknown coverage INCONCLUSIVE | 26 ECS entities alone insufficient; non-foot engine mapping unproved |
| K02 | live; K01 | Contact sequence/interval closure through accepted stop timestamp. Coherent inventory/contact sequence; every physics step; sticky forbidden veto; report interval end >=accepted stop/end; lost/duplicate/late-tail reports fail | Shared contact stream now wired; cleanup waits for paired reporting through accepted stop before seal; physical all-body coverage remains false |
| B01 | live; O04, R02 | Current-session recording/disk/rate budget and seal. >=5 wall s measured throughput; >=2×max(mean,.5s peak,current bucket)×(trial+60s), hard queue/disk failures veto; successful seal and full replay | BinaryJournal; verify_recording_capacity |
| E1 | live; L02, P01, C02, K02, B01 | Open-pocket locomotion/stop calibration before corridor. ≤90 sim/180 wall s; 1 motion slot; start .05 forward; .10/.15 only within measured margin; joints/roll/pitch/yaw/slip/SLAM+evaluation speed/lag/postzero excursion recorded; no corridor entry | Local saved/live geometry historical only; bootstrap stop allowance must be frozen anew |
| S01 | live; E1, P02, K02, B01 | Independent Start→G1 qualification and execution. Current local corridor-independent path/uncertainty/stop gates; exact G1 identity and .25 m passage; G2–G5 forbidden; ≤180 sim/300 wall s ceiling, tighter frozen limit before run | Original segment selection now wired to CLI/planner/runner/evidence; no qualified live trial |
| V01 | live; S01 | G1 sensor viewpoint and SLAM/costmap update before G1→G2. Certify observation approach/turn/stop from sensor free volume; observe both critical boundaries; no source-world guides or entering uncertified passage for visibility | Retain raw lidar, SLAM map, exact transforms and revision hashes at G1 |
| M01 | live; V01, O01 | Derived map provenance and explicit execution adapter. Original goals/PGM/source preserved; parent/raw message hashes/TF/algorithm/changed cells and evidence; unknown blocked; no erosion/free stripe; reload native map counts | map_derivation.py metadata-only parent exists; sensor correction/execution adapter absent |
| P03 | live; M01, L02, E1, C02 | Connected heading-aware mandatory corridor path and quantitative reserve. Fit tangent from sensor boundaries; aligned entry, bounded heading through full bottleneck, tail exits before turn; registration+map+body+tracking+latency+braking positive reserve | Start-only raw 300-scan replay rear boundary unobserved; source-world clearance evaluation only |
| S12 | live; S01, P03, K02, B01 | Independent G1→G2 critical segment. Mandatory portal full-body traversal/tail exit; exact ordered .25 m goals; no collision; terminal stop/cleanup; ≤180 sim/300 wall s; max2 critical trials/family | No qualified live trial |
| S23 | live; S12, P02 | Independent G2→G3 segment. Current Start connection, future-region exclusion, filled-body reserve, .25 m G3 passage, verified stop and cleanup; ≤180 sim/300 wall s ceiling | No qualified live trial |
| D34 | live; O03, M01 | Reviewed G3→G4 alternative and consistent contract revision. Exact endpoint headings/rotations; selection/planner/evaluator bound to same explicit revised ordinary corridor polygon/band; preserve G1→G2 and all goals; recertify smoothing | continuous-detour.json: .020 m extra/.029010 m residual; shared proposed5.25m band; actual sensor/error/physical recertification and external review missing |
| S34 | live; S23, D34, P02 | Independent G3→G4 critical geometry/control segment. Current map/frame and actual start; approved alternative if justified; .25 m G4 passage; future G5 excluded; verified stop/cleanup; ≤180 sim/300 wall s ceiling | No qualified live trial |
| S45 | live; S34, P02 | Independent G4→G5 segment and final yaw. Exact G5 identity/yaw, .25 m region, yaw error ≤.3 rad, full final hold and collision closure; ≤180 sim/300 wall s ceiling | No qualified live trial |
| T01 | live; S01, S12, S23, S34, S45 | Continuous transitions without stitched starts. One uninterrupted session preserves regions/portal/tail order, command ownership, turning and braking certificates at every join; any map/config change recertifies | Missing segment transition runner/physical evidence |
| F01 | live; T01, B01, K02 | One complete fresh Start→G5 route. ≤600 sim/900 wall s; all five ordered .25 m regions, mandatory passage, collision-safe, G5 final yaw, ≥5 source/3 wall s hold; all hold poses within region; actual speed≤.03 m/s yaw≤.03 rad/s command≤.01 excursion≤.03 m; contact closes hold | No full-route run |
| F03 | live; F01 | Three consecutive fresh complete accepts seeds0/1/2. Same frozen map/config/route/library hashes; any change resets count; total≤10 motion slots/3 families/2 critical trials per family; ≤21600 total live-stack wall s | 0/3 accepts; no seed-qualified route |
| A01 | live; F03 | Independent raw replay, metrics, video, review bundle and successful artifact archive. Recompute all accepts from complete raw journal; matching route/collision/stop outcome, all hashes, commands, failures, budgets, processes, sim-time video; archive only independently verified success after final success | No successful artifacts eligible for repository |


## Current offline delta and exact scope

New focused modules are included automatically in the runner: `stage4_safety_guard`,
`stage4_dispatch_deployment`, `stage4_segment_execution`, `corridor_portal`,
`stage4_contact_wiring`, `stage4_bounds`, `independent_stop`, `navigation_watchdog`.
The machine matrix adds O13–O17; all live rows remain FAIL. Source stability covers
all relevant tracked/untracked source before and after execution. Test positives use
synthetic inputs; the adapter wiring tests explicitly patch the unconditional block
only inside the process to exercise the candidate path. A separate regression calls
the real block and observes final zero. Tests cannot enable a production bypass.

Sol provides actual GPT-6 Luna commands with fresh evidence directories and verifies
session metadata `model=gpt-6-luna`, not the agent label. Luna reports one invocation,
complete logs, JUnit, exit/timing, source hashes and failures; Sol evaluates and fixes.
A changed source or new evidence is required before an affected-test rerun.

`plan --mode segment --segment {start-g1,g1-g2,g2-g3,g3-g4,g4-g5}` selects the original
single-segment planner/evidence scope with an explicit `--deployment`. Planning is
read-only but still requires current-session readiness. `run` supports the same
selection and rejects unconditionally. `--deadline-evidence` is engineering input,
not a qualified rate certificate. There is no new executable motion instruction.

Read [review dispositions](reviews/B2W_Stage4_Review_Disposition_20261020.md) for
manifest fields, missing operational handshake and remaining physical blockers.
