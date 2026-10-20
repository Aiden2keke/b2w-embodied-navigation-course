# Stage4 external NEEDS FIX response — offline candidate, NO-GO retained

Baseline: `90716f4ebd8fff713f671529d0da5dfb5fa42aae`. Exact final HEAD, diff and
fresh verification evidence belong in the APRIL packet. The independent report
is preserved verbatim beside this file; SHA256
`9a219aa7940d8ef0b02e5b20907dba599c09d95e4deb93246d566549ca825a79`.
Sol owns implementation and safety decisions; runtime-verified GPT-6 Luna executes
specified deterministic checks. Internal review never substitutes for ChatGPT.
No Gazebo/ROS stack or nonzero action was launched in this continuation.

| Finding | Offline response | Qualification still required |
| --- | --- | --- |
| P0 final predictive/braking veto | Opt-in final adapter guard after command transforms; immutable prepared saved/live/local/global grids, each frame; whole filled padded body and isotropic whole-stop/latency/yaw/error reach; coherent odometry/TF position and quaternion; expiry, unknown/off-map, physical-envelope and cycle faults latch zero/inhibit | Conservative measured entire-chain stop/error envelope; full-load callback/worker/GIL/I/O latency; current local volume. Current prototype is **not physically qualified** |
| P0 independent abort/fail closed | Separate non-ROS owner+adapter publication-health watchdog issues short leases; exact process identities, replay/conflict/alias rejection; persistent inhibit checked at final output; owner/adapter loss and command/TF faults revoke; independent stop uses inhibit watermark regardless of action terminal. Inhibit I/O failure still attempts owned UUID cancellation | Persistent session supervisor startup/owner heartbeat/bootstrap protocol, watchdog process lifecycle and physical adapter-death/hang/ROS/action/cancel/recorder fault-chain measurements. Existing RL-SAR .5 simulation-second timeout is only a code assumption until measured |
| P0 qualification | Shared unconditional block remains at CLI, direct FollowPath and final guarded adapter. No flag/environment/JSON override. Kernel/wiring test patches are confined to tests | External review plus every physical gate; this packet requests source review, **not motion permission** |
| P1 map consistency | Preflight checks saved, live and global geometry including measured Start; immutable .5s dispatch binding includes exact serialized Path, geometry/frame, graph, registration and loaded guard/deployment identities. Bounded callback ingestion rejects changed geometry; timestamp-only updates preserve geometry proof. Dispatch rechecks registration expiry, all dynamic freshness and adapter prepared-revision/source acknowledgement | Live queued DDS/update-rate/scheduling characterization; physical reserve and live recertification. Local rolling grid checks local reach, never demands full corridor coverage before Start→G1 |
| P1 deployment | Explicit session deployment replaces ROOT/.deps assumption; verify pinned executable and actual mapped core, inode/device, PID/start ticks/PGID/UID/token. Abort uses frozen loaded config even after manifest pathname replacement. Wrapper/checker support explicit copy prefix/underlay | Fresh running-process/graph proof and complete operational manifest; synthetic /proc tests are unit evidence only |
| P1 segment execution | CLI/planner/runner/evaluator select each contiguous original segment, require preceding goal region, preserve future goals and single FollowPath; segment evidence can never be FULL_ROUTE_PASS | No independent physical segment/transition/full run performed. Measured rate/path deadline evidence needs an operational authenticated producer and dispatch path binding |
| P1 G3→G4 detour | Version2 shared contract explicitly proposes5.25m center band (saved detour needs4.937525m; margin.312475m). Selection binds contract hash; planning and observation use same original phase. G1→G2 remains.35m and all goals.25m | External review of revision; current sensor path/uncertainty, heading/control/braking recertification. The extra band is **topological**, not a physical error budget |
| P1 corridor proof | Diagnostic ordered regions separated from mandatory entry/front/tail/exit evidence. Sensor/map/registration-bound portal, full padded-body orientation/lateral support, heading bound, monotonic forward motion and coherent gaps. Missing certificate cannot yield FULL_ROUTE_PASS | Actual sensor-derived portal boundaries/axis/scope/error certificate; G1 viewpoint observation; genuine full traversal. Synthetic portal fixtures do not supply this |
| P1 physical contacts | Shared paired contact stream feeds immediate sticky veto in adapter and transport; stale/missing reports deny guarded output. Cleanup freezes actual stop endpoint, waits bounded contiguous report closure before sealing; no early failed-route endpoint substitution | Positive non-foot physics-engine shape/entity mapping or qualified independent geometry evaluation; preserved self-collision exclusions; positive controls; complete route+hold closure. `physical_coverage_qualified` deliberately stays false |
| P1 registration | Historical landmark residual/p95 artifact explicitly physical_bound_qualified:false. New schema separately accounts registration XY/yaw/radius, map boundary, body and tracking terms; undersized/missing/unscoped/p95-only bounds rejected; guard checks spatial/time scope | Automatic sensor-only held-out/ambiguity-bounded registration and conservative error derivation. Schema/content hashes alone do not prove measurements or authenticate claims |
| P2 validation | Bash wrapper avoids set-e; explicit setup failures; expanded strict nonempty JUnit matrix including new guard/dispatch/portal/contact/bounds/segments; physical rows remain FAIL; limits derived from measured achieved rates with hard stage caps | Offline tests never replace live qualification. All failed/intermediate attempts remain external; no retries on unchanged failure evidence |

## Reproduction and evidence

From the experimental worktree, execute only the offline commands in
`docs/LUNA_VALIDATION_HANDOFF.md`. The runner creates exclusive external evidence,
source-before/after manifests, exact commands/timeouts, JUnit/logs and a machine
PASS/FAIL checklist. `--run-offline` exit0 means offline regressions passed; expected
`all_requirements_pass:false`, `motion_authorized:false`, `full_route_pass:false`.
The source matrix is `course_navigation/config/navigation_validation_matrix.json`.
All44 gates are represented:17 offline and27 unqualified live gates.

Current work evidence is `/tmp/b2w-review-fixes-20261020/`; the APRIL packet includes
final test/build summaries and hashes plus a reproducible source delta. Large
journals/maps/bags remain in prior external sealed campaign directories. There is
no successful experimental artifact eligible for repository archival.

## Broader verification limitations and correction

Luna's full offline pytest run at48fef81 produced857 PASS/38 FAIL/4 SKIP, retained
unchanged. Sol diagnosed/fixed the actual new baseline timer regression by making
Stage4 timing conditional on Stage4 mode. Copy-installed baseline Nav2 profiles
now compare exact bytes rather than absolute pathname; changed profile bytes still
fail. A new explicit copy/tamper regression and affected modules pass51 tests.
Final matrix now includes baseline M2/adaptive tests. The full broad suite is not
claimed green:19 low-speed tests need ignored policy ONNX,13 warehouse-floorplan
tests need ignored world/geometry (one dependent manifest-key assertion),1 editor
socket test is denied by sandbox;4 legacy tests skip absent external evidence.
No original-checkout modifications, dependency links, downloads or test weakening
were used. Final source/head/results are frozen in the new APRIL packet.

## Operational manifest sketch — intentionally no populated certificate

Deployment requires `kind:stage4_session_deployment`, version1, session_id,
absolute guard_manifest + guard_manifest_sha256, exact adapter process identity,
and controller prefix/executable/library/process/owner_token/pinned hashes.
The loaded guard also needs owner identity; distinct absolute owner heartbeat,
adapter heartbeat, lease and inhibit paths; saved_grid_path/hash; bounds_evidence_path/hash;
source-bound collision_inventory_path/hash and contact_first_sequence; explicit
stop_translation_m,stop_yaw_rad,uncertainty_m,max_speed,max_wz,max_command,
max_command_wz,abort_latency_s>=.6,max_sim_rate.
Use owned private local storage; current regular-file IPC is not a real-time proof.

A bounds record needs version/session, finite region and validity interval,
independently justified worst-case sensor-registration/map/body/tracking components,
whole-stop/abort measurements and hashed raw references. It returns motion_qualified:false.
A portal record needs sensor_evidence_path/hash, current registration/map/session
identities, measured origin/unit tangent, entry/exit, lateral bounds and conservative
uncertainty/heading bound. No guessed world geometry or p95-to-bound conversion.
These schemas are engineering contracts, **not a usable authorization manifest**.

## Stop point

Stop before the first nonzero experiment. Independent ChatGPT must review the
new source/evidence and remaining blockers. Even a favorable source review cannot
supply absent physical measurements or the unresolved operational handshake.
Next offline implementation should complete the session supervisor/rate-path
certificate lifecycle, then submit any new safety-critical delta for review.
Start→G1 remains first priority once its own local gates are independently safe;
no full-corridor visibility prerequisite is introduced.
