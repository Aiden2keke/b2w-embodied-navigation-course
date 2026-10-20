# B2-W Stage 4 External Independent Code Review — 2026-10-20

**Verdict: NEEDS FIX (source/integration); NO-GO for any nonzero Gazebo navigation motion.**

Scope: exact Git baseline `8f2f5ea673f21a129b0f199a8ee92c71fc988ce6` to `90716f4ebd8fff713f671529d0da5dfb5fa42aae`, nine commits, 100 changed files. Uploaded APRIL Markdown SHA256 `2582f8e1a5c2c682cc9a7db849e7061a7fcd9b622674f53cd8d8a35dd0496a65`; the seven embedded Git patch pieces concatenate to 660,898 bytes and match SHA256 `349f124e927847c4de11cfdf81f8ccd3580ff455f9c0ccfe12853e5165d4ed62`.

This is a **source review of the uploaded packet**, not a live ROS/Gazebo integration test, runtime reproduction on the user's machine, or physical-collision certification. The packet reports 130 selected Python tests and one native CTest; 12 offline checklist PASS and 27 live checklist FAIL. No navigation motion attempts. Earlier working-tree `PLAN.md` gate addition is not included in committed HEAD and was separately disclosed.

## Decision-driving findings

### P0 — Missing motion-safety system is genuinely unimplemented (confirmed incomplete, not a newly found regression)

- `scripts/stage4_route/motion_qualification.py:9-13` unconditionally raises `UNQUALIFIED_FOR_MOTION`. `scripts/run_candidate_route.py:25-27` and `scripts/stage4_route/ros_transport.py:428-435` correctly prevent current Stage 4 dispatch. Keep this block until a real reviewed safety mechanism exists.
- `course_navigation/course_navigation/velocity_adapter.py:163-179` still only applies current localization validity, stale-command control, speed-limiting / optional M2. It has no integrated prepared-grid predictive reach/stop veto, latched inhibit, independent supervisor or physical stop confirmation. `scripts/stage4_route/predictive_reach.py` explicitly advertises **offline-only** and uses caller-supplied, unqualified travel/yaw/uncertainty estimates.
- `scripts/stage4_route/ros_transport.py:482-561` cancels a specific ROS Action and observes stationary state; cancellation is not independent velocity inhibition or verified braking if the action/server is delayed or fails. It must not be treated as the sole safety abort.
- Source acceptance cannot be upgraded to motion GO merely by changing the unconditional exception or passing isolated tests.

**Fix/test:** build an action-independent fail-closed gate in the *only effective* adapter/low-level command path, with independently owned inhibit/abort, measured worst-case stopping/latency envelope, health/heartbeat watchdog and safe cleanup. Add offline fault injection for loss of TF/costmap, stalled action server, adapter/owner death, clock regression, contact alarm and stale command. Verify zero output and genuinely observed stopping in a bounded, **separately reviewed** Gazebo non-navigation qualification stage, before Start→G1.

### P1 — Preflight-to-FollowPath costmap TOCTOU gap (confirmed source-level)

- `scripts/stage4_route/ros_transport.py:374-414` certifies `self.checked_path` against a saved and then-current global costmap, storing a hash in `self.audit['preflight']`.
- `scripts/stage4_route/ros_transport.py:428-448` thereafter calls `verify_recording_capacity`, `sample` and `graph`, but does **not** revalidate the **full planned path** or ensure the checked global-costmap revision remains current before `send_goal_async`. `sample` checks the **current robot footprint** only (`ros_transport.py:255-264`), not the path's future sweep.
- A costmap change after preflight but before action acceptance can invalidate future segments without invalidating the immediate pose. A periodic live veto is still needed during motion; recertification only at the start is insufficient.

**Fix/test:** bind the exact checked Path, costmap revisions, frames, start-pose timestamp and registration to a short-lived immutable motion certificate; atomically recheck validity immediately before dispatch. Add a regression that changes the map between preflight and send and requires refusal; independently require a continuously running bounded predictive veto after dispatch. Do not repeatedly recalculate an expensive global certificate in the real-time command callback.

### P1 — Installed controller overlay lookup is inconsistent with cleaned experimental worktree (confirmed integration blocker from source + packet policy)

- `scripts/stage4_route/ros_transport.py:312-324` hardcodes `ROOT/'.deps/nav2-controller-1.1.20/install/nav2_controller'`, checking process binary and memory-mapped library there.
- The current `PLAN.md` and artifact policy record that the experimental worktree's `.deps` and `.venv` symlinks were removed. A correctly launched ROS controller from a separately prepared, read-only underlay cannot match this absent hardcoded prefix; readiness can fail even if it is correctly configured.

**Fix/test:** resolve the controller executable and loaded library from the *actual permitted session deployment manifest* and validate their exact pinned digests/process identity. Never restore forbidden dependency symlinks merely to satisfy a hardcoded path. Integration-test an isolated checkout with external underlay and verify effective loaded library (not only executable path).

### P1 — Segment evaluation is not segment execution, and the G3→G4 detour still fails the hardcoded acceptance band (confirmed missing integration)

- `scripts/stage4_route/evidence.py:12-24` can evaluate `count=1,start_index>0`, but `scripts/stage4_route/runner.py:6-13` does not accept/pass `start_index`; the CLI `scripts/run_candidate_route.py:18-22` offers only `critical` or `full`. No qualified end-to-end Start→G1 or individual G2→G3/G3→G4 executor yet exists.
- Both `scripts/stage4_route/spatial.py:80-82` and `scripts/stage4_route/evidence.py:57-60` enforce a 3.0 m band on non-narrow segments. The reviewed G3→G4 candidate detour requires a recorded deviation approximately 4.94 m and is therefore deterministically rejected even if its geometry is feasible.

**Fix/test:** implement an explicit, separately evaluated segment CLI/contract preserving immutable goal identities and measured predecessor-region starts, and a versioned ordinary-leg restriction revision shared by preflight and live evidence; never weaken mandatory G1→G2 portal/tolerance constraints. Test Start→G1, G1→G2, G2→G3, G3→G4, G4→G5 and post-segment continuous transitions. Per-segment pass is never FULL_ROUTE_PASS.

### P1 — Region-order evidence is not by itself proof of traversing the mandatory physical portal (confirmed acceptance gap)

- `scripts/stage4_route/evidence.py:53-73,94-107` treats ordered G1/G2 region observations plus `narrow_samples>=2` inside an abstract 0.35 m centerline band as `narrow_passage_confirmed`.
- This records region progression but has no independently registered entrance/exit cross-section, body-tail passage or actual world collision clearance. The flag is semantically stronger than the implemented evidence. Current `runner.py:38` always passes `safety_complete=False`, preventing formal FULL_ROUTE_PASS **today**; future implementation must avoid silently promoting the weak portal flag.

**Fix/test:** distinguish `ordered_G1_G2_regions_seen` from `registered_corridor_portal_and_tail_passed`. Construct sensor-derived, hash-bound entry/exit boundaries and check the padded body crosses fully, in order, with continuous fresh localization and independent physical veto coverage. Keep those separate from the raw Nav2 action terminal status.

### P1 — Live physical / registration evidence is not closed (known blocker corroborated by source)

- `scripts/contact_observer/coverage.hh:18-45` requests contact reporting for the expected 26 robot collisions, but `scripts/stage4_route/contact_evidence.py`/`contact_intervals.py` cannot by themselves prove non-foot engine coverage, self-collision configuration, or that every relevant physics contact was surfaced. Prior capture has a 47.5 ms missing final reporting tail; no post-terminal complete-contact evidence exists.
- `scripts/stage4_route/contract.py:92-111` permits landmark residuals up to 0.10 m and an occupied-point p95 up to 0.10 m; these are **fit residuals**, not conservative bounds on map/TF/localization/body tracking error. The nominal source-mesh passage margin (~45 mm) cannot be safely compared to these without a real error budget.

**Fix/test:** engine-bound collision name mapping plus simulated fault-contact injection and complete terminal intervals; enforce registered sensor-side local free-space uncertainty with explicit translation/yaw/footprint/latency components, first for an open Start→G1 pocket, then for the corridor after new G1 observations. World truth may only be an independent evaluation/veto input, never localization or goal generation.

### P2 — Harness/process and acceptance polish

- `scripts/run_navigation_validation.sh:3` uses `set -eo pipefail`, contrary to the user's explicit workflow restriction to avoid `set -e`. Handle failures with explicit exit-code propagation without changing the meaning of the Python validation script.
- `scripts/stage4_route/runner.py:6` fixes `wall_limit=120` and `sim_limit=90` for both critical and full modes. Once the proposed ~13.2 m G3→G4 detour and narrow-speed sections are used, these limits may reject a perfectly valid slow continuous route. Derive bounded per-segment/final deadlines from certified path length and measured achievable conservative speed, while preserving global campaign limits and abort watchdogs. Never solve timeouts by declaring a partial route successful.
- The offline runner's 130 targeted cases are not a full repository regression or an actual loaded-Nav2 integration test. Require affected regression suites and a bounded read-only lifecycle/graph/runtime-identity smoke before deciding any movement readiness.

## Positive findings

- The unconditional motion interlock fails closed in both the CLI and direct `RosTransport.send` under reviewed source.
- Immutable map/goal provenance and the separation of saved-map encoding correction from executable selection reduce false readiness claims.
- The new continuous closed-cell filled-body certificate is more appropriate than center-only path samples, and explicitly does not certify physical tracking or a connected path merely from a clear pose.
- The binary CDR evidence and source-coherent TF/odometry snapshots improve diagnostic provenance. An observed historical contact-tail gap is correctly retained as failure rather than asserted to be full coverage.
- The isolated worktree, checkpoint and external evidence design appropriately avoid merging inherited dirty source or calling offline unit-test success a physical route pass.

## Recommended staged decision

1. **Immediately:** maintain NO-GO for nonzero navigation. Sol fixes the confirmed offline source/integration issues above, adds tests and a machine-readable P0/P1 gate inventory. Luna runs deterministic offline + read-only lifecycle checks. Do not silently edit safety thresholds or turn off the interlock.
2. **Next review:** submit compact APRIL bundle of all source changes since `90716f4`, especially effective command inhibit, owned independent abort, path revision binding, contact coverage, registration bounds and qualified local runner. ChatGPT independent review remains a pre-motion gate.
3. **After engineering and external qualification:** separately bounded open-space local E1 (or Start→G1 if its approach and stop are independently certified), observe and validate corridor from G1, then G1→G2 and G3→G4, remaining segments, integrated Start→G5 and three fresh complete successes. Never stitch separately successful segments into a formal route result.

**External review conclusion:** `NEEDS_FIX`; `NONZERO_NAVIGATION_GO=false`; offline implementation and deterministic Luna validation may continue. This review is not approval to drive, merge or push.
