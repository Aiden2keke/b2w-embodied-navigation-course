# APRIL Review Bundle — reviewed ZeroOnly policy and first E1 plan

One trial FAILED. No retry/fix/nonzero/commit/push. Temporary review exchange.

# Reviewed ZeroOnly deadline policy + ONE Warehouse trial — 2026-10-22

**Full live probe FAILED. NONZERO NO-GO. No fix/retry after failure.**
Repository `/home/zyd/program/b2w-embodied-navigation-course-stage4`, branch
`experiment/unattended-g1-g5-20261019`, unchanged HEAD
`4ae85f4148a89a76b1ad25c5b059c2140be41b85`. Working changes remain uncommitted.
User communicated external architecture **CONDITIONAL PASS** for the immutable
ZeroOnly-only policy and expressly authorized one bounded trial after preflight.
This records that communicated decision, not a fabricated external approval.

## Exact implementation and offline evidence

Only runtime source changed: `course_navigation/course_navigation/velocity_adapter.py`.
Prepublish>=50ms still captures first overrun wall/thread CPU timing and forces an
exact-zero Twist; sticky `adapter_publication_deadline` is conditional on
`guard.zero_only is not True` (:339–342). Post-health>=50ms retains timing;
`adapter_health_publication_deadline` plus its immediate zero publish remains
identical for motion mode, conditional on that same fixed flag (:363–364).
No threshold, timer, owner, lease, freshness, contacts, guard geometry,
qualification function, controller or harness changed. No runtime parameter or
`armed=False` exemption added. Existing ctor-captured ZeroOnly mode never arms or
leases; its existing output path returns zero independently of input command.
Unexpected nonzero inputs and leases still permanently inhibit.

Tests changed only in `tests/test_stage4_startup_repair.py` and
`tests/test_stage4_health_io.py`. Tests distinguish fixed ZeroOnly from unarmed
motion mode, delay/simulated CPU load, retained first timing after recovery,
health-write delay versus actual write/rename errors, and owner/lease/nonzero
faults after an overrun. Existing zero-boundary, watchdog, malformed localization,
stale-health and hard nonzero interlock regressions also ran.

TDD RED: **7 failed/21 passed** against old source, all expected ZeroOnly policy
failures. GREEN: **71 passed in1.56s**, no ROS node/context or Gazebo in this set.
Exact selected suite (sourced health-io candidate env, system Python):

```bash
/usr/bin/python3 -m pytest -q -p no:cacheprovider \
  --basetemp=/tmp/b2w-zero-policy-green \
  tests/test_stage4_startup_repair.py tests/test_stage4_health_io.py \
  tests/test_stage4_zero_boundary.py tests/test_unqualified_motion.py \
  tests/test_navigation_watchdog.py
```

Independent static code review found no blocking defect before the live trial.
Health-stage first-overrun timing becomes visible on the next successful health
callback, because the current JSON was already serialized when that delay is
measured. Immediate shutdown may lose that final diagnostic; no same-callback
persistence claim. Review also noted waiting-TF/localization+delay combinations
are not newly tested; their unchanged zero/fault paths have existing coverage.

Regular isolated `course_navigation` copy build PASS in0.985s, under
`experiments/g1_g5/evidence/zero-policy-20261022/artifacts/install/`.
Preflight checked35 regular copied Python/launch/config files (including existing
bringup underlay), fresh import resolution to the new install, pinned native LIO,
GTSAM, contact observer/world, policy and reviewed controller. No fallback links,
downloads or dependency changes. Domain197 idle; display/GPU preflight PASS.
Actual runtime adapter argv names this new install; actual LIO executable SHA
and GTSAM mappings match preflight. Copy identities are in `raw/preflight-host.json`.

## Exact single execution and useful progress

From Bash, repository root, using existing limits:

```bash
source experiments/g1_g5/evidence/zero-policy-20261022/candidate-env.bash
/usr/bin/python3 experiments/g1_g5/evidence/zero-policy-20261022/raw/run_once.py
```

That retained one-shot wrapper starts the existing passive recorder, then exactly:

```bash
/usr/bin/python3 scripts/stage4_session.py \
  --inputs experiments/g1_g5/evidence/startup-repair-20261021/zero-inputs-candidate.json \
  --output experiments/g1_g5/evidence/zero-policy-20261022/raw/session \
  --execute-zero-smoke
```

Authoritative absolute argv, owner identities and token are in `raw/session/invocation.json`;
exclusive `raw/attempt.json` proves one attempt. No retry. Zero-only guard true,
no lease/goal/nonzero publication by the task, hard production motion interlock
still raises. Original Warehouse observer world retained; no GT localization.

| Evidence | Observed result |
| --- | --- |
| Old50ms barrier | Recovered after prepublish **82.903ms wall /1.337ms thread CPU**; ready health retained and no inhibit for that reason. |
| Bootstrap | Three prepared live/local/global grids; structural budgets7.5s/7.511243s. Ready cadence observed monotonic347220.833078; watchdog and probe created. This is new progress beyond all preceding trials. |
| Full probe | Started, **FAILED**; `first_sample:null`, `accepted_end_sim:null`, no stationary accepted interval or endpoint closure. |
| Final Twist | **716/716 received samples all six axes exact zero**; raw command1 received zero; smoothed command not observed. Best-effort recorder is not lossless proof. |
| LIO-SAM | **162 positive imu_step updates**, deltaTij approximately0.1s. No observed update_failed/b0 exception; historical b0 root cause still unresolved. |
| SLAM/map |115 odometry messages,11.4 source seconds;3 structural point maps;12 occupancy maps;19 local and6 global raw costmaps. Sustained runtime parameter/graph verification was not completed by probe. |
| Contacts |592 contacts+592 inventory messages; paired delivered sequences1–592 contiguous; four foot/ground pairs; ever_forbidden=false. Full-body engine coverage remains unqualified. |
| Stationary estimate | Max SLAM distance from first0.000704m over11.4s. This is limited localization evidence, not whole-trial physical stationarity or full probe PASS. |
| Time/cleanup | Live stack47.665s, wrapper48.381s; existing220s supervisor/90s bootstrap/150s watchdog/90+35s probe/10+5+3 cleanup bounds unchanged. Owned cleanup PASS. |

## First remaining real blocker — preserved watchdog health boundary

First inhibit file reason is **`adapter_publication_stalled`**, observed monotonic
**347222.359472**, latest received sim30.2575. This comes from the unchanged
`course_navigation/course_navigation/stage4_control.py:97–100` health predicate,
after watchdog/probe started. Watchdog exited; supervisor saw
`Owned watchdog/stack/evaluation bridge exited` and stopped the owned session.
This is **not** another50ms sticky callback fault and **not** a probe pass.

The predicate jointly checks session/adapter identity, positive sequence, finite
clock values and **0<=sampled_wall-health_callback_start<=.15**. Watchdog did not
save the exact record and wall value it rejected, so do not claim the exact failed
subcondition or prove stale age>150ms from the error string alone. The nearby
final command receiver gap is **159.635ms** (347222.130601→347222.290236), consistent
with a real scheduling/delivery interruption; it cannot identify callback versus
DDS/receiver scheduling. Latest observer health before fault was sequence713,
callback start347222.129119, observed347222.153931. Next sampled health716 is
already faulted. Intermediate714/715 and watchdog's read are not captured.
Also consider the clock-before-read ordering when reviewing possible negative
ages; no cause is established. No threshold increase, watchdog removal, new
framework, source repair or live repeat was made after this failure.

Probe was interrupted during effective-parameter checks (only planner parameters
recorded); it later reports invalid rcl context and failed stationary cleanup.
Its journal seals100/100 entries, but `cleanup_passed:false`. These are subordinate
to the earlier watchdog inhibit; owned OS-process cleanup PASS does not turn
probe cleanup into PASS. Adapter and map_projector also log invalid ROS context
on shutdown. LIO/controller were not the first fatal marker.

**Next review decision:** resolve this specific health predicate/read-order versus
actual publication interruption using preserved samples and a small offline
reproduction if authorized; retain150/250ms limits. Do not blindly repeat Gazebo
or infer that removing50ms failure alone qualified the control stack. E1 remains
blocked additionally on independent stop/abort and accepted local bounds below.

## Minimal E1 and fastest Start→G1 path

Concrete code-entry-point plan is in the **single PLAN.md NOW**, subsection
“Minimal E1 implementation plan”; no second active plan or motion implementation.

1. Review a conservative bootstrap envelope for one open Warehouse pocket. The
   present `stage4_bounds.validate_bounds` requires measured stops: historical
   forward-stop .079872m is provisional, not an accepted first-pulse bound.
2. Extend existing `stage4_session.py` / `stage4_route/session.py` for a separate
   immutable E1 session; replace unconditional qualification only with scoped,
   validated per-run evidence at all existing nonzero boundaries. No flag bypass
   or mutation of a running ZeroOnly session.
3. Extend existing watchdog/supervisor for identity-pinned wall-time Gazebo
   containment if adapter/controller dies or sim clock freezes. Current inhibit
   and sim-time rl_sar timeout alone cannot prove a stop then. Containment is
   emergency FAIL, never accepted stationary success; review and zero-only fault
   demonstration precede E1. No alternate velocity publisher.
4. Same-session sensor-only local registration, all required local/saved/live/global
   grid support, unknown blocking, conservative translation/yaw/body/tracking/
   latency/stop reserve, physical evaluation coverage and complete contact intervals.
   No G2–G5 corridor prerequisite; no GT registration.
5. Existing adapter input → final guard → sole final Twist → unchanged rl_sar.
   One proposed .05/0/0 pulse<=2sim s, then measured rest>=5source/3wall s,
   one attempt<=90sim/180wall s. No goal/turn/speed escalation. Actual speed is
   measured, not assumed equal to policy command.

The proposed extension would eventually accept this copy-ready command form,
**NOT implemented or authorized today**:

```bash
/usr/bin/python3 scripts/stage4_session.py \
  --inputs "$reviewed_e1_inputs" --qualification "$reviewed_e1_qualification" \
  --output "$fresh_e1_evidence" --execute-e1
```

PLAN.md specifies artifact contents, measurement/abort prerequisites and entry
points. Numerical safe region/clearance/bounds must come from accepted evidence;
no honest presently executable qualified E1 command exists. After fresh ZeroOnly
PASS and qualified E1, use measured bounds plus path-specific turning/registration/
clearance and a separately authorized existing Start→G1 segment command. Preserve
original G1 and .25m region. Then fresh G1 corridor observations and separate
G1→G2→G3→G4→G5 segments before a continuous run.

## Preservation and artifacts

Evidence root: `experiments/g1_g5/evidence/zero-policy-20261022/`.
`raw/` and `artifacts/` are Git-ignored local evidence, not remotely backed up.
`summary.json`, build/test/review records and checksum index are compact review data.
Raw contains the existing wrapper/observer, JSONL, invocation, all stack/watchdog/
probe logs and SQLite journal. One concise APRIL bundle is copied to the temporary
exchange directory, with a durable identical repository copy.

Independent cleanup check: **27 PID/start identities and both owner tokens absent**,
domain197 idle.525 frozen repository files unchanged during live trial. Original
checkout397 audited files and both Pipe worktrees342 each unchanged; inherited
trial58 checksum entries verified. No foreign process signaled. No commit/push,
nonzero motion, goal, lease, system upgrade or destructive Git operation.

## Sole active PLAN.md NOW including E1 implementation plan

## NOW — Stable Warehouse ZeroOnly → qualified local E1 → Start→G1 (October 22)

Worktree `/home/zyd/program/b2w-embodied-navigation-course-stage4`, branch
`experiment/unattended-g1-g5-20261019`, HEAD `4ae85f4148a89a76b1ad25c5b059c2140be41b85`.
User communicated external architecture CONDITIONAL PASS for immutable ZeroOnly-only
50ms diagnostic policy and explicitly authorized exactly ONE bounded ZeroOnly trial
if boundaries/preflight pass. NONZERO NO-GO. No commit/push or trial retry.

- [x] Preserve inherited records; expected branch/HEAD verified.
- [x] Two deadline branches exempt only ctor-fixed guard.zero_only is True;
  exact zero, other sticky faults and all motion/freshness/watchdog gates retained.
- [x] TDD RED7 expected failures/21 pass; GREEN71 pass; independent static review
  found no blocker. Regular isolated copy build and35 copied source/config/launch
  identities verified; fresh imports resolve new install; host/domain preflight PASS.
- [x] Exactly ONE trial FAILED after watchdog/probe started: first inhibit
  adapter_publication_stalled;716 observed final Twists zero, no lease.
  Full probe acceptance absent.27 owned identities cleaned; no fix/retry.
- [x] Prepare minimal E1 implementation below; E1 implementation/execution not authorized.
- [x] Deliver ONE compact APRIL review bundle; stop before commit/push or nonzero.
- [ ] Independent review of preserved watchdog health predicate failure; exact
  failing record/age not captured. No threshold change or new live approval.
  E1 stays planning-only until local stop/abort/bootstrap qualification is implemented.

Result: [ZeroOnly policy and trial](docs/reviews/B2W_ZeroOnly_Policy_E1_20261022.md).
Evidence: `experiments/g1_g5/evidence/zero-policy-20261022/`.
Architecture basis: `docs/reviews/B2W_Warehouse_Navigation_Architecture_20261022.md`.
Health-stage overrun timing becomes visible on the next successful health callback;
shutdown before it may lose that final diagnostic. No claim of same-callback durability.

### Minimal E1 implementation plan (planning only; NONZERO NO-GO)

Scope: original Warehouse, one sensor-observed open pocket, one forward command
vx=.05, vy=wz=0 for at most 2 advancing simulation seconds, then exact zero.
Normal completion needs measured rest for >=5 source seconds and >=3 wall seconds;
whole attempt ceiling <=90 simulation /180 wall seconds. No Nav2 goal, turn,
velocity escalation, G2–G5 corridor gate or automatic retry. Actual speed/distance
are measured; .05 is a policy command, not a physical speed guarantee.

**Current missing authority:** `stage4_qualification.py:9` always raises. Calls at
`velocity_adapter.py` final nonzero publication, `run_candidate_route.py:64–66`,
and `stage4_route/ros_transport.py:584–585` deliberately prohibit motion.
`stage4_route/session.py:41` returns an immutable ZeroOnly watchdog manifest;
`stage4_control.py:115–117` cannot lease in that mode. ZeroOnly PASS does not
change any of these. An operator flag, armed=False, copied health, JSON boolean,
old physics script, or removal of the unconditional raise is not qualification.

**Minimal review/implementation sequence:**

1. Independently review a local E1 bootstrap qualification contract before code
   changes to that boundary. It must distinguish a conservatively contained
   calibration pulse from the later measured motion envelope. Current
   `stage4_bounds.validate_bounds` requires measured stopping/latency evidence
   (`:23–29`); it cannot honestly accept a new pulse's future measurements.
   Historical numbers may inform a proposed conservative envelope, but cannot
   satisfy these fields merely by being relabeled. If no independently accepted
   bootstrap envelope is available, remain NO-GO rather than fabricating one.
2. Extend the EXISTING `scripts/stage4_session.py` owner and
   `scripts/stage4_route/session.py` acceptance to create a separate immutable,
   per-run E1 session from approved local evidence; never mutate a live ZeroOnly
   session. Replace the unconditional gate only with validated, scoped per-run
   qualification consumed at every existing motion boundary. Match session and
   PID/start identity, source/install hashes, controller/policy/world, bounded
   region/time, mode, command ceiling, evidence hashes and single-use attempt.
   Issue/revoke the existing short lease through `Watchdog.step`, unchanged
   150ms adapter/lease and250ms owner freshness. No general approval switch.
3. Add one bounded pulse phase to that same owned session, sending only the
   existing adapter INPUT `/course_navigation/cmd_vel`, never the final output.
   Keep Nav2 idle, prove no actions/other velocity producers, and bind the
   expected pulse producer identity in graph checks. Reuse final
   `VelocityAdapter.publish` -> `FinalCommandGuard.output` -> sole final topic ->
   unchanged rl_sar path. All motion50ms deadlines, grids, finite values,
   contacts, limits and predictive reserves remain enforced. Keep E1 entry
   separate from route authorization; E1 qualification cannot dispatch a goal.
4. Reuse existing contact journal/interval checks and `IndependentStop` rather
   than a new evaluator. Extend only the scoped E1 prerequisites/measurement
   ingestion missing from the owner. Focused offline rejection tests cover
   wrong mode/session/install, expired/out-of-region permission, excessive
   command, duplicate attempt, lease loss, abort and independent containment.
   Then copied-build identity checks and independent review precede any E1 run.

**Independent stop/abort is the first substantive motion blocker.**
`ros_transport.request_independent_abort:569` writes the immutable inhibit and
records a watermark; `stage4_control.Watchdog:121–124` latches/removes lease.
Both rely on a live final adapter to publish zero. rl_sar command timeout is .5
SIMULATION seconds (`rl_sar_controller.cpp:43,133`); its200Hz update timer uses
ROS time (`:113`). No wall-clock physical-stop guarantee follows if /clock is
frozen, adapter/controller is dead, or transport is stuck.

Proposed smallest Gazebo-specific addition: extend the EXISTING independent
watchdog/owned supervisor, with separately pinned Gazebo-server PID/start/PGID
and wall deadline, to contain the owned simulator on adapter/controller/clock
failure. An owned world-pause acknowledgement may be tried only with a short
wall timeout; failure requires verified PID-targeted server suspension/termination,
with no automatic resume. Design must account for server child identity and
stopping physics threads, not merely the launch wrapper. This is a proposed
boundary requiring review, not an implemented or qualified command. It introduces
no velocity publisher and no alternate actuator path. Freeze/termination is
emergency simulation containment, NOT measured physical rest and never E1 PASS.
Normal abort remains inhibit -> exact zeros -> measured rest, cancel only if
an owned action exists. Fresh source data must prove rest; frozen source time
must fail acceptance. Before moving, demonstrate the owned containment path in
an expressly authorized ZERO session, including dead adapter, controller loss,
frozen /clock, failed pause acknowledgement and sentinel ownership protection.
Do not claim those fault demonstrations were performed in this task.

**Local measurable prerequisites; no entire-course prerequisite:**

- Fresh ZeroOnly bootstrap/probe PASS and owned cleanup; next E1 uses a new owned
  session with matching rebuilt artifacts and rechecks all live prerequisites.
- Same-session LIO-SAM/TF localization, map/odom continuity and advancing source
  times; one open pocket containing initial full body, pulse and all-direction
  stop/abort reachable sets. Use actual sensor support with unknown/offmap veto.
- Existing saved/local/live/global guard grids remain required. For E1, crop or
  restrict scope to the pocket and register only its landmarks; do not silently
  waive saved-grid or registration gates. `contract.registration:92–111` uses
  4..32 finite landmark pairs spanning non-collinear structures over several metres, rejects >.05m translation/>.02rad nonidentity, .10m max
  residual/.05m RMS and .10m alignment p95. Those are existing diagnostic limits,
  not conservative physical error bounds. Bound registration yaw/translation,
  map boundary, body extent, tracking and stop terms independently. No GT in
  registration or navigation; GT may only audit physical movement afterward.
- Existing guard reserve (`stage4_guard.py:149–163`) is stop_translation +
  max_speed*delay + radius*(stop_yaw+max_wz*delay) + uncertainty, radius
  hypot(.75,.43), delay=pose_age+max(.5,abort_latency*max_sim_rate).
  Every required grid's filled-body clearance must exceed that reserve at every
  output. Use accepted conservative bootstrap numbers and local region/time
  scope; no guessed .1m travel bound from command*.seconds. All raw evidence
  references/hashes and uncertainty components remain explicit.
- Complete collision-inventory/contacts paired intervals from declared start
  through end of stop; allowed wheel-ground only; body tilt/height within
  independently accepted stability envelope. Plate0 alone is insufficient.
- Pin independent abort identities; demonstrate its wall detection/containment
  latency bounds before E1, including frozen simulation. Accepted reserve must
  cover that bound and maximum observed/declared simulation rate. Historical
  physics .2x8s ->2.138m, forward stop .079872m and .25 turn ->.432m translation
  remain provisional hints only (`2026-10-11-init-physics.json`).

**E1 success/abort:** Within independently frozen region and speed/yaw/body/
uncertainty/clearance bounds; some positive motion resolved above localization
uncertainty; complete contact intervals; final six-axis exact zeros; normal rest
confirmed with `IndependentStop` (speed<=.03, |wz|<=.03, command speed/|wz|<=.01,
<=.03m hold excursion, >=5 source/3wall seconds); owned cleanup PASS. Record
measured command tracking, peak velocity/yaw, whole-stop excursion, source/wall
abort latency and maximum sim rate. Any exceedance, deadline/freshness/identity/
contact fault, native exit, unknown grid, nonzero unauthorized axis, repeated
attempt or timeout: inhibit immediately, independently contain if stop cannot
be proved, preserve evidence, FAIL and stop. No fix/retry in the same approval.

**Eventual qualified command (PROPOSED CLI, DOES NOT EXIST TODAY):**

```bash
# Future extension of existing stage4_session.py, after implementation, copied
# build/hash verification, independent boundary review and explicit E1 approval.
# Supply actual absolute artifact paths; --execute-e1 alone must confer no authority.
/usr/bin/python3 scripts/stage4_session.py \
  --inputs "$reviewed_e1_inputs" --qualification "$reviewed_e1_qualification" \
  --output "$fresh_e1_evidence" --execute-e1
```

The immutable reviewed inputs must fix .05/0/0, <=2sim pulse, normal rest and
<=90sim/180wall ceiling, owned containment path, pocket and single attempt.
Do not advertise this as presently executable. Existing executable CLI offers
only `--execute-zero-smoke`; no honest qualified E1 command exists yet.

**Fastest Start→G1 after E1:** Review the measured local stop/tracking/abort
result, derive accepted bounds instead of extrapolating command speed; extend
sensor-supported registration/clearance only over Start→G1; certify the exact
selected path and G1 [-.18571805953979492,7.189985752105713] in registered SLAM map,
measured achievable-rate deadline, qualified owner/deployment and separate goal
authority. Then use EXISTING `run_candidate_route.py run --selection ...
--session ... --deployment ... --deadline-evidence ... --mode segment --segment
start-g1 --execute-user-run`. Preserve .25m G1 region and final stop. E1 forward
only cannot qualify turns: require accepted turning bounds for this path or a
separately authorized local turn probe if its geometry requires them. G2–G5 are
later independent segments, not a prerequisite for the first pocket pulse.


## Exact runtime and focused regression diff

```diff
diff --git a/course_navigation/course_navigation/velocity_adapter.py b/course_navigation/course_navigation/velocity_adapter.py
index 6f89a3d..6b91b75 100644
--- a/course_navigation/course_navigation/velocity_adapter.py
+++ b/course_navigation/course_navigation/velocity_adapter.py
@@ -336,7 +336,10 @@ class VelocityAdapter(Node):
             if not getattr(self,'stage4_first_deadline_timing',None):
                 publication_timing['deadline_stage']='prepublish'
                 self.stage4_first_deadline_timing=publication_timing
-            guard.inhibit('adapter_publication_deadline');out=Twist()
+            # Only the manifest-fixed, never-arming observation mode may recover.
+            # Motion mode keeps the same deadline even before it is armed.
+            if guard.zero_only is not True:guard.inhibit('adapter_publication_deadline')
+            out=Twist()
         self.pub.publish(out)
         mark_publication('published')
         if guard is not None and guard.config.get('adapter_heartbeat_path'):
@@ -357,7 +360,8 @@ class VelocityAdapter(Node):
                 if not getattr(self,'stage4_first_deadline_timing',None):
                     publication_timing['deadline_stage']='health'
                     self.stage4_first_deadline_timing=publication_timing
-                guard.inhibit('adapter_health_publication_deadline');self.pub.publish(Twist())
+                if guard.zero_only is not True:
+                    guard.inhibit('adapter_health_publication_deadline');self.pub.publish(Twist())
 
     def stop_stage4(self):
         if getattr(self,'stage4_guard',None) is not None:
diff --git a/tests/test_stage4_health_io.py b/tests/test_stage4_health_io.py
index 597351c..95a4921 100644
--- a/tests/test_stage4_health_io.py
+++ b/tests/test_stage4_health_io.py
@@ -55,9 +55,15 @@ def test_transient_health_atomic_secure_and_failed_replace_keeps_old_record(tmp_
     assert control.read_control(path)==new
 
 
+@pytest.mark.parametrize('zero_only',[True,False])
 @pytest.mark.parametrize('failure',['delay','write','rename'])
-def test_delayed_or_failed_adapter_write_latches_zero_and_never_leases(tmp_path,monkeypatch,failure):
+def test_health_overrun_only_zero_mode_recovers_but_write_faults_latch(tmp_path,monkeypatch,failure,zero_only):
     a,c,out=adapter(tmp_path);replace=control.os.replace
+    if not zero_only:
+        from test_stage4_safety_guard import configured
+        a.stage4_guard,_=configured(tmp_path)
+        a.stage4_guard.config['adapter_heartbeat_path']=str(tmp_path/'health')
+        Path(a.stage4_guard.config['lease_path']).unlink()
     if failure=='delay':
         def delayed(source,target):time.sleep(.065);return replace(source,target)
         monkeypatch.setattr(control.os,'replace',delayed)
@@ -70,15 +76,21 @@ def test_delayed_or_failed_adapter_write_latches_zero_and_never_leases(tmp_path,
     with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
         a.state(state());a.publish()
     g=a.stage4_guard
-    assert ('adapter_health_publication_deadline' if failure=='delay' else 'adapter_health_write:') in g.failure
+    if failure=='delay' and zero_only:assert g.failure is None
+    else:assert ('adapter_health_publication_deadline' if failure=='delay' else 'adapter_health_write:') in g.failure
     assert out and all(v==(0.,0.,0.) for v in out) and not g.armed
-    assert Path(g.config['inhibit_path']).exists() and not Path(g.config['lease_path']).exists()
+    assert Path(g.config['inhibit_path']).exists() is (failure!='delay' or not zero_only)
+    assert not Path(g.config['lease_path']).exists()
     assert not list(tmp_path.glob('.control-*'))
     if failure=='delay':
         monkeypatch.setattr(control.os,'replace',replace)
         with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):a.publish()
         first=control.read_control(g.config['adapter_heartbeat_path'])['first_deadline_timing']
         assert first['deadline_stage']=='health' and first['health_io']['wall_s']>=.05
+        assert first['health_io']['thread_cpu_s']<.03
+        assert first['sections']['published']['wall_s']<.05
+        health=control.read_control(g.config['adapter_heartbeat_path'])
+        assert health['wall']==20.  # completion must not renew original callback age
 
 
 @pytest.mark.parametrize('fault',['missing','stale','conflicting_duplicate','regression','owner'])
diff --git a/tests/test_stage4_startup_repair.py b/tests/test_stage4_startup_repair.py
index 0c70ad0..66ffea1 100644
--- a/tests/test_stage4_startup_repair.py
+++ b/tests/test_stage4_startup_repair.py
@@ -11,9 +11,16 @@ from course_navigation.stage4_guard import FinalCommandGuard
 from course_navigation.stage4_control import read_control
 
 
+@pytest.mark.parametrize('zero_only',[True,False])
 @pytest.mark.parametrize('kind',['sleep','cpu'])
-def test_publication_fault_retains_wall_and_cpu_diagnostic_without_relaxing_deadline(tmp_path,kind):
-    a,c,links,out=fixture();a.stage4_guard=FinalCommandGuard(zero_config(tmp_path))
+def test_publication_overrun_retains_timing_and_only_zero_mode_recovers(tmp_path,kind,zero_only):
+    a,c,links,out=fixture()
+    if zero_only:a.stage4_guard=FinalCommandGuard(zero_config(tmp_path))
+    else:
+        from test_stage4_safety_guard import configured
+        a.stage4_guard,_=configured(tmp_path)
+        a.stage4_guard.config['adapter_heartbeat_path']=str(tmp_path/'health')
+        Path(a.stage4_guard.config['lease_path']).unlink()  # unarmed is NOT ZeroOnly
     original=a.stage4_context
     def measured_context():
         if kind=='sleep':time.sleep(.065)
@@ -24,7 +31,8 @@ def test_publication_fault_retains_wall_and_cpu_diagnostic_without_relaxing_dead
     a.stage4_context=measured_context
     with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
         a.state(state());a.publish()
-    assert a.stage4_guard.failure=='adapter_publication_deadline'
+    assert a.stage4_guard.failure==(None if zero_only else 'adapter_publication_deadline')
+    assert Path(a.stage4_guard.config['inhibit_path']).exists() is (not zero_only)
     assert out[-1]==(0.,0.,0.) and not a.stage4_guard.armed
     health=read_control(a.stage4_guard.config['adapter_heartbeat_path'])
     timing=health['publication_timing']['sections']['prepublish']
@@ -64,7 +72,7 @@ def test_owned_evaluation_bridge_failure_stops_before_watchdog_or_probe(tmp_path
     assert not (tmp_path/'session/lease.json').exists()
 
 
-def test_first_overrun_timing_survives_following_fast_fault_callbacks(tmp_path):
+def test_first_zero_overrun_timing_survives_following_fast_callbacks(tmp_path):
     a,c,links,out=fixture();a.stage4_guard=FinalCommandGuard(zero_config(tmp_path));original=a.stage4_context
     def delayed():time.sleep(.065);return original()
     a.stage4_context=delayed
@@ -76,4 +84,24 @@ def test_first_overrun_timing_survives_following_fast_fault_callbacks(tmp_path):
     assert first['sections']['prepublish']['wall_s']>=.05
     assert second['sections']['prepublish']==first['sections']['prepublish']
     assert first['deadline_stage']==second['deadline_stage']=='prepublish'
-    assert a.stage4_guard.failure=='adapter_publication_deadline' and all(v==(0.,0.,0.) for v in out)
+    assert a.stage4_guard.failure is None and all(v==(0.,0.,0.) for v in out)
+    assert not Path(a.stage4_guard.config['lease_path']).exists()
+
+
+@pytest.mark.parametrize('fault',['owner','lease','nonzero'])
+def test_zero_overrun_does_not_hide_other_sticky_faults(tmp_path,fault):
+    from geometry_msgs.msg import Twist
+    a,c,links,out=fixture();a.stage4_guard=FinalCommandGuard(zero_config(tmp_path));g=a.stage4_guard
+    original=a.stage4_context
+    def delayed():time.sleep(.065);return original()
+    a.stage4_context=delayed
+    with patch('course_navigation.velocity_adapter.time.monotonic',lambda:c.wall):
+        a.state(state());a.publish()
+        assert g.failure is None
+        if fault=='owner':g.config['owner']['start_ticks']='lost-owner'
+        elif fault=='lease':Path(g.config['lease_path']).write_text('{}')
+        else:
+            msg=Twist();msg.linear.x=.001;a.command(msg)
+        a.publish()
+    assert g.failure and Path(g.config['inhibit_path']).exists()
+    assert not g.armed and all(v==(0.,0.,0.) for v in out)
```

## experiments/g1_g5/evidence/zero-policy-20261022/red-tests.txt

```text

tests/test_stage4_health_io.py:79: AssertionError
=========================== short test summary info ============================
FAILED tests/test_stage4_startup_repair.py::test_publication_overrun_retains_timing_and_only_zero_mode_recovers[sleep-True]
FAILED tests/test_stage4_startup_repair.py::test_publication_overrun_retains_timing_and_only_zero_mode_recovers[cpu-True]
FAILED tests/test_stage4_startup_repair.py::test_first_zero_overrun_timing_survives_following_fast_callbacks
FAILED tests/test_stage4_startup_repair.py::test_zero_overrun_does_not_hide_other_sticky_faults[owner]
FAILED tests/test_stage4_startup_repair.py::test_zero_overrun_does_not_hide_other_sticky_faults[lease]
FAILED tests/test_stage4_startup_repair.py::test_zero_overrun_does_not_hide_other_sticky_faults[nonzero]
FAILED tests/test_stage4_health_io.py::test_health_overrun_only_zero_mode_recovers_but_write_faults_latch[delay-True]
7 failed, 21 passed in 1.29s
```

## experiments/g1_g5/evidence/zero-policy-20261022/green-tests.txt

```text
.......................................................................  [100%]
71 passed in 1.56s
```

## experiments/g1_g5/evidence/zero-policy-20261022/build-result.json

```text
{
  "argv": [
    "/usr/bin/colcon",
    "--log-base",
    "/home/zyd/program/b2w-embodied-navigation-course-stage4/experiments/g1_g5/evidence/zero-policy-20261022/raw/copy-build-log",
    "build",
    "--base-paths",
    "course_navigation",
    "--packages-select",
    "course_navigation",
    "--build-base",
    "/home/zyd/program/b2w-embodied-navigation-course-stage4/experiments/g1_g5/evidence/zero-policy-20261022/artifacts/build",
    "--install-base",
    "/home/zyd/program/b2w-embodied-navigation-course-stage4/experiments/g1_g5/evidence/zero-policy-20261022/artifacts/install",
    "--executor",
    "sequential"
  ],
  "exit_code": 0,
  "wall_s": 0.9853047979995608
}
```

## experiments/g1_g5/evidence/zero-policy-20261022/index.json

```text
{
  "kind": "reviewed_zeroonly_policy_and_one_trial",
  "head": "4ae85f4148a89a76b1ad25c5b059c2140be41b85",
  "runtime_source_changed": [
    "course_navigation/course_navigation/velocity_adapter.py"
  ],
  "source_changes_after_trial": 0,
  "offline_tests": {
    "red_failed": 7,
    "red_passed": 21,
    "green_passed": 71
  },
  "copy_build_passed": true,
  "copied_files_verified": 35,
  "attempts": 1,
  "full_probe_pass": false,
  "first_inhibit": "adapter_publication_stalled",
  "owned_cleanup_pass": true,
  "probe_cleanup_pass": false,
  "motion_authorized": false,
  "motion_attempts": 0,
  "report": "docs/reviews/B2W_ZeroOnly_Policy_E1_20261022.md",
  "raw_git_ignored": true,
  "raw_remotely_backed_up": false
}
```

## experiments/g1_g5/evidence/zero-policy-20261022/review.txt

```text
External architecture decision: user communicated CONDITIONAL PASS for immutable ZeroOnly-only50ms policy. User authorized one bounded zero trial after passing boundaries/preflight; no nonzero.
Independent static code review: no blocking defect; checks ctor-captured mode, unchanged motion deadline/freshness/qualification/other faults. Health-stage overrun first timing visible only on following successful callback (preexisting instrumentation limit). Static review does not authorize another trial or motion.

Post-trial independent result sanity review: FAILED; source deadline policy recovered as designed and bootstrap reached watchdog/probe; compound watchdog predicate and receiver gap do not prove exact health age/cause. Probe stationary cleanup failed while OS owned cleanup passed. No further code/live action authorized.
```

## experiments/g1_g5/evidence/zero-policy-20261022/git-status.txt

```text
## experiment/unattended-g1-g5-20261019...origin/experiment/unattended-g1-g5-20261019
 M PLAN.md
 M course_navigation/course_navigation/velocity_adapter.py
 M docs/STAGE4_RUNTIME_READINESS.md
 M tests/test_stage4_health_io.py
 M tests/test_stage4_startup_repair.py
?? docs/reviews/B2W_Stage4_Health_IO_ZeroOnly_4ae85f4_20261022.md
?? docs/reviews/B2W_Warehouse_Navigation_Architecture_20261022.md
?? docs/reviews/B2W_ZeroOnly_Policy_E1_20261022.md
?? experiments/g1_g5/evidence/health-io-zeroonly-20261022/
?? experiments/g1_g5/evidence/navigation-unblock-20261022/
?? experiments/g1_g5/evidence/zero-policy-20261022/
```

## Content identities

SHA256 manifest: `experiments/g1_g5/evidence/zero-policy-20261022/SHA256SUMS`, SHA256 `dd3cbe903921ea7a15dfa78dc0f821dc30a1e4181512c14473b9971d18b59f7b`.
New source SHA256 `dc97d0445ec790fc0b5c2d171070f38cb9ad5e5a7d91e01a5fb9cdf86f6136c8`.
Preflight SHA256 `4775fb9601fe5b0bf342f7f0920eda29153330a1b426a26bd3c8c13ebf5c9514`.
Trial summary SHA256 `5af02a88a894b27eb0ab25e814ac0a23e1740b631ba77db6ebc78c0252d1670b`.
Owned cleanup/preservation SHA256 `69e540dfcc98fa6246b4451842b3980751c055a565a73852f91db72efdc12603`.
Raw evidence remains in repository local ignored tree, not bundled or remotely backed up.
