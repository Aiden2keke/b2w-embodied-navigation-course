# APRIL Review Bundle — B2-W dispatch attribution and offline E1

# Warehouse dispatch attribution and offline E1 — 2026-10-23

Historical181.573ms gap **UNRESOLVED**; no production dispatch patch and **0 new Gazebo trials**. Latest full ZeroOnly result remains **FAIL**; E1/NONZERO remains **NO-GO**. Completed non-authorizing E1 request accounting,237 focused regressions, fresh isolated copy build and actual installed zero-only adapter smoke. This is concrete offline progress, not motion qualification.

## A. Exact historical cause and scheduling path

Sequence696 starts at88017.609832271;697 starts88017.791405108:181.572837ms. Sequence696 published after27.314479ms;697's previous_health_io records29.911981ms. Thus approximately**124.346377ms after health I/O** remains until the next callback starts. This decomposition uses adjacent original timing records, not observer receipt times. The rejected record was191.620133ms old when Watchdog.step checked it. The150ms limit remains correct and unchanged.

Original evidence lacks callback entry/exit events, timer-ready times, worker stacks and native-thread scheduling samples within that124ms interval. A process-level S state does not distinguish GIL, executor waiting, filesystem wait or another thread. Map stamp27 occurred nearby; no causal attribution follows. No retrospective trace can honestly recover the missing stack.

Actual path, verified against installed Humble code and current source:

1. `rclpy.spin(node)` uses SingleThreadedExecutor. Command, odom, contacts, structure, all three grid subscriptions and the steady50ms timer use the default MutuallyExclusiveCallbackGroup. TF listener subscriptions have a ReentrantCallbackGroup but the same single executor thread. Callback-group changes alone do not create CPU independence.
2. The executor's ready snapshot yields timers before subscriptions, but does not preempt an active handler or reconsider a newly ready timer between every subscription in the existing snapshot. Taking/deserializing a message occurs before user callback entry.
3. Grid callback validates dimensions/quaternion, copies `msg.data` into a tuple and calls GridPreparation.submit. Its condition-protected exchange preserves at most four immutable pending generations, immediate invalidation of changed geometry, accepted-payload refresh and stale-generation exclusion. It is a Python thread in the same process, not GIL isolation.
4. Worker prepare validates every cell, converts occupancy, canonical-JSON hashes, converts tuple to NumPy and builds ClearanceGrid/cKDTree. These stages share the interpreter GIL. Locks protect short metadata/install sections; CPU work is outside the guard lock. No long held guard lock was established by these probes.
5. Publish checks ownership/inhibit/localization/TF/guard, publishes through the sole final publisher, then synchronously writes original callback-wall health using transient atomic write_adapter_health (no fsync). Releasing/reacquiring the GIL around filesystem operations can extend several short I/O operations while worker CPU runs. This differs from proving storage hardware was blocked.

## B. Offline reproduction and correction decision

Five bounded probes used the real VelocityAdapter/GridPreparation and real ROS serialized800x800 OccupancyGrid plus800x800 Costmap ingress; local grids120x120, or actual100x100 in the dynamic run. Separate producer process avoided injecting producer GIL cost into the adapter. Sparse and unknown-filled Warehouse-shaped synthetic cells were used because the original cell payload was not recorded. Ready variants supplied synthetic coherent /clock, SLAM odom and TF; no ground truth, simulator, controller, goal or motion input. They do not represent real localization qualification.

Observations are bounded in-memory enter/leave wall/thread-CPU events, timer lateness, worker validation/hash/array/tree stages, Python frame snapshots and a separate process sampling native-thread schedstat/wchan. Data flushes after shutdown; no new per-cycle disk tracing or production instrumentation. The12000-event ring may retain only a tail in the32s dynamic run; counts below describe retained observations.

| Probe | Retained publish starts | Max start gap ms | p95 gap ms | Max publish wall ms |
|---|---:|---:|---:|---:|
| ingress-baseline-1 | 311 | 143.195 | 83.827 | 142.663 |
| ingress-dynamic-baseline-1 | 554 | 131.854 | 82.931 | 114.844 |
| ingress-parked-control-1 | 320 | 64.906 | 50.728 | 12.328 |
| ingress-ready-baseline-1 | 287 | 143.076 | 104.881 | 121.927 |
| ingress-warehouse-1 | 314 | 95.671 | 77.204 | 91.222 |

In the ready probe's maximum gap, publication took121.927ms wall but3.017ms thread CPU. Worker validation,33.163ms CPU JSON encoding and51.436ms wall/30.316ms CPU NumPy conversion overlap.47/59 external samples show the main thread in futex wait while the worker runs in50/59; main runnable scheduling delay was7.498ms. In the simpler unknown-grid maximum, main runnable delay was0.007197ms and41/43 samples were futex waits. Together with frame locations, this supports **same-process worker/GIL interference in these probes**, rather than long map subscription CPU work or host runnable starvation alone.

The paired ready parked-worker attribution control kept subscriptions, data copies and invalidation, but parked prepare and accepted no grid. It reduced max publish121.927→12.328ms, p95 start gap104.881→50.728ms and max start gap143.076→64.906ms. This control is deliberately unable to bootstrap; it is not a production candidate or a disabled safety check in a live session. Single paired trials establish a contribution, not a latency guarantee.

None reproduced181.573ms or the original124.346ms post-health interval. The matching ready symptom occurs mostly *inside* publication. JSON chunking, separate callback groups, a process worker or threshold changes therefore cannot be selected as the actual historical correction from current evidence. **No dispatch correction was invented.** Adapter/guard/geometry/control/watchdog bytes remain unchanged. Independent read-only reviewer agrees. Live work stopped under mission step4.

## C. Full ZeroOnly result and cleanup

No new Gazebo trial:0/1 conditional allowance used. Latest full result remainsFAIL: original699/699 exact-zero six-axis Twists,534 paired contact candidates/inventories, bootstrap/all grids/watchdog/probe started; watchdog correctly inhibited sequence696. Stationary acceptance, full runtime graph and contact endpoint closure were not accepted; probe and probe cleanup FAIL. Prior owned OS cleanup PASS. No new full-probe PASS is claimed.

Fresh **offline installed adapter smoke** observed20/20 exact-zero six-axis Twists, one final publisher,50ms steady timer and expected subscription/group interfaces. Worker joined; fresh post-destruction domain209 graph contained only the cleanup checker. All five producer children reaped0, workers joined, recorded probe-owner PID/start identities absent, no probe survivors and no lease files. No broad process signals, Pipe/original-checkout edits or simulator containment claims.

## D. E1 offline implementation completed

Existing qualification module now checks request shape/accounting: exact six-axis .05-forward command, pulse<=2 source seconds, rest>=5source/3wall seconds, attempt<=90source/180wall, finite local region/source scope, new session/attempt identifiers, and SHA-bound references for ZeroOnly result, bootstrap envelope, independent abort and local clearance. A referenced summary must claim full ZeroOnly and owned cleanup PASS. File hashes and summary claims **do not authenticate evidence or accept its semantics**; tests deliberately use synthetic fixtures.

Existing owner offers read-only `--check-e1-request`, which creates no owner/output/lease. The exact future spelling `--qualification ... --output ... --execute-e1` is reserved and always invokes unchanged unconditional require_motion_qualification before input reads or session creation. An approval boolean, well-formed request or forged PASS cannot execute. No SessionOwner/Watchdog/IndependentStop behavior or physical qualification boundary changed.

TDD records:26 initial failures for missing feature, then a focused overflow rejection failure, then60 focused E1/boundary tests PASS; independent reviewer reproduced60PASS. Final root regression suite: **237 passed in22.21s**. Scope includes Stage4 grid/health/session/bounds/transport/final-guard and unconditional motion tests. `git diff --check` PASS; original veto AST identical.

## E. Remaining blockers to controlled motion

1. Attribute historical residual dispatch interval and establish/test/review a supported correction; then fresh full ZeroOnly PASS including stationary, grids, graph, contacts, watchdog/probe cleanup and owned cleanup.
2. Independently accepted conservative local bootstrap/stopping envelope. Proposed future pulse measurements cannot qualify their own prior safety bounds.
3. Identity-pinned independent wall-clock abort/owned simulator containment with expressly authorized zero-only fault demonstrations (dead adapter/controller, frozen clock, failed pause acknowledgement, ownership sentinel). Current inhibit/zero publication alone is insufficient on adapter death or frozen simulation. Emergency freeze/termination is not measured normal rest.
4. Sensor-only local registration and saved/live/local/global unknown/offmap-vetoed filled-body clearance, uncertainty and stopping/abort reachable sets; complete body contact/stability and advancing-source rest qualification.
5. Reviewed existing-owner extension enforcing per-run deployment/process/producer/scope/evidence identities, single attempt, current scope, live lease/abort and measured pulse/stop acceptance, with explicit E1 authority. E1 forward calibration does not authorize Start→G1 route/turn execution.

**F. Shortest next action:** extend the bounded offline ingress probe to the remaining original contact/structure/clock traffic and observed scheduling cadence, capturing the post-health→timer-dispatch interval. Change only a mechanism that reproduces that interval, then reconsider the one conditional ZeroOnly trial. No identical Gazebo retry. After qualified E1, use the existing Start→G1 segment execution and local path/turn qualification from the preserved E1 plan; no new framework.

## Verification, identities and Git

Fresh regular-copy navigation+bringup build:2 packages PASS in1.43s. All35 source/config/launch copies match actual installed bytes under `artifacts/install`; fresh imports come from that prefix. Installed graph/smoke and independently pinned LIO/GTSAM/world/contact artifacts checked. Direct read-only policy/model/ONNX/nav2-controller/actual RL-SAR/watchdog hashes PASS; see runtime-identities.json and additional-pins.json. Build emitted inherited incomplete-LIO-prefix and intentionally disabled-bytecompile warnings; neither is counted as physical validation.

Automatic approval review rejected an owner CLI validation invocation as potentially launching Gazebo; it did not execute. Completed the needed pins with direct read-only hashes, without owner invocation. No permission-blocked required verification remains, and no workaround launched a session.

HEAD remains`cff87a8fef98d5d58b1b7bf2ed05a6e6c2b94263` on experiment branch. Source/test diff is`source.diff` SHA256`4a4571b658d4d0a2ba8b21c7fad6586f171dbc25ef20a6534da6583235999365`. Changes are audited and verified but **uncommitted/unpushed**: applicable AGENTS.md says “No commit, push, full Gazebo repeat, motion goal or nonzero authorization before external review.” User authorizes commits only when normal approvals permit; local code review does not replace that external gate. The sole PLAN.md/NOW is updated and predecessor/E1 design preserved. All56 predecessor raw hashes reverified; prior bundle/checksum remain untouched.

Raw evidence is local-only/ignored, indexed by RAW_SHA256SUMS. Compact report, summaries, reviews, source diff and identities are repository records. The one final APRIL bundle/checksum is copied to the exchange directory with Python-only adaptation of the existing delivery tool and verified by SHA256/readback; historical files are not overwritten.

## Audited source/test diff

```diff
diff --git a/course_navigation/course_navigation/stage4_qualification.py b/course_navigation/course_navigation/stage4_qualification.py
index 77f33a2..32633d4 100644
--- a/course_navigation/course_navigation/stage4_qualification.py
+++ b/course_navigation/course_navigation/stage4_qualification.py
@@ -11,3 +11,73 @@ def require_motion_qualification():
         'UNQUALIFIED_FOR_MOTION: predictive adapter veto, action-independent '
         'owned abort, sensor-only bounded registration and all-body/end-watermarked '
         'physical evaluation are not qualified; see docs/LUNA_VALIDATION_HANDOFF.md')
+
+
+def validate_e1_pulse_request(request,*,session_id=None):
+    """Read-only request accounting, never qualification or approval ingestion.
+
+    Hashes bind supporting bytes; they do not establish conservative physical
+    bounds, freshness, independent containment or acceptance by a reviewer.
+    The unconditional motion veto above remains the only execution decision.
+    """
+    import hashlib
+    import json
+    import math
+    from pathlib import Path
+
+    def finite(value):
+        try:return type(value) in (int,float) and math.isfinite(value)
+        except OverflowError:return False
+
+    def pinned(reference):
+        if type(reference) is not dict or set(reference)!={'path','sha256'}:
+            raise ValueError('Explicit evidence path and SHA256 required')
+        path=reference['path'];sha=reference['sha256']
+        if (type(path) is not str or not Path(path).is_absolute() or
+                type(sha) is not str or len(sha)!=64 or any(c not in '0123456789abcdef' for c in sha)):
+            raise ValueError('Absolute evidence path and lowercase SHA256 required')
+        file=Path(path)
+        if file.is_symlink() or not file.is_file():raise ValueError('Regular nonsymlink evidence file required')
+        data=file.read_bytes()
+        if hashlib.sha256(data).hexdigest()!=sha:raise ValueError('E1 evidence content identity changed')
+        return data
+
+    keys={'kind','version','mode','session_id','attempt_id','command','pulse_source_s','rest_source_s',
+          'rest_wall_s','attempt_source_s','attempt_wall_s','scope','zero_only_result','evidence'}
+    if type(request) is not dict or set(request)!=keys:raise ValueError('Exact offline E1 request fields required')
+    r=request
+    if (r['kind']!='stage4_e1_pulse_request' or type(r['version']) is not int or r['version']!=1 or
+            r['mode']!='e1_forward_pulse' or
+            any(type(r[k]) is not str or not r[k].strip() for k in ('session_id','attempt_id')) or
+            (session_id is not None and r['session_id']!=session_id)):
+        raise ValueError('New session-bound single forward pulse request required')
+    command=r['command'];expected=dict(linear_x=.05,linear_y=0.,linear_z=0.,angular_x=0.,angular_y=0.,angular_z=0.)
+    if (type(command) is not dict or set(command)!=set(expected) or
+            any(not finite(command[k]) or command[k]!=v for k,v in expected.items())):
+        raise ValueError('Only the exact six-axis .05 forward pulse may be described')
+    times=[r[k] for k in ('pulse_source_s','rest_source_s','rest_wall_s','attempt_source_s','attempt_wall_s')]
+    if (not all(finite(v) for v in times) or not 0<r['pulse_source_s']<=2. or
+            r['rest_source_s']<5. or r['rest_wall_s']<3. or
+            not r['pulse_source_s']+r['rest_source_s']<=r['attempt_source_s']<=90. or
+            not r['rest_wall_s']<=r['attempt_wall_s']<=180.):
+        raise ValueError('Bounded pulse, measured-rest windows and whole-attempt ceilings required')
+    scope=r['scope']
+    if type(scope) is not dict or set(scope)!={'valid_from_sim','valid_until_sim','region'}:
+        raise ValueError('Explicit local spatial and source-time scope required')
+    region=scope['region'];start=scope['valid_from_sim'];end=scope['valid_until_sim']
+    if (type(region) is not list or len(region)!=4 or not all(finite(v) for v in [start,end,*region]) or
+            not 0<=start<end or end-start<r['attempt_source_s'] or
+            region[0]>=region[2] or region[1]>=region[3]):
+        raise ValueError('Finite nonempty pocket and scope covering the whole attempt required')
+    evidence=r['evidence']
+    if type(evidence) is not dict or set(evidence)!={'bootstrap_envelope','independent_abort','local_clearance'}:
+        raise ValueError('Pinned bootstrap, independent abort and local-clearance references required')
+    for reference in evidence.values():pinned(reference)
+    zero=json.loads(pinned(r['zero_only_result']))
+    if (type(zero) is not dict or zero.get('kind')!='stage4_zero_session' or
+            zero.get('zero_motion_smoke_pass') is not True or zero.get('motion_authorized') is not False or
+            type(zero.get('cleanup')) is not dict or zero['cleanup'].get('passed') is not True):
+        raise ValueError('Full ZeroOnly probe and owned cleanup PASS required')
+    return dict(request_valid=True,motion_qualified=False,motion_authorized=False,
+        limitation='Request shape/content identities only; evidence semantics, freshness, containment, '
+                   'conservative local bounds, deployment identity and single-use enforcement remain unqualified')
diff --git a/scripts/stage4_session.py b/scripts/stage4_session.py
index 8faebf2..624bf8d 100644
--- a/scripts/stage4_session.py
+++ b/scripts/stage4_session.py
@@ -151,13 +151,29 @@ def execute(value,output):
 
 def main(argv=None):
     p=argparse.ArgumentParser(description=__doc__);p.add_argument('--inputs',type=Path,required=True)
-    p.add_argument('--output',type=Path,required=True);p.add_argument('--execute-zero-smoke',action='store_true')
+    p.add_argument('--output',type=Path)
+    modes=p.add_mutually_exclusive_group()
+    modes.add_argument('--execute-zero-smoke',action='store_true')
+    modes.add_argument('--check-e1-request',action='store_true',
+        help='Read-only request/accounting check; confers no motion authority')
+    modes.add_argument('--execute-e1',action='store_true',
+        help='Reserved future interface; always rejected as unqualified')
+    p.add_argument('--qualification',type=Path,help='Reserved future evidence path; cannot authorize motion')
     a=p.parse_args(argv)
     try:
+        from course_navigation.stage4_qualification import require_motion_qualification,validate_e1_pulse_request
+        # Reject before reading artifacts, creating an owner or launching anything.
+        if a.execute_e1:require_motion_qualification()
+        if a.qualification is not None:raise ValueError('Qualification artifacts cannot authorize this zero-only owner')
+        if a.check_e1_request:
+            validate_e1_pulse_request(read_control(a.inputs))
+            print('E1 request shape/content identities verified; NONZERO remains NO-GO')
+            return 0
+        if a.output is None:raise ValueError('--output required for zero-only inputs/session')
         value=validate_inputs(a.inputs)
         if not a.execute_zero_smoke:print('Inputs verified; no session launched');return 0
         return execute(value,a.output)
     except (ValueError,KeyError,TypeError,OSError,RuntimeError) as exc:
-        print('Stage4 zero-only session rejected: '+str(exc),file=sys.stderr);return 2
+        print('Stage4 session rejected: '+str(exc),file=sys.stderr);return 2
 
 if __name__=='__main__':raise SystemExit(main())
diff --git a/tests/test_stage4_e1_offline.py b/tests/test_stage4_e1_offline.py
new file mode 100644
index 0000000..289368f
--- /dev/null
+++ b/tests/test_stage4_e1_offline.py
@@ -0,0 +1,113 @@
+"""Offline E1 request checks never create motion authority or launch a session."""
+import copy
+import hashlib
+import json
+from pathlib import Path
+import subprocess
+import sys
+import pytest
+from course_navigation import stage4_qualification as qualification
+
+ROOT=Path(__file__).resolve().parents[1]
+
+
+def reference(path,value):
+    path.write_text(json.dumps(value))
+    return dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
+
+
+def request(tmp_path):
+    zero=reference(tmp_path/'zero.json',dict(kind='stage4_zero_session',zero_motion_smoke_pass=True,
+        cleanup=dict(passed=True),motion_authorized=False))
+    refs={name:reference(tmp_path/(name+'.json'),dict(review_pending=True))
+          for name in ('bootstrap_envelope','independent_abort','local_clearance')}
+    return dict(kind='stage4_e1_pulse_request',version=1,mode='e1_forward_pulse',session_id='new-e1',attempt_id='one',
+        command=dict(linear_x=.05,linear_y=0.,linear_z=0.,angular_x=0.,angular_y=0.,angular_z=0.),
+        pulse_source_s=2.,rest_source_s=5.,rest_wall_s=3.,attempt_source_s=90.,attempt_wall_s=180.,
+        scope=dict(valid_from_sim=10.,valid_until_sim=100.,region=[-3.,-3.,3.,3.]),
+        zero_only_result=zero,evidence=refs)
+
+
+def check(value,**kwargs):
+    # An explicit assertion gives a feature-missing failure before implementation.
+    assert hasattr(qualification,'validate_e1_pulse_request'), 'Missing offline E1 request validation'
+    return qualification.validate_e1_pulse_request(value,**kwargs)
+
+
+def test_structural_success_never_qualifies_motion_or_creates_authority(tmp_path):
+    value=request(tmp_path);before=copy.deepcopy(value)
+    result=check(value,session_id='new-e1')
+    assert result['request_valid'] is True
+    assert result['motion_qualified'] is False and result['motion_authorized'] is False
+    assert value==before
+    assert sorted(p.name for p in tmp_path.iterdir())==['bootstrap_envelope.json','independent_abort.json','local_clearance.json','zero.json']
+    with pytest.raises(ValueError,match='UNQUALIFIED_FOR_MOTION'):qualification.require_motion_qualification()
+
+
+@pytest.mark.parametrize('fault',['mode','session','attempt','extra','command','axis','bool_axis','nan','pulse','rest_source','rest_wall','sim_ceiling','wall_ceiling','scope_time','scope_region','evidence_missing','relative','hash','replaced','zero_failed','cleanup_failed','zero_authorized','huge_number','symlink','bool_version','missing_axis'])
+def test_invalid_or_unbound_requests_rejected(tmp_path,fault):
+    value=request(tmp_path)
+    if fault=='huge_number':value['pulse_source_s']=10**400
+    elif fault=='symlink':
+        link=tmp_path/'link.json';link.symlink_to(value['evidence']['local_clearance']['path'])
+        value['evidence']['local_clearance']['path']=str(link)
+    elif fault=='bool_version':value['version']=True
+    elif fault=='missing_axis':del value['command']['linear_z']
+    elif fault=='mode':value['mode']='segment'
+    elif fault=='session':value['session_id']='other'
+    elif fault=='attempt':value['attempt_id']=''
+    elif fault=='extra':value['motion_authorized']=True
+    elif fault=='command':value['command']['linear_x']=.051
+    elif fault=='axis':value['command']['angular_y']=.001
+    elif fault=='bool_axis':value['command']['linear_y']=False
+    elif fault=='nan':value['pulse_source_s']=float('nan')
+    elif fault=='pulse':value['pulse_source_s']=2.001
+    elif fault=='rest_source':value['rest_source_s']=4.99
+    elif fault=='rest_wall':value['rest_wall_s']=2.99
+    elif fault=='sim_ceiling':value['attempt_source_s']=90.01
+    elif fault=='wall_ceiling':value['attempt_wall_s']=180.01
+    elif fault=='scope_time':value['scope']['valid_until_sim']=10.
+    elif fault=='scope_region':value['scope']['region']=[0.,0.,0.,1.]
+    elif fault=='evidence_missing':del value['evidence']['independent_abort']
+    elif fault=='relative':value['evidence']['local_clearance']['path']='relative.json'
+    elif fault=='hash':value['evidence']['local_clearance']['sha256']='A'*64
+    elif fault=='replaced':Path(value['evidence']['local_clearance']['path']).write_text('{}')
+    else:
+        path=Path(value['zero_only_result']['path']);zero=json.loads(path.read_text())
+        if fault=='zero_failed':zero['zero_motion_smoke_pass']=False
+        elif fault=='cleanup_failed':zero['cleanup']['passed']=False
+        elif fault=='zero_authorized':zero['motion_authorized']=True
+        value['zero_only_result']=reference(path,zero)
+    with pytest.raises(ValueError):check(value,session_id='new-e1')
+
+
+def test_request_scope_cannot_omit_stop_time_or_allow_unbounded_wait(tmp_path):
+    value=request(tmp_path);value['scope']['valid_until_sim']=16.
+    with pytest.raises(ValueError):check(value,session_id='new-e1')
+    value=request(tmp_path);value['attempt_wall_s']=2.
+    with pytest.raises(ValueError):check(value,session_id='new-e1')
+
+
+def test_offline_cli_checks_request_without_environment_or_output(tmp_path):
+    import os
+    from course_navigation.stage4_control import write_control
+    value=request(tmp_path);path=tmp_path/'request.json';write_control(path,value)
+    env={key:value for key,value in os.environ.items() if key not in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY')}
+    result=subprocess.run([sys.executable,str(ROOT/'scripts/stage4_session.py'),'--inputs',str(path),
+        '--check-e1-request'],capture_output=True,text=True,env=env)
+    assert result.returncode==0, result.stderr
+    assert 'NONZERO remains NO-GO' in result.stdout
+    assert not (tmp_path/'lease.json').exists()
+
+
+def test_reserved_e1_execution_interface_unconditionally_rejects_before_io(tmp_path):
+    from course_navigation.stage4_control import write_control
+    value=request(tmp_path);path=tmp_path/'request.json';write_control(path,value)
+    approval=tmp_path/'qualification.json';write_control(approval,dict(motion_authorized=True,approved=True))
+    for source in (path,tmp_path/'missing.json'):
+        output=tmp_path/'session'
+        result=subprocess.run([sys.executable,str(ROOT/'scripts/stage4_session.py'),'--inputs',str(source),
+            '--qualification',str(approval),'--output',str(output),'--execute-e1'],capture_output=True,text=True)
+        assert result.returncode==2
+        assert 'UNQUALIFIED_FOR_MOTION' in result.stderr
+        assert not output.exists()

```

## Fresh test output

```text
........................................................................ [ 30%]
........................................................................ [ 60%]
........................................................................ [ 91%]
.....................                                                    [100%]
237 passed in 22.21s
```

## Runtime identity evidence

```json
{
  "modules": {
    "course_navigation.velocity_adapter": {
      "path": "/home/zyd/program/b2w-embodied-navigation-course-stage4/experiments/g1_g5/evidence/dispatch-e1-20261023/artifacts/install/course_navigation/lib/python3.10/site-packages/course_navigation/velocity_adapter.py",
      "sha256": "dc97d0445ec790fc0b5c2d171070f38cb9ad5e5a7d91e01a5fb9cdf86f6136c8"
    },
    "course_navigation.stage4_guard": {
      "path": "/home/zyd/program/b2w-embodied-navigation-course-stage4/experiments/g1_g5/evidence/dispatch-e1-20261023/artifacts/install/course_navigation/lib/python3.10/site-packages/course_navigation/stage4_guard.py",
      "sha256": "8c3c16fd3053f9b4916a4fd7a06ecd55c4bc9038d16ef37aadad916fb05486f3"
    },
    "course_navigation.stage4_control": {
      "path": "/home/zyd/program/b2w-embodied-navigation-course-stage4/experiments/g1_g5/evidence/dispatch-e1-20261023/artifacts/install/course_navigation/lib/python3.10/site-packages/course_navigation/stage4_control.py",
      "sha256": "4c9e32b169dbe8d0ced5ea83cbcc8bed1ca99c96c139cac5c20c74d53e2525be"
    },
    "course_navigation.stage4_qualification": {
      "path": "/home/zyd/program/b2w-embodied-navigation-course-stage4/experiments/g1_g5/evidence/dispatch-e1-20261023/artifacts/install/course_navigation/lib/python3.10/site-packages/course_navigation/stage4_qualification.py",
      "sha256": "98725d1fc97a23ca0fe4f19893999158f1b39c97903fef6c158ef120b4301620"
    }
  },
  "copy_count": 35,
  "offline_final_twists": {
    "count": 20,
    "nonzero": 0
  },
  "timers": [
    {
      "period_ns": 50000000,
      "group": "MutuallyExclusiveCallbackGroup"
    }
  ],
  "final_publishers": [
    {
      "node": "velocity_adapter",
      "type": "geometry_msgs/msg/Twist"
    }
  ],
  "cleanup": [
    [
      "offline_cleanup_check",
      "/"
    ]
  ],
  "raw_index": {
    "raw_file_count": 64,
    "raw_total_bytes": 26383282,
    "raw_manifest_sha256": "ab476511368fe67b314f6278cc09170385e4bf5f02872a582b94fac1fc98e46a",
    "raw_remote_backed_up": false,
    "artifacts_remote_backed_up": false,
    "source_diff_sha256": "4a4571b658d4d0a2ba8b21c7fad6586f171dbc25ef20a6534da6583235999365"
  }
}
```
