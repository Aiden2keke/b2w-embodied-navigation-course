# APRIL Review Bundle — B2-W Watchdog Clock Race Fix

**Review state:** applied offline patch; independent review requested. No commit, push, Gazebo trial, nonzero command, goal, or lease.

## Checkout and preservation

- Repository: `/home/zyd/program/b2w-embodied-navigation-course-stage4`
- Branch: `experiment/unattended-g1-g5-20261019`
- HEAD: `4ae85f4148a89a76b1ad25c5b059c2140be41b85`
- Expected base: `4ae85f4148a89a76b1ad25c5b059c2140be41b85`
- Historical ZeroOnly checkpoint: all 29 previously audited files remain byte-identical; its earlier PASS policy/evidence remains uncommitted and unchanged. `PLAN.md` and its existing E1 plan were not edited by this fix.
- Prior reviewed policy bundle SHA256: `267a8a51b94e68baa4f4b51772ff41a1180390aad89d899bf56bf1990dc17f7f`; 79 prior evidence manifest entries reverified.

## Diagnosis and change

# Watchdog clock ordering fix — independent review, 2026-10-23

## Result

The offline race fix is applied in this worktree and the patched package was rebuilt in a fresh isolated regular install. Focused regressions pass. No Gazebo trial, motion lease, nonzero command, goal, commit, or push was run. This change is ready for independent review; retain the existing stop before commit/push or another ZeroOnly trial.

## Historical first fault

The October 22 live watchdog failed with `adapter_publication_stalled`, but its zero-byte watchdog log and journals did not preserve the rejected health record or sampled monotonic wall. Therefore the exact historical failed subcondition remains unknown; the nearby 159.635 ms final-command receive gap is not the watchdog age operand and does not prove a genuine health stall. The fix addresses a separately demonstrated deterministic race, not a claim that the live failure was a false positive.

## Source change

In `course_navigation/course_navigation/stage4_control.py`, `Watchdog.step()` now reads the adapter health and owner heartbeat snapshots before taking its production monotonic sample. It validates those original record timestamps against the unchanged 150 ms and 250 ms limits. It still rejects future, stale, missing, nonfinite, invalid-identity, duplicate/conflicting and regressed records. No timestamp is rewritten, clamped, retried or refreshed. Failure still latches the original inhibit reason and removes any lease.

The first failure captures one bounded in-memory snapshot with sampled wall time, health and owner identity/sequence/timestamp, computed health age and exact named predicate. `scripts/stage4_safety_watchdog.py` emits that snapshot as one flushed JSON line to its existing redirected log on failure only. There is no per-cycle logging or extra thread. The ZeroOnly adapter policy and all motion-mode 50 ms deadlines/interlocks are unchanged.

Deterministic reproduction showed the old CLI sampled 20.000 before a read whose atomic update published timestamp 20.010 and failed on negative age. The corrected order passes the same adapter and owner interleavings. A genuine 151 ms old health sample still fails as `health.age_gt_150ms`.

## Verification

- Focused repository suite: **82 passed**. Command covered the new first-fault tests plus existing health I/O, safety guard, startup repair, zero boundary and unqualified motion tests. Output is in `experiments/g1_g5/evidence/watchdog-firstfault-20261023/raw/focused-tests.log`.
- The new 12 first-fault tests also pass against the isolated installed package.
- Fresh sequential `colcon build --packages-select course_navigation` passed to `experiments/g1_g5/evidence/watchdog-firstfault-20261023/artifacts/install/`.
- All 20 installed Python package files match their source SHA256. Resolved runtime `stage4_control.py` SHA256: `4c9e32b169dbe8d0ced5ea83cbcc8bed1ca99c96c139cac5c20c74d53e2525be`. Full identities are in `experiments/g1_g5/evidence/watchdog-firstfault-20261023/runtime-identity.json`.
- `git apply --check` passed against expected HEAD before application; source patch is retained at `experiments/g1_g5/evidence/watchdog-firstfault-20261023/raw/watchdog-proposal.patch`.
- All 29 inherited compact ZeroOnly checkpoint files still match the previously audited hashes. `PLAN.md`, its existing E1 plan, reports, and evidence were not edited by this watchdog change.

## Next steps

Independent review this patch and report. If accepted, separately authorize the next single bounded ZeroOnly qualification using the now-correct isolated runtime. The prior trial failed and probe cleanup was incomplete, so full ZeroOnly PASS still requires the complete stationary interval, runtime/graph evidence, contact end closure and probe cleanup. E1 stays behind its existing PLAN.md conditions; no new E1 design or motion authority is introduced here.

## Worktree state

Branch `experiment/unattended-g1-g5-20261019`, HEAD `4ae85f4148a89a76b1ad25c5b059c2140be41b85`. The inherited checkpoint remains uncommitted, with this watchdog fix separate and also uncommitted. No inherited report/evidence was overwritten. No live process was started; the isolated build and offline test subprocesses completed. No commit or push occurred.


## Focused validation

```text
........................................................................ [ 87%]
..........                                                               [100%]
82 passed in 2.58s
```

The source/runtime identity record verifies all 20 copied Python modules match their sources. Resolved runtime module: `/home/zyd/program/b2w-embodied-navigation-course-stage4/experiments/g1_g5/evidence/watchdog-firstfault-20261023/artifacts/install/course_navigation/lib/python3.10/site-packages/course_navigation/stage4_control.py` SHA256 `4c9e32b169dbe8d0ced5ea83cbcc8bed1ca99c96c139cac5c20c74d53e2525be`.

## Inherited checkpoint file hashes

- `PLAN.md` — `1b3c8d96b1f12a33ba26e66b7bbc5e5355516890704c82d7ab91cc9a7ca251a7`
- `course_navigation/course_navigation/velocity_adapter.py` — `dc97d0445ec790fc0b5c2d171070f38cb9ad5e5a7d91e01a5fb9cdf86f6136c8`
- `docs/STAGE4_RUNTIME_READINESS.md` — `2dbe31c4ab84c652f7428db895ba82725aeb9862d7537296037f3f8159de5aae`
- `tests/test_stage4_health_io.py` — `e49a3c1026fd0d9e34f11d1e260e1d4a5d9bb0db2a26abe272c52c6f4af5fab7`
- `tests/test_stage4_startup_repair.py` — `add1665e3426a4791e84af8b2be3f02f43c4be496fbbd7bcbde29d51dfed9452`
- `docs/reviews/B2W_Stage4_Health_IO_ZeroOnly_4ae85f4_20261022.md` — `7b9234e113804d8c5b008a69ac3ab155f3b89188b6cb3bf77966d7250c900f07`
- `docs/reviews/B2W_Warehouse_Navigation_Architecture_20261022.md` — `ed1565938afbbdccaa78209f8f2300208034a4f58f94983f2fb5bb617496f868`
- `docs/reviews/B2W_ZeroOnly_Policy_E1_20261022.md` — `3b044adaeb0f8565d6adc76d76d7a91d2cd05b81b8c88a57fc6468cd16480351`
- `experiments/g1_g5/evidence/health-io-zeroonly-20261022/REPORT.md` — `7b9234e113804d8c5b008a69ac3ab155f3b89188b6cb3bf77966d7250c900f07`
- `experiments/g1_g5/evidence/health-io-zeroonly-20261022/SHA256SUMS` — `4370cf564bb0cf30d2eed9f83e38cab6ac5ea2ac130cb3969440b2943865eded`
- `experiments/g1_g5/evidence/health-io-zeroonly-20261022/committed-files.txt` — `ff78cda14a360145f41b566ad8f07c2da2aeb37d5b866e4648f5a58c0210ab33`
- `experiments/g1_g5/evidence/health-io-zeroonly-20261022/index.json` — `dbb27823319ff0b1143060bcec263642309b812ceaf5e6839f914c660d342b4a`
- `experiments/g1_g5/evidence/health-io-zeroonly-20261022/summary.json` — `88adb09518188e254d0e8b5846aae76c6f84ba35063b3f6f48c031005cab4c46`
- `experiments/g1_g5/evidence/navigation-unblock-20261022/APRIL_REVIEW_BUNDLE_B2W_Warehouse_Architecture_4ae85f4_20261022.md` — `b8ed69da84728435689322aacc5c891e2a2c3cd7660ae46ae9a555c64be10453`
- `experiments/g1_g5/evidence/navigation-unblock-20261022/SHA256SUMS` — `de132cfe1ece6163b8377af48065adb7337899c8b70643c4df35140deec92773`
- `experiments/g1_g5/evidence/navigation-unblock-20261022/index.json` — `4b3b3fb95a69cb8cd7ddae3216615ce90771ec3bdeaddcde092d15d65a0992f4`
- `experiments/g1_g5/evidence/navigation-unblock-20261022/input-validation.txt` — `6739fce366508e5977a1509cbe7160c2cbc8faa490746eebc0434d7c564649e0`
- `experiments/g1_g5/evidence/navigation-unblock-20261022/offline-tests.txt` — `ff0ddafd9b9dd3000183d7ed7df3f46f0e17c2ed2ef114e9bb4c1ccad1034055`
- `experiments/g1_g5/evidence/navigation-unblock-20261022/preservation.json` — `049235209eb7008ad150d14e193244d5aa01ea249921e329643566c67ce3aa8a`
- `experiments/g1_g5/evidence/zero-policy-20261022/APRIL_REVIEW_BUNDLE_B2W_ZeroOnly_Policy_E1_4ae85f4_20261022.md` — `267a8a51b94e68baa4f4b51772ff41a1180390aad89d899bf56bf1990dc17f7f`
- `experiments/g1_g5/evidence/zero-policy-20261022/SHA256SUMS` — `dd3cbe903921ea7a15dfa78dc0f821dc30a1e4181512c14473b9971d18b59f7b`
- `experiments/g1_g5/evidence/zero-policy-20261022/build-result.json` — `c93e251aff2553c724e8d415e90677fc70847db21a04d4d136f8bf8a5f4001b5`
- `experiments/g1_g5/evidence/zero-policy-20261022/candidate-env.bash` — `1dfe3a05337c19b8ab72bc1d35d7942ebcbf1f393504ff5c29789ed2b2e7b4f2`
- `experiments/g1_g5/evidence/zero-policy-20261022/git-status.txt` — `625cf8c2c9190b158211a3a70faa46d0d413c58d7d8ef152416ad200f67f94a7`
- `experiments/g1_g5/evidence/zero-policy-20261022/green-tests.txt` — `2b931af2024f2d10d29335a5be3ead3c54f41217dc9a5acb485ea8919686aeb1`
- `experiments/g1_g5/evidence/zero-policy-20261022/index.json` — `822f150256c79112353cae204668f76d328fa60d9b840167b5c2239c447b06f7`
- `experiments/g1_g5/evidence/zero-policy-20261022/red-tests.txt` — `c2e66eb306ca343afc2a2be7bb2dfcdf6e7633bd9219994815ace1df68ec14c7`
- `experiments/g1_g5/evidence/zero-policy-20261022/review.txt` — `6b56d0dc5541d8e12436ae7a3782dfa43788a1aae0fb98331b979c53269349b1`
- `experiments/g1_g5/evidence/zero-policy-20261022/summary.json` — `5af02a88a894b27eb0ab25e814ac0a23e1740b631ba77db6ebc78c0252d1670b`

## Exact applied patch

```diff
--- a/course_navigation/course_navigation/stage4_control.py
+++ b/course_navigation/course_navigation/stage4_control.py
@@ -10,6 +10,7 @@
 from pathlib import Path
 import tempfile
 import stat
+import time
 
 
 def process_identity(pid):
@@ -85,36 +86,53 @@
 
 class Watchdog:
     def __init__(self,config):
-        self.config=dict(config);self.sequence=0;self.last_heartbeat=None;self.last_health=None;self.failure=None
+        self.config=dict(config);self.sequence=0;self.last_heartbeat=None;self.last_health=None;self.failure=None;self.first_fault=None
         distinct_control_paths([config[k] for k in ('heartbeat_path','lease_path','inhibit_path','adapter_heartbeat_path')])
 
-    def step(self,*,wall):
-        c=self.config
+    def step(self,*,wall=None):
+        c=self.config;health=None;h=None;failed_predicate='control.inhibit'
         try:
             if self.failure or os.path.lexists(c['inhibit_path']):raise ValueError(self.failure or 'inhibit_latched')
+            failed_predicate='process.identity'
             if not all(process_matches(c[k]) for k in ('owner','adapter')):raise ValueError('owned_process_identity_lost')
+            failed_predicate='health.read'
             health=read_control(c['adapter_heartbeat_path'])
-            if (health['session_id']!=c['session_id'] or health['adapter']!=c['adapter'] or
-                    type(health['sequence']) is not int or health['sequence']<=0 or
-                    not all(type(v) in (int,float) and math.isfinite(v) for v in (wall,health['wall'])) or
-                    not 0<=wall-health['wall']<=.15):raise ValueError('adapter_publication_stalled')
+            failed_predicate='owner.read'
+            h=read_control(c['heartbeat_path'])
+            # Validate immutable read snapshots against time sampled AFTER both reads.
+            # Sampling in the caller can make a concurrent publication appear future.
+            if wall is None:wall=time.monotonic()
+            failed_predicate='health.validation'
+            if health['session_id']!=c['session_id']:failed_predicate='health.session_id'
+            elif health['adapter']!=c['adapter']:failed_predicate='health.adapter'
+            elif type(health['sequence']) is not int or health['sequence']<=0:failed_predicate='health.sequence'
+            elif type(wall) not in (int,float) or not math.isfinite(wall):failed_predicate='sampled_wall_nonfinite'
+            elif type(health['wall']) not in (int,float) or not math.isfinite(health['wall']):failed_predicate='health.wall_nonfinite'
+            elif wall-health['wall']<0:failed_predicate='health.age_negative'
+            elif wall-health['wall']>.15:failed_predicate='health.age_gt_150ms'
+            else:failed_predicate=None
+            if failed_predicate:raise ValueError('adapter_publication_stalled')
+            failed_predicate='health.regression_or_conflict'
             if self.last_health and (health['sequence']<self.last_health['sequence'] or
                     health['wall']<self.last_health['wall'] or
                     health['sequence']==self.last_health['sequence'] and health!=self.last_health):
                 raise ValueError('adapter_publication_regression_or_conflict')
             self.last_health=health
-            h=read_control(c['heartbeat_path'])
+            failed_predicate='owner.validation'
             if (h['session_id']!=c['session_id'] or type(h['sequence']) is not int or h['sequence']<=0 or
                     not all(type(v) in (int,float) and math.isfinite(v) for v in (wall,h['wall'])) or
                     not 0<=wall-h['wall']<=.25):raise ValueError('owner_heartbeat_stale_or_invalid')
+            failed_predicate='owner.regression_or_conflict'
             if self.last_heartbeat and (h['sequence']<self.last_heartbeat['sequence'] or h['wall']<self.last_heartbeat['wall']):
                 raise ValueError('owner_heartbeat_regression')
             if self.last_heartbeat and h['sequence']==self.last_heartbeat['sequence'] and h!=self.last_heartbeat:
                 raise ValueError('conflicting_owner_heartbeat_duplicate')
             self.last_heartbeat=h;self.sequence+=1
             if c.get('zero_only') is True:
+                failed_predicate='zero_only.unexpected_lease'
                 if os.path.lexists(c['lease_path']):raise ValueError('unexpected_lease_in_zero_only_session')
                 return True  # Monitor health and inhibit faults; NEVER arm the adapter.
+            failed_predicate='lease.write'
             write_control(c['lease_path'],dict(session_id=c['session_id'],sequence=self.sequence,wall=wall,
                                               owner=c['owner'],adapter=c['adapter']))
             return True
@@ -122,4 +140,19 @@
             self.failure=self.failure or str(exc)
             latch_inhibit(c['inhibit_path'],self.failure)
             Path(c['lease_path']).unlink(missing_ok=True)
+            if self.first_fault is None:
+                if wall is None:wall=time.monotonic()
+                # One bounded in-memory snapshot only AFTER permission is revoked.
+                def safe(value):
+                    if type(value) is float and not math.isfinite(value):return repr(value)
+                    if isinstance(value,dict):return {k:safe(v) for k,v in value.items()}
+                    if isinstance(value,list):return [safe(v) for v in value]
+                    return value
+                age=None
+                if (isinstance(health,dict) and all(type(v) in (int,float) and math.isfinite(v)
+                        for v in (wall,health.get('wall')))):age=wall-health['wall']
+                self.first_fault=safe(dict(reason=self.failure,failed_predicate=failed_predicate,
+                    sampled_wall=wall,health_age_s=age,
+                    health={k:health.get(k) for k in ('session_id','adapter','sequence','wall')} if isinstance(health,dict) else None,
+                    owner={k:h.get(k) for k in ('session_id','sequence','wall')} if isinstance(h,dict) else None))
             return False
--- a/scripts/stage4_safety_watchdog.py
+++ b/scripts/stage4_safety_watchdog.py
@@ -1,6 +1,7 @@
 #!/usr/bin/python3
 """Action-independent local lease watchdog; publishes no ROS velocity or goals."""
 import argparse
+import json
 import time
 from course_navigation.stage4_control import Watchdog,read_control,latch_inhibit
 
@@ -13,7 +14,10 @@
     config=read_control(a.manifest);watchdog=Watchdog(config);started=time.monotonic()
     try:
         while time.monotonic()-started<a.wall_limit:
-            if not watchdog.step(wall=time.monotonic()):return 2
+            if not watchdog.step():
+                print(json.dumps(dict(event='stage4_watchdog_first_fault',**watchdog.first_fault),
+                    sort_keys=True,allow_nan=False),flush=True)
+                return 2
             time.sleep(.05)
         return 0
     finally:latch_inhibit(config['inhibit_path'],'watchdog_terminated')
--- /dev/null
+++ b/tests/test_watchdog_first_fault.py
@@ -0,0 +1,117 @@
+"""Real atomic control reads and bounded clock interleavings; no ROS or Gazebo."""
+import importlib.util
+import json
+import math
+import os
+from pathlib import Path
+from types import SimpleNamespace
+import pytest
+from course_navigation import stage4_control as c
+
+
+def setup_records(tmp_path):
+    identity=c.process_identity(os.getpid())
+    cfg=dict(session_id='zero',owner=identity,adapter=identity,zero_only=True,
+        **{k:str(tmp_path/k) for k in ('heartbeat_path','lease_path','inhibit_path','adapter_heartbeat_path')})
+    health=dict(session_id='zero',sequence=1,wall=20.,adapter=identity)
+    c.write_control(cfg['heartbeat_path'],dict(session_id='zero',sequence=1,wall=20.))
+    c.write_adapter_health(cfg['adapter_heartbeat_path'],health)
+    return cfg,health
+
+
+def script_module():
+    path=Path(__file__).resolve().parents[1]/'scripts/stage4_safety_watchdog.py'
+    spec=importlib.util.spec_from_file_location('watchdog_cli_under_test',path)
+    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
+    return module
+
+
+@pytest.mark.parametrize('updated_record',['adapter','owner'])
+def test_cli_reads_concurrent_publication_before_sampling_clock(tmp_path,monkeypatch,updated_record):
+    cfg,health=setup_records(tmp_path);manifest=tmp_path/'manifest';c.write_control(manifest,cfg)
+    clock=SimpleNamespace(now=20.);real_read=c.read_control;changed=[]
+    def read(path):
+        target=cfg['adapter_heartbeat_path'] if updated_record=='adapter' else cfg['heartbeat_path']
+        if str(path)==target and not changed:
+            changed.append(True);clock.now=20.01
+            value=dict(health,sequence=2,wall=clock.now)
+            if updated_record=='owner':value.pop('adapter')
+            c.write_control(path,value)
+        return real_read(path)
+    monkeypatch.setattr(c,'read_control',read)
+    # Same clock is observed by caller and control module; the race is deterministic.
+    import time
+    monkeypatch.setattr(time,'monotonic',lambda:clock.now)
+    monkeypatch.setattr(time,'sleep',lambda _:setattr(clock,'now',20.2))
+    module=script_module()
+    monkeypatch.setattr('sys.argv',['watchdog','--manifest',str(manifest),'--wall-limit','.1'])
+    assert module.main()==0  # Old caller samples20.0, then rejects the new20.01 record.
+    assert changed and not Path(cfg['lease_path']).exists()
+    assert c.read_control(cfg['inhibit_path'])['reason']=='watchdog_terminated'
+
+
+@pytest.mark.parametrize(('mutation','wall','predicate'),[
+    ('session',20.01,'health.session_id'),
+    ('adapter',20.01,'health.adapter'),
+    ('sequence',20.01,'health.sequence'),
+    ('nonfinite',20.01,'health.wall_nonfinite'),
+    ('future',20.01,'health.age_negative'),
+    ('stale',20.151,'health.age_gt_150ms'),
+    ('missing',20.01,'health.read'),
+])
+def test_first_fault_records_exact_rejected_health_and_latches(tmp_path,mutation,wall,predicate):
+    cfg,health=setup_records(tmp_path)
+    if mutation=='session':health['session_id']='wrong'
+    if mutation=='adapter':health['adapter']=dict(cfg['adapter'],start_ticks='wrong')
+    if mutation=='sequence':health['sequence']=0
+    if mutation=='nonfinite':health['wall']=float('nan')
+    if mutation=='future':health['wall']=20.02
+    if mutation=='missing':Path(cfg['adapter_heartbeat_path']).unlink()
+    else:Path(cfg['adapter_heartbeat_path']).write_text(json.dumps(health))
+    watchdog=c.Watchdog(cfg)
+    assert not watchdog.step(wall=wall)
+    fault=watchdog.first_fault
+    assert fault['failed_predicate']==predicate
+    assert fault['sampled_wall']==wall
+    if mutation=='future':assert fault['health_age_s']==pytest.approx(-.01)
+    if mutation=='stale':assert fault['health_age_s']==pytest.approx(.151)
+    if mutation!='missing':assert fault['health']['sequence']==health['sequence']
+    # Error cannot be healed or its first snapshot overwritten by another record.
+    c.write_adapter_health(cfg['adapter_heartbeat_path'],dict(health,session_id='zero',sequence=2,wall=wall,adapter=cfg['adapter']))
+    assert not watchdog.step(wall=wall)
+    assert watchdog.first_fault is fault
+    assert Path(cfg['inhibit_path']).exists() and not Path(cfg['lease_path']).exists()
+
+
+def test_post_read_sample_does_not_renew_stale_health(tmp_path,monkeypatch):
+    cfg,health=setup_records(tmp_path)
+    import time
+    monkeypatch.setattr(time,'monotonic',lambda:20.151)
+    watchdog=c.Watchdog(cfg)
+    assert not watchdog.step()
+    assert watchdog.first_fault['failed_predicate']=='health.age_gt_150ms'
+    assert c.read_control(cfg['adapter_heartbeat_path'])['wall']==20.
+
+
+def test_duplicates_expire_and_conflicting_duplicate_is_rejected(tmp_path):
+    cfg,health=setup_records(tmp_path);watchdog=c.Watchdog(cfg)
+    assert watchdog.step(wall=20.) and watchdog.step(wall=20.1)
+    health['wall']=20.01;c.write_adapter_health(cfg['adapter_heartbeat_path'],health)
+    assert not watchdog.step(wall=20.1)
+    assert watchdog.failure=='adapter_publication_regression_or_conflict'
+    assert not Path(cfg['lease_path']).exists()
+
+
+def test_cli_logs_one_snapshot_only_after_inhibit(tmp_path,monkeypatch,capsys):
+    cfg,health=setup_records(tmp_path);manifest=tmp_path/'manifest';c.write_control(manifest,cfg)
+    import time
+    monkeypatch.setattr(time,'monotonic',lambda:20.2)
+    module=script_module()
+    monkeypatch.setattr('sys.argv',['watchdog','--manifest',str(manifest),'--wall-limit','.1'])
+    assert module.main()==2
+    lines=capsys.readouterr().out.splitlines()
+    assert len(lines)==1
+    logged=json.loads(lines[0]);assert logged['event']=='stage4_watchdog_first_fault'
+    assert logged['failed_predicate']=='health.age_gt_150ms'
+    assert c.read_control(cfg['inhibit_path'])['reason']=='adapter_publication_stalled'
+    assert not Path(cfg['lease_path']).exists()
```

## Worktree status at review handoff

```text
## experiment/unattended-g1-g5-20261019...origin/experiment/unattended-g1-g5-20261019
 M PLAN.md
 M course_navigation/course_navigation/stage4_control.py
 M course_navigation/course_navigation/velocity_adapter.py
 M docs/STAGE4_RUNTIME_READINESS.md
 M scripts/stage4_safety_watchdog.py
 M tests/test_stage4_health_io.py
 M tests/test_stage4_startup_repair.py
?? docs/reviews/B2W_Stage4_Health_IO_ZeroOnly_4ae85f4_20261022.md
?? docs/reviews/B2W_Warehouse_Navigation_Architecture_20261022.md
?? docs/reviews/B2W_Watchdog_Clock_Race_Fix_20261023.md
?? docs/reviews/B2W_ZeroOnly_Policy_E1_20261022.md
?? experiments/g1_g5/evidence/health-io-zeroonly-20261022/
?? experiments/g1_g5/evidence/navigation-unblock-20261022/
?? experiments/g1_g5/evidence/watchdog-firstfault-20261023/
?? experiments/g1_g5/evidence/zero-policy-20261022/
?? tests/test_watchdog_first_fault.py
```

No current commit or remote update was made. All next actions stop for independent review.
