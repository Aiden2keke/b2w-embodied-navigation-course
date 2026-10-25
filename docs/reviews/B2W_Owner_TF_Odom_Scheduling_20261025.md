# Owner TF/odom scheduling fix — offline review PASS

Base: `523dd0f65fa8807b3ab3e4220907f04726b3a2b5`, branch
`experiment/canonical-planning-20261025`, prepared checkout
`/tmp/b2w-navigation-ab-20261023`. Single agent; no Gazebo or motion.

## Evidence and reproduction

The recorded decision was clock24.5025, Owner common TF24.0, age0.5025>0.5.
Odom24.1 passed. Independent observer received coherent24.2 TF at wall281231.700041;
Owner last paired callback was281231.609214 and last spin ended281231.718863.
Maximum between-spin interval was0.116954s. The145-pose canonical path remains
valid historical planning evidence; its terminal action status remains unavailable.

`tests/fixtures/owner_tf_scheduling_20261025.json` carries these values and the
SHA256 of their retained source summary. Replay with the real tf2 Buffer rejects
24.0 and admits24.2 at the same clock, with0.3025s age. This reproduces the
failure under a modeled pending-callback ordering. Owner DDS arrival timestamps
were NOT recorded: observer receipt cannot establish Owner middleware arrival or
prove the sole historical cause. No fabricated transport trace is claimed.

Installed Humble `SingleThreadedExecutor._spin_once_impl` takes one handler per
spin_once; callback groups alone cannot run that node while the calling owner
performs synchronous work. Installed TransformListener already uses a reentrant
group, but its default spin_thread is false. Thus a group-only edit does not solve
this service gap. A MultiThreadedExecutor would also require continuous spinning
and synchronization of unrelated Monitor state. Giving the primary node to a
second executor is invalid ownership. The selected narrow correction gives only
localization its own node and one continuously serviced SingleThreadedExecutor.
Exact installed executor/listener source hashes are in local verification.json.

## Implementation and safeguards

- Move `/tf`, `/tf_static`, `/slam/odom` subscriptions to dedicated localization
  node. Preserve listener QoS, groups, authority and insertion semantics; retain
  odom sensor QoS. No new publisher, goal, command source, stamp rewrite or retry.
- One RLock covers whole TF messages (including static insertion), odom mailbox,
  pose/readiness decisions and failed-decision TF diagnostics. The worker never
  mutates Monitor histories, contact state, command state or disk evidence.
- Bounded128-entry odom mailbox carries ORIGINAL callback wall receipts to the
  owner thread. Equal stamps cannot renew receipt freshness; odom regression is
  latched before mailbox eviction. Worker exceptions/death reject admission.
- Bounded128-entry composed TF history handles ingress leading the owner clock.
  Select an existing source-stamped chain at/before clock. Do not interpolate at
  now to hide old source data. A live TF lookup exception still rejects cached
  history. No claim of exact TF/odom source stamp equality is added; both are
  evaluated within the same locked decision and existing independent predicates.
- Keep TF simulation freshness <=0.5s and odom simulation/wall freshness <=0.5s.
  Preserve contact/physical veto, sole command owner, planning-only zero authority,
  measured rest, independent watchdog and owned cleanup. Explicit bounded thread
  join/node destruction runs during final cleanup; cleanup failure forbids PASS.
- Main clock/contact/sensor/command callbacks, planner startup, Smac configuration,
  Stage4 and policy are unchanged. No new monitoring framework or thresholds.

## Verification and actual-diff review

Local evidence: `artifacts/recovery/owner-tf-scheduling-20261025/` (ignored logs).
Initial RED:1 passed,3 failed (missing independent service/lock and future ordering).
A further adversarial RED caught interpolation hiding stale source stamps; removed
that approach and added the retained-chain regression before review PASS.
Final complete focused selection: **379 passed in23.19s**, including the previously
excluded real ROS runner/recorder executor test. Current-version ingestion subset:
**10 passed in0.36s**. No deselections. Localhost DDS required sandbox escalation;
initial socket denial/domain233 selection was corrected to test domain182.
An intermediate11 failures were incomplete in-memory Monitor test doubles at
cleanup; updated those doubles and tested cleanup failure explicitly. Production
cleanup remains mandatory.

The complete selection is the18 test modules listed in the prior milestone's
reproduction command plus `tests/test_owner_ingestion.py`, with no `-k` exclusion.
It covers scheduling, concurrent partial-TF reads through actual Monitor.pose,
future/stale/duplicate/regressed inputs, worker failure, mailbox bounds, cleanup,
planning-only isolation, contact/command/stop/watchdog/route admission regressions.
Offline Profile-B planning check passed46 source/install pairs and26 collision
handles; no world started. All62 original raw trial files match original SHA256s.
`git diff --check` passes. No ROS packages/native code changed, so no rebuild.

Single-agent actual-diff self-review: **PASS for offline TF/odom correction**;
not an independent review or runtime/navigation readiness PASS. Reviewed executor
ownership, lock ordering, mailbox receipts/eviction, no-worker Monitor mutations,
whole-message TF reads, future-chain selection, fault snapshot consistency,
shutdown timeout/failure, and unchanged motion/containment call paths. No unresolved
critical/important finding. Existing host/GIL/DDS delays can still cause a genuine
freshness fault; threads provide no hard realtime guarantee. No live latency or
successful Start→G2 result is claimed.

## Prepared next experiment — NOT RUN, separate authorization required

First separately authorize qualification of the changed Owner in zero-only mode,
including advancing TF/odom, original predicates, measured rest and watchdog
containment/cleanup. Do not spend another planner capture merely to prove a path
exists. Reverify source/install/native identities and this commit's two script
hashes; historical Owner749a4169 is superseded, not silently reused.

Then the shortest already-regressed useful precheck sequence is Profile B,
**1s forward vx0.05m/s → zero/measured rest →1s turn wz0.05rad/s →zero/measured
rest**. All other axes zero. Initial rest and each stop retain >=5 advancing sim
seconds and >=3 wall seconds; per-pulse wall ceiling10s, rest ceiling30s, existing
whole-session watchdog420s. No goal, route/controller launch, mixed pulse or repeat.
Retain1.5m observed-free pocket, <=0.25m translation/0.35rad turn excursion and
0.25m/s/0.4rad/s physical vetoes. Never substitute truth for control localization.
Use a verified original-world pocket; no invented spawn or clearance certificate.

Prepared invocation shape, ONLY after separate authorization and fresh unused
DOMAIN/output/lease plus accepted zero-only qualification:

```sh
ROS_DOMAIN_ID="$DOMAIN" scripts/run_warehouse_ab.sh --profile B --precheck-only \
  --precheck-pulse-s 1 --deployment-manifest "$REVIEWED_DEPLOYMENT" \
  --output-root "$FRESH_OUTPUT" --execute --authorize-motion precheck-only
```

Collect TF/odom readiness, actual commands, sensor pose, evaluation truth, contact
pairs, whole-coast stopping displacement/yaw and rest/cleanup evidence. Any original
fault stops the attempt with no retry. This micro-check supplies initial motion
response/stop data only: it cannot qualify Profile-B0.2m/s stopping/tracking,
mixed-motion turn drift or G1 body/rotation coverage. Next session handles those
bounds and actual Start→G2 separately;0.3m navigation vs0.05m staging stays explicit.
