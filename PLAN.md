# 狭窄空间导航：唯一 CURRENT roadmap

## 动态模型路由建议（2026-10-13）

| 阶段 | 建议模型 / reasoning effort | 本任务的选择依据 |
| --- | --- | --- |
| Plan | `gpt-6.1-sol` / `high` | 需要区分配置事实、官方语义、历史物理证据和未证实因果，优先减少误诊返工。 |
| Execute | `gpt-6.1-sol` / `medium` | harness 的 action 生命周期、时间积分、进程归属清理比机械配置编辑复杂；保持上下文可减少重新解释接口的成本。 |
| Review | `gpt-6.1-sol` / `high` | 用独立上下文审查 bundle，重点反驳根因判断、指标有效性和清理证明；遇到无法裁决的跨层问题才重新比较 Astra。 |

可用性依据：本会话工具声明，以及 `/home/zyd/.codex/models_cache.json` 的 `fetched_at=2026-10-13T02:14:23.985474769Z`、`visibility=list`、`supported_reasoning_levels`。可见模型包括 GPT-6.1 Sol、GPT-6 Astra、GPT-6 Sol、GPT-6 Luna 和 GPT-5.6 系列；这不是重新验证过的账户调用授权，也不表示本轮已切换模型。

已打开的官方模型页显示，Standard 短上下文 API 每百万 input / cached input / output token：Sol 为 $2 / $0.10 / $10，Luna 为 $0.10 / $0.01 / $0.50。实际 Codex 工作区计费、套餐折算及本任务各模型成功率：**unavailable**；不宣称已求得数值最优成本。来源：[GPT-6.1 Sol](https://developers.openai.com/api/docs/models/gpt-6.1-sol)、[GPT-6 Luna](https://developers.openai.com/api/docs/models/gpt-6-luna)。

路由是任务风险判断，不是固定“强规划、弱执行”。若评审后只剩字段整理或确定性文档同步，可考虑 Luna / medium；若仍涉及 ROS action、故障归因和残留进程处理，则保留 Sol。建议 **Review 新开会话**，以本文件、diff 和 bundle 为输入；本轮不为切模型重开执行会话。

## 范围、当前状态和评审关卡

- 分支：`improve/narrow-navigation`；历史审计基准 HEAD：`66db5152b3b617d0e5ac5ed65b031573255a4169`；当前HEAD见NOW；本规划轮开始时tracked工作区干净，仅继承untracked `video/`；旧harness/文档已在当前HEAD中。
- 目标：在 B2-W + LIO-SAM + Nav2 baseline 上先复现真实窄空间失败，再做可回退、单变量或少变量改动。
- 不改 locomotion policy，不跑长实验，不 push，不 merge main。本轮复用baseline与上一轮已有的独立单plugin诊断配置；不改公共 topic/TF、消息类型或模块边界。
- 已读 `AGENTS.md`、`CODEX_GUIDE.md`、`docs/INTERFACE.md`、`docs/INTERFACE_ALIGNMENT.md`、`docs/COURSE_VALIDATION.md`、`docs/RL_SAR_SETUP.md`、`docs/LOCOMOTION_STATUS.md`。参数 SSoT 为当前 YAML/C++/Python 实现；历史 JSON 仅证明当时的有限验收范围。
- 已应用 using-superpowers、systematic-debugging、brainstorming、writing-plans、verification-before-completion，以及 OpenAI Docs 的模型选择流程。本轮设计属于已有实验流程的有界扩展；用户要求的唯一根计划和评审关卡优先于 Skill 的额外计划/提交流程。不派生子 agent。
- 旧实施计划归档至 `docs/plans/archive/2026-10-10-local-course-system-plan.md`。历史 design/validation 保留为证据，不是另一个当前 roadmap。无新增 HANDOFF。
- 历史轮次只读与 timing 交付已结束；本轮范围以最新用户请求和下方 NOW 为准。旧实验/交付记录只作历史证据。

## NOW — Pocket timing correction and one negative yaw (2026-10-26)

- [x] Single agent, exact base b2b6260c73d61300a4f65d4e836da143fb095a8a.
- [x] Root cause: early full-disk rejection preceded mandatory initial measured
      zero rest; bootstrap had only established some observed-free coverage.
- [x] Preserve early diagnostic; perform unchanged qualified rest first, require
      new actual map publication, then unchanged full 1.5m disk. Independent
      pre-publication callback dispatch/clearance/freshness guards retained.
      No added waiting, map relabeling, radius/control/policy changes.
- [x] RED→GREEN regression and saved 20261025 FREE / 20261026 UNKNOWN replays.
      165 focused offline PASS; persistent unknown/occupied and absent/stale map
      fail closed. 46 source/install identities and 26 native handles PASS.
- [x] Live BLOCKED before launch: host GPU preflight found unrelated compute
      PID3640423 (raylab Python). Domain213 lease available and closed; no
      competing sim. DDS domain discovery not reached. STOP; no wait/retry.
      Zero simulator attempts/pulses/goals; no new map, yaw, rest or cleanup.
      Unknown cell later clearing remains UNKNOWN; no delivered-duration PASS.
- [x] HANDOFF: `docs/POCKET_TIMING_YAW_20261026.md` and evidence directory
      `artifacts/recovery/pocket-timing-yaw-20261026/`. Positive yaw-.154184rad,
      rebound+.151055rad retained; no matched negative comparison. Yaw remains
      BLOCKED. Review/commit/push and ONE APRIL+SHA256 delivery, then STOP;
      exact commit/remote/checksum identities reside in sealed delivery evidence.
- [ ] NEW SESSION exact next action: after host GPU availability and renewed
      explicit authority, ONE corrected-source -.05 yaw nominal.25sim s at
      existing(0,-1), fresh domain/lease/output; capture post-rest map/cell,
      <=.30s target actual interval and signed response/whole stop. No repeated
      stationary/planning qualification. Before Start→G2 also resolve reviewed
      Profile-B directional stopping/tracking envelope and G1 rotation coverage.
      Start→G2 and Start→G5 video remain NOT DONE; no production sign/gain change.

## Historical — Negative yaw attempt rejected before motion (2026-10-26)

- [x] Single agent; expected base `8df7df454f03af8dc6a568732b37966907e93d36`,
      branch `navigation/g1-g5-recovery-20261025`; clean checkout verified.
- [x] Minimal `--precheck-axis negative-yaw` selects one pure -.05 pulse in
      existing precheck owner/loop; default forward+turn unchanged. Existing
      observer records targets/torques/named joints. 175 focused offline PASS,
      new tests RED→GREEN;46 source/install/native pins and26 handles PASS.
- [x] Exactly ONE authorized simulator attempt, domain212, spawn(0,-1), nominal
      .25sim s. REJECTED at initial1.5m observed-free pocket: one unknown cell,
      zero occupied cells; TF/odom/contact/readiness passed at exception.
      No pulse, retry, navigation goal, tuning or hardware. Actual delivered
      nonzero duration0sim s; no negative response, rebound or matched rest.
      No separate stationary session required/used; prior qualification retained.
- [x] 483 observed final Twists and837 model triplets all zero. Last observed
      five seconds truth drift.309mm, yaw2.113e-7rad; not owner-qualified rest.
      Contact failure null,362 samples. Watchdog paused36 threads; owned fallback
      cleaned19 identities. Independent host/token/server/lease audit PASS.
      Normal cleanup NOT exercised; prior stationary628-zero/19-process PASS retained.
- [x] Old positive yaw-.154184rad and+.151055rad rebound preserved. New offline
      fixed-state yaw interventions again show negative local wheel differential
      sensitivity and zero-state offset. No physical negative comparison exists;
      learned response/state bias/plant-contact/idle rebound remain unresolved.
      No production sign/gain/speed/idle/policy correction conclusively justified.
- [ ] NEXT SESSION: diagnose saved observed-free pocket/startup coverage without
      weakening predicate; separately authorize matched-rest pure negative yaw.
      Start→G2 remains BLOCKED by controllable yaw/stopping and route envelope;
      145-pose Start→G1 path retained, actual Start→G2 and continuous G1→G5 video
      NOT DONE. Navigation.3m and staging.05m remain distinct.
- [x] Detailed result: `docs/NEGATIVE_YAW_DIAGNOSTIC_20261026.md`; evidence
      `artifacts/recovery/negative-yaw-20261026/`. Single-agent scoped review PASS.
      Commit/push coherent changes after review; deliver ONE APRIL+SHA256 and
      STOP. Actual commit/push/delivery identities are in sealed delivery evidence.


## Historical — Final corrected canonical planning capture (2026-10-25)
- [x] Verified exact Owner749a4169..., watchdog/deployment46 source/install pins and26 collision handles; domain198/lease/output fresh. Waited for unrelated GPU compute job to exit naturally; clean immediate prelaunch baseline. Exactly ONE corrected-source planning-only session used; no retry, pulses, DWB or navigation.
- [x] REAL PATH OBTAINED: YES. Active SmacPlanner2D published145-pose7.212947m Start→G1 path atsim24.5. Owner FAIL atTF age.5025>.5 while independent paired TF24.2 was fresh. Terminal ComputePathToPose action status/UUID not retained; published path preserved without fabricating action success. Initial90° turn plus first.5m geometrically clear; qualified stopping/tracking bounds absent. G1 footprint intersects184 unknown /map cells; bracketing global master has0 unknown/lethal but4 inscribed cells. No motion admission.
- [x] No iterative startup patches: corrected OwnerSHA remains749a41692b21ff843e77a96003404f54a2fd8ac59b3132a6e1e641e701432fed.642 Owner/613 independent final commands and1125 policy command triplets all zero;476 contact samples only permitted foot-ground pairs. Watchdog contained36 threads and cleaned23 identities; postflight domain/lease/GPU restored. Original raw-file hashes verified.
- [x] Delivered final-canonical-planning Markdown APRIL bundle + sidecar to /home/zyd/APRIL_REVIEW_BUNDLES; destination SHA256 verified cee66344302175de133cd1f15ad9e90e4be2515bfc647e162e06606e94e991a7. This delivery bookkeeping follows the sealed evidence snapshot.
- [ ] Next authorized work: focused TF/odom ingestion/executor scheduling diagnosis using captured trace, retaining.5s freshness; existing frozen Smac replay proposal prepared but NOT executed. Then separately authorize bounded Profile-B turn/forward/mixed-motion stopping and tracking calibration in verified clearance, plus resolve G1 body/rotation map coverage and explicitly decide.05 evaluation policy before Start→G2. No further live session is authorized by this task.

## Historical — Prior canonical planning capture (2026-10-25)

- [x] Rechecked domain197, free lease, unchanged six GPU identities, empty ROS/process domain, reviewed source/install/native/watchdog pins and unused output. User explicitly authorizes exactly ONE planning-only Gazebo session; no nonzero command, NavigateToPose, DWB route or retry.
- [x] Executed ONE Profile-B planning-only Gazebo session, planning-5244992425834c4c8ae86b895a549cbe. FAIL before planner action at owner clock23.6775 / TF+odom23.0 age.6775>.5. No action/path/live parameters;640 owner/615 observer final commands and1083 policy command triplets all zero. Watchdog contained36 threads and cleaned23 identities; independent postflight domain/lease/GPU clean. No retry.
- [x] Classified pre-action owner backlog; independent observer had paired23.3 TF before fault. Actual Start map/TF captured; operational padded current clearance.777423m; G1 center free but arrival footprint has unknown cells. Initial path sweep/calibration directions remain unavailable. Historical.0648m navigation success would fail.05 staging; no criteria changed. Offline-only correction moves planner cold startup into existing zero bootstrap, reuses lifecycle activation checks, and defers bulk geometry/map evidence until owned teardown.180 focused tests and corrected offline pins PASS; no live rerun.
- [x] Delivered canonical-planning Markdown APRIL bundle + sidecar to /home/zyd/APRIL_REVIEW_BUNDLES; destination SHA256 verifiedf215b1ddc1323c926aa216980cabdaa512ba99176419f173001116be5e0528ae. This bookkeeping follows the sealed snapshot. No additional live trial.
- [ ] Exactly one authorized session is consumed. A corrected planning-only capture requires NEW explicit authorization and fresh output marker/domain recheck before any calibration or Start→G2 trial.

## Historical delivery — Practical route readiness, no Gazebo or motion (2026-10-25)

Single agent. Preserve all inherited work; no simulation, commands, unrelated builds, new safety framework or threshold relaxation.

- [x] Add explicit planning-only CLI authority and branch: retain zero adapter/contact/TF/watchdog/cleanup; planner only, no navigator/controller/goal manager; capture real plan and checked state on separately approved startup.
- [x] Reconstruct actual recorded joint configurations from authenticated observation mapping; project URDF collision primitives and body attitude. Assess operational candidate against existing Nav2 footprint and recorded canonical map; distinguish sampled support from qualification.
- [x] Add direction-aware continuous rectangle/rotation/stopping checks and reviewed-envelope input for route mode, failing closed on missing evidence without a permanent hardcoded None.
- [x] Keep .3m navigation versus .05m staging evaluation explicit; prepare validated-path G3–G5 heading extraction and bounded continuous desktop/Gazebo capture before first goal.
- [x] 245 focused tests PASS (14.99s), inline single-agent review, both planning-only offline profiles PASS46 source/install identities and26 collision handles. Delivered one full-repository-name route-ready Markdown APRIL bundle and matching sidecar to /home/zyd/APRIL_REVIEW_BUNDLES; destination checksum verified7999afd7701f7a2f6389d9ce9b0994730a95ac6fa940104f3893fe860c14667e. This bookkeeping edit follows the sealed bundle snapshot.
- [ ] Separately authorize ONE canonical planning-only capture using --planning-only --execute --authorize-planning; no route motion authority. Real canonical path, reviewed B stopping/tracking/localization envelope and G3–G5 paths remain unavailable. No Gazebo/motion/video execution in this continuation. Conditional footprint bound±.6544/±.3236m supports unchanged Nav2±.70/±.38+.05padding within recorded joint/tilt ranges; canonical padded current rectangle has .7774m occupied-cell gap. No full-start/physical success claimed.

## Historical delivery — Route-specific Start admission, offline only (2026-10-25)


User scope: single agent; no Gazebo, velocity publication or hardware. Preserve inherited changes.
Design: separate route startup from qualification pulses; request a planning-only Smac path before any goal, check current and swept footprint against known-free cells, fail closed on absent physical reaction/stopping bounds. Never promote empirical coast or YAML acceleration into a physical guarantee.

- [x] Trace historical baseline and capture canonical recorded map/geometry identities.
- [x] Derive conservative articulated collision containment from actual generated URDF; compare Nav2 footprint, padding, pose and grid error. Radius1.308972869m; recorded current footprint+.05padding PASS, full admission FAIL (no validated path).
- [x] Write failing tests for route admission, current/path/stop/rotation/unknown faults and actual execute rejection/cleanup; retain precheck-only disk and pulse behavior.
- [x] Implement small offline geometry predicate and planning-only route startup integration. Missing validated plan or bounds block motion explicitly; runtime bound provider remains absent, so executable route restoration is INCOMPLETE.
- [x] 222 focused tests PASS; inline single-agent review; both offline profiles PASS46 source/install pairs and26 shapes. G3–G5 validated-path heading procedure documented; actual headings remain unresolved without validated segment paths.
- [x] Delivered one APRIL archive and checksum to /home/zyd/APRIL_REVIEW_BUNDLES; destination SHA256 verified: 7b6e13c5a79ef25814574d775e94ba6cb254304f03bde1e13b38644595fb522e. Snapshot archive precedes this delivery bookkeeping edit. Local REPORT.md records incomplete motion qualification/path/headings; no Gazebo or velocity publication authorized/performed. Next separately authorized experiment is ONE canonical planning-only startup; no goal/pulse, expected missing-bound rejection and owned cleanup.

## Historical milestone — First-motion control risks corrected offline (2026-10-25)

- [x] Reviewed domain232 trajectories and commands without rerunning. Forward command vx=0.05 for 2.000 sim s produced 0.206482 m and accepted forward rest. Full precheck remains FAIL at turn common-TF age 0.505 s. Stationary PASS stays closed.
- [x] Final adapter vx=0.05 and actual rl_sar obs[6]=0.050000000745 agree. No applied 2x command scaling: policy commands_scale=1; lin_vel_scale=2 is absent from the 57-element observation vector. Physical pose derivatives agree with body twist. Tracking excess is in policy/plant response; no fixed-gain calibration is justified.
- [x] Peak displacement from the exact initial motion reference was 0.249047611 m at sim28.32, leaving only 0.000952389 m to the original 0.25 m boundary. Full-rotation body mean/peak vx=0.103179/0.141681 m/s. Max after-zero travel=0.042596 m; settling=0.955 sim s; accepted rest=6.0625 sim s.
- [x] Reproduced the half-updated TF chain: Owner inserted odom→base33.9 while map→odom remained33.8; observer already had both. Same-estimate edges now publish together, retaining exact transforms/stamps. Recorded real-Buffer replay age changes from0.505 to0.405 s. Exact DDS ingress versus queue waiting remains unobservable; missing whole pairs still fail the unchanged0.5 s gate.
- [x] Pulse entry now requires Owner clock advancement after evidence I/O while applying all active readiness/physical vetoes within the existing10-wall-s bound. Baseline actual execute() reproduces admission after only an internal callback; corrected execute() waits. Original0.25 m excursion now also covers all precheck motion and stops from one initial reference. Route geometry/goals are unchanged.
- [x] Fault-stop handoff: latch independent fault, send zero through the existing input publisher, output NONZERO→ZERO immediately from the sole adapter. Repeated zeros do not flood callbacks. Owner waits at most3 wall s for watchdog before cleanup; watchdog pauses physics first, then allows0.1 wall s for queued zero/observer delivery before owned termination. No drain after failed pause, no resume, no new publisher. Final delivery and physical rest still require live evidence.
- [x] Built only course_slam/course_navigation; installed imports match source. Reviewed deployment manifest retains all46 source/install identities and unchanged native pins;26 engine handles pass offline checks. Historical manifest is preserved. 291 regression cases passed with one real-ROS executor case deselected;18 focused risk cases passed (14 overlap,295 distinct). The initially attempted real-ROS case was blocked by sandbox log permissions, not a control assertion.
- [ ] No Gazebo or nonzero physical execution in this task. Proposed ONE profile-A/precheck-only session, candidate domain197, spawn(0,-1): vx=0.05≤1 sim s→zero/rest→wz=0.05≤1 sim s→zero/rest, total≤420 wall s. New explicit authorization required. Domain/lease/GPU/ownership must be rechecked immediately before launch. Empirical forward+coast estimate0.184276 m is not a worst-case guarantee; all original guards remain.
- [ ] Canonical route bootstrap remains blocked: (0,0)1.5 m disk contains occupied and unknown cells; nearby ClutteringC_01_031 geometry is1.448249 m away. Alternate precheck spawn cannot waive this gate. Circle applies at initial/pulse admission, not continuously in Nav2 corridors. No G1→G2 A/B or full-route execution; G3–G5 yaw remains unspecified.
- [x] Single agent, no delegation, no global changes/commit/push. Preserve inherited dirty worktrees and historical evidence. One first-motion-control-risks APRIL bundle plus SHA256 sidecar delivered to /home/zyd/APRIL_REVIEW_BUNDLES/; raw analysis remains in artifacts/recovery/first-motion-control-risks-20261025/.

## Closed stationary milestone — verified evidence retained (2026-10-25)

- [x] ONE authorized profile-A stationary-only Fortress attempt, fresh domain225,
  output `artifacts/qualifications/stationary-ledger-confirmation-20261025/stationary-1e05b7c38b24405a93c5fabb176c8b19`.
  Harness exit0, success=true, stationary_containment=PASS; stationary milestone CLOSED.
- [x] Physical rest sim18.065→23.8575 =5.7925sim s/5.927002wall s;
  displacement0.000342737m, yaw0.000000282212rad.626/626 final Twists six-axis exact zero.
  Truth/TF/sensor/contact acceptance held;466 contact samples, no first fault.
- [x] Intentional watchdog injection at33.8726wall s from parent start; all36 server
  threads T before owned termination. Server PID2220722/start_ticks21410213 agrees in
  server pin, actual loaded native deployment, watchdog ledger and Owner cleanup ledger.
  All19 owned identities cleaned; both remaining sets empty. Original150s limit unchanged;
  enclosing pre-to-post host evidence interval45.8462wall s.
- [x] Independent post-check: domain225 process/ROS graph empty, lease free, pinned server
  absent; same six GPU PIDs and zero compute PIDs before/after. Final /proc scan has no
  owner/domain225 survivors or unreadable same-UID entries.
- [x] Reviewed Supervisor bookkeeping/precheck-only source hashes unchanged;123 focused
  tests PASS. Stationary/precheck offline inputs PASS46 source/install pins, native pins,
  26 collision handles. Post-PASS precheck-only offline --check passed; no live precheck,
  pulse, navigation launch/goal, nonzero command, rebuild, threshold change or retry.
- Next: NEW explicit authorization for ONE existing bounded forward0.05m/s2sim s→rest→
  turn0.05rad/s2sim s→rest precheck-only attempt. Candidate domain226 must be rechecked
  free/exclusive immediately before launch, with lease/process/ROS/GPU baseline, reviewed
  pins, independent watchdog readiness and unused output/precheck.used identity.
  Proposed command, NOT executed:
  `ROS_DOMAIN_ID=226 scripts/run_warehouse_ab.sh --profile A --precheck-only --execute --authorize-motion precheck-only --output-root artifacts/trials/motion-precheck-20261025`
  Existing420wall-s global bound,10wall-s pulse/30wall-s rest limits,1.5m observed clearance,
  >=5sim/3wall-s rest and all physical/truth/contact/TF/command guards remain unchanged.
  CLI authority attests authorization; it does not validate the qualification evidence.
- After actual precheck PASS + complete cleanup: separately authorize existing DWB0.4/0.2
  Start→G1→G2 A/B (route retains its own precheck), then G3→G4→G5 after navigation evidence.
  Stationary PASS proves neither motion clearance nor route reliability. No Stage4 expansion.
- Evidence: `artifacts/recovery/stationary-final-20261025/`; one concise
  `APRIL_REVIEW_BUNDLE_b2w-embodied-navigation-course_20261025_stationary-final.md` + SHA256.
  All historical raw FAIL evidence and inherited source/work preserved; no commit/push.

### Preserved preceding NOW — Stationary gate OPEN; independent precheck-only ready offline (2026-10-25)

- Single agent; authorized evidence review/implementation completed offline. No Gazebo,
  live ROS participant, navigation goal or nonzero command. No commit/push or broad kill.
- Independently verified all36 original raw-file SHA256s. Server PID2155742/start_ticks20975887
  agrees across pin/actual_server/launch/thread leader/Owner cleanup;36 threads T,621/621
  exact-zero commands,5.455sim-s measured rest,479 contact samples; historical domain/lease/
  GPU cleanup corroborated. Raw success=false/stationary_containment=FAIL remain untouched.
- Ledger correction accepted: three lines retain ONLY same_process(token/start_ticks)-verified
  server identity before existing SIGKILL. Wrong-token and stale-start_ticks regressions reject
  recording/signaling. Preexisting validation-to-signal race is not atomic pidfd protection.
- Formal stationary qualification remains OPEN: existing full-PASS gate + stationary_passed
  identity-ledger predicate have no documented evidence override. Operational findings are
  corroborated; this review does not synthesize a PASS. ONE corrected-code zero-only confirmation
  requires separate authorization, fresh/free domain225 and fresh output root; original150s bound.
- Implemented --precheck-only: sole existing precheck input→velocity_adapter, unchanged .05m/s
  forward2sim s/rest/.05rad/s turn2sim s/rest. Reuse final measured rest for normal shutdown.
  Navigation imports/handover/launch/goals live only under else; failures go to existing abort.
  --execute requires fresh matching --authorize-motion precheck-only (or route for old A/B).
  Stationary mode rejects motion authorization; precheck.used is independent of A.used/B.used.
  CLI authority is per invocation; it does not validate/replace the prior qualification decision.
- 123 focused tests PASS, including CLI authority/retry rejection, actual execute flow with external
  ROS/process boundaries replaced, both pulses/stops, failure/deadline/watchdog rejection,
  existing TF/truth/contact/rest/watchdog/route and real owned-child cleanup tests. Offline
  checks PASS46 source/install pins +26 shape handles. AST audit preserves all original guards,
  complete pulse loop, route body and exception/cleanup logic.420s motion limit unchanged.
- Readable /proc scan: zero owner/domain224/225/226 matches, no unreadable entries, historical
  pinned PID absent;224 lease free,225/226 absent. No fresh DDS graph/GPU check claimed.
- [x] Raw-evidence audit and ledger provenance/PID-reuse regression.
- [x] RED CLI/no-route execution regressions; minimal implementation and GREEN focused suite.
- [x] Offline inputs, source/control-flow review, preservation/Git/cleanup, concise bundle+SHA256.
- Minimal separately authorizable confirmation, NOT run (from this checkout; verify fresh domain):
  `ROS_DOMAIN_ID=225 scripts/run_warehouse_ab.sh --profile A --stationary-containment --execute --output-root artifacts/qualifications/stationary-ledger-confirmation-20261025`
- Only after real corrected-code stationary PASS and a NEW explicit authorization for ONE bounded
  motion attempt, NOT run (fresh/free domain226, no previous failure bypass/retry):
  `ROS_DOMAIN_ID=226 scripts/run_warehouse_ab.sh --profile A --precheck-only --execute --authorize-motion precheck-only --output-root artifacts/trials/motion-precheck-20261025`
- Next after actual precheck PASS: separately authorized existing DWB0.4/0.2 Start→G1→G2 A/B;
  then G3→G4→G5 after navigation evidence. Route mode still includes its own existing precheck.
  Do not resume Stage4/Grid Worker redesign or broad TF diagnostics without new navigation evidence.
- Review/evidence: artifacts/recovery/motion-precheck-preparation-20261025/;
  APRIL_REVIEW_BUNDLE_b2w-embodied-navigation-course_20261025_motion-precheck-preparation.md
  with SHA256 sidecar. All prior raw evidence and unrelated inherited work preserved.

### Preserved preceding NOW — Domain224 stationary operational checks passed; raw qualification FAIL from identity-ledger omission (2026-10-24)

- ONE authorized150wall-s stationary attempt consumed, output stationary-36850bcea392404d9286ed9697519703
  in artifacts/qualifications/owner-tf-delivery-20261024/. Exit1/success=false preserved; no retry.
- No TF/sensor/contact readiness fault. Full measured rest sim19.065→24.52 (5.455sim/5.49681wall s),
 621/621 final commands six-axis zero,479 contact samples. Intentional injection reached33.4985wall s.
  Independent Fortress containment verified all36 server threads T; all19 owned identities cleaned.
  Independent domain224 ROS/process emptiness, free lease and unchanged GPU PID baseline PASS.
- Only failed acceptance predicate: watchdog's owned_pid_start_ticks omitted server PID2155742/
  start_ticks20975887 after killing it before its first /proc scan. Saved server pin, actual_server,
  paused threads and Owner cleanup ledger agree. Reproduced offline; minimal guarded pre-kill
  identity retention correction + negative ownership test.74 focused tests and input checks PASS.
- No TF first-fault snapshot exists because no readiness failure occurred; Owner per-edge callback/
  insertion/spin timing not persisted. Observer rest TF sampled max age.395; last common24.2 at24.5425.
  Historical Owner TF delivery mechanism remains unresolved, not reproduced. No listener redesign.
- Raw stationary FAIL is not rewritten or administratively closed. Next: review existing pinned
  containment/rest/cleanup evidence and bookkeeping correction for explicit qualification closure,
  then prepare precheck-only motion interface before separate nonzero authorization. Current
  nonstationary harness also launches G1/G2; no motion/precheck-only interface claimed here.
- Shortest route remains bounded forward/rest/turn/rest → Start→G1→G2 → G3→G4→G5.
  Evidence/report/bundle: artifacts/recovery/owner-tf-qualification-20261024/.

### Preserved preceding NOW — Owner TF boundary observation implemented; ONE stationary diagnostic ready for authorization (2026-10-24)

- Owner TF shares global SingleThreadedExecutor spin_once with clock/contact/sensor work;
  synchronous evidence writes, heartbeat and process work can delay dispatch. Starvation is
  plausible but not proven; Owner-specific DDS delay remains possible. No scheduling fix.
- Minimal diagnostic: existing listener subclass, same node/QoS/group/Buffer insertion path;
  fixed two-edge callback/latest-buffer stamps/times/counts, spin gaps and durations, and
  common/per-edge lookup at first fault attached to existing snapshot. No per-message IO,
  duplicate listener, thread, TF restamping or change to original .5s freshness/physical guards.
- 72 focused tests PASS; A/B offline checks PASS46 pins/26 handles. No simulator/ROS participant,
  stationary attempt, goal or nonzero motion launched. Historical evidence preserved.
- ONE profile-A stationary-only diagnostic prepared: candidate domain224 (verify fresh/free
  at launch), fresh artifacts/qualifications/owner-tf-delivery-20261024, original150wall-s
  bound. Requires separate authorization; no prior sentinel reset or retry. Callback entry
  observation is not DDS reception and cannot alone split transport from ready-queue delay.
- Next: authorize ONE zero-only trial → evidence-based local correction if needed, or full
  stationary PASS → separately authorized bounded forward/rest/turn/rest + Start→G1→G2,
  then G3→G4→G5. No physical qualification PASS yet; no pulse-only runner assumed.
- Patch/report/one APRIL bundle: artifacts/recovery/owner-tf-delivery-20261024/.

### Preserved preceding NOW — TF availability deficit localized to Owner; exact receive boundary missing (2026-10-24)

- Offline replay of domain222 raw observer TF establishes both public edges14.1 before
  Owner fault wall174199.303098952; real Humble Buffer common14.1 (age.405 at Owner14.505),
  versus actual Owner common14.0 (age.505). Publisher-only lag does not explain the
  discrepancy. Owner-specific transport/backlog vs callback/partial-chain insertion
  remains unproven; no production rewrite or threshold change justified.
- 66 focused tests PASS; A/B offline input checks PASS46 pins/26 engine handles.
  No ROS node/simulator/nonzero motion or fresh stationary attempt launched.
- ONE next observation: bounded Owner TF boundary trace with original edge stamps,
  DDS source/receipt timestamps and GID, callback/insertion times, plus per-edge/common
  latest stamps at existing decisions. Verify RMW timestamp support before attributing
  transport vs callback lag. Existing separate observer is insufficient for that split.
- Stationary success rerun NOT ready; implement/verify this observation before a separately
  authorized fresh zero-only trial. Then stationary PASS → separately authorized bounded
  forward/rest/turn/rest + Start→G1→G2, toward G3→G4→G5. Existing motion runner also
  executes G1/G2; no pulse-only authority assumed.
- Evidence/report/one APRIL bundle: artifacts/recovery/tf-freshness-20261024/.
  Historical artifacts and inherited uncommitted work preserved; no commit or push.

### Preserved preceding NOW — Domain222 stationary qualification FAIL: TF age.505s (2026-10-24)

One authorized stationary-only attempt consumed, UUIDd58d6d7381bf4698a8f06e89c8ec6f6a,
output artifacts/qualifications/readiness-loop-20261024/. No retry or nonzero motion.
- Exact first fault sim14.505: map→base_link stamp14.0, age.505s > unchanged.5s bound;
  ValueError('stale_map_base_tf'). ALL other readiness predicates PASS in complete first-fault
  snapshot. Truth/contact failure null; no reported physical safety violation. TF freshness
  loss is established; publisher/buffer/scheduling root cause remains unproven.
- 529/529 final Twists six-axis zero. Full settling/rest incomplete; intentional injection not
  reached. Actual-fault watchdog containment confirmed all36 pinned server threads stopped.
  Full owned process/ROS node/GPU context cleanup PASS; domain222 empty and lease free.
  Motion pocket passed=null/unevaluated. Original150s watchdog held; fault at29.923wall s.
- Reviewed production/source/install/native pins unchanged;110 prior raw files unchanged.
  Complete evidence and concise report: artifacts/recovery/stationary-qualification-20261024/.
- Stationary remains OPEN. Next: bounded offline check of existing TF producer/buffer cadence
  against odometry around14.0–14.505 using this evidence; no general redesign, threshold change
  or identical retry. Any further live trial requires separate authority and fresh identity.
- After future PASS, separately authorize existing bounded .05m/s forward2sim s→rest→.05rad/s
  turn2sim s→rest (10wall-s pulse,30wall-s stop,>=5sim/3wall-s rest, original1.5m clearance/
  whole-stop guards). Nonstationary runner also executes Start→G1→G2 and has no precheck-only
  switch; do not run under stationary-only/precheck-only authority. G1→G2 A/B remains next
  navigation experiment, toward G3→G4→G5 after critical-segment evidence.

### Preserved historical NOW — WORLD truth PRIMARY + session map DIAGNOSTIC composite (2026-10-16)

Branch `improve/narrow-navigation`; HEAD unchanged
`33b20fd8b86fed523c794b62325dea362d629500`. Inherited dirty benchmark/staging/
diagnostic work retained. This NOW was updated before implementation; no second
roadmap/HANDOFF, commit or push. Current change is evaluation/visualization only.

- [x] Authenticated authoritative pinned AWS no-roof warehouse + generated world:
  25 static authored collision meshes, exact native mesh decoding and SDF poses.
  Feasibility STOP gate passed: exact all-height collision projection is possible;
  no screenshot, hand drawing, guessed bounds, AABB replacement or invented TF.
- [x] Canonical PNG/JSON/projected-triangle JSON in `course_navigation/maps/`.
  WORLD frame; 700×1046; .02m/pixel; pixel centers
  `x=xmin+(col+.5)*r`, `y=ymax-(row+.5)*r`. Source/image/geometry SHA256s saved.
  Two independent generations are byte-identical across all three artifacts.
- [x] PRIMARY = canonical WORLD floorplan + passive `/sim/ground_truth/odom`
  truth Path. DIAGNOSTIC = retained same-run `/map` + map→base_link SLAM Path.
  Truth source `sim_world` is an evaluation-only identity label for the existing
  zero-offset simulator worldPose output; no TF is published or fed navigation.
- [x] Optional evaluation-only map→world SE(2) waypoint labeling: exact-stamp TF/
  truth pairs, ≥30 distinct stamps, ≥1m spatial span, RMS≤.05m, max≤.10m.
  Rejected/insufficient fit means no WORLD waypoint labels. Never a control input.
- [x] Main artifact `overlays/route-comparison.png`: PRIMARY left / DIAGNOSTIC
  right (presentation-only correction; unchanged experimental milestone). Standalone full/segment panels remain. Missing truth/floorplan/source
  identity leaves PRIMARY unavailable; missing/corrupt diagnostic data does not
  suppress valid PRIMARY. Unavailable panels remain explicit in the composite.
- [x] Local horizontal UI correction only: complete standalone panels pasted
  side by side, no resize/crop; unequal heights use centered white padding.
  MOCK preview 3000×1350, LEFT [0,0,1500,1350], RIGHT [1500,0,3000,1350].
  Horizontal RED/GREEN tests and selected offline regressions: **133 passed**;
  diff check and owned cleanup PASS. All six standalone preview PNGs unchanged.
  Correction evidence: `/tmp/b2w-horizontal-composite-20261016/`.
- [x] Human visual criterion remains no large loop / major bypass; no numeric
  detour score. All-height projection includes overhead geometry and does not
  certify body clearance. Actual detour/physical success remains untested.
- [x] RED/GREEN floorplan, truth, registration, composite, source mismatch,
  diagnostic-corruption isolation and reporting tests. Initial WORLD delivery suite:
  **132 passed** (20 truth/floorplan tests + existing 112 selected tests).
  `git diff --check` and new-text whitespace PASS; only owned-process cleanup
  **PASS**, no owned survivors. Initial unsourced ROS-message import failure
  retained in logs; sourced offline suite passed without starting any ROS node.
- [x] Read-only reviewer findings fixed: authenticate collision mesh bytes in
  addition to world XML; isolate corrupt diagnostic artifacts per panel; avoid
  Pillow/SciPy imports in default dry provenance; recognize PRIMARY-only status.
- [x] Pre-task source-hash audit: only six existing files changed this turn;
  seven new files. All other preexisting file hashes unchanged, including route
  config, goal_manager, Nav2/controller/sensor/LIO/policy/calibration files.
  `load_route`, `critical_goals`, `evaluate_goal`, `run_critical`, `find_metrics`
  ASTs identical to turn start. Existing safety/cancel tests retained.

Navigation contract unchanged: exact frozen map goals/headings; G1 navigation
success + complete TF + <=.05m staging admission before critical G1→G2; stop at
G2 or first failure, no G3–G5 live continuation or correction. Historical G1
.064844853m remains original navigation success under .3m, but does not pass
new staging admission. Truth never affects goals, SLAM, Nav2, control, staging,
safety or phase progression. Cross-startup absolute-map reproducibility remains
UNVERIFIED; accepted evaluation registration does not establish it.

No agent live ROS/Gazebo/Nav2, nonzero commands, parameter changes, commit/push.
Evidence root `/tmp/b2w-world-floorplan-20261016/`: RED/GREEN, final-selected.log,
cleanup JSONs, audit.json, deterministic rebuild and explicitly MOCK preview.
Code path + snippet + causal interpretation and significant-choice AGAINST/FOR/
verdict/unknown/reversal evidence are in `docs/NARROW_BASELINE.md` WORLD section.
Final delivery step after this snapshot: exactly one normal april-review-bundle
using the approved helper, report path + SHA256, then STOP.

## HISTORICAL — B2-W low-speed policy-command calibration and repeatability

**Scope (2026-10-16):** measurement/audit/tooling only on `improve/narrow-navigation`, committed HEAD `460111a9a37105874df694235b0b561906779cf5`. No branch, additional roadmap, HANDOFF, staging, commit, push, production calibration, or nonzero live experiment. Committed source is SSoT; reviewed captures are evidence. Previous velocity characterization is historical below, with all evidence retained.

**Hypothesis:** `actual physical vx = f(policy vx command)`. Require `f(0)=0`; the positive moving branch should be monotonic if supported by observations. Never extrapolate beyond measured support. An approximately affine moving branch is a hypothesis, not a conclusion. Select the lowest-complexity model that survives fresh validation; production inverse calibration is deferred until user-run data are reviewed.

**AGAINST:** one ascending four-speed capture cannot establish repeatability, dead-zone behavior, contact/no-slip equivalence, or a fixed command mapping independent of state. Input calibration cannot remove yaw, braking or plant variability. **FOR:** final and authenticated policy commands match requests while independent truth/SLAM show elevated steady vx. If wheel targets already encode this elevation and actual wheels track them, a mapping explanation gains support. **Neutral:** authenticate geometry/sign/order and split command→target→actual wheel→body before fitting. Reverse the preference if fresh repeats are unstable, frame/timing/geometry interpretation fails, or appropriate targets plus tracking/contact evidence explain the bias.

**Implementation sequence (sole current plan):**

- [x] Authenticate branch/HEAD/origin/worktree, preserve baseline hashes for production/model files and the entire reviewed raw run.
- [x] Authenticate deployed generated URDF/SDF, visual mesh, vendored geometry, pinned candidate robot_lab reference and wheel forward signs; derive four steady-window causal tables from existing native events only.
- [x] Regressions first, then minimally add comma-separated `--speeds` to the existing runner/core. Default remains [.10,.20,.30,.40], each 3s zero→5s target→4s zero. Experimental input envelope [.04,.40] m/s (union of requested protocols and defaults); distinct finite points, no clamp. Readiness, 20Hz path, ownership, cleanup and vy=wz=0 unchanged. All duration consumers derive 12×N; keep bounded wall watchdog.
- [x] Add pure offline calibration analysis into a fresh output directory: M0 identity, M1 proportional, M2 nonzero affine, M3 monotonic measured knots; explicit zero, unsupported-domain nulls, repeat aggregation, residual/LOSO/error/domain reporting. No production model selection and no polynomial, neural model or cubic fallback needed at this stage.
- [x] Prepare Protocol A (three independent .20 sessions) and gated Protocol B (.04,.07,.10,.13,.20), exact USER-RUN commands labelled `DO NOT RUN UNTIL CHATGPT REVIEW PASSES`; do not execute either.
- [x] Run bounded static/unit/offline checks only, verify default semantics against committed source, preserve all existing run files, production bytes, HEAD/index. No ROS/Gazebo integration smoke needed unless static evidence proves insufficient.
- [ ] Generate exactly ONE review bundle under `/home/zyd/APRIL_REVIEW_BUNDLES` with approved controlled helper, verify path/SHA256, then STOP.

**Frozen scope:** RL/ONNX/PT, Kp/Kd/wheel damping/action scales, yaw_command_gain, idle_hold, Nav2 velocities/DWB/smoother/footprint/inflation/planner/SLAM and production command semantics. Truth stays evaluation-only. No aisle/planner/controller/behavior experiments. No unrelated process termination.

**Completed diagnostic result:** deployed collision radius .1125m authenticated from
reconstructed launch URDF/SDF; visual/upstream mesh bounds differ. Four forward
wheel signs are +1, order FR/FL/RR/RL by name. Mean target wheel predictions
.168073/.270384/.371409/.459380m/s already encode the positive bias; per-wheel
tracking errors are large and cancel in the mean. Evidence favors average policy
mapping while preserving coupled plant/contact uncertainty. No production model
chosen. Audit/protocols: `docs/VELOCITY_CALIBRATION_AUDIT.md`.

**Validation:** 78 bounded unit/static/offline tests PASS, 2 live-ROS tests
excluded; default dry-run and all8 reviewed derived files byte-identical in NEW
outputs. Independent read-only review's float32 repeat grouping, full source
contract pooling and truth/SLAM validity findings fixed with RED/GREEN regressions;
recheck has no critical/important finding. Live origin read-only SHA matches
460111a9a37105874df694235b0b561906779cf5. Evidence root
`/tmp/b2w-calibration-audit-20261016`; no motion/live stack, no production bytes
changed. Final remaining action is the single controlled bundle and verification;
its actual path/SHA is returned by that final operation, with no post-bundle edit.

## HISTORICAL — B2-W control-chain audit and open-space velocity steps (2026-10-15)

> **Historical execution contract, superseded by NOW above.** Historical milestones do not authorize tuning or live experiments. No sub-agents were requested in that stage.

**Execution status (independent Sol review):** independent review, harness repairs, unit/static validation, mandatory zero-motion smoke, ONE complete 48-s characterization, exact offline replay and final-code zero smoke have passed. Only final bundle generation/verification remains. The full nonzero characterization was executed by the Sol agent despite the project’s long-test separation rule, without the required explicit user exception in that turn. It is retained as agent-run diagnostic evidence, not a user-run formal experiment. No repeat is needed merely to repair provenance. Earlier Luna static-only records remain historical evidence and do not grant authorization. No tuning, extra roadmap, commit or push. Input bundle recovered read-only from Trash; SHA256 matches `206bed7b814f5e7890e910f5ca365acf9a08aaccb4b82fc1fff3cbef7d5dd129`; embedded worktree files exactly match inherited bytes.

**Confirmed independent findings before live motion:** P0 recorder was never added to/spun by an executor; Harness imports rclpy Clock as the ROS message type; private Harness feedback requests reliable QoS despite sensor publishers being permitted best effort. P1 analysis ignores the actual experiment simulation-clock origin, settling always censors an exact boundary, missing requested-zero can raise TypeError, readiness mixes explicit gates with `all(values())`, readiness emits no raw zeros, and cleanup neither destroys recorder nor guarantees repeated zero after an exception. Regressions first; use one SingleThreadedExecutor for both nodes and a zero-only readiness-smoke mode. These repairs preserve hypothesis, control path and all production control parameters. Evidence root `/tmp/b2w-velocity-sol-review`.

**Goal:** independently review and repair the inherited harness, prove real zero-motion ROS/Gazebo readiness, preserve the existing agent-run single 48-s diagnostic characterization, and separate command-pipeline dynamics, steady bias, transients and coupling before any tuning.

**Architecture / stack:** retain the existing Fortress → private joint/IMU feedback → C++ ONNX controller and SLAM localization. The Python runner launches a bounded open-area fixture with `mode:=slam control_source:=none`, then the **unchanged** Nav2 velocity smoother and velocity adapter only. It publishes 20 Hz forward steps to the normal raw command topic, deriving 20 Hz from the unchanged controller/smoother baseline; no planner, controller_server, behavior_server, goal manager or navigation action runs. Pure Python sequence/analysis functions remain importable without ROS; rclpy is loaded only for a live user run; pytest covers metric and readiness contracts. No control-law modifications.

**Starting state:** `improve/narrow-navigation`, HEAD `b83e39c9a786f9b9dfbf70d272f1bc376bd68d87`; only `PLAN.md` was modified at task start. Read AGENTS, entire inherited plan, CODEX_GUIDE, diagnosis, both interface documents, active launch/YAML/C++, relevant legacy/Gazebo configuration, downloaded B2-W policy configs and provenance docs. Local upstream-source evidence: `/tmp/b2w-control-chain-plan/`; no ROS/Gazebo nodes were started and no characterization was run. The old official-control document's “current” 60D/SRU policy and P=100/D=3.5 statements are **historical legacy-backend evidence**, not the selected rl_sar chain.

### Global constraints and evidence rules

- Do not change actuator gains/Kp/Kd, wheel damping, action scale, policy, yaw_command_gain, idle_hold, Nav2 max_vel_x, DWB, velocity smoother parameters, footprint, planner or SLAM. A demonstrated mismatch authorizes a report, not a correction in this task.
- Formal/long nonzero tests are user-run locally. Agents may implement/test the harness and perform bounded zero-motion or minimal safe smoke checks; an exception requires explicit user authorization in that turn. The existing Sol 0.10/0.20/0.30/0.40 characterization is preserved as agent-run diagnostic evidence; correcting its provenance requires no repeat. Never substitute `check_locomotion_physics.py`, which bypasses the tested smoother/adapter path. Earlier Luna validation below remains historical static-only evidence.
- No `set -e`, `set -euo pipefail`, shell `exit`, or `pkill -f`. Clean only owned PID identities. Ground truth is **evaluation only**: presence/frame metadata can gate instrumentation readiness; truth pose/twist values never control command generation, localization/TF, feedback, phase advance or safety.
- Every control conclusion must identify path + key snippet + interpretation. Distinguish (a) exact deployed artifact, (b) accompanying upstream deployment contract, (c) inspected candidate training source, and (d) proven historical training run. Matching (b)/(c) does not establish (d).
- Preserve branch/HEAD/index, existing evidence, all non-PLAN tracked files in this turn, and all protected control/navigation/config files during implementation. Hash before/after; do not restore or overwrite inherited changes.

### Independent review outcome — real validation completed (2026-10-15)

- [x] Initial branch/HEAD/origin/status/diff and complete inherited bundle/source review. Original bundle absent from requested directory; exact SHA-matching Trash copy was read without restoration. All embedded source/doc/log bytes match inherited worktree. Evidence root `/tmp/b2w-velocity-sol-review`; no agents, additional roadmap, HANDOFF, staging, commit or push.
- [x] P0 fixed: recorder never serviced by executor; wrong rclpy Clock used as ROS message type. One explicit SingleThreadedExecutor now owns both runner/recorder throughout readiness/runtime/flush. ROS Clock message alias is distinct from timer Clock. Real DDS mutation omitting recorder registration fails with zero clock callbacks; restored test passes.
- [x] P1 fixed: private sensor QoS, recorder callback readiness, explicit readiness keys everywhere, exact root namespaces/unique clock/no command-producing Nav2 nodes, readiness20Hz zeros, deterministic zero-only smoke/flush and two-node shutdown. SIGINT/SIGTERM preserve rclpy context until finally flush; real signal regression passes. Readiness graph audit throttled so sensor callbacks remain serviced.
- [x] P1 analysis fixed: actual simulation-clock origin; no fake speed results for readiness-only smoke; half-open endpoints/valid settling; None requested-zero guard; irregular zero confirmation; native clock type rollback check; body vy extraction. RED/GREEN evidence retained.
- [x] P2 fixed after live capture: repeated full-capture scan made CSV export slow. Sorted native-time index/bisect exports the REAL78,268 events and replays all8 derived files byte-identically in6.115s. Raw SHA preserved. Exact successful-run sources reconstructed and matched to pre-run `live-code-sha.json`; later changes affect export/signal handling only, then final zero smoke passes. No full successful run repeated.
- [x] Final requested suite54 PASS plus ownership1 PASS; compile in memory,11 YAML parses, help/dry-run, synthetic CLI replay/raw SHA and diff checks PASS. Environment ready=true, model bundle verified.287 protected file hashes include285 non-PLAN tracked plus PT/ONNX; no production parameter/source/model bytes changed.
- [x] Mandatory pre-motion zero smoke `/tmp/b2w-velocity-steps-sol-smoke/20261015T103511585477Z`: readiness17.2775sim s, all gates sustained3.00053wall s; all commands zero; clock8413, request/raw549,shared547,final589,truth2115,SLAM41,IMU4232,joints2111,debug/actions953,torques4213,TF504+static1. Cleanup17 owned identities gone, launch returncodes0, owner/domain194/lease independently clean.
- [x] ONE full run `/tmp/b2w-velocity-steps-sol/20261015T103630976957Z`: termination=completed, cleanup passed,17 owned identities gone, launch returncodes0, no domain/session residue. Start14.330→end62.340sim s, elapsed48.010s from20Hz completion quantization; exact48-s schedule, vy=wz0, no truth control/adaptation/reposition.981 experiment-window requests at20.0231wall Hz,max gap.05642wall s; no rollback, epoch crossing, missing stream, invalid/nonfinite event or over-limit command/truth/SLAM source gap.
- [x] Full capture78268 events: clock25371,request/raw1471,shared1469,final1512,truth6350,SLAM495,IMU12698,joints6346,debug/actions3071,torques12683,TF2259+static1. Joint received25353=captured6346+decimated19007. Actual experiment command stamps non-null;42 final startup zeros and static TF before clock retain null stamps outside experiment.
- [x] Exactly4 per-speed results plus summary.json/.csv/.md and960-row timeseries.csv. Offline replay preserves all8 derived hashes and events SHA `b21194c17e73985f7cd24f78fd5a6e965e91bf1c5c9979261a4465edf9099e98`.
- [x] Final-code zero smoke `/tmp/b2w-velocity-steps-sol-final-smoke/20261015T104157808312Z`: readiness_smoke_completed, all gates sustained, all command values zero; independent cleanup/domain/lease PASS. Clock9026,request/raw550,shared548,final591,truth2265,SLAM44,IMU4532,joints2262. This validates final signal/export changes without another motion characterization.

Measured result (m/s unless named otherwise; all4 steady-state windows valid):

| request/raw/shared/final | truth steady | SLAM steady | truth/request=truth/final gain | signed bias | rise10–90s | peak vx | request/tail overshoot % | request0→final0s | request0→stop s | final0→stop s | stop path/net m |
|---|---|---|---|---|---|---|---|---|---|---|---|
|.10|.168775|.170325|1.68775|+.068775|.137704|.201819|101.819/19.574|.2375|.7225|.4850|.069627/.069589|
|.20|.270770|.270678|1.35385|+.070770|.210863|.313423|56.711/15.649|.4250|.8675|.4425|.127753/.127709|
|.30|.371280|.371800|1.23760|+.071280|.327962|.398797|32.932/7.377|.6225|1.0625|.4400|.204917/.204850|
|.40|.459378|.459179|1.14845|+.059378|.465136|.482297|20.574/4.997|.7625|1.1900|.4275|.283559/.283316|

All requested-centered settling is censored because elevated steady speed never maintains the requested band. Target lateral/yaw drift: -.00581m/-.01125rad, -.03049/-.03160, -.11152/-.09156, -.17960/-.12846. Stop phase lateral/yaw: -.00220m/+.00134rad, -.00355/+.01505, -.00531/+.05989, -.00696/+.08759. Largest target/stop |vy|.02734m/s,|wz|.13815rad/s. No target torque saturation; largest torque/limit.20045; previous policy actions far below±100. Wheel targets are asymmetric; actual wheel-rate differences modestly grow at higher speeds, without proof of unique causal fault. Full per-speed tables, coverage, source snippets, AGAINST/FOR/neutral/verdict/reversal evidence and standalone plot are in `/tmp/b2w-velocity-sol-review/final-review-report.md` and `physical-extra.json`.

**Interpretation:** A/E supported: final command and obs[6:9] are accurate, truth/SLAM/pose-difference independently show+.059..+.071m/s steady bias. B contributes tail-relative overshoot and.43–.49s post-final-zero stop response. C contributes substantial higher-speed yaw/lateral drift. D ramp is present but nominal: raw→shared wall zero delays.191/.389/.592/.787s match unchanged.5m/s²,adapter adds~.045wall s. Mixed downstream policy/actuator/contact/plant response; no unique Kp/Kd cause.

**AGAINST** single downstream-bias attribution: one ascending-order run, unresolved training/actuator provenance, upper shaping matters for transients, coupling/contact could contribute. **FOR:** correct raw/shared/final and actual policy commands plus consistent truth/SLAM/pose-derived positive tail, without gaps/clipping, locate the steady discrepancy downstream. **Neutral verdict:** stable bias plus braking and coupling best fits these observations. “Simply too fast” is supported as actual speed exceeding request, weakened as a sufficient narrow-space cause/remedy. No production Kp/Kd or smoother change justified. Lower speed is a conservative diagnostic candidate because absolute stop/drift fall, but neither a production value nor narrow-space fix is established; .10 still yields.169m/s.

**Limitations / next decision boundary:** one trial/speed,no repeatability/randomized-order confidence,no narrow aisle/planner test,no swept/full-body clearance/contact proof,no GUI observer,unseeded physics,receipt/source timestamps do not establish actuator timing,historical training/implicit actuator equivalence unresolved. Future authorization should first establish repeatability of a matched.20 forward/stop and inspect authenticated observation/actuator semantics and wheel/load coupling; do not tune or return to aisle based on this one run. Reversal would require unity-gain repeats, a demonstrated frame/timing artifact, inflated actual policy command, or controlled actuator/contact evidence explaining the bias/drift. This turn performs no further experiment or tuning.

- [ ] Final delivery only: generate exactly one review bundle into `/home/zyd/APRIL_REVIEW_BUNDLES`, verify absolute path/nonempty/SHA, branch/HEAD/exact worktree/protected hashes and run/cleanup evidence, then STOP. No commit/push.

### A. Provenance established now; historical training remains unresolved

| Item | Evidence / result |
|---|---|
| Selected model | `course_sim/launch/simulation.launch.py`: `policy = pretrained / 'policy/b2w/robot_lab/policy.onnx'`; rl_sar branch passes `{'policy_path': str(policy)}` to `b2w_rl_sar_controller`. `course_bringup/launch/course.launch.py` defaults `locomotion_backend='rl_sar'`. |
| Absolute ONNX | `/home/zyd/program/b2w-embodied-navigation-course/third_party/b2w_rl_sar/policy/b2w/robot_lab/policy.onnx`; fresh SHA256 **`da31c0c1820b893608e2d02746bbada16d0abb5de6116277e37ccfb02024c564`**. |
| Artifact provenance | `third_party/b2w_rl_sar/POLICY_SOURCE.json`: `repository=https://github.com/fan-ziqi/rl_sar`, `revision=376d42c9b128f963ab08579762d5a216a976ce39`, robot=b2w, policy=robot_lab. Source `policy.pt` SHA256 **`38155076408e8eccb22690c6c5be14bd1dcb9149245ca5e493308a9f6ff93b34`** matches fresh bytes. ONNX is a local conversion, PyTorch2.5.1+cpu / opset17; manifest parity max error1.239776611328125e-05. This turn does not rerun inference/parity. |
| Configuration authentication | Fresh local `base.yaml` SHA256 `a883606bdbd5450ed04227b82cde54d9a3fc0eeedb55d61b36bdf43889f8d9f8`; `robot_lab/config.yaml` SHA256 `d44bb3c36043f7216f7699a23d904fd01b5fc29ab7c35aabe56dc024d3f1cc74`. The latter matches the **downloaded pinned upstream bytes**, not just its filename. `scripts/prepare_rl_sar.py` and `scripts/export_rl_sar.py` define download/export validation. |
| Capture-time deployment | Existing `/tmp/b2w-warehouse-observer/20261014T090829.047739Z-smac2d-footprint-gate-light/runtime-parameters.json`, controller `policy_path.string_value`, records that same absolute ONNX path. This proves the recorded path; today's file hash is not a contemporaneous memory/weight hash of that historical process. No claim of a new live inspection. |
| Robot asset | Manifest names `fan-ziqi/rl_sar_zoo`, commit `7dd30bdc7806898950b354260655d5a7f0ce844e`, asset version1.0.2. `description.py` copies same-name inertia and joint dynamics from that bundle while retaining course collision geometry. Asset version is not a training-run ID. |
| Candidate training configuration | Inspected [robot_lab unitree.py at 500399e](https://github.com/fan-ziqi/robot_lab/blob/500399ed75f510aeaff28705a8ce736c514dbec3/source/robot_lab/robot_lab/assets/unitree.py), `UNITREE_B2W_CFG`: nominal hip/thigh/calf stiffness160, damping5; wheel stiffness0, damping1. The separately viewed main reference shows these same nominal numbers, but is not historical policy evidence. |
| Exact task / training commit | **provenance unresolved**. [B2-W registrations at 500399e](https://github.com/fan-ziqi/robot_lab/blob/500399ed75f510aeaff28705a8ce736c514dbec3/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/unitree_b2w/__init__.py) expose Flat and Rough tasks (`RobotLab-Isaac-Velocity-{Flat,Rough}-Unitree-B2W-v0`); neither is bound to this checkpoint by an archived run config/log/checkpoint metadata. `500399e` is an inspected reference, **not** a recovered training commit. The rl_sar distribution revision is also not a training revision. |

Nominal candidate training values are insufficient even within that source revision: the inherited `velocity_env_cfg.py::EventCfg.randomize_actuator_gains` uses `mode='reset'`, scale operation and uniform stiffness/damping multipliers0.5–2.0; the inspected B2-W Flat/Rough subclasses do not disable this event. This is candidate-source behavior, not proof of this weight's actual randomization. Recover the original run's resolved environment, Isaac Lab version, robot asset revision, event overrides and trainer/checkpoint metadata before asserting actual training dynamics. If none is recoverable, keep **provenance unresolved**; do not substitute main.

### B. Current control-law evidence and comparison contract

Evidence shorthand below names real paths, to keep the table concise:

- **D** = `course_sim/config/rl_sar.yaml`; **C** = `b2w_sim/b2w_controllers/src/rl_sar_controller.cpp`.
- **U** = `third_party/b2w_rl_sar/policy/b2w/robot_lab/config.yaml`; **UB** = `third_party/b2w_rl_sar/policy/b2w/base.yaml`.
- **G** = `course_sim/course_sim/description.py`; **L** = `course_sim/launch/simulation.launch.py`; **R** = `b2w_sim/b2w_description_ros2/urdf/b2w_description_rsl.urdf`; **RA** = `third_party/b2w_rl_sar/robot_assets/b2w_description/urdf/b2w_description.urdf`.
- **T** = inspected robot_lab `500399e` asset/B2-W task/parent sources linked above and recorded with URLs/SHA256 in `/tmp/b2w-control-chain-plan/official-sources.json`. Historical training values remain unknown throughout this table. “Reference match” never means historical training proven.

| Quantity | Training value: inspected reference / historical status | Deployment value | Match? | Evidence: key snippet and why | Risk if mismatched |
|---|---|---|---|---|---|
| hip Kp/Kd | nominal160/5; actual run unresolved |160/5 Nm/rad, Nm·s/rad | Reference match | T `hip: DCMotorCfg(stiffness=160,damping=5)`; U/D `rl_kp`, `rl_kd`; C selects these after standing | balance/response mismatch |
| thigh Kp/Kd | nominal160/5; unresolved run |160/5 | Reference match | T thigh actuator; D same arrays; C same torque formula | posture/tracking mismatch |
| calf Kp/Kd | nominal160/5; unresolved run |160/5 | Reference match | T calf actuator; D same arrays; C same formula | compliance/settling mismatch |
| wheel stiffness/damping | nominal0/1; unresolved run |0/1, velocity-error torque | Numeric reference match; implicit vs explicit differs | T wheel `ImplicitActuatorCfg`; D wheel kp0/kd1; C `kd*(velocity_target-dq)` | braking/traction dynamics differ |
| startup gains | training startup not established; UB fixed gains200/200/300, Kd3 | same leg fixed gains for first2s; wheels0/1 | Deployment-contract match | C `standing=age<warmup_`; chooses `stand_kp_`/`stand_kd_`, **then** RL gains | measuring startup as policy response |
| effort limits | reference hip/thigh200, calf320, wheel20 Nm; unresolved run | same C torque clamp and all16 generated URDF limits | Numeric reference match | D `torque_limits`; C `clamp(...,-limits_[i],limits_[i])`; R/RA `<limit effort=...>` | saturation alters dynamics |
| velocity limits | reference23/23/14/50 rad/s; unresolved run | same generated URDF declarations; **no C dq-target clamp** | Declaration match; effective enforcement unproven | R/RA `<limit velocity=...>`; C wheel target `scale*last` allows up to500rad/s at action clip100 | target vs achieved speed and physics enforcement differ |
| actuator model | reference legs `DCMotorCfg`, wheel `ImplicitActuatorCfg`; unresolved actual versions | explicit fixed-limit PD in C; force plugin per joint | Structural difference to reference | C has fixed symmetric torque clamp and no explicit torque-speed motor envelope | matching gains alone cannot establish plant equivalence |
| action scale | candidate hip.125, thigh/calf.25rad, wheel5rad/s per action; unresolved run | same | Reference + U match | U/D `action_scale`; C `ref+scale*last`, wheel `scale*last` | steady bias/transients |
| observation/action clamp | U ±100; candidate action manager ordering must be audited separately | raw ONNX actions clipped±100 **before** scaling; obs±100 | U match; do not assume all training clipping semantics match | C `last_=clamp(a,-100,100)`; T joint action clip belongs to action manager | rare large targets/saturation |
| policy frequency | reference parent dt=.005, decimation4 →50Hz; unresolved run |50Hz nominal sim time | Reference + UB match | UB `dt:.005; decimation:4`; C5ms timer, `tick_%4==0` | phase lag/time discretization |
| low-level / physics frequency | reference low-level200Hz; unresolved run | torque200Hz; Fortress physics400Hz nominal | Control period matches reference; engine differs | C5ms timer; `course_sim/worlds/confined.world` max_step_size.0025 | held torque, feedback/transport delay |
| vx/vy/wz scale | U `[1,1,1]`; candidate command observation scale1; unresolved run |1/1/**1.6** before yaw clamp | vx/vy match; yaw deployment extension | U `commands_scale`; C `obs[6+i]=command_[i]`, then `obs[8]*yaw_gain_`; D `yaw_command_gain:1.6` | yaw response changes; zero yaw does not prevent drift |
| command clamps | actual training curriculum/ranges unresolved | adapter hypot(vx,vy)≤.5m/s, abs(wz)≤.6rad/s; C vx±1, vy±.5, wz±1, scaled yaw±1 | Deployment-specific constraints | `velocity_adapter.py::VelocityGate.set_command`; C command callback + obs8 clamp | command shaping mistaken for low-level bias |
| joint order | candidate FR/FL/RR/RL hip-thigh-calf, then FR/FL/RR/RL wheels; unresolved run | exactly same16 names, name-based state lookup | Reference + UB match | T `leg_joint_names + wheel_joint_names`; D `joint_names`; C `find(m->name,...names_[i])` | wrong joint pairing is catastrophic |
| default pose | candidate each leg[0,.8,-1.5]rad, wheels0; unresolved run | same | Reference + U/UB match | T `init_state.joint_pos`; D `default_dof_pos`; C subtracts/adds ref | observation/target offset bias |
| passive joint damping/friction | candidate actuator friction0; historical model unresolved | generated model has **no dynamics tags** on all16 joints, same as RA; engine defaults unverified | No explicit nonzero passive mismatch found; not proof of physical equivalence | G removes old dynamics, copies only RA dynamics; fresh XML inspection | passive vs commanded damping confusion |
| contact/slip/inertia | candidate uses training material/randomization; actual run unresolved | course collision surfaces remain; normal WheelSlip compliance0; copied inertia | Training equivalence unresolved | G keeps collision geometry, copies inertials, adds WheelSlip; `fault_profiles.yaml normal.slip_ratio=0` | slip/asymmetry/contact can alter tracking |
| zero-command behavior | U does not specify wheel-action override; historical run unresolved | idle_hold=true: wheel actions forced0 below all command magnitudes.01, legs remain learned | Known deployment extension | C `if (idle && i>=12) last_[i]=0` | stop dynamics include discrete mode change |

No live rate or physics limit enforcement is inferred from these nominal values. Fresh `/tmp/.../local-audit.json` covers all16 generated joints and fresh model hashes; `generated-torque.urdf` contains16 ApplyJointForce plugins and **zero** JointPositionController/JointController plugins. It does not replace a recorded runtime model/SDF inspection.

Exact stage trace (units, clamps, frequency, origin):

| Stage | Source + snippet | Meaning / units / timing / origin |
|---|---|---|
| Nav command and smoother | `course_navigation/launch/navigation.launch.py`: controller remap `cmd_vel_raw`, smoother `cmd_vel_smoothed`→`cmd_vel`; `course_navigation/config/nav2.yaml::velocity_smoother`: OPEN_LOOP,20Hz, max[.4,0,.6], accel[.5,0,.6], decel[-.5,0,-.6] | m/s, m/s, rad/s; linear/angular acceleration m/s²/rad/s². Deployment-specific wall-timed command shaping; FollowPath raw nominal20Hz. Behaviors also write shared cmd_vel, hence no behavior nodes in characterization. |
| velocity_adapter | `velocity_adapter.py`: `scale=min(1,max_linear/hypot(x,y))`; `create_timer(.05,...STEADY_TIME)`; `gate.output(time.monotonic())` |20Hz wall/steady, limits.5/.6, timeout.5s; zero if command/SLAM stale or TF absent. No acceleration limiter or integral. Normal path retained. |
| rl_sar command | C `command_={clamp(x,-1,1),clamp(y,-.5,.5),clamp(wz,-1,1)}` | same body velocity units; callback latest value, timeout.5sim s. Deployment-specific acceptance; actual policy consumes at50Hz. |
| policy observation | C `gyro*.25`, body projected gravity, obs6:9 command, `(q-ref)` for legs/wheel positions0, `dq*.05`, last action; obs8 `clamp(yaw*1.6,-1,1)` |57 floats; angular velocity rad/s scaled.25, q rad, dq rad/s scaled.05; body linear velocity is **not** an observation. U-derived contract except yaw gain/private-feedback/idle extensions. |
| ONNX | C `session_->Run(...observations...actions...)`; shape57→16 | dimensionless actions50Hz sim time; no body-velocity PID loop. Inference/feedback exceptions publish zero torque; preserve diagnostic distinction from commanded zero. |
| interpretation | C `target_[i]=ref_[i]+(i<12?scale_[i]*last_[i]:0)` and `velocity_target_[i]=i>=12?scale_[i]*last_[i]:0` | legs position targets rad, wheels angular velocity targets rad/s; not wheel torque actions. Targets held between policy updates. U-derived plus idle wheel override. |
| joint control | C `tau=clamp(kp*(target-q)+kd*(velocity_target-dq),-limit,limit)` |200Hz sim time, Nm, leg PD / wheel velocity-error torque. No accumulator, integral coefficient or integral state. kp/kd from D, applied **here**. Startup fixed gains are a time-selected alternative, not simultaneous gain layers. |
| bridge / Gazebo | `bridge_rl_sar.yaml`:16 Float64 ROS_TO_GZ cmd_force; G creates ApplyJointForce; upstream `ApplyJointForce.cc::PreUpdate` writes `JointForceCmd` | event transport plus physics-tick force application, no added PD in plugin source inspected. Deployment-specific Fortress transport/physics. Original R P100/I0/D3.5 and wheel controller D3 are removed; xacro Classic ros_control is not this launch input. |
| physical base / measurement | Fortress dynamics/contact → `OdometryPublisher`; bridge truth topic; `course_slam/course_slam/pose_adapter.py`: `velocity=base.R.T @ Δp/dt`, differentiated angular velocity | truth nominal100Hz from R; SLAM rate follows its source and is measured, not assumed100Hz. Truth and SLAM retain separate stamps/frames/derivatives. C does not subscribe to truth odom. |

**Answers to the PID hypothesis:** no true integral term was found in the selected command→policy→torque path. The older URDF PID-style tags are inactive in generated torque mode (and their I was0); passive damping and contact friction are physical terms, not extra feedback gains. The right abstraction is **RL body-command tracking with leg position PD and wheel velocity-target torque control**, followed by physics. Duplicate active PD layers are not supported by generated-model evidence. Deployment extensions and actuator-model differences remain material; this does not establish a historical gain mismatch.

**AGAINST tuning first:** RL dynamics assumptions may be broken by gain changes; current bias could be command interpretation/policy behavior, lateral/yaw coupling or nominal smoother lag. A straight command cannot identify damping from one response. Candidate-source gain randomization does not authorize arbitrary deployment changes.

**FOR investigating gains:** previous physical evidence shows tracking/stopping errors; damping affects transients. A proven difference from the actual training setup would justify a separately scoped correction proposal before blind tuning, after checking units, limits and control period.

**Neutral verdict: 3 — provenance is insufficient; characterize first and avoid gain changes.** Numeric gains match the accompanying deployment contract and inspected reference. No proven Kp/Kd mismatch exists; no claim that the entire training/deployment actuator model matches.

### C. Harness design fixed for implementation

**Default path tested:** requested Twist → `/course_navigation/cmd_vel_raw` → existing velocity_smoother → `/course_navigation/cmd_vel` (adapter input) → existing velocity_adapter → `/path_manager/path_manager_ros/nav_vel` → C observations/actions/torque → physics. Requested events and observed raw receipts are separate records; adapter output is exactly final nav_vel. Do not publish directly to final nav_vel. Planning is the only upstream stage replaced with a deterministic command source; test is of the **whole downstream chain**, not an ideal step applied at policy input.

Alternatives considered: direct final-nav publishing isolates policy but skips adapter/smoother; attaching to a live navigation session risks competing publishers/recovery zeros; reusing the current physical checker skips layers and has different timings. Use an isolated owned session instead. Do not mutate live nodes or start a second session in an occupied domain. No bypass flag in the first harness.

**Fixture:** the runner generates a temporary SDF from confined-world physics, systems and floor semantics, with an open20×16m interior, four outer boundary walls, and six unequal deterministic static LiDAR landmarks outside the reserved clear strip `x[-5,5], y[-1.15,1.15]`. Landmarks are observability geometry only and do not guarantee localization. Spawn is x=-4,y=0 at source height, heading+x. Source/generated hashes and exact geometry are stored in the manifest. Bounded user sequence nominal forward command integral=5m; actual motion and drift can differ. User inspects GUI and may stop immediately; no automatic reposition, reverse or yaw correction. Low-level uses unchanged private IMU/joints; localization gates remain enabled, and ground truth values only reach recording/offline analysis.

**Lifecycle/readiness:** runner launches existing course entry directly (no automatic-abort shell wrappers), `mode:=slam control_source:=none locomotion_backend:=rl_sar fault_profile:=normal`, with the generated world override. Separately start original `nav2_velocity_smoother` using the unmodified baseline YAML and exact normal remaps, plus its owned lifecycle manager and original `velocity_adapter use_sim_time=true`. Activate only that smoother; do not import full navigation.launch. Readiness requires advancing clock, fresh private joints/IMU, fresh SLAM odom and map→base TF, unique expected command publishers/controller/clock, smoother active, final zero/valid feedback and completed startup stand. Use monotonic wall deadline120s, not fixed sleep. Truth absence does not change phases: it marks evaluation unavailable; by default fail before motion for missing required recorder streams as an **instrumentation precondition**, not a truth-derived motion decision. Do not inspect truth values for readiness or control. Recheck publisher identities during the run; competing command sources cause zero/abort with explicit reason.

**Sequence:** at20Hz wall command publication, derived from unchanged `controller_frequency: 20.0` and checked against unchanged `smoothing_frequency: 20.0`; use simulation-clock phase durations. For each speed in exact order[.10,.20,.30,.40]m/s: zero settle3s → target hold5s → zero hold4s; all other Twist components0. Half-open intervals prevent a target at the final boundary. Four blocks take48sim s; every block supplies3s initial zero and4s stop observation (7s zero between targets). Initial readiness and a final zero shutdown flush are outside these48s. No adaptive extension when unsettled: abort safely on monitored non-truth health failure; a4s stop window that never settles is censored, not automatically extended.

**Bounds/failure:** motion-phase wall budget240s in addition to120s readiness; paused/frozen clock>2wall s, clock rollback, lost SLAM/feedback/expected publishers, controller death or excessive raw-IMU tilt cause repeated zero publication and an aborted partial result. Safety thresholds must be recorded; use existing25° tilt/.4m SLAM-relative base-height only if frame validity is established, otherwise omit height-based automation. No truth-based path correction or truth-based safety controller. Finally publish zero for at least1.2wall s while servicing callbacks, then shut down only owned processes (INT→bounded wait→TERM→KILL with PID/start ticks/token verification). Preserve partial files on Ctrl+C/exceptions; parent-first death still requires descendant cleanup. Final exit status distinguishes completed sequence, incomplete recording, abort and cleanup failure.

**Recording:** retain every received command/odom/clock event, private joints at source rate (~physics400Hz possible), `/network_input_debug`50Hz, `/joint_commands`50Hz and `/joint_torques`200Hz if available; no point clouds, full costmaps, DWB evaluation or video. A lightweight recorder may decimate joint arrays to100Hz with counts documented; no message subsampling of truth/commands. Truth nominal100Hz, SLAM native source rate;50Hz analysis grid is presentation only and never invents observations. Record requested schedule/event, raw, shared, final, truth, SLAM, raw IMU (roll/pitch), joint q/dq and wheel dq by joint name, debug observations and simulation clock. Optional signal absence is null/reason, not fatal to base metrics.

Each event includes integer header sec/nanosec if present, sim receive time/epoch, monotonic wall receipt, source topic/type/frame/child_frame, source sequence and full finite payload. Unstamped Twist timestamps are **receive-time proxies**, never actuator times/transport latency. Preserve nonfinite/bad samples as invalid events. Record source received Hz/gaps, simulation/wall ratio, optional drop/decimation counts, actual node parameters, executable/config/model hashes and generated model. Separate source-stamped truth/SLAM from receive-causal command ZOH. Frame interpretation must be checked from publisher/recorded child frames; unknown body twist frame invalidates affected metrics.

Outputs under a fresh run directory: `manifest.json`, `events.jsonl` raw native events, `timeseries.csv` aligned columns with per-stream stamps/ages/valid flags, `per-speed/*.json`, `summary.json`, `summary.csv` and `summary.md`. JSON is strict (no NaN); missing/censored values null with reason/coverage. Offline replay must regenerate metrics from saved events without ROS; never overwrite raw captures.

### D. Metric definitions and testable contracts

Use pure `make_sequence()`, `requested_at()`, `unwrap_yaw()`, `analyze_run()`; exact signatures in Task2. All physical metrics are primarily truth; SLAM comparisons are separate output fields from the SLAM stream. No fallback assigning SLAM to a truth field. Body vx/vy/wz from valid declared odom twist; separately derive pose-based speed/yaw rate with endpoint stamp resolution. Record odom differentiation/smoothing limitations. Interpolate only supported pose boundaries (shortest-angle yaw), not across gaps>.15s truth/.35s SLAM; raw thresholds/constants are in manifest and unit-tested.

| Metric group | Definition |
|---|---|
| Steady-state window | fixed final2s `[target_start+3,target_start+5)` in requested phase. Time-weighted piecewise-linear truth vx mean over **complete supported interval**, with boundary interpolation; emit mean only with≥95%coverage and no over-limit gap. No sample-count average. Compute SLAM mean independently. Label `steady_state_valid` only when least-squares vx slope magnitude≤.02m/s² over that window; otherwise retain `tail_mean_vx` as descriptive and steady-state fields null with reason. Do not cherry-pick a later window. |
| Gain / bias | truth mean / requested vx; signed bias=truth mean−requested vx, absolute bias=abs(signed bias), all m/s except ratio. Also compare measured final-nav mean and truth/final gain for identifying pipeline attenuation; no division for missing/zero denominator. SLAM gain/bias remain explicitly SLAM. |
| Rise | report10–90% crossing duration relative to **requested amplitude**, baseline from last1s of zero settle; optionally measured-tail-normalized rise as separate field. Compute requested→10% and requested→90% delays; additionally reference observed final command onset. Crossings linearly interpolated within supported adjacent samples. If threshold not reached, null/reason; bias can make requested90% unreachable. |
| Peak / overshoot | peak truth vx in5s target phase; `overshoot_requested_pct=100*max(0,peak−requested)/requested`. Separately `overshoot_tail_pct=100*max(0,peak−tail_mean)/abs(tail_mean−baseline)` when denominator valid. This separates elevated steady bias from peak above attained tail; do not interpret requested overshoot alone as transient damping. |
| Settling | requested-target band±max(.02m/s,5%requested); earliest point after onset with **all remaining supported phase samples**, at least.5s, inside band. Separate tail-centered settling if valid. Null/censored when never attained, insufficient tail, gap or no valid steady window. Fixed5s horizon cannot establish asymptotic settling. |
| Stop timestamps | requested-zero actual publication event; final-nav-zero first receipt after it with all components≤1e-6 and ≥.15s confirmed zero; retain first exact-zero receipt separately. Truth effective-stop onset is first pose-derived planar speed≤.02m/s **and** abs(yaw rate)≤.05rad/s continuously≥.5s, gap-guarded. Timestamp is start of qualifying support, not mechanical rest. Nonzero yaw prevents a false stop even with vx0. |
| Stopping duration / distance | requested-zero→truth-stop and final-zero→truth-stop, separately. If physical stop precedes final-zero, retain signed timing difference but final-zero-based distance null (`stop_precedes_reference`), never clamp to fabricate0. Path distance=sum planar pose segment lengths with supported interpolated boundaries; net displacement=norm(end−start). If no stop by zero-hold end, stop time/distances null+censored, plus explicit observed4s path/net. |
| Coupling | constant-speed phase requested target start→zero start; stop phase zero start→effective-stop (or observed/censored4s separately). Signed lateral displacement in **that phase's initial body-heading frame**, from its own truth pose, plus world Δx/Δy; yaw drift from unwrapped yaw. Peak abs(vy) and peak abs(wz) per phase and full trial. Repeat independent SLAM estimates without comparing absolute origins. Full5s phase and final2s window are separately named. |
| Pipeline lag / health | raw→shared→final onset/zero receipt differences in sim **and wall** time; command integrals use receive-causal ZOH with previous boundary value and freshness limit. Negative differences retained as delivery-order uncertainty. Report requested/raw/shared/final plateaus, coverage and rate; missing output is not zero. Policy debug obs6:9 independently checks actual command consumption but does not timestamp actuator application. |

**Interpretation rules:** A consistently elevated steady truth/request gain suggests command scaling or policy steady bias; B accurate steady tracking with a larger peak suggests transient/actuator/policy dynamics; C accurate vx with vy/wz/yaw drift suggests coupling/asymmetry; D delayed final command versus request/raw suggests smoother/adapter/upper pipeline; E accurate final commands with truth lag/overshoot suggests downstream policy/plant response. None uniquely identifies a cause. Inspect temporal ordering, coverage, torque saturation, wheel symmetry, pose/twist agreement and SLAM/truth consistency jointly. One run/speed has no repeatability/confidence estimate; no gain tuning, Nav2 speed recommendation or narrow-aisle safety guarantee follows automatically.

### E. Historical Luna implementation tasks and acceptance gates

#### Task1 — Complete reproducible provenance/control audit without changing controls

**Files:** create `docs/B2W_CONTROL_CHAIN_AUDIT.md`; update only current section of root `PLAN.md` for progress. Read C/D/G/L/R/RA/U/UB and existing provenance docs; do not edit them.

- [x] Capture branch/HEAD/status and protected hashes; preserve the existing recorded `policy_path` evidence and identify its time boundary. No new live session was started. Source-vs-installed/runtime limitation is recorded; current binary behavior is not inferred from a historical capture.
- [x] Verify PT/ONNX provenance and inspect pinned robot_lab B2-W actuator/task sources; Flat/Rough run binding remains **provenance unresolved**. The candidate actuator-gain randomization event is documented, not assigned to this checkpoint.
- [x] Compare actuator semantics, gains, limits, scale/clipping, order/pose, rates, damping/contact, idle hold and command chain. Missing historical run, resolved simulator configuration and verified Fortress velocity-limit enforcement remain explicit gaps; no mismatch is inferred from nominal values.
- [x] Publish the training/deployment table, all requested runtime-control answers, source snippets/units/cadences, AGAINST→FOR→neutral verdict. New audit: `docs/B2W_CONTROL_CHAIN_AUDIT.md`; verdict3 remains.

#### Task2 — Pure sequence, parsing and metric math, with failing tests first

**Files:** create `scripts/velocity_step_core.py`, `tests/test_velocity_steps.py` (pure subset). Standard-library math/json/csv/dataclasses suffice; no ROS imports in core.

**Interfaces:** `make_sequence(speeds: tuple[float,...]=(.1,.2,.3,.4), settle_s: float=3., hold_s: float=5., stop_s: float=4.) -> list[dict]` emits `{trial,phase,start_s,end_s,vx,vy,wz}`; `requested_at(sequence: list[dict], elapsed_s: float) -> tuple[float,float,float]` returns zero before/after bounds; `unwrap_yaw(values: list[float]) -> list[float]`; `analyze_run(events: list[dict], manifest: dict) -> dict` returns `{per_speed,aggregate,quality}`. Event schema is recorder contract; manifest carries sequence, units, topic identities, thresholds and clock epochs. Clock rollback/epoch crossing marks affected physical metrics invalid/null and is reported in quality; analysis writes no files.

- [x] RED evidence captured before implementation at `/tmp/b2w-control-chain-plan-red.log` (test collection failed because the pure core module did not exist). Sequence boundary and validation cases pass.
- [x] Implement pure `make_sequence`, `requested_at`, `unwrap_yaw`, and `analyze_run` with half-open 48 s schedule, supported interpolation, freshness-guarded command ZOH, truth/SLAM separation, strict nulls, rates/gaps and stop/coupling metrics.
- [x] 16 metric-core tests and 7 runner/fixture/readiness/ownership contract tests cover sequence, fixed-window/time-weighted steady state, slope validity, rise/overshoot, stop continuity/yaw, path vs net, truth/SLAM separation, clock epoch, missing/nonfinite data, command integrals, request/zero timings, deterministic geometry, offline replay, no-motion dry-run, and owned-only cleanup. Initial core GREEN log: `/tmp/b2w-control-chain-plan-green-core.log`; final combined suite is recorded below.
- [x] Post-implementation corrections retained: half-open end samples cannot contaminate steady state; stop confirmation requires continuous supported samples and reports signed final-zero timing.

#### Task3 — Lightweight ROS recorder and bounded owned runner

**Files:** create `scripts/velocity_step_recorder.py`, `scripts/run_velocity_steps.py`; extend `tests/test_velocity_steps.py`. Reuse read-only `scripts/narrow_processes.py::OwnedProcesses` ownership pattern; no unrelated helper refactor.

**Interfaces:** `scripts/velocity_step_recorder.py::create_recorder(output_dir)` lazily imports ROS, writes native `events.jsonl`, exposes `health()`, `record_request(vx, phase, elapsed_s)`, and `close()`; it does not select commands. Runner owns publisher/sequence/zero cleanup; `main(argv=None)` supports full run, `--output-root PATH`, `--ros-domain-id INT` default194, `--gui/--no-gui`, `--dry-run` (no ROS import/start/motion) and `--analyze RUN_DIR` (offline replay). No direct-final-command or lateral/yaw mode.

- [x] Pure readiness/clock/publisher contracts are unit-tested. Existing `OwnedProcesses` is reused unchanged; its narrow-process ownership test is included in the approved suite. No ROS executor or child-process motion test was started.
- [x] Implement deterministic world generation and three owned child launch components; only course SLAM/rl_sar, velocity smoother plus lifecycle manager, and velocity adapter run. Baseline YAML/remaps are read-only. No planner, goal, service mutation, or final-nav publisher exists in the harness.
- [x] Implement20Hz raw requested publisher, 48 s sim-clock sequence, event recording, readiness/runtime health gates, bounded zero flush, partial artifacts, source hashes, per-speed/aggregate outputs and offline replay. Ground truth values are used only in offline analysis.
- [x] `--dry-run`, `--help`, and `--analyze` work without importing rclpy or launching children; deterministic fixture and raw-file preservation have tests. Full ROS readiness/cleanup and physical behavior remain unvalidated by Codex.

#### Task4 — User protocol and static validation

**Files:** create `docs/VELOCITY_STEP_RESPONSE.md`; update audit doc/root plan with actual deliverables/validation. No second roadmap. No C++ change expected.

- [x] User protocol is in `docs/VELOCITY_STEP_RESPONSE.md`, including chain, asymmetric fixture, readiness, 48 s sequence, recording schema, metric/censor definitions, interpretation A–E, safety and user commands.
- [x] The exact approved pytest command passed 38 tests. The first run without ROS sourcing failed collection on missing `sensor_msgs`; a subsequent dry-run test exposed a pytest-global `rclpy` assertion, which was corrected; the final sourced Humble/install run passed. A dummy-owned-process cleanup test exercises the reused `OwnedProcesses` path; the helper itself was not modified. In-memory Python compile, four YAML parses, help, dry-run, CLI offline synthetic replay, raw SHA preservation and `git diff --check` passed.
- [x] No C++ source changed; no C++ build was applicable. Protected-control and LFS integrity comparison is part of Task5 before bundling.
- [x] **PHYSICAL VELOCITY CHARACTERIZATION NOT RUN BY CODEX.** User command below is copied from the implemented CLI/help and dry-run.

Implemented copyable USER command for the complete bounded sequence:

```zsh
cd /home/zyd/program/b2w-embodied-navigation-course
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/course_sim/config/fastdds_local.xml"
export PYTHONPATH="$PWD/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$PWD/.deps/lib:$PWD/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
/usr/bin/python3 scripts/run_velocity_steps.py --ros-domain-id 194 --gui --output-root /tmp/b2w-velocity-steps
```

That single runner command owns the complete session and sequence. User must use a prepared environment/model/install per CODEX_GUIDE; it must not silently install/download/build or attach to an unrelated session. User stops via Ctrl+C in the owning runner terminal; no broad kill commands. Re-analysis will use `/usr/bin/python3 scripts/run_velocity_steps.py --analyze /absolute/run-directory` and leave raw files unchanged. The command was checked against the implemented `--help` and `--dry-run`; its 20 Hz cadence, 48 s sequence, and raw-topic chain are available without ROS startup in dry-run mode.

#### Task5 — Review bundle and stop

- [x] Reports, actual validation and explicit limitations are recorded above and in both new docs. No physical result is claimed. Pre-bundle protected-hash comparison passed for all285 baseline tracked files other than this updated PLAN; branch/HEAD/origin and diff checks are clean.
- [x] Previous Luna delivery completed as the recovered SHA-matching170009 bundle; historical instruction was: invoke `~/bin/april-review-bundle /home/zyd/program/b2w-embodied-navigation-course [evidence]` collection logic with output **only** `/home/zyd/APRIL_REVIEW_BUNDLES`; include PLAN, new docs/scripts/tests, RED/GREEN logs, static/CLI/test results, provenance-source records and integrity evidence. If the original helper uses prohibited shell abort/exit constructs, preserve its collection logic in a controlled `/tmp` copy and replace only those constructs with function-return error handling; leave the original helper untouched.
- [x] Previous Luna bundle verified from its recovered copy during independent review. Current final delivery is tracked only in the independent review outcome above.

**Historical Luna status:** Tasks1–4 and170009 bundle were completed; independent repair/live results and the sole current final-delivery boundary are recorded above. Historical braking audit follows unchanged.

## Historical — Offline braking / tracking mismatch and recovery-path audit (2026-10-15)

**Verdict:** downstream command lag and physical tracking mismatch are measured, but the evidence is insufficient to choose one of the three proposed navigation changes as a causal remedy for the initial stall. H8 is plausible for continued penetration/retries; its claimed first legal→illegal transition after failure is not established. The first saved illegal pose is already 90 ms before the first no-valid-control/raw-zero receipt. Recovery failures are separate: Spin pulses can be overwritten between adapter ticks, while BackUp rejects motion before publishing reverse. No reverse-x or vy recommendation; the 143034 frozen result stands.

**Scope / provenance:** current branch `improve/narrow-navigation`, HEAD `b83e39c9a786f9b9dfbf70d272f1bc376bd68d87`, clean initial tracked worktree. Read AGENTS, CODEX_GUIDE, the entire existing plan/diagnosis, current launch/config/adapter, actual `course_sim/config/rl_sar.yaml` and controller C++, interface documents, and the 143034 review bundle. Only this NOW section is changed in the repository. All temporary source/results/tests are in `/tmp/b2w-braking-review`; no Gazebo, live Nav2/ROS node, observer attach, goals, parameter tuning, commit, push, process signalling or agents. No additional roadmap.

The requested bundle was absent from `/home/zyd/APRIL_REVIEW_BUNDLES`; read its retained copy under `/home/zyd/.local/share/Trash/files/`, with original-path trash metadata. Its SHA256 is `41800165ab25ae36e66bfdcc2104503cc9e786775398fef003d67263d650733d`. Referenced 143032 archive SHA256 `e58867064e7b4e9deb4be6034363f1a0f1b7bf2beedacb93f99b4babeb155da6` also matches its retained checksum. All **3,844 files** of exact run `20261014T090829.047739Z-smac2d-footprint-gate-light` under `/tmp/b2w-warehouse-observer` were compared byte-for-byte by hash against that archive. No evidence regeneration or source writes. Historical run HEAD5a2199f differs from the present documentation HEAD; runtime capture determines run parameters.

### A. Time basis, runtime and braking timeline

Commands are unstamped: times below are observer simulation-receive ticks, preserving seq and monotonic wall receipts in JSON. They are not exact actuator/application timestamps. Truth/SLAM poses are independently interpolated at their **header** stamps (shortest-angle yaw), using delayed messages offline. No absolute SLAM/truth origin comparison or rigid alignment between their absolute coordinates is made. SLAM at header119.9 arrived after raw failure; the latest SLAM **received** at119.9075 still has header119.5. `aligned-timeline.csv` contains 119 rows covering119.7–121.5, the union of command/log/map event times plus0.1s grid, actual poses, latest header body twists and their stamps/receipts, receive-causal commands, local-map stamps, frozen legality and controller logs. Pose interpolation may use later evidence; body twists are last header≤row time, not instantaneous interpolated velocity. `window-events.json` retains exact command/log/network events.

Runtime capture: controller20Hz, failure_tolerance=.3s; StandardTrajectoryGenerator, sim_time1.7, spatial steps .05m/.025rad, discretize_by_time=false, include_last_point=true, limit_vel_cmd_in_traj=false. DWB x acceleration/deceleration +.5/−.5m/s², max_vel_x=.4; y command/acceleration/deceleration0; angular +.6/−.6rad/s². Smoother20Hz **wall timer**, OPEN_LOOP, x+.5/−.5, max/minx=.4/−.2, deadband0, scale_velocities=false, velocity_timeout=.5s. Adapter steady timer .05s, timeout .5s, max_linear .5/max_angular .6, latest-command storage. RL runtime idle_hold=true, command_timeout=.5s, feedback_timeout=.15s, yaw_command_gain1.6; source updates torque200Hz/policy50Hz and disables wheel velocity actions below command components .01, retaining learned leg actions. It supplies no measured .5m/s² body stop guarantee. **73 captured navigation/config keys match**; behavior-server parameters were not captured, so its current YAML10Hz/.6 angular limit is static evidence only. Installed DWB/controller/smoother/behaviors packages report1.1.20.

| Event | Sim receive s | Seq | Value / interpretation |
|---|---:|---:|---|
| Last nonzero raw |119.8575|29805|(.3789473684,0,0)|
| First raw zero + first no-valid-control |119.9075|29845 /29846|419 trajectories invalid; repeated zeros follow|
| First shared decrement |119.9175|29853|vx=.3539473684|
| First final decrement |119.9325|29867|vx=.3539473684|
| Patience exceeded / FollowPath abort |120.1925|30073 /30076|subsequent retry begins120.2000|
| Shared/final first vx≤.10 |120.3525|30230 /30231|vx=.0789473684; first loop-rate warning also120.3525|
| Shared / final first vx≤.05 |120.5125 /120.5450|30337 /30365|vx=.0289473684|
| Shared / final first vx≤.02 AND≤.01 |120.5700 /120.5975|30388 /30406|vx=.0039473684|
| Shared / final exact zero |120.6175 /120.6450|30420 /30441|all commanded components0|
| Truth effective stop, defined below |121.4300|header-derived|also satisfies1s sustained threshold|
| Earlier extent-based stationary detector |121.4550|samples.jsonl|different definition, through141.2075|
| SLAM effective stop |121.5000|header-derived|.1s pose sampling|

Thresholds here are absolute forward speeds; all vy/wz commands in the braking window are0, so the manifest's .02m/s and .05rad/s command thresholds are also satisfied. Effective stop means pose-differenced planar speed≤.02m/s AND |yaw rate|≤.05rad/s continuously for .5s; a1s sensitivity gives the same start. Truth interval endpoints have .01s resolution; this is not exact mechanical rest. Raw→shared-zero delay=.7100sim/.759462455wall s; raw→final-zero=.7375sim/.788937271wall s; shared-zero→final-zero=.0275sim/.029474816wall s. Sixteen decrements are .025m/s per tick except the last .0039473684. The .759s wall duration agrees with the nominal .757895s ramp; there is no evidence of an abnormally slow extra smoother ramp. Simulation-clock batching around120.35 prevents interpreting every sim-receipt gap as a timer period.

Compact view below: commands are vx (vy=wz=0 throughout); poses are Δx,Δy,Δyaw from each stream's **own**119.7 pose, in m/rad. Full poses/twists and event-time table are in the CSV.

| Sim s | Raw vx | Shared vx | Final vx | Truth Δx,Δy,Δyaw | SLAM Δx,Δy,Δyaw | Latest saved local map / legality |
|---|---:|---:|---:|---|---|---|
|119.7000|0.4000|0.2829|0.2829|0.0000,0.0000,0.0000|0.0000,0.0000,0.0000|119.215 / True|
|119.8175|0.3789|0.3579|0.3329|-0.0352,0.0067,-0.0120|-0.0356,0.0056,-0.0119|119.8175 / False|
|119.9075|0.0000|0.3789|0.3789|-0.0643,0.0124,-0.0194|-0.0651,0.0095,-0.0195|119.8175 / False|
|120.0000|0.0000|0.3289|0.3289|-0.0980,0.0181,-0.0267|-0.0988,0.0136,-0.0268|119.8175 / False|
|120.2000|0.0000|0.2039|0.2289|-0.1734,0.0261,-0.0485|-0.1743,0.0172,-0.0487|119.8175 / False|
|120.3800|0.0000|0.0789|0.0789|-0.2259,0.0287,-0.0754|-0.2268,0.0164,-0.0756|120.38 / False|
|120.5975|0.0000|0.0039|0.0039|-0.2608,0.0306,-0.1103|-0.2619,0.0162,-0.1105|120.38 / False|
|120.6450|0.0000|0.0000|0.0000|-0.2643,0.0312,-0.1177|-0.2643,0.0174,-0.1176|120.38 / False|
|120.8000|0.0000|0.0000|0.0000|-0.2683,0.0348,-0.1390|-0.2685,0.0207,-0.1391|120.38 / False|
|120.9550|0.0000|0.0000|0.0000|-0.2676,0.0404,-0.1541|-0.2685,0.0273,-0.1539|120.955 / False|
|121.2000|0.0000|0.0000|0.0000|-0.2704,0.0466,-0.1692|-0.2721,0.0330,-0.1694|120.955 / False|
|121.4550|0.0000|0.0000|0.0000|-0.2752,0.0455,-0.1801|-0.2769,0.0317,-0.1802|120.955 / False|
|121.5000|0.0000|0.0000|0.0000|-0.2758,0.0449,-0.1815|-0.2775,0.0311,-0.1818|120.955 / False|

At failure, last header≤119.9075 truth body twist=(.338049,−.026425,−.080462), SLAM=(.326161,−.024470,−.085300); both headers119.9. Pose-derived bracket velocities are(.345941,−.024957,−.078813) and(.365831,−.022263,−.077969). Body twist and pose derivatives differ because of odometry differentiation/smoothing. Truth vx continues increasing to .391037 at header120.06 despite raw zero. At final-zero time, latest truth body twist header120.64 is(.069555,.001685,−.154482), so final zero is not physical rest. CSV includes every corresponding velocity/stamp.

### B. Actual stopping envelope and H8

Command integration uses receive-time **zero-order hold**, including the final command already active at the left boundary; physical path is the sum of header-pose segments with interpolated boundary poses. Command vx integral is body-forward command distance; pose path also contains lateral motion, yaw response and small oscillation. Their difference is a tracking diagnostic, not an odometry calibration error or pure one-axis excess. Net translation is supplied separately.

| Interval | ∫final vx dt m | Truth path / net m | SLAM path / net m | Truth / SLAM Δyaw rad |
|---|---:|---|---|---|
|Raw zero→final zero|0.132849|0.201068 /0.200841|0.199846 /0.199305|-0.098258 /-0.098083|
|Raw zero→effective stop121.43|0.132849|0.224521 /0.213086|0.223968 /0.212683|-0.159815 /-0.159834|
|Final zero→effective stop121.43|0.000000|0.023453 /0.017965|0.024121 /0.019123|-0.061557 /-0.061752|
|Raw zero→121.5|0.132849|0.225868 /0.213968|0.225275 /0.213491|-0.162126 /-0.162260|
|Final zero→121.5|0.000000|0.024801 /0.017911|0.025428 /0.019039|-0.063868 /-0.064177|
|Last legal119.215→first illegal119.8175|0.160132|0.187827 /0.187502|0.188872 /0.188526|-0.042610 /-0.043048|

Ideal continuous stop under actual smoother |max_decel_x|=.5: t=v/.5, d=v²/(2×.5). Initial raw/final v=.378947 gives **.757895s/.143601m**. Truth header vx=.338049 gives .676099s/.114277m; SLAM header vx=.326161 gives .652321s/.106381m. These are ideal **vx** envelopes assuming perfect response and zero new yaw/lateral motion. Actual final-command decay lasts .7375sim s, integrates **.132849m**; truth effective stop takes1.5225sim s after raw zero, travels **.224521m path/.213086m net**, turns **−.159815rad (−9.1567°)**. Truth path exceeds command integral by.091672m, of which.068219m accrues before final zero and.023453m after. Yaw before/after final zero is−5.6298°/−3.5269°. The measured discrepancy is larger than a pure post-final-zero coast.

Frozen margin is directional and orientation-dependent, not a scalar aisle width. Installed perimeter legality accepts253 and rejects254/255; the filled-polygon distance below is **separate geometry**, never substituted for the critic. Real padded rectangle is≈1.50×.86m, local resolution .050000000745m. All maps have identity origin orientation and raw hashes validated. At118.6225 and119.215 the frozen footprint is legal; forward fixed-yaw first rejection is bracketed [.311,.312]m and[.260,.261]m at1mm sampling. Minimum filled-rectangle distance to represented lethal cell areas is respectively **.086703m/.001781m**. Holding center fixed, negative yaw alone first makes the installed footprint illegal at **−.1265rad/−.0025rad**, bracket resolution .0005rad. Thus .132849m commanded stop and .224521m truth path are below both straight margins, but much larger than the nearest-obstacle margin at119.215; neither scalar comparison proves legality of the turning rectangle. The recorded−.159815rad yaw exceeds both fixed-center yaw margins. No physical aisle clearance is inferred from cost cells.

Distance between last known legal and first illegal snapshots is truth.187827m path/.187502m net; SLAM.188872m path/.188526m net; commanded integral.160132m, yaw truth−.042610rad. These are meaningful motion measurements across .6025s, **not a clearance consumption measurement on one unchanged map**: maps/rolling origins differ, and later120.38/120.955 follow clears. Saved exact-stamp TF was reconstructed after .4325–.475s; footprint/map skew is.015–.180s. This is not the controller's synchronized private pose/map.

**H8 explicit test:** “DWB detects loss of feasibility, but downstream smoothing/tracking lets the robot continue far enough that the next controller state begins inside the footprint-illegal region.” The continued motion and persistence of saved illegality are supported. The first saved illegal map/pose119.8175 precedes raw-zero/no-valid-control119.9075 by.090s, while last valid raw119.8575 still exists. Therefore these observations do **not** establish that post-failure motion caused the first illegal transition. Without the synchronized internal DWB pose, map and rejected rollout, neither that apparent ordering nor a specific first rejecting critic can be transferred to the live controller. H8 remains unresolved for initial entry; it is consistent with worsening the state of later retries. Smoother delay alone is not demonstrated to be the next causal bottleneck.

### C. DWB prediction versus downstream rate limiting: pinned official1.1.20

Official tag1.1.20 source was fetched read-only into `official/`; package versions and source hashes are retained in the bundle. Line numbers refer to those downloaded files.

1. [StandardTrajectoryGenerator](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_plugins/src/standard_traj_generator.cpp#L151-L208),151–208: rollout starts from supplied current velocity, projects each component toward sampled target with kinematic acceleration/deceleration, then updates pose. With limit_vel_cmd_in_traj=false the published trajectory velocity remains the sampled target. Spatial timestep selection uses the **target** magnitude; a very small/zero target may yield one1.7s step, so the rollout is not a high-resolution physical stopping simulation.
2. [ControllerServer](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp#L465-L505),465–505: thresholded odom twist enters the controller; PlannerException during positive tolerance creates a zero Twist until time since last valid command exceeds tolerance. Controller publication and abort-zero handling send this through the raw remap. A tolerance zero is a server fallback, not a newly scored DWB stop trajectory.
3. [VelocitySmoother](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_velocity_smoother/src/velocity_smoother.cpp#L224-L331),applyConstraints/smootherTimer: OPEN_LOOP uses last smoothed command, clamps each delta to accel/frequency or decel/frequency, publishes and remembers it. It does not use physical twist in this mode. The20Hz wall timer and.025m/s x decrements match the raw record.

**Interpretation:** a separate downstream rate-limiting stage follows DWB's acceleration-aware prediction. DWB models measured current twist; OPEN_LOOP evolves its own last-command state; the adapter resamples that output; RL policy/physics add response dynamics. Equal numerical limits do not equate those states or make the cascade a bug. Do not add two ideal stopping distances or label this a “double deceleration bug.” Measured raw→final zero.7375sim/.78894wall s and truth path.22452m quantify the mismatch. The smoother's own wall ramp is nominal; the physical discrepancy/yaw and missing private state prevent attributing it all to smoothing.

### D. Frozen pre-failure stop counterfactuals: OFFLINE only

For each actual saved pose/map/recorded padded footprint, in-memory C++ calls installed `BaseObstacle::scorePose` and `ObstacleFootprint::scorePose` at every propagated pose; no ROS initialization or DDS. A replays the exact final-command receipt-time decay from failure (initialvx=.378947); A-matched scales the same shape to the snapshot's last received final vx. B analytically decelerates that snapshot command with.5; C uses **offline diagnostic** magnitudes1 and2m/s²; D uses entry.20/.10m/s with.5. Propagation uses exact piecewise planar body-twist integration, ≤5ms steps and every command switch. These numbers are diagnostic envelopes, not selected production settings. All nominal commanded vy/wz are0.

| Snapshot | Last received final vx | A literal / A matched distance m | B distance m | C1 / C2 distance m | D.20 / D.10 distance m | Installed sampled perimeter verdict A–D |
|---|---:|---|---:|---|---|---|
|118.6225|0.290789|0.132849 /0.101943|0.084559|0.042279 /0.021140|.040000 /.010000|all legal along the sampled straight stop|
|119.2150|0.232895|0.132849 /0.081647|0.054240|0.027120 /0.013560|.040000 /.010000|all legal along the sampled straight stop|
|119.8175|0.332895|0.132849 /0.116704|0.110819|0.055409 /0.027705|.040000 /.010000|all illegal at t=0; not a legal pre-failure state|

Snapshot body-vx sensitivity for ideal.5m/s²: at118.6225 truth.375516/SLAM.325846 gives .141012/.106176m; at119.215 truth.359982/SLAM.359353 gives .129587/.129135m. These straight sweeps are also sampled-legal. At119.8175 body-vx.309030/.301351 gives.095499/.090813m, but every envelope fails at its starting pose. “Pre-failure” by log time does not mean legal.

A literal reuses the recorded entry speed even where it differs from local speed; A-matched is explicitly an illustrative scaling, not identified plant response. Nominal geometry permits propagation, but a guarantee about **actual physical** rectangle legality under B–D is **unavailable**: saved geometry does not determine future yaw/slip/tracking for changed commands. As a sensitivity, rigidly transplant only the **relative** recorded truth or SLAM braking path from119.9075–121.455, rotating its initial heading into each frozen pose, without absolute-origin comparison. Both hit an illegal footprint from118.6225 (first relative1.0525/1.0925s) and119.215 (.0225/.0925s). This is not a valid changed-speed dynamics model; it demonstrates why straight-command distances alone cannot certify a stop. No swept physical-leg envelope or continuous between-sample guarantee is claimed. The tests compare actual installed perimeter semantics, including closing edge/interior behavior; filled geometry remains separately labelled.

### E. Separate recovery command-path audit

```text
ControllerServer -> /course_navigation/cmd_vel_raw -> VelocitySmoother
                                                      |
BehaviorServer ----------------------------------------+-> /course_navigation/cmd_vel
                                                            -> VelocityAdapter (latest command, steady20Hz)
                                                            -> /path_manager/path_manager_ros/nav_vel -> rl_sar
```

[Official Humble1.1.20 navigation_launch.py](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_bringup/launch/navigation_launch.py#L114-L183),non-composed lines114–183/composed205–242: controller→cmd_vel_nav; smoother cmd_vel_nav→cmd_vel; behavior_server directly→cmd_vel. Project changes names to raw/shared and adds the20Hz adapter. Both branches preserve the upstream behavior bypass; **two publishers alone do not establish wrong wiring**. There is no source arbitration or behavior ownership in the adapter: its command callback stores the newest value, and its timer later publishes that value after freshness/TF gating.

| Spin shared pulse→next shared zero (sim s) | Seq pulse /zero | Width sim /wall ms | Adjacent final receipts (sim s) | Final receipts inside pulse |
|---|---|---|---|---:|
|121.7300→121.7425|31296 /31313|12.500 /14.188|121.7200 /121.7675|0|
|128.4675→128.4950|36885 /36909|27.500 /15.553|128.4650 /128.5150|0|
|135.3775→135.3925|42588 /42602|15.000 /13.607|135.3675 /135.4125|0|

All pulses=(0,0,.6); adjacent final outputs are zero, and no nonzero finalwz exists after initial failure through141.2675. Wall durations14.188/15.553/13.607ms are the relevant comparison with the adapter's50ms steady period; sim widths alone would misstate the second pulse. Offline latest-shared-at-final-receipt replay predicts zero for every final sample near the three pulses. Tests exercising the **actual VelocityGate class extracted without ROS** reproduce pulse overwrite between ticks and preserve a pulse when sampled. No pulse buffer, immediate forwarding or source ownership exists. Per-message publisher GID and actual adapter callback/publish timestamps are absent, so attribution of each intervening zero and exact callback order remains an inference. Smoother periodic zeros plus adapter sampling are sufficient to explain all three losses; freshness/TF veto cannot be independently excluded at every tick from observer data.

Spin BT RUNNING→FAILURE sim121.730→121.8575,128.4675→128.6025,135.375→135.500. BackUp sim127.575→127.6075 and134.4025→134.435; no negativevx appears at raw, shared or final. Crucially, supplemental **same-session** files `~/.ros/log/behavior_server_4078269_1791968818952.log` and `2026-10-14-17-06-58-822919-zyd-MS-7D97-4078250/launch.log` were found read-only. Launch PID4078269 matches; all316 original rosout header/message pairs match that launch log exactly. They are copied with hashes and matching seqs; they are supplementary logs, not invented missing rosout events.

Behavior wall stamps: BackUp starts1791968956.698857025, logs `Collision Ahead - Exiting DriveOnHeading` at1791968956.699442327 and fails; second starts1791968963.598278350, rejects1791968963.598514870. [DriveOnHeading1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_behaviors/include/nav2_behaviors/plugins/drive_on_heading.hpp),onCycleUpdate checks collision **before** nonzero velocity publication and calls stopRobot on failure. This explains missing reverse **upstream** of adapter; it is not evidence of reverse clamping downstream. Behavior runtime params/collision pose/map and exact rejecting cell are not saved.

Spin starts wall1791968950.628275623/8957.628250298/8964.578687642, then `Collision Ahead - Exiting Spin` at8950.728890231/8957.728662323/8964.679326588 (same179196 prefix). Each runs≈100ms before its safety stop/failure, per [Spin1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_behaviors/plugins/spin.cpp). Early shared zero after14–16ms precedes this logged stop; overwriting is consistent with the smoother path, but lacks publisher identity. Restoring arbitration does not make the rejected spin/reverse collision-legal. A prospective fix must honor explicit behavior stop/abort immediately; do not stretch or latch .6 pulses across a safety zero.

**Classification:** recovery failure, not initial-stall cause (recoveries begin after121.73 versus initial119.9075). Spin has a demonstrated delivery vulnerability plus independent behavior collision abort; exact callback attribution remains unresolved. BackUp has supported pre-publication collision rejection. Recovery routing alone cannot establish successful escape from the saved footprint-illegal state.

### F. Three single-variable candidates: AGAINST before FOR

| Candidate | AGAINST | FOR | Evidence targeted | Cannot solve | Required open-space precondition before any aisle run |
|---|---|---|---|---|---|
| 1. Lower diagnostic DWB max_vel_x only | Changes sampled targets/grid and entry history; no same-state comparison proves it prevents gate loss. Changed vx does not specify yaw/slip. Existing y/reverse result stays frozen. | Ideal distance scales with v²; lower entry energy reduces required stopping room and exposure to delayed state. | Whether earlier lower entry speed reduces measured rollout/tracking divergence before illegality. | Already-illegal initial footprint, missing behavior commands, biased/delayed pose or orientation clearance by itself. | Later-authorized sole-source forward/stop trials through existing chain, measured attainable speed, signed truth/SLAM displacement, yaw and stopping/stability; full rectangle clearance, healthy TF and synchronized controller pose/map. No number selected now. |
| 2. Stronger diagnostic smoother x deceleration only | Current.759s wall ramp already matches.5; final integral.13285m is less than ideal.14360m, so no observed excess ramp time to “repair.” Physics/yaw may not follow a faster target; nominal frozen stops are already legal. | Directly isolates the observed nonzero downstream tail while leaving DWB parameters unchanged; ramp/area prediction is falsifiable. | Change raw-zero→final-zero area/delay and resulting truth displacement/yaw, measured separately. | First saved illegality predates raw zero; no guarantee against pre-failure state mismatch, yaw drift, or behavior rejection/pulse loss. | Later-authorized matched-entry open-space stop/stability trials first, no competing input; verify stronger target braking produces repeatable physical stopping without pitch/leg instability or larger yaw, with current DWB/geometry/policy unchanged. Offline1/2 are not production settings. |
| 3. Recovery command arbitration/routing fix only | Present bypass matches upstream; exact callback identities unavailable. BackUp already refuses collision before publishing; Spin also aborts. Delivering/latching a pulse after explicit stop would be wrong. | Source and all3 pulse windows directly expose latest-value timer loss; deterministic class tests reproduce it. Strongest causal support **for Spin command delivery only**. | Ownership/overwrite/sampling, including zero/abort priority and source attribution. | Initial119.9075 DWB infeasibility, physical stop mismatch, recovery collision legality, BackUp pre-publication rejection. | First use isolated open-space two-source pulse/zero tests with publisher identity and adapter callbacks recorded; show intended command reaches final while all stop/abort/stale/TF gates retain priority. Only then separately validate permissible behavior motion and stopping. |

**Single next-experiment verdict: evidence still insufficient.** None of these three has sufficient causal evidence as the next **initial-stall** remedy: the current nominal smoother ramp is correct, the measured yaw/near-zero side margin invalidates straight-distance reassurance, and the first saved illegal state precedes failure. Candidate3 has the strongest local explanation for the missing Spin output, but does not address initial stall and cannot override the independent collision failures. Do not substitute that narrower delivery finding for navigation feasibility. The missing discriminating prerequisite is synchronized controller-private pose/map/rollout plus repeatable open-space command→physical stopping/yaw evidence; no production values, reverse-x or vy are chosen, and no live experiment is authorized by this report.

### G. Validation, integrity and review bundle boundary

- Offline C++17 sweep compiled with −Wall/−Wextra/−Wpedantic against installed dwb_critics; it never creates a ROS node or initializes DDS. Recorded maps replay exact installed perimeter rules. New temporary helpers/tests stay outside the repository.
- Fresh tests: **64 passed in6.76s** =53 existing observer/frozen-critic/config tests +7 math tests +4 installed-batch/actual-adapter contract tests. Cover ZOH boundaries, independent path/net distance, shortest-yaw interpolation, time support guards, body frame, constant-twist propagation, ideal stop scaling, perimeter closing edge/interior semantics, and pulse sample/overwrite behavior. No live integration test is claimed.
- 73 runtime/YAML checks match; exact evidence archive hashes match; initial manifest protects all286 tracked files and3844 original run files. Final integrity confirms only PLAN.md / NOW changed, stable HEAD/branch, all3,844 original evidence files and285 other tracked files unchanged (4,129 protected hashes), diff whitespace clean, and in-memory Python/source static checks. `integrity.json` records these checks. Validation records and a standalone braking PNG/SVG are included.
- Reproduce offline: source `/opt/ros/humble/setup.zsh`; configure/build `/tmp/b2w-braking-review` with CMake prefix `/opt/ros/humble`; run `analyze.py` and the two temporary test files plus `tests/test_warehouse_observer.py`, `tests/test_frozen_obstacle_critic.py`, `tests/test_footprint_gate_config.py` with `/usr/bin/python3 -m pytest`, using a `/tmp` pytest cache. Reproduction requires the recorded original source paths or equivalent path edits; no Gazebo/Nav2 launch needed.
- Final operation writes a self-contained review Markdown, the exact original run plus this offline analysis/source/test evidence archive, and SHA256 checksums **only into `/home/zyd/APRIL_REVIEW_BUNDLES`**, verifies their contents/hashes within that operation, prints absolute paths/hashes and stops. No post-bundle work, commit or push. Final path/hash are reported from that operation, not guessed in advance.

## Historical — Smac2D + BaseObstacle + footprint legality gate implemented (2026-10-14)

**Current authorized action:** execute the already-reviewed design in this workspace. The candidate YAML and regression test now exist. Only offline/static validation is authorized; no Gazebo/Nav2, goals, live parameter changes, commit or push. Evidence: `/tmp/b2w-footprint-gate-implementation`; final bundle directory: `/home/zyd/APRIL_REVIEW_BUNDLES`. No additional plan-only checkpoint or agents.

**Implementation status:** exact two-line insertion completed; RED was 2 missing-candidate failures / 3 passes, then all **53** specified config/frozen-critic/observer tests passed in 4.59s. A fresh temporary no-node/no-DDS pluginlib probe compiled and instantiated both installed critics successfully. No live result exists.

**Goal:** create one independent diagnostic derived from current Smac2D + DWB, preserving BaseObstacle and adding ObstacleFootprint as a perimeter-legality gate with a bounded, small positive soft score. This is a diagnostic single-variable experiment, not the new baseline.

**Architecture / stack:** copy `course_navigation/config/nav2_smac2d_diag.yaml` verbatim, insert one critic immediately after BaseObstacle and its positive scale. Use installed ROS 2 Humble Nav2 1.1.20 pluginlib/DWB, PyYAML and pytest. Preserve the existing launch and LIGHT observer; no public topic, TF, message or module-boundary change is needed. Execution stays inline in this authorized workspace; no worktree or extra roadmap is required for this config-only change.

**Current HEAD:** `945afc9e8af2d9782fe0e901dfed5624431e44f3` on `improve/narrow-navigation`; local HEAD, cached origin ref and live `git ls-remote` agree. The plan-only turn used `b30055eba0f576a6ad170baf9979ff60b12f5c18`; the user subsequently committed/pushed four video files through Git LFS. Video contents, index pointers and `.gitattributes` are protected; no commit or branch change by this implementation turn.

**Inputs read / provenance:** `AGENTS.md`, `CODEX_GUIDE.md`, this plan, diagnosis and source configs. Approved plan-only bundle `APRIL_REVIEW_BUNDLE_b2w-embodied-navigation-course_20261014_155445.md` was absent from the requested directory but read from `/home/zyd/.local/share/Trash/files/` without restoring or modifying it; SHA256 `6ba98fc567f8c9ca6062f2178e4b39fd4eba49d0c0039863e0fb2bee0f031634` matches the prior saved verifier. It embeds the inherited plan verbatim. Historical evidence HEADs below identify their original captures, not the current repository state.

### Scientific motivation and constraints

Four exact frozen local map/recorded-pose inputs at simulation stamps **95.3975, 95.9975, 96.5975, 97.1975** are center-legal for installed BaseObstacle and perimeter-illegal for installed ObstacleFootprint because perimeter cells have cost254. First DWB feasibility failure at **95.035** precedes the first saved mismatch. The first rejecting critic and exact failing rollout are unknown; LIGHT did not subscribe to evaluation. These snapshots demonstrate an approximation gap, not the initial cause or a feasible alternate trajectory. Center253 can be Base-illegal while the perimeter is Footprint-legal: replacement loses an existing rejection. **KEEP BaseObstacle; ADD the gate.**

- Freeze baseline `nav2.yaml` and source diagnostic `nav2_smac2d_diag.yaml` byte-for-byte. Their SHA256 values are respectively `d63852ab2eed9c6cc1c258aaf2bdc2b11753ff9d9969e1885785d3cab2a87046` and `97e4eb2bcf58faa3f3a19728b4d17437b9dfe615d07964902e24bb5057a98e9e`.
- Preserve SmacPlanner2D, the absent `cost_travel_multiplier` key (installed default **1.0**, do not newly set it), and all planner parameters.
- Preserve BaseObstacle.scale **0.02**, every old critic name/order/scale (including implicit Oscillation.scale=1), and defaults such as `sum_scores=false`.
- Preserve controller20Hz, failure_tolerance0.3, all vx/vy/wz/speed/sample/acceleration/deceleration settings, reverse vx and disabled vy, both costmaps, footprint/padding/inflation, smoother, SLAM, RL policy and adapter.
- No DWB source change, weight tuning, debug subscription, interface change, autonomous warehouse experiment or success claim from one passage. No `set -e`, `set -euo pipefail`, `exit`, or `pkill -f` execution. Existing launch wrappers contain automatic-abort options; user-run commands below bypass those wrappers without modifying them.

### A. Versioned and installed semantics: evidence before configuration

Installed packages `ros-humble-dwb-core` and `ros-humble-dwb-critics` report **1.1.20**. Official sources are pinned to tag1.1.20, not rolling. Local copies and paths below were read; use `nl -ba` for exact line references when preparing the implementation report. Installed constants in `/opt/ros/humble/include/nav2_costmap_2d/nav2_costmap_2d/cost_values.hpp` are `INSCRIBED_INFLATED_OBSTACLE=253`, `LETHAL_OBSTACLE=254`, `NO_INFORMATION=255`, `MAX_NON_OBSTACLE=252`.

1. [Official BaseObstacle source](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/base_obstacle.cpp), local `/tmp/b2w-light-causal-review/official/base_obstacle.cpp`, `scorePose` / `isValidCost`:

   ```cpp
   unsigned char cost = costmap_->getCost(cell_x, cell_y);
   if (!isValidCost(cost)) {
     throw dwb_core::
           IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
   }
   // isValidCost:
   return cost != nav2_costmap_2d::LETHAL_OBSTACLE &&
          cost != nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
          cost != nav2_costmap_2d::NO_INFORMATION;
   ```

   Why: every rollout pose's **center cell** is checked;253/254/255 and off-grid centers throw. Base inherits `TrajectoryCritic::getScale()` returning `scale_`, without resolution normalization.

2. [Official ObstacleFootprint source](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp), local `/tmp/b2w-light-causal-review/official/obstacle_footprint.cpp`, `getOrientedFootprint`, `scorePose`, `lineCost`, `pointCost`:

   ```cpp
   new_pt.x = pose.x + footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th;
   new_pt.y = pose.y + footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th;
   ```

   ```cpp
   line_cost = lineCost(x0, x1, y0, y1);
   footprint_cost = std::max(line_cost, footprint_cost);
   ```

   ```cpp
   if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
     throw dwb_core::
           IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
   } else if (cost == nav2_costmap_2d::NO_INFORMATION) {
     throw dwb_core::
           IllegalTrajectoryException(name_, "Trajectory Hits Unknown Region.");
   }
   return cost;
   ```

   Why: `prepare()` obtains the padded `getRobotFootprint()`. `scorePose` rotates/translates it and checks rasterized edges, including the closing edge; `lineCost` uses LineIterator.254/255 throw;253 is accepted and scored. Center is checked for map bounds, not center cost. No interior fill or 3D moving-leg collision envelope is checked. Maximum legal pose raw cost is **253**. Both critics inherit BaseObstacle `scoreTrajectory`: `double pose_score = scorePose(traj.poses[i]); score = static_cast<double>(sum_scores_) * score + pose_score;`. `sum_scores` defaults false, so all sampled poses are checked and the returned legal trajectory score is the **last pose** cost, not the sum or maximum over poses.

3. [Official DWB source](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp), local `/tmp/b2w-warehouse-diagnosis/official/dwb_local_planner.cpp`, `scoreTrajectory`:

   ```cpp
   cs.scale = critic->getScale();
   if (cs.scale == 0.0) {
     score.scores.push_back(cs);
     continue;
   }
   double critic_score = critic->scoreTrajectory(traj);
   ```

   Why: zero scale skips scoring and every scoring exception. **scale=0 is not a hard-only gate.** Preserve `short_circuit_trajectory_evaluation=true`: earlier rejection or a partial score exceeding the current best may prevent later critics running. Inserting immediately after BaseObstacle retains existing cheap center rejection and puts the added check before alignment/distance scoring. A winning, fully competitive trajectory must reach the gate; do not claim the critic executes on every discarded candidate.

4. Installed `/opt/ros/humble/include/dwb_critics/obstacle_footprint.hpp`, matching [official header1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/include/dwb_critics/obstacle_footprint.hpp):

   ```cpp
   double getScale() const override {return costmap_->getResolution() * scale_;}
   ```

   Why: configured scale is multiplied by actual local resolution, so copying BaseObstacle's numeric scale does not preserve its effective scale.

5. Installed XML `/opt/ros/humble/share/dwb_critics/default_critics.xml`:

   ```xml
   <class type="dwb_critics::ObstacleFootprintCritic" base_class_type="dwb_core::TrajectoryCritic">
   ```

   `DWBLocalPlanner::resolveCriticClassName` appends `Critic` when absent and searches default namespace `dwb_critics`; `loadCritics` uses `createUniqueInstance`. YAML shorthand **ObstacleFootprint** resolves to this registered type. Retain the existing shorthand convention; do not invent a slash-form plugin name or add an unnecessary `.class` override.

### B/C. Exact implemented delta and numerical soft-score bound

Created `course_navigation/config/nav2_smac2d_footprint_gate_diag.yaml` from current Smac diagnostic bytes. Exact behavior delta under `controller_server.ros__parameters.FollowPath`:

```diff
       - BaseObstacle
+      - ObstacleFootprint
       - GoalAlign
 ...
       BaseObstacle.scale: 0.02
+      ObstacleFootprint.scale: 0.000001
```

No `ObstacleFootprint.sum_scores` override: installed default false is required by the bound. No other YAML key/value/order change. A YAML comment may explain the diagnostic-only weight, but regression should prefer the exact two-line insertion below to make the allowed change auditable.

Use actual configured local resolution `r=0.05 m`, Footprint maximum legal raw score `Fmax=253`, Base maximum legal raw score `Bmax=252`, and existing Base scale `b=0.02`. Choose a documented threshold **epsilon=1e-5** of the maximum Base legal contribution (0.001%, a diagnostic confound budget, not a production tuning target):

```text
Base weighted legal maximum = 252 * 0.02 = 5.04
Footprint weighted legal maximum = 253 * r * s
Require (253 * r * s) / 5.04 <= 1e-5
s <= (1e-5 * 5.04) / (253 * 0.05) = 3.984189723320158e-6
Choose s = 1e-6 > 0, below that ceiling.
Effective Footprint scale = 0.05 * 1e-6 = 5e-8
Footprint weighted legal maximum = 253 * 5e-8 = 1.265e-5
Ratio <= 1.265e-5 / 5.04 = 2.509920634920635e-6
      = 0.0002509920634920635% of Base's legal maximum.
```

The saved raw-map metadata uses float32 `.05000000074505806`; substituting that resolution yields ratio about `2.50992067232e-6`, still below epsilon. Nav2 uses its costmap resolution double; compute the regression bound from the YAML resolution and verify the recorded runtime resolution for a user trial. This bound is absolute over a legal trajectory under default `sum_scores=false`, not relative to its actual Base score (which may be0). It cannot guarantee unchanged ordering for ties or score margins≤1.265e-5, nor eliminate short-circuit effects. The purpose is to execute `IllegalTrajectoryException` legality semantics with negligible bounded additional legal score; do not call this a tuned production weight or mathematically pure hard-only gate.

### D. Implemented regression acceptance

**Files:** new YAML and `tests/test_footprint_gate_config.py` implemented; diagnosis and this root plan updated. No modifications to launch/observer, old configs, installed plugins or DWB source. Test consumes current baseline/source bytes and candidate YAML; it produces proof that removing the two insertions exactly recovers the source diagnostic.

- [x] Capture hashes/branch/HEAD and protected-file inventory before execution; reject an unexpected source hash instead of adapting the test to new unrelated config changes.
- [x] Added the approved pytest file first: RED confirmed the missing candidate. Implemented regression code:

```python
from copy import deepcopy
from hashlib import sha256
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'course_navigation/config'
SOURCE = CONFIG / 'nav2_smac2d_diag.yaml'
CANDIDATE = CONFIG / 'nav2_smac2d_footprint_gate_diag.yaml'
HASHES = {
    'nav2.yaml': 'd63852ab2eed9c6cc1c258aaf2bdc2b11753ff9d9969e1885785d3cab2a87046',
    'nav2_smac2d_diag.yaml': '97e4eb2bcf58faa3f3a19728b4d17437b9dfe615d07964902e24bb5057a98e9e',
}

def load(path):
    return yaml.safe_load(path.read_bytes())

def follow(doc):
    return doc['controller_server']['ros__parameters']['FollowPath']

@pytest.mark.parametrize('name,expected', HASHES.items())
def test_protected_config_hash(name, expected):
    assert sha256((CONFIG / name).read_bytes()).hexdigest() == expected

def test_exact_two_line_insertion_and_all_other_settings():
    original = SOURCE.read_bytes()
    assert original.count(b'      - BaseObstacle\n') == 1
    assert original.count(b'      BaseObstacle.scale: 0.02\n') == 1
    expected = original.replace(
        b'      - BaseObstacle\n',
        b'      - BaseObstacle\n      - ObstacleFootprint\n', 1).replace(
        b'      BaseObstacle.scale: 0.02\n',
        b'      BaseObstacle.scale: 0.02\n      ObstacleFootprint.scale: 0.000001\n', 1)
    assert CANDIDATE.read_bytes() == expected
    source, candidate = load(SOURCE), load(CANDIDATE)
    before, after = follow(source), follow(candidate)
    assert after['critics'].count('BaseObstacle') == 1
    assert after['critics'].count('ObstacleFootprint') == 1
    idx = before['critics'].index('BaseObstacle') + 1
    assert after['critics'] == before['critics'][:idx] + ['ObstacleFootprint'] + before['critics'][idx:]
    restored = deepcopy(candidate)
    follow(restored)['critics'].remove('ObstacleFootprint')
    assert follow(restored).pop('ObstacleFootprint.scale') == 1e-6
    assert restored == source
    # Exact bytes plus restored-tree equality preserve all old scales/defaults,
    # vx/vy/reverse/accel/decel/frequency/tolerance, both costmaps and smoother.
    for key in ('local_costmap', 'global_costmap', 'planner_server', 'velocity_smoother'):
        assert candidate[key] == source[key]
    planner = source['planner_server']['ros__parameters']['GridBased']
    assert planner['plugin'] == 'nav2_smac_planner/SmacPlanner2D'
    assert 'cost_travel_multiplier' not in planner  # Keep installed default1.0.

def test_legal_score_bound():
    doc = load(CANDIDATE)
    params = follow(doc)
    resolution = doc['local_costmap']['local_costmap']['ros__parameters']['resolution']
    scale = params['ObstacleFootprint.scale']
    assert resolution == 0.05
    assert params['BaseObstacle.scale'] == 0.02
    assert not params.get('BaseObstacle.sum_scores', False)
    assert not params.get('ObstacleFootprint.sum_scores', False)
    assert math.isfinite(scale) and 0 < scale <= (1e-5 * 252 * 0.02) / (253 * resolution)
    contribution = resolution * scale * 253
    assert contribution == pytest.approx(1.265e-5)
    assert contribution / (252 * params['BaseObstacle.scale']) <= 1e-5

def test_installed_plugin_xml_resolution():
    from ament_index_python.packages import get_package_share_directory
    xml = Path(get_package_share_directory('dwb_critics')) / 'default_critics.xml'
    tree = ET.parse(xml)
    classes = tree.findall('.//class')
    matches = [c for c in classes if c.attrib['type'] == 'dwb_critics::ObstacleFootprintCritic']
    assert len(matches) == 1
    assert matches[0].attrib['base_class_type'] == 'dwb_core::TrajectoryCritic'
    assert tree.find('.//library').attrib['path'] == 'dwb_critics'
```

- [x] Copy current source bytes and perform exactly those two insertions; rerun to GREEN. YAML parsing must succeed for all three files. The exact-byte test rules out extra settings, duplicate keys and unintended comment/format rewrites. Whole-tree equality includes footprint, padding and inflation; inventory hashes separately preserve SLAM/policy/adapter outside this YAML.
- [x] Resolve/create the installed plugin through **pluginlib**, beyond XML parsing: temporary C++ probe under `/tmp`, `pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader("dwb_core", "dwb_core::TrajectoryCritic");`, assert `loader.isClassAvailable("dwb_critics::ObstacleFootprintCritic")`, call `loader.createUniqueInstance` for it and BaseObstacle, destroy objects before loader. Link with ament dependencies `dwb_core` and `pluginlib`. Do not call rclcpp init, initialize/configure/activate plugins or create nodes/DDS. Save class/type/library path and result; missing plugin is a validation failure, not a skip.
- [x] Preserve installed-library semantic tests, including center253/254/255 rejection, perimeter253 acceptance/254/255 rejection, interior-only lethal/unknown acceptance, closing edge, yaw and off-grid. These test legality, not navigation success.

Exact allowed unit-test command at execution time (the new test file must exist first):

```zsh
cd /home/zyd/program/b2w-embodied-navigation-course
source /opt/ros/humble/setup.zsh
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -p no:cacheprovider \
  tests/test_footprint_gate_config.py tests/test_frozen_obstacle_critic.py \
  tests/test_warehouse_observer.py -q
git diff --check
sha256sum course_navigation/config/nav2.yaml course_navigation/config/nav2_smac2d_diag.yaml
```

Only config/unit, YAML parsing, plugin/static checks. No `scripts/test_course.sh`, runtime smoke, replay of navigation, long warehouse test, startup or goal dispatch by the agent. Capture exit codes explicitly; do not use automatic shell abort/exit. Existing tests compile their tiny offline harness under `/tmp`.

### E. Exact user-run commands and protocol (after candidate exists and static checks pass)

Commands below are **prepared for the user**, not executed by the agent. Read guide sections3–4 first; use an already prepared warehouse/install/policy. Terminate any previous same-domain session normally in its owning terminal before starting a fresh trial. GUI+RViz are required. Direct launch uses existing `nav_params_file` support; no wrapper/source change is needed.

Terminal1 (zsh):

```zsh
cd /home/zyd/program/b2w-embodied-navigation-course
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="b2w-warehouse-footprint-gate-$ROS_DOMAIN_ID-$$"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/course_sim/config/fastdds_local.xml"
export PYTHONPATH="$PWD/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$PWD/.deps/lib:$PWD/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=2
/usr/bin/python3 /opt/ros/humble/bin/ros2 launch course_bringup course.launch.py \
  mode:=navigation scene:=warehouse gui:=true rviz:=true \
  nav_params_file:="$PWD/course_navigation/config/nav2_smac2d_footprint_gate_diag.yaml"
```

Wait for SLAM/TF/map and Nav2 readiness, not an arbitrary sleep. Terminal1 must show `Using critic "ObstacleFootprint" (dwb_critics::ObstacleFootprintCritic)` and retain BaseObstacle. Runtime snapshots from the observer must confirm Smac2D, unchanged defaults/limits, scale and local resolution.

Terminal2 (zsh; same domain/DDS settings):

```zsh
cd /home/zyd/program/b2w-embodied-navigation-course
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/course_sim/config/fastdds_local.xml"
export PYTHONPATH="$PWD/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$PWD/.deps/lib:$PWD/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=2
/usr/bin/python3 scripts/observe_warehouse_narrow_failure.py \
  --label smac2d-footprint-gate-light --duration 0 \
  --output-root /tmp/b2w-warehouse-observer
```

Do **not** add `--dwb-debug`. Existing LIGHT observer only attaches; Ctrl+C there stops only observation. Manually use approximately the same narrow-aisle RViz map goal as the captured route; do not substitute Gazebo world coordinates.

1. Test **one outbound traversal** first. If it succeeds, return through the **same aisle**. At most **three round trips**; stop repetitions at the first failure. Record each direction/action outcome; one traversal is insufficient to claim improvement.
2. If it stalls, leave target/config/map/session untouched for **at least25 simulation seconds**, measured from `/clock`, before Ctrl+C **only the observer**. Do not send a replacement goal, cancel, retune, pause/reset or close the GUI in that window.
3. Record entry refused / aisle passed / aisle stall. Under LIGHT, refusal is an observed outcome; without critic score/rejection telemetry, do not assert that ObstacleFootprint was the rejecting critic. Record runtime plugin/scale confirmation separately.
4. Record controller20Hz timing/missed-rate warnings and active navigation command-receipt intervals, simulation versus wall duration, TF/signal freshness and observer health. Whole-attach receipt Hz includes idle and is not a measured controller loop rate. Existing unstamped Twist receipts cannot prove computation latency; absence of a warning cannot certify meeting every50ms deadline.
5. If all three round trips succeed, report only success on this bounded user trial / original intermittent failure not reproduced. Compare timing and route/map context; do not certify physical clearance or promote a baseline.

### F. Decision boundary: argue before recommending

**AGAINST:** the added footprint gate may make this aisle or its entry turns infeasible; frozen mismatches do not prove a valid alternative rollout. First95.035 failure precedes saved mismatches, so the first rollout rejection remains unexplained. Padded perimeter legality does not fill the interior or measure the moving-leg physical envelope. Extra rasterization may hurt controller timing. Tiny positive scoring can affect near-ties; retaining early critics/short-circuit evaluation prevents claims of universal gate execution on all discarded candidates.

**FOR:** exact same-map installed-library replay demonstrates four center/perimeter disagreements. Keeping BaseObstacle preserves center253 rejection and all other existing legality predicates; replacement would lose it. Adding the gate directly tests the demonstrated rectangular-perimeter approximation gap with a bounded soft-score confound and unchanged planner, motion limits, geometry, mapping and tracking chain.

**VERDICT:** recommend only an isolated **diagnostic single-variable experiment**, not a new baseline, production weight, root-cause proof or physical-safety claim. Reverse/abandon this diagnostic if the aisle becomes untraversable, controller timing degrades, the same failure remains, or evidence shows another critic, tracking or map/TF mechanism dominates. Return to original Smac diagnostic for comparison; do not compensate by changing vy, reverse vx, planner, cost_travel_multiplier, inflation, footprint, RL or SLAM in the same trial.

### G. Implementation delivery

- [x] Verified current branch/local/live remote HEAD before edits; captured 283 tracked file hashes excluding inherited `PLAN.md`, plus LFS/index snapshots.
- [x] Implemented candidate and approved regression with RED/GREEN; preserved frozen installed-library semantic tests. 53 tests pass. Both critics instantiate through installed pluginlib without nodes/DDS.
- [x] Updated diagnosis and sole root plan; kept exact commands in section E for the user only. No user experiment or live result.
- [x] Final checks passed: all three YAMLs parse; Python compiles in memory; protected baseline/source hashes unchanged; all 283 inventoried tracked files except the authorized diagnosis match their starting hashes. All four video content SHA256/size match HEAD LFS pointers; index pointers and `.gitattributes` unchanged. Diff/whitespace audit passed. Evidence: `/tmp/b2w-footprint-gate-implementation/static-integrity.json`.
- [ ] Final operation: run controlled `april-review-bundle` copy (remove only forbidden automatic-abort/exit lines) into `/home/zyd/APRIL_REVIEW_BUNDLES`; verify absolute path/SHA256, branch/HEAD, bundled diff and no video changes, then STOP. No post-bundle edits, commit or push. Actual bundle path/hash are returned in the final report.

Historical plan-only delivery completed as bundle `155445`, verified SHA256 above. Its pending implementation language is superseded by this NOW section.

## 历史已完成 — LIGHT failure frozen causal diagnosis (2026-10-14)

User A–G is the authorized bounded design. Apply debugging, brainstorming, writing-plans, TDD and verification; execute inline in this existing workspace, preserve inherited observer/doc/video edits. No extra plan, no implementation agents, no commit/push. One read-only reviewer is dispatched under the requesting-code-review skill. HEAD 0c31f08e19ce51849f2c06b107e8a89b2ff55ef7. Required original input exists: `/tmp/b2w-warehouse-observer/20261014T065029.802588Z-smac2d-light-smoke`; never modify it. Evidence output `/tmp/b2w-light-causal-review`.

- [x] A/B: audit mode/timing/freshness and active-interval command receipts; extract source-linked action/log/command/truth chronology with separate wall-header and simulation-receive clocks; verify official 1.1.20 controller semantics.
- [x] C/D: add isolated installed-library C++ critic harness and offline replay; test perimeter-only versus filled geometry, 253/254/255 and boundaries; compare each saved local/global map using recorded exact-stamp pose and already-padded footprint, hashes and skew.
- [x] E: RED timing regression with 95.x failure and 99.47 stationary onset; add explicit six-second, epoch/goal-bounded precursor chronology using original event seq/receipt/header timestamps, last raw receipt and action transitions. Preserve A–E and null first_causal_layer/missing commands; GREEN tests.
- [x] F: document AGAINST/FOR/neutral verdict and reversal conditions, physical-evidence limits; no diagnostic navigation YAML yet.
- [x] G: relevant unit tests, offline replay, compile/static scope checks, original evidence/config integrity; update diagnosis and this plan before bundling.
- [x] Historical final bundle was generated: `20261014_152842`, SHA256 `281b314c00b1e7e62d12696ea8a6561caeb928df23d1e56d1a2b965fdbd12b72`, verified again from its recovered Trash copy in the current planning turn. This is not a pending operation.

Result: pre-failure raw/smooth/final receipts each20.0009Hz over83.640–95.035; no multi-second observer backlog indicated. Four local same-map center-legal/perimeter-illegal snapshots95.3975–97.1975, late inverse legality at97.7925. First rejecting critic and physical contact unresolved. Summary adds bounded precursor without changing A–E.48 relevant tests, offline11 selected/26 sweep maps, C++/Python/static checks pass;2324 evidence files and270 protected tracked files unchanged. Read-only reviewer found no critical/important issues; two minor timestamp/header precision findings corrected. Historical bundle delivery is complete; current work is exclusively the NOW plan above.

## 上一轮已完成 — Observer transparency: LIGHT default and self-health (2026-10-14)

Inherited observer/doc implementation is retained. User A–F authorized LIGHT default causal-core subscriptions, optional `--dwb-debug` (`/evaluation`, `/received_global_plan`, `/transformed_global_plan`, `/local_plan`), pure clock/rate/age/freshness health, and manifest/summary output bytes. Command ages are receive-clock ages, never transport latency; missing/stale causal values stay null. No navigation configuration/interface change or agent-run experiment.

- [x] Read entry/guide/interfaces/observer/tests and official Nav2 Humble 1.1.20 subscriber gates.
- [x] RED 9 new failures /21 prior passes; GREEN/final30 observer tests, compile/help/static PASS. Existing helper and evaluation callback semantics retained.
- [x] Completed user smoke `/tmp/b2w-warehouse-observer/20261014T060802.404851Z-smac2d-smoke`:60.0594 wall seconds,388 samples,no errors/stall,24 expected topics and7 parameter nodes. User reports forward/narrow passage and later return success; no deterministic stall conclusion.
- [x] Whole-attach421 raw/evaluation receipts(~7.01Hz) are not a controller-rate measurement. TF map→base p50/p95/max3.6025/6.9630/8.7375s, odom→base2.8250/4.2945/5.5500s motivated the later user-run LIGHT timing smoke. Backlog/drop hypotheses were not established causality.
- [x] Read-only review had no findings;270 protected tracked files and HEAD unchanged; no live session/process touched. Evidence `/tmp/b2w-observer-light-review/`.
- [x] Historical bundle generation completed per saved bundle.log/verifier: `/home/zyd/APRIL_REVIEW_BUNDLES/APRIL_REVIEW_BUNDLE_b2w-embodied-navigation-course_20261014_144352.md`, SHA256`68b7d4cf899a5086098b74249a0ca556c8b5b67b622d92dadd87a4c6045e5c77`. That old bundle file is currently absent; the saved log/verifier remain. This is not a pending task for this review.

## 上一轮已完成 — 可复用 frozen replay + Smac 小型 sensitivity diagnostic（2026-10-13）

唯一目标两项：泛化CLI但不放松原gate；同一 frozen A/1.70m/seed0/6m做0.0/0.5/1.0/2.0/4.0五次Smac独立fresh replay。source SHA256 `33e13df9eca0acdd072c86af6fedef0bfbe89631095b61d07ade46569771e22b`。证据 `/tmp/b2w-smac-sensitivity/`。不启动Gazebo/live SLAM/NavigateToPose，不做正式sweep，不改baseline YAML/inflation/footprint/DWB/vy/policy，不commit/push。

- [x] 读取AGENTS/CODEX_GUIDE/PLAN/docs与指定212847 bundle（在回收站找到），核对HEAD=9640ca95fce5c0554bbb5343754bf86deb7dbebf；读取接口文件，只有离线观察器变更。
- [x] 先红8失败/13通过，重构source/expected-sha必填CLI、fixture元数据原样输出/width读取、严格完整输入/参数/hash gate。允许只跑Smac和单key/tmp overlay；默认仍fresh独立pair。相关45项unit tests绿。
- [x] 原A pair通过已保存响应的full harness逐项回归；fresh 1.0稳定字段逐项一致且final CSV逐字节一致，NavFn复用旧FAIL不重跑。
- [x] 恰好运行5个短命offline进程；4项严格PASS，0.0路径保存后对象/DSO teardown SIGSEGV。透明保存异常；修正观察器寿命后剩余4次正常。未重跑0.0，没有第6次。
- [ ] **五case严格验收未全部完成**：0.0 exact runtime params / applied PoseStamped读回 / wall time结果未落盘，恢复证据只允许partial，strict_case_verified=false。本轮不追加运行。
- [x] five observed final/unsmoothed routes均narrow，4个有效case均narrow，无wide切换；只得有限“不支持此权重解释本frozen case绕宽路”的结论。0.0 exposure差异为部分证据，非最终选优。核对官方1.1.20默认1.0与Node2D child raw/252边代价公式，不套rolling。
- [x] 5个owned identities均退出、remaining={}，host domain193=[]；配置/policy/HEAD摘要不变。独立记录0.0 returncode=-11，其余0；完成前验证和所有报告在bundle前完成。
- [x] 上轮已生成232440 bundle：运行april-review-bundle受控副本（原set-e/exit与用户禁令冲突，沿用已授权做法），生成后立即停止，不再回写PLAN。实际路径/成功以最终输出为准。

工具重构和1.0回归已完成；sensitivity交付为四个严格结果加一个partial异常case，不能报五case全部通过。详见docs末节与report.md；不从单case挑最佳权重。下一步仅由用户审查bundle后另行授权。

## 上一轮已完成 — Frozen same-input planner replay（2026-10-13）

唯一case：最新 Scenario A / width=1.70 m / seed0 / relative goal=6 m。只复用上一轮 `/tmp/b2w-same-server-planner-ab/smoke/20261013T113843.426811Z/A-w1.7000-seed0/` 保存的 before master raw、JSON、NavFn/Smac2D exact requests；原始raw缺失立即STOP，绝不启动仿真生成替代图。无Gazebo/live SLAM/NavigateToPose、无参数修改、无sweep、无commit/push。唯一证据根目录 `/tmp/b2w-frozen-planner-replay/`。

- [x] 读取AGENTS、CODEX_GUIDE、PLAN、NARROW_BASELINE与上一轮report/bundle.log。最新记录bundle为20261013_194403，但当前目录为空，无法重读；明确保留缺口。
- [x] 校验原始raw SHA256等于上一轮before hash、JSON数据与raw一致、grid size/resolution/origin/frame、两个请求只差planner_id、完整start/goal与use_start=true；保存input-audit.json。
- [x] 复用FrozenCostmap适配器与本机Humble1.1.20 pluginlib，增加完整PoseStamped读入/回读、plugin load/createPlan/异常及生命周期记录。每planner独立fresh process/costmap，从同raw重载，保留原配置与默认参数。
- [x] 先做拒绝缺raw/hash失配/geometry失配/request差异的unit tests，再运行相关tests与仅此一个frozen pair；保存加载前、configure后、createPlan后内存与source raw SHA，完整runtime DEBUG及owned PID/starttime/returncode cleanup。
- [x] 复用path_metrics与route分类，记录floor/实际NavFn round cells、pose count、几何length、createPlan wall time、每raw-cost sample及min/mean/p95/max/unknown/253/254/descriptive exposure；exposure明确非内部objective。
- [x] 按两planner实际结果写有限结论、未验证边界、integrity、tests/cleanup与实验说明；不增max_cycles诊断、不调参。
- [x] 前轮212847 bundle已生成并在本轮从回收站读取；不作为本轮待办。

结果：same_input_verified=true。原before raw SHA256 `33e13df9eca0acdd072c86af6fedef0bfbe89631095b61d07ade46569771e22b`，400×320/res=0.05000000074505806/origin=(-10,-8)/map；exact PoseStamped传入并回读相同。NavFn加载成功、createPlan正常返回空path，日志为PathCalc path too long；Smac2D加载/规划成功，121 poses、narrow、length=6.065064518899199m、descriptive exposure=18.072379085130063cost·m，非内部objective。各source/memory SHA完全一致，floor=(193,159)→(313,159)，NavFn runtime round=(193,160)→(313,160)。这是一份冻结输入上的算法行为差异证据，不泛化整体优劣，不是route-preference对比。29项相关unit tests通过；3个owned进程均returncode0、remaining={}（含保留的首次adapter校验失败NavFn调用），host审计PASS；参数/policy/HEAD摘要未变。证据与边界见docs末节和 `/tmp/b2w-frozen-planner-replay/report.md`。

## 上一轮已完成 — Same-server / same-costmap Scenario-A planner comparison（2026-10-13）

固定 Scenario A / width=1.70 m / seed=0 / relative goal=6 m。只比较 global planner route preference；不发 NavigateToPose、不驱动机器人、不调参、不 sweep、不 commit/push。唯一证据根目录 `/tmp/b2w-same-server-planner-ab/`。

- [x] 读取 AGENTS、PLAN、NARROW_BASELINE、CODEX_GUIDE、接口与现存 review。最新 bundle 20261013_173737 的生成记录仍在，但当前 bundle 目录为空，无法重读；按用户本轮明确 review 结论修正同输入问题。
- [x] 新建 `course_navigation/config/nav2_dual_planner_diag.yaml`：同一 planner_server 加载 NavFn / Smac2D，使用安装 XML 名称；所有非 planner 配置与 baseline 一致。
- [x] 新增 `scripts/narrow_planner_comparison.py` 与 harness 固定单 case 开关。readiness 后固定一次 fresh map→base_link 显式 PoseStamped，两个 use_start=true 请求只改变 planner_id；顺序 NavFn→Smac2D，三个 live master raw hash 及 grid geometry 完全一致才 eligible。
- [x] 单元测试先验证相同输入/请求顺序/hash confound gate/长度与 descriptive raw-cost exposure/固定 case 限制，再实现并运行全部 unit tests。
- [x] 按指南检查环境与 policy，仅运行一次 A/1.70m/seed0/6m planner-only short smoke。保存 plugin 加载日志与 runtime 参数、exact request、status、path、路线、长度、wall roundtrip、raw stats 和 unknown/253/254 样本。
- [x] 若 raw map 变化，报告 confound 并停止实验，不冻结地图、不改 update_frequency、不重试。保存 owned token+PID identity cleanup 与完整性证据。
- [x] 文件路径、关键片段和结论关联已写入实验说明与证据 report；所有交付材料在 bundle 前完成。
- [x] 上轮bundle已生成（bundle.log记录20261013_194403）；本轮该文件缺失，保留审查输入缺口，不再作为当前待办。

结果：dual planner加载成功，同一planner_server PID3315652/starttime34145262与lifecycle前后[3,3,3]；两个请求仅planner_id不同。NavFn ABORTED、空path、chosen_route=unclassified；Smac2D SUCCEEDED、chosen_route=narrow、length=6.065064519m、descriptive raw exposure=18.072379085cost·m（统一before参考图，非内部objective）。before/between raw hash=33e13df9...，after=b123085d...，316/128000 cells变化，**comparison_eligible=false**；按授权边界停止实验，没有重试/冻结map/调update_frequency。42项unit tests通过，23个owned identities已退出、remaining={}、domain192锁释放、host audit PASS；原baseline/policy/SLAM/launch/HEAD摘要不变。当前HEAD=b9acbd55b717388f2425a0bc3f6ee5323ebf32d1。

## 上一轮已完成 — 固定case Online planner A/B 与Smac导航（2026-10-13）

固定Scenario B / width=1.70m / seed0 / relative goal=6m；本轮无正式sweep、无调参、无policy改动、无commit/push。证据根目录 `/tmp/b2w-online-planner-ab/`，完整报告见 `docs/NARROW_BASELINE.md` 末节及证据report.md。

- [x] 已读AGENTS/PLAN/NARROW_BASELINE/CODEX_GUIDE及接口；指定20261013_165953 review bundle不存在，记录审查输入缺口。
- [x] harness扩展参数文件选择、固定单case GUI导航、明确观察完成才清理、runtime dump、BT/action/status/三段cmd/定位/plate完整观测；CLI和观察gate红绿测试。环境ready，rl_sar bundle校验通过。
- [x] NavFn/domain187独立启动，lifecycle=[3,3,3]及fresh SLAM/map/TF后读取live master；一次ComputePath SUCCEEDED，path=6.154882m，roundtrip=9.383127ms；没有NavigateToPose，cleanup PASS。
- [x] Smac/domain188同流程；actual loaded plugin已确认，一次ComputePath SUCCEEDED，path=6.053571m，roundtrip=10.845375ms；没有NavigateToPose，cleanup PASS。runtime cost_travel_multiplier=1.0为默认值，未改动；非planner节点参数与NavFn一致。
- [x] 两个独立raw hash不同，527/128000 cells变化，choke中心40/1360变化；各自before/after不变，中心线unknown/253/254均0，max cost151。**未复现NavFn失败；输入差异构成直接算法对比confound，不作planner优劣排名。** 对旧冻结图分别1221/964 cells变化，不能推翻原离线trap证据。
- [x] 仅Smac在线成功后进入domain189 GUI/RViz单次6m导航；先一次preplan安全关卡，然后一次NavigateToPose SUCCEEDED。SLAM path=5.846329m，time=13.9425仿真秒，fresh map error=0.238153m，plate计数0（1395样本），truth entered/passed均true。14条global plans，首条后13次重规划；feedback recovery=0；TF age0.305..0.410s，三段vy=0。旧FollowPath observed ABORTED不作primary failure，最终FollowPath成功。
- [x] 目标结束保留GUI。用户已回复“观察完成”：正常进入/通过choke/到达，运动平稳、轻微减速、转向谨慎、未观察碰撞、plan合理、RobotModel/map基本对齐。完整原评价在human-observation.json；没有擅自补独立停车评分。
- [x] 确认后SIGINT own launch，22/22/26个owned identities均已退出，remaining={}，launch exit0。host复核窗口与domain后台进程均消失，域187/188/189锁均可非阻塞重新获取；锁文件保留旧PID文本但不持锁。不清理其他会话。
- [x] 原baseline/Smac YAML/policy/SLAM/既有launch观测配置摘要不变；34项pytest及compile/shell/diff checks通过。报告、原始证据、cleanup、完整性与manifest备齐。

**在B/1.70m/seed0/6m这一固定online case中，SmacPlanner2D完成规划并使导航成功通过。** 不泛化到Smac整体优势、最优planner或最小通行宽度。本轮NavFn在线规划也成功；不把不同地图path length/roundtrip当公平性能benchmark。

当前HEAD=193170687179a8e86d096c86eb44a632aa971fc9；reflog最新提交为15:21:22+08:00，早于本轮17:10启动。此前66db515...只是历史审计基准。host审计首次旧HEAD硬编码检查失败已保留，实际cleanup均通过；更新为reflog时间证据后完整审计PASS。

唯一剩余交付动作：调用用户指定 `~/bin/april-review-bundle` 后立即停止，不commit/push，不进入新实验。生成是否成功及实际路径以最终工具输出和回复为准，不在bundle后回写本PLAN；下一轮应审查本轮bundle，后续范围须由用户另行指定。

## 上一轮已完成 — NavFn budget反证与Smac2D冻结快照诊断（2026-10-13）

本轮用户仅授权离线诊断：不启动Gazebo、不修改baseline、不发NavigateToPose、不跑正式sweep、不调任何navigation参数。先复用6m installed-library replay，仅变calcPath max_cycles；16000仍失败后新增独立Smac2D诊断配置并复用已有四距离raw snapshots。用户明确授权的单plugin诊断优先于下方历史milestone的“不替换”限制，不改变默认导航插件。证据根目录 `/tmp/b2w-navfn-budget-diagnostic/`。

- [x] 已读AGENTS/本PLAN/NARROW_BASELINE/CODEX_GUIDE和原replay。最新bundle记录路径为旧bundle.log中的20261013_164150，但文件不存在且目录为空，无法重读；此缺口明确保留。
- [x] 1600/3200/6400/16000均FAIL，随后追加32000仍FAIL。返回steps全0，npath分别耗尽各budget；第105步后持续困于cell(262,162)，unique cells=54、min距原start cell=3.437555m，exact fallback和zero-gradient均0。各次potential、原1600及相邻预算path prefix逐字节一致。结论仅为“增加 extraction cycle budget 不能解决此case”。
- [x] 网页/官方当前源码确认rolling已有max_cycles_factor默认4，本机Humble1.1.20硬编码4；只增factor不获本case证据支持。没有把rolling参数写入Humble配置。
- [x] ros2 pkg executables/pkg prefix、apt包和installed XML确认Smac plugin名 `nav2_smac_planner/SmacPlanner2D`。新增 `course_navigation/config/nav2_smac2d_diag.yaml`，全文只有plugin一行与baseline不同；其余配置、SLAM和policy不变。
- [x] Smac2D installed plugin createPlan离线1/3/4.5/6m均SUCCESS，path length分别1.000000/3.056684/4.557671/6.065065m。保存raw、path、plugin status、latency、cells/counts、parameters和logs；raw输入输出一致。没有运行action server，action_status=null，不能称live ComputePathToPose结果。
- [x] 说明同世界pose下NavFn round与Smac floor的cell差异；追加6m原NavFn同cells control也SUCCESS、约6.044869m。只得出“SmacPlanner2D在该固定case成功生成path，而NavFn失败”，不宣称整体更优。
- [x] Primary四距离+control均owned PID正常退出，remaining={}；此前sandbox DDS受限试调用也清理完毕，primary使用隔离host DDS环境复核。完整性、tests与最终cleanup证据在bundle调用前保存。

唯一剩余收尾动作：生成用户指定april review bundle后立即停止，不commit/push；实际路径以工具输出及最终回复为准，不回写本PLAN。正式planner benchmark尚未开始/未宣称就绪；先审查本bundle，再决定独立在线planner验证与正式对比范围。未启动cost_travel_multiplier/inflation/DWB/vy/实际通道导航/A-B sweep。

## 上一轮已完成 — Harness review 修复与固定 NavFn extraction 诊断（2026-10-13）

本轮按最新 review 修复 harness shell 自动中止与 controller abort 误归因，仅诊断 B / 1.70 m / seed0 的 1 / 3 / 4.5 / 6 m ComputePathToPose；每个距离独立短测并清理 owned processes，不发送 NavigateToPose。baseline 参数与 policy 完全不变。只加观测：planner/rclcpp DEBUG、实际 raw master costmap 前后快照、floor/round cell 与中心线代价分布。不调参、不跑 42-case sweep、不 commit、不 push；最终生成 april review bundle 后停止。

- [x] 读取项目入口、现行实验说明与 review 要求。`~/APRIL_REVIEW_BUNDLES` 本轮读取为空，上一轮 bundle 文件无法重读；现存 `/tmp/b2w-narrow-characterization/review.md` 已读。上一轮 bundle 动作按用户本轮明确反馈已完成，不再保留为待办。
- [x] 修复 wrapper 的 set-e 语义与 observed ABORTED 的 primary failure 归因；32项unit tests PASS，shell/compile/diff checks PASS。
- [x] 核对已安装 Humble 1.1.20 与官方源码的 propagation / extraction 两阶段；live DEBUG确认6m的 calcPath返回0为1600-step cycle exhaustion（path too long）。
- [x] 四个固定距离独立planner-only诊断已完成：1/3/4.5m规划成功（1.007098/3.181549/5.110684m），6m ABORTED。实际raw costmap、cells、counts、status、length、latency、DEBUG保存在 `/tmp/b2w-navfn-diagnostic/`。未发送导航目标。
- [x] 安装库离线回放确认6m extraction第105步起困于cell(262,162)内，直到1600步耗尽；相对起点约3.47m，choke内部。归为本代价场下NavFn extraction限制，不是253/254/unknown几何断路；尚未证明cost field驻点的生成原因或独立上游bug。源码片段、实测、回放边界见 `docs/NARROW_BASELINE.md`。4次各22个owned identities cleanup PASS、remaining={}；baseline与HEAD/所有manifest一致，policy摘要一致。
- [x] 最终bundle输入与诊断报告已准备；所有代码、测试、文档在调用前完成。

本轮唯一收尾动作：调用用户指定 `~/bin/april-review-bundle` 后立即停止；是否完成及实际路径以命令输出和本轮最终回复为准，bundle生成后不再回写本文件。下一轮状态为审查本诊断bundle，不将bundle调用保留为跨轮待办，不进入正式A/B sweep或参数调优。

以下为上一轮 characterization 已完成记录与证据边界，非另一 roadmap。

用户本机 warehouse baseline 已完整复现 PASS（用户提供）：NavigateToPose SUCCEEDED；collision=0；path=1.557 m；time=5.085 s；final error=0.226 m；TF/nav/runtime 正常。视觉评价为平稳、少量停顿、正常转弯、地图基本对齐、无碰撞、到达。该结果覆盖旧 freshness 未通过的历史状态，但不证明窄路边界。

- [x] 读取 AGENTS / CODEX_GUIDE / INTERFACE / INTERFACE_ALIGNMENT、现行 Nav2 YAML、map projector、warehouse/world generator 和启动链。
- [x] 静态分析配置与对应 Humble 1.1.20 上游语义；路径、片段、推导见 `docs/NARROW_BASELINE.md`（实验说明，不是 roadmap）。
- [x] 设计 A/B：复用 confined 的 Fortress 系统、floor 与 box 模型，通过现有 world override 启动全部原有模块。每 case 独立重启，同 start=(0,0)、相对 goal=(6,0)，2 m 长 choke；A 上方存在 1.8 m 宽绕行，B 封死绕行。
- [x] 宽度由实际 YAML 自动推导：padding 后 1.50×0.86 m，resolution=0.05 m；默认 0.80/0.85/0.90/1.00/1.20/1.60/1.70 m，覆盖内切阈值两侧、栅格余量与 inflation 叠加边界。不是安全宽度或已验证通过阈值。
- [x] 实现最小 harness：fixture/manifest，独立预规划再 NavigateToPose，map/raw costmap/preplan/BT paths、odom/三段 cmd/plate/rosout、结果和失败分类，唯一 token + PID starttime 清理；baseline/policy SHA256。
- [x] 有意义的几何、路线分类和故障分类 tests；初次缺模块红测后实现转绿。正式全套最终 29 tests PASS（含真实Humble日志级别回归、进程归属清理与unowned保留）。
- [x] 独立 B/1.70m/seed0 6m目标预规划短测：map/costmap/SLAM TF/三Nav2 active就绪后，中心线121样本unknown=0/blocked=0；NavFn合法potential后path extraction失败，未发导航目标，不归为controller失败。追加明确标注的入口1m harness smoke：SUCCEEDED、collision=0、path=0.775433m、time=2.6575s、error=0.230703m、planning roundtrip=4.293773ms；未进入/通过corridor。Codex未跑宽度sweep。
- [x] 核查指标、baseline完整性与清理：全部已启动短测会话remaining={}；最终smoke23个owned identities，三段vy=0；29 tests PASS，diff-check PASS。提供正式用户命令见实验说明。独立review三项P2已修正，复核无blocker。后续新增raw controller status日志格式仅做编译/单测检查，未重跑真实仿真。
- [x] 上一轮 bundle 动作已完成（本轮用户确认）；当前 bundle 目录为空，路径不可重读。

测量边界：保持 online 地图与 SLAM 定位；真值不进入 Nav2。A 保存两条几何候选中心线在实际 costmap 中的 unknown/blocked 样本，存在这些干扰时禁止归因为“planner 过度保守”。预规划与 BT 重规划分别保存，延迟仅为 pre-navigation action 墙钟往返。plate 碰撞反馈不是全身覆盖，缺数据为 null；minimum clearance 无可靠全身数据时为 null。重复 normal seeds 仅控制传感器噪声 RNG，不宣称 physics deterministic。B 仅规划成功、实际进入且完成的 case 能支持 controller/locomotion 通过能力；规划失败不能算作 controller 极限。

实现文件：`scripts/narrow_fixture.py`、`scripts/narrow_processes.py`、`scripts/run_narrow_baseline.py`、`scripts/run_narrow_baseline.sh`、`tests/test_narrow_benchmark.py`。无 baseline/config/公共接口/运控源码修改。证据目录 `/tmp/b2w-narrow-characterization/`。

## 历史 deferred milestones（当前以 NOW footprint-gate 计划为准）

**Milestone 2：独立配置中的最小改动实验。** 前提为 M1 完成、bundle 审查通过、失败证据明确。先验证 lateral 命令和实际 tracking，再考虑 `course_navigation/config/nav2_narrow_v1.yaml`；baseline 文件保持原样。只依据有效命令范围、adapter 限制、Nav2 语义和短测 tracking 选择值，不把 C++ clamp 当作物理安全速度证明。随后才按证据依次评估 padding/inflation、unknown。第一轮不替换 NavFn 或 DWB。

**Milestone 3：用户本地正式实验与证据复核。** Agent 只交付可复制命令；用户运行 baseline/improved、多 seed 和 wide/medium/narrow。比较失败分类与全部指标，存在碰撞或定位异常的 case 不能通过剔除来美化成功率。每次实质修改都重新打包并停止扩展。

## 执行环境与本轮检查结果

以下表为此前只读诊断的历史结果；当前环境恢复验收见上方 NOW 记录。

日志目录：`/tmp/b2w-narrow-diagnosis-ls7ys2r7/`。

| 检查 | 实际结果 | 证据与边界 |
| --- | --- | --- |
| OS / Python | Ubuntu 22.04.5 / Python 3.10.12 | 当前执行环境，非历史验证机器。 |
| 运行前提 | FAIL | `preflight.json`：`/opt/ros/humble/setup.bash`、`install/setup.bash`、LIO-SAM/GTSAM、`.deps/lib`、rl_sar ONNX/URDF 均不存在；`ros2`/`ign`/`colcon` 不在 PATH。 |
| 项目环境检查器 | FAIL，return code 1 | `environment-check.log`：`scripts/check_course_environment.py:16` 导入 `ament_index_python` 时 `ModuleNotFoundError`。不是导航失败。 |
| Python 运行依赖 | 不齐 | 缺 rclpy、ament、numpy、scipy、pytest；yaml 可用。 |
| 纯逻辑诊断 | 5/5 PASS | `static-probes.json`：从实际源码 AST 提取 VelocityGate，验证正负 vy 保留、向量/角速度限幅、过期归零；另核对 baseline YAML 的全部 y 限制为零。未运行 ROS 节点、策略或 Nav2。 |
| GPU 查询 | unavailable | `static-cleanup.json`：nvidia-smi 输出无法连接 NVIDIA driver；即使返回码为 0，也不能据此认定 GPU 可用。 |
| 实际 smoke / 目标到达 | NOT RUN | 没有启动仿真，没有发送 goal；不能宣称恢复或验证了运行链路。 |
| cleanup | PASS（本次启动进程范围） | `preflight.json` 前后进程快照无新增残留；`static-cleanup.json` 无相关后台进程。仅创建同步检查子进程，未创建 GPU workload，未发送 kill 信号。可见性仅限当前 PID namespace，未声称全宿主机或其他项目无进程。 |

## 链路证据（源码已核对，运行尚未恢复）

| 边界 | 文件与关键片段 | 证明内容 |
| --- | --- | --- |
| 仿真 LiDAR / IMU → 公共传感器 | `course_sim/config/bridge_rl_sar.yaml`：`/sim/raw/pointcloud`、`/sim/raw/imu`；`course_sim/course_sim/sensor_disturbance.py:30`：publish `/pointcloud`、`/imu` | bridge + 扰动适配产生公共输入，normal 仍含小噪声。 |
| 公共传感器 → LIO-SAM | `course_slam/config/lio_sam.yaml:4`：`pointCloudTopic: /pointcloud`、`imuTopic: /imu`；`course_slam/launch/slam.launch.py:15`：四个 `lio_sam_*` executable | 真实后端；其 TF remap 到 `/lio_internal/tf`。 |
| LIO-SAM → 公共定位 | `course_slam/course_slam/pose_adapter.py:48`：publish `/slam/odom`、`/slam/map_points`，subscribe `/lio_sam/mapping/odometry_incremental`、`odometry`、`map_global` | 公共定位来自 SLAM；没有用真值取代它。 |
| 地图 → Nav2 | `course_navigation/course_navigation/map_node.py:31`：publish `/map`、subscribe `/slam/map_points` 和 `/pointcloud`；`course_navigation/config/nav2.yaml:157`：`map_topic: /map` | 在线投影地图供 costmap 使用，静态层采用 transient-local 订阅。 |
| NavFn → DWB | `course_navigation/config/nav2.yaml:89,225`：`dwb_core::DWBLocalPlanner`、`nav2_navfn_planner/NavfnPlanner`；`goal_manager.py:106`：`NavigateToPose.Goal()` | 保留现有 Nav2 action/BT 链，不能仅由插件配置认定运行成功。 |
| DWB → smoother → adapter | `course_navigation/launch/navigation.launch.py:26`：controller remap `/course_navigation/cmd_vel_raw`；smoother 输出 `/course_navigation/cmd_vel`；`velocity_adapter.py:44` 订阅后者 | 正常 FollowPath 经 smoother；behavior_server 的 cmd_vel 直接 remap 到 `/course_navigation/cmd_vel`，恢复动作日志应与正常控制区分。 |
| adapter → rl_sar | `velocity_adapter.py:47,69,76`：publish `/path_manager/path_manager_ros/nav_vel`、读取/写回 `linear.y`；`b2w_sim/b2w_controllers/src/rl_sar_controller.cpp:97` 订阅同 topic | 接口保留三自由度命令；adapter 默认平移向量限幅 0.5 m/s、角速度 0.6 rad/s、过期/缺 TF 停车。 |
| 策略 → 机器人 | `rl_sar_controller.cpp:148,162,170,184`：三命令进入 `obs[6+i]`，ONNX 输出目标，PD 计算 `tau`；`course_sim/launch/simulation.launch.py:71` 选择 rl_sar；`bridge_rl_sar.yaml` 的 16 个 `cmd_force` | 50 Hz policy / 200 Hz 力矩链；未改变低层策略和适配增益。 |

`course_bringup/launch/course.launch.py:34` 用 20 s TimerAction 延迟 SLAM；这不是已就绪证明。`goal_manager.py:99` 另等 lifecycle active 和 TF。恢复时要检查唯一 clock/控制器、递增传感器时间、最新 SLAM、map、公共 TF、planner/controller active、三段 cmd_vel、关节输出，不用固定 sleep 代替这些条件。

## 完整 baseline 参数审计

以下行号均指当前 `course_navigation/config/nav2.yaml`，数值是源码配置，未得到本机运行时 parameter dump。

| 组件 | 关键配置片段 | 解释 |
| --- | --- | --- |
| NavFn（225–228） | `use_astar: true; allow_unknown: false; tolerance: 0.3` | A*、不穿越 unknown；tolerance 是可接受的路径终点偏差，不是障碍容差或机器人宽度。 |
| DWB 速度（90–97） | x `[0,0.4]`，y `[0,0]`，`max_vel_theta: 0.6`；`min_speed_xy: 0; max_speed_xy: 0.4; min_speed_theta: 0` | 正常 DWB 不倒车、不横移；theta 不是只能正转。Humble 使用对称 ±max_vel_theta 范围，而 min_speed_theta 是最小角速率幅值。 |
| DWB 加减速（98–103） | acc x/y/theta `[0.5,0,0.6]`；decel `[-0.5,0,-0.6]` | x/y 单位 m/s²，theta 为 rad/s²；y 动态窗口也锁为零。 |
| DWB 采样（104–109） | `vx_samples:20; vy_samples:5; vtheta_samples:20; sim_time:1.7`；linear/angular granularity `0.05/0.025` | vy_samples 非零不意味着能横移，y 范围仍为零。 |
| DWB critics（115–132） | `RotateToGoal, Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist`；权重分别 `32, 未显式配置, 0.02,24,32,32,24` | PathAlign/GoalAlign 前看距离均 0.1；RotateToGoal slowing_factor=5、lookahead_time=-1。不同 critic 的原始分数单位不同，不能仅按权重大小归因。 |
| controller 其他（68–86） | 20 Hz；velocity thresholds 三轴均 0.001；failure_tolerance=0.3；progress `0.15 m / 20 s`；goal `0.25 m / 0.3 rad` | 与 manager 默认 0.3 m 判成功阈值区分；失败日志应记录 progress checker。 |
| velocity_smoother（278–309） | 20 Hz、`OPEN_LOOP`、scale_velocities=false；max `[0.4,0,0.6]`；min `[-0.2,0,-0.6]`；acc `[0.5,0,0.6]`；decel `[-0.5,0,-0.6]` | y 第二次硬限为零；OPEN_LOOP 采用上一输出作为平滑参考，不是低层实际 tracking 反馈。odom_duration=0.1，deadband=[0,0,0]，timeout=0.5 s。 |
| 两级 costmap | footprint `[[0.70,0.38],[0.70,-0.38],[-0.70,-0.38],[-0.70,0.38]]`；padding=0.05；resolution=0.05；inflation_radius=0.8；cost_scaling_factor=3.0 | 未 padding 为 1.40×0.76 m；Humble 按各坐标符号加 padding，得到 1.50×0.86 m，内切半径 0.43 m、外接半径约 0.865 m。这是配置几何，不是实测轮腿包络。 |
| local_costmap（134–177） | frame=odom、rolling_window=true、5×5 m、update/publish=5/2 Hz | track_unknown_space 未显式设置，官方 Humble Costmap2DROS 默认 false；static layer 的 unknown 处理仍需看最终 master costmap，不能只由该默认值断言全部可通行。 |
| global_costmap（178–217） | frame=map、track_unknown_space=true、update/publish=1/1 Hz | rolling_window 未显式设置，官方 Humble 默认 false。 |
| 两级传感器层 | `static_layer, obstacle_layer, inflation_layer`；`ObstacleLayer`，非 VoxelLayer | lidar `/pointcloud` / PointCloud2；marking=clearing=true；高度 0.15–2.0 m；obstacle range 0.6–8 m；raytrace range 0.6–10 m。 |

官方版本核对：[Humble NavFn](https://api.nav2.org/nav2-humble/html/navfn__planner_8cpp_source.html)、[DWB kinematic parameters](https://api.nav2.org/nav2-humble/html/kinematic__parameters_8cpp_source.html)、[velocity smoother](https://api.nav2.org/nav2-humble/html/velocity__smoother_8cpp_source.html)、[costmap 默认参数](https://api.nav2.org/nav2-humble/html/costmap__2d__ros_8cpp_source.html)、[footprint padding](https://api.nav2.org/nav2-humble/html/footprint_8cpp_source.html)。恢复后补 installed Nav2 版本与实际 dump，不套用新版本插件拼写或参数。

### map projector 的 unknown / free / occupied

`course_navigation/course_navigation/map_projection.py:52`：初始输出 `np.full(...,-1)`；`grid[self.free]=0`；`grid[self.occupied]=100`，occupied 优先。`obstacles():28` 每次重建当前 SLAM map 的 occupied mask，仅取有限、z∈[0.15,2.0] 的点；free mask 随已观测射线累积。

`observe():32` 最多约 720 条射线，`stride=ceil(len(points)/max_rays)`；DDA 清空从雷达起点到表面前一格，finite points 才参与，未观测格不直接清空。`map_node.py:66` 使用扫描时间戳的 SLAM TF，并排除距传感器≤0.5 m 的返回；TF 不可用则保留在最多 30 帧 pending 队列中重试。`have_map` 为 false 时不发 `/map`。

默认范围 20×16 m、origin=(-10,-8)、resolution=0.05；warehouse 总 launch 覆盖为 40×40 m。map publish 只以 `have_map` gate，并写当前时间戳，因此 **仅看到最新 `/map` header 不能证明最新 SLAM map 输入**；harness 应同时观测 `/slam/map_points` 收到时间、SLAM odom、TF 与 clock。这是观测设计风险，不是已复现的 SLAM 故障。

## 窄空间失败根因候选表（按现有证据强度排序）

本轮没有窄通道失败 bag、局部 costmap 或 trajectory critic 得分。下表的排序是排查优先级，不是已证实的根因排名。

| 排序 / 候选 | 文件、关键片段与证据如何支持判断 | 裁决与缺口 |
| --- | --- | --- |
| 1 高层禁用 lateral | `nav2.yaml:91,93,99,102` 的 y min/max/acc/decel 全 0；smoother 四个向量 y 均 0。`velocity_adapter.py:29,69,76` 保留 y。`rl_sar_controller.cpp:101,148` 为 `clamp(linear.y,-.5,.5)` 并进入策略观测。 | **代码事实已确认**：高层压缩动作空间，底层接口支持 vy。尚不能把接口支持等同于稳定 holonomic 物理能力；也不能证明当前窄通道失败由此触发。 |
| 2 locomotion tracking 与预测运动不一致 | `docs/validation/2026-10-11-init-physics.json:33`：vx=0.2 持续 8 s，位移 2.138 m（理想积分 1.6 m）；`:83`：vx=0、wz=0.25 的 8 s 转向阶段位移 0.432 m。`rl_sar_controller.cpp:150` yaw 乘 gain；`course_sim/config/rl_sar.yaml` gain=1.6。 | **历史物理证据**显示宽松验收通过不等于精密跟踪；在狭窄区域可能影响 DWB 预测。未验证 vy；不能排除 tracking，也不能仅凭旧数据归为本次根因。先观测，不改 policy。 |
| 3 footprint/padding 与非圆形碰撞覆盖 | `nav2.yaml:159` 的矩形+padding 得 1.50×0.86 m；URDF `:65` 仅机身 box=0.5×0.28×0.15，`:1598` 碰撞板=0.95×0.45×0.02。`course_sim/course_sim/description.py:23` rl_sar 替换惯量/关节动力学，保留课程碰撞几何。DWB critics 包含 BaseObstacle，不含 ObstacleFootprint。 | **无法认定“过保守”或“真实”**：机身 box/单帧轮胎不能替代轮腿完整扫掠包络。BaseObstacle 在轨迹中心查代价，非多边形逐姿态碰撞检查；不能将矩形参数本身当成全角点安全证明，也不据此立即改 critic。 |
| 4 unknown 断开可规划区域 | `nav2.yaml:186,228`：track_unknown_space=true、allow_unknown=false；`map_projection.py:38,52`：射线抽样、未观测=-1。 | **阻断机制明确，触发未证实**：需失败处 map/global costmap 中的 unknown 连通性证据；正常保留未知不是 bug。 |
| 5 inflation 使低代价带变窄 | `nav2.yaml:152,153,212,213`：radius=0.8、factor=3.0；官方 inflation `computeCost` 在内切半径外指数衰减。 | 可能提高通道代价，**不能说宽<1.6 m 必然无解**。配置内切半径 0.43 m 对理想平行壁的中心硬排斥宽度约为 0.86 m，另有栅格误差；转角及完整机体可行性不能用这个数替代。 |
| 6 DWB critics 导致犹豫/振荡 | `nav2.yaml:115` critics 如上，debug_trajectory_details=false；Path/Goal 对齐项强度和 Oscillation 会影响轨迹筛选。官方 Oscillation 有方向翻转约束及 reset 条件。 | **没有运行证据**证明振荡。需记录正负角速度切换、有效候选数/失败日志、progress checker 与 costmap，再做归因；不是看到 Oscillation 名字就判它诱发振荡。 |
| 7 SLAM/map 本身错误 | `pose_adapter.py:51` 真后端输入；`map_node.py:66` 按扫描时间查 TF；`docs/COURSE_VALIDATION.md:21` 记录修正地面对齐/自身回波后的短目标成功。 | **当前证据不支持作为首要根因，也未排除**。采样里程计跳变、TF 延迟、地图陈旧、墙体厚化，并与真值评价对齐；真值不能进入导航定位。 |

相关官方实现：[BaseObstacle](https://api.nav2.org/nav2-humble/html/base__obstacle_8cpp_source.html) 在 `scorePose` 调用 `getCost(cell_x,cell_y)`，拒绝 lethal/inscribed/unknown；[InflationLayer](https://api.nav2.org/nav2-humble/html/inflation__layer_8hpp_source.html) 区分硬代价和指数衰减；[OscillationCritic](https://api.nav2.org/nav2-humble/html/oscillation_8cpp_source.html) 记录运动符号翻转。上述正常语义是解释依据，不被升级为阻止 baseline 启动的 blocker。

### 历史 baseline 没有覆盖所需窄通道实验

`course_sim/worlds/confined.world`：north/south 墙中心 y=±1.6、厚0.2，主通道净宽3.0 m；障碍中心(0,0.9)、尺寸(0.6,0.6,1)，障碍下方净宽2.1 m。`simulation.launch.py:55` 默认 spawn=(-6,0,1)。`docs/validation/2026-10-10-rl-sar-navigation.json` 是 map 目标(1,0)、路径1.124 m、用时3.893 s的一次短目标成功；它没有证明越过整条狭窄通道，更没有宽度扫描。world 坐标与 SLAM map 原点不可混用。

同文件保存 `planning_measurement_status: aborted` 和 `planning_latency_ms:1.459...`。`goal_manager.py:106–112` 同时发送 NavigateToPose 和独立 ComputePathToPose，`:157–163` 保存任何结果状态的往返耗时。因此这不是“成功规划只用1.459ms”的证据；后来 `docs/COURSE_VALIDATION.md:21` 的成功规划记录也不能覆盖掉前面的失败请求。

## 重大决策的证据式反驳 / 支持 / 裁决

### 在独立配置中启用 lateral（尚未批准为有效改进）

**A，反驳：** C++ 接受 vy 只证明输入维度与夹紧范围。现有物理验收完全没有 vy 阶段，且转向已有 0.432 m 平移；启用横移可能增加侧面碰撞和关节扫掠风险。缺少失败日志时扩大搜索空间可能浪费测试成本并掩盖 unknown/geometry 问题；只改 DWB 还会被 smoother 清零。配置可回退，但碰撞与无效实验的数据成本不能忽视。

**B，支持：** 上下游三命令接口均存在，adapter 不丢 y；DWB 和 smoother 的零限幅是直接且可重复的高层瓶颈。保持 NavFn/DWB、物理 footprint 和地图不变，在独立配置中测试 lateral，因果比较成本最低。

**C，裁决：** 证据支持“优先验证 lateral 假设”，不支持“已确认需要开启”或“已改善导航”。最大未知是实际横向 tracking/碰撞覆盖。若短测显示小 vy 不能稳定跟踪，或失败处是 unknown/硬障碍，先行 lateral 调参结论反转；若 lateral tracking 成立且同场景只在动作空间扩大后稳定通过，才支持 M2。上界来自 low-level |vy|≤0.5、adapter hypot(vx,vy)≤0.5、baseline DWB max_speed_xy=0.4；这些不唯一确定安全的速度及横向加速度，本轮不编造值。

### 缩小 padding / inflation

**A，反驳：** 没有全身动态包络和实际 costmap。缩小 footprint 会制造纸面可行性；缩小 inflation 也可能减弱 BaseObstacle 依赖的安全距离。以“radius×2”判断通道硬阻塞是错误语义。

**B，支持：** padding 当前使总宽增加0.10 m；当已测物理包络可信且失败来自代价带，而不是 unknown 或 tracking，单独测试 padding、radius 或 scaling 有明确可观察结果。

**C，裁决：** 当前反对修改的证据更强。最大未知是动态净空；只有包络测量和失败 costmap 支持才反转。物理 footprint 始终不得伪造，参数实验一次只改一项。

### allow_unknown / free-space clearing

**A，反驳：** unknown 意味没有观测；放开可能规划进墙后或未扫描区域。仅配置为 false 不是缺陷；强制清空会污染证据。

**B，支持：** 若失败路径主要被错误保留的 unknown 切断，且有效传感器射线证明该区域可见，修正观测清空或评估 unknown 策略才直接针对问题。

**C，裁决：** 当前缺少触发证据，保留 baseline。反转条件是同步 map/costmap/点云/TF 在失败区域明确证明 unknown 为主要阻断；必须保留碰撞与地图正确性验证。

### 替换 NavFn / DWB

**A，反驳：** 当前没有窄通道复现，已有可见动作限制未验证。同时换组件会引入模型、参数、计算负载和验收成本，破坏单变量比较；不能以新的插件掩盖旧环境未恢复。

**B，支持：** 只有在前述最小改动仍暴露可重复的结构性问题时，官方/主流替代实现才有针对性比较价值；目前没有该证据。

**C，裁决：** 不替换，也不提前设计迁移。出现明确结构性失败证据后先更新本文件、重新进行 A/B/C，再研究 SmacPlanner2D 或其他 controller。

## Harness 设计与实现任务（待 bundle 审查后继续）

选择：扩展现有 experiment 的思路，新增独立 `scripts/narrow_space.py`，复用 baseline launch 参数；不直接覆盖 `run_experiments.py`，不把 Gazebo 真值送给 Nav2。相比改老 runner，独立入口可保留历史 baseline 指标语义；相比新建完整仿真框架，复用现有 launch 能缩小变更范围。

拟新增文件：`scripts/narrow_space.py`（运行/进程所有权）、`course_navigation/course_navigation/narrow_metrics.py`（纯指标聚合）、`tests/test_narrow_space.py`（超时、故障和清理回归）。生成的 world、manifest、日志、bag 放在忽略的 `course_results/narrow_space/<run-id>/`，不新增另一个 roadmap。

1. **前置检查与场景固定。** runner 在 import ROS 之前提供依赖预检并保存失败 JSON；检查 ROS Humble、install、rl_sar bundle 摘要和无同域活动会话。复用 confined world 的 Fortress plugins，生成固定平行走廊（入口/出口开阔区、墙厚0.2 m、通道长3 m），宽度候选 wide=2.0、medium=1.4、narrow=1.0 m；它们仅是实验 fixture，不代表实测可通行宽度。固定 world spawn=(-2,0,1)、yaw=0，goal 为通道出口；在开阔区设1 m短 goal用于 smoke。记录生成后 world SHA256、spawn、实际 goal_map_pose 和通道局部坐标。
2. **坐标协议。** readiness 时冻结 SLAM map 中的初始位姿，将相对固定 route goal 按初始朝向转换成 map goal；不要假设 map=(world-spawn)。记录实际机器人起步偏移，宽度比较采用同一转换规则。真值只用于评价实际运动/碰撞，不用于提供导航 TF。
3. **种子与参数。** 强制 `fault_profile=normal`、`seed=0` 为 smoke 默认，保留 normal 的小传感器噪声。`sensor_disturbance.py:28` 独立 RNG 可固定；`simulation.launch.py:59–67` seed 当前未传入 Gazebo CLI，因此明确 `physics_rng_seeded=false`，不得宣称完全确定性。记录 policy/config/生成 world 摘要、ROS/Gazebo/Nav2 版本、seed 来源、cmd_vel source。已存在的 `nav_params_file` 支持独立配置，无需改共享接口。
4. **运行与发送目标。** 使用独立 ROS_DOMAIN_ID、IGN_PARTITION、进程会话与唯一 run token，检查会话租约；直接通过已 source 环境的 `ros2 launch course_bringup course.launch.py` 启动，gui=false、rviz=false。等待 clock、/pointcloud、/imu、/slam/odom、/slam/map_points、/map、TF 及 planner/controller/BT lifecycle；warmup 墙钟上限60 s，发送一次 `/goal_pose`，目标仿真时间上限10 s、墙钟另限30 s，清理上限15 s。超出上限保存失败即结束，Agent 不通过延长超时变成长测。
5. **必要日志。** 保存 runtime.log、events.jsonl、JSON结果、参数dump；记录 `/plan`、三段 cmd_vel、SLAM/真值轨迹、TF/clock、map/costmap、collision plate。短测可记录短 bag，长测由用户按日志体积选择。以 topic 的实际消息/时间和 lifecycle 判断就绪，不能仅用 `server_is_ready()` 或 `sleep(26)`。
6. **单次结算与失败分类。** 在 goal_sent 到终止期间记录 action/BT/planner/controller 事件并保留原始状态。使用独立标志记录 collision/localization；primary failure 明确区分 no_path、controller_failure、collision、timeout、localization_map_failure 和 startup_failure。未知 aborted 用 `unclassified_failure` 并附证据缺口，不能硬归因。只有明确 planner 无路证据才 no_path；仅“未见 /plan”不够。Humble action 字段按安装版本核对，不假设新版本 error_code。保存 underlying_navigation_status，允许标识 collision 与 timeout 并存。
7. **清理。** `try/finally` 覆盖启动、rclpy init、发送、采样和异常；取消本次 goal，必要时调用现有 cancel service；停止本次 rosbag、launch 及其后代。登记 PID+启动时间、PGID、run token，先 SIGINT、有限等待，再 TERM/KILL 仅限已确认属于本次的存活进程。即使父 launch 先退出也检查后代；不使用 `pkill -f ros/python/gazebo`。保存前后 CPU/GPU/PID 快照和残留清单，清理 FAIL 时整次 harness FAIL。
8. **有意义的短测试。** 先对缺失/乱序时间、非均匀采样时间积分、动作 aborted 不冒充 no_path、父进程先退出且子进程存活、Ctrl+C、缺 ROS 仍写结果进行回归，再实现。用自有 dummy 子进程证明超时清理不会杀死测试前存在的哨兵进程；这只能证明生命周期逻辑，实际 ROS smoke 另验收。无需为机械文档修改增加测试。

### 指标契约

| 字段 | 定义与有效性要求 |
| --- | --- |
| success | Nav2 成功 + 新鲜 map-frame final pose 在约定距离阈值内；另保存 collision_free_success，不掩盖到达但碰撞。 |
| collision_count | 复用 collision plate force norm >50 N、接触上升沿、1 s cooldown 的事件计数，注明传感器覆盖；缺消息为 null，0 不代表全身无碰撞。 |
| path_length_m | goal 开始后新鲜、时间严格递增的 SLAM odom/base 平移段长度之和；明确为估计实际轨迹，不是 global plan 长度。额外保存真值评价轨迹。 |
| time_to_goal_s | 仅成功时为仿真时间差；失败为 null，另存 duration_sim_s 和 duration_wall_s。现有 RunMetrics 对失败也填 elapsed，不能无注释原样继承。 |
| final_goal_error_m | 新鲜 map→base_link 下最终xy距 map goal；无有效姿态为 null。兼容读取现有 `final_goal_distance_m` 时标明来源。 |
| planning_latency_ms | 约定为成功 ComputePathToPose action 的墙钟往返，另存每次 status 和 failed_request_latency_ms；不是纯算法耗时。避免额外规划探针与 NavigateToPose 同时竞争同一 server；若只测预规划，明确它不等于 BT 内部每次规划延迟。 |
| max_abs_vy_mps / mean_abs_vy_mps | 主指标来自最终 nav_vel.linear.y。mean 为仿真时间加权积分 / 有效观测时长，不按消息个数平均；同时保留 raw/smoothed 三段数据，单位 m/s。 |
| lateral_motion_time_ratio | 有效区间内 abs(vy)>0.01 m/s 的时长比例；阈值写入 manifest，0.01 与当前 idle_hold 命令死区量级一致，仅为指标阈值。 |
| observed_vy 指标 | 从连续真值评价位姿差分并旋转到机体坐标，单独报告实测横移；不能把 nav_vel 横移命令称为实际横移。里程计 twist 的 frame/来源须先核对。 |
| missing / coverage | 每项缺失理由、样本数、有效覆盖率、stale gap。缺数据不置0；clock回拨或超大 gap 截断/标记无效，不用旧命令无限积分。 |

现有缺口依据：`scripts/run_experiments.py:23–31` 没有 world/start/config/width 参数；`:78` 用26 s等待+弱 readiness；`:88–94` 只在父进程存活时清理且没有最终后代清单；`:99` 把失败合并为 startup_or_runtime_failure。`metrics.py:42` 没有 vy 指标。故当前 runner 不能冒充所要求的 narrow-space harness。

## 历史长测交付状态（已由当前 NOW 覆盖）

**本轮不发布可执行的 narrow baseline/improved 多 seed 或宽度扫描命令。** 原因是 harness 尚未实现、真实 smoke 未通过，用户规定的“短测通过后给长测命令”条件未满足。不会提供指向不存在脚本/配置的伪可复制命令。实现后 CLI 应覆盖 `--smoke`、`--nav-params`、`--width`、`--seeds`、`--timeout-s`、`--ros-domain-id`、`--output-dir`，并从实际 `--help` 与已通过 smoke 的入口导出命令。

## 历史打包与停止（已由当前 NOW 覆盖）

提交给 review 的内容仅为本文件、旧计划归档及 `/tmp/b2w-narrow-diagnosis-ls7ys2r7/` 的实际检查证据。调用用户指定 `~/bin/april-review-bundle`，把上述日志显式传入（该脚本的固定项目文档列表没有 PLAN.md，但其 untracked 内容收集会包含新计划）。打包后不进入实现、不改参数、不提交、不 push。

审查重点：是否同意本轮严格在首次 PLAN.md 实质修改处停止；harness 的场景是否足以检验 lateral 假设；指标是否能区分指令与物理运动；恢复环境后优先复现哪一条失败。下一步是在评审通过、具备运行环境后完成 M1 剩余项，而不是直接生成 improved 参数。
