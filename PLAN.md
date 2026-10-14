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

- 分支：`improve/narrow-navigation`；历史审计基准 HEAD：`66db5152b3b617d0e5ac5ed65b031573255a4169`；当前HEAD见NOW；上一轮开始时工作区干净；本轮继承已有未提交harness/文档改动。
- 目标：在 B2-W + LIO-SAM + Nav2 baseline 上先复现真实窄空间失败，再做可回退、单变量或少变量改动。
- 不改 locomotion policy，不跑长实验，不 push，不 merge main。本轮复用baseline与上一轮已有的独立单plugin诊断配置；不改公共 topic/TF、消息类型或模块边界。
- 已读 `AGENTS.md`、`CODEX_GUIDE.md`、`docs/INTERFACE.md`、`docs/INTERFACE_ALIGNMENT.md`、`docs/COURSE_VALIDATION.md`、`docs/RL_SAR_SETUP.md`、`docs/LOCOMOTION_STATUS.md`。参数 SSoT 为当前 YAML/C++/Python 实现；历史 JSON 仅证明当时的有限验收范围。
- 已应用 using-superpowers、systematic-debugging、brainstorming、writing-plans、verification-before-completion，以及 OpenAI Docs 的模型选择流程。本轮设计属于已有实验流程的有界扩展；用户要求的唯一根计划和评审关卡优先于 Skill 的额外计划/提交流程。不派生子 agent。
- 旧实施计划归档至 `docs/plans/archive/2026-10-10-local-course-system-plan.md`。历史 design/validation 保留为证据，不是另一个当前 roadmap。无新增 HANDOFF。
- 历史轮次只读与 timing 交付已结束；本轮范围以最新用户请求和下方 NOW 为准。旧实验/交付记录只作历史证据。

## NOW — 0.0 observer-fix closeout（2026-10-14）

唯一目标：当前修复后的 replay code/binary，仅一次 Scenario A / width1.70 / seed0 / goal6m 的 SmacPlanner2D cost_travel_multiplier=0.0。source SHA256 `33e13df9eca0acdd072c86af6fedef0bfbe89631095b61d07ade46569771e22b`。证据 `/tmp/b2w-smac-observer-closeout/`。0.5/1/2/4沿用上一轮strict PASS，不重跑。禁止Gazebo/live SLAM/NavigateToPose/warehouse/其他multiplier，baseline与planner源码不改，不commit/push；若SIGSEGV保存证据STOP，禁止第二次replay。

- [x] 读取指定入口、最新232440 bundle并核对branch/HEAD与上一轮状态；现有observer复制消息，在plugin cleanup/DSO teardown前释放消息与subscription。
- [x] 编译当前replay，相关unit tests；保存源码/binary摘要与保护配置完整性。
- [x] 单次0.0：source SHA、完整applied_request、actual 0.0及其余runtime参数、loaded/before/after hash、createPlan、final/unsmoothed path、route、wall time、deactivate/cleanup、returncode=0、remaining={}和strict_case_verified=true全部验收。
- [x] PASS时更新sensitivity最终表：五值均narrow，仅说明本frozen case无wide/narrow切换；不选最佳权重、不泛化。保留旧0.0=-11异常证据。
- [x] 45项tests、CMake build、Python compile/diff-check、owned cleanup与完整性审计通过；所有报告与证据备齐。
- [ ] 最后交付动作：生成april review bundle后立即停止，不再回写PLAN；实际成功与路径以最终工具输出为准。

结果：0.0 strict_case_verified=true、121 poses/narrow、length6.000000089406967m、descriptive exposure375.2000055909157cost·m、planning_wall_ms0.475532、returncode0、remaining={}。observer在cleanup/DSO前释放已核对，本case正常teardown闭环；旧精确SIGSEGV栈仍unavailable。五值最终均strict PASS/narrow，科学结论不变。仅一次新plugin进程，baseline/policy/HEAD不变，不commit/push。

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

## 后续 milestone（均未开始）

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
