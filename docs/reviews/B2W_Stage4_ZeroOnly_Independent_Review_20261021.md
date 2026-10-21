# B2-W Warehouse Stage 4 — ZeroOnly 独立审查（2026-10-21）

## 0. 审查对象、来源与校验

- 独立实验分支 `experiment/unattended-g1-g5-20261019`，报告记录的新 checkpoint `1f33365757de7fd3d372ccf91b203f1461cd862f`（父提交 `a9991fc79910fd87ed5ce5cf2fc24c8c37c2be2f`）。
- 依据用户上传的完整 `APRIL_REVIEW_BUNDLE_B2W_Stage4_checkpoint_1f333657_20261021.md`，SHA256 `ede14f4eb77f3aaa38aee011e890cb8b46ec34ae614bd8d04d99ef1c433c78f5`；配套 commit-record、REPORT、review-summary、checklist、测试输出及归档 manifest。
- 在审查容器对上传的 14 项 manifest entries 逐项重新计算 SHA256：全部一致；Bundle 和 manifest 两个 sidecar checksum 均一致。
- 本审查是在上传的源码快照及日志上的独立静态/逻辑审查，**没有远程连接用户工作站、没有复跑 ROS/Gazebo、没有验证实际运行中的二进制身份或进程状态**。测试结果是用户提供的真实日志及 checklist，不是审查者现场复跑。

## 1. 分级结论

**CONDITIONAL PASS — 仅准许一次短时、显式零运动 Gazebo 集成观察（ZeroOnly）。**

**NONZERO NO-GO**：不准许 Start→G1、G1–G5、任何非零速度/导航 action、发放 lease、解除硬运动拦截，也不准许真实机器人运动。

**条件性通过不等于 ZeroOnly_RUNTIME_PASS**：尚未取得实机/仿真现场 ROS、TF、地图、接触、真实零速度输出、看门狗行为及完整清理证据。若任意前置身份/环境检查失败，**不得启动**；若已经启动但出现错误，按现有 fail-closed/owned-cleanup 机制停止、保留故障证据，不循环重试。

### 决策的反面理由（反对马上测试）

1. 最新外部 Zero Readiness 报告（2026-10-20）还是 `NEEDS FIX`，现有修复从未做过完整 ROS/Gazebo 执行链的现场验收；TF 精确同源时间匹配仍可能在启动时引发永久 inhibit。
2. 零命令不等于刚体必然绝对静止；仍需记录速度/位姿漂移和全体接触流的局限。
3. `docs/STAGE4_RUNTIME_READINESS.md` 中的“uncommitted delta at a9991fc”是 checkpoint 前历史表述，不能用来替代现在的源码哈希检查。

### 支持一次有界 ZeroOnly 的证据

1. 旧问题 A（零观察循环依赖）已做隔离：`scripts/stage4_session.py::observation_guard` 只要求 session ID 与哈希绑定的碰撞清单，`validate_inputs` 明确要求 v2；不提前索要制动/定位运动证据。
2. `SessionOwner` 插入不可升级的 `zero_only:true`；`FinalCommandGuard.check_zero_control` 拒绝任何 lease；`FinalCommandGuard.output` 在 `zero_only` 时输出 `(0.,0.,0.)`；`Watchdog.step` 的零模式不会写 lease。
3. `VelocityAdapter.command` 对 Twist 六轴任意非零值锁存 inhibit；`publish` 发生启动期定位/TF 未就绪时发布零并持续写 health。新测试证明 `TransformException` 暂缺可以恢复，而已 armed 的丢失仍锁存。
4. `scripts/stage4_zero_probe.py` 是被动观察器，无导航 action/goal/速度发布代码，接受条件包含至少 30 仿真秒及 5 墙钟秒、位移/速度/接触终点闭合/录制容量；工作会话和 owner watchdog 有上限及按身份清理。
5. 保存的离线日志包含 284 focused PASS、6 harness PASS、原生 CTest 1/1 PASS；18 个离线门槛 PASS，27 个 live 门槛 FAIL，`motion_authorized:false`；Checkpoint 确认为独立分支，未合入共享分支。

**权衡：** 对于“只执行一回零运动 Gazebo 观察并收集失败或成功证据”的目的，支持证据足以允许受控短测；对任何非零运动则远远不够。结论在发现 source/artifact 不匹配、非零指令传播、任何 lease/旁路、无法限定进程归属或无法可靠清理时立即撤销。

## 2. 具体代码与因果边界

- `course_navigation/course_navigation/stage4_control.py::Watchdog.step`：`if c.get('zero_only') is True: ... return True` 在 lease 写入之前，且已有 lease 时抛错。这阻断了由 watchdog 发放运动许可的路径。
- `course_navigation/course_navigation/stage4_guard.py::FinalCommandGuard.output`：`if self.zero_only: self.check_zero_control(); return result`，其中 `result=(0.,0.,0.)`；`check_zero_control` 还验证 owner、inhibit、lease。它保证这一 Guard 代码路径不向最终发布口返回非零值，但仍需在现场核对装载版本与其他发布者。
- `course_navigation/course_navigation/velocity_adapter.py::command`：`if guard is not None and guard.zero_only: ... if any(... v!=0.): guard.inhibit(...)`；六轴检测发生在速度限制之前，防止先裁剪再忽略违规请求。
- `course_navigation/course_navigation/velocity_adapter.py::publish`：启动期 `LocalizationUnready`、`TransformException` 进入保持零输出并发布带 `context_state` 的 heartbeat；非预期异常仍 inhibit，不把异常误报为 ready。
- `scripts/stage4_session.py::execute`：launch 禁用 `goal_manager`/RViz/adaptive governor，调用被动 probe，并在 finally 使用 `owner.stop`/`OwnedProcesses.cleanup`；如外部进程或限制条件不符，停止并保留证据。

## 3. 必须满足的执行前提和范围

1. Codex 必须先读取**本审查原文**及原 Bundle，并验证当前 HEAD `1f33365757de7fd3d372ccf91b203f1461cd862f`、工作树、输入 v2、source、matching copy install、policy/world/controller 身份和 `ROS_DOMAIN_ID=197` 当前未占用。旧已验 input 不能替代现场复验。`improve/narrow-navigation` 与 Pipe V0 一律不动。
2. 仅运行仓库现有、已记录的 `scripts/stage4_session.py --execute-zero-smoke` 一次；先运行其只读 `--inputs/--output` 验证。不得修改运动禁令、发放 lease、发送 goals、非零 Twist、绕过验证或调宽超时。
3. 允许正常 ROS/Gazebo 栈启动；原脚本现有上限为 bootstrap 90 wall s、probe 90 wall s 加关闭步骤、supervisor 主循环 220 wall s、watchdog 150 wall s；这**不是严格总时长 220 秒**。若进程归属或 watchdog 不成立，停止测试，不能让其遗留后台运行。
4. 逐项保留 ROS/TF/map cadence、接触报告起止区间、所有命令话题的六轴数据、位姿/速度及停止证据、health/context、owner/watchdog/stack/adapter 进程生命周期和完整 cleanup；至少记录 PASS/FAIL/NOT_OBSERVED，不能把缺失观测记作 PASS。
5. 特别留意一个**非阻断但可能导致测试失败**的条件：当前 `VelocityAdapter.stage4_context` 对精确 TF/odom 源时间相等的校验抛 `ValueError` 时，`publish` 的一般异常分支仍会永久 inhibit。若实际遇到，记录第一条原始错误并做最小根因定位，**本轮不自动重试或修改**。它是可用性风险，不是允许绕过的一项运动安全保护。
6. 单次测试完成后验证所有由该次会话启动的 CPU/GPU/ROS/Gazebo 后台进程已退出；不得使用 `pkill -f`、`set -e`、`exit`、破坏性 Git 操作或系统升级。
7. 报告、APRIL Review Bundle、SHA256、简短日志摘录与最终验收摘要必须**最终交付** `/home/zyd/APRIL_REVIEW_BUNDLES/`，验证目录存在和文件 hash；临时大 bag/原始中间日志可保留其他目录，正式报告必须列其精确路径及 hash。不得覆盖历史文件或移动用户已有报告。
8. 在新 Bundle 交由 ChatGPT Review 前不提交新的源码更改、不 Push、不进行任何非零导航。

## 4. 继续推进的分岔条件

- **ZeroOnly PASS + 实际 runtime 证据充分**：可进入下一轮针对 Start→G1 本地运动物理资格的最小测量/审查计划（停距、yaw/abort 延迟、配准误差、全体碰撞覆盖、部署/路径证据）。仍是 NONZERO NO-GO。
- **ZeroOnly FAIL**：只针对首个可信运行时根因做最小修复，并用聚焦回归及一次新的 Bundle 复审；不要重复跑 284 项全矩阵来替代现场诊断，不无限扩建安全架构。
- **ZeroOnly NOT RUN**：说明唯一实际前置阻断并保存只读证据，不把“缺历史 CONDITIONAL PASS 文件”再次作为阻断——本文件就是此次新审查决定，但前提是源/环境身份一致且没有更新的相反证据。

此审查严格限定 Warehouse Stage 4 静止观察。任何从零运动到真实导航的状态转换都必须另行独立审查。
