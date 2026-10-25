# B2-W Stage 4 — 2026-10-20 独立审查（a9991fc 后的零运动准备阶段）

## 审查结论

**分级结论：离线修复基本通过源码审查（OFFLINE ACCEPTED WITH LIMITATIONS）；零运动运行器仍 NEEDS FIX；非零导航仍 NO-GO。** 不能把现有 18 个离线 PASS 当作现场资格，更不能为了运行零运动测试伪造或放宽物理制动、定位、碰撞证据。

证据：用户提供的 `b2w-stage4-readiness-sol-20261020.zip`，解压后 425 条条目；内含 APRIL Markdown 和其 SHA256 校验一致，SHA256 `0c66f68124ad3385993c2f6fb9967ec77f5ffb746d5a3f62c60073ee62556986`。`scoped.diff` 以及重建核验记录一致；基线/HEAD 均为 `a9991fc79910fd87ed5ce5cf2fc24c8c37c2be2f`，修复是**尚未提交的工作区变更**，不要误称新提交。所有现有历史证据和原始工作区在本报告中均视为只读。

报告显示：267 个专项 Python 测试、6 个 harness 测试、1 个原生 CTest 通过；2 个 ROS 包以拷贝方式成功构建，34 项安装文件一致；18 离线资格 PASS、27 现场资格 FAIL；**没有启动 Gazebo/ROS 运行栈，也没有非零实验**。这是打包的运行记录与静态源码检查，不是审查者在用户工作站上复跑。

## 已解决的四项旧 P1

1. **结构地图时效性：** 现有 `course_navigation/structural_freshness.py` 把结构云心跳和 1 Hz 重发的 `/map` 分开；动态局部/全局 costmap 保留 2 s 约束。`velocity_adapter.py` 新增 `/slam/map_points` 订阅，避免仅用 `/map` 新消息时间戳伪装结构源更新。需要在现场验证云帧 ID、实际消息时标和 QoS。
2. **栅格更新可用率：** `GridPreparation.submit` 对完全相同的已接受几何复用 `ClearanceGrid`、刷新来源时间；不同几何仍立刻 invalidate。保存的 800×800 合成场景记录可用率从 0% 到最后的 95%，但最后一次周期 p99=37.899 ms、准备阶段 p99=100.383 ms；都是合成结果，不证明 ROS 全负荷实时性。
3. **结果退出码：** `scripts/run_candidate_route.py:17–23` 使正式退出 0 依赖 `full_route_pass`/`segment_pass`、结果状态、清理和无错误；只有 `practical_route_pass` 时为诊断退出 3，不再误报为正式成功。
4. **执行锁边界：** `scripts/run_candidate_route.py:97–100` 在建立 `execution-lock.json` 前先调用 `execution_prerequisites`；期限、配准、会话、部署身份证据缺失时拒绝。其余运行时前置门槛仍保留。

## 新发现 A — P1：零运动资格与运动资格循环依赖（确定的设计/代码边界）

**文件：** `scripts/stage4_session.py:20–31, 136–143`，`scripts/stage4_route/session.py:11–18`，`docs/STAGE4_RUNTIME_READINESS.md`。

关键代码：

```python
for key in ('bounds_evidence','saved_grid','collision_inventory'):
    file=Path(guard[key+'_path'])
    if not file.is_absolute() or digest(file)!=guard[key+'_sha256']:
        raise ValueError(...)
from course_navigation.stage4_bounds import validate_bounds
bounds=json.loads(Path(guard['bounds_evidence_path']).read_text())
validate_bounds(guard,bounds)
```

同一脚本的用户本地 `--execute-zero-smoke` 也先调用 `validate_inputs`。`stage4_bounds.validate_bounds` 要求当前会话传感器注册最坏界、测得的 `stop_translation_m/stop_yaw_rad/abort_latency_s` 及原始证据。Bundle 同时明确**目前没有可用的完整测量清单**。

因此，**新的“零运动观测”执行器无法用来获取自己所要求的那些后续物理资格证据**。这种拒绝是 fail-closed，不是碰撞安全缺陷；但与用户希望先进行独立、安全 E0 零运动观测的工作流冲突。不能通过伪造证书来绕过。

**最小修复：** 明确分开 `OBSERVE_ZERO_ONLY`（真实零速输出硬锁、会话身份、进程归属、零指令审计、传感器/日志容量与清理检查）和 `ARM_MOTION`（额外要求经过独立复核的物理制动/定位/碰撞证书和短期 lease）。前者不能产生 lease，也不能让任何非零路径进入发布口；后者目前仍无授权。为无证书零运动流程补上最小端到端/故障注入测试。否则只可使用已验证的旧被动 E0 录制流程，不运行新启动器。

## 新发现 B — P1：未 armed 阶段也可能锁存启动期 TF 短暂缺失（条件性的实测风险）

**文件：** `course_navigation/course_navigation/velocity_adapter.py:267–316`。

关键代码：

```python
if getattr(self,'stage4_guard',None) is not None:
    context=self.stage4_context()
    ...
    command=self.stage4_guard.output(command=command,**context)
...
except Exception as exc:
    if getattr(self,'stage4_guard',None) is None:raise
    self.stage4_guard.inhibit('final_guard:'+repr(exc))
```

`stage4_guard.output()` 内虽有 `lease` 缺失时直接返回零的安全分支，**但 `stage4_context()` 在进入该分支前执行**。如果静止启动期间 TF 已通过基础 `localization_valid()`、却尚不满足 Stage4 精确同源时间匹配，`stage4_context` 抛出异常会永久写入 inhibit；后续 supervisor 在 `stage4_session.py:83–108` 发现 inhibit 后放弃启动。

并非断言真实 TF 一定报错；这是源码中的**条件性永久拒绝路径**，缺乏真实启动观察。建议做非 armed、缺少精确匹配 TF 的启动序列回归：输出必须保持零，故障应明确区分“临时未就绪”和需要永久锁存的真正身份/时钟/证据篡改；任何已 armed 的 TF 失败仍须立刻拒绝/停止。不要放宽运动时的来源一致性。

## 限制 C — 尚未证明零运动脚本能实际启动（不是新的 P0）

`tests/test_stage4_session.py` 主要覆盖 `SessionOwner` 的人工 health 对象、真实短命子进程、watchdog 和 Mock Adapter；它**不构成**完整 `validate_inputs → Gazebo launch → ROS graph → zero probe → final journal → cleanup` 的现场通过。需用用户工作站上的真实现场 E0 记录检验：部署前缀与 policy/world 绝对路径、launch 参数、原生 contact 观察覆盖、心跳/QoS、终点停止、所有子进程归属。当前任何 Live FAIL 不应凭修改常数转绿。

## 审查建议及准入

- **可以接受：** 保留结构图时效性区分、相同地图复用、正式退出码与前置拒绝修复，并继续无运动的确定性回归。
- **应先修：** 解决 A；按 B 添加启动序列小测试，并用最小修复处理其复现问题。只为零运动引入明确的不可升级观察状态，不制造新运动授权开关。
- **后续：** 由 Luna 跑小测试、Sol 核对真实 worktree 和进程清理；一份 scoped APRIL Bundle 再审。若修复后生成真正可运行的无租约 E0，只授权用户本地 **零运动、短时** 观测验证；正式长测分离，命令从实际 `--help`、安装环境和校验过的输入派生。
- **仍禁止：** 第一次非零 Start→G1、全部 G1–G5、真实机器人运动。需要另外验证物理停止/中止延迟、非足部碰撞形状覆盖、传感器登记误差与线速/角速支持，才能提交后续运动资格 Review。

本次不建议重新规划 Nav2、换控制器、放宽 padding 或要求完整窄道地图先于 Start→G1；主要阻断是**E0 观测/ARM 资格层级设计尚未切开**。
