# B2-W Stage 4：第二轮外部独立源码审查

**审查时间：** 2026-10-20  
**Git 范围：** `90716f4ebd8fff713f671529d0da5dfb5fa42aae` → `a9991fc79910fd87ed5ce5cf2fc24c8c37c2be2f`，3 commits / 54 paths。  
**APRIL Bundle SHA256：** `2e083dd9fafdb6c77abcbb118177a78ed469a26cabe123391c2a7a303e35a775`（在本次容器中复核通过）。

## 结论

**源码进展：PARTIAL PASS（上一轮的主要缺失模块已形成可测试的离线实现）；整体里程碑：NEEDS FIX；首次非零导航：NO-GO。**

这个 NO-GO 不等于真实仓库物理不可通行，也不表示离线实现一无是处。此次附带证据显示最终 `a9991fc` 专项测试 232 PASS、harness 6 PASS、native CTest 1/1 PASS，实验分支干净、原始工作区不变；在 `48fef81` 进行的整个 tests 目录测试是 857 PASS / 38 FAIL / 4 SKIP，其中新引入的基线 timer / profile-copy 失配已分别由后续定向 51 PASS 修复；其余主要为被忽略的 ONNX/world 素材、受限 socket 和未提供的外部证据。本次**没有**最新整个 tests 目录的全绿结果，不能把 232+6 的专项成功写作全部通过。

本次没有真实运行 ROS/Gazebo 导航，也没有独立测试工作站运行时环境；以下源码审查基于 Bundle 所封存的 patch、完整关键 HEAD 文件和日志摘录。未在科研工作站复跑 ROS。绝不基于代码审查授予物理运动许可。

## 已确认改进

- 在 `velocity_adapter.py` 有效最终 Twist 输出处加入可选的 `FinalCommandGuard`，通过 `stage4_qualification.require_motion_qualification()` 保留硬禁止非零输出；CLI 和 transport 仍存在独立拦截。
- `stage4_control.py` 分离本地 lease/inhibit 和 watchdog；`ros_transport.py` 增加路径、地图、部署身份的短有效期 dispatch 检查，并冻结 abort destination，修复此前可被路径替换影响的取消目标。
- `run_candidate_route.py`、`runner.py`、`constraints.py` 衔接 Start→G1 等真实分段入口；G3→G4 改为版本化 5.25m 拓扑允许带，但 G1→G2 的 0.35m 强制走廊及 0.25m 航点不变。
- `portal.py` 加强入口/完整机身/尾部离开证据，`ContactIntervals` 加强接触报告截止封存时刻的闭合检查。**均不等于非足部物理碰撞已覆盖**。

## 优先需要处理的源码问题

### P1-1：新 Guard 把低频结构地图与动态 costmap 混为同一种 2s freshness（高置信度的接口/可用性冲突）

`course_navigation/course_navigation/stage4_guard.py:121–125`：

```python
for name,(checker,source,receipt,revision) in grids.items():
    if name!='saved' and (not 0<=sim-source<=2. or not 0<=wall-receipt<=2.):
        raise ValueError('guard_grid_stale')
```

同一 HEAD 的 `scripts/stage4_route/evidence.py:39–41` 则已经允许 `slam_map_age<=7.5`，并以 `slam_map_wall_budget` 区分结构地图节奏。此前实测曾识别结构地图约 0.2Hz（约 5s），而导航局部 costmap、定位和动态感知需快得多。如果此次 Guard 的 `live` `/map` 发布也采用这种节奏，一份几何不变且合法的地图会在两次发布之间使 Guard 锁存 `guard_grid_stale`。这在小段 Start→G1 就足以造成永不运动或莫名中止。

**建议：** 利用真实记录/零运动 ROS 采样，确认 `/map` 实际发布和 source-stamp 节奏；结构地图保持独立版本/心跳策略，动态局部 costmap、TF 和障碍新鲜度继续严格要求。不要简单把四张图的 timeout 全部调到 7.5s。补充一个“结构地图 5s 一次，但 20Hz 快速定位与局部障碍正常”的回归，并对实际 Stage4 配置做零运动测试。

### P1-2：无变化的高频地图消息仍使已准备好的几何短暂失效（性能/进展风险，应压力测而非直接判安全缺陷）

`stage4_guard.py:148–154`：

```python
self.guard.invalidate(name)
generation=self.generations.get(name,0)+1
self.pending[name]=(generation,frozen,metadata)
```

`stage4_guard.py:114–115`：

```python
if set(grids)!=set(REQUIRED):return result
```

甚至相同几何版本的常规更新也会使该图暂时消失，直到后台 worker 完成；worker 又要处理实际 800² 栅格、哈希与 `ClearanceGrid`。在 ROS 高频更新和 Python 单节点执行环境中，可能反复发出零指令而无法持续行驶。当前证据尚不能证明实际负载下一定出现此故障，因此不要以“理论 race”再扩充庞大架构。

**建议：** 先用 Luna 做短时模拟图流 + 20Hz 输出压力回归，记录 active-grid availability、重建耗时、zero-duty、p99；同源同内容只刷新合规元数据、不必销毁已认证的结构几何；新几何变化时继续按已有 fail-closed 策略处理。

### P1-3：正式安全 PASS 尚无法从 runner 产生；CLI 退出码可能误标诊断成功

`scripts/stage4_route/runner.py:46`：

```python
report = evidence.finish(cleanup=cleanup, safety_complete=False)
```

`RouteEvidence.finish` 将安全完整性纳入 `SEGMENT_PASS/FULL_ROUTE_PASS`，所以现版本只可能得到诊断 `PRACTICAL_ROUTE_PASS_SAFETY_UNCERTAIN`。然而 `scripts/run_candidate_route.py:74`：

```python
return 0 if report['practical_route_pass'] else 1
```

一旦日后移除总硬阻断，shell 或调度器可在 `full_route_pass=False` 时收到成功退出码。这是结果契约问题，不是现阶段已获运动许可。

**建议：** 保留诊断成功与正式成功双字段，但正式执行命令的退出码应只对其声明的验收目标成立（如 segment 的 `segment_pass`，full 的 `full_route_pass`），未满足物理证据使用单独诊断退出码/状态；在具备真实 `safety_complete` 来源且完成相应审查前继续拒绝正式 PASS。增加专项测试：动作成功、航点全到但 physical coverage 缺失 => 非零退出码、FULL_ROUTE_PASS=false。

### P1-4：阶段入口参数校验顺序导致不可用的执行锁与非预期异常（在未来解锁前修）

`scripts/run_candidate_route.py:64–65`：

```python
write_new(a.session/'execution-lock.json', ...)
metrics=json.loads(a.deadline_evidence.read_text())
```

`--deadline-evidence` 当前并非 argparse 的 `required` 参数。首次真正走到这段逻辑时，遗漏参数会在创建不可覆盖 execution lock 后触发 `AttributeError`，而 `except` 仅处理 `ValueError/KeyError/OSError/RuntimeError`。同样，错误或不匹配的 JSON 会先消费当前 session。

**建议：** CLI 或显式前置校验必需参数；读入/验证 session、配置、测量证据、有效时限等全部 prerequisites 后，才原子创建不可重复使用的 attempt lock。保留“实际已发送目标不可无证据重试”的原则。针对缺参、损坏证据、错误 session 补充“没有消费执行锁”的回归。

## 仍缺少但不能靠代码臆造的物理资格

- 当前 Stage4 控制/owner/watchdog 启动与心跳生命周期、真实进程隔离、承载完整 ROS 图的零运动烟测（当前没有任何可用的完整 session deployment manifest）。
- 实测整个 smoother→adapter→RL-SAR→Gazebo 的停止、尾部运动、偏航及 fault-to-zero 延时上界。上一轮纯前进开阔地测速不能直接变成所有转向/窄道的保证。
- **当前位置与当前局部路段的**传感器支持配准误差/边界裕度，且必须区分 Start→G1 和后续 G1→G2；不必在 Start 要求看清未来整条窄道。
- 全机身（尤其非足部）物理碰撞/接触覆盖，positive controls，以及终点有效停止与接触报告封存闭合。
- 目标身份、实际路径/朝向、合理速度下的物理通行。不因旧 SLAM 栅格的几何阻断而推定真实 Gazebo 必定走不过，也不能据原始 mesh 的名义通道直接放行。

上述为**尚需验证的资格**，不是“此时已有证据证明实现错误”。禁止为提高 PASS 数量而降低 footprint、放弃未知区、跳过坐标配准，或导入 Gazebo ground truth 用于导航。

## 下一里程碑建议

1. **一个小范围 Sol 修复回合**：优先闭环 P1-1/P1-3/P1-4；对 P1-2 先量测再决定是否改。其他确定性问题按最小修复处理。Sol 可让 Luna Medium 做回归与日志/哈希。
2. **准入准备**：只准备操作性的 session supervisor 和可重复的零运动 smoke/harness，明确哪个部分仅在用户本地执行；不在此处触发未经审查的非零实验。优先局部 Start→G1，随后利用 G1 新的传感器视角处理 G1→G2，再逐段推进。
3. **长测分离与 Review**：小测试完提供可复制的本地短测/长测命令；完整阶段生成一次小型 APRIL Bundle（含原始真实 worktree diff、关键源码片段、测试与未通过项），经 ChatGPT 独立审查后再授权下一阶段。仍不 push/merge；保持历史证据完整。

**推荐下一执行配置：** 新 Sol High 会话（本轮已完成 49 分钟阶段并有封存的 Bundle，可减少上下文成本），Luna Medium 做确定性验证；独立 Review 继续在 ChatGPT 中进行。不再让 Astra 重新做无目标的全项目分析。
