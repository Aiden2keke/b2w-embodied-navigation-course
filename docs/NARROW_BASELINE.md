# Narrow-space baseline characterization

本文件包含实验协议、历史诊断和当前在线证据；唯一当前计划为根目录 `PLAN.md`。baseline 参数全部保持原样。最新结果见末节“0.0 observer-fix closeout”；之前的“尚未验证”描述仅对应其历史轮次。

## 配置事实和语义

下表的代码片段来自当前文件；Nav2 算法语义对照已安装 Humble 1.1.20 对应上游源码。导航 footprint 是规划近似，不能称为运动中全机身/腿部的实测包络。

| 结论 | 文件与关键片段 | 解释 |
|---|---|---|
| 原始导航 footprint 1.40×0.76 m | `course_navigation/config/nav2.yaml`，local/global 两处：`footprint: '[[0.70, 0.38], [0.70, -0.38], [-0.70, -0.38], [-0.70, 0.38]]'` | 矩形 xy 极值差；不是 URDF 随姿态变化的物理宽度。 |
| padding 后 1.50×0.86 m | 同文件：`footprint_padding: 0.05` | Nav2 给各顶点的带符号 x/y 各加 0.05 m；半长0.75、半宽0.43，内切半径0.43、外接半径约0.8645 m。 |
| 两 costmap 分辨率0.05 m | 同文件 local/global：`resolution: 0.05` | local rolling 5×5 m；global 跟随 static map 的有限尺寸。 |
| 投影地图同为0.05 m | `course_navigation/course_navigation/map_node.py`：`declare_parameter('resolution', 0.05)`；`course_navigation/launch/navigation.launch.py`：`'size': [width, height], 'origin': [-width / 2, -height / 2]` | confined 默认20×16 m；warehouse 场景由 `course_bringup/launch/course.launch.py` 的 `nav_args.update(map_width='40.0', map_height='40.0')` 扩至40×40 m。新 fixture 用 confined 默认范围，无修改。 |
| unknown 为实际观测限制 | `course_navigation/course_navigation/map_projection.py`：`grid = np.full(self.free.shape, -1, dtype=np.int8)`、`grid[self.free] = 0`、`grid[self.occupied] = 100`；`cells()` 使用 `np.floor` | 未观测区域保持unknown；传感器射线和SLAM障碍投影不等同于几何真值图。墙面落格、噪声、SLAM误差会改变实际有效宽度。 |
| inflation 是代价场 | `course_navigation/config/nav2.yaml` 两层：`inflation_radius: 0.8; cost_scaling_factor: 3.0` | 最近障碍距离 d≤0.43 m为253（inscribed）；0.43<d≤0.8 时约 `floor(252*exp(-3*(d-0.43)))`，不把两侧代价简单相加；影响区外无该层新增代价。0.8 m不是必需的机身净空。 |
| NavFn 使用带代价的A* | 同文件：`plugin: nav2_navfn_planner/NavfnPlanner; tolerance: 0.3; use_astar: true; allow_unknown: false` | 253/254及unknown不可通行；0..252仍可通行但影响累计代价。因此它可能选更长但低代价路线；现阶段不是过度保守的实验证明。0.3是规划终点搜索容差，不能等同DWB到达容差。 |
| DWB运动采样 | 同文件：`plugin: dwb_core::DWBLocalPlanner; max_vel_x: 0.4; min_vel_x: 0.0; max_vel_theta: 0.6; vx_samples: 20; vy_samples: 5; vtheta_samples: 20; sim_time: 1.7` | 正常轨迹前进/转向采样；分辨率 `linear_granularity: 0.05; angular_granularity: 0.025`，20 Hz controller。backup recovery 单独记录。 |
| DWB代价与局部拒绝 | 同文件 critics：`BaseObstacle.scale: 0.02; PathAlign.scale: 32.0; GoalAlign.scale: 24.0; PathDist.scale: 32.0; GoalDist.scale: 24.0; RotateToGoal.scale: 32.0` | 配置选 BaseObstacle 而非 ObstacleFootprint；上游 BaseObstacle 检查轨迹中心所在cell并拒绝253/254/unknown，不逐姿态检查矩形全边缘。yaw角度导致的真实包络不可用内切圆保证。 |
| 到达/进度阈值 | 同文件：`xy_goal_tolerance: 0.25; yaw_goal_tolerance: 0.3; required_movement_radius: 0.15; movement_time_allowance: 20.0; failure_tolerance: 0.3` | 失败可涉及无法生成合法轨迹、进度或定位；保留controller action status与日志，不把所有 NavigateToPose ABORTED 判作controller失败。 |
| vy=0在两处约束 | 同文件DWB：`min_vel_y: 0.0; max_vel_y: 0.0; acc_lim_y: 0.0; decel_lim_y: 0.0`；smoother：`max_velocity: [0.4, 0.0, 0.6]; min_velocity: [-0.2, 0.0, -0.6]` 和y加减速度0 | `vy_samples: 5`不会产生非零横移速度；normal FollowPath 与smoother仍只能vx/wz。 |
| adapter/policy接口保留vy | `course_navigation/course_navigation/velocity_adapter.py`：`msg.linear.x, msg.linear.y, msg.angular.z` 进入gate并写回；`b2w_sim/b2w_controllers/src/rl_sar_controller.cpp`：`msg->linear.y` 与 `obs[6+i]=command_valid?command_[i]:0` | 接口具备y通道不表示本baseline启用了y，也不证明物理横移tracking。本轮只读且记录三段cmd。 |
| warehouse基础设施 | `scripts/prepare_warehouse.py`：从 `confined.world` 复制 `physics/plugin/light`，保留固定AWS模型，`mu=mu2=1.0`，输出 `warehouse.world` | AWS几何不便精确控制通道宽度；本轮复用同一Fortress模板的floor与静态box，避免新增下载和修改仓库资产。 |
| world override与spawn已存在 | `course_sim/launch/simulation.launch.py`：`world = LaunchConfiguration('world').perform(context) or ...`；spawn循环读取 `spawn_x/y/z` | harness传独立world和(0,0,1) spawn，使用同一rl_sar/传感器/SLAM/Nav2链，无共享接口或launch修改。 |

上游语义证据：[padFootprint](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/footprint.cpp)、[computeCost](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp)、[NavFn::setCostmap](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_navfn_planner/src/navfn.cpp)、[BaseObstacleCritic](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/base_obstacle.cpp)。这些片段支持代价/碰撞检测语义，不证明实际通过边界。

## 宽度与场景推导

`scripts/narrow_fixture.py:geometry` 从baseline YAML读取两层几何并检查一致性。W=0.86、L=1.50、r=0.05、R=0.8 m：默认宽度取 W 下侧的floor格与前一格（0.85/0.80），上侧ceil格（0.90），W加3/7格后量化（1.00/1.20），2R（1.60）及2R+2r（1.70）。前两项可能在planner阶段就失败，保留作为负对照，不能算controller能力失败。轴对齐真实边界会受栅格相位与观测影响，0.86只是假设模型下的连续几何阈值。旋转时矩形横向包络为 `L*abs(sin(yaw))+W*abs(cos(yaw))`，原地任意转角的最大宽度约1.729 m。

A/B均在封闭房间内，start=(0,0)，相对start goal=(6,0)，choke x∈[2,4]。下方block连到南墙。A上方block从y=Wcase/2到Wcase/2+1.5，上方留宽路 `max(L,2R)+4r=1.8 m`；B把block延伸到北墙，因此任何连续左→右路径均须经过choke。直达中心线长度6 m，默认最窄到最宽A的宽路几何参考中心线约9.95–10.80 m；实际planner路径长度单独记录。这是参考路线长度，不是宽路最短路径的严格证明。gate x=3的线段交点判别narrow/wide，缺crossing或穿block区域为unclassified。

同一world生成规则、同一start/goal、相同Nav2文件与政策摘要；每case独立Gazebo partition/重启，normal profile仅切换传感器seed。物理RNG未由现有启动代码设seed，不能声称物理完全确定。

目标由启动后新鲜map→base_link与初始yaw构造，而不使用真值定位。这依赖启动时机器人保持在world原点、初始朝向基本不变；start_map_xy_yaw记录实际SLAM偏移，fixture坐标分类存在初始漂移/定位误差边界。几何world中的goal与map目标不能在强漂移场景下无条件等同。

## 指标和归因

`scripts/run_narrow_baseline.py` 直接调用ComputePathToPose，等待result后才发NavigateToPose；不发`/goal_pose`，既有goal_manager不产生并发额外规划。记录独立preplan与随后`/plan` BT路径，不把preplan路线当作导航全程唯一规划。

每case输出fixture/world、result、cleanup、runtime、observations.jsonl、map-before-plan、costmap-before-plan、preplan。FollowPath status额外保存goal UUID、goal stamp和数值状态；`controller_abort_observed`明确表示观察窗口内曾出现ABORTED，只是 observed evidence。`controller_action_status_observed` 和 observations.jsonl 保留 UUID、原始 sec/nanosec stamp 与数值 status；没有可靠的 navigation→FollowPath goal 关联和终止原因时，`primary_controller_failure=null`，分类函数不会根据 observed ABORTED 归为 controller failure。结果包括scenario/width/seed、生成路径、chosen route、success、plate collision_count、odom实际path_length、仅成功时time_to_goal、fresh map TF final error、planning roundtrip、cmd各topic样本数/max与完整vx/vy/wz时序、FollowPath abort状态、controller告警、failure taxonomy。minimum clearance暂为null，原因是没有可靠全身边界测量。缺反馈为null，不冒充0。额外订阅`/sim/ground_truth/odom`仅用于评价，保存world轨迹与physical_gate_route / corridor_entered_truth / corridor_passed_truth；不参与Nav2定位或goal生成。

A对两条固定几何参考中心线采样真实raw global costmap，记录unknown/253/254；`route_comparison_eligible`只说明这两条采样中心线可用，不是全走廊/全footprint/地图精确性的证明。unknown/blocked时优先解释为可观测性或geometry限制，不能下“过度保守”结论。地图快照与实际path应共同复核。B先区分planning_failure与navigation结果，再区分未归因的navigation abort、controller observed status/progress日志、timeout、runtime/localization等；不可据NavigateToPose的ABORTED单独断言locomotion失败。

通过能力至少需要多次SUCCEEDED、final error≤0.25m、collision反馈已覆盖且0、实际经过gate及良好定位/odom覆盖。单次规划成功、单次smoke或缺plate反馈均不等同稳定无碰撞通过。当前实现保存真值轨迹并计算采样机器人中心的gate通过（不是全机身净空），仍需复核初始漂移、真值frame与stamp覆盖；正式结果需人工复核后给出全部seed成功的宽度和失败宽度区间，不能以均值掩盖失败。

## 操作

```bash
# 不启动ROS/Gazebo；查看由当前baseline推导的fixture和manifest
scripts/run_narrow_baseline.sh --prepare-only --output-dir /tmp/b2w-narrow-prepare
# 入口短测：B fixture、最宽推导项、seed=0、1m目标、30sim秒timeout；不证明通道通过
scripts/run_narrow_baseline.sh --smoke --ros-domain-id 97 --output-dir /tmp/b2w-narrow-smoke
# 用户正式baseline sweep；42个case，agent不执行
scripts/run_narrow_baseline.sh --scenarios A B --seeds 0 1 2 \
  --timeout-s 90 --startup-timeout-s 90 --ros-domain-id 97 \
  --output-dir course_results/narrow-baseline
```

确保所选domain无其他仿真；若其它会话在97则使用另一空闲domain。每case结束先停止指标采样并取消自己的action，再SIGINT own launch；仍存活时对唯一token且PID starttime一致的本轮进程发SIGTERM/SIGKILL，检查剩余清单。无需`pkill -f`。cleanup失败立即阻止下一case。baseline-integrity.json检查运行前后YAML摘要一致。不设置参数、不启用vy、不换算法。

## 本轮短测证据（2026-10-13）

- 全套29项单测PASS，`/tmp/b2w-narrow-characterization/tests-final.log`。fixture prepare与CLI help可用。
- 首次6m B/1.70m/seed0已达到map/costmap/SLAM TF/三Nav2 active就绪；candidate centerline 121样本unknown=0、blocked=0、max_cost=151，但ComputePathToPose ABORTED，上游运行日志为合法potential后path extraction失败。结果目录 `smoke-lifecycle/20261013T080112.872655Z`。这是单次规划失败证据，未调用NavigateToPose，不证明controller/locomotion失败。
- 最终入口1m smoke：NavigateToPose SUCCEEDED；plate collision=0；odom path=0.775433 m；simulation goal time=2.6575 s；fresh TF error=0.230703 m；preplanning action roundtrip=4.293773 ms。最终cmd最大绝对vx/vy/wz=0.4/0/0.24，三段vy均0；23个owned identities，remaining={}，cleanup PASS。结果目录 `smoke-final/20261013T080610.128607Z`。truth entered/passed均false，禁止称为corridor PASS。
- 此成功smoke的controller_abort=true仅有goal-checker WARN，未发现解释该ABORTED的controller runtime日志；保留原字段与不确定性。后续harness已增加raw action status UUID/stamp证据；新增status日志格式未再跑真实仿真，按要求停止扩展短测。
- 早期短测仅同一B/1.70m/seed0，未跑任何宽度sweep：sandbox DDS socket被拒；host每spin一次/proc扫描耗12.43ms使TF callback积压；action endpoint先于lifecycle active造成一次ready race；ROS Log.WARN是bytes造成一次回调类型错误。分别通过隔离host smoke、每秒ownership扫描、复用LifecycleReadiness、29项回归短测修正。所有启动会话cleanup均PASS；初始化前只读日志路径失败未启动仿真。
- 独立只读review的规划timeout归类、观测epoch、失败final error三项问题已修正；review复核无blocker。baseline与policy文件未修改，所有run的baseline-integrity均unchanged=true。

尚未验证：A路线偏好、正式B完整通道成功率/边界、多seed稳定性、A真实地图质量与候选路线可用性、全机身碰撞/clearance。NavFn extraction 诊断见下节。


## Review 修复与四距离 planner 诊断（2026-10-13）

范围严格固定 B / 1.70 m / seed0；独立重启四次，目标相对启动后 fresh SLAM map→base_link 的初始 pose，未发送 NavigateToPose，未改 baseline 参数、SLAM 或 policy。`success=false` 在这些原始 result 中表示没有进行完整导航验收；planner 的结果看 `planning_status` / `global_path_generated`。后续 harness 显式增加 `navigation_attempted`，不把 planning success 当作物理通过。

Harness 修复：

- `scripts/run_narrow_baseline.sh:2` 仅 `set -o pipefail`。`course_main` 对定位目录、cd、ROS source、workspace source 分别使用 `if` + FAIL + `return 1`，无 set-e / exit。子进程直接使用已加载环境的 `/usr/bin/python3 /opt/ros/humble/bin/ros2 launch course_bringup course.launch.py`，避免再经过带 set-e 的 run_course.sh；同 domain、DDS profile、OMP=2 与原 launch 参数。
- `scripts/narrow_fixture.py:81` 的关键逻辑为 `reason.startswith('planner_') -> planning_failure`、`reason=='succeeded' -> none`，其余由实际 end_reason 映射；移除了 `if controller_abort: return 'controller_abort'`。observed status 即使出现 6，也不能更改 navigation timeout / unattributed abort 的 primary 分类。未实现可靠的 NavigateToPose→FollowPath UUID/terminal-cause 关联，因此保持 null，而不是猜测 controller failure。
- `scripts/run_narrow_baseline.py` 保留 raw status UUID、原始 stamp、数值 status；名称改为 `controller_abort_observed` / `controller_action_status_observed`。warnings/progress 日志仍只是观测。无全局 topic/TF 或消息修改。
- `course_navigation/launch/navigation.launch.py` 仅在 planner-only harness 传入 `B2W_NARROW_NAVFN_DEBUG=1` 时给 planner 加 `rclcpp:=debug` 和 `planner_server:=debug`；NavFn 内部 logger 名为 rclcpp，只开启 planner_server logger 无法捕获内部 calcPath 分支。默认 launch 行为与 YAML 保持原样。

可重复的单距离入口（每次单独执行，非 sweep）：

```bash
scripts/run_narrow_baseline.sh --planner-diagnostic --goal-distance 6.0 \
  --ros-domain-id 97 --output-dir /tmp/b2w-navfn-diagnostic/distance-6
```

`--planner-diagnostic` 强制唯一 B/1.70m/seed0，distance 只允许 1/3/4.5/6；不能和 --smoke 合并。每次先等待实际 lifecycle active、新鲜定位与地图，再请求 live master `/global_costmap/get_costmap`、保存 raw snapshot、调用 ComputePathToPose、保存 after snapshot、取消自己的 action 并清理 owned processes。after snapshot 返回后规划失败也会保留证据。所有结果记录 pre-navigation action wall roundtrip；它包含 action 通信与调度，不冒充纯算法耗时。

### 每距离实测

证据根目录 `/tmp/b2w-navfn-diagnostic/`；完整机器可读表为 `planner-summary.json`。每个 `distance-*/<timestamp>/B-w1.7000-seed0/` 保存完整 `costmap-before-plan.json` / `costmap-after-plan.json`、`costmap-diagnostic.json`（中心线每样本cost与histogram、floor/round cells）、`preplan.json`、`result.json`、原始 `runtime.log` / `ros-log/planner_server*.log`、`observations.jsonl`、摘录 `navfn-debug.log`、`cleanup.json`。快照由实际在线 costmap 服务返回，不是几何生成图。NavFn 实际 start/goal cells 用 runtime DEBUG 核实；start/end 代价从实际 raw snapshot 读取。

| 相对目标 | ComputePathToPose | planner path 长度 | wall roundtrip | NavFn start→goal cells | raw start/goal cost | 中心线 unknown/253/254 |
|---|---|---|---|---|---|---|
| 1.0 m | SUCCEEDED (4) | 1.007098 m | 3.866017 ms | (193,160)→(213,160) | 0 / 0 | 0 / 0 / 0 |
| 3.0 m | SUCCEEDED (4) | 3.181549 m | 5.703110 ms | (193,160)→(253,160) | 0 / 112 | 0 / 0 / 0 |
| 4.5 m | SUCCEEDED (4) | 5.110684 m | 8.221310 ms | (193,160)→(283,160) | 0 / 85 | 0 / 0 / 0 |
| 6.0 m | ABORTED (6) | null | 7.408065 ms | (193,160)→(313,160) | 0 / 0 | 0 / 0 / 0 |

原始 6m result 的 `planning_latency_ms=null` 是旧字段仅填成功的格式，`planning_request_latency_ms=7.408065...` 实际测量已保存；本轮最终代码统一把完成请求的 roundtrip 写入两字段，未重写旧证据。

| 相对目标 | 整张128000-cell raw map unknown / 253 / 254 | 中心线cost范围 | before→after数据变化cell数 |
|---|---|---|---|
| 1.0 m | 105828 / 9431 / 798 | 0 | 0 |
| 3.0 m | 105833 / 9432 / 796 | 0..130 | 0 |
| 4.5 m | 105836 / 9427 / 804 | 0..151 | 0 |
| 6.0 m | 105823 / 9432 / 801 | 0..151 | 0 |

整图有unknown/障碍，不宣称整图全free。centerline清空只说明采样可用，不保证gradient extraction成功或全footprint安全。四次独立启动的地图略有差异，所以不能将本表当成同一冻结地图下的精确距离失败阈值；本轮距离组首次失败为6m。物理通过未测试。

### 安装版语义、exact failure 与回放

`source/installed-version.txt`：`ros-humble-nav2-navfn-planner 1.1.20-1jammy.20260915.232217`。实际库 `/opt/ros/humble/lib/libnav2_navfn_planner.so` 中可见 PathCalc debug 文本；安装 header 与官方1.1.20 header校对，版本/库/header/source SHA256 保存于 `source/sha256.txt`。官方文件保存为 `/tmp/b2w-navfn-diagnostic/source/navfn.cpp` 和 `navfn_planner.cpp`。

两个阶段不能混同。`source/navfn_planner.cpp:267` 调用 `planner_->calcNavFnAstar();` 计算potential；`:279` 判断 `potential < POT_HIGH` 找到合法endpoint，然后 `getPlanFromPotential` 才提取路径。`:398` 的预算是 `4 * max(size_x,size_y)`，`:401` 调用 `planner_->calcPath(max_cycles)`，其返回0就失败。400×320 cells意味着1600步；合法endpoint potential不是成功path的保证。`worldToMap:493` 用 `std::round`，与 raw costmap 的 floor cell 各自保存，不能用 floor 坐标替代 NavFn 实际cell。[官方1.1.20 planner源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_navfn_planner/src/navfn_planner.cpp)

6m live DEBUG 的 exact 序列见 `distance-6/20261013T082832.915846Z/B-w1.7000-seed0/navfn-debug.log`：propagation 使用140 cycles、4609 cells，起始 extraction potential 7125.5；最后 `[PathCalc] No path found, path too long`，之后才是 legal-potential/path-failed ERROR 和 action ABORTED。没有 Zero gradient / high potential / Out of bounds 终止日志。4.5m有 oscillation detected/fix 后成功，6m没有触发该fix。

`source/navfn.cpp:915` 的关键返回片段：

```cpp
RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, path too long");
return 0;
```

该行位于 n 次循环之后，证明这里的“path too long”实际为 **extraction cycle exhaustion**，并非测得几何路线超过某个米数。[官方1.1.20 NavFn源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_navfn_planner/src/navfn.cpp)

进一步用 `source/replay_installed.cpp` **链接安装版库**回放各raw snapshot，保持 A*、allow_unknown=false、默认pathStep=0.5 cell、原1600预算、原wrapper清理robot cell与反向endpoint设置。没有编译修改版NavFn，没有改变算法。保存完整potential / gradient arrays 和 path.csv。四次 replay 的propagation cycles、path steps与live debug吻合：38 / 126 / 201 / 0；6m `propagation_success=true, target_potential=7125.48, extraction_return=0, recorded_steps=1600`。

6m回放证据 `extraction-analysis.json` / `installed-replay.path.csv`：

- extraction 从目标反向回溯到起点；第105步起，剩余1496步全困于 cell square x∈[262,263)、y∈[162,163)，没有继续接近起点cell(193,160)。最后100步仍每次约移动0.5 cell，却局限在约0.547×0.593 cell区域。
- 该cell四角已安装NavFn给出的normalized gradients为 `(0.946864,0.321634)`、`(-0.647371,0.762175)`、`(-0.263010,-0.964793)`、`(-0.254059,-0.967189)`。其bilinear插值有stationary point `(262.502761,162.359893)`，map≈(3.125138,0.117995)，相对初始化SLAM起点≈(3.467339,0.118080)m，即choke内部。
- `source/navfn.cpp:805` oscillation判据使用当前path坐标和两步前坐标的x/y**完全相等**；最后100步对应比较命中数=0。`:893` 的 zero-gradient 也是精确 x/y==0。`:899` 使用固定pathStep除以梯度norm再移动。回放显示轨迹绕插值驻点持续移动，既没有exact两步重复，也没正好采到exact零梯度，于是耗完1600步。这里描述的是局部subcell振荡陷阱，不是已命中“Zero gradient”失败分支。
- trap 3×3 raw cost为 `[[111,112,112],[96,96,96],[0,83,83]]`，均<253且不是unknown；其3×3 potential均有限。原raw map中start→goal还存在120步、约6m的4连通可通行cell路径。该图不存在把两端切断的253/254/unknown几何阻塞，但不保证机器人全身通行。

`navfn-extraction.png` 为真实raw cost、回放path与finite potential/gradient可视化。`source/analyze_extraction.py` 为只读分析脚本。live故障分支已直接确认；trap机理来自安装库的相同快照回放，**不是live内存trace**。before/after map.data字节一致、propagation统计与成功path步数吻合，支持回放关联，但service快照与plugin拷贝不是原子操作。没有扩大live测试或修改NavFn来强行消除这一测量边界。

分类：**本次证据支持该实际cost field触发的 NavFn gradient path extraction 限制**，而非中心线blocked/unknown导致的costmap几何断路。尚未证明为何在线cost field与A* potential在此形成这种插值驻点，也未证明是独立于输入的上游实现bug。issue [#4655](https://github.com/ros-navigation/navigation2/issues/4655) 只作相似报告参考，不能作为本例根因证据。

当前不适合把正式 A/B sweep 当作通过能力 benchmark：harness两项review已修复，planner失败可记录，但6m baseline仍有可复现extraction失败、A路线偏好和真实B全程通过还未验证。先审查本bundle，用户明确授权后再决定有限follow-up或正式实验；本轮不进入参数调优。

最终检查：32项unit tests PASS（新增observed-abort分类、round/floor与snapshot分布、workspace source失败、planner-diagnostic强制单case回归），shell语法/compile/diff检查通过。4次各22个owned PID/starttime identities，`remaining={}`，最终没有仍匹配的live owned PID；baseline SHA256与HEAD及四个manifest一致，policy SHA256与四个manifest一致，详见 `final-audit.json`。bundle生成后停止，不commit/push。

## Extraction budget 与 Smac2D 离线follow-up（2026-10-13）


结论：**增加 extraction cycle budget 不能解决此case**。该结论限于保存的 Scenario B / width=1.70m / seed=0 / 6m raw costmap、Humble 1.1.20 安装库与原 A* potential；不泛化到所有 NavFn 场景，也不证明任何机器人通道物理通过能力。

### 证据输入与执行边界

已读 AGENTS.md、PLAN.md、docs/NARROW_BASELINE.md、CODEX_GUIDE.md 和现有 installed-library replay。最新 bundle 记录为 `/home/zyd/APRIL_REVIEW_BUNDLES/APRIL_REVIEW_BUNDLE_b2w-embodied-navigation-course_20261013_164150.md`（旧 `bundle.log`），但本轮该文件不存在且目录为空，无法重读其内容；未把旧生成日志当作本轮已审查 bundle。

第一阶段输入：`/tmp/b2w-navfn-diagnostic/distance-6/20261013T082832.915846Z/B-w1.7000-seed0/replay.raw.u8`，400×320 cells，resolution=0.05000000074505806m。SHA256 `f5b9df0a004fff0ba54d678dcd867cdab8228668d1bf575abded3b0165f82412`。原 NavFn robot cell=(193,160)，target cell=(313,160)，wrapper 反向设置内部 start/goal；清空 robot cell 的行为与原 replay 一致。

复制原 `source/replay_installed.cpp` 到本轮 `/tmp`，只把固定预算改为 CLI max_cycles，并增加 steady-clock calcPath 计时。仍链接 `/opt/ros/humble/lib/libnav2_navfn_planner.so`；未编译修改版算法。raw costmap、A*、allow_unknown=false、安装版 cost conversion、pathStep=0.5 cell、start/goal、potential propagation 均不变。各次 potential SHA256 与原文件逐字节一致，完整 path prefix 与原1600及上一 budget 一致；target potential=7125.48，propagation success=true。所有 budget 都是独立新实例。

本轮无 Gazebo、无 NavigateToPose、无正式 sweep、无 baseline 或 policy 修改，无 inflation / DWB / vy / cost_travel_multiplier 调参。第二阶段仅离线调用已安装 Smac2D 的 createPlan；没有 ComputePathToPose action server，也没有重新运行 SLAM/controller/local costmap。保留上轮真实 SLAM raw master 快照，不重建或再次 inflation。

### 1. 各 max_cycles 结果

| max_cycles | extraction | 返回steps / 已记录steps | calcPath wall ms | 最后cell | 最后subcell(x,y) | 最近cell | min到start cell(m) | unique floor cells | trap总steps / 第105步后持续steps | exact fallback / zero-gradient | 成功path length |
|---|---|---|---|---|---|---|---|---|---|---|
| 1600 | FAIL | 0 / 1600 | 0.133025 | (262,162) | (262.281403,162.231430) | (261,162) | 3.437555 | 54 | 1498 / 1496 | 0 / 0 | null |
| 3200 | FAIL | 0 / 3200 | 0.238491 | (262,162) | (262.239380,162.384796) | (261,162) | 3.437555 | 54 | 3098 / 3096 | 0 / 0 | null |
| 6400 | FAIL | 0 / 6400 | 0.472992 | (262,162) | (262.335602,162.600327) | (261,162) | 3.437555 | 54 | 6298 / 6296 | 0 / 0 | null |
| 16000 | FAIL | 0 / 16000 | 0.631096 | (262,162) | (262.302887,162.521896) | (261,162) | 3.437555 | 54 | 15898 / 15896 | 0 / 0 | null |
| 32000 | FAIL | 0 / 32000 | 1.311363 | (262,162) | (262.567657,162.053391) | (261,162) | 3.437555 | 54 | 31898 / 31896 | 0 / 0 | null |

16000失败后才追加32000。所有 budget 的 still_trapped_262_162=true；第105步到最后全部在 `[262,263)×[162,163)`，没有逃出时刻。trap总steps还包含永久困住前两次短暂进入，因此与持续steps差2。最近位置是 `(261.707245,162.455429)`，到整数 robot start cell 为68.751107 cells / 3.437555m；此后 budget 增加没有改善最接近距离。路径长度为null，因为没有成功path；绕trap的累计行程不作为有效路径长度。

wall runtime 主列只测 calcPath 调用（含末尾 DEBUG，排除文件I/O/程序启动）。完整进程墙钟另保存在 budget-results.json，约10.68/12.75/14.39/16.14/23.95ms，不是 action roundtrip，也不是性能基准。DEBUG 全部为 `No path found, path too long`，无 exact oscillation fix、Zero gradient、高potential或越界终止。确认这次1600 exhaustion是trap持续运动的终止条件，不能把问题归为仅需更多步数的长路线。

### 2. 当前 max_cycles_factor 语义

当前[官方 rolling 文档](https://ros-navigation.github.io/mkdocs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/)列出 `<name>.max_cycles_factor`，int默认4；[当前官方源码](https://github.com/ros-navigation/navigation2/blob/main/nav2_navfn_planner/src/navfn_planner.cpp)以最大 costmap cell 维度乘该factor，再调用calcPath。当前 parameter_handler.cpp 声明默认4，动态更新要求整数>0。

400-cell最大维度对应 factors 4/8/16/40/80，正是本轮1600/3200/6400/16000/32000。**对本case，单独增加factor不是已得到支持的最小解决方案**；在测试范围内只延长同一trap循环。没有测试新版完整NavFn，不把这个结果扩展为新版实现必然失败，也不声称数学上证明无限budget永不逃离。

[官方1.1.20源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_navfn_planner/src/navfn_planner.cpp)仍硬编码 `4 * max_dimension`，没有此参数。本机安装为 `ros-humble-nav2-navfn-planner 1.1.20-1jammy.20260915.232217`。**未将rolling参数或新版plugin拼写写入Humble配置**。

### 3. SmacPlanner2D 执行方式与配置

16000仍失败后创建 `course_navigation/config/nav2_smac2d_diag.yaml`，与baseline文本只有一行差异：GridBased.plugin从 `nav2_navfn_planner/NavfnPlanner` 改为 `nav2_smac_planner/SmacPlanner2D`。完整YAML结构比较验证其余键和值一致，包括global/local costmap、footprint、padding、inflation、allow_unknown=false、controller与原policy；SLAM源码/config不变。原 `use_astar: true` 保留，Smac不消费这个NavFn专用键。

先检查 `ros2 pkg executables nav2_smac_planner`（空：这是plugin包，无独立可执行）、pkg prefix、apt版本与本机 `smac_plugin_2d.xml`，实际pluginlib name为斜杠名称、C++ type为 `nav2_smac_planner::SmacPlanner2D`。安装包 `1.1.20-1jammy.20260915.232226`。通过pluginlib实际加载安装库 `libnav2_smac_planner_2d.so`，执行configure/activate/createPlan/deactivate/cleanup。

`FrozenCostmap`只是离线输入适配器：创建Costmap2DROS壳和LayeredCostmap内存，设置原frame/unknown/footprint/padding，载入已inflated raw master bytes；不configure/activate成本地图层或生成在线地图。所有primary replay的raw byte输入和输出SHA256相同。footprint padding后为约±0.75/±0.43m，与baseline一致。[Humble Smac2D源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/smac_planner_2d.cpp)的2D collision checker固有地使用radius/inscribed语义；这不是全矩形/腿部包络的物理通行验证。

Smac固有默认参数（来自实际parameter dump）原样使用：cost_travel_multiplier=1.0、downsample_costmap=false、max_iterations=1000000、max_on_approach_iterations=1000、max_planning_time=2.0及内部smoother默认值；tolerance沿用0.3、allow_unknown沿用false。未试多个值，也未写新调参字段。

### 4. 1/3/4.5/6m planner-only结果

以下primary tests用各距离上轮保存的真实raw snapshot和已记录的SLAM map请求世界坐标。status是插件createPlan返回非空path且无exception，action_status=null；planning latency是createPlan steady-clock耗时，包含Smac内部smoothing，不与旧NavFn action往返直接比速度。

| 距离 | Smac插件status | poses | path length(m) | createPlan latency(ms) | Smac floor start→goal cells | 整图unknown/253/254 | path顶点unknown/253/254 |
|---|---|---|---|---|---|---|---|
| 1.0 | SUCCESS | 21 | 1.000000 | 0.207000 | (193,159)→(213,159) | 105828/9431/798 | 0/0/0 |
| 3.0 | SUCCESS | 61 | 3.056684 | 0.834333 | (193,159)→(253,159) | 105833/9432/796 | 0/0/0 |
| 4.5 | SUCCESS | 91 | 4.557671 | 0.902524 | (193,159)→(283,160) | 105836/9427/804 | 0/0/0 |
| 6.0 | SUCCESS | 121 | 6.065065 | 1.084256 | (193,159)→(313,159) | 105823/9432/801 | 0/0/0 |

落格边界：NavFn使用round，Smac使用Costmap2D::worldToMap的floor。同一已记录世界坐标下，NavFn全部起点y=160，而Smac起点y=159；6m goal也为y=159。上轮action use_start=false，live插件实际取TF的时刻与记录初始pose不是原子关联，不能宣称保存的double坐标完全等同live插件输入；NavFn第一阶段则使用由live DEBUG确认的原整数cells，完全一致。

为确认成功不只依赖上述落格差异，追加一个单次离线6m同cell control：同一6m raw，将Smac起终点置于原NavFn `(193,160)→(313,160)` 的cell中心，无配置/代价改动。结果SUCCESS，121 poses，length=6.044869m，createPlan=0.982536ms，raw字节仍不变。control世界坐标与primary不同，独立标明，不替代同世界坐标的primary表。

**SmacPlanner2D在该固定case成功生成path，而NavFn失败。** 这只说明一个冻结case避开了NavFn gradient extraction trap；不能宣称Smac整体更优。四份snapshot略有差异，不是同图的距离阈值sweep。path顶点代价统计不保证整条连续路径/矩形全包络净空。

### 5. 正式 planner 对比关卡

尚未进入，也不把本轮诊断当作正式improved benchmark。现有证据足以支持review后提出受控planner对比，但尚无Smac在线ComputePathToPose、online SLAM/costmap运行一致性、A路线偏好、多seed结果或真实通道通过/碰撞验证；因此本轮不宣布正式planner benchmark已就绪。未开始cost/inflation/DWB/vy调参、实际通道导航或A/B sweep。

### 6. 验证与清理

主运行采用隔离ROS_DOMAIN_ID=186；5个primary/control调用各登记1个owned PID/starttime，均正常退出0，remaining={}。最初sandbox试调用的DDS初始化报告socket/接口权限错误，但直接插件仍返回path；原日志保存在smac2d/sandbox-attempt。为消除该初始化缺口，随后在授权host环境重复相同输入、配置和installed plugin，作为primary表；5次primary/control日志无DDS初始化错误，路径与sandbox同输入结果一致。这不是正式重复benchmark。

全程无set-e/set-euo测试wrapper或pkill-f，无导航action。NavFn是同步短命child，无后台任务；Smac用既有OwnedProcesses，只处理自己的token+PID identity。最终baseline/policy/原harness/安装库完整性和owned残留复核见final-audit.json；测试、语法、compile与diff检查结果见tests.log/checks.json。只新增诊断YAML并更新本PLAN/实验说明，原baseline不变。不commit、不push。

### 7. Bundle

本报告、机器可读结果、source、logs、cleanup、hash manifest与项目diff在最终bundle调用前备齐。用户指定~/bin/april-review-bundle含set-euo；调用时仅过滤这一开关行，在同一Bash脚本体中执行其余原逻辑，原工具文件不改。bundle生成后立即停止，不回写文档；实际路径以最终工具输出和最终回复为准。

## 固定case Online planner A/B 与单次Smac导航（2026-10-13）

**在B/1.70m/seed0/6m这一固定online case中，SmacPlanner2D完成规划并使导航成功通过。** 本轮NavFn在线ComputePathToPose也SUCCEEDED，没有复现旧冻结快照中的NavFn失败；因此没有在线成功率优势证据，不宣称Smac整体更优、最优planner或最小可通行宽度。

### 范围、输入与执行协议

证据根目录 `/tmp/b2w-online-planner-ab/`。用户指定的最新review bundle `APRIL_REVIEW_BUNDLE_b2w-embodied-navigation-course_20261013_165953.md` 在执行前不存在，目录只有另一项目bundle；保留这个审查输入缺口，没有声称读过该文件。已读取AGENTS/PLAN/本说明/CODEX_GUIDE与接口文件，环境ready和rl_sar bundle验证通过。

唯一fixture为Scenario B / width=1.70m / seed0 / relative distance=6.0m，world spawn=(0,0,1)，choke x∈[2,4]。三次fixture.world SHA256完全一致：`8589e21db500abc3d9c59257fd45949aed39275eb132467383309264b0c276f8`。独立启动ROS_DOMAIN_ID=187/188/189，各有唯一IGN_PARTITION和owner token；Gazebo真值只评价，不进入SLAM/Nav2。目标由各次fresh SLAM map→base_link初始pose/yaw构造，不复用另一次启动的map坐标。保存 `start_map_xy_yaw`、`start_truth_eval_xy_yaw`、完整 `planning_start_tf_estimate`、`goal_pose_world_map` 和 floor/round cells。由于ComputePathToPose use_start=false，probe起点是TF估计，不是插件原子输入；NavFn整数cells另由runtime DEBUG核对。

readiness一致：planner/controller/BT实际lifecycle active=[3,3,3]、action/service可用、fresh TF age≤0.5s、SLAM odom接收age<0.5s且stamp age≤0.5s、online /map stamp age≤3s；就绪后参数只读dump、2秒settled interval并再次fresh检查。raw master由 `/global_costmap/get_costmap` 返回，保存JSON与原始u8 before/after，既有inflation只在真实costmap链中运行；不是几何生成图或冻结替代输入。service快照不是planner拷贝的原子trace。

`--nav-params-file`仅选择现有baseline或已有Smac诊断YAML，两文件本轮均未修改，文本仍只有plugin一行不同。footprint/padding/inflation、DWB、SLAM配置、locomotion policy均保持原样；vy三段仍为0。实际Smac cost_travel_multiplier=1.0为安装插件原默认值，未设置或试其他值。runtime dump还确认所有非planner节点参数一致。

### 阶段1：真实online planner-only

每次仅一次ComputePathToPose，无NavigateToPose；action终态与插件加载日志、原始result/path、耗时和raw地图均已保存。planner-only raw result中的success=false表示未进行导航，判断规划应看planning_status=4。

| 配置 / domain | 实际加载插件 | ComputePathToPose | path长度(m) | 墙钟往返(ms) | 整图unknown / 253 / 254 | 中心线unknown / 253 / 254 |
|---|---|---|---|---|---|---|
| NavFn / 187 | nav2_navfn_planner/NavfnPlanner | SUCCEEDED (4) | 6.154882 | 9.383127 | 105832 / 9427 / 794 | 0 / 0 / 0 |
| Smac / 188 | nav2_smac_planner/SmacPlanner2D | SUCCEEDED (4) | 6.053571 | 10.845375 | 105835 / 9423 / 797 | 0 / 0 / 0 |

两次各121个中心线样本cost范围0..151，完整cost/histogram/cells见各 `costmap-diagnostic.json`。NavFn实际robot→target round cells=(193,160)→(313,160)，runtime内部反向设start/goal；Smac基于保存世界pose推得floor cells=(193,159)→(313,160)，同样保留NavFn round映射，不混用两种语义。

| raw bytes SHA256 | before | after |
|---|---|---|
| NavFn | `6c419256e2b945c28934568d5f373b1ffc6220b84a26db994589abdefb7cd132` | 与before一致 |
| Smac | `0d15a30c5891219c99f65eed6ebd47735ad178e6f8e2ef4154e8eb03c04fb183` | 与before一致 |

同400×320/resolution/origin栅格，独立启动之间**527/128000 cells不同（0.4117%）**；相对start route neighborhood x∈[-0.8,6.8], |y|≤1.3共有7904 cells，其中82不同、2个unknown转换、4个不可通行分类转换。choke中心x∈[2,4], |y|<0.85共有1360 cells，其中40不同、max |cost delta|=83、1个不可通行分类转换。两次pose导出的区域统计一致；坐标/阈值和计算代码见 `analyze_planner_ab.py`。

**输入cost field差异是直接planner优劣对比的confound**，不能因为整图百分比小就假定NavFn对通道梯度变化不敏感。本轮只确认两者在线可完成这一各自观测case；路径长和roundtrip仅描述，不用于排名，roundtrip也不是纯算法耗时。与原6m离线raw SHA256 `f5b9df0a004fff0ba54d678dcd867cdab8228668d1bf575abded3b0165f82412` 相比，当前NavFn/Smac输入分别1221/964 cells变化；当前NavFn成功不能推翻旧冻结输入上的extraction trap证据。本轮未再次做离线replay。

原始路径：
- NavFn：`/tmp/b2w-online-planner-ab/navfn/20261013T091018.976929Z/B-w1.7000-seed0`。
- Smac：`/tmp/b2w-online-planner-ab/smac/20261013T091125.669526Z/B-w1.7000-seed0`。
- 两目录各包含runtime-parameters.json、planner_server加载/运行日志、raw前后、costmap-diagnostic、compute-path-result、preplan、result、cleanup。
- 汇总 `planner-ab-summary.json`，图 `planner-ab-costmaps.png`。

### 阶段2：单次Smac GUI真实导航

仅Smac阶段1实际成功后进入；domain189，GUI=true/RViz=true，独立相同fixture、readiness和6m相对目标。阶段2先做一次单独preplan安全关卡（SUCCEEDED，length=6.053571m，roundtrip=11.384557ms），然后仅发送**一次NavigateToPose**；BT正常重规划另计。完整路径 `/tmp/b2w-online-planner-ab/navigation/20261013T091318.131156Z/B-w1.7000-seed0`。

| 指标 | 实际值及定义 |
|---|---|
| NavigateToPose / BT | SUCCEEDED (4)；bt_navigator runtime Goal succeeded |
| FollowPath | 最终goal SUCCEEDED (4)，controller runtime Reached the goal |
| 规划及重规划 | 导航期间14次/plan发布，首条后13次；日志共15个ComputePath请求，含1次独立preplan及14次BT规划 |
| recovery | feedback max number_of_recoveries=0 |
| collision plate | rising-edge计数0，1395样本，无>50N；max force=1.118986N；不是全身覆盖 |
| SLAM odom轨迹长度 | 5.846329m，不是global plan长度 |
| 真值评价轨迹长度 | 5.889731m；1395样本，sim_world frame |
| time to goal | 13.942500仿真秒；不含用户观察hold |
| final error | 0.238153m，fresh map→base_link到map目标；不是Gazebo真值误差 |
| corridor_entered_truth / passed_truth | true / true；采样机器人中心进入并穿过gate，非全身clearance证明 |
| 定位新鲜度 | 29次样本TF与SLAM odom stamp age=0.305..0.410s；odom wall age=0.0138..0.11984s；map age=0.0075..1.0075s |
| 三段cmd最大abs vy | raw / smoothed / nav_vel均0 |
| 三段最大abs vx,wz | raw=(0.4,0.6)，smoothed/nav_vel=(0.4,0.24) |
| progress/controller warnings | 仅一次默认general_goal_checker提示；无progress checker失败或No valid trajectories日志 |

保存原始BT event logs、planner/BT/FollowPath action UUID+stamp+status、全部global plans、三段cmd、完整SLAM odom、带frame/stamp真值、全部plate样本、feedback与rosout。FollowPath共14个UUID，13个旧goal终态observed ABORTED(6)，最终goal SUCCEEDED(4)，runtime同期逐次Passing new path to controller；这些observed状态**不改写**本次NavigateToPose成功结果，primary_controller_failure=null。本轮没有独立验证各旧goal终止原因，不将ABORTED孤立归因DWB、tracking或Smac失败。BT `/status`采样窗口仅收到EXECUTING(2)，最终成功由action result和BT日志确认，未伪称topic捕获了所有终态。

本次raw measurement_scope还保留旧harness标签full_corridor_baseline；实际配置/参数/日志均为Smac诊断配置。最终代码仅修正未来online-navigation输出为fixed_online_navigation，未重写已执行raw evidence。该修正无需再跑导航。

### 人工观察与正常teardown

目标结束后保持GUI/RViz运行，保存 `result-before-cleanup.json`、窗口树与owned GUI PID证据；直到用户本轮明确回复“观察完成”才写确认信号，既有harness随后SIGINT own launch并验证退出。未提前关闭GUI。

用户人工视觉评价原样记录于 `human-observation.json`：进入窄道=是；运动平稳、有轻微减速；未观察到碰撞；global plan合理；RobotModel与地图基本对齐；真正通过choke=是；最终正常到达=是；其他为“窄道内略微变慢、转向较谨慎”。没有单独给“频繁停车”评分，保留其平稳/轻微减速原评价，不擅自补评分。自动速度样本近零只作原始描述，不当成人工停车评价。

NavFn/Smac/导航各登记22/22/26个owned PID/starttime，全部cleanup PASS、remaining={}、launch exit=0。host复核全部已登记live身份和本轮domain/owner token进程消失、Gazebo/RViz窗口关闭；域187/188/189 session lock均通过LOCK_EX|LOCK_NB获取后立即释放。lock文件按原实现仍保留旧PID文本，**可获取锁才是已释放证据**，没有删除lock文件或启动新仿真来验锁。其他用户/root不可读environ记录在审计中；只对本轮可见owned身份及所属用户进程作结论，不声称清理了所有宿主机任务。

baseline/Smac YAML、policy、SLAM YAML和此前launch观测配置SHA256与本轮开始一致。当前HEAD=193170687179a8e86d096c86eb44a632aa971fc9，最新reflog时间2026-10-13 15:21:22+08:00，早于本轮17:10测试；PLAN原历史审计引用66db515...并非本轮起始HEAD。首次host审计所有cleanup项目已通过，仅硬编码旧HEAD检查失败，保存为cleanup-audit-stale-plan-reference.json；修正审计依据为reflog时间后通过，不掩盖旧检查错误。本轮未commit/push。

最终检查包括34项pytest、Python compile、shell syntax和diff-check；日志与退出码在tests-final.log/checks.json，清理证据在cleanup-host-audit.json。没有正式sweep、Scenario A、多seed/其他width、cost_travel_multiplier/inflation/DWB/vy调参。bundle调用保留原工具逻辑，仅过滤其set-euo开关行，原~/bin工具文件不修改。唯一剩余交付动作是生成用户指定april-review-bundle后停止，生成结果与路径以最终工具输出为准；bundle之后不再回写本文件。

## Same-server / same-costmap Scenario-A planner comparison（2026-10-13）

**双插件成功加载，但本次 comparison_eligible=false：三次 raw master hash 不一致。** 仅运行一次 Scenario A / width=1.70m / seed0 / relative goal=6m planner-only short smoke；无 NavigateToPose、无运动目标、无调参、无 width/multi-seed sweep、无 commit/push。发现 confound 后停止实验，没有重试或改变 map/update_frequency。

证据根目录：`/tmp/b2w-same-server-planner-ab`。原始 case：`/tmp/b2w-same-server-planner-ab/smoke/20261013T113843.426811Z/A-w1.7000-seed0`。最新 review bundle 20261013_173737 的生成记录可读，但当前 `~/APRIL_REVIEW_BUNDLES` 为空，无法重读 bundle；已读现存 `/tmp/b2w-narrow-characterization/review.md` 和上一轮 report，按用户本轮明确结论修正 same-input 协议。唯一当前 roadmap 是根目录 PLAN.md。

### 输入与加载证明

- 单一 planner_server PID/starttime = `3315652 / 34145262`，before/after 相同；同一 launch PID=3315631、ROS_DOMAIN_ID=192、owner token=`6cdd8cbcfb564ad590de3177fcea2a8b`。planner/controller/BT lifecycle before/after 均 [3,3,3]，中间没有重启/重新配置。
- 实际日志 `ros-log/planner_server_3315652_1791891524113.log` 明确 `Created global planner plugin NavFn of type nav2_navfn_planner/NavfnPlanner` 与 `Created global planner plugin Smac2D of type nav2_smac_planner/SmacPlanner2D`，随后两 plugin activation；完整摘录 `/tmp/b2w-same-server-planner-ab/planner-loaded-and-status.log`。
- installed package 均 Humble 1.1.20；pluginlib name/C++ type 来自本机 `/opt/ros/humble/share/nav2_navfn_planner/global_planner_plugin.xml` 与 `nav2_smac_planner/smac_plugin_2d.xml`，保存于 installed-pluginlib.json。实际参数 dump planner_plugins=[NavFn,Smac2D]；Smac2D.cost_travel_multiplier=1.0 默认值，没有设置或调节该参数。
- readiness 后最终只取一次 fresh map→base_link 作为显式 start。TF age=0.337500s；odom stamp/wall age=0.337500/0.087343s，map age=0.137500s。原 TF 与 exact PoseStamped header/position/quaternion 均保存。目标按该 map pose/yaw 相对前方6m构造。两个 ComputePathToPose.Goal 去掉 planner_id 后结构完全相同，use_start=true；不重新取 TF 构造第二个请求。
- exact 请求为 `/tmp/b2w-same-server-planner-ab/smoke/20261013T113843.426811Z/A-w1.7000-seed0/NavFn-request.json` 和 `Smac2D-request.json`，唯一 ID 差别为 NavFn / Smac2D。共2个 planning_request events，navigation_attempted=false、navigation_goals_sent=0。

### 三次 live raw master snapshot

服务均为 `/global_costmap/get_costmap`，不是可视化 occupancy map、几何重建图或离线冻结图；两 plugin 共用同一个 global costmap。SHA256 仅计算完整原始 master data u8 bytes；grid geometry（size/resolution/origin）另校对一致。

| 位置 | raw master SHA256 | 相对 before 变化 cells |
|---|---|---|
| before planner 1 | `33e13df9eca0acdd072c86af6fedef0bfbe89631095b61d07ade46569771e22b` | 0 |
| between planner 1 / planner 2 | `33e13df9eca0acdd072c86af6fedef0bfbe89631095b61d07ade46569771e22b` | 0 |
| after planner 2 | `b123085d4803efdb2aad2ee5432e0122f03448f9f402b62d527f2e2ef402190c` | 316 / 128000 |

三个 .raw.u8 与 JSON 全部保存，input-and-plugin-audit.json 重新从文件计算验证 hash。**costmap变化是 confound，不能把 NavFn失败/Smac成功宣称为公平算法对比。** 服务快照并非插件内存的原子 trace；未确定变化发生于插件实际读取输入的哪个时刻。

### 两个 planner 的描述结果

| 指标 | NavFn | Smac2D |
|---|---|---|
| actual loaded ID / pluginlib type | NavFn / nav2_navfn_planner/NavfnPlanner | Smac2D / nav2_smac_planner/SmacPlanner2D |
| C++ type | nav2_navfn_planner::NavfnPlanner | nav2_smac_planner::SmacPlanner2D |
| ComputePathToPose status | ABORTED (6) | SUCCEEDED (4) |
| chosen_route | unclassified | narrow |
| returned path poses | 0 | 121 |
| geometric path length | unavailable（空 path） | 6.065064519 m |
| wall action roundtrip | 5.885809 ms | 15.109847 ms |
| raw-cost path samples | 0（无 path，counts不表示全free） | 360 |
| raw cost min / mean / p95 / max | unavailable | 0 / 2.997222222 / 0 / 83 |
| descriptive integrated raw-cost exposure | unavailable（空 path） | 18.072379085 cost·m |
| unknown / 253 / 254 path samples | 无样本（JSON counts=0） | 0 / 0 / 0 |

保留完整 `/tmp/b2w-same-server-planner-ab/smoke/20261013T113843.426811Z/A-w1.7000-seed0/NavFn-compute-path-result.json`、`Smac2D-compute-path-result.json`，包含 returned path；comparison.json 保存每个 raw path sample 的 xy/cell/cost/ds、exact explicit start/goal、status 和汇总。NavFn日志出现 `[PathCalc] No path found, path too long`，本轮不进一步回放或泛化根因。

raw-cost指标统一引用 **before-planner-1** snapshot，按2D返回path每segment分成间距≤resolution/2的子段，以中点的 floor cell 取 raw cost。min/mean/p95/max 是这些中点的未加权统计（p95 nearest rank）；exposure = ∑raw_cost(midpoint)·ds，单位 cost·m，包括255/253/254；越界则exposure=null。它不是任一planner内部 objective，也不是 footprint collision/clearance 指标。因本轮 hash变化，这个共同参考快照不能称为双方已证明相同的实际算法输入，exposure只作保存的参考图上的描述，不作公平排名。wall roundtrip包含action通信/调度，不是纯算法耗时。

### 文件、关键片段与结论关联

| 文件路径 | 关键片段 | 如何支持结论 |
|---|---|---|
| `course_navigation/config/nav2_dual_planner_diag.yaml` | `planner_plugins: [NavFn, Smac2D]`（YAML逐行列表），分别为本机slash plugin名称 | 一份诊断配置使同一planner_server创建两个plugin；没有第二个global costmap；所有非planner部分与baseline结构一致，NavFn参数逐项一致 |
| `scripts/run_narrow_baseline.py` | `tf=buffer.lookup_transform('map','base_link',Time())`；`start.header=copy.deepcopy(tf.header)`；`paired=compare_planners(...)`；随后直接 `return record` | readiness后将一次fresh TF变成固定显式start；目标按同一start/yaw构造；只传入一个planner ActionClient和一个costmap service client，返回时未进入NavigateToPose分支 |
| `scripts/narrow_planner_comparison.py` | `template.start = copy.deepcopy(start); template.goal = copy.deepcopy(goal); template.use_start = True`；`request = copy.deepcopy(template); request.planner_id = planner_id` | 两个请求完整PoseStamped及use_start相同，只有planner_id不同；运行输出二次结构比较也验证这一点 |
| 同文件 | `capture('before-planner-1')`；NavFn→between-planners，Smac2D→after-planner-2；`len(set(result['costmap_hashes'].values())) == 1` | 三个raw data hash都参与eligibility，且grid geometry另检查；本次不相等所以false，停止实验 |
| 同文件 `path_metrics` | `sum(s['raw_cost']*s['ds_m'] for s in samples)`；`sampling_definition` / `exposure_definition` | 可复核几何长度、共同参考raw图统计与descriptive exposure；没有调用或仿造两个planner的内部objective |
| `scripts/narrow_processes.py`（原样复用） | token匹配、PID starttime比较、SIGINT own launch、后续仅owned identity | 自有进程清理，不使用pkill -f；host独立审计验证残留与域锁 |

### 验证、完整性、cleanup 与 bundle

42项unit tests PASS（tests-final.log，pytest_returncode=0）；没有运行额外仿真/导航测试。诊断单case CLI回归保证A/1.70m/seed0/6m和独立dual配置，配置回归比较所有非planner结构与NavFn参数；请求回归验证三snapshot顺序、显式input、middle/after变化与geometry变化confound，metrics手算验证长度与raw积分。首次6项RED保存，随后GREEN及完整42项结果保存。

cleanup.json登记23个owned PID/starttime，remaining={}、launch_returncode=0；host独立复核 matching_owner_or_domain_processes=[]、域192锁可非阻塞获取、passed=true。未处理其他会话；353个不可读environ记录可见性边界。baseline Nav2/旧Smac诊断/SLAM YAML/原launch/policy与开始时SHA256一致，HEAD仍为 `b9acbd55b717388f2425a0bc3f6ee5323ebf32d1`；完整性证据 integrity-final.json。

review bundle将包含代码、唯一PLAN、实验说明及关键日志/汇总/精确请求/hash/cleanup。原 `~/bin/april-review-bundle` 含禁止的 `set -euo pipefail` / `exit 1`；按上一轮相同做法在/tmp生成受控执行副本，去掉该set开关，把guard的exit改为函数return，保留其余打包逻辑，原工具不修改。调用后立即停止，不再回写PLAN、不commit/push；实际bundle路径以调用输出为准。


## Frozen same-input planner replay（2026-10-13）

在完全相同 frozen online costmap、start、goal 下，SmacPlanner2D 成功生成 narrow path，而 NavFn 未生成 path。

这是本机 Humble 1.1.20 安装插件在一个固定输入上的算法行为差异证据，不能泛化为 Smac 整体优于 NavFn。本轮没有 NavFn wide path，不能称为 narrow/wide route-preference 比较或启动 cost preference 调参。

### Frozen input 与完整消息

来源：`/tmp/b2w-same-server-planner-ab/smoke/20261013T113843.426811Z/A-w1.7000-seed0/` 中实际保存的 costmap-before-planner-1.json、costmap-before-planner-1.raw.u8、NavFn-request.json、Smac2D-request.json。没有重建地图、再次inflation或生成替代输入。raw SHA256=`33e13df9eca0acdd072c86af6fedef0bfbe89631095b61d07ade46569771e22b`，128000 bytes；JSON data逐字节相同。上一轮before/between/after的size/resolution/origin一致；本轮只用before原raw，不使用after变化图。frame=map，400×320，resolution=0.05000000074505806 m，origin position=(-10,-8,0)、orientation xyzw=(0,0,0,1)。

两个原始请求去掉planner_id后完全一致，use_start=true。以下是直接传入两个createPlan的完整PoseStamped；C++从实际message回读并逐字段核对，未重建xy/yaw：

```json
{
  "use_start": true,
  "start": {
    "header": {
      "stamp": {
        "sec": 21,
        "nanosec": 800000000
      },
      "frame_id": "map"
    },
    "pose": {
      "position": {
        "x": -0.342263059846654,
        "y": -2.1484376320766273e-05,
        "z": 0.6253661716939958
      },
      "orientation": {
        "x": -0.0002004063763108634,
        "y": 0.0019433711632007129,
        "z": -2.0175364453512124e-05,
        "w": 0.999998091367559
      }
    }
  },
  "goal": {
    "header": {
      "stamp": {
        "sec": 21,
        "nanosec": 800000000
      },
      "frame_id": "map"
    },
    "pose": {
      "position": {
        "x": 5.657736935078343,
        "y": -0.0002682637193574311,
        "z": 0.0
      },
      "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": -2.0564945257404033e-05,
        "w": 0.9999999997885415
      }
    }
  }
}
```

共同Costmap2D floor start→goal=(193,159)→(313,159)。NavFn实际安装库DEBUG确认round start→goal=(193,160)→(313,160)；其内部日志start=world goal、goal=robot start，按reverse endpoint语义映射。Smac原生使用floor。落格差异是各插件算法语义，未移动world poses使其cells相同。

共同footprint='[[0.70,0.38],[0.70,-0.38],[-0.70,-0.38],[-0.70,0.38]]'，padding=0.05、track_unknown_space=true、allow_unknown=false、tolerance=0.3。安装库float解析/padding的实际padded顶点为±0.7499999888241291 / ±0.4299999959766865 m；两者逐值相同，并与安装版表示核对。NavFn use_astar=true，其余原参数不变；Smac cost_travel_multiplier=1.0及原有smoother/iteration/time默认值逐项与上一轮runtime dump一致，没有调参。

### Plugin replay 与不变性

复用前轮FrozenCostmap适配器，新增 `scripts/frozen_planner_plugin/replay_plugin.cpp` 与 `scripts/frozen_planner_replay.py`。CMake直接使用/opt/ros/humble已安装依赖；pluginlib名称来自本机XML：nav2_navfn_planner/NavfnPlanner、nav2_smac_planner/SmacPlanner2D。包版本、实际DSO/XML/binary SHA在installed-library-audit.json。

每个plugin独立fresh process、node、plugin实例和FrozenCostmap；只创建Costmap2DROS壳/LayeredCostmap内存，不configure/activate任何costmap layer，不重新inflate，不启动SLAM/Gazebo/controller/BT/action server。两者均从同一source raw重新读入。每次source-before/source-after、loaded memory/configure后before-plan/after-plan五项SHA全部等于上述33e13df9…，确认source和master适配没有变化。NavFn自身clearRobotCell行为在本输入上也没有改变bytes。

### 算法层结果

| 指标 | NavFn | Smac2D |
|---|---|---|
| plugin load / configure / activate | SUCCESS | SUCCESS |
| createPlan returned / exception | 正常返回空path / null | 正常返回非空path / null |
| createPlan success | FAIL | SUCCESS |
| returned poses / route | 0 / unclassified | 121 / narrow |
| geometric path length | unavailable（空path） | 6.065064518899199 m |
| createPlan wall time | 1.24364 ms | 1.414851 ms |
| raw cost sample count | 0（无path） | 360 |
| raw cost min / mean / p95 / max | unavailable | 0 / 2.9972222222222222 / 0 / 83 |
| descriptive integrated raw-cost exposure | unavailable | 18.072379085130063 cost·m |
| unknown / 253 / 254 samples | 无样本（JSON counts=0不表示free path） | 0 / 0 / 0 |

raw-cost样本取共同frozen before master：2D返回path每段均分为间距≤resolution/2的子段，以中点floor cell取raw cost；统计是未加权min/mean/nearest-rank p95/max。exposure=∑raw_cost(midpoint)·ds，包括255/253/254；越界则null。**这是共同frozen costmap上的描述量，不是NavFn或Smac的内部objective。** 不是连续collision检查、footprint clearance或物理通过证明。wall time为steady_clock覆盖createPlan（含内部smoothing/DEBUG/异常处理），不是action roundtrip；单次DEBUG-enabled样本不支持速度排名。

完整返回path见各plugin/replay.path.csv，完整samples见summary.json与raw-cost-samples.csv，汇总comparison-summary.json保留输入/参数/hash/cleanup和全部metrics。action_status=null，不能称为本轮live ComputePathToPose或NavigateToPose结果。

### NavFn exact installed-library failure

NavFn/runtime.log原文：

```text
[DEBUG] [1791897705.150885081] [rclcpp]: [NavFn] Setting start to 313,160
[DEBUG] [1791897705.150888746] [rclcpp]: [NavFn] Setting goal to 193,160
[DEBUG] [1791897705.151868286] [rclcpp]: [NavFn] Used 140 cycles, 4609 cells visited (37%), priority buf max 198
[DEBUG] [1791897705.151880059] [rclcpp]: [NavFn] Setting start to 313,160
[DEBUG] [1791897705.151889206] [rclcpp]: [Path] Pot fn boundary, following grid (7125.5/1)
[DEBUG] [1791897705.151959799] [rclcpp]: [PathCalc] No path found, path too long
[ERROR] [1791897705.151961585] [planner_server]: Failed to create a plan from potential when a legal potential was found. This shouldn't happen.
[WARN] [1791897705.151980013] [planner_server]: NavFn: failed to create plan with tolerance 0.30.
```

与此前same-server live abort的140cycles/4609visited及legal potential→extraction failure分支一致；旧live service快照仍不是plugin内存的原子trace；可关联历史extraction trap证据。但本轮没有提取potential/gradient或新证明subcell trap位置/机理，没有增加max_cycles或任何额外budget诊断。

### 验证、cleanup、边界与打包

新增10项input/footprint守卫测试；相关unit tests合计29项PASS（tests-final.log）。原raw缺失/hash失配/JSON raw失配/grid geometry/request start/goal/use_start/frame不一致会在plugin加载前停止。安装版float footprint回归验证原值可接受、padding变动和缺顶点不可接受。编译通过，无算法源码重编译/修改。

透明保留adapter-first-attempt：首次NavFn正常执行并返回空path/returncode0，但postprocessing将安装版float顶点误与double字面值做1e-12比较而报错，且NodeOptions日志级别没有启用process-wide NavFn DEBUG。该尝试Smac未运行；三份memory raw hash仍相同。先读取实际footprint、安装float字段及旧成功adapter输出，修正测试/校验并开启rclcpp+planner_server DEBUG后，仅重做同一输入的有效pair。总计3个短命进程（NavFn2次、Smac1次），不是sweep/新case/新输入；保留所有首次证据，没有隐去adapter错误。

有效pair PID/starttime=3381608/34763364、3381621/34763390，首次3379390/34746621；均正常returncode0，deactivate/cleanup完成，owned remaining={}。独立host审计matching_owned_or_domain193=[]；354个不可读/proc项作为可见性边界保留。无pkill-f/无set-e或exit。integrity-final.json确认9项配置/launch/policy和HEAD=9640ca95fce5c0554bbb5343754bf86deb7dbebf未变。

未验证：live导航/SLAM时序、DWB/locomotion、碰撞/全机身净空、真实通道通过、多width/seed/map泛化、路线偏好的cost objective、速度优劣、新版Nav2。上一轮记录的20261013_194403 april bundle当前不存在，目录为空，无法重读；已读上一轮report/bundle.log与本轮明确协议，不声称完成缺失bundle审查。

本轮唯一PLAN NOW为Frozen same-input planner replay。完成所有代码/文档/检查后调用~/bin/april-review-bundle受控副本：只移除禁止set-euo行，将not-repo guard的exit改为函数return，保留打包逻辑和原工具文件。显式包含完整输入审计、原raw的base64、结果/样本/日志/cleanup/integrity/tests/source diff。调用后立即停止，不commit、不push、不再回写PLAN；实际bundle路径以工具输出为准。


## Frozen replay 可复用工具与 Smac sensitivity diagnostic（2026-10-13）

本轮仅实现可复用 frozen replay，并在同一 Scenario A / 1.70m / seed0 / relative goal=6m 上执行五次 Smac cost_travel_multiplier 离线诊断。**四个值严格PASS，0.0因观察器销毁阶段SIGSEGV只有部分证据；不能宣称五case全部验收通过。** 五条已保存的final与unsmoothed path均narrow，没有wide或planning fail；NavFn沿用前轮空path FAIL/unclassified参考，没有重复运行。证据根目录 `/tmp/b2w-smac-sensitivity/`。

指定最新review bundle在 `/home/zyd/.local/share/Trash/files/APRIL_REVIEW_BUNDLE_b2w-embodied-navigation-course_20261013_212847.md` 找到并读取；与本轮分支 improve/narrow-navigation / HEAD 9640ca95fce5c0554bbb5343754bf86deb7dbebf一致。初次根目录/常用bundle目录找不到的情况不是最终审查缺口。

### 可复用入口与严格gate

`scripts/frozen_planner_replay.py` 移除 SOURCE、EXPECTED_SHA和A/1.70/seed0/6m allowlist。`--source <case-directory>` / `--expected-sha <sha256>` 必填；`--binary <replay_plugin>` 必填；`--output <dir>` 指定证据位置（默认/tmp旧目录，拒绝复用已存在的planner输出子目录）。fixture完整读取source/fixture.json，scenario/width/seed/relative goal原样写入input-audit、summary和comparison；路线分类使用fixture实际宽度。工具支持该仓库fixture的既有gate-x=3m/上方绕路分类定义，不意味着适用于任意地图布局。

source目录仍须具备原before raw/JSON、between-planners JSON geometry reference、comparison.json记录SHA、NavFn/Smac2D exact requests、fixture.json与runtime-parameters.json。不是无校验raw loader；raw不存在、SHA失配、JSON bytes不等raw、几何不等已保存参考或非法、旋转origin、请求除planner_id外不同、use_start非true、frame不一致都会拒绝。加强有限数值、完整PoseStamped字段/合法stamp/非零quaternion校验；完整xyz、xyzw、stamp和frame经C++读入/回读，不重建xy/yaw。

仍逐项核对加载的全部plugin参数与source runtime dump（含默认参数），相等集合校验同时拒绝额外或缺失key。只在显式`--planner Smac2D --cost-travel-multiplier <finite nonnegative value>`下允许这一key的预期差异；其余参数严格相等。覆盖写入/tmp本case的parameters-overlay.yaml，不改任何repo YAML。默认`--planner both`保留fresh独立pair流程；`--planner Smac2D`只加载Smac。costmap/footprint runtime contract仍严格验证，source-before/source-after与loaded/configured-before-plan/after-plan内存hash全部相等才验收；最后检查还加强source-before不符时在启动plugin前拒绝并将该hash写command.json。各planner仍独立fresh process / FrozenCostmap，无layers更新或再inflation。

本轮调用范式（五个值逐一执行，非正式sweep）：

```sh
/usr/bin/python3 scripts/frozen_planner_replay.py \
  --source /tmp/b2w-same-server-planner-ab/smoke/20261013T113843.426811Z/A-w1.7000-seed0 \
  --expected-sha 33e13df9eca0acdd072c86af6fedef0bfbe89631095b61d07ade46569771e22b \
  --binary /tmp/b2w-smac-sensitivity/build/replay_plugin \
  --output /tmp/b2w-smac-sensitivity/1.0 \
  --planner Smac2D --cost-travel-multiplier 1.0
```

### 输入不变性与回归

唯一source raw SHA256 `33e13df9eca0acdd072c86af6fedef0bfbe89631095b61d07ade46569771e22b`。400×320 / resolution=0.05000000074505806 / origin=(-10,-8) / frame=map。exact start、goal与前节完整PoseStamped一致（start=(-0.342263059846654,-0.000021484376320766273,0.6253661716939958)，goal=(5.657736935078343,-0.0002682637193574311,0)），四个严格case全部applied_request读回一致。所有五case三份内存raw摘要及起始input-audit/source最终摘要一致。0.0包装器未完成其postprocessing，source_before变量和完整message/params读回result未持久化；不得将恢复字段冒充已通过原严格gate。

回归分两层：unit test用前轮两plugin已保存响应执行重构后的完整pair入口，逐项核对旧summary（原runtime gate和hash额外字段更名/增加，cleanup改为mock），不新增plugin进程；本轮fresh 1.0的全部稳定结果字段、raw采样列表、全部实际参数、完整请求、footprint、floor cells等与旧Smac逐项一致，final path CSV逐字节一致。wall time、ROS path timestamp和fresh process身份自然不同，不声称这些非确定量相等。`reference-regression.json`列出字段与排除理由。NavFn旧FAIL仅作为原证据复用，不称本轮再次验证NavFn算法。

### 五值结果与搜索路径

| actual multiplier | strict runtime gate | createPlan | poses | final route | length m | raw min / mean / p95 / max | descriptive exposure cost·m | unknown / 253 / 254 samples | wall ms |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.0 | partial / teardown SIGSEGV | SUCCESS | 121 | narrow | 6.000000089 | 0 / 62.533333333 / 151 / 151 | 375.200005591 | 0 / 0 / 0 | unavailable |
| 0.5 | PASS | SUCCESS | 121 | narrow | 6.065168877 | 0 / 2.997222222 / 0 / 83 | 18.072379085 | 0 / 0 / 0 | 1.005270 |
| 1.0 | PASS | SUCCESS | 121 | narrow | 6.065064519 | 0 / 2.997222222 / 0 / 83 | 18.072379085 | 0 / 0 / 0 | 1.009533 |
| 2.0 | PASS | SUCCESS | 121 | narrow | 6.065168877 | 0 / 2.997222222 / 0 / 83 | 18.072379085 | 0 / 0 / 0 | 1.006195 |
| 4.0 | PASS | SUCCESS | 121 | narrow | 6.065168877 | 0 / 2.997222222 / 0 / 83 | 18.072379085 | 0 / 0 / 0 | 1.085045 |

wall time仅描述各次createPlan，包括内部smoothing与DEBUG；不作速度排名。raw-cost统计使用前节共同frozen raw与同一midpoint≤resolution/2采样方法，exposure是描述量而非planner objective或净空/碰撞证明。max raw已列在stats最后一项。0.0有240个样本；其余各360。

Humble安装plugin的unsmoothed_plan lifecycle发布器在smoothing之前发布（有subscriber才发布）。适配器仅添加进程内观察器，createPlan后spin_some保存debug message，未改planner源码、smoother参数或关闭smoothing。每case保存完整消息JSON和CSV：replay.unsmoothed.path.* / replay.path.*，对应raw samples CSV。所有unsmoothed均121 poses/narrow：0.0 length=6.000000089406967m，非零均6.207106873679644m、raw min/mean/p95/max=0/2.656/0/83、exposure=16.481329006873718cost·m。非零final几何确有smoothing改变；0.0的raw/final位置相同。不能把final的细微差异理解成宽/窄路线变化。

### 0.0异常与观察器边界

首次0.0 createPlan已返回并保存非空final/unsmoothed path、parameters.txt、三份内存raw；日志显示deactivate/cleanup/插件和costmap销毁已执行。但进程在对象/DSO teardown发生SIGSEGV（PID3445195/starttime35459798、returncode=-11），最后result.json未写出。恢复证据写 `0.0/Smac2D/summary.recovered.json`：actual multiplier=0.0从实际parameters.txt核对，其他参数仅核对到value_to_string精度；完整exact params/applied PoseStamped读回与createPlan steady-clock wall time均unavailable。strict_case_verified=false；不伪造这些字段，不把teardown failure说成planning fail或wide。

排查定位到新增观察器跨plugin/class-loader寿命保留debug message shared_ptr的风险；无core/backtrace，因此这是生命周期根因假设，非精确崩溃栈证明。修正为复制消息到可执行文件拥有的存储并在plugin cleanup/DSO unload前释放；新增configured.json提前保存runtime读回。剩余0.5/1.0/2.0/4.0四次全部正常returncode0，消息各收1条、生命周期cleanup成功、全部原gate通过。依用户只准五次，不重跑0.0；本轮五case严格验收仍未全部完成。没有为消除异常启动第六次或修改planner算法。

### 官方 Humble 1.1.20 语义与有限结论

官方[SmacPlanner2D 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/smac_planner_2d.cpp#L67-L69)声明cost_travel_multiplier默认1.0并读入_search_info.cost_penalty；本轮1.0实际加载值与原默认/reference一致。[Node2D 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/node_2d.cpp#L59-L74)的边代价为 `step_length_cells × (1 + multiplier × child_raw_cost / 252)`，正交长度1、对角sqrt(2)。[initMotionModel](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/node_2d.cpp#L87-L100)将search_info.cost_penalty赋给Node2D静态字段；文件静态initializer=2.0不是plugin运行默认。0.0只取消该项代价偏好，collision/unknown gate仍执行；未使用rolling默认。源码审计见official-source-audit.json，安装库版本/hash与前轮一致。

四个严格case全部narrow，部分0.0证据亦narrow：没有支持“cost_travel_multiplier造成此frozen input宽路绕行”的路线切换证据。已保存的0.0与非零paths具有不同cost exposure，说明观察到路径对cost权重有局部代价敏感性，不能将此等同于宽/窄路线的因果敏感性；因0.0原严格runtime gate缺失，这一点仍属部分证据。不能宣称完全验收的五值全部narrow，也不能从这一个case选最终最佳权重。

尚不能推出：其他map/width/seed/start/goal上的路线因果、Smac/NavFn整体优劣、NavFn extraction失败的根因就是这一Smac参数、live导航/SLAM/controller/locomotion行为、实际碰撞/净空/物理通过、性能排名或最佳权重。NavFn参考FAIL无路线，不与Smac成功path组成narrow/wide成功路线对照。

### Tests / cleanup / integrity / bundle

红阶段8失败/13通过，source审计后变化的prelaunch gate另有1失败/25通过红证据；绿阶段45项相关unit tests通过；保留tests-red.log、tests-green.log和未source ROS时的测试环境失败日志（修复环境后通过，非planner实验重试）。CMake两次构建均成功，只构建适配器。完成前再次运行相关tests、Python compile、shell syntax与git diff --check。

恰好五个owned plugin进程：3445195/35459798、3446263/35468087、3446283/35468123、3446302/35468160、3446319/35468196。0.0=-11，其余returncode0；全部remaining={}。独立host审计matching_owned_or_domain193=[]、passed=true，348项/proc不可读记录可见性边界。无pkill-f，无禁止set-e/exit，无Gazebo/live SLAM/NavigateToPose；无commit/push。九项保护配置/launch/policy与HEAD摘要不变。只有唯一PLAN与本docs更新当前状态。

全部交付完成后运行~/bin/april-review-bundle的/tmp受控副本：仅移除原禁止set-euo行、把not-repo exit guard改成函数return，保留打包逻辑/原脚本。包含五case原始数据、0.0异常恢复证据、完整回归/源语义/cleanup/integrity/tests、原raw base64及NavFn参考；打包后立即停止，不回写PLAN、不额外replay。bundle路径以最后调用输出为准。


## 0.0 observer-fix closeout（2026-10-14）

**0.0 strict验收PASS；observer生命周期修复在此固定case闭环。** 本轮只运行一次修复后当前代码新编译的SmacPlanner2D replay，Scenario A / width1.70 / seed0 / goal6m；0.5/1/2/4保留上一轮strict PASS，无重复运行。旧0.0 SIGSEGV证据完整保留，不将失败记录改写为成功。未启动Gazebo、live SLAM、NavigateToPose或warehouse，未改变baseline参数或planner源码；未commit/push。

最新232440 bundle已从 `/home/zyd/.local/share/Trash/files/APRIL_REVIEW_BUNDLE_b2w-embodied-navigation-course_20261013_232440.md` 读取核对。branch=`improve/narrow-navigation`，HEAD=`9640ca95fce5c0554bbb5343754bf86deb7dbebf`。本轮证据根目录 `/tmp/b2w-smac-observer-closeout/`；最终汇总同时更新 `/tmp/b2w-smac-sensitivity/diagnostic-summary.json`，上一轮partial汇总备份为本轮 `diagnostic-summary.previous.json`。旧 `/tmp/b2w-smac-sensitivity/0.0/Smac2D/summary.recovered.json` 与 -11 cleanup原样保留。

### Strict验收与observer寿命

14项验收全部true，见 `strict-acceptance.json`。source-before/source-after、loaded/before-plan/after-plan及source原值全部SHA256=`33e13df9eca0acdd072c86af6fedef0bfbe89631095b61d07ade46569771e22b`。完整applied_request的frame/stamp/xyz/xyzw/use_start/planner_id逐字段等于原source请求；actual multiplier=0.0，其他全部plugin参数与source runtime dump严格相等，costmap contract和padded footprint亦验证通过。`replay.configured.json`、`replay.result.json`、`summary.json`均已落盘，`strict_case_verified=true`。

现有C++ observer callback复制Path到可执行文件拥有的存储，`scripts/frozen_planner_plugin/replay_plugin.cpp:195` 的 `unsmoothed.reset(); raw_sub.reset();` 在`:197`的deactivate/cleanup及后续class-loader作用域结束前执行；本轮没有新增planner或observer修改。当前源码SHA=`3f9ece6695a2d9f5e6c0d5b02c8f7bbc10689bd93e7450fe726e48796a193836`；新编译binary SHA=`2d58b0e657c040eeed90b19a08cb70a5939962d9a52df61bba64f3c74ed5f186`，与上一轮修复后的binary相同。静态寿命审计、runtime Deactivating→Cleaning up→Destroying日志、最终result落盘和process returncode=0共同闭环本case teardown验证。旧故障无core/backtrace，仍不能声称证明了SIGSEGV的精确栈位置或所有异常路径寿命安全。

### 最终sensitivity表

| actual multiplier | 验收 | createPlan | poses | route | final length m | raw min / mean / p95 / max | descriptive exposure cost·m | wall ms |
|---|---|---|---|---|---|---|---|---|
| 0.0 | strict PASS | SUCCESS | 121 | narrow | 6.000000089407 | 0 / 62.533333333333 / 151 / 151 | 375.200005590916 | 0.475532 |
| 0.5 | strict PASS | SUCCESS | 121 | narrow | 6.065168877445 | 0 / 2.997222222222 / 0 / 83 | 18.072379085130 | 1.005270 |
| 1.0 | strict PASS | SUCCESS | 121 | narrow | 6.065064518899 | 0 / 2.997222222222 / 0 / 83 | 18.072379085130 | 1.009533 |
| 2.0 | strict PASS | SUCCESS | 121 | narrow | 6.065168877445 | 0 / 2.997222222222 / 0 / 83 | 18.072379085130 | 1.006195 |
| 4.0 | strict PASS | SUCCESS | 121 | narrow | 6.065168877445 | 0 / 2.997222222222 / 0 / 83 | 18.072379085130 | 1.085045 |

只有0.0行为证据来自本轮；其余四行是原保存结果，wall time不作性能排名。五值final和unsmoothed route均narrow，unknown/253/254 samples均0。0.0 final与unsmoothed各121 poses，length均 `6.000000089406967` m，raw min/mean/p95/max=0/62.53333333333333/151/151，descriptive exposure=`375.2000055909157` cost·m，240 samples；smoothing未改变pose位置。unsmoothed publisher可用，收到一条消息，完整CSV/JSON均已保存。0.0 final CSV与旧失败调用已保存CSV逐字节一致；仅补齐完整runtime/applied request/wall time与正常退出的strict证据。

完整路径：`/tmp/b2w-smac-observer-closeout/0.0/Smac2D/replay.path.csv`、`replay.path.json`；搜索路径：同目录 `replay.unsmoothed.path.csv`、`replay.unsmoothed.path.json`。对应final/unsmoothed raw-cost samples CSV均保存。nonzero unsmoothed结果仍为旧121 poses / narrow / length6.207106873679644m / exposure16.481329006873718cost·m。

**科学结论不变：本frozen case的0/0.5/1/2/4均为narrow，仅说明未观察到wide/narrow route switch。** 0.0从partial升级为strict PASS改变的是证据完整度；不选择最佳权重，不泛化其他case，不把descriptive raw-cost exposure当planner内部objective、净空或物理碰撞证明，不作速度排名。

### Tests、cleanup与交付

相关unit tests共45项PASS（frozen replay + planner comparison 34项，narrow benchmark 11项）；当前CMake configure/build PASS、Python compile PASS、git diff --check PASS。Smac/NavFn安装库和plugin XML SHA均与上一轮一致；九项保护配置/launch/policy、branch/HEAD均不变。本轮未编辑replay或tests，只闭环验证与更新文档/证据。

唯一新owned PID/starttime=`3765102/39590047`，deactivate/cleanup完成、process returncode=0、remaining={}。host独立审计本轮与历史六个identity均已退出、matching_owned_or_domain193=[]、passed=true；354个/proc environ不可读条目记录可见性边界。无额外plugin进程，无禁止shell自动中止开关/exit或pkill-f。

交付前所有验证与文档完成；最后调用~/bin/april-review-bundle的/tmp受控副本（只移除原禁止shell开关、guard改函数return，原工具不改），包含新strict证据、五值最终表、旧0.0异常、源码/binary摘要、完整请求、路径、hash、cleanup和tests。bundle生成后立即停止，不回写PLAN、不commit/push。实际路径以最终工具输出为准。
# Canonical warehouse G1→G2 benchmark — 2026-10-16

The user-selected route SSoT is
[`course_navigation/config/warehouse_route.yaml`](../course_navigation/config/warehouse_route.yaml).
This is the warehouse route; the older generated A/B fixtures below retain their
own historical purpose. The only current roadmap is the root `PLAN.md` NOW.

`scripts/run_warehouse_route.py` is a thin consumer of the existing `/goal_pose`
and `goal_manager` pipeline. Its default is offline dry-run. Live execution is
**USER-RUN ONLY**, requires literal `--execute`, and attaches to an existing idle
warehouse session. The agent has not run this live path. No live/navigation
success or across-startup physical repeatability is claimed by offline tests.

The five positions are frozen exactly in the YAML. There is no jitter, random
goal seed, initial-pose rebase or Gazebo-world transform. `0.05 m` is an XY endpoint
evaluation tolerance only. G1/G2 both use `atan2(G2.y-G1.y, G2.x-G1.x)`:

```text
yaw = -2.8966601449198057 rad = -165.9664009876583 degrees
quaternion xyzw = (0, 0, -0.9925103760697063, 0.12216035933955931)
```

The captured RViz quaternion was accidental and is excluded. G3–G5 orientations
are `null` (deferred); the critical executable cannot select these goals. It
performs Start→G1, requires navigation success and .05m staging admission before
G1→G2, and stops after G2 success or failure. No retries, extra goals, planner-only preflight action or auto-launch.
This fixes target headings; the planner can still choose a nonstraight path,
and actual G1 arrival heading still uses unchanged Nav2 angular tolerance.

Navigation sequencing requires both observed Nav2 `STATUS_SUCCEEDED=4` and
existing `goal_manager.success=true`, with available fresh boundary TF evidence.
The separate .05m evaluation never changes navigation parameters or goal poses.
After G1, failed navigation/evidence stops with `staging_failed`; successful
navigation with failed .05m evaluation stops with `staging_evaluation_failed`.
Only navigation success, complete TF evidence and `evaluation_passed=true` admit
G2. These remain separate result fields; there is no retry or correction. Overall
benchmark `success` requires G2 navigation success, its .05m endpoint evaluation,
and cleanup PASS. A missing TF/error remains null/unavailable, never zero.
The benchmark evaluates position, not an independently certified yaw tolerance.

## Map-frame authentication and limitation

**Decision: absolute map-coordinate physical reproducibility across fresh
warehouse startups is UNVERIFIED, including at .05m.** Identical spawn/world
defaults make broadly similar online maps plausible; they do not register a new
SLAM map to the original user-selected route. The route is a deterministic
command contract in `map`, not a saved-map localization solution. Do not infer
`map=(Gazebo world − spawn)` or invent an alignment from the goals.

| Code/evidence path and key snippet | Causal interpretation and limit |
| --- | --- |
| `course_sim/launch/simulation.launch.py`: `('x', 0. if scene == 'warehouse' else -6.), ('y', 0.), ('z', 1.)` | Warehouse spawn defaults are fixed, but this initializes the physical model, not an absolute public map registration. |
| `scripts/fetch_lio_sam.sh` pins LIO-SAM; existing `third_party/lio_sam/src/mapOptmization.cpp`: `transformTobeMapped[i] = 0`; at first keyframe `if (!useImuHeadingInitialization) transformTobeMapped[2] = 0` | LIO begins in its own initial estimation frame. The local downloaded source was inspected; it is not a new tracked dependency. |
| `course_slam/config/lio_sam.yaml`: `useImuHeadingInitialization: false`; `course_slam/config/adapter.yaml`: `map_alignment_xyz: [0.0, 0.0, 0.0]`, `map_alignment_rpy: [0.0, 0.0, 0.0]` | Default startup has no persisted old-map registration or absolute heading anchor. |
| `course_slam/course_slam/ground_alignment.py`: `alignment[:3, :3] = rotation`; `alignment[2, 3] = -plane[2] / length` | The floor normal/datum is estimated from that session's measured cloud. Floor fitting does not identify warehouse XY/yaw landmarks. |
| `course_slam/course_slam/pose_adapter.py`: `self.alignment = alignment`; `base = self.alignment @ lidar @ self.lidar_base`; `self.broadcast(self.map, self.odom, self.correction, ...)` | Public localization combines a per-session floor estimate, extrinsics and LIO correction. No old-map relocalization is introduced here. |
| `docs/INTERFACE_ALIGNMENT.md`: online origin uses initial pointcloud ground; `CODEX_GUIDE.md` §5: fixed maps still require SLAM registration to the old map | Project conventions explicitly distinguish online mapping from registered fixed-map reuse. |
| Historical `docs/NARROW_BASELINE.md` online planner A/B section: `目标由各次fresh SLAM map→base_link初始pose/yaw构造，不复用另一次启动的map坐标` | Prior fixture runs used fresh-start-relative targets. Their grid differences do not establish warehouse landmark error or certify these absolute coordinates. |
| `/tmp/b2w-ground-gate-20261016/report.md`: known-good ground z≈.8011m versus fresh .800939415738m; prior failure unresolved | Existing zero-motion evidence establishes readiness in one rerun. Ground z alone cannot quantify cross-session warehouse XY/yaw reproducibility. |

No historical landmark registration, paired same-physical-waypoint residuals or
old-map transform with error bounds was found in the relevant saved evidence.
That is an evidence gap, not proof that the coordinates always drift by >.05m.
Reversal requires independent registration evidence across fresh warehouse
startups bounding physical landmark/route error at the required tolerance.

## Preserved observed route evidence

These existing local metric files were read, hashed and copied as small JSON
evidence to `/tmp/b2w-warehouse-route-20261016/historical-evidence.json` without
modifying originals. They are ignored runtime outputs and may be unavailable on
another checkout; the statements below preserve the observed scope.

| Existing `course_results/navigation/` file | Key snippet | Interpretation |
| --- | --- | --- |
| `20261016T083151.972156Z.json` | `"success": true`, `"end_reason": "succeeded"`, `"final_goal_distance_m": 0.06484485334757964`, `"goal_tolerance_m": 0.3` | Start→G1 remains a navigation success under the original .3m criterion; its .064844853m endpoint is not a pass of the new .05m staging-admission contract. |
| `20261016T083236.246776Z.json` | `"success": false`, `"end_reason": "replaced"`, `"planning_measurement_status": "aborted"` | User-observed G1→G2 is the first known failure segment; the recorded final manager reason is replacement, not proof of a terminal NavigateToPose ABORTED result. |
| `20261016T083310.822671Z.json` | G3: `"end_reason": "replaced"` | Issued from the preceding failure state, not an independent failure trial. |
| `20261016T083322.077354Z.json` | G4: `"end_reason": "aborted"` | Same dependent sequence; do not count as an independent route failure. |
| `20261016T083330.878149Z.json` | G5: `"end_reason": "shutdown"` | Same dependent sequence; shutdown is not an independent traversal result. |

## Implementation decisions and evidence

The major choices were argued against before support during implementation:

| Choice | Strong argument AGAINST | Argument FOR | Neutral verdict / biggest unknown / reversal condition |
| --- | --- | --- | --- |
| Freeze absolute route | Unregistered fresh maps may send identical numbers to different physical places. | Replacing user-selected coordinates with relative targets would change the experiment. | Freeze verbatim and mark comparability unverified; unknown is original-to-fresh registration; reverse on independent registration evidence, never a guessed transform. |
| Reuse `/goal_pose` + manager JSON | File IO is not transactional; concurrent goals can contaminate association and cancellation. | Existing manager owns Nav2 actions, timeout, planning measurement and metrics. A second action/recorder stack would duplicate these. | Add minimal boundary metadata and exact header+pose association; unknown is user-run timing; reconsider if the existing interface cannot associate requests reliably. |
| Require .05m G1 staging admission independently of navigation success | This can stop a run despite Nav2/manager success, including the historical .0648m offset. | The user explicitly requires the critical segment to begin within the canonical .05m staging neighborhood, without tightening controls. | Preserve navigation/evaluation flags; stop with staging_evaluation_failed before G2 when only evaluation fails. Historical navigation success remains true. Unknown is live admission frequency; change only under a revised explicit staging contract. |
| Require terminal cancellation evidence | UUID may be unavailable when cancellation precedes acceptance; conservative cleanup can fail even with no remaining action. | A Trigger response only acknowledges manager callback completion. Claiming stopped navigation from that alone is false. | Track newly observed UUIDs; confirm terminal status or report FAIL. Unknown is acceptance timing; simplify only if a future identity-aware interface supplies terminal acknowledgment. |

| Implemented path and key snippet | Causal interpretation |
| --- | --- |
| `scripts/run_warehouse_route.py`: `if not execute: ... return report` before `(transport_factory or RosGoalTransport)()` | Default path never constructs transport or imports ROS; no goals/commands are emitted. |
| Same file, `critical_goals`: `for g in (a, b)`; G1 branch: `if not segment['evaluation_passed']: ... staging_evaluation_failed`; break after critical outcome | G3–G5 cannot enter the execution sequence, including after success or failure. |
| Same file, `find_metrics`: `data.get('goal_pose') == sent`, excluding preexisting paths | Same XY with a different stamp/orientation is not accepted as this run's result; duplicate matches fail. Partial JSON writes are retried. |
| Same file, `manifest`: `route_sha256`, `config_sha256`, `head`, `source_sha256`, optional `warehouse_world_sha256` | Provenance records exact on-disk route/config/source content and HEAD. File hashes do not prove the already running stack loaded those files; the manifest states this limit. |
| `course_navigation/course_navigation/goal_manager.py`: `self.goal_message = message_to_ordereddict(msg)`; `self.nav2_status = status` only in `result()` | Exact stamped request is recorded; raw Nav2 status is observed, not inferred from a timeout/replacement string. |
| Same file, `map_base_snapshot`: `if 0 <= age <= 0.5`; record `start_map_base_link`, `end_map_base_link` | Stamped complete map→base_link transforms are sampled at manager admission/finish; stale/unavailable boundary evidence stays null. This is not a continuous TF recorder. |
| Same file: `record = self.active.finish(...)`; harness retains `goal_manager_metrics` and source path/SHA | Existing path length, collision, time and planner metrics are reused verbatim, with their original missing-data semantics. |
| `scripts/run_warehouse_route.py`, `attach_monitors`: `qos_profile_sensor_data`; `close`: `all(s in (4, 5, 6) ...)` | Monitor accepts the same best-effort goal sources as the manager; cancellation acknowledgment alone is insufficient. Foreign goals suppress global cancel. |
| Same file, `send_and_wait`: pending obligation remains unless raw `nav2_status` is terminal or reason is `rejected` | Manager timeout/clock-reset metrics do not discharge asynchronous action cancellation. |
| `course_navigation/package.xml`: `<exec_depend>rosidl_runtime_py</exec_depend>` | The metadata converter is explicitly declared, avoiding reliance on an incidental transitive installation. |

`report.json` is checkpointed with an atomic rename, including before publish.
`prepared_pose` means a stamped request was prepared; `sent_pose`/`sent_poses`
mean `Publisher.publish()` returned (not proof of DDS delivery). The manager's
matching `goal_pose` is evidence of admission. Exact fields are retained without
rounding. A route snapshot is a generated run artifact, not a second editable
route config. Reports contain no replay/tuning logic.

The lightweight attach preflight checks one manager, active Nav2 lifecycle,
discovered status publisher, observed idle status where available, goal subscriber
discovery and fresh map TF. An unused action server may have no initial status
sample. **Prerequisite: fresh idle warehouse and exclusive goal production.**
Do not click RViz goals or run another action client during execution. Topic
monitoring cannot provide a transactional exclusivity lock; unknown legacy goals
queued before attachment are not authenticated by a missing status sample.
On interference the harness stops and avoids canceling an observed foreign goal.
On its own interruption/error/manager timeout it requests cancellation and waits
boundedly for owned terminal action status; absent evidence is cleanup FAIL.
It destroys only its ROS resources, never launches or signals session processes.

## Exact commands — USER-RUN ONLY for live execution

From the repository root, offline dry-run needs no sourced ROS environment:

```bash
cd /home/zyd/program/b2w-embodied-navigation-course
/usr/bin/python3 scripts/run_warehouse_route.py --mode critical \
  --output-root /tmp/b2w-warehouse-route-dry
```

For live use, first build the changed package through the existing build script
if the checkout is not using current source symlinks. This is a build, not a live
benchmark. Follow `CODEX_GUIDE.md` prerequisites; no installs are performed by
the route harness. End an old simulation in its own terminal before a fresh
startup; do not start a second stack in the same domain. **Terminal 1, USER-RUN:**

```bash
cd /home/zyd/program/b2w-embodied-navigation-course
scripts/build_course.sh --symlink-install --packages-select course_navigation
ROS_DOMAIN_ID=91 scripts/run_warehouse.sh \
  results_directory:=/tmp/b2w-warehouse-route-metrics
```

Wait for localization/Nav2 readiness. Visually verify the frozen map-frame
targets refer to the intended warehouse corridor. This is a user check, not a
quantified cross-startup registration certificate. If correspondence is uncertain,
retain dry-run and collect registration evidence before spending a live trial.
**Terminal 2, Bash, USER-RUN ONLY:**

```bash
cd /home/zyd/program/b2w-embodied-navigation-course
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/course_sim/config/fastdds_local.xml"
export PYTHONPATH="$PWD/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$PWD/.deps/lib:$PWD/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
export OPENBLAS_NUM_THREADS=1
/usr/bin/python3 scripts/run_warehouse_route.py --mode critical --execute \
  --output-root /tmp/b2w-warehouse-route-live
```

The existing absolute `results_directory` is read from `/goal_manager` and is
required for unambiguous local file access. Runtime timeout and manager tolerance
are recorded, not overwritten. The default Nav2 configuration remains unchanged;
if the user starts an existing diagnostic config, retain that startup command/log
as loaded-config evidence. No new planner, DWB, footprint, costmap, smoother,
policy, sensor, LIO, recovery or calibration settings are introduced.

Offline verification uses `tests/test_warehouse_route.py` plus existing narrow,
readiness, metrics/math, ground-alignment and observer regression suites. It
includes real ROS message conversion and real transport methods against fake
IO, without `rclpy.init`, nodes, actions or command publication. Final counts and
owned-process cleanup evidence are in the root NOW and the single review bundle.

## Warehouse trajectory overlays — retained DIAGNOSTIC localization evidence

Future USER-RUN critical benchmarks automatically capture small evidence
sidecars and render images after transport teardown. The default remains a
ROS-free dry-run: no capture subscriptions, plotting, goals or motion. Existing
G1 admission (`navigation_success`, complete TF and .05m evaluation), G2 outcomes,
failure stops and cancellation gates are unchanged. G3–G5 are not auto-executed.

**DIAGNOSTIC map source: the exact same-session `/map` OccupancyGrid used by navigation.**
The following map-local evidence remains available; PRIMARY comparison evidence
and the default composite are defined in the WORLD section below.
The default warehouse launch builds an online map; no saved static navigation
map YAML/PGM was found in the relevant environment/config/results locations.
`scripts/save_course_map.sh` already saves `/map` through Nav2 map_saver when a
user requests a static artifact. Capturing OccupancyGrid fields directly avoids
an additional export process and retains unknown/probability cells exactly.
If fixed-map mode is selected, its map_server publishes the same `/map` interface.

| Source/code path + key snippet | Causal interpretation / choice |
| --- | --- |
| `course_navigation/launch/navigation.launch.py`: `if mode == 'online'` creates `map_projector`; fixed mode creates `nav2_map_server` | `/map` is the current stack's map interface in either supported map mode. |
| `course_navigation/course_navigation/map_node.py`: `create_publisher(OccupancyGrid, '/map', qos)`; `msg.header.frame_id = self.frame`; `msg.data = self.grid.data().reshape(-1).tolist()` | This is an existing map-frame occupancy raster, not geometry synthesized for the plot. |
| `course_navigation/course_navigation/map_projection.py`: `grid[self.free] = 0`, `grid[self.occupied] = 100`; initial value `-1` | Free, occupied and unknown stay distinguishable; unknown is not evidence of a clear corridor. |
| `scripts/save_course_map.sh`: `nav2_map_server map_saver_cli` | Existing static exports originate from the same `/map`; no separate floorplan is substituted. |
| `scripts/observe_warehouse_narrow_failure.py`: `topic.endswith('/costmap_raw')` saves raw master maps | Older observer artifacts are inflated navigation costs, including odom-frame local maps. They are not an uninflated canonical floorplan and are from different startups. |
| `scripts/prepare_warehouse.py` / `course_sim/launch/simulation.launch.py`: generated warehouse world and robot spawn | Gazebo world geometry is not registered to the online SLAM map by these files. Projecting its meshes with a guessed transform would violate the route's frame contract. |
| `scripts/narrow_planner_comparison.py:path_metrics`: `integrated_raw_cost_exposure_cost_m` and `exposure_definition` | This is descriptive costmap exposure for planner diagnostics, not a visual detour rule. No such scoring system is added. |
| `course_navigation/course_navigation/goal_manager.py`: `self.active.pose(...)` and `self.active.finish(...)` | Existing manager path length/error remain secondary metadata; this task does not alter that implementation. |

**AGAINST `/map`:** online coverage can be incomplete and change with SLAM.
**FOR:** it is the actual navigation obstacle map in the required frame, with no
hand-drawn geometry, world transform or inflation reinterpretation. **Verdict:**
save exact snapshots and show unknown cells. Biggest unknown is whether a future
run observes the corridor walls sufficiently. Reconsider if a registered static
map becomes the actual navigation source; its `/map` output remains suitable.
Cross-startup absolute-map reproducibility stays **UNVERIFIED**, as above.
Sharing a frame name is not registration; unrelated old maps/traces are rejected.

Capture uses `RosGoalTransport`'s existing TF buffer. `capture_pose()` calls
`lookup_transform('map', 'base_link', Time())`, keeps the original pose/stamp,
requires TF age in [0,.5] s, and samples at most 10 Hz into a standard
`nav_msgs/Path`-shaped dictionary. This is **SLAM localization trajectory**, not
Gazebo truth or certified physical body clearance. No new localization stack,
action client, planner preflight or velocity publisher is introduced. The passive `/map`
subscription matches transient-local/reliable map QoS. Truth capture below is
a separate evaluation-only subscription, with no new node or recorder.
No planned path overlay is added: the current route benchmark does not already
capture one, and an older observer's `/plan` belongs to a different session.

**AGAINST automatic capture:** extra work can perturb timing or fail storage.
**FOR:** the existing TF buffer avoids a parallel recorder and captures evidence
with each explicit user trial. **Verdict:** passive sidecars; errors do not change
navigation outcomes. Biggest unknown is live overhead/availability, untested here;
prefer already-captured same-run paths if available in future. Rendering occurs
offline after cleanup; pending-goal cancellation precedes map serialization.
Successful-segment serialization can add inter-segment wall time and is not a
measurement of controller timing. No live non-perturbation claim is made.

Artifacts inside the printed benchmark run directory:

```text
route-snapshot.json                  generated copy of the canonical config
report.json                          original outcomes + visualization status
evidence/index.json                  run/route binding, errors, segment sources
evidence/00-Start-G1.path.json        stamped map-frame nav_msgs/Path fields
evidence/01-G1-G2.path.json           only when that segment was executed
evidence/map-<sha256>.json            exact OccupancyGrid, original header/info/data
overlays/full-route.png              actual executed prefix, all G1..G5 nominal labels
overlays/Start-G1.png                 full context + segment detail
overlays/G1-G2.png                    full context + corridor/detail view
overlays/trajectory-overlays.json     per-image provenance and secondary metrics
```

Every image row records `route_id`, `segment_id`, benchmark HEAD, source `/map`
file/hash/header, image path and exact waypoint XY. Existing manager path length
and endpoint error are included per segment when present. The full overview uses
the latest available matching segment map; each segment uses its own end-of-
capture snapshot. Map stamps are retained; this is not a historical map mosaic.
The offline renderer supports supplied artifacts for all five segments, but the
live critical benchmark continues to stop at G2. Unexecuted goals stay nominal
markers; they do not get trajectory lines. Separate segments are never joined by
invented connecting lines. A full image with missing capture says **INCOMPLETE**.

`scripts/warehouse_route_evidence.py:observe_clock()` invalidates all run segments
on an observed backward clock, including between segments. The transport clears
its cached map on that reset. `path_chunks()` disconnects gaps >.5s and rejects
mixed frames/nonmonotonic poses. `map_geometry()` validates ROS occupancy values,
dimensions and planar origin quaternion; plotting respects resolution, origin
translation/yaw and bottom-up row order. No cells are manually filled for real
evidence. `RouteEvidence.finish()` releases failed capture as unavailable, so
one storage failure cannot poison the next segment. No map/path means no diagnostic
trajectory PNG; the composite explicitly shows its unavailable panel.
`route_content_sha256` binds the captured config; sent goal name/frame/XY are
checked against it, and map/path SHA checks reject mismatched or corrupt inputs.
These are consistency checks, not a new registration certificate.

The composite below is the primary human-facing artifact: visually check its
PRIMARY truth panel for a large loop/major bypass
and intended-corridor traversal. Context panels include the whole map; detail
bounds include all captured segment points so a bypass is not cropped out.
Start/end markers, nominal G1..G5 and unknown/occupied legend are shown. Gaps,
unknown walls or incomplete capture can make the verdict inconclusive. The
report deliberately leaves `detour_judgment: null` for human review. No automatic
numeric detour threshold or planner ranking is introduced.

Use the exact USER-RUN startup and Bash environment in the preceding section.
The existing command now also captures and generates overlays automatically:

```bash
# USER-RUN ONLY, in the sourced terminal for the existing idle warehouse:
/usr/bin/python3 scripts/run_warehouse_route.py --mode critical --execute \
  --output-root /tmp/b2w-warehouse-route-vis-live
```

For offline re-rendering of a saved run (no ROS environment needed), replace
`RUN_DIRECTORY` with the exact directory printed by the benchmark and choose
a new output directory. Existing images are never overwritten:

```bash
/usr/bin/python3 scripts/plot_warehouse_route.py RUN_DIRECTORY \
  --output-directory /tmp/b2w-overlay-review --label method-name
```

System Python Matplotlib is available on this machine; no installation was run.
This delivery ran only offline/mock tests. Inspected example PNGs under
`/tmp/b2w-route-overlays-20261016/mock-preview/overlays-checked/` explicitly say
**MOCK MAP + MOCK TRAJECTORY — NOT WAREHOUSE EVIDENCE**. They test rendering;
they do not establish a real corridor traversal or no-detour result. Actual
canonical-route overlays require a future USER-RUN capture; old endpoint-only
metrics cannot reconstruct a continuous trajectory.

## WORLD truth PRIMARY + session map DIAGNOSTIC (current evidence contract)

The default human-facing artifact is **`overlays/route-comparison.png`**, one
horizontal figure: LEFT **PRIMARY: world truth floorplan + truth trajectory**;
RIGHT **DIAGNOSTIC: session map + SLAM trajectory**. Judge “no obvious detour”
primarily from the left panel: no large loop / major bypass between adjacent
waypoints, with traversal through the intended corridor. The right panel helps
explain localization/map effects. It remains available and is not the primary
comparison map. No numeric detour scoring or automatic pass is added.

The canonical artifacts are `course_navigation/maps/warehouse_ground_truth_floorplan.png`,
`.json` and `.geometry.json`. The authoritative source is the pinned AWS small
warehouse `ee0af733315e78432408c3cd98d378ecee5f767c`, its no-roof world,
and `scripts/prepare_warehouse.py`'s generated
`third_party/aws_warehouse/warehouse.world`. All 25 static collision meshes are
loaded by the installed simulator's `ignition-common4-graphics` mesh loader,
including COLLADA units/node transforms, then transformed by SDF model, link,
collision poses and mesh scale. This is an **all-height orthographic projection
of authored collision triangles**, including overhead objects. GroundB support
is light, other collision projections dark, outside support gray. It is not a
height-sliced or inflated traversability/clearance map. No bounding-box
replacement, hand drawing, screenshot, guessed pose or guessed bounds is used.
Projected triangle vertices are retained in the geometry JSON; pixel-center
triangle coverage is the explicit rasterization rule.

**AGAINST this floorplan:** projecting overhead collisions does not establish
body clearance, and visual collision meshes can differ from visual textures.
**FOR:** authored static collision triangles define a reproducible, fixed WORLD
reference without registering SLAM or choosing an undocumented height slice.
**Verdict:** canonical collision projection for visual route comparison.
Biggest unknown: real traversal/clearance and live evidence availability remain
untested. Reverse the projection choice only with an explicit new height/surface
contract and regenerated provenance; do not silently substitute a screenshot.

Exact image/world contract, also stored in the metadata:

```text
frame_id: world; resolution: 0.02 m/pixel; width: 700; height: 1046
xmin: -6.99023499       xmax: 7.00976501
ymin: -10.45333252      ymax: 10.466667480000002
image row 0 is north/top; column 0 is west/left
pixel-center x = xmin + (column + 0.5) * resolution
pixel-center y = ymax - (row + 0.5) * resolution
column = (x - xmin) / resolution - 0.5
row    = (ymax - y) / resolution - 0.5
```

Bounds start at the exact transformed source minimum and round the dimensions
up to whole .02m cells; the last row/column includes deterministic padding. JSON
records raw source-file SHA256s (world, models, collision meshes, preparer,
extractor, builder), portable normalized world SHA, loader version, image SHA
and projected-geometry SHA. World identity normalizes only absolute mesh URI
prefixes, not geometry/poses. Run manifests additionally hash collision mesh
bytes; PRIMARY requires both the normalized world and collision hashes to match.
These are **on-disk provenance checks**, not proof of a future simulator's loaded
runtime scene. Keep the user's warehouse startup/log as runtime evidence.

| Code path + key snippet | Causal interpretation |
| --- | --- |
| `scripts/prepare_warehouse.py`: `REVISION = 'ee0af733315e78432408c3cd98d378ecee5f767c'`; `ET.SubElement(model, 'pose').text = wrapper.findtext('pose', '0 0 0 0 0 0')` | The existing environment generator pins authored assets and retains world placement; its output is the geometric authority. |
| `scripts/warehouse_collision_mesh.cc`: `MeshManager::Instance()->Load(argv[1])`, `sub->Vertex(index)` | The offline extractor uses the installed simulator mesh decoder, not a new approximate COLLADA parser; no simulator or ROS node starts. |
| `scripts/warehouse_truth_floorplan.py`: `pose_matrix(model) @ pose_matrix(link) @ pose_matrix(collision)`; `(triangles*scale) @ transform[:3, :3].T + transform[:3, 3]` | Transformed authored triangles define exact WORLD geometry. Unsupported named relative frames/shapes fail rather than being guessed. |
| `scripts/warehouse_truth_floorplan.py:rasterize`: `points = pixel_to_world(...)`; `region[inside] = ...` | Real triangle membership at the documented pixel centers determines the raster. AABB bounds only limit computation. |
| `course_sim/course_sim/description.py`: `plugin.find('odom_frame').text = 'sim_world'`; `sub(plugin, 'odom_topic', '/sim/raw/ground_truth/odom')` | Truth uses an existing simulator output with a frame label that must be authenticated, rather than assumed to be SLAM `map`. |
| `b2w_sim/b2w_description_ros2/urdf/b2w_description_rsl.urdf`: `<robot_base_frame>base_link</robot_base_frame>`; no `xyz_offset`/`rpy_offset` in the odometry plugin | The existing truth plugin has no configured pose offset; converted SDF has root `base_link` with identity link pose. |
| `course_sim/config/bridge_rl_sar.yaml`: `ros_topic_name: /sim/ground_truth/odom`, `direction: GZ_TO_ROS` | The evaluation harness reuses the existing bridge; it changes no public topics or TF publishers. |
| `scripts/warehouse_route_evidence.py:sample_truth`: `pose['header']['frame_id'] = 'world'`; `truth_source` retains `source_frame_ids` | Only saved evaluation data aliases authenticated zero-offset `sim_world` to WORLD. Original source frame is recorded; no TF is broadcast. |
| `scripts/run_warehouse_route.py:pair_truth_evidence`: `lookup_transform('map', 'base_link', Time(seconds=..., nanoseconds=...))` | Registration pairs use TF at the exact truth timestamp; unavailable synchronized TF yields no pair, not an interpolated/guessed registration. |
| `scripts/warehouse_truth_overlay.py:fit_registration`: `minimum_pairs=30`, `minimum_span_m=1.`, `rms_limit_m=.05`, `maximum_residual_limit_m=.10` | All distinct synchronized samples fit one proper SE(2) map→world transform. No outlier removal. Rejection leaves WORLD waypoint labels unavailable. |
| `scripts/warehouse_truth_overlay.py`: `waypoints = ... @ ...rotation... + ...translation...` | Registration is consumed only by the renderer to label frozen map goals; it never returns navigation inputs. |
| `scripts/plot_warehouse_route.py`: `role='DIAGNOSTIC'`; `render_primary_and_composite(...)` | Existing `/map` full-route and per-segment PNGs remain, now clearly diagnostic. Map/path corruption is reported per panel, so valid truth can still render. Run/route identity mismatches still reject the operation. |
| `scripts/warehouse_truth_overlay.py`: `composite.paste(left_image, (0, left_y))`; `composite.paste(right_image, (left_image.width, right_y))` | One combined PNG shows PRIMARY left / DIAGNOSTIC right without resampling or cropping. Complete panels retain aspect ratio; shorter panels are vertically centered with white padding. |
| `scripts/run_warehouse_route.py`: `if args.execute:` rendering after `run_critical(...)` | Plotting runs after transport teardown, never during navigation; the default dry run creates no subscription, node, plot or goal. |

The installed Fortress 6.18.0 publisher implementation explicitly uses
`rawPose = worldPose(this->model.Entity(), _ecm)` and applies the configured
pose offset (zero here). This authenticates the **WORLD-coordinate meaning** of
the `sim_world` label; it is not an invented SLAM→world transform.
Source: [official OdometryPublisher.cc, 6.18.0](https://raw.githubusercontent.com/gazebosim/gz-sim/ignition-gazebo6_6.18.0/src/systems/odometry_publisher/OdometryPublisher.cc).
Absolute **SLAM map coordinates across fresh startups remain UNVERIFIED**.
Accepted per-run evaluation registration does not establish cross-startup
navigation repeatability and is never fed back to SLAM/Nav2/goals.

Truth is a passive, evaluation-only subscription to `/sim/ground_truth/odom`,
sampled at most 10 Hz while a segment is active. WORLD x/y are saved as standard
stamped Path-shaped data. Registration attempts buffered exact-time TF lookups;
30 distinct timestamps and ≥1m map spatial extent are required, with RMS ≤.05m
and maximum residual ≤.10m. All samples participate; inconsistent/insufficient
registration means no WORLD G1..G5 labels. Trajectories/start/end still render.
Map goals, SLAM, Nav2, control, staging admission, safety, benchmark progression,
cancellation, endpoint evaluation and G2 success never read truth or the fit.
Missing truth, canonical files, matching provenance or diagnostic maps produces
explicit unavailable/incomplete panels, never a navigation failure or guessed
map. Old runs without truth cannot acquire it from endpoint metrics.

**AGAINST passive capture:** extra callbacks/IO can affect timing and live
availability is untested. **FOR:** reuse one existing transport, TF buffer,
truth topic and compact per-segment sidecars. **Verdict:** evaluation-only;
truth errors are contained and rendering occurs after teardown. Biggest unknown
is live capture overhead; reverse to externally captured same-run artifacts if
measured overhead is unacceptable. No live non-perturbation claim is made.

Additional artifacts per run (the earlier diagnostic images remain):

```text
evidence/00-Start-G1.truth.path.json   WORLD truth, original stamp, source frame in index
evidence/00-Start-G1.registration.json synchronized map_xy/world_xy pairs
# Equivalent 01-G1-G2 files only if that critical segment was executed.
overlays/world-truth-full-route.png   standalone PRIMARY, executed prefix only
overlays/world-truth-Start-G1.png     standalone PRIMARY segment
overlays/world-truth-G1-G2.png        standalone PRIMARY critical segment
# WORLD segment PNGs require valid canonical floorplan + truth capture.
overlays/full-route.png              retained standalone DIAGNOSTIC
overlays/Start-G1.png                 retained DIAGNOSTIC context + detail
overlays/G1-G2.png                    retained DIAGNOSTIC corridor/detail
overlays/route-comparison.png         DEFAULT: PRIMARY left / DIAGNOSTIC right
overlays/trajectory-overlays.json     image roles, sources, coordinates, registration,
                                     residuals, composite rectangles, unavailable reasons
```

A full-route image always means the executed prefix, not an implicit G3–G5
trial. Captured segments are never joined by fabricated connecting lines;
within-segment gaps >.5s are disconnected. All actual points remain inside plot
bounds, including detours outside the floorplan. Secondary WORLD path length
sums captured chunks only; endpoint error remains the existing map-frame
measurement, explicitly labeled as such. No new success metric is introduced.

The USER-RUN startup/environment and critical `--execute` command above remain
valid and now generate the composite automatically. The agent ran no live ROS,
Gazebo, Nav2 or motion. Safe offline re-rendering uses the same existing CLI:

```bash
/usr/bin/python3 scripts/plot_warehouse_route.py RUN_DIRECTORY \
  --output-directory /tmp/b2w-world-comparison-review --label method-name
```

To reproduce canonical artifacts **offline**, with existing installed development
libraries and pinned assets, compile the extractor and run the deterministic
builder (no dependency installation or simulation):

```bash
c++ -std=c++17 -Wall -Wextra -Wpedantic scripts/warehouse_collision_mesh.cc \
  -o /tmp/b2w-warehouse-collision-mesh \
  $(pkg-config --cflags --libs ignition-common4-graphics)
/usr/bin/python3 scripts/warehouse_truth_floorplan.py \
  --mesh-exporter /tmp/b2w-warehouse-collision-mesh \
  --output-directory /tmp/b2w-warehouse-floorplan-rebuild
```

Pillow/Matplotlib are needed only for offline rendering; SciPy for regeneration.
No dependency installation occurred. Mock preview artifacts in
`/tmp/b2w-horizontal-composite-20261016/mock-preview/` explicitly label synthetic
truth/SLAM data. They verify images, not physical traversal or no-detour success.
