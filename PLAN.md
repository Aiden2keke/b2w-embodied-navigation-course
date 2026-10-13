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

- 分支：`improve/narrow-navigation`；审计基准 HEAD：`66db5152b3b617d0e5ac5ed65b031573255a4169`；开始时工作区干净。
- 目标：在 B2-W + LIO-SAM + Nav2 baseline 上先复现真实窄空间失败，再做可回退、单变量或少变量改动。
- 不改 locomotion policy，不跑长实验，不 push，不 merge main。本轮不修改导航配置、公共 topic/TF、消息类型或模块边界。
- 已读 `AGENTS.md`、`CODEX_GUIDE.md`、`docs/INTERFACE.md`、`docs/INTERFACE_ALIGNMENT.md`、`docs/COURSE_VALIDATION.md`、`docs/RL_SAR_SETUP.md`、`docs/LOCOMOTION_STATUS.md`。参数 SSoT 为当前 YAML/C++/Python 实现；历史 JSON 仅证明当时的有限验收范围。
- 已应用 using-superpowers、systematic-debugging、brainstorming、writing-plans、verification-before-completion，以及 OpenAI Docs 的模型选择流程。本轮设计属于已有实验流程的有界扩展；用户要求的唯一根计划和评审关卡优先于 Skill 的额外计划/提交流程。不派生子 agent。
- 旧实施计划归档至 `docs/plans/archive/2026-10-10-local-course-system-plan.md`。历史 design/validation 保留为证据，不是另一个当前 roadmap。无新增 HANDOFF。
- **本轮交付是只读诊断与计划，不是 harness 实现。** 用户要求“代码、测试或 PLAN.md 一旦发生实质修改”即生成 review bundle 并停止，因此本文件首次落盘后进入评审，不继续写 harness 或 M2 配置。总体目标和 M1 尚未完成。

## NOW — Milestone 1：只读诊断 + 最小复现 harness

- [x] 核对分支、项目入口、接口约束、相关 Skill/SSoT 和历史验证。
- [x] 核查可用模型信息与官方价格，给出按风险调整的路由建议。
- [x] 静态追踪完整导航链路、参数、地图语义与低层命令接口；形成下方候选表。
- [x] 执行环境预检及 5 项只读逻辑探针，并检查本次进程清理。
- [x] 明确 harness 的场景、指标、故障分类、日志和清理设计。
- [x] 运行环境恢复：真实 Ubuntu 22.04 host 的 Humble/Fortress、bridge/Nav2、colcon/rosdep CLI、项目 venv、GTSAM、rl_sar ONNX 和 warehouse 就绪；12 包构建、环境检查、20 项 tests、无目标启动 smoke 与 cleanup 均通过。启动节点和参数文件路径已记录；完整运行参数 dump 与 action/lifecycle 验收属于后续 harness。
- [ ] 实现最小 harness，完成有意义的短单测。
- [ ] 单次真实 smoke：启动、发 goal、收指标、结束、清理均有证据；不能用离线探针代替。
- [ ] 生成实现后的 review bundle，并给用户经过 smoke 验证的长测命令。

环境恢复结果（2026-10-13）：真实 host `zyd-MS-7D97` / 用户 `zyd` / Ubuntu 22.04.5 / Python 3.10.12，分支仍为 `improve/narrow-navigation`。用户已完成系统依赖安装。本次没有执行新的 sudo 系统安装、apt upgrade，或修改 NVIDIA/CUDA/kernel。Humble、Nav2 1.1.20、ros_gz_bridge 0.244.26、Fortress 6.18.0 验收通过。

source 异常已复现并裁决：zsh 的 `BASH_SOURCE[0]` 为空，`setup.bash:6` 因此将 cwd 设为 `AMENT_CURRENT_PREFIX`，`:11` 误读源码根目录 `setup.sh`。不需要 prefix 污染也能复现；启动文件未发现旧 workspace source，`dpkg -V ros-humble-ros-workspace` 无异常。干净 zsh source `setup.zsh`、干净 Bash source `setup.bash`、用户 zsh 子进程 source `setup.zsh` 均通过。正常运行采用 Humble `setup.zsh` → 项目 `install/setup.zsh`。未修改系统 setup 或用户 shell 配置，已补充 CODEX_GUIDE 的对应说明。

GTSAM 固定 revision 构建为 4.1.1 并安装到 `.deps`；LIO-SAM 固定 commit 和已有补丁校验通过，四个 executable 已构建并在 smoke 中启动。项目 `.venv` 完成 requirements，NumPy 1.24.4、ONNX Runtime 1.23.2，继承系统 SciPy 1.8.0/PyYAML 5.4.1；仅在 venv 补齐 cffi 后 `pip check` PASS，系统 NumPy 仍为 1.21.5。固定 pretrained policy 导出 ONNX 成功，输入 `[batch,57]`、输出 `[batch,16]`，数值对照最大绝对误差 `1.24e-05`，ONNX SHA256 `da31c0c1820b893608e2d02746bbada16d0abb5de6116277e37ccfb02024c564`。原权重未替换。warehouse 25 个模型与 50 个资源 URI 完整，未重复下载。

完整 colcon build：12 包完成，约 63 秒，return code 0。stderr 保留未使用 Python 变量、CMP0074 和 OpenCV/TBB 链接警告。环境检查 `ready: true`，构建后 20 项 tests PASS。独立域 96、独立 Gazebo partition 的约 27 秒 warehouse/navigation bringup smoke 未发送 goal：clock 8261 条、点云 147 条、原始关节 8259 条、原始 IMU 4131 条，Gazebo/bridge/描述/控制器/LIO-SAM/Nav2 进入启动阶段；cleanup PASS，本轮 24 个进程身份已记录且无运行残留。SIGINT 后的 KeyboardInterrupt/退出码 -2/lifecycle 关闭竞争保留在日志，不冒充启动前崩溃。未验收 Nav2 全部 active、目标到达、物理 tracking 或窄空间实验；rosdep CLI 可用但 init/update 尚未执行。

本次证据目录：`/tmp/b2w-env-resume-20261013T033710Z/`。上一轮 bundle 原路径已不存在，上轮未完成结论由保留的 `/tmp/b2w-env-setup-20261013T024746Z/final-status.json` 和会话报告核对。按要求重新生成 review bundle 后停止；review 通过后可继续本 milestone 的 harness，不能进入 Milestone 2。

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

## 长测交付状态

**本轮不发布可执行的 narrow baseline/improved 多 seed 或宽度扫描命令。** 原因是 harness 尚未实现、真实 smoke 未通过，用户规定的“短测通过后给长测命令”条件未满足。不会提供指向不存在脚本/配置的伪可复制命令。实现后 CLI 应覆盖 `--smoke`、`--nav-params`、`--width`、`--seeds`、`--timeout-s`、`--ros-domain-id`、`--output-dir`，并从实际 `--help` 与已通过 smoke 的入口导出命令。

## 本轮打包与停止

提交给 review 的内容仅为本文件、旧计划归档及 `/tmp/b2w-narrow-diagnosis-ls7ys2r7/` 的实际检查证据。调用用户指定 `~/bin/april-review-bundle`，把上述日志显式传入（该脚本的固定项目文档列表没有 PLAN.md，但其 untracked 内容收集会包含新计划）。打包后不进入实现、不改参数、不提交、不 push。

审查重点：是否同意本轮严格在首次 PLAN.md 实质修改处停止；harness 的场景是否足以检验 lateral 假设；指标是否能区分指令与物理运动；恢复环境后优先复现哪一条失败。下一步是在评审通过、具备运行环境后完成 M1 剩余项，而不是直接生成 improved 参数。
