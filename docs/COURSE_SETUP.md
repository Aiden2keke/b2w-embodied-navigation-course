# 本机课程系统

本机适配目标为 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress、Python 3.10。保留上游运动策略、模型和原始入口，新增四个 `course_*` package。团队公共接口仍以 [INTERFACE.md](INTERFACE.md) 为对齐依据，具体实现选择见 [INTERFACE_ALIGNMENT.md](INTERFACE_ALIGNMENT.md)。

新增开源 rl_sar 控制器已通过本机平地站立、运动、停车和短目标导航复测，下载与启动见 [RL_SAR_SETUP.md](RL_SAR_SETUP.md)。原 `legacy` 策略曾出现后腿异常内收和漂移；历史诊断及新验证边界见 [LOCOMOTION_STATUS.md](LOCOMOTION_STATUS.md)。

## 安装与构建

在仓库根目录执行：

```bash
scripts/setup_course_humble.sh --install
scripts/fetch_lio_sam.sh
# 首次下载及 ONNX 转换按 docs/RL_SAR_SETUP.md 执行
/usr/bin/python3 scripts/prepare_rl_sar.py
scripts/build_course.sh --symlink-install
scripts/setup_course_humble.sh --json
scripts/test_course.sh
```

安装脚本使用 sudo 安装系统依赖，额外 Python 依赖放在 `.venv`，不替换系统 Python。获取脚本固定 LIO-SAM ros2 commit `08af3f32f01725372d4269838dc44c19c6d9e76b` 与 GTSAM 4.1.1 commit `69a3a75195b65356d6e56669e5199d325c7962c9`，应用仓库中保存的 TransformFusion 修复，GTSAM 安装到 `.deps`。已有 checkout revision 不匹配时保留它并停止。

Humble 必须使用 `/usr/bin/python3`；系统默认 Python 或 Conda 环境可能不兼容。构建、运行脚本已设置解释器、依赖库路径和大点云传输配置。首次 GTSAM 编译较慢。`.venv`、`.deps`、第三方 checkout、bag 和实验结果均不提交。

## 启动

```bash
# A：传感器、机器人、有限空间场景
scripts/run_course.sh mode:=sim

# A：手柄控制并录制测量
scripts/run_course.sh mode:=record control_source:=manual bag_directory:=bags

# B：真实 LIO-SAM 建图
scripts/run_course.sh mode:=slam rviz:=true

# C：完整导航，默认由唯一 Nav2 来源产生速度
scripts/run_course.sh mode:=navigation gui:=true rviz:=true
```

SLAM 在启动后等待 20 秒再初始化，让渲染器、传感器和机器人站稳。Nav2 的 lifecycle 会等待定位 TF。slam/navigation 的 RViz fixed frame 为 `map`，使用 2D Goal Pose 发布 `/goal_pose`；sim/record 自动使用 `base_link`。

## 开源仓库场景

使用 [AWS RoboMaker Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/tree/ros2) 的 MIT-0 模型资产，固定 commit `ee0af733315e78432408c3cd98d378ecee5f767c`。上游已归档；本项目只使用场景资产，由本机 Fortress 和现有课程节点运行，不安装其 Classic/云端运行依赖。

```bash
# 新终端，仓库根目录；下载准备脚本可重复运行
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
scripts/run_warehouse.sh
```

脚本默认打开 Gazebo 和 RViz、启动 SLAM/Nav2，使用独立 Gazebo partition。参数可覆盖，例如 `scripts/run_warehouse.sh mode:=sim`、`scripts/run_warehouse.sh gui:=false`。其他发送目标、录包、检查话题的终端也必须 `export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1` 并加载本项目 ROS 环境。已有运行会话应先在其终端 Ctrl+C，避免同一域中重复启动。

场景保留 25 个上游模型的网格、碰撞体和摆放位置，使用无顶盖版本方便观察。适配脚本展开 Classic 嵌套 include，修正资源 URI，移除静态模型中无用且不合法的惯性参数，加入 Fortress 系统和 400 Hz physics。地面摩擦系数设为现有课程基线的 1.0，供现有 B2-W 运动策略使用；这不是实测仓库摩擦系数。机器人默认出生位置 `(0,0,1)`，在线地图扩大到 40×40 米。场景不会缩放机器人，也不会取消真实障碍物碰撞。

通过 `scene:=warehouse` 可以在所有模式使用仓库，`spawn_x/y/z` 可指定出生位置，`world:=/absolute/path/file.world` 可覆盖场景文件；生成器会读取实际 world 名称。下载与生成入口为 `/usr/bin/python3 scripts/prepare_warehouse.py`，资产、原许可证、来源版本和适配记录保存在 `third_party/aws_warehouse/`，不提交第三方模型。录制 metadata 包含场景名称、world 哈希和来源版本。

```bash
# 仓库正常工况的单次建图导航；目标坐标属于 SLAM map
scripts/run_experiments.sh --scene warehouse --profiles normal --seeds 0 --goal 1 -1
# 换成 --profiles normal mild medium severe 可做干扰实验
```

RViz 用于显示机器人、点云和估计地图；完整仓库的带纹理三维场景在 Gazebo 中观察。发送目标时选择地图中的可达空闲区域，货架内部的目标会规划失败。

## RViz 机器狗显示修复

此前生成 URDF 将网格替换成普通绝对路径，Fortress 能读取，但 RViz 的 `resource_retriever` 不能读取，导致 TF 存在而几何模型加载失败。现在使用 `file://` URI，并明确配置 RobotModel 的 Topic、Reliable、Transient Local 和 Visual Enabled。原生加载器回归验证覆盖全部 17 个网格，且实际 RViz 画面已确认机器狗可见。

如果独立打开 RViz，请加载 `course_bringup/rviz/course.rviz` 并设置 `use_sim_time:=true`；无 SLAM 时 Fixed Frame 选 `base_link`。课程入口和仓库入口默认使用域 91、本机通信，并将 RobotModel 与 Gazebo 生成器的描述话题统一为 `/b2w/robot_description`，避免其他项目的 `/robot_description` 覆盖模型。显式设置环境变量可以覆盖域号或启用网络通信；所有检查终端必须使用同一域、本机设置和 DDS profile。

近景和实测地面对齐证据见 [validation/2026-10-10-rviz-ground-closeup.png](validation/2026-10-10-rviz-ground-closeup.png) 与 [validation/2026-10-10-rviz-ground-alignment.json](validation/2026-10-10-rviz-ground-alignment.json)。运行中的检查入口：`/usr/bin/python3 scripts/check_rviz_alignment.py`。

同一台机器、同一 ROS_DOMAIN_ID 只允许一套课程仿真；重复启动会显示已有 launch PID 并退出，不会影响原会话。Gazebo、传感器桥接或底层控制器退出时，整套 launch 自动关闭，避免残留节点接到下一次仿真的时钟和关节反馈。每次默认生成独立 Gazebo partition。重新初始化请先在原启动终端 Ctrl+C，再运行启动命令。

控制器遇到时钟回退会清空动作、指令和反馈，取得新的关节与 IMU 消息后重新进入站立过渡；反馈检查采用消息时间戳。此检查验证底层恢复，不代表 LIO-SAM 支持在原会话中直接重置整个 world。RViz 二维地图使用 Draw Behind，避免与地面网格竞争深度。

静止会话的检查入口（检查期间不要发导航目标）：`ROS_LOCALHOST_ONLY=1 /usr/bin/python3 scripts/check_initialization.py --domain 91 --duration 20 --output /tmp/init-check.json`。

所有课程节点使用仿真时间。GUI 关闭时仍需渲染 GPU LiDAR。`headless_rendering:=auto` 在已有 `DISPLAY` 时使用显示服务器，在没有显示时启用 EGL；可以显式指定 `true/false`。本机双显卡的 EGL 默认设备曾启动失败，已有 X 显示的服务器模式已实际产生点云。`gui:=false` 不等于禁用传感器渲染。

可用参数：`world`、`gui`、`headless_rendering`、`rviz`、`fault_profile`、`seed`、`horizontal_samples`、`control_source`、`joy_dev`、`slam_params_file`、`adapter_params_file`、`nav_params_file`、`map_mode`、`map_file`、`bag_directory`、`results_directory`、`timeout_s`、`goal_tolerance_m`。自定义 world 的 SDF world 名须为 `course_world`。

`control_source:=auto` 在 navigation 模式选择 Nav2，其他模式不启用高层速度源。`manual` 可用于 sim/record/slam；`sru` 需要 mode:=slam 和手柄。SRU 的里程计 remap 到 `/slam/odom`，不使用真值定位。navigation 模式拒绝同时启用手柄或 SRU。手柄/SRU 的原有使能开关保留，速度经过课程限速和超时适配器。不要另外启动一个向公共速度 topic 发布的控制源。

手动发目标、取消：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/course_sim/config/fastdds_local.xml"
ros2 topic pub --once --qos-reliability reliable /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}'
ros2 service call /course_navigation/cancel_goal std_srvs/srv/Trigger '{}'
```

目标坐标在 SLAM map frame 中，不能直接把 Gazebo world 坐标当作目标。目标替换会取消旧 action 并保存其结果。结果 JSON 写到 `results_directory`；success 同时要求 action 成功和估计基座到达容差内。规划延迟是额外 ComputePathToPose action 的往返耗时，不是规划器内部纯计算时间；结果同时记录 planning_measurement_status，失败请求的耗时不能当作成功规划耗时。碰撞只统计现有 collision plate 阈值事件，不能代表所有机身和轮端接触。

## 录制、回放和评价

```bash
# 在另一个已 source 的终端录制，Ctrl-C 正常结束
ros2 run course_sim record_course --output-dir course_results --fault-profile normal --seed 0

# 仿真停止后回放到真实 SLAM；不回放之前的估计 TF/估计里程计
ros2 launch course_slam replay.launch.py bag:=/absolute/path/to/run/bag

ros2 run course_slam export_trajectory /absolute/path/to/run/bag --output-dir course_results/trajectory
ros2 run course_slam evaluate_trajectory course_results/trajectory/estimate.tum \
  course_results/trajectory/truth.tum --output course_results/slam_metrics.json

# 只读检查实际消息、频率、frame 和公共 TF
/usr/bin/python3 scripts/smoke_course.py --duration 20 --output /tmp/course-smoke.json
```

外部检查、回放终端应使用相同 ROS_DOMAIN_ID 和 Fast DDS profile。评估采用 base_link、时间戳最近邻匹配（默认 20 ms）、SE(3) 对齐且不缩放；报告未经对齐误差、匹配覆盖率及 1 秒 RPE。未经对齐误差包含不同原点/朝向造成的差异。无有效轨迹或时间重置时报错，缺失 RPE 写 null。

录制目录包含配置、seed、点云密度、配置哈希、源码 revision 和 dirty 状态。完整 32×2000×10 Hz 点云录制约产生每秒十几 MB 数据，按需要及时停止录制。仿真采用瞬时扫描，所有点的 time=0；回放不能把它当作实机旋转扫描的运动畸变数据。

## 地图和扰动

```bash
scripts/save_course_map.sh course_results/new_map
scripts/run_course.sh mode:=navigation map_mode:=fixed \
  map_file:="$PWD/course_results/new_map/map.yaml" \
  adapter_params_file:="$PWD/course_slam/config/adapter.yaml"

scripts/run_course.sh mode:=navigation fault_profile:=mild seed:=1
```

地图保存包括真实 SLAM 点云 PCD 及二维 PGM/YAML。fixed 模式加载保存的二维地图，定位仍来自当前 LIO-SAM；应在 adapter 配置中提供当前 SLAM frame 到保存地图的配准变换。默认零变换只适用于相同初始化原点/朝向。这里没有实现对任意旧地图的自动全局重定位。

二维地图只把有测量射线支持的区域标为 free，其余保持 unknown。在线 SLAM 从点云拟合初始地面，统一变换地图点云、公开里程计和 TF，使地面为 map/odom 的 z=0；障碍高度范围 0.15–2.0 米因此相对地面。缺少足够地面回波时等待而不使用仿真真值补齐。此初始平面仅适用于当前平地场景，坡道和多层地图需要专门处理。固定地图模式不自动重设地面，仍由保存地图的配准定义坐标。footprint 为当前模型的保守矩形（前后各 0.70 m，左右各 0.38 m），不是厂家精确外形测量。

仿真雷达用 SDF visibility mask 排除机器狗自身的 render geometry，避免顶部回波被写入 SLAM/导航障碍。GUI/RViz 几何模型和物理碰撞保持有效。这是理想化的自身回波过滤：不会模拟机身对环境回波的遮挡，实机需要另行实现自身回波过滤。机制参见 [Gazebo Sensors 6 LiDAR API](https://staging.gazebosim.org/api/sensors/6/classignition_1_1sensors_1_1Lidar.html)。目标管理器同时等待 controller/planner/BT navigator 的 lifecycle 为 active，接口刚出现时的目标会继续排队等待。

normal/mild/medium/severe 是 `course_sim/config/fault_profiles.yaml` 中的候选实验设置，尚需团队确认。`slip_ratio` 映射到 Gazebo WheelSlip 的纵向/横向 compliance；它不是实测运动学打滑率，也不能把 0.6 解释为真实 60% 打滑。对比实验应固定目标、seed、地图初始化与点云密度并记录实际运动效果。降低 `horizontal_samples` 是显式性能选项，不能继续声称完整 640k 点/秒密度；后端 Horizon_SCAN 也必须同步调整。

## 批量联调

```bash
scripts/run_experiments.sh --profiles normal mild medium severe --seeds 0 1 2 --goal 1.0 0.0 --timeout 45
```

脚本串行启动独立 Gazebo partition 与 ROS domain，等待真实 SLAM/map 后发目标，保存配置、runtime.log、每次导航 JSON 和 summary.json。启动失败/没有结果时指标写 null，并保留原因。默认并不录制大型点云 bag；正式 ATE/RPE 对比实验应另行录制。seed 控制测量扰动随机序列，不宣称物理引擎的所有随机过程已被固定。
