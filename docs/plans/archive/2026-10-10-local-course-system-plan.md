# B2-W 本机课程系统实施计划

> ARCHIVED 2026-10-13：历史实施记录，不再作为当前工作指令。唯一 CURRENT roadmap 是根目录 [PLAN.md](../../../PLAN.md)。下文保留当时的计划与验证边界。

日期：2026-10-10

设计依据：`docs/superpowers/specs/2026-10-10-local-course-system-design.md`，用户已确认。

执行方式：建议由当前 agent 在本工作区顺序实施，不启动子 agent。本文待用户确认执行方式后进入实现。

## 1. 完成标准

交付本机 Humble/Fortress 适配与 `course_sim/`、`course_slam/`、`course_navigation/`、`course_bringup/`。雷达采用用户选定的 XT32 32 线仿真基线；机身 IMU 使用独立配置。

完成状态须分别说明：文件实现、静态检查、构建、节点运行、Gazebo 运行、完整导航闭环、扰动实验。仅写好配置不能标记为闭环完成。实验结论只来自实际运行记录。

`docs/INTERFACE.md` 保持草案地位；新增实现对齐清单说明采用的默认值和需要团队确认的选择。已有模型、策略权重和原始运行入口保持可复用。

## 2. 顺序与依赖

```text
环境检查与安装入口
  → 上游 package 构建适配
  → 课程仿真模型、场景、桥接
  → 传感器格式、扰动和数据采集
  → LIO-SAM 获取、构建与运行
  → SLAM 输出、TF 和评价
  → 地图与 Nav2
  → 速度/目标适配与指标
  → 总 launch 与闭环联调
  → 运行文档和验证记录
```

依赖或权限问题出现时，先完成不受影响的代码与验证；对缺少的系统依赖生成明确安装命令，不能用模拟定位输出冒充 SLAM。

## 3. 阶段一：本机运行环境

新增文件：

- `scripts/check_course_environment.py`
- `scripts/setup_course_humble.sh`
- `scripts/build_course.sh`
- `requirements-course.txt`

实施步骤：

1. 检查系统版本、ROS distro、Gazebo/Ignition CLI、ament package、共享库、模型文件和 Python 模块，输出人可读报告及 JSON。
2. 显式使用 `/usr/bin/python3` 或基于其创建的工作区虚拟环境；检测 Python 3.14 与 Humble 不匹配情况。
3. 支持依赖检查、输出安装命令与显式执行安装的模式；使用 Humble/Fortress 的系统包，不替换系统 Python。
4. 虚拟环境放在工作区 `.venv`，允许访问 ROS 系统 Python 包；优先 CPU ONNX Runtime，GPU 仅为可选配置。
5. 构建脚本 source Humble，显式传递 CMake Python 解释器并使用正确的 colcon 解释器；不删除已有 build/install/log。
6. 安装和构建失败时保留具体日志及退出码，不静默忽略失败。

验证：shell 语法、检查脚本在当前缺依赖环境的实际报告、模块导入与模型加载。实际安装成功后才记录环境就绪。

## 4. 阶段二：上游 package 的必要构建适配

检查和按失败证据修改：

- `b2w_sim/b2w_controllers/CMakeLists.txt`、`package.xml`
- `b2w_sim/b2w_gazebo_ros2/CMakeLists.txt`、`package.xml`
- 其他现有 package 的直接依赖声明和安装规则

实施步骤：

1. 构建现有基础 package，确认 Humble ament/C++ API 与 bundled ONNX Runtime 可用。
2. 修复必要的依赖缺漏、链接配置、动态库查找和 CMake 安装顺序问题。
3. 保持运动策略输入、动作顺序和权重不变；策略路径通过现有参数配置。
4. 确认 installed share 中存在机器人 mesh、模型、策略和启动文件。
5. 使用上游 SRU 模式时通过 remapping 适配课程 topic，避免对高层学习策略作无关改写。

验证：实际 colcon 构建；运行低层节点加载策略，确认输出 joint topic 与现有 Gazebo 命令接口一致。

## 5. 阶段三：course_sim 模型、场景和桥接

新增主要文件：

- `course_sim/package.xml`、`setup.py`、`setup.cfg`、resource marker
- `course_sim/course_sim/description.py`
- `course_sim/config/sensors.yaml`、`fault_profiles.yaml`、`bridge.yaml`
- `course_sim/worlds/confined.world`
- `course_sim/launch/simulation.launch.py`

实施步骤：

1. 从 installed 上游 URDF 读取模型，用 XML 结构变换生成课程描述；只修改版本兼容、传感器及课程专用物理参数。
2. 转换 `gz-sim-*` 插件 filename/class 为实际 Fortress 支持的名称，资源搜索路径同时适配 Ignition；检查 force-torque 插件位置和碰撞反馈是否真实产生。
3. 在 `lidar_link` 增加 XT32 名义参数传感器：32 通道、水平 360°、垂直 31°、0.05–120 m、10 Hz。完整密度为每通道 2,000 水平采样。
4. 依据官方手册/角度文件核对具体 beam 角度和 ring 排序；理想模型与实机每通道标定差异单独记录。
5. 在 `imu_link` 增加 200 Hz IMU。frame 与外参使用现有 URDF 默认值，并允许配置覆盖。
6. 构建有限空间场景，包含走廊、转角、障碍和可通行目标区域；全尺寸根据机器人实际模型确认。
7. 配置 clock、关节、传感器、深度图、碰撞反馈和真值 bridge。公共 TF 不桥接 Gazebo 真值。
8. 运动控制器读取专用本体仿真状态；真值评价 topic 与本体状态用途分别记录，不提供给 SLAM/Nav2。
9. 提供 GUI 与无 GUI 启动配置，实际验证传感器的渲染需求。

验证：XML/SDF 解析、官方 Fortress 的 SDF 转换/检查、实际 spawn、关节状态和 IMU/点云频率、frame、时间戳、静态 TF。检查传感器安装位置没有因 URDF 固定关节合并而改变。

## 6. 阶段四：传感器适配、扰动与录制

新增主要文件：

- `course_sim/course_sim/cloud_format.py`、`sensor_disturbance.py`
- `course_sim/course_sim/imu_attitude.py`
- `course_sim/launch/record.launch.py`
- `course_sim/test/test_cloud_format.py`、`test_sensor_disturbance.py`

实施步骤：

1. 检查输入 PointCloud2 字段、point_step、row_step、大小端、padding 和有效点；保留原始 beam 关系。
2. 输出带 `x/y/z/intensity/ring/time` 的点云，以实际扫描模型明确时间单位。快照扫描的偏移为零，不伪造旋转去畸变数据。
3. 使用 sensor QoS 处理点云/IMU；按 profile/seed 施加测距噪声、逐点丢弃、IMU 噪声与 bias。
4. 如需姿态输入，使用惯性测量生成姿态，不从真值位姿复制。检查无效 quaternion、初始化和时间重置。
5. 实现轮地物理扰动，优先使用 Fortress 支持的接触/摩擦/滑移设置；实际验证所选 physics 后端支持的字段，记录扰动等级和测得滑移。
6. 建立完整密度默认配置与显式 reduced-density 配置；实验记录保留实际点频和密度。
7. 为四个 profile 提供候选数值及 baseline 校准说明，避免将实验设置当作实机噪声参数。
8. rosbag 记录 `/pointcloud`、`/imu`、`/sim/ground_truth/odom`、`/tf`、`/tf_static`、`/clock`，并保存配置 sidecar。

验证：测试 padding/大小端/NaN 点与 ring 保持、固定 seed 的扰动逻辑、非法参数拒绝和 IMU 时间处理；实际 rosbag 录制回放与 profile 生效；物理滑移单独验收。

## 7. 阶段五：course_slam 后端与输出适配

新增主要文件：

- `scripts/fetch_lio_sam.sh`
- `course_slam/package.xml`、Python package 配置
- `course_slam/config/lio_sam.yaml`
- `course_slam/course_slam/pose_adapter.py`、`transforms.py`
- `course_slam/launch/slam.launch.py`、`replay.launch.py`
- `course_slam/test/test_transforms.py`

实施步骤：

1. 获取官方 LIO-SAM ROS 2 源码并固定 revision，保留许可证；第三方源码放在独立目录，依赖检查明确 GTSAM/PCL 等需求。
2. 阅读该 revision 的输入/输出代码及参数后，配置 XT32 点云、IMU、外参、QoS、frame 和 `use_sim_time`。
3. 后端 TF 隔离至内部 topic，避免公共 TF 多源发布。
4. 以同一时间的连续位姿与优化位姿产生公共 `odom → base_link` 和 `map → odom`。固定外参转换包含 pose、twist 和 covariance，不能只改 frame_id。
5. 发布 `/slam/odom`、`/slam/map_points`，保留后端日志与数据来源。
6. 分别提供真实仿真输入与独立 rosbag 输入入口；缺少后端时明确失败。

验证：真实后端构建与数据运行、位姿/地图产生、TF 单发布者、外参解析已知答案测试、协方差有效性、bag 回放、时间重置。没有输出时检查字段与姿态，而不替换成真值定位。

## 8. 阶段六：定位评价

新增主要文件：

- `course_slam/course_slam/trajectory.py`、`evaluate.py`
- `course_slam/test/test_evaluation.py`

实施步骤：

1. 将估计/真值转换为基座轨迹并导出 CSV/TUM；支持 rosbag2 读取和可独立运行的轨迹输入。
2. 按时间戳关联，配置最大偏差和 RPE 时间间隔，报告匹配覆盖率。
3. 实现不调整尺度的 SE(3) 对齐 ATE，并另输出未经对齐的误差。
4. 实现平移和旋转 RPE；拒绝重复/乱序/非有限数据及不可归一化四元数。
5. 输出 JSON/CSV，包含指标单位、对齐方式、有效样本数和失败原因。

验证：零误差、已知平移/旋转、尺度误差不被修正、稀疏时间匹配及空数据场景；实际 bag 评价与轨迹图检查。

## 9. 阶段七：地图与 Nav2

新增主要文件：

- `course_navigation/package.xml`、Python package 配置
- `course_navigation/course_navigation/map_projection.py`、`map_node.py`
- `course_navigation/config/nav2.yaml`
- `course_navigation/launch/navigation.launch.py`
- `course_navigation/test/test_map_projection.py`

实施步骤：

1. 根据 SLAM 地图投影障碍格子，根据有真实观测来源的实时点云射线标记 free；其余区域为 unknown。
2. 点云与射线起点按对应时间的 SLAM TF 变换，处理高度范围、self-filter、分辨率、地图扩展与原点；不使用真值清空地图。
3. 发布带正确 QoS 的 `/map`；保存固定地图并提供独立固定地图运行模式。
4. 配置 Humble 版本的 Nav2 planner、controller、behavior、BT navigator、生命周期和 costmap。
5. footprint 按当前模型占地范围设置，确认包含腿轮外伸范围，不仅取机身 collision box。
6. 配置 `/slam/odom`、base frame、点云障碍层和内部速度 topic；在线/固定地图配置明确分开。

验证：已知障碍和射线的栅格答案、unknown 保留、负坐标/边界地图原点、数据时间有效性；真实 Nav2 lifecycle 激活、路径输出与 costmap 可视化。

## 10. 阶段八：目标、速度与导航指标

新增主要文件：

- `course_navigation/course_navigation/velocity_adapter.py`
- `course_navigation/course_navigation/goal_manager.py`
- `course_navigation/course_navigation/metrics.py`
- `course_navigation/test/test_velocity_adapter.py`、`test_metrics.py`

实施步骤：

1. `/goal_pose` 转 NavigateToPose，处理目标替换、取消、action 拒绝与失败。
2. 内部速度转换到上游速度 topic；限速、非有限数值拒绝、超时停车及状态/TF 就绪检查。
3. 目标容差与超时使用仿真时间；状态变化只结算一次，失败与取消保存结果。
4. 计算成功、碰撞去抖计数、路径长度与到达时间；明确轨迹来源。
5. 规划延迟记录明确的规划 action 请求/响应，不将其命名为纯规划器内部耗时。
6. 记录缺失指标为 null 和原因；结果不覆盖已有实验文件。

验证：超时/时间跳变停车、限速、无效输入、成功/失败/取消/替换结算、碰撞事件去抖；真实机器人仿真运动和至少一个目标到达。

## 11. 阶段九：course_bringup 与完整联调

新增主要文件：

- `course_bringup/package.xml`、Python package 配置
- `course_bringup/launch/course.launch.py`
- `course_bringup/config/course.yaml`
- `course_bringup/rviz/course.rviz`
- `scripts/run_course.sh`

实施步骤：

1. 总 launch 提供 sim/record/slam/navigation 模式，场景、GUI/RViz、profile、seed、地图模式和录制路径参数。
2. 所有公共 topic/frame 可配置；只选择一个速度来源，手柄/SRU/Nav2 互斥启动。
3. 生命周期和状态就绪显式检查；不通过固定 sleep 假设 Gazebo、SLAM 或 Nav2 已就绪。
4. 从 install 空间运行各入口，确认文件安装完整且不依赖工作目录。
5. 实际完成 baseline 导航，再逐个运行扰动 profile，保存运行配置、日志、bag 和指标。

验证：至少一次 baseline 从传感器到 SLAM 到 Nav2 到机器人运动的闭环；失败阶段保留证据并修复，无法完成的外部依赖逐项报告。

## 12. 阶段十：文档、对齐清单与交付

新增或更新：

- `docs/COURSE_SETUP.md`
- `docs/INTERFACE_ALIGNMENT.md`
- `docs/VALIDATION.md`
- 根 README 的课程运行章节
- `.gitignore` 的依赖、bag 与结果目录规则

交付说明包括：安装、构建、单模块验收、总系统启动、数据录制回放、指标计算、实机参数替换、XT32 仿真近似和当前限制。

接口对齐清单标出实际 frame、后端选择、草案默认 topic、候选 profile、滑移变量语义和规划延迟定义，供团队确认；不擅自将 `INTERFACE.md` 改为团队已确认规范。

最终运行检查只按新改动和失败扩大，不重复无必要的测试。检查未意外改动权重、其他成员代码或已有数据。

## 13. 已知限制与执行约定

- 本机缺少多个系统依赖，网络和系统写入受到权限限制。能够完成的代码、数学验证、构建与本地运行持续推进；需要系统安装时使用已有授权尝试允许的方式，自动审批拒绝时说明具体动作和原因。
- 公共外部源码固定 revision 后再编译；缺少 `writing-plans` 技能文件，不影响本计划文档的编写，但不能声称已使用该技能。
- 不创建外部消息、部署或自动推送；不擅自修改受限 Git 元数据。
- 默认由当前 agent 顺序执行，用户确认本计划后不再为常规文件修改、测试和联调逐项请求许可。

## 实施验证状态（2026-10-10）

已安装本机依赖并实现四个课程包；12 个 package 完成构建，12 项数学/接口测试通过。真实仿真、SLAM、Nav2、两次 normal 目标、三个单 seed 扰动目标、bag 回放、轨迹评价及地图保存已验证。fixed 地图的加载与 lifecycle active 已验证。实机外参/传感器型号、物理实测打滑率、多 seed 长程统计、带手柄 SRU 和任意旧地图全局重定位仍不属于已验证结果；详见 ../../COURSE_VALIDATION.md。
