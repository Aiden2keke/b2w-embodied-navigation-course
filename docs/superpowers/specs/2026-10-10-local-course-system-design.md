# B2-W 课程系统本机适配设计

日期：2026-10-10

状态：用户已确认本机路线、XT32 仿真雷达基线与本设计；本文不代表三人团队已经确认接口。

## 1. 目标与约束

用户要求适配本机，并按 `docs/INTERFACE.md` 中提出的课程要求完成仿真、定位建图、导航和系统集成。用户已选择本机路线，并根据所提供宣传图的“32线车规级激光雷达”标注，选择禾赛 XT32 作为仿真雷达基线。图片只证明宣传配置为 32 线，并不独立证明具体实机的品牌、型号和标定外参。

目标环境为 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress，ROS Python 节点使用 `/usr/bin/python3` 的 Python 3.10。保留现有机器人模型、运动控制策略及原有启动入口；新增课程入口完成独立配置和集成。

`INTERFACE.md` 是待团队对齐的修改要求。实现采用其中的 topic、指标和模块边界作为默认值，允许通过配置调整。不得将本文中的实现选择描述为团队已达成的决定。

交付包括四个 ROS 2 package、环境检查与安装说明、分阶段运行入口、数据记录和评价工具、必要测试及实际验证记录。实机部署、重新训练运动策略及同时维护两个 SLAM 后端不在此次范围内。

## 2. 已确认的现状

- 本机安装了 ROS 2 Humble、colcon、robot_state_publisher、tf2_ros 和 RViz2。
- `/usr/bin/python3` 为 Python 3.10，能够找到 rclpy、launch、launch_ros 和 OpenCV。
- shell 默认的 Python 为 3.14，不能直接用于 Humble 节点与构建脚本。
- 尚未找到 Gazebo CLI、ros_gz_sim、ros_gz_bridge、Nav2 和 Python ONNX Runtime。
- 现有运动策略、导航策略和深度编码器的 ONNX 文件均存在。
- `load.launch.py` 实际加载 `b2w_description_rsl.urdf`，其根 link 为 `base_link`。
- 此 URDF 中已有 `lidar_link`、`imu_link`、深度相机及关节控制插件，但没有 LiDAR/IMU sensor 定义。
- 当前 Gazebo 真值里程计被桥接到 `/dlio/odom_node/odom`，且真值 TF 被桥接到公共 `/tf`。
- 现有模型采用 Harmonic 的 `gz-sim-*` 插件名称；课程入口需要适配 Fortress 的 Ignition 插件及消息名称。

## 3. 架构与数据流

```text
Gazebo Fortress + B2-W + 有限空间场景
  ├── 原始 LiDAR/IMU → course_sim 扰动与格式适配 → /pointcloud、/imu
  │                                                   ↓
  │                                          LIO-SAM ROS 2 后端
  │                                                   ↓
  │                                        course_slam 输出适配
  │                                        ├── /slam/odom
  │                                        ├── /slam/map_points
  │                                        └── 公共定位 TF
  │                                                   ↓
  │                                        二维地图投影 → /map
  │                                                   ↓
  │                                      Nav2 → 速度适配与限速
  │                                                   ↓
  │                       /path_manager/path_manager_ros/nav_vel
  │                                                   ↓
  │                                      现有 ONNX 运动控制器
  ├── 关节状态 → /joint_states
  ├── 碰撞反馈 → /collision_plate/force_torque
  └── 真值 → /sim/ground_truth/odom → 记录与评价
```

真值不得进入 SLAM、Nav2 或公共定位 TF。底层运动控制器如需独立的本体状态输入，使用明确标记的内部仿真状态 topic；该输入不提供给高层定位与导航，并在实验记录中说明。

课程默认采用 LIO-SAM 官方 `ros2` 分支作为一个完整后端；记录具体源码 revision 和许可证。其 ROS 2 分支文档明确列出 Humble/Ubuntu 22.04 测试环境。此次不承诺同时实现 FAST-LIO2。保留后端适配边界，允许队友后续替换。

## 4. package 边界

### 4.1 course_sim

负责机器人仿真、场景、传感器、扰动、真值及采集入口。

实现内容：

- 从现有 URDF 生成课程用描述，转换 Fortress 插件名称及 sensor/bridge 配置，避免直接重写上游运动策略。
- 在已有 lidar/imu 固定连接处加入 XT32 参数的 32 线 3D LiDAR 和机身 IMU。传感器外参从实际模型读取，标为仓库默认值而非实机标定结果；不额外假设位置。
- 使用自包含 SDF 场景，提供走廊、窄通道、转角及障碍物；不依赖启动时下载远程模型。
- 原始传感器使用 `/sim/raw/pointcloud` 和 `/sim/raw/imu`，公开经过扰动后的 `/pointcloud` 和 `/imu`。
- 真值经专用桥接输出 `/sim/ground_truth/odom`，不发布 `odom → base_link` 公共 TF。
- 保留关节命令与状态、深度图和碰撞反馈，提供单独仿真与手动运动的验收入口。
- 提供 rosbag 录制入口，至少包含接口草案要求的六个 topic；另录制运行配置和传感器参数。

扰动规则：

- `sigma_lidar` 沿测距方向添加零均值噪声，保留合法字段布局与 frame。
- `p_lidar_drop` 明确表示逐点丢弃概率；处理有组织点云时保留 beam 索引信息，不能由过滤后的数组下标推导 ring。
- `sigma_imu_gyro`、`sigma_imu_acc` 和对应 bias 作用于测量值，统一单位为 rad/s 和 m/s²。
- orientation 输入的来源明确记录。若后端需要有效姿态但传感器不提供，则通过陀螺积分和重力方向生成姿态初值/估计；不得从真值里程计复制姿态。
- `slip_ratio` 是草案中的轮地打滑等级控制变量，通过轮地接触/摩擦设置影响仿真物理行为；仅缩放速度命令不算打滑实现。配置等级和实际测得的打滑程度分别记录，不声称二者数值相等。
- `fault_profile` 支持 normal/mild/medium/severe，各 profile 的初始数值标为候选实验设置。baseline 验收后再形成可比较的实验配置。
- 随机扰动支持 seed；固定 seed 不等于保证整个物理仿真逐位复现。

#### 4.1.1 XT32 与机身 IMU 基线

| 参数 | 课程默认值 | 依据与适用范围 |
|---|---|---|
| 雷达型号配置名 | `hesai_xt32` | 用户选择的仿真基线；实机型号仍需铭牌/驱动核实 |
| 垂直通道数 | 32 | 用户图片标注与禾赛 XT32 官方资料 |
| 水平视场 | 360° | 禾赛 XT32 官方规格 |
| 垂直视场跨度 | 31°，名义通道间隔 1° | 禾赛 XT32 官方规格；实机各通道使用对应角度标定文件 |
| 量程 | 0.05–120 m | 禾赛 XT32 官方典型规格；仿真不实现材质反射率对应的真实探测概率 |
| 名义单回波点频 | 640,000 点/s | 禾赛 XT32 官方规格 |
| 课程扫描发布频率 | 10 Hz | 课程运行配置，须与实际驱动设置分别核对 |
| 名义每帧水平采样 | 2,000 | 由 640,000 / 10 / 32 推导；完整帧名义共 64,000 点 |
| 测距标准差基线 | 0.005 m | 官方典型 precision 0.5 cm（1σ）；扰动 profile 可增加噪声，不混同系统准确度 |
| 基线扫描模型 | Gazebo 瞬时快照 | 明确的仿真近似；逐点时间偏移为零，不宣称复现旋转过程中的运动畸变 |
| 雷达外参 | xyz=(0.34218, 0, 0.17851) m，rpy=(0,0,0) | 当前 URDF 的 `base_link → lidar_link`，不是地面高度或实机标定 |
| 机身 IMU 外参 | xyz=(0,-0.02341,0.04927) m，rpy=(0,0,0) | 当前 URDF 的 `base_link → imu_link`，不是已确认的芯片安装标定 |
| 机身 IMU 发布频率 | 200 Hz | 满足当前 LIO-SAM 基线输入要求的课程设置，实机输出频率另行核对 |

IMU 使用机身惯性测量的独立仿真配置；不将 MID-360 的内置 IMU 参数移植到 XT32 上。未经芯片型号、驱动与实机数据确认，IMU noise/bias 数值只作为可配置实验变量，不标成实机规格。

默认保持完整名义采样密度。若本机性能无法满足实时运行，可通过明确命名的 reduced-density 配置降低水平采样数；日志和实验元数据必须记录该选择，不把降低后的点频继续标为 XT32 标称点频。

机身 link 外参、雷达 beam 参数及时间模型分别可配置。匹配名义线数与视场不等于完全复现实机点云；反射率、遮挡、回波、每通道出厂标定和扫描运动畸变等差异单独记录。

### 4.2 course_slam

负责真实 LiDAR–IMU SLAM 接入、输出适配、坐标转换和定位评价。

实现内容：

- 提供可重复获取和构建 LIO-SAM ROS 2 的方式，固定源码 revision；第三方依赖与课程源码分别管理。
- 检查 PointCloud2 的坐标、intensity、ring 和逐点时间字段，明确扫描类型与单位。
- Gazebo LiDAR 若为瞬时快照，各点时间偏移统一为零并记录此事实，不人为伪造滚动扫描采样时间；这种配置不能用于证明真实旋转 LiDAR 去畸变性能。
- ring 由已知 beam 元数据/垂直采样配置产生，禁止对任意无结构点云随意补 ring。
- 缺少必要字段、外参、IMU 姿态或数据时，给出具体诊断并禁止静默启动不完整的估计链路。
- 将后端位姿按固定外参转换为机器人基座位姿，输出 `/slam/odom`；协方差和 twist 随位姿 frame 一致转换，不能只修改 header 名称。
- 输出 `/slam/map_points`，保存轨迹和地图，并支持脱离 Gazebo 的 rosbag 回放模式。
- 提供 `ATE_RMSE_m`、`RPE_trans_m` 和 `RPE_rot_deg` 离线评价。

评价规则：

- 根据时间戳匹配估计与真值，设置最大匹配误差；报告有效匹配数量和覆盖率。
- ATE 使用明确记录的 SE(3) 刚体对齐，不调整尺度；同时输出未经对齐的误差，防止对齐掩盖 frame 配置错误。
- RPE 使用配置的固定时间间隔，单位分别为 m 和 deg。
- 全部基于有效机器人基座轨迹计算；缺失数据、异常四元数、仿真时间重置分别处理。
- 无有效数据时输出失败原因，不能写出看似成功的零误差结果。

### 4.3 course_navigation

负责地图投影、Nav2、目标输入、速度适配和导航评价。

实现内容：

- 将 `/slam/map_points` 按配置高度范围投影成 `/map`。地图 frame、原点、分辨率和未知区域语义明确一致。
- 全局地图只根据 SLAM 输出形成；未知空间默认保持 unknown，采用明确配置的清空/raycast 规则才能标记 free。不能把所有未落入障碍点的格子直接当作已知自由空间。
- 支持在线建图与固定地图模式，分别提供配置；两种模式均使用真实定位输出。
- Nav2 使用 `/slam/odom` 和公共 TF；局部 costmap 使用 `/pointcloud` 作为障碍输入。
- 为 B2-W 的尺寸配置 footprint、膨胀半径和速度限制；初始使用保守的平面速度范围。
- 将 Nav2 内部速度 topic 适配到草案要求的速度 topic；设置超时停车、限速和数据就绪检查。
- 提供 `/goal_pose` 到 NavigateToPose action 的入口，明确单目标、替换目标与取消行为。
- 默认课程模式只启用一个速度来源；手柄与已有 SRU 模式通过启动参数互斥选择，避免多个 publisher 争抢机器人。

导航指标：

- `success`：action 成功且基座进入配置的 `goal_tolerance_m`。
- `collision_count`：基于碰撞反馈阈值和事件去抖，不把连续消息重复计数；检测范围只包含实际接入的碰撞反馈来源。
- `path_length_m`：累计有效基座轨迹的平面距离，默认使用 SLAM 轨迹；可另输出真值路径长度并标注来源。
- `time_to_goal_s`：使用仿真时间，从目标接受到成功/失败/超时。
- `planning_latency_ms`：用 monotonic clock 记录明确的规划 action 请求到返回耗时，标注为 action 往返延迟。无法获取规划器内部耗时时不能将此指标声称为纯算法计算时间。
- 每次运行输出结构化结果与配置；超时和失败保留结果，数据缺失写 null 和原因。

### 4.4 course_bringup

负责统一 launch、参数配置、依赖检查与分阶段联调。

提供入口：

1. 仿真与运动控制。
2. 仿真数据采集。
3. rosbag 回放 + SLAM。
4. 仿真 + SLAM + 地图。
5. 仿真 + SLAM + Nav2 完整系统。

总入口参数包括场景、GUI/RViz、fault_profile、随机 seed、参数文件、控制模式、地图模式及录制目录。实际 ROS 参数统一 `use_sim_time=true`；规划耗时等 wall-clock 测量与仿真时间明确区分。

## 5. TF 与接口决策

实际 URDF 根 frame 是 `base_link`，课程默认先使用：

```text
map → odom → base_link
                 ├── lidar_link
                 ├── imu_link
                 └── 相机实际 link
```

草案的 `robot_base/lidar/imu/camera` 暂视为逻辑名称，实际名称写入配置与接口对齐清单。若团队最终要求改变名称，统一修改机器人描述和所有配置，避免向既有树反向增加别名而造成两个 parent。

TF 归属：

- robot_state_publisher 发布机器人内部固定和关节 TF。
- SLAM 输出适配层负责公共 `odom → base_link` 与 `map → odom`。
- 后端的原始 TF 放在私有 topic，或明确禁用重复发布；适配层使用后端连续局部位姿与优化地图位姿计算校正，不同时发布静态及动态 `map → odom`。
- Gazebo 真值 TF 不进入公共树。
- 每条公共 TF 只能有一个发布者；启动验收检查 parent、发布来源、时间戳及外参。

数据接口默认保留 `INTERFACE.md` 的公共 topic。新增的原始传感器、后端 topic、内部速度与诊断 topic 属于各模块内部接口。

## 6. 本机环境适配

- 安装/构建操作显式选择 Humble 与 Python 3.10，防止默认 Python 3.14 覆盖构建解释器或 console-script shebang。
- 系统依赖使用与 Humble/Fortress 配对的 ros_gz 包；不同时安装冲突的 Harmonic ros_gz 包。
- Python 额外依赖放在工作区虚拟环境，允许访问 ROS 系统包；保留原始 setup 脚本并新增课程安装入口。
- 环境检查覆盖 CLI、ament package、动态库、模型路径、Python 模块及图形/无显示模式。
- 安装脚本明确区分检查、生成安装命令和执行安装，不隐藏使用 sudo，不擅自更换系统 Python。
- 参数、描述和 bridge 文件安装进 package share；安装后运行不能依赖源码 cwd。

## 7. 错误处理与实验记录

传感器缺失、时间停滞/重置、SLAM 未初始化、TF 不完整、Nav2 未激活时，以可检查的日志/诊断说明原因。导航链路在缺少有效状态或速度输入超时时输出零速度。

运行记录至少包括：源码版本、第三方版本、场景、seed、fault_profile 数值、传感器设置、地图设置、目标、评价参数、运行结束原因和指标来源。真实仿真结果与测试合成数据分别存放并标明用途。

## 8. 实现顺序与验收

| 阶段 | 工作 | 通过条件 |
|---|---|---|
| 1 | 环境与现有包适配 | 依赖检查、目标 package 构建、实际 Fortress 启动成功 |
| 2 | 仿真与数据 | 机器人可控，点云/IMU/关节/真值存在，frame 与时间有效，rosbag 可回放 |
| 3 | SLAM | 后端实际产生位姿与地图，公共 TF 无冲突，评价可产生有效匹配结果 |
| 4 | 导航 | Nav2 激活，目标被接受，产生速度，机器人在至少一个场景完成闭环目标导航 |
| 5 | 扰动与评价 | 四个 profile 可运行，扰动实际生效，失败/成功均记录，指标可导出 |

必要自动化验证覆盖有风险的边界：PointCloud2 字段与索引保持、外参变换、轨迹关联与评价、未知地图语义、速度超时停车、目标状态与指标结算。使用解析已知答案的合成输入检验数学逻辑；不以合成测试代替仿真验收。

验证结果分别记录为静态检查、构建、节点运行、仿真闭环和实验结果。缺少依赖或权限时继续完成不受影响的实现，逐项列明未通过的验收和具体原因；不能把只写好 launch 文件描述为系统已经跑通。

## 9. 待团队对齐的具体选择

以下是本实现的候选默认值，需在团队对齐时明确：

- 实际 frame 使用 `base_link/lidar_link/imu_link`。
- XT32 是用户选定的仿真基线，32 线图示不替代实机型号与外参核验；机身 IMU 与雷达配置分别管理。
- 先完成 LIO-SAM 一个后端，保留替换接口。
- 仿真 LiDAR 为快照扫描，逐点时间偏移为零。
- 四个扰动 profile 的数值须在 baseline 后确定。
- `slip_ratio` 表示扰动等级，实际滑移量独立测量。
- 规划延迟记录 action 往返耗时，若要求纯规划器耗时须增加后端测量。

这些待团队对齐的选择不阻止按可配置默认值进行开发，但不得用未确认值宣称最终实验结论。

## 10. 依据

- 项目：`README.md`、`docs/INTERFACE.md`、实际 URDF、现有 launch/bridge/controller 配置。
- [Gazebo 官方 ROS 版本配对说明](https://gazebosim.org/docs/fortress/ros_installation/)：Humble 的推荐配对为 Fortress。
- [LIO-SAM 官方 ROS 2 分支](https://github.com/TixiaoShan/LIO-SAM/tree/ros2)：Humble 支持说明与 LiDAR/IMU 输入要求。
- 用户提供的 B2 宣传图片：标注“32线车规级激光雷达”。
- [禾赛 XT32/16 官方下载页](https://www.hesaitech.com/product_downloads/xt32-16/)：分别提供 32 线与 16 线手册和角度标定资料。
- [禾赛 XT32 官方产品规格](https://www.hesaitech.com/product/xt16-32-32m/)：视场、量程、名义点频和典型测距精度。
