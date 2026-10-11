# 本机验证记录：2026-10-10

本记录区分代码完成、构建、实际节点运行和实验结果。机器为 Ubuntu 22.04、Humble、Fortress、RTX 4090、原生 Python 3.10。原始结构化数据见 [validation/2026-10-10-course-runtime.json](validation/2026-10-10-course-runtime.json)。

## 2026-10-11 初始化与残留会话修复

诊断时 Gazebo 和 RViz 已退出，旧 launch、运控和定位节点仍存在，日志出现时间回退与 TF_OLD_DATA。隔离复现同域重复启动：两个 clock/控制器/描述发布者，10 秒内时钟回退 41 次；单次启动的 10 秒物理最大倾角 0.220°、漂移 0.412 mm。不能将重复运行中的 TF 显示跳变直接当作物理跌倒证据。

已增加同域进程锁、默认独立 Gazebo partition，以及 Gazebo/bridge/controller 退出时整套关闭。原生进程测试确认重复启动被拒绝、原会话保持运行、关闭后无残留子进程并释放锁，见 [init-session-lifecycle.json](validation/2026-10-11-init-session-lifecycle.json)。实际 C++/ONNX 测试先复现回拨时钟复用旧力矩，再验证修复后清空旧动作、反馈、指令并等待新反馈，见 [rl-sar-controller.json](validation/2026-10-11-rl-sar-controller.json)。

20 项测试通过。仓库 51 秒物理五阶段全部通过：15 秒站立漂移 0.935 mm、最大倾角 0.222°；前进位移 2.138 m；前进后停车最大位移 0.080 m；转向 2.151 rad、最大倾角 4.822°；转向后停车最大位移 0.039 m。测量见 [init-physics.json](validation/2026-10-11-init-physics.json)。平地短测不等于长时间或实机运控验收。 最终 GUI 会话另采样 20 秒：物理站立漂移 0.737 mm、最大倾角 0.222°，时钟/传感器时间戳/TF 回退均为 0，clock、运控和描述各只有一个发布者；RViz 连续截图中模型保持可见、Global Status 为 OK。见 [运行检查](validation/2026-10-11-init-final-runtime.json) 与 [RViz 画面](validation/2026-10-11-init-rviz.png)。

## RViz 模型与地面对齐修复

当前 19 项测试通过。故障会话在域 0 同时收到 B2-W 和 marvin_robot 的 `/robot_description`，后者引用的网格包不存在；原地图零高度又处于雷达原点附近，造成模型与网格平面错位。原始消息诊断见 [validation/2026-10-10-rviz-before-diagnosis.json](validation/2026-10-10-rviz-before-diagnosis.json)。课程统一使用私有描述 `/b2w/robot_description`，脚本默认域 91、本机 UDP 回环与 SHM 通信。

在线定位从点云拟合初始地面，对地图、odom 与定位 TF 应用同一变换；固定地图关闭自动地面原点重设。实际 RViz 近景模型可见，测得四轮底面与零高度相差毫米量级，见 [近景截图](validation/2026-10-10-rviz-ground-closeup.png) 和 [运行检查](validation/2026-10-10-rviz-ground-alignment.json)。回归覆盖倾斜地面、墙面/离群点、无地面、地图与 odom 一致性，以及原生 URDF→SDF 转换后的雷达自身可见性。

地面坐标修正后暴露了顶部自身回波形成伪障碍的问题，仿真雷达现在排除本机 render geometry；它是理想自身过滤，未模拟机身遮挡环境回波。另修复 action 接口出现但 Nav2 未激活时发送目标的竞争，目标管理器等待 controller/planner/BT 的 active 状态。

最终从全新本机会话启动，仓库 normal/seed=0、map `(1,-1)` 导航成功：路径 1.577 米、耗时 6.708 秒、最终误差 0.223 米、碰撞板事件 0；独立规划请求成功、往返耗时 5.704 ms。首次启动目标被拒绝、未过滤自身回波时规划失败及最终成功的记录均保留在 [validation/2026-10-10-rviz-alignment-navigation.json](validation/2026-10-10-rviz-alignment-navigation.json)。此结果仅验证当前平地短目标。

## 开源运控复测

新增 rl_sar 控制器编译通过，14 项课程测试通过，实际 C++/ONNX 时序测试约 200 Hz 输出、50 Hz 推理，暂停更新 0。51 秒物理测试的五个阶段均通过，最后一帧轮胎位于各自正确侧；有限空间 normal/seed=0、map `(1,0)` 导航成功，路径 1.124 m、耗时 3.893 s、最终距离 0.235 m、碰撞板事件 0。详细参数、验收标准、局限和失败记录见 [LOCOMOTION_STATUS.md](LOCOMOTION_STATUS.md)。开源仓库 normal/seed=0、map `(1,-1)` 也成功：路径 1.568 m、耗时 5.660 s、最终距离 0.192 m、碰撞板事件 0。课程入口已默认选择新后端；不能与下面原策略的历史实验混在一起统计。

## 原始策略复核后的限制（历史）

后续长时间运行发现后脚交叉和无目标漂移，原始底层运控未通过稳定性验收。下面的导航成功案例是历史短目标联调记录，不能推断当前机器人能够稳定站立或长期导航。诊断、控制链路及保留的修正见 [LOCOMOTION_STATUS.md](LOCOMOTION_STATUS.md)，结构化证据见 [validation/2026-10-10-locomotion-runtime.json](validation/2026-10-10-locomotion-runtime.json)。

已修复控制器使用墙上时钟的问题：实际 C++ 控制器在受控 ROS 时钟测试中输出约 200 Hz、推理约 50 Hz，仿真暂停后更新为 0。此测试使用固定反馈，只验证时钟和数据链路，不验证机器人平衡。尝试的静态/冻结姿态方案没有通过物理验证，均已撤回。保留时钟修正、原始策略的仓库导航复测仍失败（试验起点 world `(1.5,0,1)`，到目标距离 1.4315 米、碰撞事件 5），结果保留在 `course_results/locomotion_navigation/20261010T074024.850197Z/summary.json`。默认出生位置已恢复原有 world `(0,0,1)`。

## 开源仓库与 RViz 补充验证

结构化记录见 [validation/2026-10-10-warehouse-runtime.json](validation/2026-10-10-warehouse-runtime.json)。已在原生 Fortress 加载 AWS 开源仓库的 25 个模型，Gazebo GUI 能显示完整纹理场景；实际 RViz 画面已显示 B2-W 几何模型和地图。此前普通绝对路径使 RViz 原生资源加载器读取失败，改为 `file://` 后全部 17 个网格成功读取；新增原生加载器回归测试，当前共 13 项测试通过。

仓库 10 秒采样：点云 10.00 Hz、IMU 199.80 Hz、关节状态 100.00 Hz、SLAM 里程计 10.00 Hz、全局点云地图 0.20 Hz、二维地图 1.00 Hz；800×800 栅格、分辨率 0.05 米。公开 TF 未包含真值坐标系。该采样及截图来自调整地面摩擦和到达阈值之前的版本。

最终配置下，从全新隔离会话启动到导航完成，normal/seed=0、目标 map `(1,-1)`：成功，SLAM 路径长度 1.5336 米，耗时 5.6275 秒，最终误差 0.1237 米，碰撞事件 0。结果保存于 `course_results/warehouse_experiments/20261010T062139.180627Z/summary.json`。独立规划测量首次请求因初始地图未准备充分而 aborted，耗时 3.3854 ms；Nav2 行为树后续重新规划并完成任务。

此前试运行中，map `(2,0)` 被 Nav2 判定到达但复核距离为 0.3148 米，所以课程结果保留为失败；随后 `(4,0)` 与 `(1,-2)` 的请求在靠近货架的起点规划失败，后者恢复过程中记录 3 次碰撞事件。均保留于 `course_results/warehouse_check/`，没有纳入成功统计。适配后的地面摩擦使用课程基线 1.0，Nav2 到达阈值改为 0.25 米，为课程 0.3 米验收保留余量。仓库目前完成一次正常导航验证，长路线、多随机种子与全套干扰实验仍须运行。

## 已完成的实际验证

- 四个课程 package，以及原有机器人、控制、SRU package 可构建；底层 C++ 控制器实际加载原始 ONNX 权重并驱动机器人。
- 三个 ONNX 模型均完成 CPU 推理，输出有限数值；这不代表带手柄的 SRU 导航已经实测。
- 12 项测试通过：点云大小端/行 padding/字段、beam 与 ring、噪声/dropout 可复现、IMU 姿态与时钟重置、刚体/协方差几何、ATE/RPE、不缩放、未知地图语义、速度限幅/超时/NaN、碰撞事件与缺失指标、迟到轨迹消息去重。
- 统一 course.launch.py 实际启动 Gazebo、真实 LIO-SAM、地图投影、Nav2 和目标管理器。Gazebo 真值没有进入公共 TF；正常检查时真值订阅者只有评价探针。
- rosbag 实际录制；轨迹导出和评价完成。单独 ROS domain 中只回放测量、静态外参、时钟和评价真值，重新产生 220 条 SLAM 位姿与 4 条地图消息；没有回放原估计轨迹/动态 TF。
- 实际保存二维 PGM/YAML 和 SLAM 点云 PCD；fixed 地图模式在真实回放 SLAM 输入下加载该地图，Nav2 与 map_server 已实际进入 active。

最终 normal 检查约 20 秒，实际接收频率（按消息仿真时间戳计算）：

| 数据 | 实测 |
|---|---:|
| 点云 | 10.00 Hz，约 6.2 万有效点/帧 |
| IMU | 199.23 Hz |
| 关节状态 | 99.90 Hz，16 个运动关节 |
| SLAM 里程计 | 10.00 Hz |
| 后端全局点云地图 | 0.20 Hz |
| 二维地图 | 1.00 Hz |

有效点数小于 64000 是因为无返回/无效测量被过滤，并非降低配置采样密度。频率是该次订阅采样结果，不是每次运行的性能保证。物理 step 为 2.5 ms，让 200 Hz IMU 周期与 physics step 对齐；公共关节状态约 100 Hz，保留原有 200 Hz 控制输出。

## 导航联调结果

所有目标为 map 中 (1,0) 或后续 (2,0)，容差 0.30 m。mild/medium/severe 各从单独场景初始化，seed=0，timeout=15 s。normal 两个目标连续运行，timeout=45 s。

| 测试 | success | 路径 m | 仿真时间 s | 碰撞板事件 | 规划 action 往返 ms |
|---|---|---:|---:|---:|---:|
| normal 目标 1 | true | 1.075 | 3.675 | 0 | 3.020 |
| normal 目标 2 | true | 0.567 | 2.332 | 0 | 1.376 |
| mild | true | 1.060 | 3.782 | 0 | 2.568 |
| medium | true | 1.097 | 4.797 | 0 | null |
| severe | true | 1.175 | 4.113 | 0 | null |

该轮 medium/severe 的独立规划测量请求没有取得成功结果，当时的实现保留 null；Nav2 行为树后续规划和导航完成。随后完善了测量逻辑：失败 action 的耗时也保留，并单独记录状态。复测 medium 仍成功到达（4.490 s），独立规划测量为 1.412 ms、status=aborted，不能当作成功规划耗时比较。路径由真实 SLAM 基座轨迹积分，碰撞只覆盖 collision plate。轮端 WheelSlip 系统已接入，原生 DART 插件包含 slip compliance 功能；未单独测量实际运动学打滑率，不能据此得出物理打滑与导航性能的定量关系。

以上为单 seed、短目标的联调测试，不能宣称四种扰动下长期导航均可靠。批量脚本保留每次失败和结果，可继续增加目标、seed 和重复次数。

## 轨迹评价

较早一次修复点云传输后的 normal 录制包含 1186 个匹配 SLAM 位姿（覆盖率 100%），20 ms 时间匹配、1 秒 RPE、SE(3) 对齐且不缩放：

| 指标 | 值 |
|---|---:|
| ATE_RMSE_m | 0.069108 |
| RPE_trans_m | 0.046787 |
| RPE_rot_deg | 0.419597 |
| 未经对齐位置 RMSE m | 5.554190 |

该 bag 来自早期 physics step 和频率配置，不作为最后版本的统一 benchmark。未经对齐误差包含 Gazebo world 与 SLAM 初始化原点/朝向差异，不能与对齐 ATE 混用。

## 修复过的问题与保留的边界

- 默认 Fast DDS SHM 容量下大点云曾只有约 2 Hz，并引发明显定位漂移；独立 64 MiB segment 配置后恢复 10 Hz，实时与回放均验证。
- 本机双显卡 EGL 设备选择曾导致 Ogre2 启动失败；有 X 显示时的无 GUI 服务器渲染已验证。纯无显示 EGL 尚未通过。
- 较短冷启动等待与不合适的近距离过滤曾导致定位漂移，完整入口当次目标失败；失败 JSON 保留在 course_results/final_navigation，没有纳入成功统计。最终默认等待 20 秒并使用已验证的近距离配置。
- LIO-SAM Best Effort QoS 已匹配；后端 TF 隔离，修复了相同 lidar/base frame 时内部 TF 空 parent 和空队列访问。
- 目标管理器兼容 RViz/CLI QoS；Nav2 内置 goal_pose 入口已 remap，保证只由课程入口派发 action 和记录指标。
- 原始 ONNX 策略、URDF 外参并不意味着实机配置一致。实际 XT32 铭牌、逐通道标定、IMU 型号和外参仍需实机确认。
- fixed 地图加载已提供，但要求已知 SLAM→旧地图配准，不包含任意旧地图的自动全局重定位；带旧地图导航和硬件手柄/SRU 未作为上述成功案例验证。

本机原始 bag、地图、失败和成功结果保留在忽略提交的 course_results/。可提交的 JSON 证据及文档不包含密码。
