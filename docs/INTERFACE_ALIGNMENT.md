# 接口实现与团队对齐项

本文件记录本机实现采用的默认值，不替代三人共同确认 [INTERFACE.md](INTERFACE.md) 的流程。原接口文件保持不变。

| 项目 | 本机实现 | 需要确认/测量的内容 |
|---|---|---|
| 机器人 | 仓库 B2-W URDF；原有策略与开源 rl_sar 16 关节策略可选 | 实机具体版本和已有传感器清单；新控制器见 [RL_SAR_SETUP.md](RL_SAR_SETUP.md) |
| LiDAR | 用户选定 XT32 仿真基线，32 线、10 Hz、2000 水平采样、−16°…+15° | 图片只证明宣传标注 32 线，不能证明具体型号；需铭牌、驱动或采购清单确认 |
| IMU | imu_link 上 200 Hz 仿真 IMU，陀螺/重力姿态滤波 | 实机 IMU 型号、噪声、偏置、是否提供厂商融合姿态均未确认 |
| base→LiDAR | xyz [0.34218, 0, 0.17851] m；rpy [0,0,0] | 来自仓库 URDF，未做实机外参标定 |
| base→IMU | xyz [0, −0.02341, 0.04927] m；rpy [0,0,0] | 同上，IMU 与雷达间平移已提供给 LIO-SAM |
| 公共 TF | map→odom→base_link→lidar_link / imu_link / 相机实际 link | 草案中的 robot_base/lidar/imu 为逻辑名称；团队确认实际 frame 名 |
| 在线坐标原点 | 点云初始地面定义 map/odom 的 z=0；地图、里程计和 TF 采用同一刚体变换 | 当前仅平地；固定地图使用既有配准，不重新定义高度 |
| 雷达自身回波 | SDF visibility mask 排除本机 render geometry，公共点云为理想自身过滤后的环境扫描 | 未模拟机身遮挡；实机需独立做自身回波过滤 |
| 本机描述/通信 | /b2w/robot_description；脚本默认 ROS_DOMAIN_ID=91、ROS_LOCALHOST_ONLY=1，并限制 UDP 到回环接口 | 其他终端使用同一域与 DDS profile；联网时显式调整环境参数 |
| 公共 topics | 保留原接口表，真实 LIO-SAM 产生 /slam/odom 和 /slam/map_points | /map 投影代码目前在 course_navigation，B/C 确认维护归属 |
| 真值 | /sim/ground_truth/odom 只用于评价/录制 | 同一 Gazebo 状态另发 /sim/body_state 供原底层本体控制器；不进入 SLAM/Nav2 |
| 新底层反馈 | rl_sar 读取 /sim/raw/joint_states 和 /sim/raw/imu 的最新消息 | 私有高频本体反馈；公开 /joint_states、/imu 和导航接口保持既定定义 |
| TF 发布者 | RSP 管机器人内部 TF；pose_adapter 管公共定位 TF | LIO-SAM 原始 TF 隔离到 /lio_internal/tf；Gazebo 真值 TF 不桥接 |
| backend | 固定官方 LIO-SAM ros2 commit、GTSAM 4.1.1 | 后续如选择 FAST-LIO2，需要另做点云/外参/TF 验证 |
| 扰动 | 四个候选 profile，独立 LiDAR/IMU RNG，轮端物理 compliance | 团队确认数值与实验协议，实测打滑率需单独评价 |
| 导航结果 | 五个约定指标，缺失写 null，失败/替换/取消保留 JSON | 碰撞阈值、plate 覆盖范围、目标容差、超时及规划延迟定义 |
| 定位不确定度 | 上游 mapping covariance[0] 实际为退化标志，公共消息不伪造协方差 | 若需要统计一致的 pose/twist covariance，应扩展后端或做独立估计 |

XT32 名义参数依据 [Hesai 官方手册](https://www.hesaitech.com/wp-content/uploads/2025/02/PandarXT_User_Manual_X01-en-250220.pdf) 和 [官方下载页](https://www.hesaitech.com/product_downloads/xt32-16/)。理想模型采用均匀 beam 角度，ring 从低到高；实机逐通道角度、发光时间、回波模式和硬件 ring 顺序需要设备自己的标定文件。雷达外观沿用仓库 mesh，没有据宣传图推断精确实体尺寸。

当前扫描是 Gazebo 瞬时快照，点 time 全为 0；公开 `/imu` 的 yaw 由陀螺积分，没有磁场或绝对航向输入，orientation 不使用 Gazebo 真值。低层 rl_sar 使用私有原始仿真 IMU 的理想姿态和角速度；它们没有进入 SLAM/Nav2。噪声/偏置与 orientation covariance 是仿真滤波设置，不是厂家测量值。

本机大点云传输采用独立 Fast DDS profile：[官方 SHM 配置说明](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html)。完整点云超过默认 segment 容量时曾实际丢失，修复后的频率和定位结果单独记录在验证报告中。物理扰动参数使用 [Gazebo WheelSlip](https://raw.githubusercontent.com/gazebosim/gz-sim/ign-gazebo6/src/systems/wheel_slip/WheelSlip.cc) 的定义。
