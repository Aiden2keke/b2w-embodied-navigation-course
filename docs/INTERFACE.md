# B2-W Course Project Interface Contract

本文件定义三人协作时统一使用的 ROS 2 topic、TF、实验变量和模块边界。
除非三人共同确认，否则不要单独修改这里定义的接口。

## 1. 模块分工

### A：仿真与机器人
负责：
- B2-W Gazebo 仿真
- 有限空间场景
- LiDAR / IMU / 本体传感器
- 传感器噪声与轮端打滑扰动
- 仿真 ground truth

### B：定位与建图
负责：
- FAST-LIO2 / LIO-SAM
- LiDAR-IMU 融合
- TF 与坐标系
- 实时定位与建图
- ATE / RPE 评价

### C：导航与系统集成
负责：
- Nav2
- 全局 / 局部规划
- 避障与目标点导航
- 总 launch
- 导航实验指标与可视化

---

## 2. 统一 ROS 2 Topic

| 含义 | Topic | Type | Producer | Consumer |
|---|---|---|---|---|
| LiDAR 点云 | `/pointcloud` | `sensor_msgs/msg/PointCloud2` | A | B, C |
| IMU | `/imu` | `sensor_msgs/msg/Imu` | A | B |
| Joint state | `/joint_states` | `sensor_msgs/msg/JointState` | upstream/A | debug |
| 仿真真值 | `/sim/ground_truth/odom` | `nav_msgs/msg/Odometry` | A | B-eval |
| SLAM 位姿 | `/slam/odom` | `nav_msgs/msg/Odometry` | B | C |
| 3D SLAM 地图 | `/slam/map_points` | `sensor_msgs/msg/PointCloud2` | B | RViz |
| Nav2 地图 | `/map` | `nav_msgs/msg/OccupancyGrid` | B | C |
| 导航目标 | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz/C | C |
| 导航速度命令 | `/path_manager/path_manager_ros/nav_vel` | `geometry_msgs/msg/Twist` | C | upstream locomotion |
| 碰撞反馈 | `/collision_plate/force_torque` | `geometry_msgs/msg/WrenchStamped` | A/upstream | C/eval |
| 仿真时间 | `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo | all |

重要：

`/sim/ground_truth/odom` 只能用于评价，禁止作为 SLAM 或 Nav2 的定位输入。

upstream 当前把 Gazebo odometry 映射到了：

`/dlio/odom_node/odom`

课程项目中需要将其重新区分为：

- `/sim/ground_truth/odom`：Gazebo 真值
- `/slam/odom`：真正 SLAM 输出

---

## 3. TF 约定

逻辑结构统一为：

map
└── odom
    └── robot_base
        ├── lidar
        ├── imu
        └── camera

注意：

当前 upstream 中实际的 base / lidar / imu frame 名称，在第一次运行仿真后统一确认。

不要三个人自行假设不同 frame 名称。

坐标约定：

- x：前
- y：左
- z：上

所有节点统一：

`use_sim_time:=true`

---

## 4. 扰动变量名称

统一使用以下变量名：

| Variable | Meaning |
|---|---|
| `fault_profile` | `normal / mild / medium / severe` |
| `sigma_lidar` | LiDAR range noise std |
| `p_lidar_drop` | LiDAR dropout probability |
| `sigma_imu_gyro` | Gyroscope noise std |
| `sigma_imu_acc` | Accelerometer noise std |
| `bias_imu_gyro` | Gyroscope bias |
| `bias_imu_acc` | Accelerometer bias |
| `slip_ratio` | Wheel-ground slip level |
| `goal_tolerance_m` | Goal success tolerance |
| `timeout_s` | Max navigation time |

具体数值等 baseline 跑通以后再统一确定。

---

## 5. 评价指标名称

### SLAM

- `ATE_RMSE_m`
- `RPE_trans_m`
- `RPE_rot_deg`

### Navigation

- `success`
- `collision_count`
- `path_length_m`
- `time_to_goal_s`
- `planning_latency_ms`

---

## 6. 开发边界

推荐新增：

A：
`course_sim/`

B：
`course_slam/`

C：
`course_navigation/`

系统集成：
`course_bringup/`

原则：

- 不直接修改其他成员负责的 package
- upstream B2-W locomotion controller 默认只读
- topic / TF / parameter 名称变更时，先改本文件并由三人确认
- README、INTERFACE 和总 launch 通过 PR 修改

---

## 7. 解耦开发

A 首先提供 rosbag，至少包含：

- `/pointcloud`
- `/imu`
- `/sim/ground_truth/odom`
- `/tf`
- `/tf_static`
- `/clock`

B 可以脱离 Gazebo 开发 SLAM。

B 提供固定地图和 SLAM 输出样例后，C 可以脱离 SLAM 开发 Nav2。

最终三个模块分别通过独立验收后再进行全系统联调。
