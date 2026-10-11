# B2-W 官方公开运控资料核查

核查日期：2026-10-10。范围：宇树 GitHub 组织 `unitreerobotics` 下的官方模型、SDK、ROS 2 和三个 RL 仓库；另核对当前工程所用权重的上游来源。

## 结论

找到了 B2-W 官方机器人模型、底层电机命令示例和实机高层运动接口，但在本次检查的官方 RL 仓库默认分支中，**没有找到与当前 `policy_force_new.onnx` 对应的 B2-W 训练配置或权重**。不能据此断言所有分支、历史版本或其他未检查的仓库都没有公开资料。

模型、通信 SDK 和行走控制器是三个不同部分。官方 MuJoCo 仿真可以加载 B2-W，并接收低层电机命令；其 README 明确说明当前主要支持低层开发、用于验证用户自己的控制器。因此，安装官方仿真与 SDK，并不会自动得到实机内置的完整行走运控。[官方 MuJoCo README](https://github.com/unitreerobotics/unitree_mujoco#readme)

## 已核实的官方资料

| 官方来源 | 公开内容 | 对当前问题的用途与边界 |
|---|---|---|
| [unitree_ros 的 B2-W 模型](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/b2w_description) | URDF、网格、机器人几何和惯性参数 | 可以核对模型版本与关节轴；不能恢复 ONNX 的训练配置。 |
| [unitree_mujoco 的 B2-W 模型](https://github.com/unitreerobotics/unitree_mujoco/tree/main/unitree_robots/b2w) | `b2w.xml`、平地与地形场景 | 可作为官方模型的另一套物理仿真参照；更换模拟器仍须提供运控。 |
| [SDK2 B2-W 底层示例](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/b2w/b2w_stand_example.cpp) | `rt/lowcmd` / `rt/lowstate`、电机位置/速度/增益/力矩字段、站起和轮子转动示例 | 可建立独立站立基线；这是演示程序，不是当前 RL 权重的训练参数。 |
| [ROS 2 B2-W Sport 示例](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/b2w/b2w_sport_client.cpp) | `StandUp`、`StopMove`、`Move(vx, vy, vyaw)` 等实机高层请求 | 可适配导航速度指令到实机内置运动服务。其 [客户端实现](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/common/ros2_b2_sport_client.cpp) 发送 API 请求，没有实现步态或平衡算法。 |

SDK2 示例的站立目标每条腿为 `[0, 0.67, -1.3]` rad，示例腿部增益 `Kp=1000`、`Kd=10`，写入周期 2 ms；轮电机索引为 12–15，以速度目标控制。这些参数与当前 Gazebo 腿部 `P=100、D=3.5` 不同，但**不能把硬件示例增益直接认定为 Gazebo RL 应使用的增益**：电机接口、力矩限幅、离散周期和物理引擎均须单独验证。[SDK2 源码](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/b2w/b2w_stand_example.cpp)

## 官方 RL 仓库检查范围

下表版本通过 GitHub 默认分支递归目录 API 核对，返回均未截断；三个 RL 仓库目录中均未出现 `b2w` 或 `policy_force` 路径。下面列的是 README / 注册配置实际支持的范围，不将同一目录中的其他机器人部署骨架视为 B2-W 策略。

| 仓库及检查版本 | 公开训练支持范围 |
|---|---|
| [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym/tree/276801e46c5d433564f24658bac64f254b7d2d4b) | Go2、H1、H1_2、G1；[任务注册源码](https://github.com/unitreerobotics/unitree_rl_gym/blob/276801e46c5d433564f24658bac64f254b7d2d4b/legged_gym/envs/__init__.py) 没有 B2-W。 |
| [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab/tree/4960b84732b0c2ec593dccbfe963fda1bcd7b1e3) | README 列出 Go2、H1、G1-29dof；[locomotion 目录](https://github.com/unitreerobotics/unitree_rl_lab/tree/4960b84732b0c2ec593dccbfe963fda1bcd7b1e3/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion) 未找到 B2-W 任务。 |
| [unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab/tree/1425b15f73bd4095f0df53709d7c389c3eb9e790) | README 列出 Go2、A2、As2、G1、R1、H1_2、H2；[速度任务配置目录](https://github.com/unitreerobotics/unitree_rl_mjlab/tree/1425b15f73bd4095f0df53709d7c389c3eb9e790/src/tasks/velocity/config) 没有 B2-W。官方仓库名是 `unitree_rl_mjlab`。 |

官方 MuJoCo 模型核查版本为 `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`，其中存在 B2-W 模型，未找到 `policy_force` 路径。[固定版本模型](https://github.com/unitreerobotics/unitree_mujoco/tree/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/unitree_robots/b2w)

## 当前 ONNX 的来源与缺口

当前权重文件来自 ETH/Legged Robotics 的 [sru-robot-deployment 策略目录](https://github.com/leggedrobotics/sru-robot-deployment/tree/main/b2w_sim/b2w_controllers/src/policy)，该目录公开了 `policy_force_new.onnx` 和两个导出脚本；它不是宇树官方策略目录。导出脚本引用 `policy_blind_b2w.pt`、使用 60 维输入，但该目录没有对应 `.pt` 文件或训练环境配置。[导出脚本](https://github.com/leggedrobotics/sru-robot-deployment/blob/main/b2w_sim/b2w_controllers/src/policy/pt_to_onnx.py)

来源不只依据文件同名：本地 `git hash-object` 得到 Git blob SHA `911cab97bf0e343004b6c4621e9d68167b6f06dc`，与上游 GitHub 文件 API 返回的 SHA 完全一致；上游文件大小为 1,999,962 字节。[上游 blob 元数据](https://api.github.com/repos/leggedrobotics/sru-robot-deployment/git/blobs/911cab97bf0e343004b6c4621e9d68167b6f06dc)

本地部署的 60 维观测、256 维循环状态、16 维动作以及腿/轮动作缩放 0.5 / 5，只能说明推理接口目前如何运行；不能证明它与原训练中的质量、摩擦、关节约定、观测归一化、轮角处理和 PD 增益一致。仍须取得原始训练配置，或建立一套明确配置、重新训练并验证的 B2-W 策略。

本地模型与本次下载的官方 `b2w.xml` 比较：12 个腿部关节的轴与角度限位一致，但本地 URDF 各 link 质量总和为 82.529857 kg，官方 MJCF 各 inertial 质量总和为 87.170292 kg。官方 actuator 数组按 FR、FL、RR、RL 排列每腿的 hip/thigh/calf，再按 FR、FL、RR、RL 排列四个 wheel；本地策略接口按关节类型分组，组内为 FL、FR、RL、RR。数组顺序不同需要按关节名称映射，并不直接说明当前映射错误；质量差异也不能直接证明它就是脚部交叉的原因。**没有权重训练配置时，不应简单把策略输入改为官方 actuator 顺序，或把本地质量改为官方 MJCF 数值。**[官方模型源码](https://github.com/unitreerobotics/unitree_mujoco/blob/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/unitree_robots/b2w/b2w.xml)

对当前脚部交叉的修复，官方资料最适合先用于核对模型与建立独立站立验证，再决定是否保留现有未知训练配置的策略；官方 Sport 客户端用于实机速度接口，不能替代本机仿真中的完整步态实现。
