# 可下载的 B2-W 运控策略候选

核查日期：2026-10-10。这里的“策略”指预训练运控权重，不仅是机器人 URDF 或训练代码。

优先候选是 **fan-ziqi/rl_sar 的 B2-W / robot_lab 策略**。公开仓库实际包含 `policy.pt`、机器人专用部署配置和推理代码，无需登录或向作者申请权重。它是社区项目，不能称为宇树厂家的运动控制器。[项目与支持列表](https://github.com/fan-ziqi/rl_sar#rl_sar)

## 本机下载与验证

已下载到 `third_party/b2w_rl_sar/`：完整部署源码、`policy/b2w/robot_lab/policy.pt`、两份 YAML 配置，以及 `robot_assets/b2w_description/` 的 URDF、网格和 MuJoCo 场景。根目录 `POLICY_SOURCE.json` 保存固定版本和文件 SHA256；该目录设置 `COLCON_IGNORE`，不会混入课程 colcon 构建。下载无登录限制，不需要人工补传。

真实 TorchScript CPU 推理已通过，参考姿态下测试停止、前进、转向输入均得到有限的 16 维动作。已导出同目录 `policy.onnx` 并检查模型结构；32 组输入与 TorchScript 最大输出误差为 1.43e-6。此 ONNX 为本机格式转换产物，不是上游另行训练的权重。

临时 MuJoCo 3.2.7 无 GUI 平地检查使用配套 `scene.xml`、原始部署观测/控制参数：30 秒零指令、10 秒前进后 10 秒零指令、10 秒转向后 10 秒零指令均未触发摔倒判定（倾斜 >45° 或机身高度 <0.35 m）。但零指令 30 秒位移约 2.066 m，前进后的零指令 10 秒位移约 0.854 m；因此**没有通过稳定站立和停止验收**，不能仅以未摔倒宣布可靠。这份早期 MuJoCo 检查没有接入 ROS、雷达、SLAM、Nav2 或 Fortress，脚本是临时可行性探针。后续已完成独立 Fortress 控制器适配，平地运动与一次短目标导航通过，见 [RL_SAR_SETUP.md](RL_SAR_SETUP.md) 和 [LOCOMOTION_STATUS.md](LOCOMOTION_STATUS.md)。

结构化证据见 [validation/2026-10-10-pretrained-b2w.json](validation/2026-10-10-pretrained-b2w.json)。权重和部署参数可获取、控制器迁移及本机平地验证已经完成；训练仿真与本机的动力学仍存在差异，后续崎岖地形和实机需要独立验收。

## 已找到完整权重与部署接口

| 项目 | 已核查结果 |
| --- | --- |
| rl_sar 固定版本 | `376d42c9b128f963ab08579762d5a216a976ce39` |
| 权重 | [policy/b2w/robot_lab/policy.pt](https://github.com/fan-ziqi/rl_sar/blob/376d42c9b128f963ab08579762d5a216a976ce39/policy/b2w/robot_lab/policy.pt)，本地文件 796,064 字节；[直接下载](https://raw.githubusercontent.com/fan-ziqi/rl_sar/376d42c9b128f963ab08579762d5a216a976ce39/policy/b2w/robot_lab/policy.pt) |
| 部署配置 | [robot_lab/config.yaml](https://github.com/fan-ziqi/rl_sar/blob/376d42c9b128f963ab08579762d5a216a976ce39/policy/b2w/robot_lab/config.yaml)、[base.yaml](https://github.com/fan-ziqi/rl_sar/blob/376d42c9b128f963ab08579762d5a216a976ce39/policy/b2w/base.yaml) |
| 输入 / 输出 | 57 维输入、16 维输出；输入依次为身体角速度、投影重力、速度指令、相对关节位置、关节速度、上一动作；轮子位置观测置零。不是当前 60 维 LSTM 的替换文件。 |
| 关节顺序 | FR 三个腿关节、FL 三个、RR 三个、RL 三个，随后 FR/FL/RR/RL 四个轮关节；应按名称映射。 |
| 控制参数 | 腿参考角 `[0, 0.8, -1.5]`；动作缩放 hip=0.125、thigh/calf=0.25、wheel=5；腿 Kp=160、Kd=5；轮 Kp=0、Kd=1。控制步长 0.005 s、策略降采样 4，即策略 50 Hz。 |

输入构造及动作转换有[完整推理源码](https://github.com/fan-ziqi/rl_sar/blob/376d42c9b128f963ab08579762d5a216a976ce39/src/rl_sar/library/core/rl_sdk/rl_sdk.cpp)。配套执行器以位置和速度误差产生力矩：腿位置 PD，轮速度阻尼控制；迁移到 Fortress 时应匹配该控制方式，不能保留旧项目的强制轮速接口后就认为动力学一致。[Gazebo 控制器](https://github.com/fan-ziqi/rl_sar/blob/376d42c9b128f963ab08579762d5a216a976ce39/src/robot_joint_controller/ros2/src/robot_joint_controller_group.cpp)、[MuJoCo 控制器](https://github.com/fan-ziqi/rl_sar/blob/376d42c9b128f963ab08579762d5a216a976ce39/src/rl_sar/src/rl_sim_mujoco.cpp)

## 训练与机器人模型配套边界

作者同时提供 [robot_lab B2-W 训练环境](https://github.com/fan-ziqi/robot_lab/blob/500399ed75f510aeaff28705a8ce736c514dbec3/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled/unitree_b2w/rough_env_cfg.py) 和[执行器配置](https://github.com/fan-ziqi/robot_lab/blob/500399ed75f510aeaff28705a8ce736c514dbec3/source/robot_lab/robot_lab/assets/unitree.py)。当前训练源码与部署配置的关节顺序、观测屏蔽、动作缩放和 PD 参数吻合。但权重目录没有记录唯一训练运行及训练提交，因此不能宣称核实了权重的准确训练版本。

配套仿真描述来自 [rl_sar_zoo](https://github.com/fan-ziqi/rl_sar_zoo/tree/7dd30bdc7806898950b354260655d5a7f0ce844e/b2w_description)，版本 `1.0.2`，同时提供 URDF 与 MuJoCo 场景。robot_lab 的 B2-W URDF 总质量约 82.42 kg，身体约 35.61 kg；这是模型参数，不是实机称重结果。

rl_sar 声明支持 B2-W Gazebo 和 MuJoCo 仿真；其 Gazebo ROS 插件对应 Gazebo Classic，当前项目使用 Fortress，仍须适配和实际验证。支持列表的 B2-W 实机栏为空心圆，不能据此声称已经验证实机部署。[运行说明](https://github.com/fan-ziqi/rl_sar#simulation)

rl_sar 与 robot_lab 源码采用 Apache-2.0。[rl_sar 许可证](https://github.com/fan-ziqi/rl_sar/blob/376d42c9b128f963ab08579762d5a216a976ce39/LICENSE)、[robot_lab 许可证](https://github.com/fan-ziqi/robot_lab/blob/500399ed75f510aeaff28705a8ce736c514dbec3/LICENSE)。rl_sar_zoo 根目录未找到许可证文件，机器人资产的上游授权需要另行保留和核对；不能自动视为同一许可证。

## 其他候选：不能把文件缺失说成下载失败

| 项目 | 权重核查 | 判断 |
| --- | --- | --- |
| [Logic-TARS/ATEC2026](https://github.com/Logic-TARS/ATEC2026) | 核查提交 `b78c4afd1b84302fe8f88bcfd287eac64c33692c`：递归文件清单没有 `.pt/.pth/.onnx`，GitHub Releases 为空；[忽略配置](https://github.com/Logic-TARS/ATEC2026/blob/b78c4afd1b84302fe8f88bcfd287eac64c33692c/.gitignore) 排除权重。 | README 和旧说明出现策略文件名，实际公开仓库未附权重；机器人还搭载 Piper 机械臂，不能当作裸 B2-W 直接替换。 |
| [EmberLuo/wheel_quadruped_genesis](https://github.com/EmberLuo/wheel_quadruped_genesis) | 核查提交 `79a5ea6481321ab9de3c01ad21feb763d97a4953`：递归清单没有 `.pt/.pth/.pkl/.onnx`，Releases 为空。 | 有 B2-W/Go2-W 资产与训练、导出代码；说明中的 ckpt 8300 是本地运行示例，不是下载链接。 |
| [LauraMQuiros/b2w-rl](https://github.com/LauraMQuiros/b2w-rl) | 核查 `master` 提交 `ad82b971bf69a84170f027035c1e2e3bf97e4ef9`：递归文件清单只有学习曲线图片、配置与训练脚本，没有权重；Releases API 返回空列表。 | 可研究训练方案，不作为当前直接下载部署的首选。 |
| [leggedrobotics/sru-robot-deployment](https://github.com/leggedrobotics/sru-robot-deployment) | 公开 ONNX 已与当前项目权重核对相同。 | 是当前已有策略，不是解决其异常的新候选。 |

本次已有公开可下载的首选，不需要用户代下载。下载完成、推理接口验证和真实物理站立/行走验证是三个不同阶段；本文件记录候选证据，不代表运动稳定性已通过。
