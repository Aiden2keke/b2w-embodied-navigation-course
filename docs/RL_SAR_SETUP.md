# B2-W 开源运控本机适配

采用 [fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar) 的 `b2w/robot_lab` 策略，固定提交 `376d42c9b128f963ab08579762d5a216a976ce39`。这是社区预训练策略。原来的 60 维循环网络与它的 57 维前馈网络接口不同，因此提供独立控制器，而不是只换一个 ONNX 文件。

## 本机启动

当前下载目录是 `third_party/b2w_rl_sar`，权重、配置、配套机器人资产和派生 ONNX 已准备好。运行：

```bash
scripts/run_course.sh mode:=sim locomotion_backend:=rl_sar gui:=true rviz:=true
```

课程入口默认选择 `rl_sar`。`locomotion_backend:=legacy` 选择原控制器；`pretrained_directory:=/absolute/path` 可以更改模型目录。GUI 与 RViz 使用相同机器人描述。

## 在新机器上复现下载

下载脚本取得固定版本源码、权重和 B2-W 资产，并验证权重 SHA256、机器人资产 Git 对象摘要。首次转换使用独立的 CPU 环境，运行控制器不需要 PyTorch：

```bash
/usr/bin/python3 -m venv .deps/rl-sar-export
.deps/rl-sar-export/bin/pip install torch==2.5.1 --index-url https://download.pytorch.org/whl/cpu
.deps/rl-sar-export/bin/pip install onnx==1.16.2 onnxruntime==1.18.1 numpy==1.26.4
/usr/bin/python3 scripts/prepare_rl_sar.py --export-python "$PWD/.deps/rl-sar-export/bin/python"
scripts/build_course.sh --symlink-install --packages-select b2w_controllers course_sim course_bringup
```

转换脚本执行 TorchScript/ONNX 数值对照后才安装 ONNX，`POLICY_SOURCE.json` 保存提交、文件摘要、转换版本和误差。已有目录或摘要异常时会停止，避免覆盖其他模型。

## 实际控制链路

`Nav2/手柄 → /path_manager/path_manager_ros/nav_vel → b2w_rl_sar_controller → 16 个关节力矩 → Fortress`。

策略按仿真时间 50 Hz 推理，PD 力矩按 200 Hz 计算。输入按配置排列：机身角速度、投影重力、速度指令、相对关节角、关节速度、上一动作。关节按名称映射为 FR、FL、RR、RL 的腿关节，随后四个轮关节。轮位置观测置零。腿动作转换成位置目标，轮动作转换成角速度目标，然后统一计算并限幅力矩。

低层控制读取 `/sim/raw/joint_states` 和 `/sim/raw/imu`，队列只保留最新反馈。公开 `/joint_states` 仍按课程接口发布。SLAM 使用公开的加扰传感器，导航使用 SLAM 里程计；底层私有反馈没有接到导航定位。

配套 URDF 的同名惯量和关节动力学用于生成模型，其中 `trunk` 映射到课程 `base_link`。课程的轮胎简化碰撞形状、相机和碰撞板仍保留，不能声称动力学与训练仿真或实机完全一致。

`course_sim/config/rl_sar.yaml` 中的 `idle_hold` 和 `yaw_command_gain` 是本机适配参数，超出原始部署配置。停车保持继续使用策略控制腿，只将轮速目标设为零；转向系数改变送入策略的 yaw 指令。它们需要物理验收，不能以输入/输出单测替代。

## 验证入口

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 scripts/check_rl_sar_runtime.py
/usr/bin/python3 scripts/check_locomotion_physics.py --output course_results/rl-sar-physics.json
/usr/bin/python3 scripts/run_experiments.py --locomotion-backend rl_sar --profiles normal --seeds 0 --goal 1 0
```

时序测试检查实际 C++ 与 ONNX 的名称重排、观测缩放、暂停和超时处理。物理检查分别测量 15 秒站立、8 秒前进、10 秒停车、8 秒转向及 10 秒停车，失败退出码为 1，结果与日志保留。实验清单和 rosbag 元数据记录后端、策略来源及权重摘要。平地短测不能证明崎岖地形或实机可靠性。

当前验收状态和失败记录见 [LOCOMOTION_STATUS.md](LOCOMOTION_STATUS.md)；候选来源见 [B2W_PRETRAINED_CANDIDATES.md](B2W_PRETRAINED_CANDIDATES.md)。
