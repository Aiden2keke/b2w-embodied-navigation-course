# B2-W 仿真与导航：Codex 使用指南

本文件是其他用户及其 Codex 的统一操作入口。以下命令从仓库根目录执行；使用相对路径，不依赖原开发机的用户名或目录。

## 1. 选择环境和任务

- 当前课程分支：`zsx-dev`；运行环境：Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress、系统 Python 3.10。
- 默认链路：Gazebo 传感器 → LIO-SAM → Nav2 → `rl_sar` 运控 → 16 个关节力矩。
- README 下半部分保留上游 Jazzy/Harmonic/SRU 说明；本指南对应 `course_*` 的 Humble/Fortress 入口。
- 新克隆从第 2 节准备；已配置的机器先执行第 3 节检查，再启动。保留已有模型和依赖目录，按脚本输出处理版本或摘要不匹配。
- 本指南覆盖仿真。实机部署、崎岖地形、长路线和多层地图需要额外适配与验证。

交互终端为 zsh 时，加载 ROS 使用 `source /opt/ros/humble/setup.zsh`，构建完成后再加载 `source install/setup.zsh`。`setup.bash` 使用 Bash 的 `BASH_SOURCE` 定位自身；在 zsh 中 source 它会把当前目录误当成 prefix，并尝试读取当前目录的 `setup.sh`。源码根目录本来没有这些 setup 文件；workspace 的 setup 文件由 colcon 生成在 `install/` 中。不要为这个错误修改 apt 管理的 `/opt/ros/humble/setup.*`。本指南的 Bash 命令块在 Bash 中执行；带 Bash shebang 的项目脚本从 zsh 直接执行即可，不要 source 它们。

## 2. 新克隆的完整准备流程

### 2.1 基础环境

先确认已安装 ROS 2 Humble，并配置其 apt 软件源。纯 Ubuntu 安装不是 `setup_course_humble.sh` 的覆盖范围；缺少 `/opt/ros/humble/setup.bash` 时先完成 ROS 安装。

```bash
git clone --branch zsx-dev https://github.com/Aiden2keke/b2w-embodied-navigation-course.git
cd b2w-embodied-navigation-course
test -f /opt/ros/humble/setup.bash
/usr/bin/python3 --version
```

完成条件：ROS 环境文件存在，系统 Python 为 3.10。退出 Conda 环境，使用 `/usr/bin/python3`。图形运行需要可用的显示服务器及支持传感器渲染的显卡驱动。

### 2.2 安装依赖并准备 SLAM

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git libboost-all-dev \
  python3-colcon-common-extensions python3-pytest ros-humble-rviz2 ros-humble-joy
scripts/setup_course_humble.sh --install
scripts/fetch_lio_sam.sh
```

安装脚本准备 ros_gz、Nav2、PCL、Eigen 和独立 `.venv`。获取脚本下载固定版本 LIO-SAM/GTSAM、应用保存的 SLAM 修复，并将 GTSAM 构建到 `.deps`；首次编译较慢。

完成条件：两条项目脚本均退出 0，`third_party/lio_sam`、`third_party/gtsam` 和 `.deps/lib` 存在。网络下载失败时保留错误与现有目录，先解决网络或版本问题。

### 2.3 下载默认运控模型并转换 ONNX

```bash
/usr/bin/python3 -m venv .deps/rl-sar-export
.deps/rl-sar-export/bin/pip install torch==2.5.1 --index-url https://download.pytorch.org/whl/cpu
.deps/rl-sar-export/bin/pip install onnx==1.16.2 onnxruntime==1.18.1 numpy==1.26.4
/usr/bin/python3 scripts/prepare_rl_sar.py \
  --export-python "$PWD/.deps/rl-sar-export/bin/python"
```

该脚本下载 `fan-ziqi/rl_sar` 的 B2-W `robot_lab` 策略及配套资产，验证固定摘要，然后对 TorchScript 和派生 ONNX 做数值对照。PyTorch 只用于转换；运行 C++ 运控使用仓库内的 ONNX Runtime。

完成条件：脚本退出 0，以下文件存在，`POLICY_SOURCE.json` 包含 ONNX 摘要与转换误差：

```text
third_party/b2w_rl_sar/policy/b2w/robot_lab/policy.pt
third_party/b2w_rl_sar/policy/b2w/robot_lab/policy.onnx
third_party/b2w_rl_sar/robot_assets/b2w_description/urdf/b2w_description.urdf
third_party/b2w_rl_sar/POLICY_SOURCE.json
```

仅执行 `prepare_rl_sar.py` 而不指定转换解释器，在新克隆中会下载成功后提示仍需导出 ONNX；这不是可直接运行的完成状态。已有完整 bundle 时可用 `/usr/bin/python3 scripts/prepare_rl_sar.py` 校验并复用。

### 2.4 准备仓库场景并构建

```bash
/usr/bin/python3 scripts/prepare_warehouse.py
scripts/build_course.sh --symlink-install
scripts/setup_course_humble.sh --json
scripts/test_course.sh
```

场景脚本下载固定版本 AWS 小型仓库资产，并生成 Fortress world。默认有限空间场景已在 Git 中；只使用该场景时可省略仓库资产下载。

完成条件：构建退出 0，环境检查报告 `ready: true`，测试全部通过。基础 ROS 环境或依赖缺失时按实际检查结果补齐；不要仅凭文档中的历史测试结果判定新机器安装成功。

### 2.5 哪些文件随 Git 分发

| 内容 | 分发方式 |
|---|---|
| 课程源码、配置、脚本、教程、验证 JSON/截图 | Git 中已包含 |
| 原始机器狗 URDF/网格、旧运控权重、SRU 权重、C++ ONNX Runtime | Git 中已包含 |
| 当前默认 `rl_sar` 权重、派生 ONNX、配套 B2-W 资产 | 第 2.3 节下载/转换；位于被忽略的 `third_party/b2w_rl_sar/` |
| AWS 仓库网格/纹理/world | `prepare_warehouse.py` 下载/生成；被忽略 |
| LIO-SAM、GTSAM | `fetch_lio_sam.sh` 下载/构建；被忽略 |
| `.venv`、`.deps`、`build`、`install`、bag、大型实验数据 | 本机生成；被忽略 |

克隆完成不等于模型、运行依赖和构建产物已齐全。Git 内的旧运控权重不用于替代默认 `rl_sar`，两者观测和动作接口不同。

## 3. 已配置机器的检查与启动

```bash
scripts/setup_course_humble.sh --json
/usr/bin/python3 scripts/prepare_rl_sar.py

# 完整仓库导航：Gazebo + RViz + LIO-SAM + Nav2
scripts/run_warehouse.sh
```

可选 SmacPlanner2D + Humble RPP + 几何自适应速度的一次完整路线测试见
[ADAPTIVE_SPEED.md](docs/ADAPTIVE_SPEED.md)。必须显式提供实测走廊宽度
`w_ref`；默认仍为原 DWB。该功能只完成离线测试，实际速度由用户路线测试记录。

`run_warehouse.sh` 会在首次使用时准备仓库资产。选择其他模式时只运行所需的一条：

```bash
# 有限空间，仅仿真与运控
scripts/run_course.sh mode:=sim gui:=true rviz:=true
# 有限空间，建图
scripts/run_course.sh mode:=slam gui:=true rviz:=true
# 有限空间，完整导航
scripts/run_course.sh mode:=navigation gui:=true rviz:=true
# 仓库，仅仿真
scripts/run_warehouse.sh mode:=sim
```

完成条件：Gazebo 中机器人站立，RViz 中可见几何模型；SLAM 在约 20 秒启动等待后产生定位与地图，Nav2 在定位就绪后激活。实际就绪时间还受机器性能影响。

同一 ROS 域只运行一套课程仿真。重新初始化先在原启动终端 `Ctrl+C`，等待退出，再启动。重复启动会被进程锁阻止；关闭 Gazebo、bridge 或运控会触发整套 launch 退出。直接重置 world 的 SLAM 恢复尚未验证，使用完整重启。

## 4. 导航目标与其他终端的环境

额外终端先进入同一仓库根目录。下面是 Bash 的加载顺序；zsh 使用后面的对应脚本：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/course_sim/config/fastdds_local.xml"
export PYTHONPATH="$PWD/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$PWD/.deps/lib:$PWD/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
```

zsh 终端使用相同的环境变量配置，只将两个 source 命令换成以下顺序：

```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
```

入口默认使用域 91、本机通信；若主启动终端显式覆盖了域或通信设置，其他终端同步采用该设置和匹配的 DDS profile。

在 RViz 使用 **2D Goal Pose**，向 `/goal_pose` 发布目标；选择已观测的可达空闲区域。命令行示例：

```bash
ros2 topic pub --once --qos-reliability reliable /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}'
ros2 service call /course_navigation/cancel_goal std_srvs/srv/Trigger '{}'
```

目标坐标属于 SLAM `map`，不是 Gazebo world。navigation 模式使用唯一 Nav2 速度来源；手柄/SRU 的其他模式与录制流程见 [COURSE_SETUP.md](docs/COURSE_SETUP.md)。

## 5. 验证与结果

已完成第 4 节环境配置后，在静止的运行会话中做只读检查，期间不发送目标：

```bash
/usr/bin/python3 scripts/check_initialization.py --domain 91 --duration 20 \
  --output /tmp/b2w-init-check.json
/usr/bin/python3 scripts/check_rviz_alignment.py --output /tmp/b2w-alignment-check.json
/usr/bin/python3 scripts/smoke_course.py --duration 20 --output /tmp/b2w-smoke.json
```

完成条件：各检查退出 0，初始化 JSON 的 `passed` 为 true；存在唯一 clock/运控/机器人描述，时间与 TF 单调，实际机身稳定。sim 模式没有 SLAM；包含定位和地图的检查使用 slam/navigation 模式。

下面两项会启动独立测试会话，不是在当前 GUI 中发动作；第二项会实际移动测试机器人：

```bash
/usr/bin/python3 scripts/check_rl_sar_runtime.py --output /tmp/b2w-controller-check.json
/usr/bin/python3 scripts/check_locomotion_physics.py --scene warehouse \
  --output course_results/rl-sar-physics.json
```

物理检查覆盖站立、前进、停车、转向和再次停车，退出 0 且 `success: true` 才是通过。运控时序检查固定使用域 88，执行前确认该域没有其他仿真；物理检查默认使用域 87，可用 `--ros-domain-id` 调整。新结果写 `/tmp` 或 `course_results`，保留失败日志，不覆盖 Git 中的历史证据。

正常工况短目标实验：

```bash
scripts/run_experiments.sh --scene warehouse --profiles normal --seeds 0 --goal 1 -1
```

结果保存在 `course_results/`。录包、回放、ATE/RPE、地图保存和多扰动实验见 [COURSE_SETUP.md](docs/COURSE_SETUP.md)。固定地图仍需 SLAM 到旧地图的配准；未实现任意旧地图的自动全局重定位。

## 6. 运行故障的检查顺序

| 症状 | 优先检查 |
|---|---|
| 缺少 `policy.onnx` / 模型 bundle | 完成第 2.3 节；旧 ONNX 的维度和关节顺序不能替代 |
| 找不到 ROS 包或动态库 | 第 3 节环境检查；source 当前 install；检查 `.deps/lib` 和构建结果 |
| RViz 只有 TF，没有机器狗 | RobotModel 话题 `/b2w/robot_description`；Reliable + Transient Local；网格使用 `file://`；加载课程 RViz 配置 |
| TF 闪烁、时钟回退、重复会话提示 | 核对同域 clock/运控数量；正常结束原启动会话后重启；保留 PID 和日志定位残留 |
| 地面与模型错位 | slam/navigation 用 `map`，等待地面回波与初始地面拟合；sim/record 用 `base_link` 且不显示地面网格 |
| 无点云、SLAM 一直未就绪 | 检查显卡驱动和 DISPLAY/EGL；`gui:=false` 仍需要 GPU LiDAR 渲染；服务器 EGL 路径需要在目标机验证 |
| 目标不移动或规划失败 | `/slam/odom`、公共 TF、二维地图和 Nav2 lifecycle 就绪；目标在已观测空闲区；各终端域/DDS 设置一致 |
| 机器狗腿异常或站不稳 | 确认默认 `rl_sar`、对应资产和唯一运控；用物理检查区分真实运动异常与 TF 显示异常 |

## 7. Codex 修改代码时的边界

修改 topic、TF、消息类型或模块边界前，读取 [INTERFACE.md](docs/INTERFACE.md) 与 [INTERFACE_ALIGNMENT.md](docs/INTERFACE_ALIGNMENT.md)。传感器处理在 `course_sim`，定位在 `course_slam`，Nav2/目标/指标在 `course_navigation`，集成在 `course_bringup`，底层运控在 `b2w_sim/b2w_controllers`。

公共定位由 LiDAR/IMU SLAM 产生，Gazebo 真值只用于评价。雷达是项目选定的 XT32 参数仿真，型号与外参未经过实机测量确认；瞬时点云 `time=0`，自身回波过滤也有理想化假设。参数文件是代码的当前来源，硬件对应关系和限制见 [INTERFACE_ALIGNMENT.md](docs/INTERFACE_ALIGNMENT.md)。

运行或修改后的结果应区分“代码完成”“构建通过”“节点就绪”“实际物理通过”。已有平地短测、失败记录和验证边界见 [COURSE_VALIDATION.md](docs/COURSE_VALIDATION.md)；策略来源与控制参数解释见 [RL_SAR_SETUP.md](docs/RL_SAR_SETUP.md)。修改后按影响运行相应检查，提交源码、文档和必要的小型证据，保留 `.gitignore` 对下载依赖与大型结果的排除。

其他用户可直接给 Codex 这条指令：

> 先读取 AGENTS.md 和 CODEX_GUIDE.md，检查当前环境，按指南补齐模型与依赖，构建并启动仓库导航。报告实际检查结果；遇到下载或登录问题时给出具体缺失项。
