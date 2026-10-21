#!/usr/bin/env bash
# No preparation/download step. Only local robot bundle and installed dependencies.
set -eo pipefail
pipe_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$pipe_root"
pipe_runtime="${B2W_RUNTIME_ROOT:-$pipe_root}"
pipe_install="${B2W_PIPE_INSTALL:-$pipe_root/install_pipe}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-92}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export IGN_PARTITION="${IGN_PARTITION:-b2w-pipe-$ROS_DOMAIN_ID-$$}"
export PATH="/usr/bin:/bin:/opt/ros/humble/bin:$PATH"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
export ROS_LOG_DIR="${ROS_LOG_DIR:-$pipe_root/course_results/pipe_logs}"
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/humble/setup.bash
source "$pipe_runtime/install/setup.bash"
if [[ -f "$pipe_install/local_setup.bash" ]]; then source "$pipe_install/local_setup.bash"; fi
export PYTHONPATH="$pipe_runtime/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$pipe_runtime/.deps/lib:$pipe_root/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
exec /usr/bin/python3 /opt/ros/humble/bin/ros2 launch course_sim pipe.launch.py \
  "pretrained_directory:=${B2W_PRETRAINED_DIRECTORY:-$pipe_runtime/third_party/b2w_rl_sar}" "$@"
