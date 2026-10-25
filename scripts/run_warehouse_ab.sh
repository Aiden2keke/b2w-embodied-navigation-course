#!/usr/bin/env bash
set -eo pipefail
ab_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ab_root"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "$ab_root/install/setup.bash"
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$ab_root/artifacts/recovery/cli-ros-log"
export FASTRTPS_DEFAULT_PROFILES_FILE="$ab_root/course_sim/config/fastdds_local.xml"
export PYTHONPATH="$ab_root/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$ab_root/.deps/lib:$ab_root/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
exec /usr/bin/python3 "$ab_root/scripts/warehouse_ab_trial.py" "$@"
