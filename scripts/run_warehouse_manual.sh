#!/usr/bin/env bash
set -Ee

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
export DISPLAY="${DISPLAY:-:1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export IGN_PARTITION="${IGN_PARTITION:-b2w-warehouse-manual-${ROS_DOMAIN_ID}-$$}"
export PATH="/usr/bin:/bin:/opt/ros/humble/bin:$PATH"
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
export PYTHONPATH="$root/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$root/course_sim/config/fastdds_local.xml"
export LD_LIBRARY_PATH="$root/.deps/lib:$root/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
out="$root/course_results/tracks/$stamp"
mkdir -p "$out"
export ROS_LOG_DIR="$out/ros-logs"
mkdir -p "$ROS_LOG_DIR"
gui_config="$out/gazebo_gui.config"
/usr/bin/python3 "$root/scripts/prepare_warehouse_manual_view.py" "$gui_config"
export IGN_GUI_CONFIG_PATH="$gui_config"

stack_pid= rviz_pid= logger_pid= map_pid=
stop() {
  trap - INT TERM EXIT
  for pid in "$logger_pid" "$rviz_pid" "$map_pid" "$stack_pid"; do
    [[ -n "$pid" ]] && kill -INT "$pid" 2>/dev/null || true
  done
  for pid in "$logger_pid" "$rviz_pid" "$map_pid" "$stack_pid"; do
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
  done
}
trap stop INT TERM EXIT

echo "Manual warehouse visualization only: guarded Nav2 motion is unavailable; no goal can command the robot."
echo "Recording public /map and map->base_link poses to $out"
/usr/bin/python3 "$root/scripts/warehouse_manual_logger.py" --output-dir "$out" & logger_pid=$!
/usr/bin/python3 /opt/ros/humble/bin/ros2 run course_navigation map_projector --ros-args \
  -p use_sim_time:=true -p size:='[40.0, 40.0]' -p origin:='[-20.0, -20.0]' & map_pid=$!
/usr/bin/python3 /opt/ros/humble/bin/ros2 launch course_bringup course.launch.py \
  mode:=slam scene:=warehouse gui:=true rviz:=false control_source:=none & stack_pid=$!
sleep 3
rviz2 -d "$root/course_bringup/rviz/warehouse_manual.rviz" --ros-args -p use_sim_time:=true & rviz_pid=$!
wait "$stack_pid"
