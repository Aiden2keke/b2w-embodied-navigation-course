#!/usr/bin/env bash
set -eo pipefail
course_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$course_root"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export IGN_PARTITION="${IGN_PARTITION:-b2w-course-$ROS_DOMAIN_ID-$$}"
printf 'B2-W ROS_DOMAIN_ID=%s ROS_LOCALHOST_ONLY=%s\n' "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY"
export PATH="/usr/bin:/bin:/opt/ros/humble/bin:$PATH"
source /opt/ros/humble/setup.bash
source "$course_root/install/setup.bash"
export PYTHONPATH="$course_root/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
course_dds_profile=fastdds.xml
if [[ "$ROS_LOCALHOST_ONLY" == 1 ]]; then course_dds_profile=fastdds_local.xml; fi
export FASTRTPS_DEFAULT_PROFILES_FILE="$course_root/course_sim/config/$course_dds_profile"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=2
export LD_LIBRARY_PATH="$course_root/.deps/lib:$course_root/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
exec /usr/bin/python3 /opt/ros/humble/bin/ros2 launch course_bringup course.launch.py "$@"
