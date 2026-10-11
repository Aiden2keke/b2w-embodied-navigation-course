#!/usr/bin/env bash
set -eo pipefail
course_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$course_root/install/setup.bash"
export PATH="/usr/bin:/bin:/opt/ros/humble/bin:$PATH"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
course_dds_profile=fastdds.xml
if [[ "$ROS_LOCALHOST_ONLY" == 1 ]]; then course_dds_profile=fastdds_local.xml; fi
export FASTRTPS_DEFAULT_PROFILES_FILE="$course_root/course_sim/config/$course_dds_profile"
if [[ $# -ne 1 ]]; then echo 'Usage: scripts/save_course_map.sh NEW_DIRECTORY' >&2; exit 2; fi
mkdir "$1"
ros2 run nav2_map_server map_saver_cli -f "$1/map" --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
ros2 run course_slam export_map --output "$1/cloud.pcd"
