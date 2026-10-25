#!/usr/bin/env bash
# Opt-in environment confined to this child process. Parent shell is unchanged.
set -eo pipefail
course_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
prefix="$course_root/.deps/nav2-controller-1.1.20/install/nav2_controller"
[[ -f "$course_root/.deps/nav2-controller-1.1.20/install/local_setup.bash" && -x "$prefix/lib/nav2_controller/controller_server" ]] || {
  echo "Build the pinned overlay first: scripts/prepare_nav2_controller_overlay.sh" >&2; exit 1;
}
[[ $# -gt 0 ]] || { echo "Usage: $0 command [args...]" >&2; exit 2; }
export PATH="/usr/bin:/bin:/opt/ros/humble/bin:$PATH"
source /opt/ros/humble/setup.bash
source "$course_root/install/setup.bash"
# Must be last: sourcing the standard course startup afterward can reorder prefixes.
source "$course_root/.deps/nav2-controller-1.1.20/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export IGN_PARTITION="${IGN_PARTITION:-b2w-course-$ROS_DOMAIN_ID-$$}"
export PYTHONPATH="$course_root/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
course_dds_profile=fastdds.xml
if [[ "$ROS_LOCALHOST_ONLY" == 1 ]]; then course_dds_profile=fastdds_local.xml; fi
export FASTRTPS_DEFAULT_PROFILES_FILE="$course_root/course_sim/config/$course_dds_profile"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
export LD_LIBRARY_PATH="$course_root/.deps/lib:$course_root/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
# Fail closed if a later environment change has displaced the opt-in package.
resolved="$(/usr/bin/python3 -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("nav2_controller"))')"
[[ "$resolved" == "$prefix" ]] || { echo "Overlay displaced: $resolved" >&2; exit 1; }
cd "$course_root"
exec "$@"
