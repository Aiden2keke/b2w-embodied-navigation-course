#!/usr/bin/env bash
# Opt-in environment confined to this child process. Parent shell is unchanged.
set -o pipefail
course_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
prefix="${B2W_CONTROLLER_PREFIX:-$course_root/.deps/nav2-controller-1.1.20/install/nav2_controller}"
controller_install="$(dirname "$prefix")"
course_underlay="${B2W_COURSE_UNDERLAY:-$course_root/install}"
[[ -f "$controller_install/local_setup.bash" && -x "$prefix/lib/nav2_controller/controller_server" ]] || {
  echo "Supply B2W_CONTROLLER_PREFIX for the verified copy install, or build the historical local overlay explicitly" >&2; exit 1;
}
[[ $# -gt 0 ]] || { echo "Usage: $0 command [args...]" >&2; exit 2; }
export PATH="/usr/bin:/bin:/opt/ros/humble/bin:$PATH"
source /opt/ros/humble/setup.bash || exit 2
source "$course_underlay/setup.bash" || exit 2
# Must be last: sourcing the standard course startup afterward can reorder prefixes.
source "$controller_install/local_setup.bash" || exit 2
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export IGN_PARTITION="${IGN_PARTITION:-b2w-course-$ROS_DOMAIN_ID-$$}"
if [[ -n "${B2W_PYTHON_SITE:-}" ]]; then export PYTHONPATH="$B2W_PYTHON_SITE:${PYTHONPATH:-}"; fi
course_dds_profile=fastdds.xml
if [[ "$ROS_LOCALHOST_ONLY" == 1 ]]; then course_dds_profile=fastdds_local.xml; fi
export FASTRTPS_DEFAULT_PROFILES_FILE="$course_root/course_sim/config/$course_dds_profile"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
if [[ -n "${B2W_NATIVE_LIB_DIR:-}" ]]; then export LD_LIBRARY_PATH="$B2W_NATIVE_LIB_DIR:${LD_LIBRARY_PATH:-}"; fi
export LD_LIBRARY_PATH="$course_root/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
# Fail closed if a later environment change has displaced the opt-in package.
resolved="$(/usr/bin/python3 -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("nav2_controller"))')"
[[ "$resolved" == "$prefix" ]] || { echo "Overlay displaced: $resolved" >&2; exit 1; }
cd "$course_root" || exit 2
exec "$@"
