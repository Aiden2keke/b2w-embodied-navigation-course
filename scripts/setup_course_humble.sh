#!/usr/bin/env bash
set -eo pipefail
course_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" != "--install" ]]; then
  echo 'Install with: scripts/setup_course_humble.sh --install'
  source /opt/ros/humble/setup.bash
  if [[ -f "$course_root/install/setup.bash" ]]; then source "$course_root/install/setup.bash"; fi
  export PYTHONPATH="$course_root/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
  export LD_LIBRARY_PATH="$course_root/.deps/lib:$course_root/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
  exec /usr/bin/python3 "$course_root/scripts/check_course_environment.py" "${@}"
fi
sudo apt-get install -y ros-humble-ros-gz ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-perception-pcl ros-humble-pcl-msgs ros-humble-xacro \
  python3.10-venv python3-scipy python3-yaml libeigen3-dev
/usr/bin/python3 -m venv --system-site-packages "$course_root/.venv"
"$course_root/.venv/bin/python" -m pip install -r "$course_root/requirements-course.txt"
echo 'Next: scripts/fetch_lio_sam.sh; prepare the B2-W bundle using docs/RL_SAR_SETUP.md; scripts/build_course.sh'
