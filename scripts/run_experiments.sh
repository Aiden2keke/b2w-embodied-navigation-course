#!/usr/bin/env bash
set -eo pipefail
course_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$course_root/install/setup.bash"
export PATH="/usr/bin:/bin:/opt/ros/humble/bin:$PATH"
exec /usr/bin/python3 "$course_root/scripts/run_experiments.py" "$@"
