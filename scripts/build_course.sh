#!/usr/bin/env bash
set -eo pipefail
course_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$course_root"
export PATH="/usr/bin:/bin:/opt/ros/humble/bin:$PATH"
export CMAKE_BUILD_PARALLEL_LEVEL=2
source /opt/ros/humble/setup.bash
if [[ -f .venv/bin/activate ]]; then source .venv/bin/activate; fi
if [[ -d .deps ]]; then
  export CMAKE_PREFIX_PATH="$course_root/.deps:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$course_root/.deps/lib:${LD_LIBRARY_PATH:-}"
fi
exec /usr/bin/python3 /usr/bin/colcon build --parallel-workers 2 "$@" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
