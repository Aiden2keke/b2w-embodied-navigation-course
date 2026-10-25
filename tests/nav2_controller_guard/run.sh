#!/usr/bin/env bash
# Reproduce RED and GREEN with a disposable test build, without ROS nodes.
set -eo pipefail
course_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
workspace="$course_root/.deps/nav2-controller-1.1.20"
result_dir="${1:-$(mktemp -d /tmp/b2w-nav2-guard-test.XXXXXX)}"
mkdir -p "$result_dir"
result_dir="$(cd "$result_dir" && pwd)"
source /opt/ros/humble/setup.bash
mkdir -p "$result_dir/original"
tar -xzf "$workspace/nav2_controller-1.1.20-1.tar.gz" --strip-components=1 -C "$result_dir/original"
for phase in red green; do
  controller_source="$result_dir/original/src/controller_server.cpp"
  if [[ "$phase" == green ]]; then controller_source="$workspace/src/nav2_controller/src/controller_server.cpp"; fi
  cmake -S "$course_root/tests/nav2_controller_guard" -B "$result_dir/$phase" \
    -DCONTROLLER_SOURCE="$controller_source" -DCMAKE_BUILD_TYPE=Debug \
    -DPython3_EXECUTABLE=/usr/bin/python3 > "$result_dir/$phase-build.log" 2>&1
  cmake --build "$result_dir/$phase" -j2 >> "$result_dir/$phase-build.log" 2>&1
  status=0
  "$result_dir/$phase/regression" > "$result_dir/$phase.log" 2>&1 || status=$?
  cat "$result_dir/$phase.log"
  if [[ "$phase" == red ]]; then
    [[ "$status" == 1 ]] && rg -q '^Failures: 3$' "$result_dir/$phase.log"
  else
    [[ "$status" == 0 ]]
  fi
done
printf 'RED/GREEN evidence: %s\n' "$result_dir"
