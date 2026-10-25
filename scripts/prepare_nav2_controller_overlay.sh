#!/usr/bin/env bash
# Build only the pinned controller, never install dependencies or modify /opt.
set -eo pipefail
course_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="$course_root/.deps/nav2-controller-1.1.20"
archive="$workspace/nav2_controller-1.1.20-1.tar.gz"
expected=bac776b1b5e5cc1dd6365aec941d71d19e1a06fa5ad5d5ee84f299b05d0a6235
installed="$(dpkg-query -W -f='${Version}' ros-humble-nav2-controller)"
[[ "$installed" == 1.1.20-1jammy.20260915.232401 ]] || {
  echo "Unsupported installed controller: $installed; re-review compatibility." >&2; exit 1;
}
mkdir -p "$workspace"
if [[ $# == 1 ]]; then
  cp "$1" "$archive"
elif [[ $# != 0 ]]; then
  echo "Usage: $0 [pinned-release-archive]" >&2; exit 2
elif [[ ! -f "$archive" ]]; then
  curl -fL --max-time 120 \
    https://codeload.github.com/ros2-gbp/navigation2-release/tar.gz/refs/tags/release/humble/nav2_controller/1.1.20-1 \
    -o "$archive.download"
  mv "$archive.download" "$archive"
fi
printf '%s  %s\n' "$expected" "$archive" | sha256sum -c -
# Refuse to overwrite a source tree; a fresh workspace is required for re-prepare.
[[ ! -e "$workspace/src/nav2_controller" ]] || {
  echo "Source already prepared; rebuild with the documented colcon command." >&2; exit 1;
}
mkdir -p "$workspace/src/nav2_controller"
tar -xzf "$archive" --strip-components=1 -C "$workspace/src/nav2_controller"
diff -rq "$workspace/src/nav2_controller/include/nav2_controller" /opt/ros/humble/include/nav2_controller
cmp "$workspace/src/nav2_controller/package.xml" /opt/ros/humble/share/nav2_controller/package.xml
patch --batch --fuzz=0 -d "$workspace/src/nav2_controller" -p1 \
  < "$course_root/patches/nav2_controller_1.1.20/endpoint-transform.patch"
export CMAKE_BUILD_PARALLEL_LEVEL=2
export PATH="/usr/bin:/bin:/opt/ros/humble/bin:$PATH"
source /opt/ros/humble/setup.bash
cd "$workspace"
/usr/bin/python3 /usr/bin/colcon build --base-paths src --packages-select nav2_controller \
  --allow-overriding nav2_controller --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3
