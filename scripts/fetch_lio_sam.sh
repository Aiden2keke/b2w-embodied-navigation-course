#!/usr/bin/env bash
set -euo pipefail
course_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$course_root/third_party"
fetch() {
  local name="$1" url="$2" revision="$3"
  local directory="$course_root/third_party/$name"
  if [[ ! -d "$directory" ]]; then
    git clone "$url" "$directory"
    git -C "$directory" checkout --detach "$revision"
  fi
  if [[ "$(git -C "$directory" rev-parse HEAD)" != "$revision" ]]; then
    echo "$name revision mismatch; preserve existing checkout and inspect manually" >&2
    exit 1
  fi
}
fetch lio_sam https://github.com/TixiaoShan/LIO-SAM.git 08af3f32f01725372d4269838dc44c19c6d9e76b
fetch gtsam https://github.com/borglab/gtsam.git 69a3a75195b65356d6e56669e5199d325c7962c9
patch="$course_root/scripts/patches/lio_sam-transform-fusion.patch"
if git -C "$course_root/third_party/lio_sam" apply --reverse --check "$patch" 2>/dev/null; then
  : # Already applied.
elif git -C "$course_root/third_party/lio_sam" apply --check "$patch"; then
  git -C "$course_root/third_party/lio_sam" apply "$patch"
else
  echo 'LIO-SAM local patch conflicts with checkout; preserve it and inspect manually' >&2
  exit 1
fi
# Keep GTSAM out of colcon discovery; it is a separate CMake dependency.
touch "$course_root/third_party/gtsam/COLCON_IGNORE"
cmake -S "$course_root/third_party/gtsam" -B "$course_root/build/gtsam" \
  -DCMAKE_INSTALL_PREFIX="$course_root/.deps" -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_USE_TBB=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
cmake --build "$course_root/build/gtsam" -j2
cmake --install "$course_root/build/gtsam"
