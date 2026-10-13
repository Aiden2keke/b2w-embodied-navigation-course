#!/usr/bin/env bash
set -o pipefail
course_main() {
    local course_root
    if ! course_root="$(dirname "${BASH_SOURCE[0]}")/.."; then
        printf 'FAIL: cannot locate course root\n' >&2
        return 1
    fi
    if ! cd "$course_root"; then
        printf 'FAIL: cannot cd to course root\n' >&2
        return 1
    fi
    course_root="$PWD"
    if ! source /opt/ros/humble/setup.bash; then
        printf 'FAIL: ROS environment source\n' >&2
        return 1
    fi
    if ! source "$course_root/install/setup.bash"; then
        printf 'FAIL: workspace environment source\n' >&2
        return 1
    fi
    export PYTHONPATH="$course_root/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
    export LD_LIBRARY_PATH="$course_root/.deps/lib:$course_root/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
    export OPENBLAS_NUM_THREADS=1
    exec /usr/bin/python3 "$course_root/scripts/run_narrow_baseline.py" "$@"
}
course_main "$@"
