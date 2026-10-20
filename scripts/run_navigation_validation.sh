#!/usr/bin/env bash
# Execute as bash (never source this file into zsh). No nodes or simulation.
set -o pipefail
task_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
course_underlay="${B2W_COURSE_UNDERLAY:-/home/zyd/program/b2w-embodied-navigation-course/install}"
experiment_underlay="${B2W_EXPERIMENT_UNDERLAY:-/tmp/b2w-resume-20261020T030926Z/install}"
source /opt/ros/humble/setup.bash || exit 2
source "$course_underlay/setup.bash" || exit 2
source "$experiment_underlay/setup.bash" || exit 2
export OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
cd "$task_root" || exit 2
exec /usr/bin/python3 scripts/navigation_validation.py "$@"
