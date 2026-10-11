#!/usr/bin/env bash
set -eo pipefail
course_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$course_root"
/usr/bin/python3 scripts/prepare_warehouse.py
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
export IGN_PARTITION="${IGN_PARTITION:-b2w-warehouse-$ROS_DOMAIN_ID-$$}"
printf 'Warehouse ROS_DOMAIN_ID=%s; use the same domain for goal/recording terminals.\n' "$ROS_DOMAIN_ID"
exec "$course_root/scripts/run_course.sh" mode:=navigation scene:=warehouse gui:=true rviz:=true "$@"
