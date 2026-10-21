# Stage4 offline preparation / future reviewed measurement; no auto-launch.
source /opt/ros/humble/setup.bash
source /home/zyd/program/b2w-embodied-navigation-course/install/setup.bash
source /home/zyd/program/b2w-embodied-navigation-course/.deps/nav2-controller-1.1.20/install/setup.bash
source /home/zyd/program/b2w-embodied-navigation-course-stage4/experiments/g1_g5/evidence/startup-repair-20261021/artifacts/copy-install/setup.bash
export AMENT_PREFIX_PATH="/home/zyd/program/b2w-embodied-navigation-course-stage4/experiments/g1_g5/evidence/startup-repair-20261021/artifacts/lio-diagnostic-prefix:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="/home/zyd/program/b2w-embodied-navigation-course-stage4/experiments/g1_g5/evidence/startup-repair-20261021/artifacts/lio-diagnostic-prefix/lib:/home/zyd/program/b2w-embodied-navigation-course-stage4/experiments/g1_g5/evidence/startup-repair-20261021/artifacts/lio-build:/home/zyd/program/b2w-embodied-navigation-course/.deps/lib:/home/zyd/program/b2w-embodied-navigation-course-stage4/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="/home/zyd/program/b2w-embodied-navigation-course/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
export FASTRTPS_DEFAULT_PROFILES_FILE="/home/zyd/program/b2w-embodied-navigation-course-stage4/course_sim/config/fastdds_local.xml"
export B2W_STAGE4_LIO_DIAGNOSTICS=1 ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
