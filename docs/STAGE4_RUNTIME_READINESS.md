# Stage4 zero-only observation milestone — October 20

Current target is the uncommitted delta at HEAD a9991fc in
`/tmp/b2w-unattended-worktree-20261019`. Nonzero navigation remains NO-GO.
The earlier readiness bundle is preserved at
`/tmp/b2w-stage4-readiness-sol-20261020/APRIL_REVIEW_BUNDLE_Stage4_a9991fc_20261020.md`.
Its structural cadence, exact-grid caching, formal exit and pre-lock fixes remain.
This milestone removes circular motion-evidence prerequisites from observation.
No live stack was launched here; real ROS scheduling/TF/collision readiness is untested.

The version-2 `stage4_zero_session_inputs` manifest requires an absolute hash-bound
observer world, unchanged RL-SAR bundle, and `guard` with only `session_id`,
`collision_inventory_path`, `collision_inventory_sha256`. Inventory names are checked
against their hash-bound converted SDF. Existing reviewed controller and matching
copy-installed Python/launch files remain mandatory. No saved-grid registration,
stop/yaw/latency bound or motion qualification evidence is needed to observe zero.
Version 1 inputs are rejected. No dependency links, downloads or install fallback.

SessionOwner inserts `zero_only:true` and private owner/health/lease/inhibit paths.
A zero-only FinalCommandGuard cannot arm even with a fresh correctly bound lease:
any lease is a permanent fault. It always returns exact zero, independently of
physical bounds or motion qualification. The independent zero-only watchdog never
writes a lease. Nonzero requests on any of six axes latch; the sole final adapter
continues publishing zeros. Motion-mode bounds, contact/geometry/lease checks,
command expiry and the unconditional production motion interlocks remain intact.

Before arming, missing/stale localization or a TF lookup still awaiting delivery
emits zero with `waiting_localization` / `waiting_tf` health. Health is written even
when localization is absent; bootstrap requires `context_state:ready`, three prepared
live/local/global grids and measured structural cadence before starting the probe.
Malformed data, frame/quaternion/coherence errors, clock regression, owner loss,
publication deadline and contact failures remain permanent. Armed TF/localization
loss still latches. This addresses source-level startup failures; a live TF trace
is still needed before claiming a particular ROS startup is healthy.

Both adapter and probe independently bind zero-observation contact reporting at
the first received sequence. Their paired contiguous intervals can cover only
that interval onward. Missing pairs, gaps, sticky historical forbidden contacts,
unknown names and unclosed endpoints still fail. Motion streams still require an
explicit first sequence. Observed names/report continuity never prove non-foot
engine coverage or self-collision qualification.

The passive probe creates no goals/actions/velocity. It checks effective parameters,
active lifecycles, exclusive graph, fast data and structural cadence, unchanged
filled footprint/unknown blocking on live costmaps, stationary pose/velocity,
all observed command axes, measured recording capacity and paired contact closure.
It requires 30 simulation seconds and at least 5 wall seconds of stationary data;
transport cleanup additionally requires 5 source seconds and 3 wall seconds rest.
Bootstrap is capped at 90 wall seconds, probe working loop at 90 plus cleanup35,
supervisor working loop at220; owned cleanup escalates over10+5+3 seconds plus child
reaping. Ctrl+C inhibits and cleans only token/start-ticks-owned children. Retain
all failed evidence; do not automatically retry or extend into a long test.

## Copy-ready bounded user-local command

Run only after external review, on this prepared workstation, from Bash. This is
one zero-motion trial, not physical safety qualification. It uses the copy build
and real observation-only inputs prepared by this milestone. Source or artifact
changes cause validation rejection; rebuild matching copies before another review.
The observer world already exists and carries the contact reporter. Rendering
still requires the workstation's configured DISPLAY/EGL path per CODEX_GUIDE.
Do not start another session in domain197.

```bash
cd /tmp/b2w-unattended-worktree-20261019
source /opt/ros/humble/setup.bash &&
source /home/zyd/program/b2w-embodied-navigation-course/install/setup.bash &&
source /tmp/b2w-resume-20261020T030926Z/install/setup.bash &&
source /home/zyd/program/b2w-embodied-navigation-course/.deps/nav2-controller-1.1.20/install/setup.bash &&
source /tmp/b2w-zero-unblock-sol-20261020/copy-install/setup.bash &&
export ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 &&
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/course_sim/config/fastdds_local.xml" &&
export PYTHONPATH="/home/zyd/program/b2w-embodied-navigation-course/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}" &&
export LD_LIBRARY_PATH="/home/zyd/program/b2w-embodied-navigation-course/.deps/lib:$PWD/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:${LD_LIBRARY_PATH:-}" &&
stage4_zero="/tmp/b2w-zero-user-$(date -u +%Y%m%dT%H%M%SZ)" &&
/usr/bin/python3 scripts/stage4_session.py \
  --inputs /tmp/b2w-zero-unblock-sol-20261020/zero-inputs.json --output "$stage4_zero" &&
/usr/bin/python3 scripts/stage4_session.py \
  --inputs /tmp/b2w-zero-unblock-sol-20261020/zero-inputs.json --output "$stage4_zero" --execute-zero-smoke
```

The first invocation validates only and starts no processes. The second creates
an exclusive directory. Exit0 requires probe success and owned cleanup; inspect
`session-result.json`, `probe/zero-result.json`, journal and stack/watchdog logs.
All results retain motion_authorized:false, safety_complete:false and no route or
segment pass. Account actual live_stack_wall_s against the inherited campaign
budget (211.308248/21600 seconds already recorded, zero motion attempts before
this milestone); this command was NOT executed here. Longer tests stay user-run
and need separate scope after inspection of the bounded trial.

## Deterministic verification

Luna runs `bash scripts/run_navigation_validation.sh --run-offline --output NEW`
with the strict zero-boundary suite included; no skips or source drift allowed.
Evidence and exact counts are in this milestone's APRIL bundle. Copy build uses
existing underlays with colcon, packages course_navigation/course_bringup, isolated
build/install/log directories, no --symlink-install and no dependency install.
Full-repository tests and live ROS/Gazebo tests are not claimed green. Physical
stop/abort latency, sensor-only registration uncertainty, all-body coverage,
independent external review and real runtime timing remain required for motion.
