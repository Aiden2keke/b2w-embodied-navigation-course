# Saved-view stationary setup and recording (retired)

The automatic recording workflow documented below is retired. Do not use
`scripts/run_g1_g5_record.py --run`, `--prepare-only`, or `--record-only` for the
Warehouse presentation. The current user-operated command is documented in
[CODEX_GUIDE.md](../CODEX_GUIDE.md):

```bash
scripts/run_warehouse_navigation.py --run --review-config course_results/tracks/reviewed_run.json
```

It does not capture video, manipulate Gazebo cameras, repair WebM, or produce
MP4. The user records the screen manually. A unique `course_results/tracks/`
run directory contains `trajectory.png`, `trajectory.csv`, and `result.json`,
including for interrupted/failed executions when localization evidence exists.
The independent motion qualification and runtime safety gates remain mandatory.
At present, autonomous route execution is additionally fail-closed: the
reviewed `ComputePathToPose` result is not yet the path executed by Nav2's
separate `NavigateToPose` action. The command therefore cannot dispatch motion
until that binding or an equivalent continuous clearance veto is implemented
and reviewed. Planning and artifact generation remain available without
claiming route qualification.

The historical material below describes the former recording system only.

From the repository root:

```bash
DISPLAY=:1 python3 scripts/run_g1_g5_record.py --stationary-setup --stationary-seconds 60 --max-total-minutes 3
```

This starts the existing canonical Warehouse server, an owned GUI-only Gazebo
client, the real RViz configuration/map/LiDAR, and read-only waypoint markers.
There is no navigation server, route goal, qualification pulse or hardware.
It holds exact zero through the existing owner, initial/final measured rest,
contact/TF/physical checks and original150s watchdog. Viewing is bounded to60wall
seconds after initial qualified rest; normal rest and cleanup follow. The
stationary view report cannot be reused as controller-motion qualification.

Wait for **STATIONARY VIEWS READY** and the real map. Adjust RViz's TopDownOrtho
view, then use **File → Save Config As** and select:

```text
artifacts/video/manual_views/g1_g5.rviz
```

Ctrl+S alone saves the per-session working copy, not necessarily this file.
The current saved file is Orbit; a working copy starts in TopDownOrtho. Once a
TopDownOrtho view has been saved, its view parameters are reused without fitting
or changing its angle/scale. Goal-publishing tools are removed only from the
working copy during this zero-only setup. Original files are not automatically
written. The setup ends automatically; Ctrl+C is an owned safety abort, not a
route retry. Do not pause/reset physics while adjusting the RViz view.

## Exact Gazebo camera preservation

Required user inputs:

- `artifacts/video/manual_views/gazebo_gui.config`
- `artifacts/video/manual_views/gazebo_camera_pose_final.txt`
- `artifacts/video/manual_views/g1_g5.rviz`

The GUI config is copied byte-for-byte into the attempt and loaded using the
installed `ign gazebo -g --gui-config` interface. Its `<camera_pose>` still has an
older default. The runner first verifies the mapped Gazebo X11 window, the actual
Fortress pose publisher, and a pose received by its already-running listener.
Fortress `ign topic -i` reports publishers, not subscribers; receiving a real
pose is the subscription proof. It then sends the authoritative final pose text
VERBATIM to `/gui/move_to/pose`. The Boolean response means the request was
accepted, so completion is checked separately from fresh post-request feedback:
at least three consecutive samples matching within5e-5 and spanning at least
150ms. The installed CameraTracking plugin publishes the current camera pose at
50Hz when its camera is initialized and a subscriber is connected.
The saved world pose is never transformed, refitted, searched or rewritten.
The source camera config/pose SHA256 values are checked before/after. Stationary
setup and recording preserve window layouts; they do not move or resize the
saved Gazebo viewport. Only presentation/evaluation uses paired public TF and
truth history; truth never supplies navigation or public TF.

Server/client ownership: physics remains the original pinned `-s -r` server.
The owner creates a `-g` GUI-only client after the watchdog pins that server,
before waiting for SLAM. The private partition/token are shared; the unchanged
server-signature rule never accepts a GUI as a physics identity. RViz/markers/
recorders inherit the same owned token and are included in cleanup.

## Recording-only restore check

After saving RViz:

```bash
DISPLAY=:1 python3 scripts/run_g1_g5_record.py --record-only --max-total-minutes 3
```

`--prepare-only` is also a zero-only recording check. Each invocation uses a
fresh output/domain/lease,60s viewing and guarded rest. Both normal, stable,
exact-token-owned windows are captured at25FPS using the existing window-only
GStreamer recorder. Files live under:

```text
course_results/g1_g5/<UTC>-<id>/attempt_01/
  radar.webm, warehouse.webm
  prepare_radar.mp4, prepare_warehouse.mp4, prepare_combined.mp4
  manual-camera-restore.json, correspondence.json, framing.json
  attempt_report.json, owner/.../session-report.json
course_results/g1_g5/<UTC>-<id>/results_report.json
```

The H.264 MP4 exports are decoded and screened for black/empty frames and common
interval/duration. These are stationary CHECK videos, never route success videos.
No success files/report are fabricated on a recording failure. X11/XWayland is
supported through the intended window IDs; native Wayland without an X display
is rejected. Existing local FFmpeg/libx264 is reused; no system installation.

## Route mode remains blocked

The single route entry point is:

```bash
/usr/bin/python3 scripts/run_g1_g5_record.py --run --max-attempts 12 --max-total-minutes 120
```

The current controller has NOT qualified retained heading or the Profile-B route
envelope. This currently stops before simulator launch with zero attempts until the
existing independently reviewed deployment/envelope/control evidence, source
pins, G3–G5 headings, saved-view visibility review, and a separately reviewed
real recording-only check are supplied in
`course_results/g1_g5/reviewed_run.json` with its externally reviewed
`course_results/g1_g5/reviewed_run.json.sha256`.
The script never generates accepted empirical qualification or envelope values.
The experimental controller's route restriction is retained. No navigation or
batch is authorized merely by a successful stationary camera/recording check.

When qualified, the existing full route owner, admission, per-waypoint physical
verification, watchdog and bounded12-attempt/120-minute loop are used. Only an
explicit ordinary Nav2 abort with complete safe rest/cleanup can retry; repeated
same failure or any safety/integrity/recording/cleanup fault stops. Every run uses
fresh state and outputs, and exports `success_*` only after same-attempt physical
arrival and continuous-video checks. Original goal tolerances remain unchanged.

## Validation boundary for this change

119 focused tests cover the saved pose request/file preservation, saved TopDown
reuse, stationary/no-motion scope, server/client flags, time-aligned camera
correspondence, stable window selection, dual recorder gates, bounded retries,
real waypoint errors and existing owner guards. A real media fixture exports
and decodes25FPS H.264, explicitly not robot footage. An isolated unmapped-X11
fixture verifies BadWindow becomes a Python exception rather than killing the
wrapper. Manual stationary modes now bypass arrangement entirely.

The first actual stationary owner completed60s,1985 exact-zero commands, measured
rest and normal cleanup. A later authorized `--record-only` attempt found both
owned X11 windows but did not produce camera feedback. Its wrapper's one-message
read timed out after3s and the coordinator sent SIGINT to the owner. At wall
465887.279, sim time was0.7975s: truth, IMU, contacts and final six-axis zero
were fresh, but `/odom`, map, and both TF edges were absent, so no qualified
initial or final physical rest existed. The owner recorded `KeyboardInterrupt`,
could not publish its fault-stop zero after its ROS context was invalid, and the
independent watchdog correctly emergency-paused the pinned server with
`physical_stop_measured:false`. The final empty process list proves cleanup only.
The GUI log contains the CameraTracking plugin load but no camera-pose publisher
announcement; the attempt did not record transport discovery, so the more
specific camera-init versus connection delay remains unproven.

The offline correction uses the existing15s camera-service admission bound; it
does not wait up to the owner watchdog deadline. It requires the installed pose
publisher plus an actual listener message before one exact saved-pose request,
then stable target feedback after the request. Camera failures no longer SIGINT the zero-only owner; it is
allowed its existing guarded startup/view/rest/cleanup path, and the report only
marks physical rest qualified when measured rest, normal watchdog and empty
owned cleanup all pass. This is source/test evidence, not a live qualification.
No new Gazebo session was launched for this correction. **Real map/LiDAR,
restored-camera, and dual-view MP4 validation remain pending.** No route success
or controller qualification is claimed. No GitHub push was attempted.

After separate explicit live authorization, the exact one-shot validation command
is:

```bash
DISPLAY=:1 python3 scripts/run_g1_g5_record.py --record-only --max-total-minutes 3
```

It performs one zero-only recording check and never launches a route goal. The
admission record is `artifacts/video/camera_watchdog_one_shot_admission.json`;
its current decision is `NOT AUTHORIZED / NOT RUN` until that separate approval.

## Ten-result collection mode

```bash
DISPLAY=:1 python3 scripts/run_g1_g5_record.py --run --collect-attempts 10 --max-total-minutes 120
```

This mode requires the same independently reviewed controller, envelope,
deployment, source and saved-view admissions as ordinary `--run`, including a
hash-pinned real dual-video check that the runner decodes again. At present it
reports zero attempts and does not launch Gazebo. The output directory is
`course_results/g1_g5/G1_G5_batch_10/`; an existing directory causes a fail-closed
error until it is archived explicitly. `summary.json` and `summary.csv` report
actual attempted/successful counts and failure causes. Category directories
contain `attempt_NN/report.json`, linked MP4s where valid, and a `telemetry`
link to the untouched owner session. Ordinary Nav2 abort MP4s are labeled
`partial_video: true`; recording and safety faults stay in `blocked/` and stop
the batch. Multiple verified successes are retained; `summary.json` names the
fastest verified successful combined video, if any. Motion begins only after
both recorders are ready and existing route admission passes.
