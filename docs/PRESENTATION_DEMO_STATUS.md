# Presentation video status — blocked by active rolling steering

**No complete G1–G5 video or success MP4 was produced.** One safe rolling
diagnostic was executed; zero route attempts were started. The100-attempt
authorization remains conditional on legitimate route/tracking admission.

## One concrete blocker

During forward rolling, a **positive delivered yaw command produced negative
body yaw before stopping**. Continuous waypoint traversal cannot be safely
promoted on this steering interface. This rejection is not a heading-retention
qualification or centimeter-accuracy requirement.

The single reviewed host/domain230 run is
`course_results/imported_evidence/presentation-demo-20261028/rolling-live/precheck-d453aabd343e4e54b09c684336508e36`.

| Measured quantity | Actual |
| --- | ---: |
| Delivered vx,wz | +.05m/s, +.05rad/s |
| Policy input vx,wz | +.05, +.080000006 |
| Received final-command interval | .9925sim s |
| Sampled owner interval | 1.0300sim s (nominal1.0) |
| Body turn during delivered command | **−.146038rad** |
| Mean policy wheel right-minus-left target | −2.96753rad/s |
| Mean measured wheel differential | −.99498rad/s |
| Model-active gyro mean | approximately−.14759rad/s |
| Maximum translation / speed | .115555m / .119114m/s |
| Maximum absolute yaw rate | .211497rad/s |
| Received-zero→qualified-rest start | 1.9875sim s |

All physical limits remained intact. No contact/truth/readiness fault was
recorded. Six-axis zero, measured rest27.965…33.3225sim s, normal watchdog and
empty cleanup passed. All19 owned identities disappeared; domain230 lease was
independently reacquired. The attempt marker is consumed; do not repeat this
same point. This was a controlled diagnostic, not an autonomous route trial.

Independent Sol Medium review confirmed the delivered command, negative yaw,
gyro and wheel differential from raw data. A global yaw-sign flip is unjustified:
the earlier stronger pure-turn command turned positively. Blind minimum-yaw
rescaling is also unsupported. It would be unsafe to spend100 attempts hoping
this deterministic command-response problem disappears.

**Smallest next fix boundary:** a separate DEMO signed-steering mapping at the
command interface, preserving the original policy/controller, followed by the
same bounded rolling readmission. Existing recorded-state ONNX replay can reject
wrong neutral/sign mappings, but does not establish a safe live inverse. No such
correction is currently reviewed, so there is no honest ready-to-run full-route
command. Do not replace this blocker with looser waypoint tolerances.

Raw metrics and audit: `course_results/imported_evidence/presentation-demo-20261028/result.json`,
`rolling-analysis/yaw-analysis.json`, `rolling-cleanup.json`.

## Completed work toward the video

The repository's canonical task is
`course_navigation/config/warehouse_route.yaml`: five fixed map positions,
G1/G2 corridor headings, G3–G5 unspecified headings and a historical.05m
evaluation contract. It was not edited. No separate external assignment rubric
was supplied/found; the new presentation scope is represented separately.

Installed Humble Nav2's NavigateThroughPoses does follow a concatenated path
continuously. Its default BT removes intermediate goals at.7m and includes
spin/backup recovery; these are unsuitable as independent physical-passage
proof or as a rolling-only demonstration contract. Verified against the installed
tree and [Nav2 1.1.20 source](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml).

Separate **unadmitted draft** files preserve the canonical task:

- `course_navigation/config/nav2_presentation_demo.yaml`: Hybrid-A* Dubins,
  .8m minimum radius, RPP without rotate-to-heading/reverse, no waypoint pause,
  loose final yaw and.30m final XY tolerance. Original footprint/padding remain.
- `course_navigation/behavior_trees/presentation_through_poses.xml`: continuous
  ComputePathThroughPoses/FollowPath with no recovery and.20m removal radius.
  BT removal is not passage proof; independent paired truth/localization checks
  through all five.30m regions in order remain mandatory before claiming success.
- `course_navigation/config/velocity_adapter_presentation_demo.yaml`: final
  .10m/s/.20rad/s caps and original timeout. Must be explicitly consumed by a
  future reviewed demo launch; loading only the Nav2 YAML does not apply it.

These files are not deployment admission. No auto-batch or success-export entry
was enabled after the first tracking gate failed. An offline installed Hybrid-A*
probe on the retained old map found no full route; that stale-map result is not
a substitute for current registration/clearance. It does not justify moving.

## Real recording repair

The former `radar.webm` failure was a false rejection: FFmpeg fully decoded it,
but the null output's coarse clock rounded valid timestamps into duplicate DTS
ticks. `g1_g5_media.inspect_video` now gives the verification sink microsecond
timebase. Decoder errors remain strict (`-xerror`, `-err_detect explode`), and
complete progress/content checks remain. No source frames or timestamps were
edited, dropped or hidden.

Actual retained captures now fully verify:

| Source | Decoded frames | Duration |
| --- | ---: | ---: |
| radar.webm | 1962 | 78.465s |
| warehouse.webm | 1118 | 78.314s |

These are genuine existing stationary captures, **not route-success videos**.
The approved Gazebo view and RViz map/base_link TopDownOrtho scale remain
unchanged. No new camera framework was built.

Media regression: real retained radar fails on old code, passes after the sink
fix; genuinely truncated media still rejects. Media/capture tests42passed.
Rolling/admission tests144passed, including single combined motion, zero/rest,
immutable baseline identity and rejected command-scope mutation. Full affected
suite: **245 passed in12.43s**. Original
controller binary, pretrained policy, task definitions and safety thresholds
were preserved. No APRIL bundle, hardware motion or GitHub push.
