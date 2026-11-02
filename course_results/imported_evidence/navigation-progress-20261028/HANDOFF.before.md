# Handoff — camera feedback corrected offline; next live check awaits approval

Repo `/tmp/b2w-navigation-ab-20261023`, branch
`navigation/g1-g5-recovery-20261025`. Read `docs/G1_G5_RECORDING.md` and the
sole `PLAN.md` NOW.

The 2026-10-27 attempt timed out on a one-message camera read; coordinator SIGINT
caused owner `KeyboardInterrupt`, then watchdog emergency containment. At sim
0.7975s `/odom`, map and TF were absent, so no measured rest existed. Process
cleanup is not physical rest. Camera plugin initialization/transport readiness
was not established in that attempt.

Offline correction verifies a viewable owned Gazebo window, actual typed pose
publisher and a received pose before applying the exact saved camera pose. It
requires stable fresh target feedback after the accepted request. Camera failure
no longer SIGINTs the zero-only owner. Focused tests and the affected suites are
recorded in PLAN NOW. No Gazebo was launched here.

User view files are hash-verified unchanged. Exact next one-shot command, only
after separate explicit authorization:

```bash
cd /tmp/b2w-navigation-ab-20261023
DISPLAY=:1 python3 scripts/run_g1_g5_record.py --record-only --max-total-minutes 3
```

Ten-route collection remains blocked: no accepted live dual-view recording,
retained heading, Profile-B directional turn/stop envelope, G1 swept-turn/map
admission, or matched reviewed deployment/source evidence. No route attempts,
videos, approval artifact or GitHub push were created here.
