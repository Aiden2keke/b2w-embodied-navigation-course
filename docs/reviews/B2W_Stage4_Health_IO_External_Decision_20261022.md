# Externally communicated Stage4 Health I/O decision — 2026-10-22

Source: the user's external review decision communicated in this session.
This record transcribes that authorization; it is not an independently authored
review or agent self-approval.

Verdict: **CONDITIONAL PASS** for the Health I/O repair and **exactly ONE bounded
ZeroOnly Gazebo trial**. **NONZERO NO-GO**.
Reviewed APRIL bundle SHA256:
`e6996e405fc58fabb969de7bbfcc94c3c461817ce5a215a3d07d65250331fe8a`.
Previous checkpoint: `5178993636287a22bf7cd847038d4904828fd898`.
Repository: `/home/zyd/program/b2w-embodied-navigation-course-stage4`;
branch `experiment/unattended-g1-g5-20261019`.

Authorized: audit/stage only the reviewed source, tests and compact durable records,
including the prior uncommitted diagnostic result; one checkpoint commit and normal
fast-forward push only to this experiment branch; verify matching installed repair,
immutable inputs and separately pinned diagnostic LIO/GTSAM, zero-only guard,
hard motion interlock, idle domain197 and owned cleanup before one existing bounded
ZeroOnly trial. Preserve all records and raw evidence with SHA256 indexes.

No retry, patch during/after failure, nonzero velocity, navigation action, lease,
threshold change, safety bypass, Ground Truth navigation, upgrade, shared-branch
merge, force operation or Pipe work. Stop at first credible failure, clean only
owned processes, deliver one APRIL result bundle and checksum, and seek independent
review before further source changes or Start→G1. Missing evidence is NOT_OBSERVED.
The existing bootstrap/supervisor/watchdog/probe/cleanup limits are unchanged.
