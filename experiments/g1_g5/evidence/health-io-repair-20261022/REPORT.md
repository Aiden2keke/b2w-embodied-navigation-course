# Stage4 adapter transient health I/O repair — 2026-10-22

**Offline repair/tests PASS; live ZeroOnly UNVERIFIED; NONZERO NO-GO.**
Worktree: `/home/zyd/program/b2w-embodied-navigation-course-stage4`.
Branch `experiment/unattended-g1-g5-20261019`, unchanged HEAD
`5178993636287a22bf7cd847038d4904828fd898`. No staging, commit, push, Gazebo,
ROS launch, goal, motion lease, nonzero command, dependency installation or upgrade.
Preserved all56 inherited uncommitted diagnostic/evidence files byte-for-byte;
all407 indexed original-checkout source files unchanged. Pipe untouched.

## Confirmed failure and causal limit

Read the actual prior raw/session/adapter.json, original journal and diagnostic
result/index. The first sticky health-stage fault has prepublish28.004ms,
published28.216ms, health write41.062ms; combined69.278ms wall /3.531ms thread CPU.
`runtime-failure.json` preserves the exact source timing. This establishes a
wall-dominant synchronous health write overrun, not a specific41ms syscall cause.
No Gazebo repeat was used to measure or claim a runtime fix.

## Writer/caller trace and preserved contracts

The original `write_control` serializes JSON (finite values,8192-byte cap), creates
an owned0600 temporary file in the target directory, writes/flushes/fsyncs it,
then replaces the visible path. It remains byte-identical. Its remaining production
callers are SessionOwner guard configuration and owner heartbeat, the session's
watchdog manifest, and watchdog motion leases. The offline grid benchmark also
uses it for synthetic leases; this task did not run that benchmark or mint leases.
Only velocity_adapter's Stage4 adapter heartbeat now uses `write_adapter_health`.

| Contract | Preserved behavior |
| --- | --- |
| Atomic visibility | Same-directory0600 temporary file, fully flushed and closed before atomic os.replace; reader sees complete old or new JSON. |
| Ownership/validation | Same current-UID file creation, finite JSON8192-byte cap; unchanged read_control requires regular owned file, denies group/world writable and symlink paths with O_NOFOLLOW. |
| Sequence/timestamp | Adapter increments sequence synchronously once per callback; wall timestamp remains callback START, never completion/restamped by another thread. No async queue, worker or backlog. |
| Freshness/process identity | Watchdog retains live PID/start-tick/UID/PGID checks; adapter health<=150ms, owner heartbeat<=250ms. SessionOwner retains<=150ms, owned token, coherent context, prepared grids/cadence and fault checks. |
| Duplicates | Watchdog accepts identical duplicate reads only within ORIGINAL timestamp age; changed same-sequence data, sequence/time regression or stale/missing health inhibits. No duplicate freshness renewal. |
| Deadline/fault | Both original50ms prepublish/total-health checks unchanged. Slow replacement still trips health deadline; failed write latches adapter_health_write; inhibit remains first-fault sticky. |
| ZeroOnly | Guard remains immutable zero-only; watchdog never creates a motion lease; all fault tests emit zero and remain unarmed. Hard nonzero interlocks untouched. |
| Shutdown | Existing guard inhibit/final zero/owned grid-worker join behavior unchanged. Tests verify join even if final publisher raises; historical rclpy shutdown exception is not repaired. |

**Intentional storage distinction:** transient adapter health no longer requests
file fsync. It is not a crash-recovery record. A lost/unpublished write leaves an
old or missing heartbeat, which fails freshness; process death fails identity;
configuration/owner/lease writes retain their existing fsync semantics. A reboot
cannot qualify an old record under live identities and monotonic freshness.
The former general writer never fsynced the parent directory; no stronger crash
persistence promise is invented here. General control persistence is not weakened.
Temporary cleanup in the new writer covers write/flush/rename exceptions.

## Short staged-I/O probe and recorder contribution

Command (same disk-backed persistent evidence filesystem):
`/usr/bin/python3 scripts/benchmark_stage4_health_io.py --output experiments/g1_g5/evidence/health-io-repair-20261022/raw/io-probe --payload experiments/g1_g5/evidence/zeroonly-diagnostic-20261021/raw/session/adapter.json --samples 60`

One bounded ABBA invocation (recorder off/on/on/off),60 paired durable/transient
samples per phase, alternating write order.480 writes total, roughly5s of sampled
phases. Separate wall/thread-CPU clocks for JSON, temporary-file creation, write,
flush, fsync, close and rename; all samples retained in raw/io-probe/results.json.

| Stage (durable samples) | median wall ms | max wall ms |
| --- | ---: | ---: |
| JSON |0.072|0.495|
| temporary file |0.131|0.913|
| write |0.021|0.202|
| flush |0.020|0.144|
| fsync |6.176|7.367|
| close |0.027|0.135|
| rename |0.105|0.446|

Fsync median thread CPU0.074ms versus6.176ms wall. This supports removing the
unneeded durability wait from transient health; it does not prove historical fsync
alone consumed41.06ms. Median total staged durable6.657ms versus transient0.430ms;
maxima8.832ms versus3.243ms. No50ms probe sample occurred; the live stall was not
reproduced under this lighter offline load.

| Recorder surrogate | fsync writer median/max ms | transient median/max ms |
| --- | --- | --- |
| off |6.780 /8.832|0.530 /2.392|
| on |6.532 /8.177|0.370 /3.243|

The surrogate implements the original recorder's0.1s /proc environment/stat scans,
control JSON reads and approximate buffered140KiB/s journal traffic. Its two owned
processes consumed0.155s and0.123s CPU over1.206s each;10 scans each, maxima21.89ms
and16.31ms. Measurable CPU cost is established, but no consistent positive writer
latency effect in these phases. ROS subscription deserialization, physics, original
first-seen executable hashing and full live process load were NOT reproduced.
Thus full passive-recorder contribution remains unresolved, not declared harmless.
Per-phase producer context-switch deltas are recorded; no per-syscall scheduler or
kernel off-CPU attribution is claimed. No threads/processes were pinned or tuned.

## Actual API/callback before versus after

`raw/measure_actual_api.py` ran60 writes per real API and30 real adapter fixture
callbacks per variant on the same filesystem. The before callback binds the new
call site to unchanged write_control; after uses write_adapter_health. TF/odometry
are fixtures, no ROS node or publisher context. Each API/callback sample includes
wall and thread CPU, with20ms pacing after samples. All captured commands zero,
no guard failure in either baseline, no leases.

| Measurement | before median/max wall ms | after median/max wall ms |
| --- | --- | --- |
| Actual health API |7.158 /15.313|0.772 /2.985|
| Callback (includes fixture state handling) |7.920 /11.084|1.666 /3.936|

Median thread CPU: health API0.951→0.769ms; callback1.675→1.660ms. The improvement
removes measured blocking cost rather than widening a threshold. This is not a
hard latency bound and cannot be compared as equivalent to the live69.28ms callback.

## Regression/build validation

RED: new adapter test forbids fsync; existing callback fails with
adapter_health_write:OSError, as expected (raw/red.log). Then changed only the
heartbeat call site and added the dedicated writer. General write_control remains
unchanged, including a regression that confirms it still invokes fsync.

Final affected command: source the recorded candidate underlay, then
`/usr/bin/python3 -m pytest -q tests/test_stage4_health_io.py tests/test_stage4_startup_repair.py tests/test_stage4_zero_boundary.py tests/test_stage4_session.py tests/test_stage4_safety_guard.py --junitxml=experiments/g1_g5/evidence/health-io-repair-20261022/raw/final-targeted.xml`

**66 passed in2.10s**, including15 focused new cases. Covers65ms delayed replacement
and retained sticky first timing, failed write/flush/rename with old-record retention
and temporary cleanup, secure atomic visibility, oversized/nonfinite data, unchanged
fsync elsewhere, missing/stale/regressed/conflicting duplicate health, identical
read duplicate expiry, owner loss, zero/no-lease faults and joined owned shutdown.
Existing tests cover SessionOwner token/cadence/bootstrap checks and a reaped real
watchdog process. These tests provide no new physical response qualification.

One isolated pure-Python course_navigation regular-copy build PASS (colcon0.89s;
no dependency changes). Warnings retained: deliberate overlay, diagnostic prefix
lacks setup files, byte-compiling disabled. Prior reviewed artifacts not modified.
`copy-installed-identities.json` verifies61 current regular source/config/launch
copies and resolution of both changed modules to this new overlay. Diagnostic IMU
SHA remains3bd8109f580ceac90670f5d12e57267192731e2bfd2af46eb1740d08672e50f5.
`candidate-env.bash` layers the new local_setup on the previous pinned environment;
no auto-launch. Read-only v2 validation PASS, no output/session directory created.
It does not pin the backend: the unchanged backend was checked separately.

## Scope, cleanup and remaining uncertainty

Changed production files: only stage4_control.py (new transient function) and
velocity_adapter.py (two-line import/call substitution). New bounded benchmark and
fault tests; sole PLAN.md/NOW updated, previous NOW preserved as history. Exact patch
and source hashes retained. Watchdog/SessionOwner, LIO, navigation parameters,
geometry/maps/thresholds, interlocks and Pipe remain unchanged.

Final verification: original control definitions (reader, writer, identities,
inhibit, path validation and Watchdog) byte-identical;56 inherited records and407
shared files unchanged. Both owned recorder child PIDs absent; no B2W_NARROW_OWNER
process remains. All test children/workers joined. nvidia-smi shows unrelated desktop
processes only; none were killed. git diff --check PASS, staging empty, HEAD unchanged.
Raw/build data remain ignored and LOCAL ONLY, not remotely backed up. Compact new
and inherited records remain uncommitted. Hash indexes cover the new raw evidence.

File creation/flush/rename, unrelated owner fsync, guard/localization work and
scheduler delay may still stall. This fix removes the evidenced unnecessary storage
synchronization path, not all possible>50ms delays; unchanged deadlines will still
fail closed. Historical b0, global readiness, watchdog live behavior, whole-trial
stationarity and full-body physical coverage remain unresolved/unqualified.

**Next action: independent review of this repair bundle before commit/push or a
new bounded ZeroOnly trial.** No nonzero navigation readiness or authorization.
