# Watchdog clock ordering fix — independent review, 2026-10-23

## Result

The offline race fix is applied in this worktree and the patched package was rebuilt in a fresh isolated regular install. Focused regressions pass. No Gazebo trial, motion lease, nonzero command, goal, commit, or push was run. This change is ready for independent review; retain the existing stop before commit/push or another ZeroOnly trial.

## Historical first fault

The October 22 live watchdog failed with `adapter_publication_stalled`, but its zero-byte watchdog log and journals did not preserve the rejected health record or sampled monotonic wall. Therefore the exact historical failed subcondition remains unknown; the nearby 159.635 ms final-command receive gap is not the watchdog age operand and does not prove a genuine health stall. The fix addresses a separately demonstrated deterministic race, not a claim that the live failure was a false positive.

## Source change

In `course_navigation/course_navigation/stage4_control.py`, `Watchdog.step()` now reads the adapter health and owner heartbeat snapshots before taking its production monotonic sample. It validates those original record timestamps against the unchanged 150 ms and 250 ms limits. It still rejects future, stale, missing, nonfinite, invalid-identity, duplicate/conflicting and regressed records. No timestamp is rewritten, clamped, retried or refreshed. Failure still latches the original inhibit reason and removes any lease.

The first failure captures one bounded in-memory snapshot with sampled wall time, health and owner identity/sequence/timestamp, computed health age and exact named predicate. `scripts/stage4_safety_watchdog.py` emits that snapshot as one flushed JSON line to its existing redirected log on failure only. There is no per-cycle logging or extra thread. The ZeroOnly adapter policy and all motion-mode 50 ms deadlines/interlocks are unchanged.

Deterministic reproduction showed the old CLI sampled 20.000 before a read whose atomic update published timestamp 20.010 and failed on negative age. The corrected order passes the same adapter and owner interleavings. A genuine 151 ms old health sample still fails as `health.age_gt_150ms`.

## Verification

- Focused repository suite: **82 passed**. Command covered the new first-fault tests plus existing health I/O, safety guard, startup repair, zero boundary and unqualified motion tests. Output is in `experiments/g1_g5/evidence/watchdog-firstfault-20261023/raw/focused-tests.log`.
- The new 12 first-fault tests also pass against the isolated installed package.
- Fresh sequential `colcon build --packages-select course_navigation` passed to `experiments/g1_g5/evidence/watchdog-firstfault-20261023/artifacts/install/`.
- All 20 installed Python package files match their source SHA256. Resolved runtime `stage4_control.py` SHA256: `4c9e32b169dbe8d0ced5ea83cbcc8bed1ca99c96c139cac5c20c74d53e2525be`. Full identities are in `experiments/g1_g5/evidence/watchdog-firstfault-20261023/runtime-identity.json`.
- `git apply --check` passed against expected HEAD before application; source patch is retained at `experiments/g1_g5/evidence/watchdog-firstfault-20261023/raw/watchdog-proposal.patch`.
- All 29 inherited compact ZeroOnly checkpoint files still match the previously audited hashes. `PLAN.md`, its existing E1 plan, reports, and evidence were not edited by this watchdog change.

## Next steps

Independent review this patch and report. If accepted, separately authorize the next single bounded ZeroOnly qualification using the now-correct isolated runtime. The prior trial failed and probe cleanup was incomplete, so full ZeroOnly PASS still requires the complete stationary interval, runtime/graph evidence, contact end closure and probe cleanup. E1 stays behind its existing PLAN.md conditions; no new E1 design or motion authority is introduced here.

## Worktree state

Branch `experiment/unattended-g1-g5-20261019`, HEAD `4ae85f4148a89a76b1ad25c5b059c2140be41b85`. The inherited checkpoint remains uncommitted, with this watchdog fix separate and also uncommitted. No inherited report/evidence was overwritten. No live process was started; the isolated build and offline test subprocesses completed. No commit or push occurred.
