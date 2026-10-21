# Stage4 ZeroOnly startup repair — offline result, 2026-10-21

**Overall: PARTIAL / live qualification NEEDS FIX. NONZERO NO-GO.**
The experimental worktree now lives at `/home/zyd/program/b2w-embodied-navigation-course-stage4` on
`experiment/unattended-g1-g5-20261019`, HEAD1f333657. No commit/push or full Gazebo
session occurred. No goal, nonzero command or motion lease was sent.

## Preservation and storage

`git worktree move` succeeded after checking idle ownership and a free destination.
All662 pre-move file/symlink identities, HEAD/history, branch registration and four
untracked review records matched immediately after relocation. All other worktree
registrations were retained. The shared source checkout was not edited.
Original trial evidence is copied under `../zeroonly-20261021/raw/`; all19 entries
of the existing manifest matched SHA256. Compact index/input-index map original
paths to persistent copies. Original world, converted SDF, inventory, observer
library, inputs and policy metadata bytes are also preserved. Historical reports
and manifests retain their original paths/bytes; indexes provide the new locations.
Raw files and build artifacts are ignored, local only and NOT remotely backed up.
Compact records/indexes are staged for Git review; this task makes no commit/push.
APRIL_REVIEW_BUNDLES is a temporary upload/exchange area, not the archive.

## Backend diagnosis: reproduced mechanism, runtime cause still unproved

The first real failure remains GTSAM IndeterminantLinearSystemException near b0.
Current installed/build IMU and GTSAM binaries have equal build IDs and equal ELF
`.text` hashes. Whole-file hashes differ across installation; code sections/build IDs agree.
RPATH details are retained separately; no changed machine code was evidenced. Properly sourced ldd resolves the expected local .deps GTSAM;
headers use system Eigen, march-native OFF. No mismatch was evidenced in the current
environment. Historical process library maps were not captured, so exact historical
loaded identities cannot be retroactively proved. Full identity/flags/link/cache
records are in provenance.json and raw/.

The initial graph has pose sigma0.01, velocity sigma10000 and bias sigma0.001 priors;
all15 initial state dimensions are constrained. A standalone native probe using the
installed library passes initial priors and a positive0.005s IMU interval. A zero
preintegration interval reproduces the **same b0 exception**, in both Cholesky and
QR. Thus blindly adding priors, tuning noise or switching QR is not justified.
The existing update path admits an empty integration interval and constructs an
ImuFactor plus bias factor scaled by sqrt(deltaTij); this is a credible mechanism.
It is NOT proof that zero deltaTij occurred in the real run.

Original public IMU evidence contains364 sampled, strictly advancing headers
(first0.18,last19.21s; sampled deltas0.015–0.075s), not all3807 callback timestamps.
Acceleration/gyro vectors, internal correction stamps, queue contents and graph
phase were not recorded. Sensor corruption versus empty startup interval cannot
be settled from those headers. Build cache also shows GTSAM_WITH_TBB=ON despite
fetch script passing the unused GTSAM_USE_TBB=OFF option; that fact is retained,
not changed or asserted as the crash cause.

Repair preparation: `scripts/patches/lio_sam-stage4-startup-diagnostics.patch`
adds opt-in phase/key/correction/queue bounds/last IMU/integrated dt/factor/value
logging and rethrows every optimizer exception. No factor, noise, estimate or
interval is altered. With the flag unset the optimizer call passes through.
It applies to an isolated copied source tree and the IMU target builds successfully.
The original third-party source/library was not modified. A local regular package
copy selects this diagnostic executable while retaining unchanged other backend
executables. This is diagnostic preparation, NOT a backend crash fix.

## Adapter diagnosis: original attribution remains unresolved

The original reason adapter_publication_deadline occurs before publish and health
I/O in that callback. Original logs did not separate callback thread CPU from
elapsed wall time; they cannot attribute compute work versus descheduling/blocking.
No causal connection to the independent backend abort is asserted.

Minimal added health fields record cumulative wall/thread CPU sections for
localization, context, guard and publication, plus previous health-file I/O cost.
The first deadline timing remains sticky across later fast fault callbacks; a
RED→GREEN regression verifies this. The existing50ms prepublish and total-health
checks, .15s/.25s watchdog rules, motion lease checks and all hard interlocks remain.

Bounded offline callback measurements (fixture TF/odom, real guard and file I/O):

| Case | prepublish wall max | thread CPU max | Outcome |
|---|---:|---:|---|
|30 baseline callbacks|1.761ms|1.747ms|No fault, all zero|
|Injected65ms sleep|66.174ms|1.106ms|Same deadline latch, all zero|
|Injected60ms thread CPU|60.800ms|60.720ms|Same deadline latch, all zero|

These distinguish injected wall-dominant from CPU-dominant callbacks, not historical
runtime load. Blocking I/O and scheduler delay both contribute to wall-minus-CPU;
sections are needed for the next reviewed measurement. No threshold was increased.

## Contact repair: offline source/delivery contract PASS, physics unqualified

The actual Observer constructor advertises both native evaluation topics in an
isolated Ignition partition without a world. The installed get_factory mappings
accept Contacts and StringMsg; native→ROS conversion preserves sequence frame,
stamp, collision names and inventory JSON. None of this establishes engine/body
coverage or actual world-time contact intervals.

The Stage4-only session now launches exactly one owned GZ_TO_ROS evaluation bridge
for `/b2w/evaluation/full_contacts` and `/b2w/evaluation/collision_inventory`.
Bridge exit fails bootstrap/supervision. Baseline launch, command and Ground Truth
paths remain unchanged. A negative regression proves bridge failure prevents
watchdog/probe creation. No live delivery PASS is claimed.

## Verification and candidate preparation

- Final affected Python set: **83 passed**, no failures/skips (5 new regressions).
- Native CTest **4/4 passed**:2 positive graph cases,1 expected zero-dt failure,
 1 actual native-advertisement/installed-conversion contract. Separate QR zero-dt
 probe also returns3 with b0, as expected. This is not a physics test.
- Diagnostic IMU target and native observer plugin build. Four pure Python course
 packages have regular copy installs; **61** current module/config/launch
 identities verified. No dependencies or system packages installed.
- Candidate v2 inputs use persistent inventory/SDF/world/plugin paths; readonly
 launcher validation passes. Initial candidate mode0664 was correctly rejected;
 changing only this new control record to0600 passes the unchanged security check.
 No session/output directory was created.
- Early test setup mistakes are retained: a ROS log-directory permission error,
 then missing course package underlay. Correcting environment only passed the
 affected launch test; final83-test evidence uses the correct environment.

`candidate-env.bash`, `zero-inputs-candidate.json`, artifact-identities.json and
copy-installed-identities.json describe the **unapproved future candidate**.
The environment has no auto-launch and no /tmp underlay dependency. The candidate
prefix is selected explicitly and B2W_STAGE4_LIO_DIAGNOSTICS=1 enables diagnostics.
The existing v2 validator pins its original fields, not the added backend identity;
manual comparison against the recorded backend hash remains a required preflight.
These local ignored artifacts have no remote backup or live qualification.

## Before/after and next gate

Exact old/new source excerpts are the accompanying actual patch: adapter gained
only timing/retention metadata, the session gained the one-way evaluation bridge,
and the backend patch wraps optimizer.update with diagnostics/rethrow. No policy,
footprint, safety bounds, motion guard or solver/noise parameter was changed.

Remaining uncertainty: actual graph phase/deltaTij/IMU coverage at the b0 abort;
actual wall/CPU section causing the50ms miss; world-time contact delivery/closure;
bootstrap TF/maps/costmaps and watchdog readiness under real load. Original
ZeroOnly_RUNTIME_FAIL remains. Start→G1 and subsequent motion remain NO-GO.

**Next bounded experiment, only after external review:** one existing ZeroOnly
startup/trial using the pinned diagnostic backend, sticky adapter timings and
one-way contact bridge, with unchanged90s bootstrap/220s supervisor and existing
probe/cleanup limits. Record the first internal correction/IMU interval and first
deadline evidence, then stop on failure. No trial has been repeated here.

External review is the next action. No commit, push, live repeat or motion before it.
