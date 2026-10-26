# Negative pure-yaw diagnostic — 2026-10-26

Single agent, branch `navigation/g1-g5-recovery-20261025`, exact session base
`8df7df454f03af8dc6a568732b37966907e93d36`. Evidence root:
`artifacts/recovery/negative-yaw-20261026/` in the delivered APRIL payload.

## Outcome

Exactly ONE simulator attempt, domain 212, spawn request (0,-1). **Rejected
before motion**, at the initial unchanged observed-free pocket admission.
Zero pulses, zero navigation goals, no retry and no separate stationary session.
Actual nonzero duration **0 simulated seconds**: no delivered nonzero interval.
This is NOT a successful 0.25-second yaw diagnostic or a duration qualification.
The nominal .25-second pulse would stop on the first clock deadline observed
by the existing loop; the requested <=.30 delivered-duration criterion was never
exercised. No hard real-time expiry is claimed.

`live/precheck-3d48bb2a35944476bc1ab161d69393da/session-report.json` has
`precheck=[]`, success=false and `Sensor-observed 1.5m open pocket absent`.
At sim18.595, the sim18.0 map has exactly ONE unknown intersecting cell among
2944 cells checked; zero occupied/outside/invalid cells. Cell (376,425), map
bounds approximately x[-1.20,-1.15], y[1.25,1.30], nearest point 1.487999m from
the checked sensor pose (-.342927,-.000109). Unknown is correctly rejected.
Map coordinates differ from spawn world coordinates; truth at snapshot is
(-.053974,-1.010581), yaw-.056151, after startup settling. A historically
verified spawn does not make every new live map immediately admissible.
No gate, radius, unknown-cell rule or startup sequencing was changed to admit it.

At rejection, the recorded readiness snapshot's predicates all passed: TF and
odom age .195s; truth age .035s; fresh contact support all four wheels; final
six-axis exact zero. `readiness_first_fault` is the exception-time diagnostic
snapshot with `passed=true`, not a TF/odom fault. Truth/contact first faults null.
Owner initial measured rest was not reached. Last observed five seconds of
truth (13.74–18.74) show .000309m maximum drift, 2.113e-7rad yaw excursion,
.0000899m/s peak speed and 5.013e-7rad/s peak yaw speed. This retrospective
stationarity does not substitute for the harness's qualified rest phase.

## Signed response comparison

| Layer | Recorded positive request | New negative request |
| --- | --- | --- |
| Requested command | vx=vy=0, wz=+.05 | planned vx=vy=0, wz=-.05; never published |
| Model command | +.08 after unchanged gain1.6 | all 837 recorded command triplets zero |
| Raw wheel targets | negative right-minus-left differential | no negative-command physical sample |
| Measured wheel dq | negative right-minus-left differential | stationary feedback only |
| Body gyro/truth | negative gyro and yaw | no admitted yaw response |
| Zero transition | idle wheel targets0, learned legs continue | remained zero/idle throughout |
| Rebound/rest | opposite yaw, then several seconds to rest | no pulse/rebound/zero-to-rest measurement |

The prior positive run follows forward/rest, whereas the new run fails before
initial qualified rest. **There is no matched-rest physical negative comparison.**
Old owner pulse window32.2–33.2: yaw-.154183956rad; after zero to rest,
+.151055087rad; net-.003128869rad. Observer final-command receipt clocks are
32.235–33.21 (.975s); model-command receipt clocks32.24–33.22 (.980s).
These unstamped-message receive intervals are not exact DDS delivery timestamps.
Using truth nearest the observer zero at33.21 instead gives-.155518163rad and
+.152389293rad; the boundary change explains the difference, not a new result.
No historical values were overwritten or silently reinterpreted.

Old first .1s: mean applied wheel differential about-2.855rad/s,
dq differential-.769rad/s, gyro-.0807rad/s. Next .15s gyro-.1865rad/s.
Whole old turn target means FR/FL/RR/RL approximately
[.0375,1.5719,-2.2389,2.7375]rad/s; feedback[.1202,.7372,-.4778,1.0730].
At zero, wheel targets are overridden while learned leg actions continue.
At33.3–33.5 dq differential+1.6984rad/s and gyro+.1651rad/s follow.
Whole pulse+stop peaks: .056074m translation from turn start, .157630rad yaw,
.058336m/s speed and .220313rad/s yaw speed. After observed zero, rest window
starts2.265sim s later and ends7.6775sim s later (nearest truth sample40.89).
Old actual torque and contact wrenches were not recorded, so no sole braking,
leg-transition or contact cause is claimed. `comparison.json` retains raw leg
means/RMS, all windows and precise timing bases; `analyze.py` reproduces them.

New observer evidence: 483 final Twists, all six-axis zero (479 have a receive
clock); 837 network arrays, all command triplets zero; 837 joint target arrays,
3747 controller torque arrays, 1874 named joint feedback samples. Target/torque
arrays map using `course_sim/config/rl_sar.yaml` order FR/FL/RR/RL legs then
FR/FL/RR/RL wheels; named feedback is joined by joint name, not array position.
Torque topic is the controller's published torque demand, not an independent
measurement of applied contact torque. No new telemetry subsystem was built.

Additional OFFLINE fixed-state interventions on ten new preadmission observations
(sim18.3–18.5), changing only obs[8] to-.08/0/+.08, give raw right-minus-left
wheel target means+.209046/-1.501758/-2.748857rad/s. They closely match old
pre-turn rest+.204/-1.503/-2.751. Raw zero-command inference is bypassed by idle
wheel override. This strengthens the evidence for a model-origin negative local
yaw sensitivity and nonzero state-conditioned offset. It is not a physical
negative-yaw result or a proof of a global reversed convention. Raw target
changes alone do not isolate coupled legs, contact or training/plant mismatch.
Original training-run provenance remains incomplete.

## Cleanup and retained boundaries

362 contact samples; contact failure null. Watchdog accepted the fault latch,
paused all36 server threads (T) and reported emergency containment. Owned
fallback cleanup PASS: all19 registered PID/start-tick identities gone,
remaining sets empty; localization ingress/thread cleanup PASS. Independent host
audit rechecked server absence, empty token ownership and reacquired/released
domain212 lease. No broad kills. **Normal cleanup was not exercised this run**;
its earlier 628-zero/19-process stationary PASS remains historical evidence.
No physical postfault rest was certified; pausing physics is containment.

Unchanged: 1.5m observed-free disk, .35rad yaw/.25m translation bounds,
.4rad/s angular/.25m/s physical speed vetoes, TF/odom<=.5s, contact checks,
exact zero/rest, phase deadlines, independent420s watchdog, normal/fallback
owned cleanup. No controller, policy, gain, actuator, hardware or goal changes.
No policy requiring renewed stationary qualification for this isolated axis
selection was found; prior qualification retained, no extra session consumed.

## Minimum patch, review and next actual correction

Only opt-in `--precheck-axis negative-yaw` selects one -.05 pure-yaw pulse;
default `forward-turn` retains the existing two pulses. Invalid use outside
precheck mode is rejected before input checks. Same publisher, loop and stop.
Observer adds subscriptions to existing targets/torques/named feedback only.
175 focused offline tests PASS. New yaw tests first failed against old behavior,
then passed; existing default, gate, zero/rest, veto, watchdog and cleanup tests
remain passing. Offline preflight PASS46 source/install pairs, native pins and
26 collision handles. Single-agent scoped diff/evidence review PASS; no
independent reviewer or yaw-control PASS claimed.

**Most likely narrow cause remains command-dependent reversed wheel differential
inside the deployed learned policy, plus a zero/idle transition involved in the
opposite stopping motion.** Inverted local yaw response, state bias and
model/actuation/contact mismatch remain an unresolved combination. A global
sign flip, speed rescale, gain/idle change or policy replacement is not justified.
There is **no conclusively supported production control correction to patch**.
The smallest next step is to resolve observed-free admission timing/coverage
from this saved grid and startup evidence without changing its predicate, then
seek separate authority for one matched-rest negative-yaw diagnostic. No
admission fix or second motion experiment is included in this session.

Start→G2 remains BLOCKED by yaw/stopping qualification and route envelope;
this attempt adds an observed-free admission failure. The145-pose Start→G1
path remains planning evidence. Actual Start→G2 and continuous G1→G5 video
are NOT DONE. Navigation .3m vs staging .05m tolerances remain distinct.
Commit/push reviewed changes, deliver exactly ONE APRIL bundle+SHA256, STOP;
further live work requires a new session.
