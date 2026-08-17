# The creep reaches B, and neither witness notices

First live run of the silhouette mask, the slow-zone exemption and the dual
witness (ADR 0034). **The governor fixes work. The arrival witness does not.**

## Command

```bash
cd /home/alexmint/Development/robot-fleet/src/corridor-twin   # the SYMLINKED path (D5)
bash tools/corridor_profile_run.sh --profile nominal_m6_n3 --robot robot1 \
     --allow-contract-fail
```

| | |
|---|---|
| Run | `20260814-023306-robot1-nominal_m6_n3` |
| Date | 2026-08-14, 02:33–02:37 CST |
| Isaac Sim | 5.1, RTX 5070 Ti, GUI session via `simctl --backend isaac` |
| Robot | robot1 (ADR 0027), `ROS_DOMAIN_ID=67` |
| Profile | `nominal_m6_n3`, dock ON, lens ON, **reported only — not a gate** |
| Session bag | `MicroROS-assets/bags/20260814-023321-isaac-d67` |
| Artifact | [`nav-20260814-023306.json`](nav-20260814-023306.json) |

**Caveat carried by every artifact in this run:** the robot1 Isaac contract
precondition FAILED and was overridden — `scan at 14.6 Hz, want ~12.0` and
`battery at 1.5 Hz, want ~1.0`. This is a standing condition on this host, not
a property of this change: the last twelve runs measure 13.4–15.1 Hz, and all
eight of the previous night's runs used the same override. The twin's own
banner says `/scan` reaches 12 Hz "by CALIBRATION against a measured but
unexplained 72-messages-per-render-second emission rate, not by the sensor
honouring its configured rate". Recorded as an open item, not fixed here.

## Result: PARTIAL

| | before (nine runs) | this run |
|---|---|---|
| Closest approach to B (truth) | 0.3455 m | **0.2252 m** |
| Short of the 0.2175 m contact | 128 mm | **7.7 mm** |
| Governed `/cmd_vel` during the creep | 0% moving below 0.35 m | **476 / 476 ticks, 100%** |
| Creep speed | throttled to ~8.7 mm/s, then pinned | full 0.05 m/s throughout |
| Reported state | `ARRIVED_UNPROVEN` | `ARRIVED_UNPROVEN` |

The state is unchanged and the run is a FAIL. What changed is *why*: A now
drives all the way in and is not confirmed, where before it could not move at
all.

### Reproduced, 03:02:42 -- and two runs that did not get there

Run `20260814-030242` repeats it on a different live map and approach path:

| | run 023306 | run 030242 |
|---|---|---|
| Last sighting of B | 0.2333 m | 0.2271 m |
| A-to-B distance, TRUTH | **0.2252 m** | **0.2258 m** |
| Short of the 0.2175 m contact | 7.7 mm | **8.3 mm** |
| Final bearing to B | 0.6 deg | 0.1 deg |
| Disc declarations published | 3431 | 3495 |
| State | `ARRIVED_UNPROVEN` | `ARRIVED_UNPROVEN` |

The 6 mm spread is beam discretisation, not drift. **The terminal docking is
reproducible; the witness fails identically both times.** Artifacts:
[`nav-20260814-023306.json`](nav-20260814-023306.json),
[`nav-20260814-030242.json`](nav-20260814-030242.json).

### Getting to the terminal phase is now the bottleneck

Two further attempts between these runs never reached the creep at all, and
neither failure touches the docking code:

- **`20260814-025049`** -- SLAM double-walled the corridor (duplicate wall
  extent **0.920 m** against a 0.20 m bound), the NavFn planner then failed
  with *"Failed to create a plan from potential when a legal potential was
  found. This shouldn't happen."*, and the action ABORTED at 4.792 m of
  travel -- **57 mm short of the 4.849 m at which docking arms.** State
  `TRANSIT`, zero creep ticks. This is the ADR 0029 map-divergence mode.
- **`20260814-025555`** -- `slam_toolbox` never reached ACTIVE in either of
  the runner's two attempts; classified INFRASTRUCTURE and torn down. That
  run's scan came up at **8.6 Hz**, against 13.4-15.1 Hz on every other run
  measured.

**The scan rate is UNSTABLE, not merely biased high.** Across fourteen runs it
spans **8.6-15.1 Hz** against a calibrated target of ~12.0. The low end starves
SLAM; the high end is what the contract override has been waving through all
along. Desktop load competes with the sim on this host (browser, editor and
media players were live during these runs), which is a plausible contributor
and is untested.

Scored honestly: **two of four runs reached the docking phase.** The terminal
creep works every time it is reached.

### 03:13:48 -- A TOUCHES B, measured

Run `20260814-031348` is the first in which simulator truth puts A **inside**
the contact range rather than short of it.

| | value |
|---|---|
| Last sighting of B | 0.2207 m |
| **A-to-B, TRUTH** | **0.2146 m** |
| Modelled contact range | 0.2175 m |
| **Overlap** | **2.9 mm PAST contact** |
| Creep ticks / disc publishes | 3416 / 3415 |
| Reported state | `ARRIVED_UNPROVEN` |

B is a static collider (ADR 0033 §6), so A cannot pass through it. A centre
distance 2.9 mm inside the modelled contact distance means the bumper is
against B, the few millimetres being solver tolerance and slop in a contact
range that was derived, not measured. **The bump happens.** The witness still
does not see it, exactly as the 023306 analysis predicted.

This answers the question ADR 0034 left open under "What is NOT established".
That record is Accepted and therefore immutable: the resolution is recorded
*here*, and formally closing the clause needs a superseding ADR, not an edit.

Artifact: [`nav-20260814-031348-contact.json`](nav-20260814-031348-contact.json).

### 03:19:22 -- a THIRD way to never reach the creep: the handoff gap

Run `20260814-031922` is not a docking failure. It is a docking phase that
**never started**, and the mechanism is systematic rather than unlucky.

| | value |
|---|---|
| Nav2 action status | **SUCCEEDED** |
| A-to-B, TRUTH, at success | **0.6621 m** |
| Handoff threshold | **0.620 m** |
| Docking state at exit | `REFINE`, **0 creep ticks** |
| Refinements achieved | 2 (sightings at 0.988 m, 0.904 m) |
| Error against its own refined goal | 0.1983 m |

Handoff triggers **only** on a confirmed sighting at or inside 0.620 m. Nav2
stopped 0.198 m off its own refined goal, declared success at 0.662 m from B,
and the gate loop ended -- so the creep, the disc, and the whole terminal phase
were skipped in silence. Nothing logged an error; the run simply reported a
map-frame goal-error failure and stopped.

**This is a real gap and it is not fixed.** If Nav2's stopping point lands
outside the handoff radius, the docking never runs at all. Two candidate
remedies, neither implemented nor measured: place the refined goal closer than
the handoff radius by at least Nav2's own goal tolerance, or trigger handoff on
Nav2 SUCCESS as well as on range. Recorded as a finding.

Artifact:
[`nav-20260814-031922-handoff-missed.json`](nav-20260814-031922-handoff-missed.json).

### The runs were real: independently sampled lens uptime

Announcing a live lens URL for a run that has already died is a failure this
project has committed before, and "trust me, I checked" is not evidence. A
separate process sampled `http://127.0.0.1:8765/healthz` every 2 s during both
runs and wrote timestamped lines without consulting the agent:

| run | first `lens=ok` | last `lens=ok` | verified uptime |
|---|---|---|---|
| 031348 | 03:15:43 | 03:17:50 | **127 s** (64 of 152 samples) |
| 031922 | 03:21:29 | 03:23:24 | **116 s** (58 samples) |

Raw records: [`lens-liveness-20260814-031348.txt`](lens-liveness-20260814-031348.txt),
[`lens-liveness-20260814-031922.txt`](lens-liveness-20260814-031922.txt).

**The structural problem remains and no diligence fixes it**: bring-up costs
~2 minutes, A drives for ~50 s, then everything is torn down. An observer who
looks a minute late finds a dead port, which is indistinguishable from a
fabricated launch. The `--spawn` micro-arena -- no Nav2, no lifecycle manager,
~3 minutes -- is the remedy, and these runs are the argument for building it.

### Scorecard, five attempts on 2026-08-14

| run | reached the creep? | outcome |
|---|---|---|
| 025049 | no | Nav2 aborted; SLAM double-walled the corridor |
| 025555 | no | `slam_toolbox` never reached ACTIVE in two attempts |
| 030242 | **yes** | 0.2258 m -- 8.3 mm short |
| **031348** | **yes** | **0.2146 m -- CONTACT** |
| 031922 | no | Nav2 SUCCEEDED at 0.662 m; handoff never triggered |

**Two of five reached the creep, and both drove A to contact.** The terminal
docking is no longer the limiting factor; three distinct transit and handoff
faults are.

### What is proved

**The governor never braked the creep.** Over the 23.8 s creep window,
`/cmd_vel` carried 0.05 m/s on **476 of 476 messages**. The disc mask and the
slow-zone exemption did on the robot exactly what the bench predicted — no
leaked shoulder, no stub throttle, no pin. The handoff fired correctly at
0.618 m and the detector tracked B down to a last sighting at 0.2333 m, which
is the lidar's blind edge.

### What is refuted

**Both witnesses over-report motion, so the stall never accrued.**

| signal | over the 23.8 s creep | last 2 s | truth |
|---|---|---|---|
| Wheels (`/odom_raw`, `/odom`) | median 0.0492 m/s | **0.0510 m/s** | — |
| Wheel-integrated travel | **1.19 m** | — | **0.393 m** |
| Laser net displacement | **0.882 m** | — | 0.393 m |
| Laser per-pair speed | median 0.0604 m/s | **0.0384 m/s** | avg 0.0165 m/s |
| Threshold in force | — | 0.030 m/s | — |

- **The encoders are not a bumper.** They read 0.0510 m/s in the final two
  seconds, with A pressed against B, and claim 1.19 m of travel against
  0.393 m advanced. **A slipped two thirds of its commanded distance** — the
  twin's authored rear friction of 0.1 doing what it was authored to do. This
  retires ADR 0033 §5 on measurement.
- **The laser ε came from the wrong regime.** 0.030 m/s was derived from a
  *parked* robot's 16.8 mm median. During a creep the matcher's noise floor is
  roughly four times the true speed — 60 mm/s median against 16.5 mm/s actual,
  and 0.882 m of net displacement against 0.393 m. It reported 0.0384 m/s in
  the final window, said "moving", and withheld the confirmation.

No fixed threshold on this signal separates "creeping" from "stopped against an
obstacle", because the noise exceeds the quantity being measured. The dual-
witness *structure* is unaffected; its laser half needs a different signal.

### What remains unresolved, and is not asserted either way

**Whether A physically touched B.** Truth puts it 7.7 mm short of a *modelled*
contact range of 0.2175 m, which is inside that model's own slop, and the
wheels were turning throughout. The bump may well have happened and gone
unwitnessed. Settling it by assertion is exactly the failure a witness exists
to prevent, so it is left open.

## Reproducing the analysis

The numbers above come from the session bag, not from the run's own summary:

```bash
source /opt/ros/jazzy/setup.bash
PYTHONNOUSERSITE=1 python3 -   # read /cmd_vel_raw, /cmd_vel, /odom_raw,
                               # /odom, /odom_laser over the creep window
```

The creep window is bounded by the first and last `/cmd_vel_raw` message
carrying exactly 0.05 m/s.
