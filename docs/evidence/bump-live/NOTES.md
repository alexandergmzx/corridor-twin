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
