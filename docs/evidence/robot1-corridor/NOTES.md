# robot1 in the corridor — v2 evidence night (2026-08-11)

ADR 0027 fixed robot A = robot1. These are robot1's first corridor runs, plus
the measurement that reframes the whole degeneracy study: **the corridor was
built at 37x the robot's width, and scaling it to the robot removes the
degeneracy almost entirely.**

## Environment

| Field | Value |
|---|---|
| Isaac Sim | 5.1.0.0; RTX 5070 Ti, driver 580.173.02 |
| Domain | 67 (scratch), fresh session per run, Isaac lock held |
| Stack | `simctl start --robot robot1 --backend isaac` + this repo's governed Nav2 |
| EKF | pn-fix (simctl default), encoders + IMU, **no laser-pose input** |

## Finding 1 — robot1's twin misses its own scan contract, everywhere

`check_isaac_contract.py` (robot1's own, unmodified, `WANT_HZ` scan 12 / odom_raw
11 / imu 25) fails on scan rate:

| Arena | scan | odom_raw | imu | verdict |
|---|---|---|---|---|
| corridor (6 m scale), run 1 | 14.3 Hz | 11.2 | 25.1 | **FAIL** (want 12.0 ±10%) |
| corridor (6 m scale), run 2 | 14.2 Hz | — | — | **FAIL** |
| **stock yahboom `arena.usd`** | **14.3 Hz** | 11.2 | 25.2 | **FAIL** |

**The stock-arena control is the point.** The overshoot is identical in the
fleet's own arena, so it is a property of robot1's Isaac twin on this machine
and **not caused by the corridor scene**. It is a fleet-level finding and is
parked to the morning list — writes outside this repo were not delegated.

Runs after this were taken with `--allow-contract-fail`, which records the
failure in every artifact rather than lowering the bar. Blocking on it would
have forfeited the night to a pre-existing twin defect.

## Finding 2 — scale removes the degeneracy

Same robot, same stack, same instrument; only the world's size changes.

| | 6 m corridor | robot-scale (x0.2) |
|---|---|---|
| First `odom_laser` at station | **9.66 m** | **0.03 m** |
| `odom_laser` msgs / rate | 12 / 0.13 Hz | **1052 / 11.69 Hz** |
| Max consecutive withheld | 936 | 8–35 |
| Worst EKF gap | 1.46 s | 0.36–0.54 s |
| Distance driven in 90 s | 11.74 m | 4.11 m |
| Nav2 map-frame error | 18.46 m | **3.17 m** |

At 6 m scale the matcher produced twelve messages in ninety seconds. At robot
scale it acquires immediately and runs at rate.

## Finding 3 — robot1's architecture holds up, as predicted

At 6 m scale, with the matcher effectively dead (12 messages), robot1 still
drove 11.74 m and mapped 6376 cells, because its EKF fuses wheel encoders and
IMU and does not consume the matcher (`ekf_sim_pnfix.yaml:138-146`). Robot2 in
the same corridor had no odometry at all and its pose never left the origin
(drift 1.000). Robot1's drift was 0.066 — a real number rather than a total
loss. **That is the encoder-vs-matcher contrast ADR 0027 predicted.**

## What still FAILS

- **Nav2 never SUCCEEDED.** At robot scale the planner reaches "Failed to create
  plan with tolerance of 0.150000" through the 0.6 m corner — path feasibility
  through a narrow turn, no longer a localization problem. Earlier aborts were
  "outside bounds" (global costmap sized by the explored map only) and a dead
  `controller_server` (nav2 declares local costmap `width`/`height` as integers
  and rejects a double).
- **Drift did not settle**: 0.125 on one small-scale run, 0.049 on another,
  against a 0.05 bound. One passing and one failing run is **not a pass**, and
  n=2 does not say which is representative.
- **EKF continuity is marginal**: 0.362 s and 0.536 s against a 0.4 s limit.

## Scope

- One profile only (`nominal_m6_n3`). wide_corner and uniform were not run at
  robot scale.
- `robot_radius` 0.12 is inherited from robot2 and unverified for robot1.
- No enforcement, camera, or detector claim.
