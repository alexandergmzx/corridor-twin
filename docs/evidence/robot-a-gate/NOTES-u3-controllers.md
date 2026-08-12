# U3 — DWB vs MPPI: **INCONCLUSIVE**, and why

**2026-08-12 03:42–03:50 CST.** robot1, `nominal_m6_n3`, 0.42 scale, domain 67.

## The arms

Two param files differing in exactly one block, pinned by tests
(`test_robot1_nav_governed.py`): `planner_server`, `behavior_server` and both
costmaps must compare equal, and both arms must carry the measured
`robot_radius: 0.128` and `inflation_radius: 0.30`. A comparison run on two
different robots measures nothing.

Both arms carry `vx_max 0.22` and `wz_max 0.4`, the latter because the governor
clamps yaw to `max_yaw_near` 0.4 rad/s inside `stop_distance`, which is most of a
1.26 m corridor.

```bash
bash tools/corridor_profile_run.sh --robot robot1 --profile nominal_m6_n3 \
  --controller mppi --gated --allow-contract-fail --domain 67
```

## Result

| arm | closest approach (world, truth) | landmark | note |
|---|---|---|---|
| DWB | 0.244 / 0.404 / 0.574 / 0.647 / 0.774 m | detected | five transits |
| **MPPI** | **7.085 m — did not move** | not detected | goal not accepted |

**MPPI is not measured.** Two attempts were spent on a configuration error of
mine (`CostCritic.consider_footprint: true` against a costmap that describes the
robot as a circle, which aborts lifecycle bringup); the third brought
`bt_navigator` to ACTIVE on the first try and then the goal was not accepted, so
the robot never moved.

The map scored **0.000 m duplicate wall extent** on that run. That is a
**degenerate pass** — a stationary robot maps one viewpoint perfectly — and must
not be quoted as MPPI producing a better map.

## What this does and does not support

It supports nothing about MPPI's behaviour in a narrow corridor. The arm is
committed, tested and selectable with `--controller mppi`, so the comparison is
one working run away; it was not obtained tonight.

DWB's numbers stand, and they are the arm the demonstration currently uses.

## The honest caveat on any controller comparison right now

The map diverges, and `goal not accepted` has recurred across the night on both
arms. Until the fusion anomaly (`NOTES-fusion-anomaly.md`) is resolved, a
controller comparison risks measuring the divergence rather than the controllers
— which is why this was evaluated on world-frame delivery from truth, and why
its failure is reported rather than dressed up.
