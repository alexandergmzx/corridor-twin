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

## Result — **DWB stays**

| arm | closest approach (world, truth) | goal | outcome |
|---|---|---|---|
| DWB | **0.244 / 0.404 / 0.574 / 0.647 / 0.774 m** | accepted | five transits, reached B every time |
| MPPI | **6.750 m** (spawn is 7.22) | accepted | ABORTED after ~0.5 m |

**MPPI does not work on this hardware at these settings, and the log says why:**

```
controller_server: Control loop missed its desired rate of 20.0000 Hz.
                   Current loop rate is 11.7759 Hz.
                   ... 4.8195 Hz.  ... 5.0921 Hz.  ... 5.0610 Hz.
```

The optimizer runs at **4.8–11.8 Hz against the 20 Hz `controller_frequency`**
it is configured for, at `batch_size: 2000` and `time_steps: 56` on a box that
is simultaneously running Isaac Sim, SLAM and RViz. A controller sampling at a
quarter of its intended rate steers badly, and the progress checker aborts it.

That is a hardware/settings result, not a verdict on MPPI as an algorithm. A
smaller batch or fewer time steps might well change it, and that experiment is
not run.

**DWB stays**, on numbers rather than on inertia: it reached B on five transits
out of five that got a goal.

## An earlier attempt, and its degenerate pass

Two earlier attempts were lost to a configuration error of mine
(`CostCritic.consider_footprint: true` against a costmap that describes the robot
as a circle, which aborts lifecycle bringup), and one run activated but had its
goal refused and never moved.

That stationary run scored **0.000 m duplicate wall extent**. It is recorded here
as a **degenerate pass** — a robot that does not move maps one viewpoint
perfectly — precisely so nobody later quotes it as MPPI improving the map.

## The honest caveat on any controller comparison right now

The map diverges, and `goal not accepted` has recurred across the night on both
arms. Until the fusion anomaly (`NOTES-fusion-anomaly.md`) is resolved, a
controller comparison risks measuring the divergence rather than the controllers
— which is why this was evaluated on world-frame delivery from truth, and why
its failure is reported rather than dressed up.
