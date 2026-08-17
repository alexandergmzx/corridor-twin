# Loop closing OFF — a falsified hypothesis, kept

**2026-08-12 03:20 CST.** Isaac Sim 5.1.0.0, RTX 5070 Ti, domain 67, robot1,
`nominal_m6_n3` at the committed 0.42 scale.

## The hypothesis

The map→odom correction takes a single **2.83 m step** with 68 steps over 5°
(`transit-audit-225511.json`). That magnitude cannot come from the correlative
matcher, which `correlation_search_space_dimension` bounds to ±0.3 m per match —
so it must be the pose graph re-optimising, i.e. the loop-closure path.

A's delivery is a single pass and never revisits, so there is no loop to close
and any accepted closure is necessarily false. A tapered corridor is the
textbook generator of them.

## The test

`config/robot1/slam_robot1_corridor.yaml`, one deviation from the fleet
canonical: `do_loop_closing: false`. Launched via `slam_launch.py
params_file:=` with `simctl start --no-slam`, so no fleet file changed.

```bash
bash tools/corridor_profile_run.sh --robot robot1 --profile nominal_m6_n3 \
  --gated --allow-contract-fail --domain 67
```

## Result — **FALSIFIED**

| map | duplicate wall extent | limit |
|---|---|---|
| authored reference ("perfect SLAM") | 0.000 m | — |
| loop closing ON, four transits | 0.740 – 2.680 m | 0.20 m |
| **loop closing OFF** | **1.740 m** | 0.20 m |

Squarely inside the range with it on. **Disabling loop closure changes
nothing**, so the 2.83 m correction step is not a false closure and the graph
optimiser is not the mechanism.

The setting is kept, because the scenario argument for it stands on its own — a
single-pass delivery has no loop to close — but it is **not a fix** and must not
be quoted as one. `--fleet-slam` restores the canonical for an A/B.

## What this leaves

The fusion anomaly. `robot_localization`'s yaw scale against truth, every run
measured tonight:

| run | yaw scale ratio |
|---|---|
| 22:55 transit | 1.213 |
| 23:39 transit | 1.606 |
| 00:14 transit | −1.707 |
| 00:28 transit | **23.434** |
| 03:02 transit | 0.594 |
| 03:12 transit | −1.518 |
| 03:20 transit (loop closing off) | **0.140** |

Seven samples spanning **0.14× to 23.4×, in both signs**, from a filter whose
only yaw input measures 0.987–0.993 of truth. A calibration error has a sign and
a magnitude; this has neither. It is the top open item and it lives outside this
repository.
