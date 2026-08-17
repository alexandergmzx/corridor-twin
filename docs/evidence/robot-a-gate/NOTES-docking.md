# A drives to what it sees — first successful dock

**2026-08-12 09:39 CST.** Isaac Sim 5.1.0.0, RTX 5070 Ti, domain 67, robot1,
`nominal_m6_n3` at 0.42 scale. Artifact: `nav-robot1-docked.json`.

```bash
bash tools/corridor_profile_run.sh --robot robot1 --profile nominal_m6_n3 \
  --gated --allow-contract-fail --domain 67        # docking is on by default
```

## What happened

```
dock: armed, expecting a landmark of radius 0.063 m
dock: refine 1 -> (3.881, -2.240) [map], landmark seen at 1.470 m
dock: refine 2 -> (3.748, -1.650) [map], landmark seen at 1.227 m
```

| measure (world frame, from truth) | before docking | **with docking** |
|---|---|---|
| closest approach to the standoff | 0.244 – 0.774 m | **0.281 m** |
| final error | 0.85 – 8.70 m | **0.281 m** |
| **walked away after arriving** | 0.85 – 5.62 m | **0.0002 m** |

**The last row is the result.** Every previous run reached B and then left,
because the goal lived in a map frame that drifts. A now arrives and stays.

## Why it works while the map is still broken

The refined goal is the robot's current pose composed with a range and bearing
it can see *at that instant*. It depends on the transform now, never on the
map's history, so accumulated drift cannot move it. The map on this run was as
wrong as ever — the arrival gate is still red — and A still stopped beside B.

Two refinements were spent of a budget of four, and the second was issued from
1.227 m. Arrival is judged on the **sensed range** to the landmark, never on a
map-frame number.

## What this does not do

- It does not fix the map. The fusion anomaly is untouched, and the arrival
  gate (Nav2 `SUCCEEDED`, ≤ 0.15 m map-frame) remains **red**.
- It is not a motion primitive. Every metre is governed Nav2 executing a
  `NavigateToPose`; docking only chooses where that goal is. No raw `cmd_vel`,
  no search, no exploration.
- `--no-dock` runs transit-only, which is the configuration the demonstration
  must still pass in.

## The constraint this relaxes, deliberately

The ratified scope said the landmark is "terminal-docking refinement, never the
arrival mechanism", triggered after Nav2 reports `SUCCEEDED`. Nav2 never reports
SUCCEEDED here, so that trigger could never fire and the landmark stayed inert.
Relaxing it was an explicit operator decision on 2026-08-12 ("Both, landmark
first"), taken so the demonstration works end to end while the fusion anomaly is
chased underneath.
