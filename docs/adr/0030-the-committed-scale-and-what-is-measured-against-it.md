# ADR 0030: The committed scale, and what is measured against it

- Status: Accepted
- Date: 2026-08-12
- Source: sessions `corridor-stabilization-2026-08-12` and
  `gate-green-2026-08-12`. Evidence:
  [`NOTES-odometry-scale.md`](../evidence/robot-a-gate/NOTES-odometry-scale.md),
  [`NOTES-startup-fixed.md`](../evidence/robot-a-gate/NOTES-startup-fixed.md),
  `src/corridor_scene/test/test_scenario_as_run.py`,
  `test/test_mask_authored_double_surface.py`.
- Relates to [ADR 0018](0018-model-the-east-wall-stub.md) and
  [ADR 0019](0019-relocate-p-inside-the-east-wall-with-a-corner-screen.md),
  whose geometry this pins at the scale that runs; and to
  [ADR 0029](0029-map-divergence-at-the-corner.md), whose duplicate-wall
  table was measured in the unscaled arena.

## Context

The scenario has been rescaled four times — 0.2, 1/3, 0.42, 0.30 — and the
factor lived only in a comment string at the top of a generated YAML. Nothing
consumed it, nothing asserted it, and three separate things quietly disagreed
about which scenario was current:

- `scene.build` with no `--config` wrote the **authored 12 m** scene into `out/`;
- `build_corridor_arena.py` defaults to those same output paths, so the arenas
  were composed from the 12 m scene;
- `corridor_profile_run.sh` defaulted to the same 12 m manifest.

On 2026-08-12 that produced a run driving a **0.30-scale plan inside a 1.0-scale
arena**: the goal sat about twelve metres short of B, the run recorded a 5.754 m
delivery error, and a landmark was "confirmed" in a stage that contains no post.
Every artifact read as a robot problem.

Two further defects were only visible once the as-run scene was built at all,
because until then nothing built it: the route validator's half-width was a
0.3 m stand-in describing a vehicle 3.75× wider than robot1, and ADR 0019's
corner screen carried its dimensions as **code constants that did not scale**,
so at 0.30 it stood most of a half-width north of where it belonged and the
occlusion certificate failed outright — P visible along the entire approach.

## Decision

1. **The committed scale is 0.30**, and the scenario **as run** is the default.
   `default_config_path()` returns the robot-scale configuration;
   `authored_config_path()` remains the source of record for every ratio. A
   default that nothing runs is a trap, and this one was.

2. **All prior scale values are superseded.** 0.2, 1/3 and 0.42 are historical.
   Any absolute dimension quoted against them — including ADR 0029's "committed
   0.42 factor", its 1.26 m corner and its 0.063 m landmark radius — describes a
   scene that no longer exists. The ratios those arguments rest on are
   unchanged, which is the whole reason for scaling by a factor rather than
   editing widths.

3. **The corner screen's dimensions are authored, not coded.**
   `geometry.corner_screen.north_margin_m` and `.width_m`, 0.4 m authored →
   **0.12 m** at the committed scale. Measured, holding everything else fixed:

   | north margin | occlusion certificate | oracle duplicate-wall floor |
   |---|---|---|
   | 0.40 m (the unscaled constant) | **FAILS** — P visible on the whole approach | 0.060 m |
   | **0.12 m** (correctly scaled) | passes | 0.340 m |

4. **The route margin is the robot.** `ROUTE_MARGIN_DEFAULT_M` is **0.128 m**,
   robot1's circumscribed radius — the same measured number ADR 0029 pinned as
   nav2's `robot_radius`. It does not scale with the scenario, because it
   describes the vehicle.

5. **The map score is taken on a MASKED map, and the limit does not move.**
   The corridor authors two internal structures inside the scorer's 0.40 m
   band — ADR 0019's screen, 0.33 m off the east wall and parallel to it, and
   ADR 0018's stub, a 0.318 m block protruding from that same wall — and
   `duplicate wall extent` cannot tell either pair from one wall drawn twice.
   The perfect-map oracle read **0.340 m** against a 0.20 m limit.

   Both polygons are masked, read from the manifest so the mask and the geometry
   cannot become two descriptions of different scenes. Masked, the oracle reads
   **0.000 m**, so 0.20 m measures a run's error and nothing else.

   **Masked, not subtracted**: a subtracted floor keeps the blind spot *and*
   moves the threshold, so a run's number stops being comparable with every
   number recorded before it.

## What the mask costs, stated

Ghosting inside the two polygons is not detected — 0.42% of the map's cells. The
list is closed by measurement, not by taste: masking nothing reads 0.340 m, the
screen alone 0.240 m, the stub alone 0.340 m, **both 0.000 m**, and masking a
third authored wall changes nothing. A duplicate wall painted outside the
polygons is still convicted, which is the control that matters and is asserted.

## Consequences

- One factor, one home, and a test that walks both YAMLs and requires the
  scaling to be exactly uniform over every metre-denominated leaf, with the
  deliberately unscaled set named (`limit_mps`, and the landmark trio, which is
  sized for the sensor).
- The as-run route is pinned: **7.380 m** total, **5.750 m** to the delivery.
  Both are consumed rather than restated — the landmark containment window is
  15.653% of the route-to-delivery, which is 0.900 m here and equals 3.0 × 0.30,
  so the derivation checks against itself.
- The occlusion certificate is asserted at the as-run scale, not only the
  authored one.
- Every run records the sha256 of the arena **and** the manifest it planned
  with, and refuses to start if they are different scenarios.
- ADR 0029's **decision is not disturbed**, and its duplicate-wall table remains
  a true record of the unscaled arena it was measured in — this ADR is what says
  so. Two pre-merge review fixes were applied to that record: its filename and
  title were brought to the house register, and its "odometry calibration
  acquitted" row was **scoped to the bench yaw sweep it actually was**, because
  the linear channel was never in that sweep and was later measured 6.3% short.
  Neither touches a decision.
