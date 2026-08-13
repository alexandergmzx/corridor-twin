# ADR 0031: B is the cylinder, and what "delivered" means

- Status: Accepted
- Date: 2026-08-12
- Source: the 2026-08-12 evening handoff, which ratified the merge; the
  2026-08-12 13:16 phantom run recorded in
  [ADR 0029](0029-map-divergence-at-the-corner.md) and in
  [`corridor_dock.py`](../../tools/corridor_dock.py)'s containment comment.
- **Supersedes the beside-B geometry of
  [ADR 0029](0029-map-divergence-at-the-corner.md)** — "a cylindrical post
  beside B, authored from config so it scales" becomes one cylinder **at** the
  delivery point. 0029's decision that B carries a *lidar-detectable, geometric,
  laser-frame* landmark is retained in full; only the second prim is retired,
  along with the separation floor that existed to keep the two apart.
- Builds on [ADR 0030](0030-the-committed-scale-and-what-is-measured-against-it.md),
  whose named unscaled set this record splits: the radius stays out of the
  scaling, the height joins it.
- Relates to [ADR 0028](0028-goal-directed-navigation-on-a-live-map.md): the
  arrival gate is unchanged.

## Context

The scene described B twice. `/World/Actors/B` was a 0.135 m box — the thing a
viewer and P's camera see — and `/World/Actors/BLandmark` was a 0.12 m-radius
cylinder standing 0.8 m south of it, the thing A's lidar can actually fit a
circle to. The manifest carried five keys across the pair, and five consumers
read them.

Two descriptions of one recipient is a place for the scene to contradict
itself, and it did. On 2026-08-12 at 13:16 the detector confirmed something at
0.910 m near the spawn, docking re-aimed the mission at it, Nav2 drove half a
metre and correctly reported "Reached the goal!", and the world-frame delivery
error was 5.754 m. On the lens that run is two circles far apart — the yellow
manifest marker for where B is, and the pink crosshair for what was confirmed.
Nothing about the merge would have prevented that particular failure, which was
a stale arena; what the merge removes is the *class*: with one object there is
no second place for the scene to say where B is.

The separation between them was never a scenario fact either. It existed for
one reason — so the detector's clustering would see two objects rather than a
box with a bump on it — and its floor was `B's half-width + the post's radius +
the clustering gap`. That is a constraint about a problem created by having two
objects.

## Decision

### 1. One cylinder, at the delivery point

`/World/Actors/B` is a `UsdGeom.Cylinder` in the drawing's pocket. The person
prop, the separate post prim, the `landmark_offset_m` constraint set,
`geometry.landmark_xyz()` and the B-to-post separation floor are all removed.
The manifest carries `b_xyz_m`, `b_radius_m`, `b_height_m` and `a_size_xyz_m`;
`b_size_xyz_m` and the `landmark_*` trio are retired.

### 2. The radius is the sensor's; the height is the person's

`b_radius_m` = 0.12 m, **absolute**, and stays in `scale_scenario`'s
`NOT_LENGTHS`. It is set by the MS200's 1° angular resolution: at the 3.0 m
arming radius a beam lands every 5.2 cm, so a 0.24 m body gives ~4.6 returns
against the 4 the circle fit needs. At the 0.045 m it once scaled to, that was
1.7 — undetectable at range, and small enough that ordinary corner geometry
fitted a circle the same size.

`b_height_m` = 1.7 m authored → **0.51 m** at the committed 0.30 factor, because
a height describes a person and a person scales.

**Stated cost:** at robot scale B is 0.24 m across and 0.51 m tall — stout for a
person. The sensor's requirement wins, and the alternative (a 0.135 m-wide B
that looks right and cannot be detected at range) is the failure this project
already paid for once. Whether B wants a slimmer, taller profile is a scenery
question, not a detection one, and it is left open.

### 3. Contact semantics: derived, and the governor is never bypassed

The final-approach distance — how close A may come to B's **centre** — is

```
max( governor stop floor + b_radius,  a_length/2 + b_radius + goal_tolerance )
```

computed by [`corridor_dock.final_approach_m`](../../tools/corridor_dock.py),
never authored. Both terms come from committed constants:

| term | value | source |
|---|---|---|
| governor `stop_distance` | 0.35 m of **laser range** | `yahboomcar_safety/governor.py:44` |
| `b_radius_m` | 0.12 m | the manifest |
| A's length | 0.195 m | the manifest (`a_size_xyz_m`) |
| arrival tolerance | 0.15 m | ADR 0022, `corridor_nav_gate.GOAL_TOLERANCE_M` |

At the committed scenario the governor's floor is **0.470 m** and geometric
contact is 0.368 m, so **the governor decides**. That is the point: the demo win
is declared at a distance the safety envelope actually permits, and **the
governor is never bypassed to reach it**.

The ordering is computed, not assumed. A bigger B does not flip it — the radius
enters both terms and cancels — but a robot longer than 0.40 m does, and the
`max` is what makes that automatic rather than a future bug.

**Demo win = `DELIVERED` with world-frame distance-to-B ≤ that value**, measured
from simulator truth **on the evaluation plane**. It is a report, never an
observer input.

**The Nav2 arrival gate is unchanged**: `SUCCEEDED` within 0.15 m in the map
frame (ADR 0028), and the demonstration must still pass with the detector
disabled.

### 4. Containment is re-derived against the merged B, and one guard weakens

The transit goal stands 0.6 m from B's centre, so the detection and the goal are
now **0.600 m** apart rather than the 1.000 m of the beside-B geometry. Every
containment number was re-derived rather than carried, and one of them moved:

- **Travel window** — unchanged. Still `ARM_WINDOW_ROUTE_FRACTION × route-to-
  delivery`, 0.900 m at the committed scale.
- **Map-frame goal proximity** — unchanged.
- **Bearing cone — widened, 60° → 76°, and this is a real cost.** With the post
  0.8 m south of B and the goal 0.6 m west, the two sat on different sides and
  ±60° was generous. With one object, B lies 0.6 m lateral of the goal on the
  side A approaches from, so the bearing swings *harder* as A closes: measured
  **63.4° at 0.3 m from the goal**, which the old cone refused. The cone is now
  derived as `atan2(delivery_standoff, goal_tolerance)` = 75.96°.

  A 76° cone excludes little beyond "behind the robot". Said plainly: **the
  merge makes the weakest of the three containment guards weaker still.** What
  actually excluded the spawn phantom was the travel test, and the negative
  control asserts against that test by name — it is unaffected, and it stays
  red.

## Consequences

- The lens draws one yellow ring. The pink confirmed-detection crosshair is now
  supposed to land **on** it; two circles far apart remains the phantom
  signature, and it is easier to read, not harder.
- `check_arena_matches_manifest` compares one prim instead of two. The stale-
  arena control keeps working through B's own displacement rather than through
  a missing post.
- `DELIVERY_STANDOFF_M` moves from `corridor_nav_gate` into `corridor_dock`,
  because the bearing cone is derived from it and one number with two homes is
  how two numbers get born.
- Two stale constants were corrected in passing, both touched by this change:
  `test_delivery_standoff.py` held `robot_radius 0.12 / inflation 0.16` against
  the live 0.128 / 0.18 that ADR 0029 measured, and built its scenario at factor
  0.3333 rather than the committed 0.30 — a standoff test measuring a scene
  nothing runs.
- Arenas must be rebuilt; the manifest hash changes, and the runner's
  arena-vs-manifest precondition refuses the old ones, which is the guard
  working.
- Any run recorded before this change measured a different scene. Closest-
  approach figures are not comparable across it; map and yaw figures are,
  because neither depends on B.

## Alternatives considered

- **Keep both prims, make the post B's "badge".** Rejected: it keeps two
  descriptions and buys only the visual, which the cylinder's height already
  provides.
- **Scale the radius with the scenario so B stays person-proportioned.**
  Rejected by measurement — this is exactly the 0.045 m post that produced the
  phantom, and ADR 0030 already named it as a deliberate exception.
- **Keep the ±60° cone and accept that it refuses B inside 0.35 m of the goal.**
  Rejected: a guard that refuses the thing it guards is a guard that gets
  switched off. Widening it and *saying* it is weaker is the honest trade.
- **Make the contact distance a chosen round number, e.g. 0.40 m.** Rejected:
  it is the number the demo is scored on, and a scored number that is not
  derived from the safety envelope invites the question this ADR exists to
  answer.
