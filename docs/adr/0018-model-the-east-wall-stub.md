# ADR 0018: Model the east-wall stub, recess B behind it, and extend the route

- Status: Accepted
- Date: 2026-07-28
- Source: [`ROBO_TASK.pdf`](../ROBO_TASK.pdf)
- Overturns the "not modelled" disposition recorded in
  [`evidence/source-diagram/NOTES.md`](../evidence/source-diagram/NOTES.md) and
  `DESIGN.md`. Extends [ADR 0010](0010-supplied-diagram-geometry.md); does not
  disturb [ADR 0017](0017-relocate-p-to-diagram-east-corner.md).

## Context

The drawing puts an unlabelled block on the next street's east wall, beside B.
It was measured, annotated on the overlay as "unlabelled stub, not modelled",
and excluded with a recorded reason:

> Unlabelled and unmentioned in the prose; a doorway, step, or bench. Modelling
> it would invent a requirement.

That reasoning was sound on its own terms. The decision changes because
fidelity to the drawn scene was preferred over minimality — the drawn block is
scene geometry, drawn in the same wall style as every other wall, and leaving it
out meant the scene and its own source evidence disagreed about what the street
contains.

### What transfers, and what does not

| Quantity | Measured | Transfers as |
|---|---|---|
| Stub protrusion | 166 px of a 358 px channel | **0.4637 of the street** → 2.78 m |
| Stub extent along street | 63 px, at its own 2.635 : 1 aspect | **1.06 m** |
| B lateral position | label centres at 0.7989 of the channel | **x = 16.79 m** |
| B south of the stub | 36 px | **1.05 m** |
| Position along the street | 16.0 m south of the north face | **Does not transfer** |

The along-street position cannot be read off. The drawing's own ratios give a
43.4 m corridor and B at 20.3 m against the scene's chosen 12.0 m and 8.0 m, so
it is pinned instead to the one relationship the drawing does fix: B stands
immediately south of the stub.

Protrusion transfers as a *fraction of the street*, not through the `m` arrow.
The two disagree — 2.78 m against 4.84 m — because the scene's 6.0 m street does
not match the drawn `street/m` ratio of 1.74. The fraction is the meaningful
invariant because it is what decides whether A can still get past.

### B is behind the stub, which a line-arc-line route cannot reach

B's label centres at 0.7989 across the channel, which is **inside the stub's
x-shadow of 15.22–18.0**, and one stub-length south of it. B therefore stands in
the pocket the stub makes against the east wall, not out in the lane.

Moving B into the open lane would have been the cheap resolution, and it would
have been the same inconsistency twice over: applying "transfer as a fraction of
the street" to the stub's depth and then declining to apply it to B.

## Decision

Model the stub as a collider, leave B where the drawing puts it, and give the
route the two pieces it needs to arrive there.

- `EastWallStub` joins the building footprints and is excluded from `is_clear`,
  so it is solid to the route validator and to physics like every other wall.
- **A drives the lane the stub leaves, not the street's geometric centre.**
  `street_drive_center_x_m` is the middle of what is actually clear. The old
  centreline at `x = 15.0` is 0.218 m from the stub's west face, inside A's own
  0.225 m half-width — the robot would clip the wall before the trajectory
  margin was even considered.
- `DeliveryTrajectory` gains a left-handed **delivery arc** and a short
  **delivery run** east to B, tangent at both joins exactly as the first turn
  is, so heading stays continuous across all five pieces.

## Consequences

- **Yaw is no longer monotonic, and `yaw_range` had to stop assuming it was.**
  Yaw fell across the whole old route, so `yaw_range` read the interval's two
  endpoints and was right. The delivery turn is left-handed and yaw rises
  through it, so an interval spanning both turns has its extremes in the
  interior. Endpoint sampling would have bounded the camera over a narrower cone
  than it actually traverses — a silent false pass in the visibility gate, which
  is the one place in this project a silent weakening matters most. `yaw_range`
  now takes extremes piece by piece, which is exact because yaw stays monotonic
  *within* each piece. Both properties are pinned by tests.
- **The arc enclosure generalised.** `_camera_source_vertices` was written
  against the single right-handed turn. It now takes the centre, radius,
  direction and start along the route from whichever arc it is enclosing.
- The route grows from 23.851 m to **24.601 m**: approach 11.449, arc 3.390,
  lane 5.436, delivery arc 3.142, delivery run 1.184.
- **Every figure in `evidence/live-demo/` is superseded** — length, sim span,
  update count, and the gate crossing times behind the speed figures. The
  demonstration was re-run and re-recorded in the same change.
- Manifest schema moves to **0.4.0** for the five new trajectory fields. They
  default to zero, so the dataclass still describes a line-arc-line route and an
  older manifest still parses.
- The occlusion certificate is unaffected in outcome: still passing on all three
  profiles, still with `EastBuilding` as the only blocking prim, now over five
  intervals instead of three because there are five pieces to cover.
- No change to the speed policy, the enforcement gates, the fiducials, or the
  estimator. All four gates sit on the approach, which keeps its heading and its
  `approach_s_at_x` mapping.

## Alternatives considered

- **Keep it unmodelled.** The prior decision. Rejected here in favour of
  fidelity, but it was not wrong: the block is unlabelled and the prose never
  mentions it, so modelling it does assert something the source does not state.
  What tips it is that the drawing states the block's *presence* as plainly as
  it states the walls', and the scene already commits to the drawing's topology.
- **Model the stub but move B into the lane.** Rejected as internally
  inconsistent: it takes the drawing's word for the stub's depth and ignores it
  for B's position, when both are the same kind of measurement off the same
  drawing. It also dodges the interesting part — the pocket is *why* the route
  needs a delivery turn.
- **Model it shallower so the old route survives.** Rejected: the depth is the
  one thing about the stub the drawing does fix well, and shrinking it to avoid
  touching the trajectory would invent a dimension to protect code.
- **Add the stub to `occluders()`.** Rejected. It cannot lie between A's route
  and P at the north-east corner, so including it would contradict that
  function's stated rationale. The composed-mesh audit discovers it from the
  stage by collision schema regardless, which is the same treatment the north
  building gets.
