# ADR 0010: Reconcile scenario geometry with the supplied diagram

- Status: Accepted
- Date: 2026-07-27
- Source: [`ROBO_TASK.pdf`](../ROBO_TASK.pdf)

## Context

Until now every geometric number carried the marker
`provisional_geometry_pending_diagram_reconciliation`, and the design stated
that A, B, P, and the path were not frozen until the supplied diagram arrived.
The diagram has now been supplied.

It is a plan-view drawing with no scale bar, no dimension text, and widths
labelled only with the symbols `m` and `n`. It therefore carries strong
evidence about topology and none about metric scale. Treating measured pixel
ratios as recovered survey values would dress a drawing assumption up as a
requirement.

The drawing also places the corridor's two faces asymmetrically: the upper face
is straight across the full length and only the lower face slopes. The previous
model tapered both faces symmetrically about `y = 0`.

## Decision

Separate what the source fixes from what the project chooses.

**Binding, taken from the source:**

- `m` is the wider entry and `n` the narrower corner, so `m >= n`.
- One corridor face is straight and the other carries the whole taper. The
  north face is held at `+m/2` and the south face is `north - width(station)`.
- A turns onto a perpendicular next street to reach B, so the junction, its
  flanking walls, and a real corner mass are authored.
- B stands along that next street and P stands at its corner.

**Project choices, recorded as such:** corridor length, next-street width and
length, turn radius, and B's distance along the street. These stay at the
existing compact values so the marker budget, VRAM envelope, and validated
15 Hz camera contract are not disturbed by a geometry change.

Provenance is published in the config and the manifest as
`topology: reconciled_with_supplied_diagram` and
`metric_scale: demo_assumption`.

P's placement is expressed as offsets from the occluding wall faces rather than
as absolute coordinates, so selecting a different `(m,n)` profile moves P with
the geometry instead of stranding it inside a wall or in the road.

One module owns the taper. `geometry.corridor_faces` is the single source of
truth for the wall footprints, the marker survey, the delivery trajectory, and
the visibility witnesses.

## Consequences

- The corridor centreline is no longer straight; it drifts toward the fixed
  north face. Anything that assumed `y = 0` had to follow it, including the
  synthetic camera.
- Station remains world X, which is also how markers are surveyed. Because the
  path now runs at an angle to X, an X displacement is shorter than the
  distance travelled, and speed must be converted before it is reported. See
  [ADR 0002](0002-camera-only-speed-observation.md).
- The corridor's south wall and the next street's west wall are authored as two
  overlapping convex prims rather than one L-shaped prim, because the walls
  carry `convexHull` collision approximations and an L would silently fill the
  junction A must drive through.
- The `corridorProfile` variant set moved from `/World/Environment/Corridor` to
  `/World`, because a variant only contributes opinions inside its own prim's
  namespace and A, P, and the path now depend on the profile.
- Building prim names changed: `LeftBuilding` became `NorthBuilding`,
  `RightBuilding` became `SouthBuilding`, and `CornerBuilding` and
  `EastBuilding` are new. The old names described a symmetry that no longer
  exists.

## Alternatives considered

- **Scale the drawing by measured pixel ratios.** Rejected: the drawing has no
  scale bar and includes illustrative extrusion, so the resulting lengths would
  be presented with a precision the source does not support.
- **Keep the symmetric taper and change only the actors.** Rejected: the
  drawing's asymmetry is one of the few things it states unambiguously.
- **Freeze P at absolute coordinates.** Rejected: correct for one profile only,
  and silently wrong for the others.
