# ADR 0012: Enclose curved camera motion before certifying visibility

- Status: Accepted
- Date: 2026-07-27
- Extends: [ADR 0005](0005-continuous-occlusion-verification.md) and
  [ADR 0011](0011-visibility-semantics.md)

## Context

The reconciled delivery route contains a circular turn. The first implementation
evaluated a trajectory interval using only its two camera positions. That is
exact on a straight route piece because every intervening source lies on the
endpoint segment. It is not exact on a turn: the segment is the arc's chord, and
the real camera can lie well outside it.

A review fixture made the gap concrete. Both endpoint-to-target rays crossed a
short wall and the checker returned `passed`, while every mid-arc ray crossed
above that wall. Dense audit sampling happened to find no leak in the real scene,
but it could not turn the endpoint argument into a continuous proof.

The same review found an evidence-schema defect. Constant-X and constant-Y
witnesses were both serialized as `witness_x_m`, so a correct Y coordinate was
published under the wrong axis name.

## Decision

Use a convex source enclosure that contains every camera position in the route
interval before attempting either a wall or frustum proof.

| Route piece | Enclosure |
|---|---|
| Straight approach or departure | The exact segment whose vertices are its endpoints |
| Circular turn | The axis-aligned rectangle determined analytically by the interval's endpoint angles and every cardinal angle it contains |

The circular extrema are closed form; they are not samples. The full arc lies in
the resulting rectangle. Wall witnesses enumerate its vertices against the
corners of the current P sub-volume. At a separating plane the ray crossing is
linear-fractional in the source and target coordinates with a fixed-sign
denominator, so extrema over these convex boxes occur at vertices.

Frustum exclusion uses the same position enclosure and the interval's yaw range.
The Cartesian product includes combinations the real correlated trajectory does
not take, which is conservative: it can force extra subdivision but cannot prove
an occlusion that the real motion lacks.

Serialize a witness as `witness_axis` plus `witness_coordinate_m`. The axis is
part of the evidence, not an implementation detail.

## Consequences

- The curved-source fixture is a mandatory negative regression and no longer
  passes the certificate.
- The nominal scene remains certified with 78 interval/sub-volume pairs: 50
  constant-X and 28 constant-Y witnesses. Its independent audit remains 204 rays
  with zero failures and a 3.116 m nearest blocking surface.
- The wide and uniform variants also remain certified independently.
- A conservative rectangle may require more subdivision than a tighter curved
  enclosure. The scene is small, and proof integrity is worth that bounded CPU
  cost.
- ADR 0010's phrase “the centreline is no longer straight” was imprecise. The
  one-sided linear taper produces a straight centreline rotated away from world
  X. This is a terminology correction, not a geometry decision change.

## Alternatives considered

- **Keep endpoints and rely on dense raycasts.** Rejected: sampling can support
  an audit but cannot establish coverage between samples.
- **Subdivide until the chord is visually close to the arc.** Rejected: without
  a quantified curvature bound, every finite chord still omits real sources.
- **Sample extra points on each arc interval.** Rejected for the same reason;
  more samples are not an enclosure.
- **Use a tighter sector or tangent polygon.** Viable later, but unnecessary for
  this small scene while the exact axis-aligned bounds certify all profiles.
