# ADR 0017: Relocate P to the junction's east corner, behind the east wall

- Status: Accepted
- Date: 2026-07-28
- Source: [`ROBO_TASK.pdf`](../ROBO_TASK.pdf)
- Supersedes **one rejected alternative** in
  [ADR 0011](0011-visibility-semantics.md) — "move P to wherever the drawing
  appears to place it and accept visibility" — on new grounds, not because that
  rejection was wrong. ADR 0011 otherwise remains accepted in full. Extends
  [ADR 0010](0010-supplied-diagram-geometry.md).

## Context

**The supplied source contradicts itself, and the text outranks the drawing.**
The prose states plainly that *the robot cannot see the traffic police*. The
drawing places P's label in the open street channel at corridor level, where
nothing stands between A and P and A would see it. Both cannot be satisfied
literally. Where an unscaled drawing and the written requirement disagree, the
written requirement wins — it is unambiguous, while the drawing carries no
scale bar, no dimensions, and widths given only as the symbols `m` and `n`.
[ADR 0010](0010-supplied-diagram-geometry.md) already draws that line for every
metric quantity in the scene; this is the same line applied to an actor.

The diagram places P at the **east** side of the junction: its label sits past
the corner mass, between the next street's two walls, level with the corridor.
The scene placed P on the **west** side, south of the corridor and behind the
corner mass. The sides did not match, and nothing but a YAML comment recorded
why.

[ADR 0011](0011-visibility-semantics.md) considered this and rejected "move P
to wherever the drawing appears to place it and accept visibility", on exactly
the grounds above. **That rejection was correct and remains correct.** Placing
P at the label's literal position means A sees P, and no amount of drawing
fidelity buys back the requirement the scene exists to demonstrate.

What ADR 0011 did not consider is that the alternative it rejected is not the
only way to honour the drawing. Its framing bundled two things — the drawing's
*side* and the drawing's *exact standoff* — and rejected them together because
the second one costs visibility. They separate. The drawing states the side
unambiguously; it fixes the standoff no better than it fixes any other length.
Taking the side while choosing the standoff, as ADR 0010 already does for every
distance in the scene, puts P on the far side of the next street's east wall:
the diagram's side, at no visibility cost.

This ADR therefore supersedes that alternative on new grounds rather than
reversing a mistake. ADR 0011 rejected a proposal that paid for the drawing
with the requirement. This one does not pay.

Measured before deciding, on all three profiles:

| P placement | Occluders | Certificate |
|---|---|---|
| East corner | existing list | **fails** — 2 camera-visible intervals, no wall witness |
| East corner | existing list **+ EastBuilding** | **passes** — line of sight blocked everywhere |

The east wall was excluded from `occluders()` by an argument that held only for
the west placement: that the north and east buildings sit on the far side of
both A's route and P. With P east of the junction that is false, and the
exclusion is what made the first row fail.

## Decision

Place P at the junction's east corner, on the far side of the next street's
east wall, and add that wall to the analytic occluder list.

The placement stays expressed as offsets from wall faces rather than absolute
coordinates, per ADR 0010:

- `east_offset_m` — clear air east of `EastBuilding`'s **outer** face.
- `north_offset_m` — south from the north wall's inner face at `m/2`.

`validate_layout` is re-derived accordingly: P must clear the east wall's outer
face by its margin, and must stay within that wall's north–south span so the
wall covers its whole body rather than only part of it.

The scenario's meaning is unchanged. P still stands at the corner watching for
speed violations, still cannot be seen by A, and still receives A's camera feed
over ROS 2 — a network relationship, not a sightline, exactly as ADR 0011
insists.

## Consequences

- **The east wall becomes load-bearing.** It is now the only blocking prim in
  the default certificate, on every profile, with nearest blocking distance
  4.02–4.58 m. `test_removing_the_east_wall_fails_the_certificate` deletes it
  from the slab list and requires the proof to fail, so the exclusion cannot
  return unnoticed.
- **P no longer moves between the configured profiles.** Its Y comes off the
  north face at `m/2`, and all three configured profiles share `m = 6.0`. The
  old anchor used the south face, which varies with both `m` and `n`. The
  ADR 0010 principle is intact — the placement is derived, not frozen — but the
  test asserting observable movement between the configured profiles was
  asserting a property of the old anchor and now varies `m` instead.
- **The proof got stronger, not just cheaper.** The east wall is a single plane
  of constant X, and it separates P from *every point of the route at once*:
  P on one side, all of A's travel on the other. The certificate now covers
  each profile in 3 intervals rather than 70, and the argument fits on a
  whiteboard — one plane, two sides, no case analysis.

  The west placement needed the general machinery because no single plane
  worked: where A drew level with P, no plane of constant X separated them at
  all, which is why ADR 0011 called the crosswise witness "not an optimisation
  but a necessity". That was a true statement about that placement. The default
  scene simply no longer *requires* the general machinery, because a simpler
  argument is available to it. A proof that needs less structure to hold is a
  better proof, and an easier one to defend under questioning.

- **The general machinery keeps earning its keep, and keeps its regression.**
  The constant-Y witness is still correct and still necessary for curved-path
  placements where no single plane separates the route — it is only the default
  scene that has stopped needing it. Rather than let that coverage lapse
  because the headline case got easier,
  `test_a_crosswise_witness_is_still_required` drives the superseded west
  placement against the current occluders and requires both orientations.
- No change to the speed policy, the enforcement gates, the fiducials, or the
  estimator. P's position is not an observer input.

## Alternatives considered

- **Keep the west placement and document the deviation.** Rejected. The side is
  one of the few things the drawing states unambiguously, so departing from it
  is not a metric-scale choice of the kind ADR 0010 legitimises. Documenting a
  contradiction is weaker than removing one, and here removal is free: the east
  placement satisfies the same gate with a simpler proof.
- **Place P literally where the label sits, in the street channel.** Rejected,
  for exactly the reason ADR 0011 gave: at corridor level between the two
  street walls nothing intervenes, and A would see P. This is the case where
  the drawing and the written requirement genuinely conflict, and the text
  outranks. This ADR changes which side P stands on; it does not touch that
  principle.
- **Add a fourth corridor profile with a different `m`, to restore a visible
  "change the corridor and P follows" moment.** Rejected as not worth its
  price. The three configured profiles are deliberately an `n` sweep at a fixed
  `m = 6.0`, so P is genuinely in the same place in all of them; a fourth
  member varying `m` would change what the variant set is for, add a USD
  variant, and lengthen every one of the fourteen profile-iterating tests. The
  claim is carried instead by
  `test_p_is_derived_from_the_geometry_and_not_frozen`, which builds `m = 8.0`
  on demand and requires P to move exactly 1.0 m north, and the demonstration
  can still show it live through `scene.build --m 8.0` and `CORRIDOR_PROFILE`.
- **Move P east but leave `occluders()` alone and rely on the composed-mesh
  audit.** Rejected. The mesh raycast would still pass, since it discovers
  prims from the stage, but the analytic certificate — the half that reasons
  continuously over the whole route rather than sampling rays — would have no
  witness. Two independent halves are the point; degrading one to a formality
  is not.
- **Anchor P's Y to the corner mass so it varies with `n` as well.** Rejected:
  P is nowhere near the corner mass under this placement, and anchoring a body
  to a surface it has no relationship with would manufacture profile
  sensitivity rather than derive it.
