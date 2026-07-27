# ADR 0011: Keep "A cannot see P" a geometric gate over a continuous turn

- Status: Accepted
- Date: 2026-07-27
- Extends: [ADR 0005](0005-continuous-occlusion-verification.md), which remains
  accepted and is not superseded.

## Context

The supplied task states that *the robot cannot see the traffic police, but the
police can read the data from the robot*. Two readings were proposed. One treats
this as a statement about software information flow: A's controller does not
consume anything about P. The other treats it as a statement about what A's
camera can image.

The weaker reading is not sufficient. P could stand in plain view and fill A's
pixels while A's code ignored them; the requirement would be reported as met and
the scene would contradict its own description. The direction of the sentence
also matters: it says A cannot see P. It does not say P sees A. P reading A's
camera feed is a network relationship, not a sightline.

[ADR 0005](0005-continuous-occlusion-verification.md) established the proof, but
against the previous geometry: a symmetric corridor and a route modelled as a
polyline with one heading per segment. The reconciled scene in
[ADR 0010](0010-supplied-diagram-geometry.md) has an asymmetric corridor, a
corner mass, and a real turn.

## Decision

Keep four concepts distinct in code, tests, documentation, and the demo:

| Concept | Question | Directional? |
|---|---|---|
| Physical line of sight | Does an opaque wall intersect the segment between A's camera and P's body? | No; normally reciprocal |
| A-camera visibility | Is any part of P inside the frustum *and* unoccluded? | Yes |
| A software awareness | Does A detect, model, or react to P? | Yes |
| P data access | Does P subscribe to A's Image, CameraInfo, and the survey? | Yes |

`camera_visible` must be false over every trajectory interval. That is the
binding gate. The software-awareness rule is enforced separately by a source
contract and is additive; it can never stand in for the gate.

The default scene must additionally satisfy the stronger, reciprocal claim that
an opaque wall does the hiding. The certificate reports
`direct_line_of_sight_blocked` and frustum exclusion as separate fields, and
pursues a wall witness even where P is already off-screen. Settling for
off-screen is a last resort, never a shortcut, and an off-screen P is never
relabelled as wall-occluded.

Extend the proof to the reconciled scene:

- Consume the shared continuous trajectory, sweeping the turn as a yaw *range*
  per interval rather than one heading per segment.
- Cover P's full body volume, subdividing that volume as well as the route.
- Solve witness planes in closed form rather than sampling them.
- Admit witness planes of constant X *and* of constant Y.
- Discover audited meshes from the composed stage by applied collision schema,
  so renaming or adding a building cannot silently shrink the audit.

## Consequences

- A visible negative control remains mandatory. Without it a pass means nothing.
- Three findings during implementation showed why each extension was needed, and
  none of them were visible to the previous method:
  - A sampled witness search stepped over a feasible window only 8 mm wide near
    the corridor entry, reporting a false positive for visibility.
  - No single plane can contain rays to opposite corners of P inside one 0.5 m
    wall, even though the wall blocks each of them at its own depth. Subdividing
    P's volume was required.
  - Where A draws level with P no plane of constant X separates them at all, so
    the crosswise witness is not an optimisation but a necessity.
- The certificate records the blocking prim and the nearest blocking distance,
  so the interview overlay quotes measured evidence rather than a claim.

## Alternatives considered

- **Reduce the requirement to software information flow.** Rejected: it would
  pass with P in open view.
- **Accept frustum exclusion alone.** Rejected: it is directional and proves
  nothing about a reciprocal sightline, so it is far harder to explain and much
  easier to break by re-aiming the camera.
- **Move P to wherever the drawing appears to place it and accept visibility.**
  Rejected: the written requirement outranks a label position in an unscaled
  drawing.
