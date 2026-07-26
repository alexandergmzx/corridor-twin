# ADR 0005: Verify occlusion continuously and audit composed USD geometry

- Status: Accepted
- Date: 2026-07-24

## Context

A dense set of ray samples can miss a visible interval and does not prove the
claim that P is hidden from every point on A's path. Checking generator inputs
alone can repeat a generator error.

## Decision

Build a continuous horizontal shadow-volume certificate for every path segment
and P's conservative bounding volume, with a vertical interval proof. Independently
read composed world-space USD meshes and run diagnostic segment/triangle tests.

## Consequences

- Occlusion becomes a first-class pass/fail artifact.
- The path and P must have explicit conservative bounds.
- Tests must include visible negative controls.
- Frustum exclusion may be reported but cannot substitute for wall occlusion.

## Alternatives considered

- Screenshot inspection: rejected as subjective.
- Fixed-step ray sampling only: rejected as non-continuous evidence.
- Relying on Isaac/PhysX raycasts: rejected for Phase 1 dependency and because it
  would not independently validate authored geometry.
