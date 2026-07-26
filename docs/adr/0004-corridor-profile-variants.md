# ADR 0004: Represent finite corridor profiles as a USD variant set

- Status: Accepted
- Date: 2026-07-24

## Context

The requested widths `m` and `n` are numeric, while a USD variant selects among
finite named authored alternatives. Geometry derived jointly from both endpoints
cannot be recomputed automatically by changing arbitrary attributes.

## Decision

Create one `corridorProfile` variant set. Each named variant contains a complete
`(m,n)` pair, dependent geometry, and numeric metadata. CLI `--m`/`--n` defines
and selects the nominal profile; additional profiles come from configuration.

## Consequences

- Profiles can be switched visibly in the USD/Isaac UI.
- Arbitrary live sliders require regeneration or a future procedural adapter.
- Physics may need pause/reset after selection changes; this will be verified
  against the installed Isaac release.

## Alternatives considered

- Independent `m` and `n` variant sets: rejected because both would author
  conflicting opinions on the same mesh points.
- Custom numeric attributes only: rejected because USD does not procedurally
  rebuild dependent geometry from them.
