# ADR 0006: Generate one scenario manifest for all consumers

- Status: Accepted
- Date: 2026-07-24

## Context

Duplicating marker poses, widths, camera models, and policies between scene and
observer configurations invites silent disagreement.

## Decision

Generate `corridor.manifest.json` beside the USDA. It contains all profile,
survey, camera, path, P-bound, and speed-policy data needed by validators,
synthetic generation, and the observer.

## Consequences

- All consumers can check a schema version and selected profile.
- Variant selection and observer profile must be synchronized explicitly until a
  future Isaac adapter coordinates them.
- Tests can compare manifest coordinates with composed USD transforms.

## Alternatives considered

- Copy YAML into each package: rejected due to drift.
- Read every runtime value directly from USD inside the observer: rejected because
  the production observer should not depend on a USD stage or simulator.
