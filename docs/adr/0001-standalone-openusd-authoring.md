# ADR 0001: Author the base scene with standalone OpenUSD

- Status: Accepted
- Date: 2026-07-24

## Context

The scene must be reproducible, testable without a renderer, and available before
Isaac Sim hardware/API qualification. Manual GUI authoring is difficult to review
and repeat.

## Decision

Author the Phase 1 stage with pip `usd-core` and `pxr` only. Emit human-readable
USDA and validate the composed stage programmatically. Isaac Sim later loads this
artifact as a consumer.

## Consequences

- Scene generation and geometry tests run without Isaac Sim.
- GUI edits cannot become the only copy of a scene change.
- Isaac-specific schemas are excluded until the installed release is known.
- Imaging is unavailable in `usd-core`; rendering remains an Isaac concern.

## Alternatives considered

- Isaac GUI as source of truth: rejected for reproducibility and current hardware
  dependency.
- Isaac standalone Python authoring: deferred because it couples the generator to
  release-specific extension namespaces.
