# ADR 0007: Configure speed policy and emit conservative violations

- Status: Accepted for demonstration policy
- Date: 2026-07-24

## Context

Road width alone does not imply a legal speed limit, and noisy estimates should
not produce single-frame enforcement events.

## Decision

Treat the width-to-limit mapping as an explicit demonstration policy in the
scenario manifest. Emit a violation only from a valid estimate whose conservative
confidence comparison exceeds the limit for the configured confirmation rule.

## Consequences

- The demo does not misrepresent an invented policy as law.
- Every event records the estimate, width, limit, uncertainty, gates, profile,
  and confirmation duration.
- The owner must approve policy values before implementation can be complete.

## Alternatives considered

- Hard-coded formula from width: rejected as unjustified.
- `speed > limit` on one frame: rejected due to noise and false positives.
- Latched violation topic: rejected because late subscribers could interpret an
  old event as current.
