# ADR 0016: Move the strict speed zone to 4.0 m clear width

- Status: Accepted
- Date: 2026-07-27
- Extends: [ADR 0007](0007-speed-policy-and-violation.md), which remains
  accepted and is not superseded. ADR 0007 requires the owner to approve policy
  values; that approval was given for the value recorded here.

## Context

[ADR 0007](0007-speed-policy-and-violation.md) makes the width-to-limit mapping
an explicit demonstration policy and requires a violation to be confirmed over
a configured number of consecutive measurements — currently two — before it is
emitted. That debounce is deliberate: it stops a single noisy frame becoming an
enforcement event.

The original strict tier began at 3.5 m clear width. On the nominal taper
(`m = 6.0`, `n = 3.0`, width falling 0.25 m per metre) that boundary sits at
`x = 10`, so exactly **one** enforcement gate fell inside the strict zone.

A confirmation rule needing two consecutive over-limit measurements cannot be
satisfied by one gate. The consequence was not a marginal result, it was a
structural impossibility: a robot speeding only through the corner could be
*evaluated* at gate 10 and never *confirmed*, so a sustained 1.0 m/s run past
the narrowest point reported nothing at all. The scenario's central claim —
that the corridor narrows and the rule tightens — had no reachable
demonstration.

The interview objective this serves is that changing the corridor width
visibly changes both geometry and policy. A rule that can never fire does not
demonstrate that.

## Decision

Move the strict tier's boundary from 3.5 m to **4.0 m** clear width. The full
demonstration policy becomes:

| Maximum clear width | Limit |
|---:|---:|
| 4.0 m | 0.8 m/s |
| 5.0 m | 1.2 m/s |
| unbounded | 1.5 m/s |

On the nominal taper 4.0 m corresponds to `x = 8`, so gates 8.0 and 10.0 both
fall inside the strict zone and the two-estimate confirmation becomes
satisfiable from camera evidence.

Adopt **gate-derived speed error** as the acceptance criterion for the
estimator, with per-frame station error retained as a secondary health check.
Speed between two surveyed gates is what the observer actually delivers and
what this policy is written about; gate crossing times are interpolated between
observations, so per-frame station noise partly averages out and the two
quantities are not interchangeable.

## Consequences

> **Correction, 2026-07-27.** The measured figures below cited the live run
> recorded before the R17 plate relocation in `a101b28`. `cdb6f79` re-recorded
> that run on the corrected geometry but did not carry the new numbers here.
> Exceedance 0.194 → 0.195 m/s, gate 8 0.963 → 0.967 m/s, 2σ lower bound
> 0.935 → 0.941 m/s. The decision, its boundary and its rationale are
> unchanged; only the evidence citation moved. Following ADR 0014's precedent,
> the correction is marked rather than applied silently.

- A corner-confined violation is confirmable. Measured live at a constant
  1.0 m/s: compliant at gates 4.0 and 6.0 under the 1.2 m/s tier, over-limit at
  gates 8.0 and 10.0, exactly one violation with 0.195 m/s exceedance. See the
  [live evidence](../evidence/live-demo/NOTES.md).
- The conservative debounce is unchanged. Widening the zone was chosen
  specifically so the confirmation rule did not have to be weakened.
- **Two gates is the minimum that satisfies the rule, so there is no spare.**
  Both gate 8.0 and gate 10.0 must be measured and over-limit or the run
  produces no violation at all — a silent absence, not a wrong answer. The
  margin when they are measured is comfortable (gate 8 read 0.967 m/s against
  0.80 m/s, 2σ lower bound 0.941 m/s), so the exposure is a missed measurement
  rather than a borderline one. Accepted as a documented risk; the mitigation
  is rehearsal, not a second policy move.
- The compliant stretch of the demonstration shortens. Gates 4.0 and 6.0 remain
  under the 1.2 m/s tier, which is enough to show a legal approach before the
  rule tightens.
- The other authored profiles have no gate in the strict zone: on
  `wide_corner_m6_n4_5` the narrowest gate is 4.75 m (1.2 m/s tier) and
  `uniform_m6_n6` is 1.5 m/s throughout. Neither can produce a violation at
  1.0 m/s. That is the intended policy story rather than a defect, but it means
  a live variant switch shows a green readout and needs its explanation ready.
- Metric values remain a demonstration choice, not surveyed law, exactly as
  ADR 0007 and ADR 0010 record.

## Alternatives rejected

- **Weaken the confirmation to one measurement.** Would make the corner rule
  fire, at the cost of the debounce that keeps a single noisy estimate from
  becoming an enforcement event. That trade runs against ADR 0007's core
  decision.
- **Add another enforcement gate inside the old 3.5 m zone.** The camera
  coverage ends around x = 10.8, which brackets gate 10.0 but leaves no room to
  bracket a further gate past it. A gate whose crossing cannot be bracketed
  produces no measurement, so this would have added a station that looked like
  coverage and was not.
- **Leave the boundary and demonstrate speeding only on the wide approach.**
  Abandons the scenario's central claim that the narrowing corner is where the
  rule bites.
