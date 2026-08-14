# ADR 0038: The speed policy is pinned to A's measured profile, not to the scale factor

- **Status:** Accepted
- **Date:** 2026-08-14
- **Completes:** [ADR 0023](0023-governed-nav2-live-slam.md), which re-pins the
  speed policy to robot scale and left the table as
  `[to pin after first profile run]`. The profile has been run; this is the
  pin. **Owner-ratified**, as [ADR 0007](0007-speed-policy-and-violation.md)
  requires for any policy change.
- **Does not touch** [ADR 0016](0016-corner-enforcement-policy-boundary.md).
  The zone boundaries are unchanged; only the three speeds are set.

## Context

The demonstration's second headline claim is that the corridor narrows, the
local limit tightens, and A's own driving crosses it. The as-run scenario could
not produce that, and the reason is arithmetic rather than tuning.

`tools/scale_scenario.py` deliberately does not scale `limit_mps`: speeds are a
policy, not a dimension, and scaling them would be inventing one. So the
robot-scale scenario carried v1's 0.8 / 1.2 / 1.5 m/s against robot-scale
widths. Scaling them by 0.30 instead would give 0.24 / 0.36 / 0.45.

**Robot1 cannot reach any of those numbers.** Nav2's `max_vel_x` is 0.22 m/s
and the governor's ceiling is 0.35. Measured over six delivery runs on
2026-08-14 from `/sim/ground_truth`, secant speed over a ±0.30 m window of
travel:

| gate | clear width | mean | slowest run | fastest run |
|---|---|---|---|---|
| 0.6 m | 1.65 m | 0.1967 | 0.1731 | 0.2066 |
| 1.2 m | 1.50 m | 0.1960 | 0.1891 | 0.2023 |
| 1.8 m | 1.35 m | 0.1689 | 0.1399 | 0.1818 |
| 2.4 m | 1.20 m | 0.1285 | 0.1125 | 0.1420 |
| 3.0 m | 1.05 m | 0.0807 | 0.0555 | 0.1028 |

A's whole measured band is 0.056–0.207 m/s, entirely below every candidate
tier. A geometrically scaled policy is one no fleet robot can ever violate.

`tools/measure_speed_profile.py` produced this;
`out/evidence/speed-profile/measured-profile.json` is the artifact. The profile
is **ground truth and an evaluation input** (invariant 1): it derives the policy
the observer is later judged against, and it never reaches the observer.

Two facts fell out of measuring it that are recorded because they cost time
otherwise. Isaac's `/sim/ground_truth` publishes pose but leaves `twist`
identically zero, so speed truth must be differentiated from position — the
`twist` column of the artifact is all zeros and is not a measurement. And the
profile is not flat: A decelerates from 0.197 to 0.081 m/s between the first
gate and the last, which is Nav2's controller slowing into the goal, so "A's
cruise speed" is not one number.

## Decision

**Widest tier first: 0.30 / 0.25 / 0.04 m/s.** Zone thresholds untouched.

```
wide     1.5 m <  width           0.30 m/s
mid      1.2 m <  width <= 1.5    0.25 m/s
strict            width <= 1.2    0.04 m/s
```

One rule, applied three times: **a permissive tier sits above the fastest
measurement in its zone, a strict tier below the slowest.**

| tier | measured extreme in its zone | limit | margin |
|---|---|---|---|
| wide | 0.2066 max | 0.30 | +45% |
| mid | 0.2023 max | 0.25 | +24% |
| strict | 0.0555 **min** | 0.04 | −39% |

The margins are against the *extreme of six runs*, not the mean, so the
demonstration does not depend on which run is recorded.

The result, from the shipped `ViolationDetector` rather than from this table:
compliant at gates 0.6, 1.2 and 1.8; over the limit at 2.4 and 3.0; **exactly
one violation, confirmed at gate 3.0**, on the mean, slowest and fastest cases
alike. A compliant stretch and a speeding episode in one run, which is what
`CLAUDE.md`'s definition of done asks for.

The pin lives in one constant, `PINNED_LIMITS_MPS` in `tools/scale_scenario.py`,
is printed by the generator, is stamped into the derived scenario's header, and
is asserted against the manifest the observer actually loads.

## Why not 0.05 for the strict tier

0.05 m/s is `CREEP_SPEED_MPS`, the governor's clamp on A's plane (ADRs 0033 and
0034). Reusing it as an enforcement limit on P's plane would put one number in
two unrelated meanings on two isolated planes, and the first reader to notice
would reasonably assume one derives from the other. It also places a measured
quantity in exact boundary equality with a commanded one, which is the worst
place for a `<=` to live.

The margin argument points the same way: against the slowest run's 0.0555 m/s
at gate 3.0, a 0.05 limit leaves 11% and 0.04 leaves 39%. A test asserts the
strict limit is not the creep clamp.

## Why not widening the strict zone

The alternative considered — moving the strict boundary to 1.35 m so gate 1.8
joins the zone, with a 0.10 m/s strict limit — produces a better-shaped
episode: it would *end* at gate 3.0 because A had slowed below the limit,
rather than running to the end of the corridor, and a 3:1 tier ratio reads more
credibly than 7.5:1.

It was rejected on two counts. It supersedes ADR 0016 on the day of delivery,
for a shape improvement rather than a correctness one. And it is less stable:
gate 3.0's measured band straddles 0.10, so the episode would be two gates long
on five runs and three on the sixth, and gate 2.4's margin would fall to 29%.

**Closure by deceleration is recorded as a future refinement**, with this
paragraph as its argument, to be revisited on evidence rather than on ship day.

## A defect this uncovered

Verifying the two-gate floor against the shipped code — rather than against the
table above — found that **it had not held since ADR 0030, and no test saw it**.

`MarkerMap.width_at(2.4)` returns `1.2000000000000002`, so a bare
`width <= 1.2` placed the gate in the permissive zone. The strict zone held one
gate; `consecutive_estimates` is 2; **a corner-confined violation could never
have been confirmed on the as-run scenario, under this policy or any other.**

It is scale-dependent, which is why it hid: at v1's authored metres the same
expression is exactly 4.0, so ADR 0016's arithmetic was right when written and
ADR 0030's 0.30 scaling broke it silently while every v1 test stayed green.

Fixed in its own commit as a one-nanometre tolerance on the boundary
comparison, with a regression test at both scales. The threshold itself does
not move — that is ADR 0016's decision.

## Consequences

**The policy is a demonstration choice and stays labelled one.** `status:
demonstration_only` is unchanged. The strict tier is 1/7.5 of the wide tier
where v1's was 1/1.9, and that steepness is a consequence of pinning to a
0.22 m/s vehicle in a corridor authored for a road one: the tiers must
bracket a band 0.15 m/s wide.

**The limits reach P's plane only.** A's stack reads Nav2 parameters and the
governor's constants; nothing on A's plane loads `speed_policy`. The observer
and `enforcement_view` both take it from the same manifest field, so the
rendered limit and the enforced limit cannot disagree.

**An episode open at route end emits nothing further, by design.** Under this
pin A is still over the strict limit at the final gate, so the episode never
closes. ADR 0014 has no close event: the episode *is* the one emitted
`Violation`, and closure exists only to rearm the detector. Asserted rather
than assumed, because "one event per episode" and "an episode that never
closes" sound like they conflict.

**Re-pinning is a regeneration, not an edit.** Change `PINNED_LIMITS_MPS`, run
`scale_scenario.py` and `scene.build`, and update this record with a new one.
The derived scenario says so in its own header.

**Uncertainty is not yet in these margins.** Every verdict above assumes a
perfect estimator. The confirmation rule discounts by `confidence_sigma` = 2,
so what the real per-gate σ does to the exceedance at gate 3.0 — measured
0.0807 against a 0.04 limit — is F4's measurement and is not claimed here.
