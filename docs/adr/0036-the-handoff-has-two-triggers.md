# ADR 0036: The handoff has two triggers, because the radius has no margin

- **Status:** Accepted
- **Date:** 2026-08-14
- **Extends:** [ADR 0033](0033-arrival-is-contact.md) §3, which defines the
  handoff by name. ADRs are immutable, so a second trigger is a new record
  rather than an edit; 0033's first trigger is unchanged and unloosened.

## Context

Run `20260814-031922` skipped the entire terminal phase and nothing said so.
Nav2 reported **SUCCEEDED at 0.6621 m** from B — 0.198 m off its own refined
goal — while the handoff fires only on a confirmed sighting at or inside
`docked_max_range_m` = **0.620 m**. The machine sat in `REFINE` with **zero
creep ticks**, the dock loop's exit condition was satisfied, control fell
through to reporting, and the run's only complaint was an unrelated map-frame
goal error. From the outside it reads as a docking failure. Docking was never
attempted.

**This is arithmetic, not luck.** The two numbers are the same number:

```
docked_max_range_m   = standoff + GOAL_TOLERANCE_M     (corridor_dock.py:205)
Nav2 SUCCEEDED within = standoff + xy_goal_tolerance    (nav2_robot1_corridor.yaml:66,131)

GOAL_TOLERANCE_M == xy_goal_tolerance == 0.15
```

They move together. **No choice of standoff creates margin between them**: a
goal Nav2 legitimately completes can land exactly on the handoff radius, or
outside it, with nothing left over.

Moving the refined goal inward instead is blocked twice, both measured:

- B's inflated footprint is `0.12 + robot_radius 0.128 + inflation_radius 0.18
  = 0.428 m`, so there is **under 42 mm** of room before NavFn refuses to plan
  — against a 42 mm measured miss.
- 0.470 m *is* the governor's floor (`GOVERNOR_STOP_DISTANCE_M 0.35 + b_radius
  0.12`), which ADRs 0031 and 0033 derive as a constraint. Commanding inside it
  makes Nav2 fight the safety envelope.

## Decision

**Nav2 finishing the refined goal is also a handoff**, gated on four
independent conditions. All four must hold; none loosens the first trigger.

1. **Nav2 reported SUCCEEDED.** Never `ABORTED`. An abort mid-corridor with the
   `EastWallStub` decoy confirmed is the one failure mode that looks like
   success, and this is the condition that excludes it.
2. **At least one refinement.** The goal Nav2 completed must have been derived
   from the *confirmed landmark*, not from the manifest.
3. **Inside `handoff_ceiling_m` = `docked_max_range_m + GOAL_TOLERANCE_M` =
   0.770 m.** The map/landmark disagreement that put 0.6621 m outside 0.620 m,
   bounded by the same tolerance a second time. A detection a metre out cannot
   qualify.
4. **Everything `armed()` already demands** — travel ≥ 4.849 m, the 100° body
   cone, k-of-n persistence across scans, radius unambiguous against the
   runner-up.

Run `025049` is correctly excluded by conditions 1 and 2 and by its travel
(4.792 m < 4.849 m). Run `031922` passes all four.

The handoff event records `trigger: "range" | "nav_succeeded"`, so which path
fired is a field in the artifact rather than an inference.

## What this recovers, stated honestly

**A measurement, not a delivery.**

All three runs that *did* hand off — from 0.618–0.620 m — ran the full 25 s
`CREEP_TIMEOUT_S` and reached the blind radius without a confirmed bump; two of
them reached 0.2252 m and 0.2258 m, and one reached 0.2146 m, *past* the
modelled contact. Handing off 44 mm further out will most likely also end in
`ARRIVED_UNPROVEN`.

What changes is that it ends there **with a full creep trace** — closest
approach, walked-away distance, the governor's permitted duty, the witness
record — comparable with the other three, instead of with nothing at all. The
budget allows it: `(0.770 − 0.2175) / 0.05 = 11.05 s` against a 25 s timeout.

If the creep turns out not to reach from beyond 0.62 m, the finding is that
`CREEP_TIMEOUT_S` is binding — a separate change on its own evidence, never
bundled into this one.

## Consequences

The failure mode this introduces is creeping onto the `EastWallStub` decoy and
reporting a delivery against the wrong object — the one failure that looks like
success. Conditions 1 and 2 are what stand between the change and that outcome,
and they are the ones to re-read if this is ever revisited.

The zero-margin arithmetic is asserted as a **cross-file test**: the corridor's
`xy_goal_tolerance` is read out of the Nav2 params and compared against
`GOAL_TOLERANCE_M`. A future Nav2 parameter change therefore breaks a test in a
second, rather than a run artifact three hours later — and if it ever does, the
argument for this record needs re-deriving, not patching.

**Not established:** whether the second trigger fires at all in practice. It has
never run against Isaac. Only one archived run would have used it.
