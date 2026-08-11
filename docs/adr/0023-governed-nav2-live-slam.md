# ADR 0023: Autonomy is governed Nav2 on a live SLAM map

- Status: Accepted
- Date: 2026-08-11
- Source: interview feedback of 2026-08-04 (autonomy correction); fleet
  session-6 Nav2 evidence; wiring and parameter values verified in code in
  [`docs/v2-plan.md`](../v2-plan.md) F23–F24.
- Extends [ADR 0003](0003-ros-time-and-clock-discipline.md) (the whole Nav2
  stack rides `/clock`), [ADR 0016](0016-corner-enforcement-policy-boundary.md)
  (zone *structure* unchanged), and
  [ADR 0010](0010-supplied-diagram-geometry.md), whose principle this record
  applies to a new axis.
- Amends the *values* of the width→limit table that ADR 0016 pinned under
  [ADR 0007](0007-speed-policy-and-violation.md)'s policy rule; the semantics
  of both records — explicit demonstration policy, owner approval,
  conservative confirmation, episode rules, and 0016's two-gate-minimum
  consequence — are untouched.

## Context

The fleet has already proven the pattern this scenario needs: Nav2 routed
*through* the safety governor rather than around it — `controller_server` and
`behavior_server` publish `cmd_vel_raw`; the governor and deadman own
`cmd_vel` — with a demonstrated mid-goal override and a map-frame success
check. Reinventing an autonomy pattern for the corridor would discard measured
evidence for prose.

One v1 assumption does not survive contact with the fleet's numbers. The v1
speed policy (0.8 / 1.2 / 1.5 m/s by clear width) was scaled to a v1 robot
commanded at 1.0 m/s. The fleet robots cannot reach the *lowest* of those
limits: robot2's Nav2 caps are 0.22 m/s, deliberately set under the governor's
0.35 m/s safety ceiling. The governor's defaults are carried as
`[estimate]`-tagged Yahboom-measured priors — braking on the RaspTank chassis
has never been measured, and the fleet's P2 program exists to replace each
number with one earned on this robot. A "violation from the robot's own
profile" is arithmetically impossible against the v1 table.

## Decision

1. **Governed path, always.** Every Nav2 output — path following and recovery
   motions alike — passes the same governor a teleop operator would. The
   corridor adopts the fleet launch pattern (`cmd_vel → cmd_vel_raw` remap on
   both writers of motion) unmodified.
2. **Live `slam_toolbox` map, no prior map, no AMCL.** A starts with no
   knowledge of the corridor and builds its map while delivering. This is the
   stronger autonomy claim, it is the configuration proven in session 6, and
   it keeps *prior scene knowledge* out of A's navigation stack — the
   simulator truth topic itself stays on A's domain per 0020 decision 4
   (retained by 0021), unbridged, consumed only by the offline evaluator. It
   holds for either ADR 0022 outcome (robot1's encoder EKF slots under the
   same stack).
3. **The speed policy re-pins to robot scale; neither ceiling moves.** This
   is ADR 0010's own principle applied to the velocity axis: topology from
   the task, scale a project choice. The governor cap (0.35 m/s) is a safety
   envelope carried as an `[estimate]` Yahboom prior pending the fleet's P2
   braking measurements on this chassis — not this scenario's to raise; the
   Nav2 cap (0.22 m/s) is the profile generator and needs no raising once
   policy re-pins. Policy values live on the evaluation/P side only — A
   never reads them — so re-pinning touches zero of A's stack.
4. **The violation is produced by A's own autonomous profile,** not a
   scripted override. Final limit values are pinned **after** measuring A's
   natural profile through the corridor, because the numbers select the
   episode's shape: a strict limit below cruise across the whole narrow
   stretch yields one long episode (per ADR 0014's semantics), while a strict
   limit set between cruise speed and the measured throat speed can yield a
   short, boundary-localized episode as Nav2 decelerates through the taper.
   One structural constraint binds the choice: 0016's no-spare consequence —
   both strict-zone gates (8.0 and 10.0 on the nominal taper) must measure
   over-limit or the two-estimate confirmation cannot fire at all. A limit
   the measured profile undercuts at gate 10.0 is therefore not pinnable
   under the retained zone structure; if the measured profile leaves no
   limit satisfying both the short-episode shape and the two-gate floor, the
   long-episode shape is the remaining choice, and changing the zone
   structure instead would be a new record amending 0016, not an edit here.
   Until that run exists the policy table carries a
   `[to pin after first profile run]` marker beside its v1 values, and ADR
   0007's owner-approval rule applies to the final numbers.
5. **Throat passability is measured, not assumed:** footprint radius,
   inflation radius, and costmap resolution recorded with the tuning evidence
   for whichever robot ADR 0022 selects. (For robot2 today those read
   0.12 m / 0.35 m / 0.05 m; at corridor scale the throat is generous, and
   the record exists so the claim is measured rather than eyeballed.)

## Consequences

- Route time and speed profile become stochastic within Nav2's controller
  behavior; v2 evidence records per-run values with seeds and bags rather
  than single canonical constants.
- Goal semantics for success: action status SUCCEEDED **and** map-frame pose
  error within tolerance — the status-unchecked anti-pattern is named here so
  it cannot recur (robot1's retracted accuracy figure is the cautionary
  precedent).
- Two measured fleet traps are adopted as requirements, not rediscovered: the
  global costmap's `map_topic` stays **absolute** (`/robot2/map` — a relative
  name resolves against the costmap's nested namespace and silently kills the
  planner), and lifecycle managers run with `bond_timeout: 0.0` (the bond
  deactivated SLAM mid-session when enabled).
- The Isaac path requires the scan conditioner between the RTX lidar and
  slam_toolbox (beam-count metadata and no-return encoding), or the map
  silently never builds.
- Sim-only scope is inherited from the fleet's standing floor rule: Nav2
  bypasses the governor on hardware until the stopping numbers exist. The
  corridor demonstration is simulation; nothing here claims a floor
  clearance.

## Alternatives considered

- **Authored occupancy map + AMCL.** Rejected: weaker autonomy claim (prior
  map), and it would hand A ground-truth scene knowledge that the corrected
  reading works hard to keep out of both planes. Recorded as the fallback if
  live SLAM proves unstable on the selected robot — adopting it would be a
  new ADR, not an edit here.
- **Scripted speed override to guarantee a violation.** Rejected: it
  reintroduces exactly the "fixed line" character v1 was corrected for, one
  layer up.
- **Raise the robot's velocity caps to meet the v1 policy table.** Rejected:
  the governor ceiling is a safety envelope whose numbers await this
  chassis's own braking measurements (raising an `[estimate]` prior is the
  worst direction to guess in), the Nav2 caps were chosen under it
  deliberately, and the policy table is the demonstration variable ADR 0007
  designed to be re-pinned — the robot is not.
