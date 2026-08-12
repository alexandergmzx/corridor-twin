# ADR 0028: A is told B's address; the route is emergent

- Status: Accepted
- Date: 2026-08-12
- Source: robot1 corridor runs of 2026-08-11/12, artifacts under
  [`docs/evidence/robot-a-gate/`](../evidence/robot-a-gate/) —
  `corner-probe-*.json`, `gate-robot1-landmark-run.json`,
  `transit-audit-225511.json`, `NOTES-landmark.md`.
- Implements the autonomy correction that [ADR 0023](0023-governed-nav2-live-slam.md)
  left open: 0023 pinned *how* A moves (governed Nav2, live SLAM, no prior map,
  no AMCL). It never said how A knows where to go. This does.

## Decision

**A is given B's address. A is never given the route.**

The delivery goal is one `NavigateToPose` at a standoff beside B, expressed
relative to A's own spawn and rotated into A's heading, and the path is whatever
continuous replanning produces against a map SLAM is still building. Three
layers, none of which authors a path:

1. **Optimistic global planning** — NavFn with `allow_unknown: true`, planning
   through cells SLAM has not seen yet, on a rolling costmap sized to keep the
   goal in bounds.
2. **Continuous replanning** — Nav2's stock replanning-with-recovery behaviour
   tree, re-planning as the map fills. **The route emerges here.**
3. **Governed local control** — unchanged from 0023; Nav2 is an operator
   upstream of the governor, never a bypass of it.

## Why a goal and not a search

[ADR 0022](0022-robot-a-selection-gate.md) records that v1's authored line and
waypoints "were read, correctly, as a level indicator". The correction that
demands is that the *route* be emergent, and it says nothing about the
destination. A courier is told an address; being told an address is not being
told a route, and deriving the route from replanning against a map built live is
the thing v1 was faulted for not doing.

Search was considered and rejected. Frontier exploration is not packaged for
Jazzy, and more decisively it would be answering a question nobody asked: the
scenario is a delivery to a known person, not a survey.

## The goal is a standoff, never B's centre

B carries no `PhysicsCollisionAPI` and is easy to read as decoration. The RTX
lidar sees **render** geometry, so B lands in `/scan` and therefore in the
costmap. A goal at B's centre sits inside its own inflated footprint and can
never be reached to any tolerance, however well Nav2 behaves.

B being an obstacle is correct — a person is one. Aiming at their centre is not.
The goal is 0.6 m to B's street side, validated against the scene's own
free-space oracle rather than against a number copied out of the config, and
that construction is what the landmark's refined goal would reuse.

## Validation, and the order it happened in

**This method was adopted before it was measured, and that is recorded here
rather than smoothed over.** The plan was written, ratified and implemented on
2026-08-11 on the strength of the argument above; the measurements below came
afterwards and could have contradicted it.

They did not. Measured from simulator truth, world frame, against the standoff:

| run | closest approach | at |
|---|---|---|
| `20260812-001456` | 0.769 m | t+119 s |
| `20260812-002857` | 0.631 m | t+135 s |
| `20260812-011939` | **0.244 m** | t+60 s |
| `20260812-015929` | 0.404 m | t+81 s |

Governed Nav2 on a live SLAM map, with no authored route, took A from the
corridor mouth around the corner to the delivery standoff on every run that got
a goal. **The method works.** The emergent route is real, not a relabelled
waypoint list.

## What is NOT validated, in bold

**No run has passed the arrival gate.** The gate is Nav2 `SUCCEEDED` and
≤ 0.15 m map-frame error, and it remains unchanged by this record.

The reason no run passes is not navigation. It is that **the map diverges**, and
a map-frame number computed in a frame SLAM owns stops describing the robot when
that frame is wrong: one run reported `travelled_m: 1.32` and 6–7 m of goal
error for a robot that had physically driven the whole corridor and come within
0.768 m of B. A map-frame error of 0.15 m would be equally meaningless in the
other direction.

So the evaluation plane now measures **world-frame delivery error from simulator
truth** (evaluation-only, CLAUDE.md invariant 1), reporting closest approach
alongside final position — because "never arrived" and "arrived and left" are
different defects and the final position alone cannot tell them apart.

That measurement validates the *method*. It does not retire the arrival gate,
and the gate stays red until the map is trustworthy.

## Consequences

- A may read B's position from the manifest. It may not read the authored route,
  the trajectory, or a prior map. The A-side contract test continues to forbid
  police tokens; `b_xyz` was already P-side only and the nav gate already read it.
- The goal is start-relative: SLAM's map frame is anchored at A's spawn, and the
  three profiles spawn A on three different headings, so a shared literal would
  be quietly wrong on two profiles out of three.
- The speed policy's width→limit table stays `[to pin after first profile run]`.
  Pinning it needs a profile run whose speeds are trustworthy, and speeds derived
  from a diverged map are not.
- Superseded by nothing. Amended if terminal docking lands, which would add a
  refinement *after* arrival and never a second arrival mechanism.
