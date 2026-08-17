# ADR 0033: Arrival is contact, and the terminal phase is its own controller

- Status: Accepted
- Date: 2026-08-13
- Source: the 2026-08-13 delivery-close session, its twelve measured deliveries,
  and the ratified ruling that A **must** be able to bump B and that the bump
  **is** the arrival.
- **Supersedes the arrival criterion of
  [ADR 0028](0028-goal-directed-navigation-on-a-live-map.md)**, quoted below,
  and the two independent restatements of it in
  [ADR 0029](0029-map-divergence-at-the-corner.md) and
  [ADR 0031](0031-b-is-the-cylinder.md).
- **Supersedes `corridor_dock.py`'s "no raw `cmd_vel`" principle, by name.**
- Builds on [ADR 0031](0031-b-is-the-cylinder.md): B is one cylinder, and the
  contact distance is still derived from the two bodies rather than authored.

## Context

Three records said the same thing in three places:

> **No run has passed the arrival gate.** The gate is Nav2 `SUCCEEDED` and
> ≤ 0.15 m map-frame error, and it remains unchanged by this record.
> — ADR 0028

> **The arrival gate is unchanged** — Nav2 `SUCCEEDED` within 0.15 m map-frame —
> and the demonstration must pass with the detector disabled.
> — ADR 0029:127-130

> **The Nav2 arrival gate is unchanged**: `SUCCEEDED` within 0.15 m in the map
> frame (ADR 0028), and the demonstration must still pass with the detector
> disabled. — ADR 0031:104-106

**The map-frame criterion was never measuring the delivery.** The overshoot
diagnosis established why: over the first 2 m the EKF registers 0.94–1.05 of
truth while the scan matcher registers 0.13–0.72, so SLAM's pose runs 0.8–2.2 m
behind reality, longitudinally, exactly where the delivery happens. Thirteen of
22 runs drove to the street's far wall while their map said they were short.

ADR 0031 then defined a demo win as `DELIVERED` with world-frame
distance-to-B ≤ 0.470 m, the governor's floor. Twelve measured deliveries land
at **0.590–0.619 m**: at the transit standoff, never at the floor. Under 0031's
own definition none of them is a win, and the gap is structural rather than
accidental — the arrival band ends 0.15 m outside the floor by construction.

## Decision

### 1. Arrival is contact

A drives into B. The delivery is **`DELIVERED_CONFIRMED`**: A commanded forward
while its own encoders report no motion, over a debounce window, with B last
seen closing.

Contact is derived, never authored:

```
contact = a_length/2 + b_radius = 0.0975 + 0.120 = 0.2175 m   centre to centre
```

This retires ADR 0031's `≤ 0.470 m` demo win. 0.470 is the *governor's floor* —
the closest A may safely come — and a floor is a constraint, not a target.
Requiring A to arrive exactly at its own safety limit was a strange definition
of success, and no run ever met it.

### 2. The map-frame gate is retired; the world frame reports

Nav2 `SUCCEEDED` within 0.15 m map-frame is **no longer the arrival gate**. It
is still computed and still written to every artifact, **ungated**, because it
remains the best available measure of how far the map has drifted.

### 3. Transit is governed Nav2; terminal is a governed docking controller

The ordinary AMR split. `opennav_docking` is the pattern precedent; this is its
minimal in-house form. Nav2 owns the robot from the corridor mouth to the
handoff at `standoff + goal_tolerance` = 0.620 m; the docking controller owns it
from there to contact.

The handoff exists because **Nav2 will not make a contact**. Its local costmap
inflates B — a lethal cell, because the scan returns from it — by
`inflation_radius` 0.18 plus `robot_radius` 0.128, so the planner keeps roughly
0.31 m clear of the very object the mission is to touch. Nothing short of
disabling the inflation that also keeps A off the 0.9 m corridor walls would
change that, and the terminal phase is the standard place to solve it.

**`corridor_dock.py`'s "WHAT THIS DELIBERATELY IS NOT" clause is superseded by
name.** It read: *"Every metre A moves is still governed Nav2 executing a
`NavigateToPose`; this only chooses where that goal is. There is no raw
`cmd_vel`."* That was right while arrival was a distance. It is wrong for a
contact, and the replacement is not a bypass — see 4.

### 4. The governor is informed, never bypassed

The terminal creep goes to `/cmd_vel_raw`, through the same safety filter as
every other motion in this project. The filter is **told** what is happening:

- The mask suppresses the proximity floor **only** inside a ±15° cone toward a
  bearing the docking controller has already confirmed, and **only** for returns
  no farther than the range it has already measured plus a margin.
- Deadman, stale-scan stop and command-timeout stop are evaluated **before** the
  mode is consulted at all. A terminal approach cannot buy permission to drive
  on dead data.
- The off-cone obstacle stop keeps full strength. A person stepping in from the
  side still stops A.
- The empty-sector fail-closed survives: if the mask consumes every return, the
  result is still `inf` and `inf` still means stop.
- The speed cap **tightens**, to a 0.05 m/s creep, applied before the ordinary
  caps.

The mask expires on **silence**, not on an exit message. Every way of losing the
docking controller — crash, kill, starved topic — releases it by the same path,
and there is no "stop" message that can go missing.

The existing `--fun` preset was rejected for this: it sets `stop_distance` to
zero *and* raises `max_speed` to 1.0 and `max_yaw` to 5.0, disabling braking for
the corridor walls as well. Raising caps is the wrong tool for a terminal phase
that wants to go slower.

### 5. The encoders are the bumper, and the laser cannot witness the bump

**A has no bumper.** Contact is detected as commanded `vx > 0` against EKF
`vx ≈ 0` over a debounce window. Measured EKF noise while genuinely stationary
is 0.014 mm per sample, three orders of magnitude below the 0.01 m/s threshold.

And the laser cannot confirm it. B's surface enters the MS200's 0.120 m minimum
range while A's centre is still 0.240 m away, against a contact at 0.2175 m:

```
handoff        0.6200 m   Nav2 cancels here
B invisible    0.2400 m   <- the last 22.5 mm are driven blind
contact        0.2175 m
```

So the laser's role is not to see the touch but to testify that B was **closing**
when it was last visible. A stall with B last seen inside 0.390 m
(`lidar_min + b_radius + goal_tolerance`) is the bump. A stall with B last seen
a metre out is a kerb, a wall or a jammed wheel — that is **`ARRIVED_UNPROVEN`**,
and it is **never reported as success**.

### 6. B is solid, and the stub is the scenario's negative control

B carried no collider until this record — a bare `Cylinder` with no physics
schemas, while every wall has had one since the beginning. A would have driven
straight through it. B is now static-collidable: it stops A without being
pushable.

`EastWallStub` **stays, permanently.** It is the task author's topology
(ADR 0010 lineage, transferred by ADR 0018) and it defines the drivable lane
centre — 4.083 m against a 4.500 m geometric mid. Its free west end took **5 of
10 docked runs** before convexity separated it from B, and it is retained
deliberately as **the scenario's standing negative control**: a post-shaped,
free-standing, correctly-sized decoy 0.806 m from the target. A detector change
that cannot tell them apart is a detector change that has regressed.

## Acceptance

**TRANSIT → ACQUIRE → REFINE → DOCKING → `DELIVERED_CONFIRMED`**, with a
visible, gentle bump on the lens capture and the world-frame contact distance in
the artifact.

Reported alongside, **ungated**: map-frame goal error, closest approach to the
transit standoff, and duplicate-wall extent.

**The detector-disabled clause of ADR 0029 and ADR 0031 cannot survive this
record, and is retired rather than quietly dropped.** `DELIVERED_CONFIRMED`
requires a confirmed detection to enter DOCKING at all, so "the demonstration
must pass with the detector disabled" and "arrival is contact" cannot both hold.
With the detector off, A completes the transit and stops at the Nav2 goal; that
remains a valid, reportable outcome and it is **not** a delivery.

## Consequences

- **The speed policy is unreachable and is not fixed here.** The governor caps
  at 0.35 m/s and Nav2 at 0.22, against a strictest scene limit of 0.80 m/s, so
  no violation is currently demonstrable. ADR 0023 defers the width→limit table
  until measured profile runs exist; they now do. Pinning it is a separate
  record and is owed.
- The creep is bounded: 25 s, then `ARRIVED_UNPROVEN`.
- Contact and the blind radius are derived from the two bodies, so a rescale
  moves them together.
- Nothing here reopens `slam_toolbox` or Nav2 parameter tuning.
