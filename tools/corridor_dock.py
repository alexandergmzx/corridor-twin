#!/usr/bin/env python3
"""Terminal docking: drive to B by SENSING it, not by trusting the map.

Used by `corridor_nav_gate.py --dock`. This module holds the geometry and the
state machine; the gate owns the action client and the ROS spinning, so this
file stays testable without standing up a robot.

WHY THIS EXISTS
---------------
A reaches the delivery standoff and then leaves. Not because navigation fails --
governed Nav2 on a live map takes it around the corner every time -- but because
the goal lives in the SLAM map frame and that frame drifts near the corner. A
drives accurately to a goal that has stopped corresponding to where B is.

The landmark detector measures B in the LASER frame. That measurement is immune
to the drift, and this turns it into motion.

THE ONE PROPERTY THAT MAKES THIS WORK
-------------------------------------
A goal computed from a detection depends only on the transform **at the instant
it is computed** -- the robot's current map pose composed with a range and
bearing it can see right now. Accumulated map error does not move it, because it
is never expressed against the map's history. The map can be globally wrong by
metres and this goal still lands beside B.

That is also its limit: the map keeps drifting while A drives the last few
metres, so a goal fixed once at 3 m out drifts by whatever the map does over
that distance. Hence a BOUNDED re-issue -- not a control loop, and not one shot
either.

WHAT THIS DELIBERATELY IS NOT
-----------------------------
Not a motion primitive. Every metre A moves is still governed Nav2 executing a
`NavigateToPose`; this only chooses where that goal is. There is no raw
`cmd_vel`, no search, no exploration. The MS200 is 360 deg, so acquisition needs
no motion at all.

ARRIVAL IS MEASURED IN THE LASER FRAME
--------------------------------------
"A is this close to B, by its own sensor" is a claim the map cannot corrupt.
`docked` is decided on the detected range, never on a map-frame number, and
the range it is compared against is derived by `final_approach_m` rather
than authored (ADR 0031).
"""

from __future__ import annotations

import math

#: Arm on the DETECTED RANGE, which is a laser-frame quantity.
#:
#: The first version armed on the map-frame distance from the robot to the
#: nominal goal, which is precisely the number this whole mechanism exists to
#: not trust: on a run where A came within 0.49 m of B physically, its drifted
#: map pose never came within 3 m of the map goal, so docking never armed and
#: the machine sat in TRANSIT with zero refinements.
#:
#: Gating a map-independent measurement on a map-frame number gives away the
#: only property that made it worth having. The detector's own shape and radius
#: tests are what reject corridor geometry -- proved against a wall, a convex
#: corner and a wrong-radius cylinder -- and 3-of-5 agreement is what rejects a
#: lucky frame. Range alone is enough of a gate on top of that.
ARM_RADIUS_M = 3.0

#: CONTAINMENT. Range alone was not enough of a gate, and a run proved it: on
#: 2026-08-12 13:16 the detector confirmed something at 1.06 m near the spawn,
#: docking re-aimed the mission at it, Nav2 drove 0.58 m and reported SUCCEEDED,
#: and the world-frame delivery error was 5.754 m. The post was not even in that
#: arena. Shape and 3-of-5 agreement reject the wrong SHAPE; they cannot reject
#: a right-shaped thing in the wrong PLACE.
#:
#: So arming also requires the robot to be where B is: near the end of its own
#: route, near the goal in the map frame, and looking at it.
#:
#: The window is a FRACTION OF THE ROUTE, not a copied metre. 3.0 m was 15.653%
#: of the authored 19.166 m route-to-delivery; at the committed scale that route
#: is 5.750 m and the window is 0.900 m -- which is also exactly 3.0 x the 0.30
#: scale factor, so the derivation checks against itself. A literal 3.0 here
#: would be more than half the route.
ARM_WINDOW_ROUTE_FRACTION = 0.15653

#: How far the transit goal stands off from B's CENTRE.
#:
#: Lives here rather than in the gate because since ADR 0031 it is a
#: contact-semantics number: it is the lateral separation between what the
#: detector confirms and where the transit goal sits, and the bearing cone
#: below is derived from it. `corridor_nav_gate` imports it from here so the
#: nominal and refined standoffs stay one rule.
#:
#: It clears B's inflated footprint by construction -- B's radius 0.12 +
#: robot_radius 0.128 + inflation_radius 0.18 = 0.428 -- and it stands OUTSIDE
#: the derived final approach (0.470 m), which is the right ordering: transit
#: gets A near B, docking closes the last stretch on the sensor.
DELIVERY_STANDOFF_M = 0.6


def bearing_cone_deg(lateral_m: float, tolerance_m: float) -> float:
    """The widest the detection may sit off the goal bearing, derived.

    **ADR 0031 widened this, and the widening is a real cost of the merge.**

    The cone exists to refuse a right-shaped thing in the wrong PLACE -- the
    2026-08-12 13:16 phantom, confirmed behind the robot near spawn. Its old
    +/-60 deg was sized when the detectable post stood 0.8 m south of B while
    the goal sat 0.6 m west, putting detection and goal 1.000 m apart on
    different sides.

    With one object the detection IS B, 0.6 m lateral of the goal on the side A
    approaches from, so the bearing to it swings *harder* as A closes: 63.4 deg
    at 0.3 m from the goal, which the old cone refused. Measured, not guessed --
    the test that caught it is
    `test_the_bearing_gate_admits_the_REAL_B_from_the_real_manifest`.

    The floor is therefore the bearing at A's closest legitimate position: the
    goal itself, give or take the arrival tolerance. `atan2(0.6, 0.15)` is
    76.0 deg, and that is what this returns.

    Stated plainly: **the cone is a weaker guard after the merge.** What still
    excludes the spawn phantom is the travel test and the map-frame proximity
    test, neither of which the merge touches, and the spawn negative control is
    asserted against the travel test by name.
    """

    return math.degrees(math.atan2(lateral_m, tolerance_m))

#: The governor's hard-stop range, from the fleet safety node that owns
#: `/cmd_vel` on this chassis (`yahboomcar_safety/governor.py:44`,
#: `stop_distance: float = 0.35`). It is a LASER range: the governor stops the
#: robot when the nearest scan return is inside it. The laser sits within a
#: centimetre of `base_footprint` (measured x = -4.6 mm), so it is read here as
#: a distance from A's centre without correction, an order of magnitude below
#: the docking tolerance.
GOVERNOR_STOP_DISTANCE_M = 0.35

#: ADR 0022's pinned arrival tolerance, restated from `corridor_nav_gate`'s
#: constant of the same name. A goal reached "within tolerance" may be reached
#: this much NEARER than commanded, so the contact term has to carry it or the
#: derivation permits a delivery that ends inside B.
GOAL_TOLERANCE_M = 0.15

#: Derived, never authored. See `bearing_cone_deg`.
MAX_BEARING_ERROR_DEG = bearing_cone_deg(DELIVERY_STANDOFF_M, GOAL_TOLERANCE_M)


#: Re-issue only when the estimate has actually moved. Below this the goal is
#: the same goal and re-sending it just interrupts a working approach.
REISSUE_IF_MOVED_M = 0.20

#: BOUNDED. Not a control loop: each re-issue is a fresh Nav2 goal, and a robot
#: that cannot converge in this many is not going to converge in more.
MAX_REFINEMENTS = 4

#: Arrived, measured by the sensor. The tolerance is generous against the
#: standoff because what is being claimed is "A is beside B", not a survey.
DOCKED_TOLERANCE_M = 0.25


def final_approach_m(b_radius_m: float, a_length_m: float) -> float:
    """How close A may end up to B's CENTRE, derived rather than authored.

    ADR 0031. Two terms, and the larger wins:

    **The governor's floor.** `stop_distance` is measured to B's *surface*, so
    in centre-to-centre terms it is ``0.35 + b_radius``. The governor is never
    bypassed -- the demo win is defined at a distance the safety envelope
    actually permits, which is the whole point of deriving this instead of
    picking a number that looks good in a video.

    **Geometric contact.** A's half-length plus B's radius is the distance at
    which the two touch; the arrival tolerance is added because A is allowed to
    stop that much short of its goal, or that much past it.

    At the committed scenario these are 0.470 m and 0.368 m, so the governor
    decides. That ordering is not assumed anywhere -- both are computed and
    compared on every call, because a larger B or a longer robot flips it.
    """

    governor_floor = GOVERNOR_STOP_DISTANCE_M + b_radius_m
    geometric_contact = a_length_m / 2.0 + b_radius_m + GOAL_TOLERANCE_M
    return max(governor_floor, geometric_contact)


def landmark_in_map(detection: dict, robot_xy: tuple[float, float],
                    robot_yaw: float) -> tuple[float, float]:
    """Detection (laser frame) -> map frame, using the CURRENT robot pose.

    The laser sits within a centimetre of `base_footprint` on this chassis
    (measured: x = -4.6 mm, y = 0.0), so the laser-to-base offset is neglected
    and the detection is treated as base-relative. That approximation is an
    order of magnitude below the docking tolerance.
    """

    cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
    return (
        robot_xy[0] + detection["x"] * cos_yaw - detection["y"] * sin_yaw,
        robot_xy[1] + detection["x"] * sin_yaw + detection["y"] * cos_yaw,
    )


def dock_goal(landmark_xy: tuple[float, float], robot_xy: tuple[float, float],
              standoff_m: float) -> tuple[float, float]:
    """Stop `standoff_m` short of B, on the side A is approaching from.

    `standoff_m` is required, not defaulted: since ADR 0031 it is derived from
    the scenario by `final_approach_m`, and a default here would be a stale
    literal waiting for the next rescale to make it wrong.

    The bearing comes from A's own position rather than from the scene, so the
    goal is always reachable from where A actually is -- no assumption about
    which way it arrived, and nothing read from the authored route.
    """

    dx, dy = landmark_xy[0] - robot_xy[0], landmark_xy[1] - robot_xy[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-6:
        return robot_xy
    if distance <= standoff_m:
        # Already inside the standoff: hold position rather than reversing into
        # a place the robot has not sensed.
        return robot_xy
    scale = (distance - standoff_m) / distance
    return (robot_xy[0] + dx * scale, robot_xy[1] + dy * scale)


class DockingMachine:
    """TRANSIT -> ACQUIRE -> REFINE -> DOCKED, with every transition bounded."""

    TRANSIT = "TRANSIT"
    ACQUIRE = "ACQUIRE"
    REFINE = "REFINE"
    DOCKED = "DOCKED"
    UNREFINED = "DELIVERED_UNREFINED"

    def __init__(self, nominal_goal: tuple[float, float], *,
                 standoff_m: float,
                 arm_radius_m: float = ARM_RADIUS_M,
                 max_refinements: int = MAX_REFINEMENTS,
                 route_length_m: float | None = None,
                 window_m: float | None = None,
                 max_bearing_error_deg: float = MAX_BEARING_ERROR_DEG) -> None:
        self.nominal_goal = nominal_goal
        #: Derived by `final_approach_m` from B's radius and A's length, never
        #: authored. It is both where the refined goal is placed and, below,
        #: the range at which arrival is declared.
        self.standoff_m = standoff_m
        self.arm_radius_m = arm_radius_m
        self.max_refinements = max_refinements
        # Containment. `route_length_m` is the route TO THE DELIVERY, not the
        # whole trajectory: the departure leg runs past B, and requiring travel
        # against a length that includes it would mean the detector could never
        # arm at all.
        self.route_length_m = route_length_m
        self.window_m = (
            window_m if window_m is not None
            else (route_length_m * ARM_WINDOW_ROUTE_FRACTION
                  if route_length_m is not None else None)
        )
        self.min_travel_m = (
            route_length_m - self.window_m
            if route_length_m is not None and self.window_m is not None
            else None
        )
        self.max_bearing_error_deg = max_bearing_error_deg
        self.rejections: dict[str, int] = {}
        self.state = self.TRANSIT
        self.refinements = 0
        self.current_goal = nominal_goal
        self.landmark_map: tuple[float, float] | None = None
        self.history: list[dict] = []

    def _reject(self, reason: str) -> bool:
        self.rejections[reason] = self.rejections.get(reason, 0) + 1
        return False

    def armed(self, verdict: dict | None,
              robot_xy: tuple[float, float] | None = None,
              robot_yaw: float | None = None,
              travelled_m: float | None = None) -> bool:
        """Close enough to B by the LASER, and standing where B is.

        The range test is still laser-frame and still the thing the map cannot
        corrupt. The containment tests around it are map-frame and travel-based,
        and they are deliberately COARSE -- a 0.9 m window and a 60 deg cone --
        because their job is to exclude the spawn region, not to localise.

        FAIL CLOSED. Containment configured but not supplied means the caller
        did not pass what it promised, and arming anyway would restore exactly
        the failure this exists to prevent.
        """

        if not verdict or not verdict.get("confirmed"):
            return False
        candidate = verdict["candidate"]
        if candidate["range_m"] > self.arm_radius_m:
            return self._reject("out of laser range")

        if self.min_travel_m is None:
            return True     # containment not configured: transit-only behaviour

        if travelled_m is None or robot_xy is None or robot_yaw is None:
            return self._reject("containment configured but not supplied")
        if travelled_m < self.min_travel_m:
            return self._reject("too early in the route")
        if math.dist(robot_xy, self.nominal_goal) > self.window_m:
            return self._reject("too far from the goal in the map frame")

        goal_bearing = math.atan2(
            self.nominal_goal[1] - robot_xy[1], self.nominal_goal[0] - robot_xy[0]
        ) - robot_yaw
        error = abs((candidate.get("bearing_rad", 0.0) - goal_bearing + math.pi)
                    % (2.0 * math.pi) - math.pi)
        if math.degrees(error) > self.max_bearing_error_deg:
            return self._reject("detection is not where the goal is")
        return True

    def step(self, robot_xy: tuple[float, float], robot_yaw: float,
             verdict: dict | None, travelled_m: float | None = None) -> dict | None:
        """One detector verdict in; a new goal to issue, or None to keep going."""

        if self.state in (self.DOCKED, self.UNREFINED):
            return None
        if not self.armed(verdict, robot_xy, robot_yaw, travelled_m):
            return None

        self.state = self.ACQUIRE if self.state == self.TRANSIT else self.state

        landmark = landmark_in_map(verdict["candidate"], robot_xy, robot_yaw)
        detected_range = verdict["candidate"]["range_m"]

        # Arrival is decided on the SENSOR, never on a map-frame number.
        if abs(detected_range - self.standoff_m) <= DOCKED_TOLERANCE_M:
            self.state = self.DOCKED
            self.landmark_map = landmark
            self.history.append({
                "event": "docked", "range_m": detected_range,
                "landmark_map": [round(v, 4) for v in landmark],
            })
            return None

        moved = (
            self.landmark_map is None
            or math.dist(landmark, self.landmark_map) > REISSUE_IF_MOVED_M
        )
        if not moved:
            return None
        if self.refinements >= self.max_refinements:
            # Bounded: stop refining, let the last goal finish, and say so.
            if self.state != self.UNREFINED:
                self.state = self.UNREFINED
                self.history.append({"event": "refinement_budget_spent"})
            return None

        self.landmark_map = landmark
        self.refinements += 1
        self.state = self.REFINE
        goal = dock_goal(landmark, robot_xy, self.standoff_m)
        self.current_goal = goal
        self.history.append({
            "event": "refine",
            "n": self.refinements,
            "detected_range_m": detected_range,
            "landmark_map": [round(v, 4) for v in landmark],
            "goal_map": [round(v, 4) for v in goal],
        })
        return goal

    def report(self) -> dict:
        return {
            "state": self.state,
            "refinements": self.refinements,
            # What containment refused, and why. A run where docking never armed
            # should say whether the detector saw nothing or saw something it
            # was right to distrust -- the difference between a sensor problem
            # and a phantom, which cost a day when it was invisible.
            "containment": {
                "route_length_m": (
                    round(self.route_length_m, 4) if self.route_length_m else None
                ),
                "window_m": round(self.window_m, 4) if self.window_m else None,
                "min_travel_m": (
                    round(self.min_travel_m, 4) if self.min_travel_m else None
                ),
                "max_bearing_error_deg": self.max_bearing_error_deg,
                "rejections": dict(self.rejections),
            },
            "landmark_map_frame": (
                [round(v, 4) for v in self.landmark_map] if self.landmark_map else None
            ),
            "final_goal_map_frame": [round(v, 4) for v in self.current_goal],
            "history": self.history,
        }
