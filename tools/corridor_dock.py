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
from collections import deque

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
#: So arming also requires the robot to be where B is. It used to check that
#: three ways -- near the end of its own route, near the goal IN THE MAP FRAME,
#: and looking at the goal -- and two of those three read the map pose. On
#: 2026-08-13 the map-frame proximity test was deleted and the bearing test
#: became a body-frame forward cone; see `armed`. What survives here is the
#: TRAVEL test, which reads A's own EKF and is the one that excludes the spawn.
#:
#: The window is therefore now a tolerance on travel alone: how much less than
#: the full route-to-delivery A may have driven and still be believed. It is
#: still a FRACTION OF THE ROUTE, not a copied metre. 3.0 m was 15.653%
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


#: At the instant of closest approach, B is ABEAM -- exactly 90 deg off the
#: nose. That is not an observation, it is what "closest" means: the range
#: derivative is zero when the bearing is perpendicular to the heading. So 90
#: deg is the floor for any forward cone that must admit the real B, and a
#: bearing wider than abeam means A is already past B.
ABEAM_DEG = 90.0

#: The approach is curved, so the abeam instant is approximate and the measured
#: maximum overshoots it slightly. Across seven bags the widest approach bearing
#: to B inside one metre was 85.2, 89.9, 90.1, 90.4, 90.8, 90.9 and 91.6 deg --
#: straddling 90 exactly as the geometry predicts, with 1.6 deg of excess.
#: Ten degrees is six times that.
#: (`tools/diagnostics/bearing_to_b.py`, 2026-08-13.)
APPROACH_CURVATURE_MARGIN_DEG = 10.0


def bearing_cone_deg() -> float:
    """The widest a detection may sit off A's NOSE and still be believed.

    **This became a body-frame test on 2026-08-13, and that is the point.**

    It used to ask "is the detection where the GOAL is?", comparing the
    detection's bearing against the bearing to the nominal goal in the MAP
    frame. That made a map-free measurement conditional on the map -- and the
    overshoot diagnosis showed the map pose is 0.8-2.2 m wrong along the
    corridor exactly when docking needs to arm.

    The replacement asks a question the laser can answer alone: is the thing
    ahead of me? Nothing about the goal, the map, or where A believes it is.

    It is a WEAK guard and that is deliberate -- at 100 deg it excludes only
    what is behind A. It is not what rejects a phantom. The travel gate, the
    radius-uniqueness test, the shape and isolation tests, and k-of-n
    persistence are what do that work; this one exists to catch the specific
    2026-08-12 13:16 failure, a right-shaped thing confirmed BEHIND the robot
    near spawn, and to stop docking re-aiming at B after A has driven past it.
    """

    return ABEAM_DEG + APPROACH_CURVATURE_MARGIN_DEG


#: How much better the best candidate's radius must fit than the runner-up's
#: before the frame counts as unambiguous.
#:
#: `LandmarkDetector.candidates` already refuses anything outside
#: MAX_RADIUS_ERROR_FRACTION of B's radius, so every survivor is roughly the
#: right size; this asks whether ONE of them is distinctly the right size. Two
#: equally good fits mean the scene is ambiguous and the wrong one is a coin
#: flip -- which is the 5.754 m failure mode, arrived at honestly.
#:
#: A fifth of B's radius: 0.024 m at the committed scale, which is larger than
#: the fit residual the detector admits and smaller than the gap between B and
#: anything else the corridor offers.
RADIUS_UNIQUENESS_FRACTION = 0.2

#: Arming persists over SCANS, not over calls. The docking loop spins at 10 Hz
#: whether or not a scan arrived -- 8119 `step()` calls against 3031 frames on
#: a docked run, 2.7x over -- so counting invocations would confirm on a single
#: measurement seen 27 times. Keyed on the detector's frame token instead.
ARM_CONFIRM_K = 3
ARM_CONFIRM_N = 5

#: How far apart two laser-frame detections may be and still be called the same
#: object. Unused by `_persisted` as shipped -- see the reverted experiment
#: documented there -- and kept because the decoy study needs the number named.
ARM_AGREEMENT_M = 0.25

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
MAX_BEARING_ERROR_DEG = bearing_cone_deg()


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
                 expected_radius_m: float | None = None,
                 arm_confirm_k: int = ARM_CONFIRM_K,
                 arm_confirm_n: int = ARM_CONFIRM_N,
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
        #: B's authored radius, for the uniqueness test. None means the caller
        #: did not supply one, and uniqueness is then not tested -- stated
        #: rather than silently defaulted, because a wrong radius here would
        #: reject the real B on every frame.
        self.expected_radius_m = expected_radius_m
        self.arm_confirm_k = arm_confirm_k
        self._arm_frames: deque = deque(maxlen=arm_confirm_n)
        self._last_arm_frame: int | None = None
        self.rejections: dict[str, int] = {}
        self.state = self.TRANSIT
        self.refinements = 0
        self.current_goal = nominal_goal
        self.landmark_map: tuple[float, float] | None = None
        self.history: list[dict] = []

    def _reject(self, reason: str) -> bool:
        self.rejections[reason] = self.rejections.get(reason, 0) + 1
        return False

    def armed(self, verdict: dict | None, travelled_m: float | None = None) -> bool:
        """Close enough to B by the LASER, far enough along by A's own odometry.

        **Nothing here reads the map pose, and that is the whole change.**

        The previous version required A to be within `window_m` of the nominal
        goal IN THE MAP FRAME. That gated a map-free measurement on the one
        number the overshoot diagnosis showed is wrong by 0.8-2.2 m along the
        corridor precisely when docking needs to arm: on a docked run it
        refused 2812 times while A's own laser was measuring B correctly at
        0.63 m, fitted radius 0.1244 against an authored 0.12. A guard that
        cannot distinguish "the robot is not there" from "the map thinks the
        robot is not there" is not a guard.

        What replaces it does the same job -- refuse a right-shaped thing in
        the wrong PLACE -- out of quantities the map cannot corrupt:

        * **Travel**, from A's own EKF. Path length, not displacement. This is
          what excludes the spawn region, and the spawn negative control is
          asserted against it by name.
        * **Radius uniqueness**, from the frame's runner-up. One thing of the
          right size, not two.
        * **A forward cone**, from the detection's own bearing. Abeam plus
          margin; it excludes what is behind A and little else, by design.
        * **k-of-n over SCANS**, keyed on the detector's frame token so a fast
          caller cannot confirm one measurement many times.

        FAIL CLOSED. Containment configured but not supplied means the caller
        did not pass what it promised, and arming anyway would restore exactly
        the failure this exists to prevent.
        """

        if not verdict or not verdict.get("confirmed"):
            self._forget_frame(verdict)
            return False
        candidate = verdict["candidate"]
        if candidate["range_m"] > self.arm_radius_m:
            self._forget_frame(verdict)
            return self._reject("out of laser range")

        if self.min_travel_m is None:
            return True     # containment not configured: transit-only behaviour

        if travelled_m is None:
            return self._reject("containment configured but not supplied")
        if travelled_m < self.min_travel_m:
            self._forget_frame(verdict)
            return self._reject("too early in the route")

        bearing_deg = abs(math.degrees(candidate.get("bearing_rad", 0.0)))
        bearing_deg = min(bearing_deg, 360.0 - bearing_deg)
        if bearing_deg > self.max_bearing_error_deg:
            self._forget_frame(verdict)
            return self._reject("detection is behind the robot")

        if not self._radius_is_unambiguous(verdict):
            self._forget_frame(verdict)
            return self._reject("two candidates fit B's radius equally well")

        return self._persisted(verdict)

    def _radius_is_unambiguous(self, verdict: dict) -> bool:
        """Is exactly one thing in this frame the right SIZE?

        The detector ranks by residual -- how circle-like -- so its best
        candidate is not necessarily its best-sized one. Two clusters that both
        fit B's radius make the choice between them a coin flip.
        """

        runner_up = verdict.get("runner_up")
        if runner_up is None:
            return True
        if self.expected_radius_m is None:
            return True
        margin = self.expected_radius_m * RADIUS_UNIQUENESS_FRACTION
        best_error = abs(verdict["candidate"]["fitted_radius_m"] - self.expected_radius_m)
        other_error = abs(runner_up["fitted_radius_m"] - self.expected_radius_m)
        return other_error - best_error > margin

    def _frame_of(self, verdict: dict | None) -> int | None:
        return verdict.get("frame") if verdict else None

    def _forget_frame(self, verdict: dict | None) -> None:
        """A frame that failed a test counts as a NO, not as a silence.

        Recorded as None -- a scan that saw nothing believable -- so that k-of-n
        measures agreement across recent scans rather than across recent
        *passing* ones. Otherwise a detection that qualifies once every two
        seconds confirms just as fast as one that qualifies every scan.
        """

        frame = self._frame_of(verdict)
        if frame is None or frame == self._last_arm_frame:
            return
        self._last_arm_frame = frame
        self._arm_frames.append(None)

    def _persisted(self, verdict: dict) -> bool:
        """k of the last n SCANS carried a believable detection.

        **A stronger-looking version of this was written, measured, and
        reverted on 2026-08-13, and the measurement is why the weaker one is
        here.** It additionally required the qualifying frames to agree on a
        laser-frame position, clearing the run when they disagreed -- which
        sounds strictly better, and reads as "three scans that agree they are
        looking at the same object" rather than merely "three scans that each
        saw something".

        Replayed over seven recorded runs it was WORSE: arming fired on the
        `EastWallStub` decoy on six of seven instead of four. The reason is
        ordering, not agreement. The stub's west end cap sits at
        (4.565, -1.926), between A and B on the approach, so A resolves it
        FIRST; requiring consecutive agreement just hands the decision to
        whichever object accumulates a run first, and that is the decoy.

        Neither version is acceptable and neither is shipped as a fix. See
        `docs/evidence/robot-a-gate/NOTES-the-eastwallstub-decoy-20260813.md`.
        """

        frame = self._frame_of(verdict)
        if frame is None:
            # No frame token: an older detector, or a hand-built verdict in a
            # test. Persistence is the detector's own k-of-n only.
            return True
        if frame != self._last_arm_frame:
            self._last_arm_frame = frame
            self._arm_frames.append(True)
        if sum(1 for entry in self._arm_frames if entry) < self.arm_confirm_k:
            return self._reject("not yet persistent across scans")
        return True

    def step(self, robot_xy: tuple[float, float], robot_yaw: float,
             verdict: dict | None, travelled_m: float | None = None) -> dict | None:
        """One detector verdict in; a new goal to issue, or None to keep going."""

        if self.state in (self.DOCKED, self.UNREFINED):
            return None
        if not self.armed(verdict, travelled_m):
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
