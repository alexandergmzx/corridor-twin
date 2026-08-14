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
#: the full route-to-delivery A may have driven and still be believed.
#:
#: **RE-BASED on 2026-08-13, in the same commit that corrected the route.**
#: 0.15653 was 3.0 m of the authored 19.166 m route, carried across the rescale.
#: It was applied to a `route_to_delivery_m` that dropped the departure leg, so
#: the effective `min_travel_m` was 4.850 m -- and that number worked, while the
#: 0.900 m window it appeared to express did not describe anything real.
#:
#: The fraction has to absorb a gap nobody had measured: **A does not drive the
#: authored route.** Nav2 plans its own, and the measured odometry travel at
#: first arming on the real B is 5.699-6.695 m against an authored 7.380 m --
#: a shortfall of 0.685 to 1.681 m across seven bags
#: (`tools/diagnostics/arming_replay.py`, 2026-08-13).
#:
#: So the window is bounded on both sides by measurement, and the band is wide:
#:
#:   * it must EXCEED route - earliest measured arming = 7.380 - 5.699 = 1.681 m,
#:     or a good arming is refused;
#:   * it must FALL SHORT of route - the spawn control = 7.380 - 0.58 = 6.800 m,
#:     or the spawn phantom is admitted again.
#:
#: 0.343 puts the window at 2.531 m: **50% margin above the largest measured
#: shortfall**, and 4.27 m clear of the spawn control. It also holds `min_travel`
#: at 4.849 m on nominal, which is the value under which 8 of 8 post-convexity
#: docked runs armed on B -- so the correction changes the derivation without
#: disturbing the behaviour the evidence was gathered on. Both bounds are
#: asserted in `test_corridor_nav_gate.py`, by name.
ARM_WINDOW_ROUTE_FRACTION = 0.343

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

#: Arrived, measured by the sensor -- and the test is ONE-SIDED.
#:
#: It used to be `abs(detected_range - standoff) <= 0.25`, symmetric, and that
#: cost a live delivery 0.196 m of the 0.247 m it missed by. On 2026-08-13 A
#: saw B at 0.6655 m against a 0.470 m standoff; the symmetric band admitted it
#: (|0.6655 - 0.470| = 0.196 < 0.25), so the machine declared arrival while
#: still a fifth of a metre too far out, cancelled the goal, and stopped. The
#: refinement loop that exists to close exactly that distance never ran:
#: `refinements: 0`.
#:
#: Being too FAR OUT is not arrival. It is the condition refinement is for, and
#: a tolerance wide enough to swallow the refinement step disables the
#: mechanism it is supposed to be a tolerance on. Being NEARER than the standoff
#: is arrival -- A is beside B, the governor's floor is what stopped it, and
#: there is nothing left to refine.
#:
#: The near side therefore has no bound at all, and the far side is
#: `GOAL_TOLERANCE_M` -- ADR 0022's pinned figure, which already means "a goal
#: reached within tolerance may be reached this much nearer or further than
#: commanded", the same quantity `final_approach_m` uses for the same reason.
def docked_max_range_m(standoff_m: float) -> float:
    """The furthest detected range that still counts as arrived.

    **Since ADR 0033 this is a HANDOFF, not an arrival.** Reaching it ends
    Nav2's part of the delivery and begins the creep; arrival is the contact
    that follows. The threshold is unchanged, and the name is kept because it
    is still the same boundary -- what changed is what happens at it.
    """

    return standoff_m + GOAL_TOLERANCE_M


#: The MS200's minimum range. Below this the lidar reports nothing, so B stops
#: existing as far as A's own sensing is concerned.
LIDAR_MIN_RANGE_M = 0.12

#: The creep, commanded. The governor clamps to its own
#: `docking_creep_max_speed` independently -- this is not relying on that, it is
#: agreeing with it, and a mismatch is a bug in one of the two.
CREEP_SPEED_MPS = 0.05

#: THE CREEP HAS TO STEER, and the first version did not.
#:
#: It commanded pure forward motion, on the unexamined assumption that A is
#: pointing at B when the handoff happens. A is not: the yaw study measured
#: arrival headings **51-79 degrees** off the delivery heading, because A comes
#: round the corner mid-turn. Run 20260814-003034 is what that costs -- the
#: creep ran its full 25 s without ever stalling, and B went from 0.6133 m at
#: handoff to 0.6335 m by the timeout. A drove a metre and a quarter at a
#: tangent and the range grew.
#:
#: Proportional yaw toward the confirmed bearing. The gain is deliberately soft:
#: the governor caps yaw to `max_yaw_near` = 0.4 rad/s inside its stop distance
#: anyway, so a stiffer gain would only be clipped, and a clipped controller is
#: one whose behaviour is set somewhere else.
CREEP_YAW_GAIN = 0.8

#: Do not drive forward while badly misaligned -- turn first. Forward speed is
#: scaled by the cosine of the bearing error and cut entirely past this, so A
#: pivots onto B rather than spiralling around it. 30 degrees keeps the
#: alignment well inside the governor's 15-degree mask cone by the time A is
#: actually closing.
CREEP_MAX_BEARING_RAD = 0.5236

#: Below this, A is not moving. Measured EKF noise while genuinely stationary is
#: 0.014 mm per sample, so this is three orders of magnitude above the floor and
#: well under the 0.05 m/s being commanded.
STALL_SPEED_MPS = 0.01

#: How long A must be commanded forward while not moving before it counts as
#: contact rather than as a slow start. Ten samples at the gate's 10 Hz spin.
STALL_DEBOUNCE_S = 1.0

#: The creep is bounded like every other loop here. 0.40 m at 0.05 m/s is 8 s;
#: this is generous enough for a slow start and short enough that a robot stuck
#: on nothing gives up while the session still has time to say so.
CREEP_TIMEOUT_S = 25.0


def contact_range_m(b_radius_m: float, a_length_m: float) -> float:
    """Centre-to-centre distance at which A touches B.

    Derived from the two bodies, never authored: A's front face is half its
    length from its centre, and B's surface is one radius from its. At the
    committed scenario that is 0.0975 + 0.120 = 0.2175 m.
    """

    return a_length_m / 2.0 + b_radius_m


def last_sighting_ceiling_m(b_radius_m: float, a_length_m: float) -> float:
    """How near B must have been, last time A could see it, for a stall to be B.

    **A cannot see the bump.** B's surface enters the MS200's 0.12 m minimum
    range while A's centre is still `LIDAR_MIN_RANGE_M + b_radius` away -- 0.240 m
    at the committed scenario -- and contact is at 0.2175 m. So the final 22 mm
    are driven blind, and no laser reading exists at the moment of contact to
    confirm it with.

    What the laser CAN do is testify that B was closing. If A stalls having last
    seen B at 1.5 m, something else stopped it and the delivery is unproven. If
    A stalls having last seen B just before it went blind, the stall is the
    bump. This is that threshold, and it is the blind radius plus the arrival
    tolerance rather than a chosen number.
    """

    return LIDAR_MIN_RANGE_M + b_radius_m + GOAL_TOLERANCE_M


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


def facing_yaw(from_xy: tuple[float, float], to_xy: tuple[float, float]) -> float:
    """Which way to point, standing at `from_xy` and looking at `to_xy`.

    Trivial, and it exists so that the two places that need it cannot drift
    apart. The transit goal derives its yaw from B's bearing off the delivery
    standoff (`corridor_nav_gate.goal_yaw_in_map_frame`); every REFINED goal
    needs the same rule applied at the point the detector just chose, and until
    2026-08-13 it did not get it -- the refine path overwrote the goal's x and y
    and left the transit yaw in place, an angle derived for a different
    position. One function, so "facing B" means the same thing in both.
    """

    return math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0])


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
    #: Nav2 has handed off and A is creeping onto B under the docking
    #: controller. ADR 0033: transit is governed Nav2, terminal is a governed
    #: docking controller -- the standard AMR split.
    DOCKING = "DOCKING"
    #: The bump happened and the encoders witnessed it.
    DELIVERED_CONFIRMED = "DELIVERED_CONFIRMED"
    #: A stopped without a witnessed bump. **Never reported as success.**
    ARRIVED_UNPROVEN = "ARRIVED_UNPROVEN"
    #: Retained: the refinement budget ran out during the Nav2 phase.
    UNREFINED = "DELIVERED_UNREFINED"

    #: Terminal states. `DOCKED` is gone -- reaching the old arrival band is now
    #: a handoff into DOCKING, not an arrival.
    TERMINAL = (DELIVERED_CONFIRMED, ARRIVED_UNPROVEN, UNREFINED)

    def __init__(self, nominal_goal: tuple[float, float], *,
                 standoff_m: float,
                 arm_radius_m: float = ARM_RADIUS_M,
                 max_refinements: int = MAX_REFINEMENTS,
                 route_length_m: float | None = None,
                 window_m: float | None = None,
                 expected_radius_m: float | None = None,
                 a_length_m: float | None = None,
                 arm_confirm_k: int = ARM_CONFIRM_K,
                 arm_confirm_n: int = ARM_CONFIRM_N,
                 max_bearing_error_deg: float = MAX_BEARING_ERROR_DEG) -> None:
        self.nominal_goal = nominal_goal
        #: Derived by `final_approach_m` from B's radius and A's length, never
        #: authored. It is both where the refined goal is placed and, below,
        #: the range at which arrival is declared.
        self.standoff_m = standoff_m
        #: One-sided. Derived from the standoff, so a rescale moves it too.
        self.docked_max_range_m = docked_max_range_m(standoff_m)
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
        # Terminal creep state (ADR 0033). `b_radius_m` and `a_length_m` are
        # taken from the scenario rather than assumed, so a rescale moves the
        # contact distance and the blind radius together.
        self.contact_range_m = (
            contact_range_m(expected_radius_m, a_length_m)
            if expected_radius_m is not None and a_length_m is not None else 0.0
        )
        self.last_sighting_ceiling_m = (
            last_sighting_ceiling_m(expected_radius_m, a_length_m)
            if expected_radius_m is not None and a_length_m is not None
            else LIDAR_MIN_RANGE_M + GOAL_TOLERANCE_M
        )
        self._creep_started_s: float | None = None
        self._stall_since_s: float | None = None
        self._last_seen_range_m: float | None = None
        self._last_seen_bearing_rad: float = 0.0
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

        if self.state in self.TERMINAL or self.state == self.DOCKING:
            return None
        if not self.armed(verdict, travelled_m):
            return None

        self.state = self.ACQUIRE if self.state == self.TRANSIT else self.state

        landmark = landmark_in_map(verdict["candidate"], robot_xy, robot_yaw)
        detected_range = verdict["candidate"]["range_m"]

        # THE HANDOFF, not the arrival. Reaching the old arrival band used to be
        # the end of the delivery; since ADR 0033 it is where Nav2's part ends
        # and the creep begins, because the delivery is a contact and Nav2 will
        # not plan into an inflated lethal cell to make one.
        if detected_range <= self.docked_max_range_m:
            self.state = self.DOCKING
            self.landmark_map = landmark
            self.history.append({
                "event": "handoff_to_docking", "range_m": detected_range,
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

    def creep(self, verdict: dict | None, measured_vx: float | None,
              now_s: float, governed_vx: float | None = None,
              laser_stationary: bool | None = None) -> dict | None:
        """One creep tick. Returns what to command, or None when not creeping.

        **This is the only place in the project that commands a velocity.**
        Everywhere else, motion is Nav2 executing a `NavigateToPose`, and
        `corridor_dock`'s own header said so as a design principle. ADR 0033
        supersedes that by name: transit stays governed Nav2, and the terminal
        phase becomes a governed docking controller. The split is the ordinary
        AMR one -- `opennav_docking` is the pattern -- and this is its minimal
        in-house form.

        "Governed" is not decoration. The creep goes to `/cmd_vel_raw`, through
        the same safety filter as everything else, and the filter is TOLD what
        is happening rather than switched off: the returned `approach` is
        published to the governor's docking topic, which masks the proximity
        floor over B's own SILHOUETTE -- a disc of the authored radius plus a
        margin, at the confirmed bearing and range -- and nothing else. Deadman,
        stale-scan, empty-sector and off-object stops stay live throughout.

        It was a fixed 15-degree cone until 2026-08-14, which is a shape no
        target of finite size fits inside: B subtends 33.5 degrees at contact,
        so the cone brakes on B's own leaked shoulders and contact is
        unreachable. See `DockingDisc` in the fleet governor.

        `governed_vx` is what the safety filter actually PERMITTED, read back
        off /cmd_vel, and `laser_stationary` is the scan matcher's verdict that
        the robot did not move. Both are optional so the unit tests and the
        bench can exercise the machine alone, and both are supplied live --
        without them a governor stop forges a bump and a wheel slip hides one.

        Returns a dict with `vx`, `wz`, `approach` (bearing/range/target radius
        for the governor) and `reason`, or None when the machine is not in
        DOCKING.
        """

        if self.state != self.DOCKING:
            return None

        if self._creep_started_s is None:
            self._creep_started_s = now_s
            self._stall_since_s = None
            self.history.append({"event": "creep_begin", "at_s": round(now_s, 3)})

        # The freshest sighting drives the mask, so it narrows as A closes.
        candidate = (verdict or {}).get("candidate") if verdict else None
        if candidate and (verdict or {}).get("confirmed"):
            self._last_seen_range_m = candidate["range_m"]
            self._last_seen_bearing_rad = candidate.get("bearing_rad", 0.0)

        elapsed = now_s - self._creep_started_s

        # STALL. Commanded forward, not moving. The debounce is what separates
        # contact from a slow start, and `measured_vx` is A's own EKF -- the
        # encoders are the bumper, because this chassis has no bumper.
        #
        # **Only while actually asking for forward motion.** With steering
        # added, A pivots in place when it is badly misaligned: vx is zero by
        # design and the encoders report zero because nothing is translating.
        # Counting that as a stall would confirm a contact that never happened,
        # which is the one failure mode here that looks like success.
        # PERMITTED, not merely commanded. `governed_vx` is what the safety
        # filter actually let through, read back off /cmd_vel. Without it a
        # governor-imposed stop is indistinguishable from a bump: the machine
        # asks for motion, the filter zeroes it, the encoders read nothing, and
        # the debounce forges a delivery against thin air. The measured cone
        # leak pinned A at 0.31-0.35 m -- inside the 0.39 m sighting ceiling --
        # so that forgery was one uninterrupted second away on every run.
        #
        # When the caller supplies nothing, fall back to the bearing test. That
        # keeps the old unit tests meaningful, and the live gate always supplies
        # it.
        if governed_vx is None:
            permitted = abs(self._last_seen_bearing_rad) <= CREEP_MAX_BEARING_RAD
        else:
            permitted = governed_vx > STALL_SPEED_MPS

        # AND THE WHEELS ARE NOT THE WITNESS. The twin authors rear friction at
        # 0.1 and its EKF fuses wheel twist only, so at a real bump the wheels
        # keep turning and `measured_vx` never falls -- measured on the bench,
        # the slip case reaches contact and reports ARRIVED_UNPROVEN forever.
        #
        # `laser_stationary` is the scan matcher's verdict that the ROBOT did
        # not move, which is the quantity that actually matters. Its own author
        # notes it "correctly reports no translation while the wheels spin".
        # Used as a witness only, never fused into control, which is the limit
        # its docstring asks callers to respect. When it is unavailable the
        # encoders stand in, and a slip then costs a delivery rather than
        # forging one -- the safe direction to fail.
        if laser_stationary is None:
            stationary = measured_vx is not None and abs(measured_vx) <= STALL_SPEED_MPS
        else:
            stationary = laser_stationary

        if not permitted or not stationary:
            self._stall_since_s = None
        elif self._stall_since_s is None:
            self._stall_since_s = now_s

        stalled_for = (
            0.0 if self._stall_since_s is None else now_s - self._stall_since_s
        )
        if stalled_for >= STALL_DEBOUNCE_S:
            return self._finish_creep(now_s, elapsed)

        if elapsed >= CREEP_TIMEOUT_S:
            self.state = self.ARRIVED_UNPROVEN
            self.history.append({
                "event": "creep_timeout", "elapsed_s": round(elapsed, 2),
                "last_seen_range_m": self._last_seen_range_m,
                "last_seen_bearing_deg": round(
                    math.degrees(self._last_seen_bearing_rad), 1),
            })
            return {"vx": 0.0, "wz": 0.0, "approach": None,
                    "reason": "creep timed out"}

        # STEER ONTO B. Pure forward motion was the first version's mistake:
        # A arrives mid-turn, so "ahead" is not where B is.
        bearing = self._last_seen_bearing_rad
        aligned = abs(bearing) <= CREEP_MAX_BEARING_RAD
        vx = CREEP_SPEED_MPS * math.cos(bearing) if aligned else 0.0
        wz = CREEP_YAW_GAIN * bearing

        return {
            "vx": vx,
            "wz": wz,
            "approach": {
                "bearing_rad": bearing,
                "range_m": self._last_seen_range_m,
                # The AUTHORED radius, never the fitted one: fitted spans
                # 0.072-0.168 m across 38 runs, so a disc sized from a fit
                # would intermittently be smaller than B and unmask its nose.
                "target_radius_m": self.expected_radius_m,
                "margin_m": 0.10,
            } if self._last_seen_range_m is not None else None,
            "reason": (f"creeping, {elapsed:.1f}s, bearing "
                       f"{math.degrees(bearing):+.0f} deg"
                       + ("" if aligned else " -- turning first")),
        }

    def _finish_creep(self, now_s: float, elapsed: float) -> dict:
        """A stall happened. Was it B?

        The laser cannot answer at the moment of contact -- B is inside its
        minimum range by then -- so the question is whether B was closing when
        it was last visible. A stall with B last seen at arm's length is a
        bump; a stall with B last seen a metre away is something else stopping
        A, and that is `ARRIVED_UNPROVEN`, never a success.
        """

        seen = self._last_seen_range_m
        confirmed = seen is not None and seen <= self.last_sighting_ceiling_m
        self.state = (
            self.DELIVERED_CONFIRMED if confirmed else self.ARRIVED_UNPROVEN
        )
        self.history.append({
            "event": "stall",
            "elapsed_s": round(elapsed, 2),
            "last_seen_range_m": seen,
            "last_sighting_ceiling_m": round(self.last_sighting_ceiling_m, 4),
            "last_seen_bearing_deg": round(
                math.degrees(self._last_seen_bearing_rad), 1),
            "verdict": self.state,
        })
        return {
            "vx": 0.0, "wz": 0.0, "approach": None,
            "reason": ("contact confirmed" if confirmed
                       else f"stalled but B last seen at {seen} m -- unproven"),
        }

    def report(self) -> dict:
        return {
            "state": self.state,
            "refinements": self.refinements,
            "creep": {
                "last_seen_range_m": self._last_seen_range_m,
                "last_sighting_ceiling_m": round(self.last_sighting_ceiling_m, 4),
                "contact_range_m": round(self.contact_range_m, 4),
                "commanded_speed_mps": CREEP_SPEED_MPS,
            },
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
