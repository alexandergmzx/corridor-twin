#!/usr/bin/env python3
"""Gate: Nav2 delivers A to B through the corridor, on the governed path.

    ROS_DOMAIN_ID=67 python3 tools/corridor_nav_gate.py \
        --profile nominal_m6_n3 --out out/evidence/robot-a-gate/nav-nominal.json

Adapted from rasptank-ros2's `tools/test_nav_governed.py` (v2 plan T3.2). The
fleet original is untouched; this is the corridor's question, which is different
from the fleet's.

WHAT IS DIFFERENT, AND WHY
--------------------------
**No obstacle injection.** The fleet script's real subject is the governor
override -- it injects a phantom obstacle mid-goal and proves Nav2's command is
overridden. That was demonstrated in fleet session 6 and is not re-litigated
here. The corridor asks a narrower question: does A complete the delivery, and
how accurately, when the map is being built live around it.

**The enforced bound is 0.15 m, and it is the number that gets printed.** The
fleet script prints "tolerance was 150 mm" and then enforces `err < 0.30` two
lines later. A gate whose stated threshold is twice its enforced one is not the
gate anyone thinks they are reading, so here both come from ONE constant, and
0.15 is the value ADR 0022 pins.

**The goal is computed, not hardcoded.** B's position and A's spawn both live in
the scenario manifest, and each corridor profile spawns A on its own heading. A
goal literal would silently be wrong on two profiles out of three.

Simulator truth is read only to report how far A actually travelled. It never
enters the stack, and no gate decision here depends on it -- the position gate
is measured in the MAP frame via TF with the action status checked, which is the
MicroROS 177 mm anti-pattern (odom-frame pose, unchecked status) refused
explicitly.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

#: Default robot2 so pre-existing artifacts stay reproducible.
NS = "/robot2"

#: robot1 runs at ROOT with unprefixed frames (architecture.md:46-51).
#: `ekf_topic` is A's OWN filtered odometry, which the containment gate
#: integrates for travel. robot1's EKF publishes /odom at root;
#: robot2's is odometry/filtered inside its namespace
#: (bringup_corrected_launch.py:82). Same split as corridor_sim_gate's table.
ROBOT_TARGETS = {
    "robot2": {"namespace": "/robot2", "base_frame": "robot2/base_footprint",
               "ekf_topic": "odometry/filtered"},
    "robot1": {"namespace": "", "base_frame": "base_footprint",
               "ekf_topic": "/odom"},
}

#: ADR 0022's pinned delivery tolerance. Printed AND enforced from here.
GOAL_TOLERANCE_M = 0.15

#: How far the delivery goal stands off from B's centre. Since ADR 0031
#: this lives with the rest of the contact semantics in `corridor_dock`,
#: because the docking bearing cone is derived from it and two homes for
#: one number is how they drift apart.
sys.path.insert(0, str(Path(__file__).parent))
from corridor_dock import DELIVERY_STANDOFF_M  # noqa: E402

#: nav2_msgs action status codes.
STATUS_NAMES = {2: "EXECUTING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
STATUS_SUCCEEDED = 4

#: The safety governor's two topics, ABSOLUTE because the governor is not
#: namespaced: `safety_launch.py` declares it with no `namespace=`, and the node
#: subscribes to a literal `/scan` and `/cmd_vel_raw`.
#:
#: The creep drives through `/cmd_vel_raw` like every other motion in this
#: project. Publishing to `/cmd_vel` would bypass the governor entirely, which
#: is the failure the governor logs a warning about on every startup.
GOVERNOR_CMD_TOPIC = "/cmd_vel_raw"
#: `~/docking_approach` on the governor node, resolved. ADR 0033: the governor
#: is informed of the terminal approach, never bypassed for it.
GOVERNOR_DOCKING_TOPIC = "/cmd_vel_governor/docking_approach"
#: `~/docking_disc` on the same node. A SEPARATE topic from the cone, because
#: the third field changes meaning between them -- margin there, target radius
#: here -- and a stale sender's margin must never be read as a radius.
GOVERNOR_DISC_TOPIC = "/cmd_vel_governor/docking_disc"
#: What the safety filter actually PERMITTED, read back. Subscribing to
#: `/cmd_vel` is not a bypass: nothing is published here, and the creep still
#: drives through `/cmd_vel_raw`. It is the only way to tell "the governor
#: stopped me" from "I hit something", which are otherwise identical to a
#: robot whose encoders read zero either way.
GOVERNED_CMD_TOPIC = "/cmd_vel"

#: How long a window of laser odometry the stationary witness looks at.
LASER_WITNESS_WINDOW_S = 2.0
#: Below this median speed the scan matcher says the ROBOT did not move.
#:
#: Provenance, and its weakness. Replay of session bag 20260814-003844 puts the
#: matcher's stationary 1-second displacement at a median of 16.8 mm but a p95
#: of 374 mm -- ICP re-registration jumps, which is why this is a MEDIAN over
#: per-sample pairs and not a mean or a maximum. A creep exempt from the slow
#: zone runs at the full 0.05 m/s clamp, so the separation is 16.8 mm/s
#: stationary against 50 mm/s moving and this sits between them.
#:
#: It is not a wide margin and it has not been validated live. Failing the
#: wrong way costs a delivery (ARRIVED_UNPROVEN), never a forged one, because
#: this witness can only ever WITHHOLD a confirmation -- see `DockingMachine.creep`.
LASER_STATIONARY_EPS_MPS = 0.030
#: Fewer pairs than this in the window and the witness abstains rather than
#: guessing from two samples.
LASER_WITNESS_MIN_PAIRS = 6


def laser_stationary_from_track(track, now_s: float) -> bool | None:
    """Did the robot move, per the scan matcher? None means "cannot say".

    A MEDIAN over consecutive-sample speeds, not a mean and not a maximum. The
    matcher re-registers occasionally and jumps: replay of bag 20260814-003844
    puts its stationary p95 at 374 mm against a median of 16.8 mm, so any
    statistic sensitive to the tail calls a parked robot moving, and a contact
    that really happened is never confirmed.

    Abstains when the matcher is silent or the window is too thin to have a
    median worth the name. Abstaining is not a safety hole: the caller falls
    back to the encoders, and this witness can only ever WITHHOLD a
    confirmation, never grant one.

    Pure, so the tail behaviour can be tested against synthetic tracks rather
    than hoped for on a robot.
    """

    window = [s for s in track if now_s - s[0] <= LASER_WITNESS_WINDOW_S]
    speeds = []
    for (t0, x0, y0), (t1, x1, y1) in zip(window, window[1:], strict=False):
        dt = t1 - t0
        if dt > 1e-6:
            speeds.append(math.dist((x0, y0), (x1, y1)) / dt)
    if len(speeds) < LASER_WITNESS_MIN_PAIRS:
        return None
    return statistics.median(speeds) < LASER_STATIONARY_EPS_MPS


#: Dock states reachable only by way of the handoff.
_PAST_HANDOFF = ("DOCKING", "DELIVERED_CONFIRMED", "ARRIVED_UNPROVEN")


def delivery_reconciliation(dock_report: dict, action_status: str):
    """-> (excuse_the_cancel, failure_or_None). Pure, so it is testable.

    Two questions about the same moment, and they used to share one branch.

    **Was the cancel ours?** A goal this gate cancelled ITSELF at the handoff is
    not a navigation failure; recording it as one reports a delivery as ABORTED.

    **Did the handoff happen at all?** It did not, on run 20260814-031922, and
    NOTHING SAID SO. Nav2 reported SUCCEEDED at 0.6621 m from B -- 0.198 m off
    its own refined goal -- while the handoff only fires on a confirmed sighting
    inside `docked_max_range_m` (0.620 m). The machine sat in REFINE with zero
    creep ticks, the dock loop's exit condition was satisfied, control fell
    through to reporting, and the run's only complaint was an unrelated
    map-frame goal error. The entire terminal phase was skipped in silence.

    That is systematic rather than unlucky: `GOAL_TOLERANCE_M` and Nav2's own
    `xy_goal_tolerance` are both 0.15, so the SUCCEEDED envelope and the handoff
    radius move together and no choice of standoff creates margin between them.

    Guarded to require creep_ticks == 0 AND a state outside TERMINAL, so a run
    that really delivered cannot trip it -- the three that handed off recorded
    3416 to 3496 ticks.
    """

    if not dock_report.get("enabled"):
        return False, None

    state = dock_report.get("state")
    excuse = state in _PAST_HANDOFF
    if excuse or dock_report.get("creep_ticks"):
        return excuse, None
    if action_status != "SUCCEEDED":
        # An abort short of the handoff is already reported by the status check.
        return excuse, None

    creep = dock_report.get("creep") or {}
    seen = creep.get("last_seen_range_m")
    ceiling = creep.get("last_sighting_ceiling_m")
    return excuse, (
        f"Nav2 reported SUCCEEDED and the handoff never fired: state {state}, "
        f"creep_ticks 0, last detected range {seen} m against a handoff radius "
        f"of {ceiling} m. The refined goal's SUCCEEDED envelope and that radius "
        f"are the same number, so the terminal phase was skipped in silence."
    )


def route_to_delivery_m(manifest: dict, profile: str) -> float:
    """Path length from A's spawn to the delivery, from the manifest's own legs.

    **The previous version dropped a leg on a false premise.** It excluded
    `departure_length_m` because "the departure leg runs PAST B". It does not.
    `scene.trajectory.DeliveryTrajectory.segments` orders the five pieces
    approach -> corner arc -> **departure** -> delivery arc -> delivery, and the
    route ENDS at B: measured, the station closest to B is the full trajectory
    length at a distance of 0.0000 m. The departure leg is the third of five and
    lies entirely before the delivery.

    So the sum under-reported the route by 1.631 m at the committed scale --
    5.750 against 7.380 -- and `min_travel_m` therefore unlocked arming
    **2.531 m** before B rather than the 0.900 m its own window implied.
    """

    legs = manifest["profiles"][profile]["delivery_trajectory"]
    return (
        legs["approach_length_m"]
        + legs["arc_radius_m"] * legs["arc_sweep_rad"]
        + legs["departure_length_m"]
        + legs["delivery_arc_radius_m"] * legs["delivery_arc_sweep_rad"]
        + legs["delivery_length_m"]
    )


def delivery_standoff_world(
    manifest: dict, standoff_m: float = DELIVERY_STANDOFF_M
) -> tuple[float, float]:
    """Where A should stop to deliver: beside B, not on top of it.

    The direction is taken from the street's own centreline, NOT from the
    authored delivery route. B stands against the east wall
    (`geometry.person_b_xyz`), so the reachable free space is toward the lane,
    and that is derivable from `next_street` alone. Reading the authored
    route's final heading would work equally well and would be exactly the
    "authored line and waypoints" ADR 0022:15-17 keeps out of A's navigation.
    """

    b_x, b_y, _b_z = manifest["actors"]["b_xyz_m"]
    lane_offset = float(manifest["next_street"]["center_x_m"]) - float(b_x)
    # B is against the east wall, so the lane is west; copysign keeps this
    # correct if a future scenario ever mirrors the street.
    direction = math.copysign(1.0, lane_offset) if lane_offset else -1.0
    return (float(b_x) + direction * standoff_m, float(b_y))


def delivery_facing_world(
    manifest: dict, standoff_m: float = DELIVERY_STANDOFF_M
) -> float:
    """Which way A should be pointing when it delivers: at B.

    Derived from the same two manifest facts as the standoff itself -- B's
    position and the street centreline -- and deliberately NOT from the
    authored route's final heading, which numerically agrees (the standoff sits
    on B's approach ray, so it must) but is the "authored line and waypoints"
    ADR 0022:15-17 keeps out of A's navigation. The agreement is asserted in
    the tests as a property; it is not the derivation.
    """

    b_x, b_y, _b_z = manifest["actors"]["b_xyz_m"]
    stand_x, stand_y = delivery_standoff_world(manifest, standoff_m)
    return math.atan2(float(b_y) - stand_y, float(b_x) - stand_x)


def goal_yaw_in_map_frame(
    manifest: dict, profile: str, standoff_m: float = DELIVERY_STANDOFF_M
) -> float:
    """`delivery_facing_world` expressed in the map frame, for one profile.

    The map frame is anchored on A's spawn pose, so a map yaw of zero means
    "A's spawn heading" -- which is +7.13 deg of world on `nominal_m6_n3` and
    +3.58 on `wide_corner_m6_n4_5`. The identity quaternion this replaced was
    therefore not a neutral default; it was an instruction to finish on the
    heading A started on, which is only correct on `uniform_m6_n6` by accident.

    What this does NOT fix: A arrives mid-turn, 51-79 deg from any sensible
    delivery heading, and with a 1.3 m map-frame position error it never gets
    to rotate. Measured, the correction moves the yaw error from 58.5-85.7 deg
    to 51.4-78.6 against a 34.4 deg tolerance -- still failing. See
    docs/evidence/robot-a-gate/NOTES-why-A-overshoots-B-20260813.md. This is a
    correctness fix, not the fix for the delivery.
    """

    entry = manifest["profiles"][profile]
    heading_x, heading_y = entry["delivery_trajectory"]["approach_heading"]
    spawn_yaw = math.atan2(float(heading_y), float(heading_x))
    delta = delivery_facing_world(manifest, standoff_m) - spawn_yaw
    return math.atan2(math.sin(delta), math.cos(delta))


def goal_in_map_frame(manifest: dict, profile: str) -> tuple[float, float]:
    """The delivery standoff expressed in the map frame, for one profile.

    SLAM's map frame is anchored where the robot started, so the goal is B
    relative to A's spawn, rotated into A's spawn heading. Both the spawn and
    the heading are per-profile, which is why this is computed rather than
    written down: the three profiles start A on different headings (7.13, 3.58
    and 0.00 degrees), so one literal would be quietly wrong on two of them.
    """

    b_x, b_y = delivery_standoff_world(manifest)
    entry = manifest["profiles"][profile]
    a_x, a_y, _a_z = entry["a_start_xyz_m"]
    heading_x, heading_y = entry["delivery_trajectory"]["approach_heading"]
    yaw = math.atan2(float(heading_y), float(heading_x))

    delta_x, delta_y = float(b_x) - float(a_x), float(b_y) - float(a_y)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return (
        delta_x * cos_yaw + delta_y * sin_yaw,
        -delta_x * sin_yaw + delta_y * cos_yaw,
    )


class NavGate(Node):
    def __init__(self, target: dict | None = None) -> None:
        super().__init__("corridor_nav_gate")
        self.target = target or ROBOT_TARGETS["robot2"]
        namespace = self.target["namespace"]
        self.truth: tuple[float, float] | None = None
        self.start_xy: tuple[float, float] | None = None
        self.create_subscription(
            Odometry, f"{namespace}/sim/ground_truth", self._on_truth, 10
        )
        # A's OWN integrated travel, from the EKF. Truth is above and is
        # report-only (CLAUDE.md invariant 1): the containment that gates
        # docking must be computable by the robot, so it is computed from the
        # filter A already navigates on.
        self.odom_travel_m = 0.0
        self._last_odom_xy: tuple[float, float] | None = None
        ekf_topic = self.target.get("ekf_topic", "/odom")
        if not ekf_topic.startswith("/"):
            ekf_topic = f"{namespace}/{ekf_topic}"
        self.create_subscription(Odometry, ekf_topic, self._on_odom, 20)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, f"{namespace}/navigate_to_pose")
        #: Set by main() when --dock is on. BEST_EFFORT because the twin offers
        #: /scan with sensor QoS and a RELIABLE subscription matches nothing.
        self.detector = None
        self.last_verdict = None
        # TERMINAL CREEP (ADR 0033). `/cmd_vel_raw`, never `/cmd_vel`: the creep
        # is governed like every other motion here, and publishing to /cmd_vel
        # would bypass the safety filter -- the thing it warns about on startup.
        self.measured_vx: float | None = None
        self.creeping = False
        # Counted, because "the mask never applied" and "the creep never
        # ticked" look identical from the outside and cost two runs to tell
        # apart. A creep whose ticks and publishes disagree with its elapsed
        # time says so in the artifact now.
        self.creep_ticks = 0
        self.approach_publishes = 0
        # BOTH TOPICS ARE ABSOLUTE, and deliberately not built from `namespace`.
        #
        # The governor is not namespaced. `safety_launch.py` declares the node
        # with no `namespace=`, and the node itself subscribes to a literal
        # `/scan` and `/cmd_vel_raw`, so it lives at `/cmd_vel_governor` whatever
        # robot it is governing. Deriving these from this gate's namespace was
        # correct for robot1 only by coincidence -- robot1's namespace is the
        # empty string -- and would have published into the void for robot2,
        # which looks exactly like "the mask never applied".
        #
        # `~/docking_approach` on the governor resolves to the absolute name
        # below. If the governor ever gains a namespace, these two constants and
        # that launch file move together or the creep silently loses its mask.
        self.cmd_pub = self.create_publisher(Twist, GOVERNOR_CMD_TOPIC, 10)
        self.approach_pub = self.create_publisher(
            Vector3Stamped, GOVERNOR_DOCKING_TOPIC, 10
        )
        self.disc_pub = self.create_publisher(
            Vector3Stamped, GOVERNOR_DISC_TOPIC, 10
        )
        # THE TWO WITNESSES. Neither is the wheels, and that is the point.
        #
        # `/cmd_vel` is read, never written: it says what the safety filter
        # PERMITTED, which is the only thing that distinguishes "the governor
        # stopped me" from "I hit something". Without it a governor stop forges
        # a delivery, and the leak pinned A inside the sighting ceiling on every
        # run last night, so the forgery was one second away each time.
        self.governed_vx: float | None = None
        self.create_subscription(Twist, GOVERNED_CMD_TOPIC, self._on_governed, 10)
        # `/odom_laser` is the scan matcher. The twin authors rear friction at
        # 0.1 and the EKF fuses wheel twist only, so at a real bump the wheels
        # spin and the encoders never report the stop.
        self._laser_track: deque = deque(maxlen=64)
        self.create_subscription(
            Odometry, f"{namespace}/odom_laser", self._on_odom_laser, 50
        )
        if True:
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import LaserScan

            self.create_subscription(
                LaserScan, f"{namespace}/scan", self._on_scan,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
            )

    def _on_odom(self, message: Odometry) -> None:
        """Path length, not displacement: a robot that goes out and comes back
        has still travelled, and the containment asks how far it has driven."""

        position = message.pose.pose.position
        here = (position.x, position.y)
        if self._last_odom_xy is not None:
            self.odom_travel_m += math.dist(self._last_odom_xy, here)
        self._last_odom_xy = here
        # A's OWN measured forward speed. This is the bumper: the terminal creep
        # calls contact when it is commanding motion and this reports none.
        self.measured_vx = message.twist.twist.linear.x

    def drive(self, vx: float, wz: float, approach: dict | None) -> None:
        """Command the creep, and tell the governor what it is for.

        Both halves go out together and neither is optional. The velocity goes
        to `/cmd_vel_raw`, so it passes through the safety filter exactly like
        every other motion in this project -- publishing to `/cmd_vel` would
        bypass the governor, which is what the governor warns about on startup.

        The approach is what makes the mask legitimate. Without it the governor
        brakes at 0.35 m and the contact never happens; with it, the governor
        stops braking for ONE object in a 15 degree cone and keeps every other
        protection live. Republished every tick, because the mask expires on
        silence -- that is how a crashed docking controller releases it.
        """

        command = Twist()
        command.linear.x = float(vx)
        # AND THE YAW. The creep steers onto B: A arrives mid-turn, 51-79 deg
        # off the delivery heading, so pure forward motion drives a tangent.
        # Run 20260814-003034 did exactly that -- 25 s of creep during which B
        # went from 0.6133 m to 0.6335 m away.
        command.angular.z = float(wz)
        self.cmd_pub.publish(command)

        if approach is None:
            return
        declaration = Vector3Stamped()
        declaration.header.stamp = self.get_clock().now().to_msg()
        declaration.vector.x = float(approach["bearing_rad"])
        declaration.vector.y = float(approach["range_m"])
        declaration.vector.z = float(approach["margin_m"])
        self.approach_pub.publish(declaration)

        # AND THE DISC, which is the one the governor actually acts on. The cone
        # above is still published so a governor predating the disc keeps its
        # old behaviour rather than losing the mask entirely; a governor that
        # understands both prefers this one. The third field is the target's
        # AUTHORED radius here, not a margin -- the margin is the filter's own
        # parameter, because slack on a safety mask is not the caller's to set.
        if approach.get("target_radius_m") is not None:
            disc = Vector3Stamped()
            disc.header.stamp = declaration.header.stamp
            disc.vector.x = float(approach["bearing_rad"])
            disc.vector.y = float(approach["range_m"])
            disc.vector.z = float(approach["target_radius_m"])
            self.disc_pub.publish(disc)

    def _on_governed(self, message: Twist) -> None:
        """What the safety filter let through. Read-only; nothing publishes here."""

        self.governed_vx = float(message.linear.x)

    def _on_odom_laser(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self._laser_track.append(
            (time.monotonic(), float(position.x), float(position.y))
        )

    def laser_stationary(self) -> bool | None:
        """Did the ROBOT move, according to the scan matcher? None = cannot say."""

        return laser_stationary_from_track(self._laser_track, time.monotonic())

    def _on_scan(self, message) -> None:
        if self.detector is None:
            return
        self.last_verdict = self.detector.feed(
            message.ranges, message.angle_min, message.angle_increment,
            message.range_min, message.range_max,
        )

    def map_pose_yaw(self) -> tuple[float, float, float]:
        """(x, y, yaw) of base_footprint in map. A's own localization, not truth."""

        transform = self.tf_buffer.lookup_transform(
            "map", self.target["base_frame"], rclpy.time.Time()
        ).transform
        q = transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return transform.translation.x, transform.translation.y, yaw

    def _on_truth(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.truth = (position.x, position.y)
        if self.start_xy is None:
            self.start_xy = self.truth

    def travelled_m(self) -> float:
        if self.truth is None or self.start_xy is None:
            return 0.0
        return math.hypot(self.truth[0] - self.start_xy[0], self.truth[1] - self.start_xy[1])

    def map_pose(self) -> tuple[float, float]:
        transform = self.tf_buffer.lookup_transform(
            "map", self.target["base_frame"], rclpy.time.Time()
        )
        return transform.transform.translation.x, transform.transform.translation.y


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--profile", required=True)
    parser.add_argument(
        "--caveat",
        default="",
        help="Stamped into the artifact when a precondition failed but the run proceeded.",
    )
    parser.add_argument("--robot", choices=sorted(ROBOT_TARGETS), default="robot2")
    parser.add_argument("--manifest", default="out/corridor.manifest.json")
    parser.add_argument("--out", required=True)
    parser.add_argument(
        "--dock",
        action="store_true",
        help="drive the final approach from the LANDMARK rather than the map goal",
    )
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument(
        "--gated",
        action="store_true",
        help="This profile's result is a gate; without it the run is reported only.",
    )
    arguments = parser.parse_args()

    manifest = json.loads(Path(arguments.manifest).read_text(encoding="utf-8"))
    goal_x, goal_y = goal_in_map_frame(manifest, arguments.profile)
    goal_yaw = goal_yaw_in_map_frame(manifest, arguments.profile)

    rclpy.init()
    gate = NavGate(ROBOT_TARGETS[arguments.robot])
    report: dict = {
        "robot": arguments.robot,
        "profile": arguments.profile,
        "caveat": arguments.caveat,
        "gated": arguments.gated,
        "goal_map_frame": [round(goal_x, 4), round(goal_y, 4)],
        "goal_map_yaw_deg": round(math.degrees(goal_yaw), 4),
        "tolerance_m": GOAL_TOLERANCE_M,
    }

    def finish(code: int) -> int:
        report["pass"] = code == 0
        destination = Path(arguments.out)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        print(f"\nwritten: {destination}")
        gate.destroy_node()
        rclpy.shutdown()
        # A non-gated profile reports rather than gates (ADR 0022 gates
        # nominal_m6_n3 and wide_corner_m6_n4_5 only).
        return code if arguments.gated else 0

    if arguments.dock:
        radius = manifest.get("actors", {}).get("b_radius_m")
        if radius:
            sys.path.insert(0, str(Path(__file__).parent))
            from landmark_detector import LandmarkDetector

            gate.detector = LandmarkDetector(radius)
            print(f"  dock: armed, expecting a landmark of radius {radius} m")
        else:
            print("  dock: requested but the scene authors no landmark; transit only")

    if not gate.client.wait_for_server(timeout_sec=30.0):
        report["failure"] = "navigate_to_pose action server absent; is the nav stack up?"
        return finish(1)

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x = goal_x
    goal.pose.pose.position.y = goal_y
    # Facing B, not facing A's spawn heading. `orientation.w = 1.0` is map yaw
    # zero, which the map frame defines as the spawn heading -- an accidental
    # instruction, wrong by 7.13 deg on nominal and 3.58 on wide_corner.
    goal.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

    print(
        f"goal ({goal_x:.3f}, {goal_y:.3f}) yaw {math.degrees(goal_yaw):+.2f} deg "
        f"[map] for {arguments.profile}"
    )

    # SEND IT MORE THAN ONCE. bt_navigator reports ACTIVE to the runner's
    # lifecycle poll, the hold-check sees no abort, and the goal arriving
    # moments later is still answered "Action server is inactive. Rejecting the
    # goal." Three runs died that way on 2026-08-12 -- the robot was never given
    # an instruction and never moved -- and each was a whole Isaac session spent
    # to discover that the stack needed another few seconds.
    #
    # The 45 s wait below is a different failure and stays: bt_navigator can
    # ACCEPT the goal and have its response miss a client that asked too early
    # ("Failed to send goal response (timeout)"), which this gate then reported
    # as "goal not accepted" -- a nav failure that never happened.
    #
    # Bounded at three. A stack that has not activated in three tries thirty
    # seconds apart is not activating, and that is infrastructure for the runner
    # to classify rather than something to keep asking about.
    handle = None
    attempts = 3
    for attempt in range(1, attempts + 1):
        send = gate.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(gate, send, timeout_sec=45.0)
        handle = send.result()
        if handle is not None and handle.accepted:
            report["goal_send_attempts"] = attempt
            break
        if attempt < attempts:
            print(f"  goal not accepted on attempt {attempt}/{attempts}; the "
                  f"stack may still be activating -- waiting 10 s and asking again")
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                rclpy.spin_once(gate, timeout_sec=0.1)
    if handle is None or not handle.accepted:
        report["failure"] = "goal not accepted"
        report["goal_send_attempts"] = attempts
        return finish(1)
    report["goal_accepted"] = True

    result_future = handle.get_result_async()

    # --- terminal docking ----------------------------------------------------
    # While Nav2 executes the transit, watch for B's landmark and, once it is
    # confirmed, re-aim at where A can SEE B rather than where the map thinks B
    # is. Every metre is still driven by Nav2; this only chooses the goal.
    dock_report = {"enabled": False}
    if arguments.dock and gate.detector is not None:
        from corridor_dock import DockingMachine, facing_yaw, final_approach_m

        route_m = route_to_delivery_m(manifest, arguments.profile)
        actors = manifest.get("actors", {})
        # ADR 0031: derived from B's radius and A's length, never authored.
        standoff_m = final_approach_m(
            float(actors["b_radius_m"]), float(actors["a_size_xyz_m"][0])
        )
        machine = DockingMachine(
            nominal_goal=(goal_x, goal_y),
            standoff_m=standoff_m,
            route_length_m=route_m,
            expected_radius_m=float(actors["b_radius_m"]),
            a_length_m=float(actors["a_size_xyz_m"][0]),
        )
        print(f"  dock: containment -- route {route_m:.3f} m, window "
              f"{machine.window_m:.3f} m, arm after {machine.min_travel_m:.3f} m "
              f"of A's OWN travel, detection within "
              f"{machine.max_bearing_error_deg:.0f} deg of A's nose, "
              f"{machine.arm_confirm_k}-of-{machine._arm_frames.maxlen} scans, "
              f"radius unambiguous against the runner-up. No map-frame test.")
        print(f"  dock: final approach {standoff_m:.3f} m from B's centre, "
              f"derived -- the governor's floor and geometric contact, larger wins")
        print(f"  dock: handoff at {machine.docked_max_range_m:.3f} m, then CREEP to "
              f"contact at {machine.contact_range_m:.3f} m. B goes invisible at "
              f"{0.12 + float(actors['b_radius_m']):.3f} m, so the last "
              f"{0.12 + float(actors['b_radius_m']) - machine.contact_range_m:.3f} m "
              f"is blind and the encoders are the bumper.")
        deadline = time.monotonic() + arguments.timeout
        # THE LOOP MUST OUTLIVE THE NAV2 GOAL.
        #
        # `result_future.done()` was the only exit besides the deadline, and the
        # handoff CANCELS the goal -- which completes that future. So the first
        # run with a creep in it logged `creep_begin` and then fell straight out
        # of the loop on the next iteration, leaving the machine stuck in
        # DOCKING with the robot untouched. The cancel is not the end of the
        # delivery any more; it is the middle of it.
        #
        # Once creeping, the docking controller owns the exit: it ends on stall,
        # on its own timeout, or on this deadline.
        while time.monotonic() < deadline and not (
            result_future.done() and not gate.creeping
        ):
            rclpy.spin_once(gate, timeout_sec=0.1)

            # THE CREEP RUNS BEFORE THE TF LOOKUP, AND WITHOUT IT.
            #
            # This block used to sit below `map_pose_yaw()`, whose `except:
            # continue` skips the rest of the iteration on any TF gap. So every
            # gap silently cost a creep tick -- no velocity, and no approach
            # republished, which lets the governor's mask EXPIRE and puts the
            # proximity floor straight back. Run 20260814-003844 crept from
            # 0.614 m to 0.346 m and then asymptoted against a floor that
            # should not have been there, with the governor logging "obstacle
            # at 0.24 m" throughout.
            #
            # It is also wrong in principle. The terminal phase is map-free by
            # design -- range and bearing in the laser frame, nothing else --
            # so gating it on a map-frame transform gives away the property
            # that made it worth building.
            if gate.creeping:
                command = machine.creep(
                    gate.last_verdict, gate.measured_vx, time.monotonic(),
                    governed_vx=gate.governed_vx,
                    laser_stationary=gate.laser_stationary(),
                )
                if command is not None:
                    gate.creep_ticks += 1
                    if command["approach"] is not None:
                        gate.approach_publishes += 1
                    gate.drive(command["vx"], command["wz"], command["approach"])
                if machine.state in machine.TERMINAL:
                    gate.drive(0.0, 0.0, None)     # leave it stopped, always
                    print(f"  dock: {machine.state} -- {command['reason']}")
                    break
                continue

            try:
                pose_x, pose_y, pose_yaw = gate.map_pose_yaw()
            except Exception:  # noqa: BLE001 - TF gaps are normal mid-transit
                continue
            # The pose is used to EXPRESS the goal, never to decide whether to
            # dock. Since 2026-08-13 `armed()` cannot read it even by accident:
            # it is not passed one.
            refined = machine.step(
                (pose_x, pose_y), pose_yaw, gate.last_verdict,
                travelled_m=gate.odom_travel_m,
            )

            # THE HANDOFF. Nav2's part of the delivery ends here and the creep
            # takes the robot (ADR 0033).
            #
            # Cancelling first is not tidiness -- it is the whole reason the
            # handoff is a discrete event. Two controllers publishing motion at
            # once is the failure that produced the 3.43 m walk-away: reaching
            # the band used to stop the machine ISSUING goals while Nav2 carried
            # on executing the last one it had, a map-frame goal that keeps
            # drifting. The creep must own the robot outright before it commands
            # a single centimetre.
            if machine.state == machine.DOCKING and not gate.creeping:
                seen = gate.last_verdict["candidate"]["range_m"]
                print(f"  dock: HANDOFF at {seen:.3f} m -- cancelling the Nav2 goal, "
                      f"creeping to contact at {machine.contact_range_m:.3f} m")
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(gate, cancel, timeout_sec=10.0)
                gate.creeping = True

            if refined is None:
                continue
            print(f"  dock: refine {machine.refinements} -> "
                  f"({refined[0]:.3f}, {refined[1]:.3f}) [map], "
                  f"landmark seen at {gate.last_verdict['candidate']['range_m']:.3f} m")
            goal.pose.pose.position.x, goal.pose.pose.position.y = refined
            # AND ITS ORIENTATION. Overwriting only x/y left every refined goal
            # carrying the TRANSIT goal's yaw -- an angle derived for the
            # nominal standoff, not for the point the detector just chose. It
            # is W1's defect one level down: an orientation picked for a
            # different position. Recomputed here from the landmark the goal
            # was derived from, by the same rule W1 applies to the transit
            # goal, so both mean "facing B" rather than one meaning it by
            # construction and the other by luck.
            refined_yaw = facing_yaw(refined, machine.landmark_map)
            goal.pose.pose.orientation.z = math.sin(refined_yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(refined_yaw / 2.0)
            send = gate.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(gate, send, timeout_sec=20.0)
            new_handle = send.result()
            if new_handle is not None and new_handle.accepted:
                handle = new_handle
                result_future = handle.get_result_async()
                goal_x, goal_y = refined
        dock_report = {
            "enabled": True,
            "creep_ticks": gate.creep_ticks,
            "approach_publishes": gate.approach_publishes,
            **machine.report(),
        }
    report["docking"] = dock_report

    rclpy.spin_until_future_complete(gate, result_future, timeout_sec=arguments.timeout)
    if result_future.result() is None:
        report["failure"] = f"no action result within {arguments.timeout} s"
        report["travelled_m"] = round(gate.travelled_m(), 3)
        return finish(1)

    status = result_future.result().status
    report["action_status"] = STATUS_NAMES.get(status, status)
    # A goal this gate cancelled ITSELF is not a navigation failure. Recording
    # it as one would report a successful delivery as ABORTED.
    #
    # Since ADR 0033 the cancel happens at the HANDOFF rather than at arrival,
    # so it precedes the creep instead of ending the run. Every state reachable
    # only by way of that handoff has to be listed here, and `DOCKED` -- the
    # state this used to test for -- no longer exists.
    excuse_cancel, handoff_failure = delivery_reconciliation(
        dock_report, report["action_status"])
    if excuse_cancel:
        report["docked_and_cancelled"] = True
    dock_report["handoff"] = {
        "fired": bool(excuse_cancel or dock_report.get("creep_ticks")),
        "creep_ticks": dock_report.get("creep_ticks", 0),
    }
    report["travelled_m"] = round(gate.travelled_m(), 3)

    try:
        pose_x, pose_y = gate.map_pose()
    except Exception as error:  # noqa: BLE001 - TF failure is a gate failure, reported
        report["failure"] = f"no map-frame pose via TF: {error}"
        return finish(1)

    error_m = math.hypot(pose_x - goal_x, pose_y - goal_y)
    report["final_pose_map_frame"] = [round(pose_x, 4), round(pose_y, 4)]
    report["goal_error_m"] = round(error_m, 4)

    failures = []
    if handoff_failure:
        failures.append(handoff_failure)
        print(f"  ** {handoff_failure} **")
    if status != STATUS_SUCCEEDED and not report.get("docked_and_cancelled"):
        failures.append(f"action status {report['action_status']}, not SUCCEEDED")
    if error_m > GOAL_TOLERANCE_M:
        failures.append(
            f"map-frame goal error {error_m * 1000:.0f} mm exceeds "
            f"{GOAL_TOLERANCE_M * 1000:.0f} mm (ADR 0022)"
        )
    report["failures"] = failures

    print(
        f"status {report['action_status']}; final map pose "
        f"({pose_x:.3f}, {pose_y:.3f}); error {error_m * 1000:.0f} mm "
        f"against a {GOAL_TOLERANCE_M * 1000:.0f} mm tolerance"
    )
    return finish(1 if failures else 0)


if __name__ == "__main__":
    sys.exit(main())
