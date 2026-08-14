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
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
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
        )
        print(f"  dock: containment -- route {route_m:.3f} m, window "
              f"{machine.window_m:.3f} m, arm after {machine.min_travel_m:.3f} m "
              f"of A's OWN travel, detection within "
              f"{machine.max_bearing_error_deg:.0f} deg of A's nose, "
              f"{machine.arm_confirm_k}-of-{machine._arm_frames.maxlen} scans, "
              f"radius unambiguous against the runner-up. No map-frame test.")
        print(f"  dock: final approach {standoff_m:.3f} m from B's centre, "
              f"derived -- the governor's floor and geometric contact, larger wins")
        deadline = time.monotonic() + arguments.timeout
        while time.monotonic() < deadline and not result_future.done():
            rclpy.spin_once(gate, timeout_sec=0.1)
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

            # ARRIVING MEANS STOPPING. Reaching DOCKED used to only stop the
            # machine ISSUING goals, while Nav2 carried on executing the last
            # one it had -- a map-frame goal that keeps drifting. Measured: A
            # came within 0.0993 m of B, docked on a real detection at 0.522 m,
            # and then walked 3.43 m away to chase the stale goal. Cancelling
            # is the difference between arriving and passing through.
            if machine.state == machine.DOCKED:
                print(f"  dock: DOCKED at {gate.last_verdict['candidate']['range_m']:.3f} m "
                      f"from the post -- cancelling the goal so A stays")
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(gate, cancel, timeout_sec=10.0)
                break

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
        dock_report = {"enabled": True, **machine.report()}
    report["docking"] = dock_report

    rclpy.spin_until_future_complete(gate, result_future, timeout_sec=arguments.timeout)
    if result_future.result() is None:
        report["failure"] = f"no action result within {arguments.timeout} s"
        report["travelled_m"] = round(gate.travelled_m(), 3)
        return finish(1)

    status = result_future.result().status
    report["action_status"] = STATUS_NAMES.get(status, status)
    # A goal this gate cancelled ITSELF, because docking said A had arrived, is
    # not a navigation failure. Recording it as one would report a successful
    # delivery as ABORTED.
    if dock_report.get("state") == "DOCKED":
        report["docked_and_cancelled"] = True
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
