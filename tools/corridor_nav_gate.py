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
ROBOT_TARGETS = {
    "robot2": {"namespace": "/robot2", "base_frame": "robot2/base_footprint"},
    "robot1": {"namespace": "", "base_frame": "base_footprint"},
}

#: ADR 0022's pinned delivery tolerance. Printed AND enforced from here.
GOAL_TOLERANCE_M = 0.15

#: How far the delivery goal stands off from B's centre.
#:
#: B is NOT decoration. It carries no PhysicsCollisionAPI, but the RTX lidar
#: sees RENDER geometry, so B appears in /scan and therefore in the costmap as
#: an obstacle. A goal at B's centre is a goal inside its own inflated
#: footprint: unreachable at a 0.15 m tolerance no matter how well the planner
#: and controller behave. That is a delivery robot stopping *inside* the
#: recipient, which is wrong even before it is infeasible.
#:
#: The value clears the obstacle by construction -- B's half-diagonal (0.106 m
#: at robot scale) + robot_radius (0.12) + inflation_radius (0.16) = 0.386 --
#: and is then rounded up to the 0.6 m floor the docking spec pins for the
#: refined goal, so the nominal and refined standoffs are one rule rather than
#: two numbers that can drift apart.
DELIVERY_STANDOFF_M = 0.6

#: nav2_msgs action status codes.
STATUS_NAMES = {2: "EXECUTING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
STATUS_SUCCEEDED = 4


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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, f"{namespace}/navigate_to_pose")

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
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument(
        "--gated",
        action="store_true",
        help="This profile's result is a gate; without it the run is reported only.",
    )
    arguments = parser.parse_args()

    manifest = json.loads(Path(arguments.manifest).read_text(encoding="utf-8"))
    goal_x, goal_y = goal_in_map_frame(manifest, arguments.profile)

    rclpy.init()
    gate = NavGate(ROBOT_TARGETS[arguments.robot])
    report: dict = {
        "robot": arguments.robot,
        "profile": arguments.profile,
        "caveat": arguments.caveat,
        "gated": arguments.gated,
        "goal_map_frame": [round(goal_x, 4), round(goal_y, 4)],
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

    if not gate.client.wait_for_server(timeout_sec=30.0):
        report["failure"] = "navigate_to_pose action server absent; is the nav stack up?"
        return finish(1)

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x = goal_x
    goal.pose.pose.position.y = goal_y
    goal.pose.pose.orientation.w = 1.0

    print(f"goal ({goal_x:.3f}, {goal_y:.3f}) [map] for {arguments.profile}")
    send = gate.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(gate, send, timeout_sec=20.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        report["failure"] = "goal not accepted"
        return finish(1)
    report["goal_accepted"] = True

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(gate, result_future, timeout_sec=arguments.timeout)
    if result_future.result() is None:
        report["failure"] = f"no action result within {arguments.timeout} s"
        report["travelled_m"] = round(gate.travelled_m(), 3)
        return finish(1)

    status = result_future.result().status
    report["action_status"] = STATUS_NAMES.get(status, status)
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
    if status != STATUS_SUCCEEDED:
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
