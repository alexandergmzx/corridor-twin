#!/usr/bin/env python3
"""Which yaw does the SLAM input chain actually carry, and does it lie?

    ROS_DOMAIN_ID=67 python3 tools/corridor_yaw_audit.py \
        --out out/evidence/robot-a-gate/yaw-audit.json

BENCH TOOLING. This commands a pivot on /cmd_vel_raw and is therefore NEVER
valid during a mission run -- the mission's motion is Nav2's alone. It exists to
answer a bring-up question that no mission run can answer, because a mission run
does not pivot on the spot.

WHY THIS RATHER THAN check_odom_vs_imu.py
-----------------------------------------
The fleet instrument compares `/odom_raw`'s yaw rate against the IMU gyro from a
bag. That answers "does the wheel odometry lie?" -- known to be yes on this twin,
~2.8-3.0x under pivot slip. It does NOT answer the question that matters here,
which is about a different signal:

slam_toolbox consumes `/scan` plus the TF `odom -> base_footprint`. That
transform is published by `ekf_filter_node` alone (`ekf_sim_pnfix.yaml:93-99`;
nothing else in the corridor bring-up broadcasts it, and `laser_odometry`
publishes only the `/odom_laser` TOPIC). The pn-fix EKF takes vx from the wheels
(`odom0_config` index 6, `:117-122`) and yaw rate from the IMU (`imu0_config`
index 11, `:150-155`) -- so wheel yaw is structurally absent from the chain.

"Structurally absent" is a claim about a config file. This measures it.

THE THREE SIGNALS
-----------------
  * `/odom_raw`  -- the wheel odometry. Expected to over-report; it is the
                    NEGATIVE CONTROL. If it does not lie, the pivot did not
                    slip and the run proves nothing.
  * `/odom`      -- the EKF output, and the pose behind the TF SLAM consumes.
                    This is the signal under test.
  * `/sim/ground_truth` -- evaluation input only (CLAUDE.md invariant 1).

A run in which `/odom_raw` is badly wrong and `/odom` is faithful is the result
that clears the launch assembly. A run in which BOTH are wrong points at the
IMU or the EKF and means no SLAM or planner tuning should happen yet. A run in
which both are right is inconclusive about the chain and says only that this
pivot did not slip.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

#: A pivot slow enough that the governor has no reason to intervene and fast
#: enough that wheel slip is provoked. Slip is the phenomenon under test: a
#: pivot that does not slip cannot distinguish a faithful chain from a lucky one.
PIVOT_RAD_S = 0.6
PIVOT_S = 8.0

#: The EKF's yaw may differ from truth by sensor noise and by integration error
#: over the pivot. It may not differ by a FACTOR. This threshold separates
#: "noisy" from "carrying the wheel-yaw error", which is the only distinction
#: this audit needs to make; the observed wheel error is ~2.8-3.0x.
EKF_RATIO_TOLERANCE = 0.15


def yaw_from(message: Odometry) -> float:
    """Planar yaw from a quaternion, without importing a transform library."""

    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def total_turned_rad(yaws: list[float]) -> float:
    """Unwrapped cumulative rotation along a yaw track.

    Summed as absolute per-step deltas rather than as (last - first): a pivot
    can pass through +/-pi, and a net difference would silently discard whole
    turns. Each step is wrapped into (-pi, pi] first, which is valid because no
    sample interval here is anywhere near half a revolution.
    """

    total = 0.0
    for earlier, later in zip(yaws, yaws[1:], strict=False):
        delta = (later - earlier + math.pi) % (2.0 * math.pi) - math.pi
        total += abs(delta)
    return total


class YawAudit(Node):
    def __init__(self) -> None:
        super().__init__("corridor_yaw_audit")
        self.tracks: dict[str, list[float]] = {"odom_raw": [], "odom": [], "truth": []}
        self.create_subscription(
            Odometry, "/odom_raw", lambda m: self.tracks["odom_raw"].append(yaw_from(m)), 10
        )
        self.create_subscription(
            Odometry, "/odom", lambda m: self.tracks["odom"].append(yaw_from(m)), 10
        )
        self.create_subscription(
            Odometry,
            "/sim/ground_truth",
            lambda m: self.tracks["truth"].append(yaw_from(m)),
            10,
        )
        self.publisher = self.create_publisher(Twist, "/cmd_vel_raw", 10)

    def pivot(self, rad_s: float, seconds: float) -> None:
        command = Twist()
        command.angular.z = rad_s
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)
        for _ in range(10):
            self.publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--rad-s", type=float, default=PIVOT_RAD_S)
    parser.add_argument("--seconds", type=float, default=PIVOT_S)
    parser.add_argument("--out", required=True)
    arguments = parser.parse_args()

    rclpy.init()
    node = YawAudit()
    # Let every stream arrive before commanding anything, so a missing
    # publisher is reported as missing rather than as a zero rotation.
    settle = time.monotonic() + 3.0
    while time.monotonic() < settle:
        rclpy.spin_once(node, timeout_sec=0.05)
    present = {name: len(track) > 0 for name, track in node.tracks.items()}

    node.pivot(arguments.rad_s, arguments.seconds)

    turned = {name: round(total_turned_rad(track), 4) for name, track in node.tracks.items()}
    truth = turned["truth"]
    ratios = {
        name: (round(turned[name] / truth, 4) if truth > 0.1 else None)
        for name in ("odom_raw", "odom")
    }

    failures = []
    if not all(present.values()):
        failures.append(f"streams missing: {[n for n, ok in present.items() if not ok]}")
    elif truth <= 0.1:
        failures.append(f"the robot did not turn (truth {truth:.3f} rad); nothing is measured")
    else:
        if ratios["odom"] is not None and abs(ratios["odom"] - 1.0) > EKF_RATIO_TOLERANCE:
            failures.append(
                f"EKF yaw ratio {ratios['odom']} is not 1.0 +/- {EKF_RATIO_TOLERANCE}: "
                "the chain SLAM consumes does not track truth"
            )

    # A control that does not fire is reported, never silently passed. If the
    # wheels did not slip, the EKF was not asked to protect anything and a
    # clean EKF ratio is luck rather than evidence.
    control_fired = ratios["odom_raw"] is not None and abs(ratios["odom_raw"] - 1.0) > 0.2

    report = {
        "instrument": "corridor_yaw_audit",
        "pivot_rad_s": arguments.rad_s,
        "pivot_s": arguments.seconds,
        "streams_present": present,
        "samples": {name: len(track) for name, track in node.tracks.items()},
        "turned_rad": turned,
        "ratio_vs_truth": ratios,
        "negative_control_fired": control_fired,
        "control_note": (
            "wheel odometry is wrong as expected; the EKF was genuinely under test"
            if control_fired
            else "wheel odometry tracked truth: this pivot did NOT slip, so a clean "
                 "EKF ratio is inconclusive about the chain"
        ),
        "ekf_ratio_tolerance": EKF_RATIO_TOLERANCE,
        "failures": failures,
        "pass": not failures,
    }

    destination = Path(arguments.out)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))

    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
