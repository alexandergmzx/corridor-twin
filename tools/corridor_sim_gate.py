#!/usr/bin/env python3
"""Gate: does the fleet ground-station stack map the CORRIDOR, not an open room?

    ROS_DOMAIN_ID=67 python3 tools/corridor_sim_gate.py --seconds 90 \
        --profile nominal_m6_n3 --out out/evidence/robot-a-gate/nominal.json

Forked from the fleet's `tools/robot2_sim_gate.py` (v2 plan T3.1). The fleet
copy stays untouched: this repository is a scenario member, not a co-owner of
fleet tooling, and the corridor needs a different drive schedule rather than a
different threshold on the same one.

WHAT CHANGED FROM THE FLEET GATE, AND WHY
-----------------------------------------
**The polygon is gone.** The fleet gate drives forward legs alternating with
2.5 s rotations, which is right for a 4x4 m room: the governor stops the
forward leg at a wall and the rotation frees it. In a corridor that tapers to
3 m the same schedule fights the walls through the governor -- every rotation
puts the robot's shoulder toward a wall it is already close to, so the governor
brakes, the robot rotates on the spot, and the run measures the governor rather
than the matcher. The corridor schedule is straight passes with brief settles.

**Covariance is recorded against station, not just sampled at the end.** That
trace IS the degeneracy study (ADR 0027): a corridor is the classic scan-match
degeneracy, because the along-corridor direction is weakly constrained when
both walls are parallel and featureless. A single end-of-run sample cannot show
the covariance growing as the robot advances and then collapsing when the
corner comes into view; the trace can.

**Every run writes JSON.** Fleet finding F15: a gate whose number lives only in
a README is a number nobody can re-check.

The truth topic is consumed HERE and nowhere else. This is evaluation tooling,
so simulator truth is a permitted input (CLAUDE.md invariant 1); nothing A's
stack subscribes to may read it.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node

#: Default target is robot2, so every artifact committed before this
#: parameterization stays reproducible by re-running the same command.
NS = "/robot2"

#: Per-robot wiring. robot1 runs at ROOT (architecture.md:46-51) with
#: unprefixed frames, and its EKF output is /odom rather than
#: odometry/filtered (bringup_corrected_launch.py:82).
ROBOT_TARGETS = {
    "robot2": {
        "namespace": "/robot2",
        "ekf_topic": "odometry/filtered",
        "base_frame": "robot2/base_footprint",
        "odom_frame": "robot2/odom",
        "scan_hz": 10.0,
        # robot2 HAS no encoders (fleet D-05): the matcher IS its odometry, so
        # withholding starves localization and is gated.
        "gate_withholding": True,
    },
    "robot1": {
        "namespace": "",
        "ekf_topic": "/odom",
        "base_frame": "base_footprint",
        "odom_frame": "odom",
        "scan_hz": 12.0,
        # robot1's EKF fuses wheel encoders + IMU and does NOT consume the
        # matcher at all (ekf_sim_pnfix.yaml:117-155; laser pose removed at
        # :138-146 as "measured HARMFUL"). Withholding therefore cannot starve
        # localization: it is RECORDED as study data and deliberately NOT
        # gated. The criterion that replaces it is EKF output continuity.
        "gate_withholding": False,
    },
}

#: Replacement criterion for robot1, derived by ADR 0022's own logic: blind
#: travel must stay under the goal tolerance. 0.35 m/s governor cap
#: (yahboomcar_safety/governor.py:41-60) x 0.4 s = 0.14 m < 0.15 m. At the
#: EKF's 10 Hz (ekf_sim_pnfix.yaml:86) that is 4 consecutive missed updates.
#: The governor cap is used rather than the gate's drive speed because it is
#: the true worst case. Measured from drive start, so initial silence counts.
MAX_EKF_GAP_S = 0.4

#: Straight-pass schedule. Forward at a governed crawl, with short settles that
#: let the matcher publish against a stationary scan -- a corridor's weakest
#: constraint is along its own axis, and a settle is where that shows.
FORWARD_MPS = 0.15
FORWARD_S = 8.0
SETTLE_S = 1.5

#: Pinned by ADR 0022 via the v2 plan section 6.
MAX_CONSECUTIVE_WITHHELD = 5
MAX_MIDPOINT_DRIFT_FRACTION = 0.05


# --- pure geometry, kept out of the node ------------------------------------
# These are what ADR 0027's numbers are computed from, so they are module-level
# functions rather than methods: a figure that can only be produced by standing
# up a ROS node inside a GPU session is a figure nobody can check. Same lesson
# as the pose gate (T3.0).


def path_length_m(track: list[tuple[float, float, float]]) -> float:
    """Cumulative planar distance along a (time, x, y) track."""

    return sum(
        math.dist((a[1], a[2]), (b[1], b[2]))
        for a, b in zip(track, track[1:], strict=False)
    )


def midpoint_drift(
    truth: list[tuple[float, float, float]],
    estimate: list[tuple[float, float, float]],
) -> dict:
    """Estimated vs true distance travelled at the halfway point of the run.

    Compared as PATH LENGTH rather than as a position difference: the EKF and
    the truth publisher do not share a frame origin, and forcing them into one
    would turn this into a measure of frame alignment. Along a corridor,
    travelled distance is the quantity that degrades under scan-match
    degeneracy anyway.
    """

    truth_total = path_length_m(truth)
    if truth_total <= 0.0 or len(estimate) < 2:
        return {"available": False, "reason": "no truth or estimate track"}

    half = truth_total / 2.0
    running = 0.0
    midpoint_time = None
    truth_at_midpoint = 0.0
    for earlier, later in zip(truth, truth[1:], strict=False):
        running += math.dist((earlier[1], earlier[2]), (later[1], later[2]))
        if running >= half:
            midpoint_time = later[0]
            truth_at_midpoint = running
            break
    if midpoint_time is None:
        return {"available": False, "reason": "never reached the halfway point"}

    estimate_to_midpoint = [row for row in estimate if row[0] <= midpoint_time]
    if len(estimate_to_midpoint) < 2:
        return {"available": False, "reason": "no estimate before the midpoint"}
    estimated = path_length_m(estimate_to_midpoint)

    # Both tracks are truncated at the SAME INSTANT and compared there. An
    # earlier version compared the estimate at the midpoint time against half
    # the total distance, which are not the same point: truth crosses the
    # halfway mark partway through a sample interval, so a perfectly tracking
    # estimate was reported as drifting by a whole sample's travel. On a coarse
    # truth stream that manufactures drift out of sampling alone.
    drift = abs(estimated - truth_at_midpoint)
    return {
        "available": True,
        "midpoint_time_s": round(midpoint_time, 4),
        "truth_distance_m": round(truth_at_midpoint, 4),
        "estimated_distance_m": round(estimated, 4),
        "longitudinal_drift_m": round(drift, 4),
        "drift_fraction": (
            round(drift / truth_at_midpoint, 4) if truth_at_midpoint else None
        ),
    }


def covariance_at_midpoint(
    trace: list[tuple[float, float, float, float]], total_distance_m: float
) -> dict | None:
    """The trace sample nearest half the travelled distance."""

    if not trace:
        return None
    target = total_distance_m / 2.0
    station, xx, yy, yaw = min(trace, key=lambda row: abs(row[0] - target))
    return {
        "station_m": round(station, 4),
        "cov_xx": xx,
        "cov_yy": yy,
        "cov_yawyaw": yaw,
        "trace": xx + yy + yaw,
    }


def max_consecutive_withheld(gaps_s: list[float], scan_period_s: float) -> int:
    """How many scan periods the matcher skipped in its worst gap.

    A withheld update is a gap materially longer than one scan period. The
    matcher withholds degenerate scans upstream, so gaps are the visible
    symptom of degeneracy rather than an error in themselves.
    """

    if scan_period_s <= 0.0:
        raise ValueError("scan period must be positive")
    return max((round(gap / scan_period_s) - 1 for gap in gaps_s), default=0)


class CorridorGate(Node):
    def __init__(self, target: dict | None = None) -> None:
        super().__init__("corridor_sim_gate")
        self.target = target or ROBOT_TARGETS["robot2"]
        namespace = self.target["namespace"]
        self.counts = {"odom_laser": 0, "ekf": 0, "map": 0}
        self.map_msg: OccupancyGrid | None = None
        # (monotonic_s, x, y) so station and drift are both derivable.
        self.truth: list[tuple[float, float, float]] = []
        self.estimate: list[tuple[float, float, float]] = []
        # (station_m, cov_xx, cov_yy, cov_yawyaw) -- the degeneracy trace.
        self.covariance_trace: list[tuple[float, float, float, float]] = []
        self.last_odom_laser_s: float | None = None
        self.withheld_gaps: list[float] = []
        # When the drive began. The interval from here to the FIRST odom_laser
        # is withholding too, and the most consequential kind: it was missed
        # entirely by a gaps-between-messages metric, which scored a run where
        # the matcher produced nothing for the first 5.9 m as "1 consecutive
        # withheld update".
        self.drive_started_s: float | None = None
        self.first_odom_laser_station_m: float | None = None

        ekf_topic = self.target["ekf_topic"]
        if not ekf_topic.startswith("/"):
            ekf_topic = f"{namespace}/{ekf_topic}"
        # The EKF gap list mirrors the matcher's: same instrument, different
        # subject, so the two robots' numbers stay directly comparable.
        self.last_ekf_s: float | None = None
        self.ekf_gaps: list[float] = []
        self.create_subscription(Odometry, f"{namespace}/odom_laser", self._on_odom_laser, 10)
        self.create_subscription(Odometry, ekf_topic, self._on_ekf, 10)
        self.create_subscription(OccupancyGrid, f"{namespace}/map", self._on_map, 10)
        self.create_subscription(Odometry, f"{namespace}/sim/ground_truth", self._on_truth, 10)
        self.publisher = self.create_publisher(Twist, f"{namespace}/cmd_vel_raw", 10)

    # --- callbacks ---------------------------------------------------------
    def _on_odom_laser(self, message: Odometry) -> None:
        now = time.monotonic()
        if self.last_odom_laser_s is not None:
            self.withheld_gaps.append(now - self.last_odom_laser_s)
        elif self.drive_started_s is not None:
            self.withheld_gaps.append(now - self.drive_started_s)
            self.first_odom_laser_station_m = round(path_length_m(self.truth), 4)
        self.last_odom_laser_s = now
        self.counts["odom_laser"] += 1
        covariance = message.pose.covariance
        self.covariance_trace.append(
            (path_length_m(self.truth), covariance[0], covariance[7], covariance[35])
        )

    def _on_ekf(self, message: Odometry) -> None:
        now = time.monotonic()
        if self.last_ekf_s is not None:
            self.ekf_gaps.append(now - self.last_ekf_s)
        elif self.drive_started_s is not None:
            self.ekf_gaps.append(now - self.drive_started_s)
        self.last_ekf_s = now
        self.counts["ekf"] += 1
        self.estimate.append(
            (
                time.monotonic(),
                message.pose.pose.position.x,
                message.pose.pose.position.y,
            )
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        self.counts["map"] += 1
        self.map_msg = message

    def _on_truth(self, message: Odometry) -> None:
        self.truth.append(
            (
                time.monotonic(),
                message.pose.pose.position.x,
                message.pose.pose.position.y,
            )
        )


def drive(gate: CorridorGate, seconds: float) -> None:
    """Straight passes with settles. No rotation: see the module docstring."""

    gate.drive_started_s = time.monotonic()
    end = time.monotonic() + seconds
    phase_end, phase = 0.0, "settle"
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= phase_end:
            phase = "forward" if phase == "settle" else "settle"
            phase_end = now + (FORWARD_S if phase == "forward" else SETTLE_S)
        command = Twist()
        if phase == "forward":
            command.linear.x = FORWARD_MPS
        gate.publisher.publish(command)
        rclpy.spin_once(gate, timeout_sec=0.05)

    for _ in range(10):
        gate.publisher.publish(Twist())
        rclpy.spin_once(gate, timeout_sec=0.02)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seconds", type=float, default=90.0)
    parser.add_argument("--profile", required=True)
    parser.add_argument(
        "--caveat",
        default="",
        help="Stamped into the artifact when a precondition failed but the run proceeded.",
    )
    parser.add_argument("--out", required=True)
    parser.add_argument(
        "--robot",
        choices=sorted(ROBOT_TARGETS),
        default="robot2",
        help="Which robot's namespace, frames, odom source and criteria to use.",
    )
    parser.add_argument(
        "--scan-hz",
        type=float,
        default=None,
        help="Matcher rate for the withholding arithmetic; defaults to the robot's.",
    )
    parser.add_argument(
        "--gated",
        action="store_true",
        help="This profile's result is a gate; without it the run is reported only.",
    )
    arguments = parser.parse_args()

    target = ROBOT_TARGETS[arguments.robot]
    scan_hz = arguments.scan_hz if arguments.scan_hz is not None else target["scan_hz"]

    rclpy.init()
    gate = CorridorGate(target)
    drive(gate, arguments.seconds)

    import tf2_ros

    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, gate)
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        rclpy.spin_once(gate, timeout_sec=0.05)
    tf_odom_base = buffer.can_transform(
        target["odom_frame"], target["base_frame"], rclpy.time.Time()
    )
    tf_map_odom = buffer.can_transform("map", target["odom_frame"], rclpy.time.Time())

    truth_distance = path_length_m(gate.truth)
    occupied = free = None
    if gate.map_msg is not None:
        occupied = sum(1 for value in gate.map_msg.data if value > 50)
        free = sum(1 for value in gate.map_msg.data if 0 <= value <= 50)

    consecutive = max_consecutive_withheld(gate.withheld_gaps, 1.0 / scan_hz)
    worst_ekf_gap_s = round(max(gate.ekf_gaps), 4) if gate.ekf_gaps else None

    report = {
        "robot": arguments.robot,
        "profile": arguments.profile,
        "caveat": arguments.caveat,
        "gated": arguments.gated,
        "seconds": arguments.seconds,
        "odom_laser_msgs": gate.counts["odom_laser"],
        "odom_laser_hz": round(gate.counts["odom_laser"] / arguments.seconds, 2),
        "ekf_msgs": gate.counts["ekf"],
        "ekf_hz": round(gate.counts["ekf"] / arguments.seconds, 2),
        "map_updates": gate.counts["map"],
        "map_occupied_cells": occupied,
        "map_free_cells": free,
        "map_resolution": gate.map_msg.info.resolution if gate.map_msg else None,
        "tf_odom_to_base": tf_odom_base,
        "tf_map_to_odom": tf_map_odom,
        "ground_truth_distance_m": round(truth_distance, 3),
        "max_consecutive_withheld_updates": max(0, consecutive),
        "withholding_is_gated": target["gate_withholding"],
        "worst_ekf_gap_s": worst_ekf_gap_s,
        "max_ekf_gap_s_limit": MAX_EKF_GAP_S,
        "first_odom_laser_station_m": gate.first_odom_laser_station_m,
        "midpoint_drift": midpoint_drift(gate.truth, gate.estimate),
        "midpoint_covariance": covariance_at_midpoint(gate.covariance_trace, truth_distance),
        # The degeneracy study's primary artifact. Kept whole: it is a few
        # hundred rows, and downsampling the one trace the study rests on would
        # be curating away the shape it exists to show.
        "covariance_trace_station_xx_yy_yawyaw": [
            [round(station, 4), xx, yy, yaw]
            for station, xx, yy, yaw in gate.covariance_trace
        ],
    }

    failures = []
    if gate.counts["odom_laser"] < arguments.seconds * scan_hz * 0.5:
        failures.append("odom_laser too slow or absent")
    if gate.covariance_trace and not all(
        0 < value < 1e5 for value in gate.covariance_trace[-1][1:]
    ):
        failures.append("matcher covariance not plausible (degeneracy path broken?)")
    if gate.counts["ekf"] < arguments.seconds * 10:
        failures.append("EKF output too slow or absent")
    if not tf_odom_base:
        failures.append("TF robot2/odom->robot2/base_footprint missing")
    if not tf_map_odom:
        failures.append("TF map->robot2/odom missing")
    if occupied is None or occupied < 200:
        failures.append(f"map missing or too sparse (occupied={occupied})")
    if truth_distance < 1.0:
        failures.append(f"robot barely moved ({truth_distance:.2f} m) - map proves nothing")
    if target["gate_withholding"]:
        if consecutive > MAX_CONSECUTIVE_WITHHELD:
            failures.append(
                f"matcher withheld {consecutive} consecutive updates "
                f"(limit {MAX_CONSECUTIVE_WITHHELD}, ADR 0022)"
            )
    else:
        # The matcher is not this robot's odometry, so its withholding is
        # recorded rather than gated. What must hold instead is that the EKF --
        # which Nav2 actually consumes -- never goes quiet for long enough to
        # blind the robot past its goal tolerance.
        if worst_ekf_gap_s is None:
            failures.append("no EKF output at all")
        elif worst_ekf_gap_s > MAX_EKF_GAP_S:
            failures.append(
                f"EKF output gap {worst_ekf_gap_s:.3f} s exceeds {MAX_EKF_GAP_S} s "
                f"(0.35 m/s governor cap x that gap must stay under the 0.15 m tolerance)"
            )
    drift = report["midpoint_drift"]
    if drift.get("available") and drift["drift_fraction"] > MAX_MIDPOINT_DRIFT_FRACTION:
        failures.append(
            f"midpoint longitudinal drift {drift['drift_fraction']:.3f} exceeds "
            f"{MAX_MIDPOINT_DRIFT_FRACTION} (ADR 0022)"
        )

    report["failures"] = failures
    report["pass"] = not failures

    destination = Path(arguments.out)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    summary = {key: value for key, value in report.items() if key != (
        "covariance_trace_station_xx_yy_yawyaw"
    )}
    print(json.dumps(summary, indent=2))
    print(f"\nwritten: {destination}")

    gate.destroy_node()
    rclpy.shutdown()

    if failures:
        print("\nGATE FAILURES:")
        for failure in failures:
            print(f"  FAIL {failure}")
        # A non-gated profile is a stress report: its failures are findings, not
        # a red gate (ADR 0022 gates nominal and wide_corner only).
        return 1 if arguments.gated else 0
    print(f"\ncorridor gate passed for {arguments.profile}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
