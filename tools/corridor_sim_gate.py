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

**`--observe-only` exists because the mission has ONE motion source.** The
drive schedule above is BENCH tooling: it characterises the matcher against a
known input on a bare twin. It must never run during a delivery, because the
delivery's motion policy is that A's motion is 100% governed Nav2
`NavigateToPose` -- no warm-up, no patrol, no exploration. Every corridor run
before 2026-08-11 violated that: this gate's forward passes, `sim_patrol`'s
1.0 m legs, and Nav2's controller all published `/cmd_vel_raw` at once, and the
resulting odometry described a robot being fought over by three writers.
`--observe-only` keeps the instrument and drops the publisher -- literally: no
publisher object is created, so the mode cannot regress into commanding motion.

The truth topic is consumed HERE and nowhere else. This is evaluation tooling,
so simulator truth is a permitted input (CLAUDE.md invariant 1); nothing A's
stack subscribes to may read it.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan

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

#: Rate floors, enforced against each stream's OWN observed span -- never
#: against `--seconds`, which is a budget remainder rather than a measurement
#: window (corridor_profile_run.sh:456 derives it from whatever the watchdog cap
#: has left after bring-up). Dividing by the request under-reported a 11.50 Hz
#: matcher as 5.35 Hz and manufactured two "too slow or absent" failures on a
#: run whose streams were both healthy (2026-08-12 13:16, gate JSON committed).
#:
#: The matcher legitimately withholds when the scan constrains nothing, so its
#: floor is half the scan rate -- the same 0.5 this gate has always used, now
#: expressed as a rate rather than folded into a count comparison.
MIN_ODOM_LASER_HZ_FRACTION = 0.5
#: The EKF does not withhold, so its floor is a fraction of its CONFIGURED
#: 10.0 Hz (ekf_sim_pnfix.yaml:86) rather than the rate itself.
#:
#: I set this to 10.0 earlier today and it was wrong on first contact with a
#: measurement: three consecutive runs read 9.98-9.99 Hz, because a filter
#: running exactly at its configured rate measures a hair under it once the
#: first interval is counted. A floor a real measurement cannot clear is not a
#: gate, it is arithmetic, and it failed runs whose EKF was healthy.
#:
#: 9.0 is 90% of nominal: it still catches a filter that is missing a tenth of
#: its updates, and it is not the continuity criterion -- that is MAX_EKF_GAP_S,
#: which is ADR-derived and unchanged. Recorded rather than quietly adjusted.
MIN_EKF_HZ = 9.0

#: The gate had NO yaw criterion, and passed a transit whose heading ended
#: 138 deg wrong: longitudinal drift was 4.9% (green) while the estimate
#: believed it had turned 365 deg against truth's 227 -- a 1.61 scale error.
#: Distance and heading are independent failure modes and a map is destroyed by
#: the second one first, because a yaw error compounds with every turn.
#: 0.10 admits the +/-4% measured across the pivot sweep with margin, and
#: excludes the 1.15-1.61 measured across transits.
MAX_YAW_SCALE_ERROR = 0.10


def _yaw_of(q) -> float:
    """Planar yaw from a quaternion. Kept module-level for the same reason the
    rest of the arithmetic is: a figure that needs a GPU session to reproduce
    is a figure nobody can check."""

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


# --- pure geometry, kept out of the node ------------------------------------
# These are what ADR 0027's numbers are computed from, so they are module-level
# functions rather than methods: a figure that can only be produced by standing
# up a ROS node inside a GPU session is a figure nobody can check. Same lesson
# as the pose gate (T3.0).


def stream_rate(
    messages: int,
    first_stamp_s: float | None,
    last_stamp_s: float | None,
) -> dict:
    """Publish rate on the stream's OWN observed span, from header stamps.

    The window a recorder ASKED for is not the window it observed. `--seconds`
    is sized to the transit's worst case and the recorder stops early when the
    transit ends, so on 2026-08-12 13:16 a 551.0 s request observed 256.11 s --
    and 2946 matcher messages, a healthy 11.50 Hz, were reported as 5.35 Hz and
    failed as "too slow or absent". The count was right; the denominator was a
    budget.

    `(n - 1) / span`, not `n / span`: n stamps bound n-1 intervals, and dividing
    by n overstates the rate on short windows.

    Fewer than two messages has no span to divide by, so the rate is null and
    the stream reports as ABSENT rather than as slow -- the distinction the old
    comparison could not make. Same shape as `crossing_measure.py:144-149`.
    """

    if messages < 2 or first_stamp_s is None or last_stamp_s is None:
        return {"msgs": messages, "span_s": None, "hz": None}
    span = last_stamp_s - first_stamp_s
    if span <= 0.0:
        return {"msgs": messages, "span_s": round(span, 4), "hz": None}
    return {
        "msgs": messages,
        "span_s": round(span, 3),
        "hz": round((messages - 1) / span, 2),
    }


def path_length_m(track: list[tuple[float, float, float]]) -> float:
    """Cumulative planar distance along a (time, x, y) track."""

    return sum(
        math.dist((a[1], a[2]), (b[1], b[2]))
        for a, b in zip(track, track[1:], strict=False)
    )


def yaw_scale(
    truth: list[tuple],
    estimate: list[tuple],
) -> dict:
    """Estimated rotation divided by true rotation over the run.

    SIGNED cumulative rotation on both sides. An absolute sum is blind to an
    inverted channel and accumulates per-sample noise instead of cancelling
    it -- it once scored fifteen revolutions for a robot that turned 810 deg.

    Reported as unavailable rather than as a pass when the robot barely turned:
    a ratio of two small numbers is noise, and a transit that never turned says
    nothing about a yaw scale error either way.
    """

    def turned(track: list[tuple]) -> float:
        return sum(
            (later[3] - earlier[3] + math.pi) % (2.0 * math.pi) - math.pi
            for earlier, later in zip(track, track[1:], strict=False)
        )

    if len(truth) < 2 or len(estimate) < 2:
        return {"available": False, "reason": "no truth or estimate track"}
    truth_turned = turned(truth)
    if abs(truth_turned) < math.radians(45.0):
        return {
            "available": False,
            "reason": f"the robot turned only {math.degrees(truth_turned):.1f} deg",
        }
    estimated = turned(estimate)
    return {
        "available": True,
        "truth_deg": round(math.degrees(truth_turned), 2),
        "estimated_deg": round(math.degrees(estimated), 2),
        "ratio": round(estimated / truth_turned, 4),
    }


def integrated_gyro_deg(series: list[tuple[float, float]]) -> float | None:
    """Rotation implied by a yaw-rate stream, integrated on its own stamps."""

    if len(series) < 2:
        return None
    total = sum(
        rate * (later_t - earlier_t)
        for (earlier_t, rate), (later_t, _rate) in zip(series, series[1:], strict=False)
    )
    return round(math.degrees(total), 2)


def landmark_report(gate, radius_m) -> dict:
    """What A actually saw of B's post, in the LASER frame.

    Reported, never gated, and deliberately so: the arrival gate is Nav2
    SUCCEEDED within tolerance, and the demo must pass with this detector
    switched off entirely. What this earns is a claim nothing else in the run
    can make -- that A perceived B, rather than that A reached a coordinate a
    diverged map believed in.
    """

    if radius_m is None:
        return {"available": False, "reason": "scene authors no landmark"}
    if not gate.landmark_hits:
        return {
            "available": True, "detected": False,
            "scan_frames": gate.landmark_frames,
            "expected_radius_m": radius_m,
        }

    first = gate.landmark_hits[0]
    closest = min(gate.landmark_hits, key=lambda hit: hit["range_m"])
    residuals = [hit["residual_m"] for hit in gate.landmark_hits]
    radii = [hit["fitted_radius_m"] for hit in gate.landmark_hits]
    return {
        "available": True,
        "detected": True,
        "scan_frames": gate.landmark_frames,
        "confirmed_frames": len(gate.landmark_hits),
        "expected_radius_m": radius_m,
        "first_detection": {
            "range_m": first["range_m"], "bearing_rad": first["bearing_rad"],
            "frames_to_confirm": first["frames_agreeing"],
        },
        "closest_detection_m": closest["range_m"],
        "fitted_radius_m": {
            "min": min(radii), "max": max(radii),
            "mean": round(sum(radii) / len(radii), 4),
        },
        "residual_m": {
            "min": min(residuals), "max": max(residuals),
            "mean": round(sum(residuals) / len(residuals), 5),
        },
    }


def world_frame_delivery(
    truth: list[tuple],
    standoff: tuple[float, float],
) -> dict:
    """How close A actually got to the delivery point, in WORLD coordinates.

    The map-frame goal error the nav gate reports is computed in a frame SLAM
    owns, so when SLAM diverges that number stops describing the robot at all:
    it read 6-7 m on runs where the robot physically came within 0.8 m of the
    standoff, and 0.15 m of map-frame error would be equally meaningless in the
    other direction. This is the evaluation plane's own measurement and it
    cannot be fooled by a bad map.

    CLOSEST approach is reported alongside the final position because they
    answer different questions. A run that reaches the standoff and then drives
    away has succeeded at navigation and failed at knowing it -- which is a
    completely different defect from one that never arrives, and the final
    position alone cannot tell them apart.

    Evaluation only (CLAUDE.md invariant 1): nothing A's stack subscribes to
    reads this.
    """

    if len(truth) < 2:
        return {"available": False, "reason": "no truth track"}
    distances = [
        (row[0], math.dist((row[1], row[2]), standoff)) for row in truth
    ]
    closest_t, closest = min(distances, key=lambda pair: pair[1])
    return {
        "available": True,
        "standoff_world_m": [round(standoff[0], 4), round(standoff[1], 4)],
        "final_position_m": [round(truth[-1][1], 4), round(truth[-1][2], 4)],
        "final_error_m": round(distances[-1][1], 4),
        "closest_approach_m": round(closest, 4),
        "closest_at_s": round(closest_t - truth[0][0], 2),
        # The signature of "arrived, then left": it got there and did not stay.
        "walked_away_m": round(distances[-1][1] - closest, 4),
    }


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
    def __init__(self, target: dict | None = None, *, observe_only: bool = False) -> None:
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
        # Each stream's own first and last stamp, which is what its rate is
        # divided by. `first_stamp_s` below is the first stamp on ANY stream and
        # anchors the silence-before-first-message gaps; it is deliberately not
        # the same quantity, because a stream that starts late must not have
        # another stream's head start counted into its span.
        self.first_odom_laser_s: float | None = None
        self.withheld_gaps: list[float] = []
        # When the drive began. The interval from here to the FIRST odom_laser
        # is withholding too, and the most consequential kind: it was missed
        # entirely by a gaps-between-messages metric, which scored a run where
        # the matcher produced nothing for the first 5.9 m as "1 consecutive
        # withheld update".
        self.drive_started_s: float | None = None
        self.first_stamp_s: float | None = None
        self.first_odom_laser_station_m: float | None = None

        ekf_topic = self.target["ekf_topic"]
        if not ekf_topic.startswith("/"):
            ekf_topic = f"{namespace}/{ekf_topic}"
        # The EKF gap list mirrors the matcher's: same instrument, different
        # subject, so the two robots' numbers stay directly comparable.
        self.last_ekf_s: float | None = None
        self.first_ekf_s: float | None = None
        self.ekf_gaps: list[float] = []
        self.create_subscription(Odometry, f"{namespace}/odom_laser", self._on_odom_laser, 500)
        self.create_subscription(Odometry, ekf_topic, self._on_ekf, 500)
        self.create_subscription(OccupancyGrid, f"{namespace}/map", self._on_map, 1)
        self.create_subscription(Odometry, f"{namespace}/sim/ground_truth", self._on_truth, 500)
        # The yaw chain, stage by stage. A yaw scale error is measurable at the
        # /odom output, but which STAGE introduced it is not -- and the fix
        # differs completely between the sensor, the orientation filter, and
        # the fusion. Integrating the gyro at each tap answers that from an
        # ordinary run instead of from a bespoke experiment.
        #   /imu       raw from the twin
        #   /imu/data  after imu_filter_madgwick
        #   /odom      after robot_localization  <- already tracked as `estimate`
        self.gyro: dict[str, list[tuple[float, float]]] = {"imu": [], "imu_data": []}
        self.create_subscription(
            Imu, f"{namespace}/imu",
            lambda m: self.gyro["imu"].append((self._stamp_s(m), m.angular_velocity.z)), 500
        )
        self.create_subscription(
            Imu, f"{namespace}/imu/data",
            lambda m: self.gyro["imu_data"].append((self._stamp_s(m), m.angular_velocity.z)), 500
        )
        # B's landmark, if the scene carries one. This is the ONLY measurement
        # of B in the whole system that does not pass through the SLAM map: it
        # is taken in the laser frame, so it stays true while every map-frame
        # number is fiction. Recorded, never acted on -- the detector reports,
        # and nothing here steers.
        self.landmark_detector = None
        self.landmark_hits: list[dict] = []
        self.landmark_frames = 0
        # BEST_EFFORT, not the default RELIABLE. The twin offers /scan with
        # sensor QoS, and a RELIABLE subscription does not match a BEST_EFFORT
        # offer at all -- it simply receives nothing, silently, which is what
        # made the first landmark run report scan_frames: 0 with the detector
        # correctly armed. BEST_EFFORT matches any offer; the reverse starves
        # (fleet OI-20).
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        self.create_subscription(
            LaserScan, f"{namespace}/scan", self._on_scan,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        # No publisher AT ALL in observe-only mode. A flag consulted inside a
        # drive loop would still leave the node able to command the robot if
        # some later caller forgot to check it; withholding the object makes
        # the guarantee structural, and `drive()` raises rather than crashing
        # on a missing attribute.
        self.observe_only = observe_only
        self.publisher = (
            None if observe_only
            else self.create_publisher(Twist, f"{namespace}/cmd_vel_raw", 10)
        )

    def _stamp_s(self, message) -> float:
        """The message's OWN clock, never the receiver's.

        Every gap and every track in this gate is timed by header stamp. Wall
        time here measured the RECORDER, not the robot: this node spins rclpy
        in a Python loop that also deserializes a growing /map OccupancyGrid,
        and when that blocks, the messages it misses were reported as the
        EKF failing to publish. Measured 2026-08-11 -- the gate called a
        3.052 s "EKF output gap" on a run whose bag shows the EKF's true worst
        gap was 0.398 s, with none at all over the 0.4 s threshold.

        Header stamps are immune to that: a burst delivered late still carries
        the cadence it was published at. It is also what CLAUDE.md requires
        under simulation, independently of this defect.
        """

        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        # The anchor for "silence before the first message" has to live on this
        # same clock, so it is the first stamp seen on ANY stream rather than a
        # wall-clock reading taken when observation began.
        if self.first_stamp_s is None:
            self.first_stamp_s = stamp
        return stamp

    # --- callbacks ---------------------------------------------------------
    def _on_odom_laser(self, message: Odometry) -> None:
        now = self._stamp_s(message)
        if self.last_odom_laser_s is not None:
            self.withheld_gaps.append(now - self.last_odom_laser_s)
        elif self.first_stamp_s is not None:
            self.withheld_gaps.append(now - self.first_stamp_s)
            self.first_odom_laser_station_m = round(path_length_m(self.truth), 4)
        if self.first_odom_laser_s is None:
            self.first_odom_laser_s = now
        self.last_odom_laser_s = now
        self.counts["odom_laser"] += 1
        covariance = message.pose.covariance
        self.covariance_trace.append(
            (path_length_m(self.truth), covariance[0], covariance[7], covariance[35])
        )

    def _on_ekf(self, message: Odometry) -> None:
        now = self._stamp_s(message)
        if self.last_ekf_s is not None:
            self.ekf_gaps.append(now - self.last_ekf_s)
        elif self.first_stamp_s is not None:
            self.ekf_gaps.append(now - self.first_stamp_s)
        if self.first_ekf_s is None:
            self.first_ekf_s = now
        self.last_ekf_s = now
        self.counts["ekf"] += 1
        self.estimate.append(
            (
                now,
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                _yaw_of(message.pose.pose.orientation),
            )
        )

    def _on_scan(self, message: LaserScan) -> None:
        if self.landmark_detector is None:
            return
        self.landmark_frames += 1
        verdict = self.landmark_detector.feed(
            message.ranges, message.angle_min, message.angle_increment,
            message.range_min, message.range_max,
        )
        if verdict["confirmed"]:
            hit = dict(verdict["candidate"])
            hit["t"] = self._stamp_s(message)
            hit["frames_agreeing"] = verdict["frames_agreeing"]
            self.landmark_hits.append(hit)

    def _on_map(self, message: OccupancyGrid) -> None:
        self.counts["map"] += 1
        self.map_msg = message

    def _on_truth(self, message: Odometry) -> None:
        self.truth.append(
            (
                self._stamp_s(message),
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                _yaw_of(message.pose.pose.orientation),
            )
        )


def observe(gate: CorridorGate, seconds: float) -> None:
    """Record for `seconds` while SOMETHING ELSE moves the robot.

    The mission's something-else is Nav2, and it is the only permitted one. The
    metrics are identical to `drive()`'s -- same subscriptions, same clocks --
    so an observe-only run and a bench drive run stay directly comparable; the
    only difference is who commanded the motion being measured.
    """

    if gate.publisher is not None:
        raise RuntimeError("observe() requires a gate constructed observe_only=True")

    # SIGTERM ends the observation and still writes the report. The window is
    # sized to the transit's worst case, so a delivery that succeeds in a
    # quarter of it would otherwise hold the run open -- and killing the
    # recorder outright would discard the very measurements of the successful
    # run. Stopping early is a complete result, not a truncated one.
    stopping = False

    def _stop(_signum, _frame) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    # Left as None until the first stamped message arrives: the "silence before
    # the first message" gap has to be measured on the message clock too, and
    # there is no stamp to anchor it to until one exists.
    started_wall_s = time.monotonic()
    end = started_wall_s + seconds
    while time.monotonic() < end and not stopping:
        rclpy.spin_once(gate, timeout_sec=0.05)
    # How long this node WATCHED is a wall-clock duration, and the one quantity
    # here that legitimately is one.
    gate.observed_s = round(time.monotonic() - started_wall_s, 2)


def drive(gate: CorridorGate, seconds: float) -> None:
    """Straight passes with settles. No rotation: see the module docstring.

    BENCH TOOLING. Never valid during a delivery: see the module docstring on
    `--observe-only`.
    """

    if gate.publisher is None:
        raise RuntimeError("drive() called on an observe-only gate")
    gate.drive_started_s = time.monotonic()   # bench drive only; gaps are stamp-timed
    started_wall_s = gate.drive_started_s
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
    # Set on this path too. It was set only in observe(), so every bench-drive
    # artifact carried "observed_s": null and its rates could never be checked
    # against the window that produced them.
    gate.observed_s = round(time.monotonic() - started_wall_s, 2)


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
        "--manifest",
        help="scene manifest; enables the world-frame delivery measurement",
    )
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
    parser.add_argument(
        "--observe-only",
        action="store_true",
        help="Record without commanding motion. REQUIRED during a mission run: "
             "A's motion is Nav2's alone.",
    )
    arguments = parser.parse_args()

    target = ROBOT_TARGETS[arguments.robot]
    scan_hz = arguments.scan_hz if arguments.scan_hz is not None else target["scan_hz"]

    # Arm the landmark detector from the manifest, whose radius is the authored
    # prop's own. A literal here would keep matching an old post after a rescale.
    landmark_radius = None
    if arguments.manifest:
        actors = json.loads(
            Path(arguments.manifest).read_text(encoding="utf-8")
        ).get("actors", {})
        landmark_radius = actors.get("b_radius_m")

    rclpy.init()
    gate = CorridorGate(target, observe_only=arguments.observe_only)
    if landmark_radius:
        sys.path.insert(0, str(Path(__file__).parent))
        from landmark_detector import LandmarkDetector

        gate.landmark_detector = LandmarkDetector(landmark_radius)
    if arguments.observe_only:
        observe(gate, arguments.seconds)
    else:
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

    # World-frame delivery needs the scene, not the map: it is the one measure
    # of arrival that a diverged map cannot corrupt.
    delivery = {"available": False, "reason": "no --manifest given"}
    if arguments.manifest:
        sys.path.insert(0, str(Path(__file__).parent))
        from corridor_nav_gate import delivery_standoff_world

        manifest = json.loads(Path(arguments.manifest).read_text(encoding="utf-8"))
        delivery = world_frame_delivery(gate.truth, delivery_standoff_world(manifest))

    consecutive = max_consecutive_withheld(gate.withheld_gaps, 1.0 / scan_hz)
    worst_ekf_gap_s = round(max(gate.ekf_gaps), 4) if gate.ekf_gaps else None

    odom_laser = stream_rate(
        gate.counts["odom_laser"], gate.first_odom_laser_s, gate.last_odom_laser_s
    )
    ekf = stream_rate(gate.counts["ekf"], gate.first_ekf_s, gate.last_ekf_s)
    min_odom_laser_hz = round(scan_hz * MIN_ODOM_LASER_HZ_FRACTION, 2)
    # Every pinned number this run was judged against, in the artifact and on
    # stdout, on a PASS as well as a fail. Three of these were previously
    # visible only inside the string of the failure they produced, so a green
    # run recorded no evidence of what it had cleared (CLAUDE.md gate
    # discipline: "a pinned threshold is printed and enforced from one
    # constant").
    thresholds = {
        "min_odom_laser_hz": min_odom_laser_hz,
        "min_odom_laser_hz_fraction": MIN_ODOM_LASER_HZ_FRACTION,
        "scan_hz": scan_hz,
        "min_ekf_hz": MIN_EKF_HZ,
        "max_ekf_gap_s": MAX_EKF_GAP_S,
        "max_consecutive_withheld": MAX_CONSECUTIVE_WITHHELD,
        "max_midpoint_drift_fraction": MAX_MIDPOINT_DRIFT_FRACTION,
        "max_yaw_scale_error": MAX_YAW_SCALE_ERROR,
    }

    report = {
        "robot": arguments.robot,
        "profile": arguments.profile,
        "caveat": arguments.caveat,
        "gated": arguments.gated,
        # Who moved the robot while these numbers were taken. Not cosmetic: the
        # same metric means something different under a bench drive than under
        # Nav2, and every artifact before 2026-08-11 was silently a third thing
        # (this gate + sim_patrol + Nav2 at once).
        "motion_source": "nav2" if arguments.observe_only else "gate_bench_drive",
        "seconds": arguments.seconds,
        # The window ASKED for, vs the window actually observed. They differ
        # when the transit ended early and the recorder was stopped with it.
        "observed_s": getattr(gate, "observed_s", None),
        # Which denominator produced the rates below. Stamped so an artifact
        # says for itself: everything before 2026-08-12 divided by `seconds`
        # and is not comparable with anything after it.
        "rate_basis": "observed_stream_span",
        "odom_laser_msgs": odom_laser["msgs"],
        "odom_laser_span_s": odom_laser["span_s"],
        "odom_laser_hz": odom_laser["hz"],
        "ekf_msgs": ekf["msgs"],
        "ekf_span_s": ekf["span_s"],
        "ekf_hz": ekf["hz"],
        "thresholds": thresholds,
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
        "yaw_scale": yaw_scale(gate.truth, gate.estimate),
        "world_frame_delivery": delivery,
        "landmark": landmark_report(gate, landmark_radius),
        "yaw_chain_deg": {
            "imu_raw": integrated_gyro_deg(gate.gyro["imu"]),
            "imu_filtered": integrated_gyro_deg(gate.gyro["imu_data"]),
        },
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
    # Absent and slow are separate verdicts now. They were one string, and a
    # healthy stream divided by the wrong window read as the same defect as a
    # stream that never published at all.
    if odom_laser["hz"] is None:
        failures.append(f"odom_laser absent ({odom_laser['msgs']} messages)")
    elif odom_laser["hz"] < min_odom_laser_hz:
        failures.append(
            f"odom_laser {odom_laser['hz']} Hz over {odom_laser['span_s']} s "
            f"is below {min_odom_laser_hz} Hz ({MIN_ODOM_LASER_HZ_FRACTION} x {scan_hz} Hz scan)"
        )
    if gate.covariance_trace and not all(
        0 < value < 1e5 for value in gate.covariance_trace[-1][1:]
    ):
        failures.append("matcher covariance not plausible (degeneracy path broken?)")
    if ekf["hz"] is None:
        failures.append(f"EKF output absent ({ekf['msgs']} messages)")
    elif ekf["hz"] < MIN_EKF_HZ:
        failures.append(
            f"EKF {ekf['hz']} Hz over {ekf['span_s']} s is below its configured "
            f"{MIN_EKF_HZ} Hz"
        )
    if not tf_odom_base:
        failures.append(f"TF {target['odom_frame']}->{target['base_frame']} missing")
    if not tf_map_odom:
        failures.append(f"TF map->{target['odom_frame']} missing")
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
    turning = report["yaw_scale"]
    if turning.get("available") and abs(turning["ratio"] - 1.0) > MAX_YAW_SCALE_ERROR:
        failures.append(
            f"yaw scale {turning['ratio']} (estimated {turning['estimated_deg']} deg vs "
            f"truth {turning['truth_deg']} deg) exceeds 1.0 +/- {MAX_YAW_SCALE_ERROR}"
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
