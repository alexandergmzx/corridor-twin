#!/usr/bin/env python3
"""Split a diverged map into its two possible causes, offline, from the bag.

    python3 tools/corridor_transit_audit.py --bag <session bag> \\
        --out out/evidence/robot-a-gate/transit-audit.json

WHY THIS EXISTS
---------------
A map with doubled walls has exactly two sources, and they call for opposite
fixes:

  * **the odometry lied** -- the odom -> base_footprint transform did not
    describe the robot's real motion, so slam_toolbox laid correct scans down
    at wrong places. The fix is in the odometry chain.
  * **the scan matcher jumped** -- odometry was fine and slam_toolbox's own
    map -> odom correction moved discontinuously, which is what corridor
    scan-match degeneracy looks like. The fix is in SLAM, or in the geometry.

Tuning the wrong one is worse than tuning nothing, so this measures both from
one recorded run rather than inferring either from the map.

`corridor_yaw_audit.py` already cleared the odometry chain for a PIVOT WITH
TRACTION -- EKF yaw 1.025x truth while the wheels reported 3.28x. That is not
the same manoeuvre as a transit, where the robot arcs and translates at once
and slips differently, so the chain is measured again here under the motion
that actually produced the bad map.

WHAT IS COMPARED
----------------
Both tracks are expressed RELATIVE TO THEIR OWN FIRST POSE before comparison.
The odom frame and the truth publisher's frame share no origin, and comparing
them absolutely would report a constant frame offset as a growing error. What
survives that alignment is real: accumulated translation error, accumulated
yaw error, and the size of the discontinuities in SLAM's correction.

Simulator truth is an evaluation input here (CLAUDE.md invariant 1); nothing
A's stack subscribes to reads this file.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

#: A correction step larger than this is a JUMP, not tracking. slam_toolbox
#: nudges map -> odom continuously as it refines; a discrete relocalisation
#: moves it by a wheel-base or more between consecutive publications, and that
#: is the event that lays a second copy of a wall into the map.
JUMP_M = 0.10
JUMP_RAD = math.radians(5.0)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def relative_to_start(track):
    """Re-express a (t, x, y, yaw) track in the frame of its own first sample."""

    if not track:
        return []
    _t0, x0, y0, yaw0 = track[0]
    cos0, sin0 = math.cos(-yaw0), math.sin(-yaw0)
    out = []
    for stamp, x, y, yaw in track:
        dx, dy = x - x0, y - y0
        out.append((stamp,
                    dx * cos0 - dy * sin0,
                    dx * sin0 + dy * cos0,
                    wrap(yaw - yaw0)))
    return out


def resample_onto(reference, other):
    """Nearest-neighbour sample of `other` at each `reference` timestamp."""

    if not other:
        return []
    paired, index = [], 0
    for stamp, *ref in reference:
        while index + 1 < len(other) and abs(other[index + 1][0] - stamp) <= abs(
            other[index][0] - stamp
        ):
            index += 1
        paired.append((stamp, ref, list(other[index][1:])))
    return paired


def read_tracks(bag: str) -> dict:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))

    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage

    ekf, truth, correction = [], [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        seconds = stamp * 1e-9
        if topic == "/odom":
            message = deserialize_message(data, Odometry)
            pose = message.pose.pose
            ekf.append((seconds, pose.position.x, pose.position.y, yaw_of(pose.orientation)))
        elif topic == "/sim/ground_truth":
            message = deserialize_message(data, Odometry)
            pose = message.pose.pose
            truth.append((seconds, pose.position.x, pose.position.y, yaw_of(pose.orientation)))
        elif topic == "/tf":
            for transform in deserialize_message(data, TFMessage).transforms:
                # SLAM's correction, and the only transform whose discontinuities
                # can move already-mapped geometry.
                if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                    translation = transform.transform.translation
                    correction.append((seconds, translation.x, translation.y,
                                       yaw_of(transform.transform.rotation)))
    return {"ekf": ekf, "truth": truth, "correction": correction}


def audit(tracks: dict) -> dict:
    ekf = relative_to_start(tracks["ekf"])
    truth = relative_to_start(tracks["truth"])

    paired = resample_onto(truth, ekf)
    position_errors, yaw_errors = [], []
    for _stamp, (tx, ty, tyaw), (ex, ey, eyaw) in paired:
        position_errors.append(math.dist((tx, ty), (ex, ey)))
        yaw_errors.append(abs(wrap(tyaw - eyaw)))

    # SIGNED cumulative rotation, and deliberately never the absolute sum.
    # Summing |delta| over a track accumulates the yaw NOISE of every sample:
    # on a 453 s run at 11 Hz it scored 5496 deg of "rotation" -- fifteen
    # revolutions -- for a robot that actually turned 810. Signed summation
    # cancels that noise, and the yaw SCALE error is the quantity wanted here.
    def signed_rotation(track):
        return sum(wrap(later[3] - earlier[3])
                   for earlier, later in zip(track, track[1:], strict=False))

    turned_ekf, turned_truth = signed_rotation(ekf), signed_rotation(truth)

    # SLAM's correction is judged by its STEPS, not its magnitude. A large but
    # smoothly-accumulated correction is SLAM doing its job against drifting
    # odometry; a large single step is a relocalisation, and it is what smears
    # a map.
    correction = tracks["correction"]
    steps_m, steps_rad = [], []
    for earlier, later in zip(correction, correction[1:], strict=False):
        steps_m.append(math.dist((earlier[1], earlier[2]), (later[1], later[2])))
        steps_rad.append(abs(wrap(later[3] - earlier[3])))

    return {
        "samples": {name: len(track) for name, track in tracks.items()},
        "odometry_vs_truth": {
            "final_position_error_m": round(position_errors[-1], 4) if position_errors else None,
            "max_position_error_m": round(max(position_errors), 4) if position_errors else None,
            "final_yaw_error_deg": round(math.degrees(yaw_errors[-1]), 3) if yaw_errors else None,
            "max_yaw_error_deg": round(math.degrees(max(yaw_errors)), 3) if yaw_errors else None,
            "signed_rotation_ekf_deg": round(math.degrees(turned_ekf), 2),
            "signed_rotation_truth_deg": round(math.degrees(turned_truth), 2),
            # The headline number. A yaw SCALE error compounds with every turn,
            # so it destroys a map long before it is visible as a bad heading.
            "yaw_scale_ratio": (
                round(turned_ekf / turned_truth, 4) if abs(turned_truth) > 0.1 else None
            ),
        },
        "slam_correction": {
            "total_travel_m": round(sum(steps_m), 4),
            "max_step_m": round(max(steps_m), 4) if steps_m else None,
            "max_step_deg": round(math.degrees(max(steps_rad)), 3) if steps_rad else None,
            f"jumps_over_{JUMP_M:.2f}_m": sum(1 for step in steps_m if step > JUMP_M),
            "jumps_over_5_deg": sum(1 for step in steps_rad if step > JUMP_RAD),
            "final_offset_m": (
                round(math.hypot(correction[-1][1], correction[-1][2]), 4)
                if correction else None
            ),
            "final_offset_deg": (
                round(math.degrees(correction[-1][3]), 3) if correction else None
            ),
        },
    }


def verdict(report: dict) -> str:
    """Name the cause, or say plainly that the run does not distinguish them."""

    odom = report["odometry_vs_truth"]
    slam = report["slam_correction"]
    odom_bad = (odom["max_yaw_error_deg"] or 0) > 10.0 or (odom["max_position_error_m"] or 0) > 0.5
    slam_jumped = slam["jumps_over_5_deg"] > 0 or (slam["max_step_m"] or 0) > 0.5

    if odom_bad and slam_jumped:
        return ("BOTH: odometry drifted AND SLAM relocalised. The jumps may be SLAM "
                "correctly chasing bad odometry, so this run does not isolate a cause.")
    if odom_bad:
        return "ODOMETRY: the odom -> base transform did not describe the real motion."
    if slam_jumped:
        return ("SLAM: odometry tracked truth, and the map -> odom correction moved "
                "discontinuously anyway. Scan matching is the cause.")
    return "NEITHER instrument fired: this run does not explain a diverged map."


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bag", required=True)
    parser.add_argument("--out", required=True)
    arguments = parser.parse_args()

    report = audit(read_tracks(arguments.bag))
    report["bag"] = arguments.bag
    report["verdict"] = verdict(report)

    destination = Path(arguments.out)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
