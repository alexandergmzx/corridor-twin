#!/usr/bin/env python3
"""Replay a bag's /scan through the NEW arming tests. Would docking have fired?

    source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
    PYTHONNOUSERSITE=1 python tools/diagnostics/arming_replay.py <bag> [<bag> ...]

WHY
---
W2 deletes the map-frame proximity test and adds two new ones: radius
uniqueness against the frame's runner-up, and k-of-n persistence across scans.
Both are guards, and a guard that never lets go is indistinguishable from
docking being switched off -- which is the exact failure being repaired, so
shipping a replacement on reasoning alone would be repeating the mistake.

This replays recorded scans through the real detector and the real
`DockingMachine.armed`, and reports for each bag whether arming would ever have
fired, at what travel and range, and which test refused how often.

The travel figure comes from /odom, which is what the live gate integrates.
Truth is not read at all -- this replay sees exactly what the robot saw.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from corridor_dock import DockingMachine, final_approach_m  # noqa: E402
from corridor_nav_gate import route_to_delivery_m  # noqa: E402
from landmark_detector import LandmarkDetector  # noqa: E402

# The route length is IMPORTED, not re-derived. This file used to carry its own
# copy of the sum, and it carried the same bug -- both dropped the departure leg
# on the premise that it runs past B, which it does not. Two copies of one
# derivation is how they drift, and a replay that computes a different arming
# threshold from the live gate answers a question nobody asked.


def replay(bag: Path, manifest: dict, profile: str) -> dict:
    actors = manifest["actors"]
    radius = float(actors["b_radius_m"])
    machine = DockingMachine(
        nominal_goal=(0.0, 0.0),
        standoff_m=final_approach_m(radius, float(actors["a_size_xyz_m"][0])),
        route_length_m=route_to_delivery_m(manifest, profile),
        expected_radius_m=radius,
    )
    detector = LandmarkDetector(radius)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    scan_type = get_message(types["/scan"])
    odom_type = get_message(types["/odom"])

    # Truth, read for ONE purpose: to say whether the thing arming locked onto
    # was actually B. "It arms" is not the claim that matters -- the 5.754 m
    # failure armed too. Evaluation-plane only; nothing here feeds the machine.
    truth_type = get_message(types["/sim/ground_truth"]) if "/sim/ground_truth" in types else None
    b_x, b_y, _b_z = manifest["actors"]["b_xyz_m"]

    travelled = 0.0
    last_xy: tuple[float, float] | None = None
    truth_pose: tuple[float, float, float] | None = None
    scans = 0
    with_runner_up = 0
    armed_at: dict | None = None
    while reader.has_next():
        topic, data, _stamp = reader.read_next()
        if topic == "/odom":
            position = deserialize_message(data, odom_type).pose.pose.position
            here = (position.x, position.y)
            if last_xy is not None:
                travelled += math.dist(last_xy, here)
            last_xy = here
            continue
        if topic == "/sim/ground_truth" and truth_type is not None:
            pose = deserialize_message(data, truth_type).pose.pose
            q = pose.orientation
            truth_pose = (
                pose.position.x, pose.position.y,
                math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
            )
            continue
        if topic != "/scan":
            continue
        message = deserialize_message(data, scan_type)
        verdict = detector.feed(
            message.ranges, message.angle_min, message.angle_increment,
            message.range_min, message.range_max,
        )
        scans += 1
        if verdict.get("runner_up") is not None:
            with_runner_up += 1
        if machine.armed(verdict, travelled_m=travelled) and armed_at is None:
            candidate = verdict["candidate"]
            armed_at = {
                "travel_m": round(travelled, 3),
                "range_m": candidate["range_m"],
                "fitted_radius_m": candidate.get("fitted_radius_m"),
            }
            if truth_pose is not None:
                # Where the detection really was, via A's true pose.
                cos_yaw, sin_yaw = math.cos(truth_pose[2]), math.sin(truth_pose[2])
                world = (
                    truth_pose[0] + candidate["x"] * cos_yaw - candidate["y"] * sin_yaw,
                    truth_pose[1] + candidate["x"] * sin_yaw + candidate["y"] * cos_yaw,
                )
                armed_at["detection_world"] = [round(v, 3) for v in world]
                armed_at["miss_from_b_m"] = round(math.dist(world, (b_x, b_y)), 3)

    return {
        "bag": bag.name,
        "scans": scans,
        "frames_with_a_runner_up": with_runner_up,
        "armed": armed_at is not None,
        "armed_at": armed_at,
        "min_travel_m": round(machine.min_travel_m, 3),
        "rejections": dict(sorted(
            machine.rejections.items(), key=lambda kv: -kv[1]
        )),
    }


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    manifest = json.loads(
        Path("out/corridor.manifest.json").read_text(encoding="utf-8")
    )
    for argument in sys.argv[1:]:
        result = replay(Path(argument), manifest, "nominal_m6_n3")
        print(f"\n  {result['bag']}: {result['scans']} scans, "
              f"{result['frames_with_a_runner_up']} with a runner-up")
        if not result["armed"]:
            print("    **NEVER ARMED**")
        else:
            a = result["armed_at"]
            miss = a.get("miss_from_b_m")
            flag = "" if miss is None else ("  <-- ON B" if miss <= 0.25 else "  <-- **NOT B**")
            print(f"    ARMED at {a['travel_m']} m travel, detection {a['range_m']} m away, "
                  f"fitted r={a['fitted_radius_m']}")
            print(f"    detection really at {a.get('detection_world')} vs B, "
                  f"miss {miss} m{flag}")
        print(f"    (arms after {result['min_travel_m']} m of travel)")
        for reason, count in result["rejections"].items():
            print(f"      {count:6}  {reason}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
