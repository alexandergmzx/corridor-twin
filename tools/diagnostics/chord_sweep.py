#!/usr/bin/env python3
"""Does the chord ceiling separate B from the EastWallStub decoy?

    source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
    PYTHONNOUSERSITE=1 python tools/diagnostics/chord_sweep.py <bag> [<bag> ...]

WHY
---
`MAX_CHORD_FACTOR` is 1.4, which admits a visible chord of up to 2.8r. **A
circle of radius r has a maximum chord of exactly 2r**, so everything above
1.0 is slack for measurement noise and partial occlusion -- 40% of it.

The decoy is the west face of `EastWallStub`: a flat 0.318 m wall end against
B's 0.240 m chord ceiling. So the ceiling is, on paper, exactly the
discriminator -- and unlike a radius or residual threshold it is not a tuned
number, it is a geometric impossibility with a noise allowance.

On paper is not measured. A face seen obliquely or partly occluded presents a
SHORTER chord, and then the ceiling says nothing. This sweeps the factor and
reports, per bag, what the detector would have locked onto -- so the answer is
a curve rather than an opinion.

Truth is read once per frame to label what the detection WAS. It never enters
the detector.
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

import landmark_detector as detector_module  # noqa: E402

#: The stub's west face centre, from `east_wall_stub_bounds` at the committed
#: scale: x = street_east - clear_width * depth_fraction, y = the face midpoint.
STUB_XY = (4.56534, -1.926)
NEAR_M = 0.25

FACTORS = (1.4, 1.3, 1.2, 1.1, 1.05, 1.0)


def _yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def sweep(bag: Path, b_xy: tuple[float, float], radius: float) -> dict:
    """One pass over the bag, scoring every factor from the same scans."""

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    scan_type = get_message(types["/scan"])
    truth_type = get_message(types["/sim/ground_truth"])

    detector = detector_module.LandmarkDetector(radius)
    tallies = {factor: {"B": 0, "stub": 0, "other": 0} for factor in FACTORS}
    truth_pose = None
    original = detector_module.MAX_CHORD_FACTOR
    try:
        while reader.has_next():
            topic, data, _stamp = reader.read_next()
            if topic == "/sim/ground_truth":
                pose = deserialize_message(data, truth_type).pose.pose
                truth_pose = (pose.position.x, pose.position.y, _yaw(pose.orientation))
                continue
            if topic != "/scan" or truth_pose is None:
                continue
            message = deserialize_message(data, scan_type)
            points = detector_module.scan_to_xy(
                message.ranges, message.angle_min, message.angle_increment,
                message.range_min, message.range_max,
            )
            cos_yaw, sin_yaw = math.cos(truth_pose[2]), math.sin(truth_pose[2])
            for factor in FACTORS:
                detector_module.MAX_CHORD_FACTOR = factor
                found = detector.candidates(points)
                if not found:
                    continue
                best = found[0]
                world = (
                    truth_pose[0] + best["x"] * cos_yaw - best["y"] * sin_yaw,
                    truth_pose[1] + best["x"] * sin_yaw + best["y"] * cos_yaw,
                )
                if math.dist(world, b_xy) <= NEAR_M:
                    tallies[factor]["B"] += 1
                elif math.dist(world, STUB_XY) <= NEAR_M:
                    tallies[factor]["stub"] += 1
                else:
                    tallies[factor]["other"] += 1
    finally:
        detector_module.MAX_CHORD_FACTOR = original

    return {"bag": bag.name, "tallies": tallies}


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    manifest = json.loads(
        Path("out/corridor.manifest.json").read_text(encoding="utf-8")
    )
    b_x, b_y, _ = manifest["actors"]["b_xyz_m"]
    radius = float(manifest["actors"]["b_radius_m"])
    print(f"B diameter {2 * radius:.3f} m; the stub's west face is 0.318 m.")
    print("A chord above 2r is impossible for a real cylinder, so 1.0 is the "
          "geometric floor and everything above it is noise allowance.\n")

    totals = {factor: {"B": 0, "stub": 0, "other": 0} for factor in FACTORS}
    for argument in sys.argv[1:]:
        result = sweep(Path(argument), (b_x, b_y), radius)
        print(f"  {result['bag']}")
        for factor in FACTORS:
            tally = result["tallies"][factor]
            for key in tally:
                totals[factor][key] += tally[key]
            print(f"      chord x{factor:<5} B={tally['B']:5}  "
                  f"stub={tally['stub']:5}  other={tally['other']:5}")

    print("\n  TOTAL across all bags")
    for factor in FACTORS:
        tally = totals[factor]
        seen = tally["B"] + tally["stub"] + tally["other"]
        share = 100.0 * tally["stub"] / seen if seen else 0.0
        print(f"      chord x{factor:<5} B={tally['B']:6}  stub={tally['stub']:5}  "
              f"other={tally['other']:5}   stub share {share:5.2f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
