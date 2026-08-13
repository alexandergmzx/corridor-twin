#!/usr/bin/env python3
"""How far off A's nose does B sit, while A is close enough to dock?

    source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
    PYTHONNOUSERSITE=1 python tools/diagnostics/bearing_to_b.py <bag> [<bag> ...]

WHY
---
The docking arm test asks "is the detection where the goal is?", and it asks it
by taking the bearing to the goal IN THE MAP FRAME. That is the number the
overshoot diagnosis showed is wrong by 0.8-2.2 m, and D1 deletes it.

The map-free replacement is a forward cone in the body frame: refuse anything
that is not roughly ahead of the robot. But the cone half-angle has to admit the
REAL B on every run or it becomes a new way to never arm -- and A rounds a
corner on approach, so "ahead" is not obviously generous.

This measures it instead of guessing: over every bag, at every instant A is
within the arm radius of B, the body-frame bearing from A to B. The widest of
those, plus margin, is the smallest honest cone.

Truth is used deliberately. This is an offline sizing measurement on the
evaluation plane, not an observer input -- the constant it produces is then
checked in the live run by the detector alone.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

#: The range inside which docking may arm (`corridor_dock.ARM_RADIUS_M`).
ARM_RADIUS_M = 3.0

TRUTH_TOPIC = "/sim/ground_truth"


def _yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def _b_world(profile: str) -> tuple[float, float]:
    manifest = json.loads(
        Path("out/corridor.manifest.json").read_text(encoding="utf-8")
    )
    b_x, b_y, _ = manifest["actors"]["b_xyz_m"]
    return float(b_x), float(b_y)


def sweep(bag: Path, b_xy: tuple[float, float]) -> dict:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if TRUTH_TOPIC not in types:
        return {"bag": bag.name, "error": f"no {TRUTH_TOPIC}"}
    message_type = get_message(types[TRUTH_TOPIC])

    # Every in-range sample, in order, so the approach can be separated from
    # the overshoot afterwards. Measuring across the whole in-range window
    # answers the wrong question: A drives PAST B and out to the far wall, so
    # most of that window has B behind it and the widest bearing is 180 deg by
    # construction. Docking's job is to preempt during the approach, so the
    # approach is what the cone must admit.
    samples: list[tuple[float, float]] = []
    while reader.has_next():
        topic, data, _stamp = reader.read_next()
        if topic != TRUTH_TOPIC:
            continue
        pose = deserialize_message(data, message_type).pose.pose
        dx, dy = b_xy[0] - pose.position.x, b_xy[1] - pose.position.y
        distance = math.hypot(dx, dy)
        if distance > ARM_RADIUS_M:
            continue
        relative = math.atan2(dy, dx) - _yaw(pose.orientation)
        samples.append((
            distance,
            math.degrees(abs((relative + math.pi) % (2.0 * math.pi) - math.pi)),
        ))

    if not samples:
        return {"bag": bag.name, "samples": 0, "closest_m": None}
    closest_index = min(range(len(samples)), key=lambda i: samples[i][0])
    approach = [bearing for _distance, bearing in samples[: closest_index + 1]]
    ordered = sorted(approach)
    return {
        "bag": bag.name,
        "samples": len(approach),
        "of_in_range": len(samples),
        "closest_m": round(samples[closest_index][0], 3),
        "max_bearing_deg": round(max(approach), 1),
        "median_bearing_deg": round(ordered[len(ordered) // 2], 1),
        "p95_bearing_deg": round(ordered[int(0.95 * (len(ordered) - 1))], 1),
        # The cone only has to admit B for long enough to arm k-of-n, and only
        # once A is inside the docking envelope. This is the window that
        # matters most: the last metre of the approach.
        "max_within_1m_deg": round(
            max((b for d, b in samples[: closest_index + 1] if d <= 1.0), default=0.0), 1
        ),
    }


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    b_xy = _b_world("nominal_m6_n3")
    print(f"B at {b_xy}, arm radius {ARM_RADIUS_M} m\n")
    worst = 0.0
    for argument in sys.argv[1:]:
        result = sweep(Path(argument), b_xy)
        if result.get("error") or not result.get("samples"):
            print(f"  {result['bag']}: {result.get('error') or 'never within range'}")
            continue
        worst = max(worst, result["max_within_1m_deg"])
        print(f"  {result['bag']}: approach n={result['samples']:5}/{result['of_in_range']:5}"
              f"  closest {result['closest_m']:.2f} m"
              f"  median {result['median_bearing_deg']:6.1f}"
              f"  p95 {result['p95_bearing_deg']:6.1f}"
              f"  max {result['max_bearing_deg']:6.1f}"
              f"  | max inside 1 m {result['max_within_1m_deg']:6.1f} deg")
    print(f"\nwidest bearing to B on APPROACH inside 1 m: {worst:.1f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
