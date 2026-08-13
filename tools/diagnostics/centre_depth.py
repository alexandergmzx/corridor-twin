#!/usr/bin/env python3
"""Is the fitted centre BEHIND the nearest return, as a convex object demands?

    source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
    PYTHONNOUSERSITE=1 python tools/diagnostics/centre_depth.py <bag> [<bag> ...]

WHY
---
A cylinder of radius r seen by a lidar puts its CENTRE exactly r beyond the
nearest point of its own surface. That is not a heuristic, it is what convex
means: the closest return lies on the segment from the sensor to the centre.

The `EastWallStub` decoy is a flat face. A circle fitted to part of a flat face
-- or to noise across it -- has no such constraint, and can place its centre
level with, or even in FRONT of, the measured surface. If that is what the
decoy's fits actually do, then

    centre_depth = |centre| - min(|point|)   over the cluster

separates them on a property of the OBJECT rather than of A's route, costs
three lines, and needs no new threshold beyond "positive, and about r".

This measures the distribution before anything is changed. Truth labels each
frame; the statistic itself never sees truth.
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

import landmark_detector as LD  # noqa: E402

STUB_XY = (4.56534, -1.926)
NEAR_M = 0.25


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def depths(bag: Path, b_xy, radius: float) -> dict:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    scan_type, truth_type = get_message(types["/scan"]), get_message(types["/sim/ground_truth"])
    pose = None
    out = {"B": [], "stub": [], "other": []}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/sim/ground_truth":
            p = deserialize_message(data, truth_type).pose.pose
            pose = (p.position.x, p.position.y, _yaw(p.orientation))
            continue
        if topic != "/scan" or pose is None:
            continue
        msg = deserialize_message(data, scan_type)
        points = LD.scan_to_xy(msg.ranges, msg.angle_min, msg.angle_increment,
                               msg.range_min, msg.range_max)
        # Re-cluster exactly as `candidates` does, so the group behind each
        # accepted fit is the one whose depth we measure.
        cursor = 0
        for group in LD.cluster(points, radius * 2.0 * LD.CLUSTER_GAP_FACTOR):
            start, end = cursor, cursor + len(group) - 1
            cursor = end + 1
            if len(group) < LD.MIN_POINTS:
                continue
            fit = LD.fit_circle(group)
            if fit is None:
                continue
            cx, cy, r, residual = fit
            if residual > radius * LD.MAX_RESIDUAL_FRACTION:
                continue
            if abs(r - radius) > radius * LD.MAX_RADIUS_ERROR_FRACTION:
                continue
            if math.dist(group[0], group[-1]) > radius * 2.0 * LD.MAX_CHORD_FACTOR:
                continue
            if not LD.is_isolated(points, start, end):
                continue
            depth = math.hypot(cx, cy) - min(math.hypot(px, py) for px, py in group)
            cos_yaw, sin_yaw = math.cos(pose[2]), math.sin(pose[2])
            world = (pose[0] + cx * cos_yaw - cy * sin_yaw,
                     pose[1] + cx * sin_yaw + cy * cos_yaw)
            key = ("B" if math.dist(world, b_xy) <= NEAR_M
                   else "stub" if math.dist(world, STUB_XY) <= NEAR_M else "other")
            out[key].append(depth)
    return out


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    manifest = json.loads(Path("out/corridor.manifest.json").read_text(encoding="utf-8"))
    b_x, b_y, _ = manifest["actors"]["b_xyz_m"]
    radius = float(manifest["actors"]["b_radius_m"])
    print(f"B radius {radius} m -- a convex fit should have centre_depth ~= {radius} m, "
          f"and NEVER <= 0.\n")

    totals = {"B": [], "stub": [], "other": []}
    for argument in sys.argv[1:]:
        got = depths(Path(argument), (b_x, b_y), radius)
        for key in totals:
            totals[key].extend(got[key])
        print(f"  {Path(argument).name}: " + "  ".join(
            f"{k} n={len(v):5}" for k, v in got.items()))

    print("\n  pooled centre_depth (m)")
    for key, values in totals.items():
        if not values:
            continue
        values.sort()
        n = len(values)
        negative = sum(1 for v in values if v <= 0.0)
        print(f"    {key:6} n={n:6}  min {values[0]:+.4f}  p05 {values[n // 20]:+.4f}  "
              f"median {values[n // 2]:+.4f}  p95 {values[min(n - 1, 19 * n // 20)]:+.4f}  "
              f"max {values[-1]:+.4f}   depth<=0: {negative} ({100.0 * negative / n:.1f}%)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
