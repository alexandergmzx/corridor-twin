#!/usr/bin/env python3
"""How far does the robot think it went, against how far it went? Offline.

    python3 tools/odometry_scale_audit.py --bag <session bag dir> --out audit.json
    python3 tools/odometry_scale_audit.py --bags ~/…/bags/20260812-*-isaac-d67 \
        --out out/evidence/robot-a-gate/odometry-scale.json

Reads bags. Starts nothing, drives nothing, needs no GPU. That is the point:
the linear scale question was about to be answered by spending Isaac minutes on
a fresh transit, and thirteen recorded transits already carry the answer.

TWO SOURCES, TWO DIFFERENT QUESTIONS
------------------------------------
* ``/odom_raw`` is the twin's WHEEL odometry: joint velocities through an
  effective rolling radius (``sim_runner.py:720``). Its ratio against truth is a
  property of the drive conversion.
* ``/odom`` is the EKF's output, which under ``pn-fix`` fuses wheel **vx** and
  IMU **yaw rate** only. Its ratio is the drive conversion plus the filter.

Splitting them is what says whether a scale error is in the wheels or in the
fusion, and no single-number "drift fraction" can.

WINDOWED MEDIAN, NOT TOTAL PATH LENGTH
--------------------------------------
Summing |dp| over every consecutive sample turns position noise into distance:
a stationary robot accumulates metres. The transit gate's own ``midpoint_drift``
compares total path lengths, and on the 13:16 run it read 0.159 over 0.77 m of
travel -- a ratio of two short, noisy sums.

Here each source's distance is accumulated over WINDOWS (default 1 s), windows
in which truth barely moved are dropped, and the reported scale is the median of
the per-window ratios. That is the same shape as the yaw measurement in
``NOTES-fusion-anomaly.md``, which was corrected once already for exactly this
reason: a peak-versus-cap comparison conflated a transient with a scale factor.

The median is reported with the inter-quartile range beside it, because a scale
error that is stable and one that swings are different faults with different
fixes, and a single number hides which one you have.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

#: Seconds per accumulation window.
WINDOW_S = 1.0

#: A window in which truth moved less than this says nothing about scale: it is
#: a ratio of two noise floors. 0.02 m at the governed 0.35 m/s cap is ~6% of a
#: window's travel, so this drops standing still, not slow driving.
MIN_WINDOW_TRAVEL_M = 0.02


def yaw_of(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def read_tracks(bag: str) -> dict[str, list[tuple[float, float, float, float]]]:
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    wanted = {"/odom": "ekf", "/odom_raw": "wheel", "/sim/ground_truth": "truth"}
    tracks: dict[str, list] = {name: [] for name in wanted.values()}

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        name = wanted.get(topic)
        if name is None:
            continue
        message = deserialize_message(data, Odometry)
        pose = message.pose.pose
        tracks[name].append(
            (stamp * 1e-9, pose.position.x, pose.position.y, yaw_of(pose.orientation))
        )
    return tracks


def windowed_travel(track: list[tuple], window_s: float) -> dict[int, float]:
    """Distance covered inside each window index, keyed by window."""

    if len(track) < 2:
        return {}
    start = track[0][0]
    travelled: dict[int, float] = {}
    for earlier, later in zip(track, track[1:], strict=False):
        index = int((later[0] - start) // window_s)
        step = math.dist((earlier[1], earlier[2]), (later[1], later[2]))
        travelled[index] = travelled.get(index, 0.0) + step
    return travelled


#: A window whose heading changed by more than this is a TURNING window. Wheel
#: odometry on a four-wheel skid-steer slips most while turning, and a drive
#: conversion error does not care whether the robot is turning -- so separating
#: the two populations is what says whether a scale deficit is a wrong radius or
#: ordinary slip. 5 deg over a 1 s window at the 0.4 rad/s command cap is well
#: inside "driving straight".
STRAIGHT_WINDOW_YAW_DEG = 5.0


def windowed_turn(track: list[tuple], window_s: float) -> dict[int, float]:
    """Absolute heading change inside each window, in degrees."""

    if len(track) < 2:
        return {}
    start = track[0][0]
    turned: dict[int, float] = {}
    for earlier, later in zip(track, track[1:], strict=False):
        index = int((later[0] - start) // window_s)
        step = (later[3] - earlier[3] + math.pi) % (2.0 * math.pi) - math.pi
        turned[index] = turned.get(index, 0.0) + step
    return {index: abs(math.degrees(value)) for index, value in turned.items()}


def scale_against_truth(source: list[tuple], truth: list[tuple], window_s: float) -> dict:
    """Median per-window ratio of a source's travel to truth's."""

    if len(source) < 2 or len(truth) < 2:
        return {"available": False, "reason": "one of the tracks is empty"}

    # Both tracks are indexed from TRUTH's first stamp, so the windows line up.
    origin = truth[0][0]
    shifted = [(stamp - origin, x, y, yaw) for stamp, x, y, yaw in source]
    truth_shifted = [(stamp - origin, x, y, yaw) for stamp, x, y, yaw in truth]

    source_travel = windowed_travel([(t + origin, *rest) for t, *rest in shifted], window_s)
    truth_travel = windowed_travel([(t + origin, *rest) for t, *rest in truth_shifted], window_s)

    truth_turn = windowed_turn(
        [(t + origin, *rest) for t, *rest in truth_shifted], window_s
    )

    ratios, straight_ratios = [], []
    for index, distance in sorted(truth_travel.items()):
        if distance < MIN_WINDOW_TRAVEL_M or index not in source_travel:
            continue
        ratio = source_travel[index] / distance
        ratios.append(ratio)
        if truth_turn.get(index, 0.0) <= STRAIGHT_WINDOW_YAW_DEG:
            straight_ratios.append(ratio)
    if len(ratios) < 5:
        return {"available": False, "reason": f"only {len(ratios)} windows with real motion"}

    def spread(values: list[float]) -> dict:
        values = sorted(values)
        quartile = len(values) // 4
        return {
            "windows": len(values),
            "median": round(statistics.median(values), 4),
            "mean": round(statistics.fmean(values), 4),
            "iqr": [round(values[quartile], 4), round(values[-quartile - 1], 4)],
            "min": round(values[0], 4),
            "max": round(values[-1], 4),
        }

    report = {"available": True, **spread(ratios)}
    # The two populations, separated. A wrong wheel radius shows up equally in
    # both; slip shows up in the turning one.
    report["straight_only"] = (
        spread(straight_ratios) if len(straight_ratios) >= 5
        else {"windows": len(straight_ratios), "median": None}
    )
    # The blunt instrument beside the careful one, so the difference between
    # them is visible rather than assumed.
    report["total_path_ratio"] = round(
        sum(source_travel.values()) / sum(truth_travel.values()), 4
    )
    return report


def audit_bag(bag: str, window_s: float = WINDOW_S) -> dict:
    tracks = read_tracks(bag)
    truth = tracks["truth"]
    report = {
        "bag": Path(bag).name,
        "window_s": window_s,
        "samples": {name: len(track) for name, track in tracks.items()},
        "truth_path_length_m": round(
            sum(
                math.dist((a[1], a[2]), (b[1], b[2]))
                for a, b in zip(truth, truth[1:], strict=False)
            ),
            3,
        ),
    }
    for name in ("wheel", "ekf"):
        report[name] = scale_against_truth(tracks[name], truth, window_s)
    return report


def summarise(reports: list[dict]) -> dict:
    """Across bags. One run's ratio is an anecdote; thirteen is a distribution."""

    summary = {}
    for name in ("wheel", "ekf"):
        available = [r[name] for r in reports if r.get(name, {}).get("available")]
        if not available:
            summary[name] = {"available": False}
            continue

        def across(values: list[float]) -> dict:
            values = sorted(values)
            return {
                "bags": len(values),
                "median_of_medians": round(statistics.median(values), 4),
                "min": round(values[0], 4),
                "max": round(values[-1], 4),
                "spread": round(values[-1] - values[0], 4),
            }

        summary[name] = {"available": True, **across([r["median"] for r in available])}
        straight = [
            r["straight_only"]["median"] for r in available
            if r["straight_only"].get("median") is not None
        ]
        summary[name]["straight_only"] = across(straight) if straight else {"bags": 0}
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bag", action="append", default=[], help="one bag directory")
    parser.add_argument("--bags", nargs="*", default=[], help="many bag directories")
    parser.add_argument("--window-s", type=float, default=WINDOW_S)
    parser.add_argument("--out", type=Path, required=True)
    arguments = parser.parse_args()

    bags = [*arguments.bag, *arguments.bags]
    if not bags:
        print("no bags given", file=sys.stderr)
        return 2

    reports = []
    for bag in bags:
        try:
            report = audit_bag(bag, arguments.window_s)
        except Exception as error:  # noqa: BLE001 - one bad bag must not lose twelve
            reports.append({"bag": Path(bag).name, "error": str(error)})
            print(f"  {Path(bag).name}: ERROR {error}", file=sys.stderr)
            continue
        reports.append(report)
        wheel, ekf = report["wheel"], report["ekf"]
        print(
            f"  {report['bag']:32s} truth {report['truth_path_length_m']:7.3f} m  "
            f"wheel {wheel.get('median', '--')!s:>7}  ekf {ekf.get('median', '--')!s:>7}"
        )

    payload = {"window_s": arguments.window_s,
               "min_window_travel_m": MIN_WINDOW_TRAVEL_M,
               "bags": reports, "summary": summarise(reports)}
    arguments.out.parent.mkdir(parents=True, exist_ok=True)
    arguments.out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"\nwritten: {arguments.out}")
    print(json.dumps(payload["summary"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
