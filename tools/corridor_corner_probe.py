#!/usr/bin/env python3
"""What happens to A at the end of the corridor: is it WEDGED, or just lost?

    python3 tools/corridor_corner_probe.py --bag <session bag> \\
        --manifest out/corridor-small.manifest.json \\
        --out out/evidence/robot-a-gate/corner-probe.json

WHY THIS EXISTS
---------------
A reaches B. Measured from truth, governed Nav2 on a live map took A around the
corner to within 0.768 m of the delivery standoff, and then A drove back to its
spawn. The map diverges somewhere in there, and the operator's observation is
that it stays clean until the end of the corridor and then A "gets stuck".

"Stuck" is not one thing, and the two candidates need opposite fixes:

  * **WEDGED** -- A is commanded to move and does not, because the chassis is
    against something. The wheels turn anyway, so the encoders report a rotation
    that never happened. This is the documented near-wall failure of this stack
    (`docs/slam-research/near-wall-stability.md`; `simctl:641-644`): the wheel
    yaw channel "lies ~6-26x with the body blocked at a wall, and the map fans".
    The fix is geometric -- clearance, footprint, approach -- not SLAM tuning,
    which that study explicitly falsified on three bags.
  * **LOST** -- A moves freely and drives to the wrong place, because its pose
    estimate is wrong. The fix is in localization.

This probe separates them from a recorded bag, with no GPU and no rerun.

THE WEDGE SIGNATURE
-------------------
Commanded motion with no true motion, at the same instant, with the wheel yaw
rate diverging from the body's. All four channels are needed: `/cmd_vel` alone
says what was asked, truth alone says what happened, and only their conjunction
distinguishes "blocked" from "correctly stopped". `/scan`'s minimum range says
whether there was anything to be blocked BY.

Simulator truth is an evaluation input (CLAUDE.md invariant 1); nothing A's
stack subscribes to reads this file.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

#: Commanded-but-not-moving. The governor's floor is well under this, and A's
#: nav speed is 0.22 m/s, so 0.03 m/s of true speed against a command above
#: 0.05 m/s is not "creeping slowly" -- it is not going anywhere.
COMMANDED_MPS = 0.05
STOPPED_MPS = 0.03

#: The free-floor encoder lie is ~2.9x on this chassis; the blocked regime runs
#: 6-26x. 3.0 separates them, and is quoted from the near-wall study rather than
#: chosen here.
WHEEL_LIE_RATIO = 3.0

#: Sustained, not instantaneous. Rows are one second each, so this is a count of
#: consecutive seconds. One or two are an acceleration transient or a governor
#: brake; the measured wedges ran 40 and 41 s, so 3 separates them by more than
#: an order of magnitude and does not need to be finer.
WEDGE_MIN_S = 3.0


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def read_channels(bag: str) -> dict:
    """Every channel timed by its own header stamp, except /cmd_vel.

    Twist carries no header, so commands are timed by bag receive time. That is
    the one unavoidable clock mix here, and it is acceptable because commands
    are only ever compared over ~1 s windows, far longer than the transport
    skew.
    """

    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))

    truth, wheel, command, scan = [], [], [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        received = stamp * 1e-9
        if topic == "/sim/ground_truth":
            message = deserialize_message(data, Odometry)
            pose = message.pose.pose
            truth.append((
                message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                pose.position.x, pose.position.y, yaw_of(pose.orientation),
            ))
        elif topic == "/odom_raw":
            message = deserialize_message(data, Odometry)
            wheel.append((
                message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                message.twist.twist.angular.z,
            ))
        elif topic == "/cmd_vel":
            message = deserialize_message(data, Twist)
            command.append((received, message.linear.x, message.angular.z))
        elif topic == "/scan":
            message = deserialize_message(data, LaserScan)
            finite = [
                r for r in message.ranges
                if math.isfinite(r) and message.range_min <= r <= message.range_max
            ]
            scan.append((
                message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                min(finite) if finite else float("nan"),
            ))
    return {"truth": truth, "wheel": wheel, "command": command, "scan": scan}


def nearest(series, when: float, default=None):
    """Value of `series` nearest in time to `when`. Series are short; linear is fine."""

    if not series:
        return default
    best = min(series, key=lambda row: abs(row[0] - when))
    return best[1:] if len(best) > 2 else best[1]


def per_second(channels: dict) -> list[dict]:
    """One row per second of the run: what was asked, what happened, what was near."""

    truth = channels["truth"]
    if len(truth) < 2:
        return []
    start, end = truth[0][0], truth[-1][0]

    rows = []
    second = start
    while second < end - 1.0:
        window = [row for row in truth if second <= row[0] < second + 1.0]
        if len(window) >= 2:
            distance = sum(
                math.dist((a[1], a[2]), (b[1], b[2]))
                for a, b in zip(window, window[1:], strict=False)
            )
            turned = sum(wrap(b[3] - a[3]) for a, b in zip(window, window[1:], strict=False))
            span = window[-1][0] - window[0][0]
            truth_mps = distance / span if span > 0 else 0.0
            truth_wz = turned / span if span > 0 else 0.0

            commands = [row for row in channels["command"] if second <= row[0] < second + 1.0]
            commanded_mps = max((abs(row[1]) for row in commands), default=0.0)
            commanded_wz = max((abs(row[2]) for row in commands), default=0.0)

            wheels = [row for row in channels["wheel"] if second <= row[0] < second + 1.0]
            wheel_wz = (sum(abs(row[1]) for row in wheels) / len(wheels)) if wheels else 0.0

            ratio = (wheel_wz / abs(truth_wz)) if abs(truth_wz) > 0.05 else None

            rows.append({
                "t": round(second - start, 2),
                "truth_mps": round(truth_mps, 4),
                "truth_wz": round(truth_wz, 4),
                "commanded_mps": round(commanded_mps, 4),
                "commanded_wz": round(commanded_wz, 4),
                "wheel_wz": round(wheel_wz, 4),
                "wheel_truth_ratio": round(ratio, 3) if ratio is not None else None,
                "min_scan_m": round(nearest(channels["scan"], second, float("nan")), 4),
                "x": round(window[-1][1], 4),
                "y": round(window[-1][2], 4),
            })
        second += 1.0
    return rows


def find_wedge(rows: list[dict]) -> dict:
    """The longest stretch of commanded-but-not-moving, and what was near it."""

    best_start = best_len = 0
    run_start = None
    for index, row in enumerate(rows):
        blocked = (
            max(row["commanded_mps"], row["commanded_wz"]) > COMMANDED_MPS
            and row["truth_mps"] < STOPPED_MPS
        )
        if blocked:
            run_start = index if run_start is None else run_start
            if index - run_start + 1 > best_len:
                best_start, best_len = run_start, index - run_start + 1
        else:
            run_start = None

    if best_len < WEDGE_MIN_S:
        return {"wedged": False, "longest_blocked_s": best_len}

    stretch = rows[best_start:best_start + best_len]
    clearances = [row["min_scan_m"] for row in stretch if math.isfinite(row["min_scan_m"])]
    ratios = [row["wheel_truth_ratio"] for row in stretch if row["wheel_truth_ratio"]]
    return {
        "wedged": True,
        "longest_blocked_s": best_len,
        "from_t": stretch[0]["t"],
        "to_t": stretch[-1]["t"],
        "at_position": [stretch[0]["x"], stretch[0]["y"]],
        "min_clearance_m": round(min(clearances), 4) if clearances else None,
        "peak_wheel_truth_ratio": round(max(ratios), 3) if ratios else None,
        "commanded_through_it_mps": round(
            max(row["commanded_mps"] for row in stretch), 4
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bag", required=True)
    parser.add_argument("--manifest")
    parser.add_argument("--out", required=True)
    parser.add_argument("--print-rows", type=int, default=0,
                        help="also print this many rows around the wedge")
    arguments = parser.parse_args()

    channels = read_channels(arguments.bag)
    rows = per_second(channels)
    wedge = find_wedge(rows)

    peak_ratio = max(
        (row["wheel_truth_ratio"] for row in rows if row["wheel_truth_ratio"]),
        default=None,
    )
    report = {
        "bag": arguments.bag,
        "seconds": len(rows),
        "wedge": wedge,
        "peak_wheel_truth_ratio": peak_ratio,
        "blocked_regime_seconds": sum(
            1 for row in rows if (row["wheel_truth_ratio"] or 0) > WHEEL_LIE_RATIO
        ),
        "thresholds": {
            "commanded_mps": COMMANDED_MPS,
            "stopped_mps": STOPPED_MPS,
            "wheel_lie_ratio": WHEEL_LIE_RATIO,
        },
        "rows": rows,
    }

    if arguments.manifest:
        import sys
        sys.path.insert(0, str(Path(__file__).parent))
        from corridor_nav_gate import delivery_standoff_world

        manifest = json.loads(Path(arguments.manifest).read_text(encoding="utf-8"))
        standoff = delivery_standoff_world(manifest)
        distances = [
            (row["t"], math.dist((row["x"], row["y"]), standoff)) for row in rows
        ]
        closest_t, closest = min(distances, key=lambda pair: pair[1])
        report["delivery"] = {
            "standoff_world_m": [round(standoff[0], 4), round(standoff[1], 4)],
            "closest_approach_m": round(closest, 4),
            "closest_at_s": closest_t,
        }

    destination = Path(arguments.out)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    verdict = report["wedge"]
    print(f"bag              {Path(arguments.bag).name}")
    print(f"seconds          {report['seconds']}")
    if "delivery" in report:
        print(f"closest approach {report['delivery']['closest_approach_m']} m "
              f"at t+{report['delivery']['closest_at_s']}s")
    print(f"peak wheel/truth {report['peak_wheel_truth_ratio']}")
    print(f"blocked regime   {report['blocked_regime_seconds']} s over {WHEEL_LIE_RATIO}x")
    if verdict["wedged"]:
        print(f"**WEDGED** {verdict['longest_blocked_s']} s from t+{verdict['from_t']}s "
              f"at ({verdict['at_position'][0]}, {verdict['at_position'][1]}), "
              f"clearance {verdict['min_clearance_m']} m, "
              f"peak wheel/truth {verdict['peak_wheel_truth_ratio']}")
    else:
        print(f"not wedged (longest commanded-but-stopped stretch "
              f"{verdict['longest_blocked_s']} s)")

    if arguments.print_rows:
        anchor = verdict.get("from_t", report.get("delivery", {}).get("closest_at_s", 0))
        near = [row for row in rows if abs(row["t"] - anchor) <= arguments.print_rows]
        print(f"\n{'t':>7} {'truth':>7} {'cmd':>7} {'wheelwz':>8} {'ratio':>7} {'minscan':>8}")
        for row in near:
            print(f"{row['t']:>7.0f} {row['truth_mps']:>7.3f} {row['commanded_mps']:>7.3f} "
                  f"{row['wheel_wz']:>8.3f} "
                  f"{str(row['wheel_truth_ratio']):>7} {row['min_scan_m']:>8.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
