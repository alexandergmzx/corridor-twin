#!/usr/bin/env python3
"""Did the robot start the mission, or perform first? Ground truth, from a bag.

    python3 tools/startup_acceptance.py --bag <session bag> --out startup-acceptance.json

The startup circle was diagnosed three times from the wrong signal, so its
acceptance is measured from `/sim/ground_truth` -- what the robot actually did --
rather than from what anything commanded.

THREE CRITERIA, all of them about the beginning of a run:

* **Pre-transit rotation.** Cumulative |heading change| before the robot first
  makes real forward progress. The defect was 253 deg of it, from a health check
  driving a 0.4 m-radius arc on /cmd_vel before Nav2 existed.
* **Time to forward progress**, measured from GOAL-SEND -- for which the first
  `/cmd_vel_raw` message is the proxy, because Nav2 is now the only publisher
  there and it commands within a second of accepting the goal. Measuring from
  the first truth sample instead would charge the mission for SLAM and Nav2
  bring-up, which is 88 s of legitimate waiting.
* **Transit accuracy**, so a fix that quiets the start by breaking the mission
  is not mistaken for a fix. That one comes from the gate's own
  `world_frame_delivery`, and is passed in rather than recomputed here.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

#: Displacement from the start pose that counts as the transit having begun.
#: Larger than settling and sensor noise, smaller than any real leg.
PROGRESS_M = 0.30

#: Pinned acceptance, session 2026-08-12 P1.
MAX_PRE_TRANSIT_ROTATION_DEG = 45.0
MAX_SECONDS_TO_PROGRESS = 30.0
MAX_CLOSEST_APPROACH_M = 0.15


def first_nav_command_s(bag: str) -> float | None:
    """When Nav2 first commanded anything -- the goal-send proxy."""

    from geometry_msgs.msg import Twist
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic.endswith("/cmd_vel_raw") or topic == "/cmd_vel_raw":
            deserialize_message(data, Twist)
            return stamp * 1e-9
    return None


def measure(bag: str) -> dict:
    from odometry_scale_audit import read_tracks

    tracks = read_tracks(bag)
    truth = tracks["truth"]
    if len(truth) < 2:
        return {"available": False, "reason": "no ground truth in the bag"}

    goal_s = first_nav_command_s(bag)
    start = truth[0]
    t0 = start[0]
    progress_at = None
    rotation = 0.0
    rotation_at_progress = None

    for earlier, later in zip(truth, truth[1:], strict=False):
        step = abs((later[3] - earlier[3] + math.pi) % (2.0 * math.pi) - math.pi)
        if progress_at is None:
            rotation += step
            if math.dist((later[1], later[2]), (start[1], start[2])) >= PROGRESS_M:
                progress_at = later[0]
                rotation_at_progress = rotation

    # Where the robot stood when Nav2 first spoke: pre-transit rotation is about
    # everything before the mission, and progress is about everything after.
    if goal_s is not None:
        at_goal = min(truth, key=lambda row: abs(row[0] - goal_s))
        rotation_before_goal = 0.0
        for earlier, later in zip(truth, truth[1:], strict=False):
            if later[0] > goal_s:
                break
            rotation_before_goal += abs(
                (later[3] - earlier[3] + math.pi) % (2.0 * math.pi) - math.pi
            )
        progress_after_goal = None
        for row in truth:
            if row[0] < goal_s:
                continue
            if math.dist((row[1], row[2]), (at_goal[1], at_goal[2])) >= PROGRESS_M:
                progress_after_goal = row[0] - goal_s
                break
    else:
        rotation_before_goal = rotation
        progress_after_goal = None

    return {
        "available": True,
        "truth_samples": len(truth),
        "progress_threshold_m": PROGRESS_M,
        "goal_send_s_into_bag": round(goal_s - t0, 2) if goal_s else None,
        "pre_goal_rotation_deg": round(math.degrees(rotation_before_goal), 2),
        "seconds_to_progress_after_goal": (
            round(progress_after_goal, 2) if progress_after_goal is not None else None
        ),
        "pre_transit_rotation_deg": (
            round(math.degrees(rotation_at_progress), 2)
            if rotation_at_progress is not None
            else round(math.degrees(rotation), 2)
        ),
        "seconds_to_progress": (
            round(progress_at - t0, 2) if progress_at is not None else None
        ),
        "never_moved": progress_at is None,
    }


def verdict(measured: dict, closest_approach_m: float | None) -> dict:
    if not measured.get("available"):
        return {"pass": False, "failures": [measured.get("reason", "no measurement")]}

    failures = []
    rotation = measured["pre_goal_rotation_deg"]
    if rotation > MAX_PRE_TRANSIT_ROTATION_DEG:
        failures.append(
            f"pre-goal rotation {rotation} deg exceeds {MAX_PRE_TRANSIT_ROTATION_DEG}"
        )
    delay = measured["seconds_to_progress_after_goal"]
    if delay is None:
        failures.append(f"never moved {PROGRESS_M} m after the goal was sent")
    elif delay > MAX_SECONDS_TO_PROGRESS:
        failures.append(
            f"forward progress began {delay} s after goal-send, "
            f"limit {MAX_SECONDS_TO_PROGRESS}"
        )
    if closest_approach_m is None:
        failures.append("no delivery measurement to check against")
    elif closest_approach_m > MAX_CLOSEST_APPROACH_M:
        failures.append(
            f"closest approach {closest_approach_m} m exceeds "
            f"{MAX_CLOSEST_APPROACH_M} m -- the start was quieted at the "
            f"mission's expense"
        )
    return {"pass": not failures, "failures": failures}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bag", required=True)
    parser.add_argument("--gate", type=Path,
                        help="the run's gate.json, for world_frame_delivery")
    parser.add_argument("--out", type=Path, required=True)
    arguments = parser.parse_args()

    measured = measure(arguments.bag)
    closest = None
    if arguments.gate and arguments.gate.is_file():
        gate = json.loads(arguments.gate.read_text(encoding="utf-8"))
        delivery = gate.get("world_frame_delivery") or {}
        if delivery.get("available"):
            closest = delivery.get("closest_approach_m")

    report = {
        "bag": Path(arguments.bag).name,
        "thresholds": {
            "max_pre_transit_rotation_deg": MAX_PRE_TRANSIT_ROTATION_DEG,
            "max_seconds_to_progress": MAX_SECONDS_TO_PROGRESS,
            "max_closest_approach_m": MAX_CLOSEST_APPROACH_M,
        },
        "measured": measured,
        "closest_approach_m": closest,
        **verdict(measured, closest),
    }
    arguments.out.parent.mkdir(parents=True, exist_ok=True)
    arguments.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "thresholds"}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
