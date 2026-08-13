#!/usr/bin/env python3
"""Which STAGE of the yaw chain introduces the scale error, and does the corner?

    python3 tools/corridor_yaw_stage_audit.py \
        --bag ~/…/bags/20260812-184959-isaac-d67 \
        --manifest out/corridor.manifest.json --profile nominal_m6_n3 \
        --out out/evidence/robot-a-gate/yaw-stage-nominal.json

Reads bags. Starts nothing, drives nothing, needs no GPU.

THE QUESTION
------------
The 2026-08-12 acceptance runs failed the transit gate on yaw scale: 1.166 on
`nominal_m6_n3`, 1.108 on `wide_corner_m6_n4_5`, and **1.060 -- inside the
+/-0.1 bound -- on `uniform_m6_n6`, the profile with no tight corner**. That
pattern is a hypothesis with a shape: the excess is made at the corner arc.

Two separable questions follow, and this answers both on the same samples:

1. **WHICH STAGE.** truth -> `/imu` (the twin's raw gyro) -> `/odom` (after
   robot_localization). A ratio that is 1.00 at the sensor and 1.17 at the
   filter is a fusion problem; one that is already 1.17 at the sensor is a twin
   problem, and they live in different repositories.
2. **WHERE ON THE ROUTE.** The same three ratios computed over the corner arc
   alone and over the straights alone.

WHAT IS NOT HERE, AND WHY
-------------------------
**`/imu/data` -- the madgwick output -- is in no bag.** The session recorder's
topic list (`yahboomcar-ros2/tools/_session_record.py:55-57`) carries `/imu`
and not `/imu/data`, so the filter stage cannot be measured offline. The only
existing measurement of it is the LIVE tap each run's `gate.json` already
carries as `yaw_chain_deg`, and `--gate-json` reconciles this audit against it.

That reconciliation is worth doing rather than skipping. `wide_corner`'s live
tap reports `/imu` -> `/imu/data` as x1.095, and `imu_filter_madgwick`
republishes `angular_velocity` UNCHANGED -- an identity step cannot scale
anything. `uniform`'s tap reports the two as bit-identical, which is the
control. So x1.095 is a claim about the RECEIVER, not the filter, and the bag
can falsify it because the bag lost no messages.

TWO LABELLINGS, CROSS-CHECKED
-----------------------------
The arc is labelled **geometrically**, from the manifest's own trajectory legs
(`trajectory_from_manifest`): a sample is in the corner when its truth position
projects onto the arc segment. It is ALSO labelled **kinematically**, by truth's
own yaw rate. Neither is trusted alone -- agreeing labels mean the frames are
registered, and disagreeing ones mean something is wrong with the comparison
rather than with the robot.

SIGNED, NEVER ABSOLUTE
----------------------
Rotation is summed SIGNED. Summing |delta| accumulates the yaw noise of every
sample: on a 453 s run at 11 Hz that scored 5496 deg of "rotation" for a robot
that turned 810 (`corridor_transit_audit.py:138-145`). The same discipline
applies to the gyro integral, which is integrated on its own stamps.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(os.path.abspath(__file__)).parent.parent
sys.path.insert(0, str(ROOT / "src" / "corridor_scene"))
sys.path.insert(0, str(ROOT / "tools"))

#: A truth sample is "turning" when its own yaw rate exceeds this. The
#: kinematic cross-check on the geometric arc label, not a substitute for it.
TURNING_RAD_S = 0.15

#: How far off the authored arc a truth sample may be and still count as being
#: on it. The robot does not drive the authored arc -- it drives its own path
#: around the same corner -- so the label is a corridor around the arc, not the
#: arc itself. 0.35 m is comfortably wider than the measured 2 cm SLAM-vs-truth
#: divergence and narrower than the corner's own clear width.
ARC_CORRIDOR_M = 0.35


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def read_bag(bag: str) -> dict:
    """Truth and EKF poses, plus the raw gyro's yaw RATE on its own stamps."""

    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from sensor_msgs.msg import Imu

    poses = {"/odom": "ekf", "/sim/ground_truth": "truth", "/odom_raw": "wheel"}
    tracks: dict[str, list] = {"ekf": [], "truth": [], "wheel": [], "imu": []}

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        seconds = stamp * 1e-9
        if topic in poses:
            message = deserialize_message(data, Odometry)
            pose = message.pose.pose
            tracks[poses[topic]].append(
                (seconds, pose.position.x, pose.position.y, yaw_of(pose.orientation))
            )
        elif topic == "/imu":
            message = deserialize_message(data, Imu)
            # The twin publishes no orientation (publishOrientation: False,
            # orientation_covariance[0] = -1), so the gyro's yaw RATE is the
            # only yaw this topic carries. It is integrated, not differenced.
            tracks["imu"].append((seconds, float(message.angular_velocity.z)))
    return tracks


def arc_window(manifest: dict, profile: str) -> dict:
    """The corner arc's centre, radius and station interval, from the manifest."""

    from scene.trajectory import trajectory_from_manifest

    legs = manifest["profiles"][profile]["delivery_trajectory"]
    trajectory = trajectory_from_manifest(legs)
    approach = float(legs["approach_length_m"])
    radius = float(legs["arc_radius_m"])
    sweep = float(legs["arc_sweep_rad"])
    centre = [float(v) for v in legs["arc_center_xy_m"]]
    # THE ROUTE HAS TWO ARCS AND THEY TURN OPPOSITE WAYS. The corner arc swings
    # A off the corridor onto the street; the delivery arc swings it back in
    # toward B. Labelling only the first one puts ~90 deg of counter-rotation
    # into a bucket called "straight", which is how the first version of this
    # audit reported a 14 deg "straight" on a route with no such thing.
    return {
        "corner": {
            "centre_xy_m": centre,
            "radius_m": radius,
            "sweep_rad": sweep,
            "sweep_deg": math.degrees(sweep),
            "arc_length_m": radius * sweep,
            "station_start_m": approach,
            "station_end_m": approach + radius * sweep,
        },
        "delivery": {
            "centre_xy_m": [float(v) for v in legs["delivery_arc_center_xy_m"]],
            "radius_m": float(legs["delivery_arc_radius_m"]),
            "sweep_rad": float(legs["delivery_arc_sweep_rad"]),
            "sweep_deg": math.degrees(float(legs["delivery_arc_sweep_rad"])),
        },
        "trajectory": trajectory,
    }


def label_on_ring(truth: list[tuple], leg: dict) -> list[bool]:
    """True where the truth sample sits in the corridor around one arc.

    A distance-to-centre test with a tolerance band. The robot does not drive
    the authored arc -- it drives its own path around the same corner -- so the
    label is a corridor around the arc rather than the arc itself.
    """

    cx, cy = leg["centre_xy_m"]
    radius = leg["radius_m"]
    return [
        abs(math.hypot(x - cx, y - cy) - radius) <= ARC_CORRIDOR_M
        for _t, x, y, _yaw in truth
    ]


#: Below this speed the robot is not transiting. Used to bound the audit to the
#: drive itself, because a bag also contains bringup, the contract check and
#: teardown, and a signed rotation summed over all of it answers a different
#: question from the one the transit gate asks.
MOVING_M_S = 0.02


def transit_span(truth: list[tuple]) -> tuple[float, float] | None:
    """First to last moment truth was actually moving."""

    moving = []
    for earlier, later in zip(truth, truth[1:], strict=False):
        dt = later[0] - earlier[0]
        if dt <= 0:
            continue
        speed = math.dist((earlier[1], earlier[2]), (later[1], later[2])) / dt
        if speed > MOVING_M_S:
            moving.append(later[0])
    return (moving[0], moving[-1]) if len(moving) >= 2 else None


def label_kinematic(truth: list[tuple]) -> list[bool]:
    """True where truth's own yaw rate says the robot is turning."""

    labels = [False] * len(truth)
    for index in range(1, len(truth)):
        dt = truth[index][0] - truth[index - 1][0]
        if dt <= 0:
            continue
        rate = abs(wrap(truth[index][3] - truth[index - 1][3])) / dt
        labels[index] = rate > TURNING_RAD_S
    return labels


def _interval_spans(stamps: list[float], labels: list[bool]) -> list[tuple[float, float]]:
    """Contiguous [t0, t1] spans where the label holds."""

    spans, start = [], None
    for stamp, flag in zip(stamps, labels, strict=True):
        if flag and start is None:
            start = stamp
        elif not flag and start is not None:
            spans.append((start, stamp))
            start = None
    if start is not None:
        spans.append((start, stamps[-1]))
    return spans


def _in_spans(stamp: float, spans: list[tuple[float, float]]) -> bool:
    return any(low <= stamp <= high for low, high in spans)


def signed_rotation(track: list[tuple], spans=None) -> float:
    """Signed cumulative heading change, optionally restricted to spans."""

    total = 0.0
    for earlier, later in zip(track, track[1:], strict=False):
        if spans is not None and not _in_spans(later[0], spans):
            continue
        total += wrap(later[3] - earlier[3])
    return total


def integrated_gyro(samples: list[tuple], spans=None) -> float:
    """Integrate a yaw-rate series on its own stamps, optionally in spans."""

    total = 0.0
    for earlier, later in zip(samples, samples[1:], strict=False):
        dt = later[0] - earlier[0]
        if dt <= 0 or dt > 1.0:
            continue
        if spans is not None and not _in_spans(later[0], spans):
            continue
        total += 0.5 * (earlier[1] + later[1]) * dt
    return total


def _stage_row(truth_deg: float, imu_deg: float, ekf_deg: float, wheel_deg: float) -> dict:
    def ratio(value: float) -> float | None:
        # Below ~15 deg of real rotation a ratio is two small numbers dividing,
        # and the answer says more about noise than about scale.
        return round(value / truth_deg, 4) if abs(truth_deg) > 15.0 else None

    return {
        "truth_deg": round(truth_deg, 3),
        "imu_deg": round(imu_deg, 3),
        "ekf_deg": round(ekf_deg, 3),
        "wheel_deg": round(wheel_deg, 3),
        "imu_over_truth": ratio(imu_deg),
        "ekf_over_truth": ratio(ekf_deg),
        "wheel_over_truth": ratio(wheel_deg),
        "ekf_over_imu": (
            round(ekf_deg / imu_deg, 4) if abs(imu_deg) > 15.0 else None
        ),
    }


def audit(tracks: dict, arc: dict) -> dict:
    truth, ekf, wheel, imu = (
        tracks["truth"], tracks["ekf"], tracks["wheel"], tracks["imu"]
    )
    if len(truth) < 2 or len(imu) < 2 or len(ekf) < 2:
        return {"available": False, "reason": "a required track is empty"}

    stamps = [row[0] for row in truth]
    corner = label_on_ring(truth, arc["corner"])
    delivery = label_on_ring(truth, arc["delivery"])
    # A sample on both rings belongs to the corner: the delivery arc is the
    # later, smaller manoeuvre and the corner is the one under test.
    delivery = [d and not c for d, c in zip(delivery, corner, strict=True)]
    straight = [not (c or d) for c, d in zip(corner, delivery, strict=True)]
    kinematic = label_kinematic(truth)

    arc_spans = _interval_spans(stamps, corner)
    delivery_spans = _interval_spans(stamps, delivery)
    straight_spans = _interval_spans(stamps, straight)
    kinematic_spans = _interval_spans(stamps, kinematic)
    transit = transit_span(truth)
    transit_spans = [transit] if transit else None

    def segment(spans) -> dict:
        return _stage_row(
            math.degrees(signed_rotation(truth, spans)),
            math.degrees(integrated_gyro(imu, spans)),
            math.degrees(signed_rotation(ekf, spans)),
            math.degrees(signed_rotation(wheel, spans)),
        )

    turning = [c or d for c, d in zip(corner, delivery, strict=True)]
    agreement = sum(1 for a, b in zip(turning, kinematic, strict=True) if a == b)
    return {
        "available": True,
        "samples": {name: len(track) for name, track in tracks.items()},
        "arc": {key: value for key, value in arc.items() if key != "trajectory"},
        "labels": {
            "corner_arc_samples": sum(corner),
            "delivery_arc_samples": sum(delivery),
            "straight_samples": sum(straight),
            "kinematic_turning_samples": sum(kinematic),
            "agreement_fraction": round(agreement / len(turning), 4),
            "corner_seconds": round(sum(high - low for low, high in arc_spans), 2),
            "transit_span_s": (
                [round(transit[0] - stamps[0], 2), round(transit[1] - stamps[0], 2)]
                if transit else None
            ),
        },
        "whole_bag": segment(None),
        "transit_only": segment(transit_spans),
        "corner_arc_only": segment(arc_spans),
        "delivery_arc_only": segment(delivery_spans),
        "straight_only": segment(straight_spans),
        "turning_only_kinematic": segment(kinematic_spans),
    }


def reconcile(report: dict, gate_path: Path) -> dict:
    """Compare this audit's `/imu` integral against the run's LIVE tap."""

    gate = json.loads(gate_path.read_text(encoding="utf-8"))
    chain = gate.get("yaw_chain_deg") or {}
    if not chain:
        return {"available": False, "reason": "gate.json carries no yaw_chain_deg"}

    bag_imu = report["transit_only"]["imu_deg"]
    live_imu = chain.get("imu")
    live_filtered = chain.get("imu_data")
    verdict = None
    if live_imu is not None and live_filtered is not None and abs(live_imu) > 1e-6:
        step = live_filtered / live_imu
        # imu_filter_madgwick republishes angular_velocity unchanged, so this
        # step is an identity. Anything else is receiver-side loss on one of the
        # two live subscriptions, and the bag is the arbiter because it lost
        # nothing.
        verdict = (
            "identity, as an angular-velocity passthrough must be"
            if abs(step - 1.0) < 0.01
            else f"live tap reports x{step:.4f} across an identity filter -- "
                 "receiver-side sample loss, not a filter effect"
        )
    return {
        "available": True,
        "live_yaw_chain_deg": chain,
        "bag_imu_deg": bag_imu,
        "bag_vs_live_imu_ratio": (
            round(bag_imu / live_imu, 4) if live_imu else None
        ),
        "madgwick_step_verdict": verdict,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bag", required=True)
    parser.add_argument("--manifest", default="out/corridor.manifest.json")
    parser.add_argument("--profile", required=True)
    parser.add_argument("--gate-json", default=None,
                        help="the run's gate.json, for the live-tap reconciliation")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    manifest = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    arc = arc_window(manifest, args.profile)
    report = audit(read_bag(args.bag), arc)
    report["bag"] = args.bag
    report["profile"] = args.profile
    if args.gate_json:
        report["live_reconciliation"] = reconcile(report, Path(args.gate_json))

    destination = Path(args.out)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    if report.get("available"):
        print(f"{args.profile}  corner {report['arc']['corner']['sweep_deg']:.1f} deg "
              f"over {report['labels']['corner_seconds']:.1f} s, "
              f"delivery arc {report['arc']['delivery']['sweep_deg']:.1f} deg, "
              f"label agreement {report['labels']['agreement_fraction']:.2f}")
        header = f"{'segment':<22}{'truth':>10}{'imu/tr':>9}{'ekf/tr':>9}{'ekf/imu':>9}"
        print(header)
        for name in ("whole_bag", "transit_only", "corner_arc_only",
                     "delivery_arc_only", "straight_only", "turning_only_kinematic"):
            row = report[name]
            def show(value):
                return f"{value:>9.4f}" if value is not None else f"{'--':>9}"
            print(f"{name:<22}{row['truth_deg']:>10.2f}"
                  f"{show(row['imu_over_truth'])}{show(row['ekf_over_truth'])}"
                  f"{show(row['ekf_over_imu'])}")
    print(f"written: {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
