#!/usr/bin/env python3
"""Verify the live Isaac camera and clock streams from an external ROS 2 node."""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image

IMAGE_TOPIC = "/robot/front_camera/image_raw"
CAMERA_INFO_TOPIC = "/robot/front_camera/camera_info"
CLOCK_TOPIC = "/clock"
FRAME_ID = "robot_front_camera_optical_frame"
WIDTH = 640
HEIGHT = 360
EXPECTED_RATE_HZ = 15.0


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--minimum-pairs", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=60.0)
    return parser.parse_args()


def stamp_ns(message: Image | CameraInfo) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


@dataclass
class Observations:
    images: dict[int, Image] = field(default_factory=dict)
    infos: dict[int, CameraInfo] = field(default_factory=dict)
    paired_stamps: set[int] = field(default_factory=set)
    clocks_ns: list[int] = field(default_factory=list)


class CameraContractProbe(Node):
    def __init__(self) -> None:
        super().__init__("corridor_camera_contract_probe")
        self.observations = Observations()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, IMAGE_TOPIC, self._on_image, sensor_qos)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_info, sensor_qos)
        self.create_subscription(Clock, CLOCK_TOPIC, self._on_clock, clock_qos)

    def _on_image(self, message: Image) -> None:
        key = stamp_ns(message)
        self.observations.images[key] = message
        self._pair(key)

    def _on_info(self, message: CameraInfo) -> None:
        key = stamp_ns(message)
        self.observations.infos[key] = message
        self._pair(key)

    def _on_clock(self, message: Clock) -> None:
        value = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        self.observations.clocks_ns.append(value)

    def _pair(self, key: int) -> None:
        if key in self.observations.images and key in self.observations.infos:
            self.observations.paired_stamps.add(key)


def _assert_sensor_qos(node: Node, topic: str) -> None:
    endpoints = node.get_publishers_info_by_topic(topic)
    if len(endpoints) != 1:
        raise AssertionError(f"expected one publisher on {topic}; found {len(endpoints)}")
    qos = endpoints[0].qos_profile
    if qos.reliability != ReliabilityPolicy.BEST_EFFORT:
        raise AssertionError(f"{topic} publisher is not best effort")
    if qos.durability != DurabilityPolicy.VOLATILE:
        raise AssertionError(f"{topic} publisher is not volatile")


def validate(node: CameraContractProbe, minimum_pairs: int) -> tuple[int, str, float]:
    observed = node.observations
    stamps = sorted(observed.paired_stamps)
    if len(stamps) < minimum_pairs:
        raise AssertionError(f"received only {len(stamps)} synchronized camera pairs")
    if stamps[0] <= 0:
        raise AssertionError("camera acquisition timestamp must be nonzero")
    if any(later <= earlier for earlier, later in zip(stamps, stamps[1:], strict=False)):
        raise AssertionError("camera acquisition timestamps are not strictly increasing")
    if len(observed.clocks_ns) < minimum_pairs:
        raise AssertionError("insufficient /clock samples")
    if any(
        later < earlier
        for earlier, later in zip(observed.clocks_ns, observed.clocks_ns[1:], strict=False)
    ):
        raise AssertionError("/clock moved backward")
    if max(observed.clocks_ns) < stamps[-1]:
        raise AssertionError("/clock did not reach the final image acquisition time")

    encodings: set[str] = set()
    for key in stamps:
        image = observed.images[key]
        info = observed.infos[key]
        if image.header.frame_id != FRAME_ID or info.header.frame_id != FRAME_ID:
            raise AssertionError("camera frame ID does not match the feed contract")
        if (image.width, image.height) != (WIDTH, HEIGHT):
            raise AssertionError("image dimensions do not match the feed contract")
        if (info.width, info.height) != (WIDTH, HEIGHT):
            raise AssertionError("CameraInfo dimensions do not match the image")
        if image.encoding not in {"rgb8", "bgr8"}:
            raise AssertionError(f"unsupported image encoding {image.encoding!r}")
        if len(image.data) < image.step * image.height:
            raise AssertionError("image payload is shorter than step * height")
        if info.k[0] <= 0.0 or info.k[4] <= 0.0 or info.k[8] != 1.0:
            raise AssertionError("CameraInfo intrinsic matrix is invalid")
        if info.distortion_model != "plumb_bob" or any(info.d):
            raise AssertionError("CameraInfo must describe the zero-distortion pinhole model")
        encodings.add(image.encoding)

    intervals_s = [
        (later - earlier) / 1_000_000_000
        for earlier, later in zip(stamps, stamps[1:], strict=False)
    ]
    measured_hz = 1.0 / statistics.median(intervals_s)
    if not 14.5 <= measured_hz <= 15.5:
        raise AssertionError(f"camera stamp rate {measured_hz:.3f} Hz is outside tolerance")
    _assert_sensor_qos(node, IMAGE_TOPIC)
    _assert_sensor_qos(node, CAMERA_INFO_TOPIC)
    _assert_sensor_qos(node, CLOCK_TOPIC)
    return len(stamps), ",".join(sorted(encodings)), measured_hz


def main() -> int:
    args = arguments()
    if args.minimum_pairs < 2:
        raise ValueError("--minimum-pairs must be at least 2")
    rclpy.init()
    node = CameraContractProbe()
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                len(node.observations.paired_stamps) >= args.minimum_pairs
                and len(node.observations.clocks_ns) >= args.minimum_pairs
            ):
                break
        pairs, encodings, measured_hz = validate(node, args.minimum_pairs)
        print(
            "ROS_CAMERA_PROBE_PASS",
            f"pairs={pairs}",
            f"encoding={encodings}",
            f"resolution={WIDTH}x{HEIGHT}",
            f"stamp_rate_hz={measured_hz:.3f}",
            "publishers=1,1,1",
            flush=True,
        )
    except Exception as exc:
        print(
            "ROS_CAMERA_PROBE_FAIL",
            f"error={type(exc).__name__}: {exc}",
            file=sys.stderr,
            flush=True,
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
