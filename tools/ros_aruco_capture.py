#!/usr/bin/env python3
"""Capture the production Isaac Image/CameraInfo stream for offline gating."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image

CAPTURE_CONTRACT = {
    "image_topic": "/p_cam/image_raw",
    "camera_info_topic": "/p_cam/camera_info",
    "clock_topic": "/clock",
    "frame_id": "p_cam_optical_frame",
    "width": 640,
    "height": 360,
    "encoding": "rgb8",
}


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("out/evidence/static-fiducials/capture"),
    )
    parser.add_argument("--minimum-pairs", type=int, default=18)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--idle-after-minimum", type=float, default=2.0)
    return parser.parse_args()


def stamp_ns(message: Image | CameraInfo) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _clock_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _image_array(message: Image) -> np.ndarray:
    channels = {"mono8": 1, "bgr8": 3, "rgb8": 3}.get(message.encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding {message.encoding!r}")
    row_bytes = int(message.width) * channels
    if int(message.step) < row_bytes:
        raise ValueError("image step is shorter than its encoded row")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    expected = int(message.step) * int(message.height)
    if raw.size < expected:
        raise ValueError("image payload is shorter than step * height")
    rows = raw[:expected].reshape(int(message.height), int(message.step))[:, :row_bytes]
    if channels == 1:
        return rows.reshape(int(message.height), int(message.width))
    pixels = rows.reshape(int(message.height), int(message.width), channels)
    if message.encoding == "rgb8":
        return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    return pixels


@dataclass
class CaptureState:
    images: dict[int, Image] = field(default_factory=dict)
    infos: dict[int, CameraInfo] = field(default_factory=dict)
    paired: set[int] = field(default_factory=set)
    frames: list[dict[str, object]] = field(default_factory=list)
    clock_count: int = 0
    first_clock_ns: int | None = None
    latest_clock_ns: int | None = None
    last_pair_wall_s: float | None = None


class ArucoCameraCapture(Node):
    """Save synchronized camera pairs without access to commanded motion."""

    def __init__(self, output_dir: Path) -> None:
        super().__init__("corridor_aruco_camera_capture")
        self.output_dir = output_dir
        self.state = CaptureState()
        self.create_subscription(
            Image,
            CAPTURE_CONTRACT["image_topic"],
            self._on_image,
            _sensor_qos(),
        )
        self.create_subscription(
            CameraInfo,
            CAPTURE_CONTRACT["camera_info_topic"],
            self._on_info,
            _sensor_qos(),
        )
        self.create_subscription(
            Clock,
            CAPTURE_CONTRACT["clock_topic"],
            self._on_clock,
            _clock_qos(),
        )

    def _on_image(self, message: Image) -> None:
        key = stamp_ns(message)
        self.state.images[key] = message
        self._pair(key)

    def _on_info(self, message: CameraInfo) -> None:
        key = stamp_ns(message)
        self.state.infos[key] = message
        self._pair(key)

    def _on_clock(self, message: Clock) -> None:
        value = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        if self.state.first_clock_ns is None:
            self.state.first_clock_ns = value
        self.state.latest_clock_ns = value
        self.state.clock_count += 1

    def _pair(self, key: int) -> None:
        if key in self.state.paired:
            return
        image = self.state.images.get(key)
        info = self.state.infos.get(key)
        if image is None or info is None:
            return
        contract = CAPTURE_CONTRACT
        if image.header.frame_id != info.header.frame_id:
            raise ValueError("Image and CameraInfo frame IDs differ")
        if image.header.frame_id != contract["frame_id"]:
            raise ValueError("camera optical frame differs from capture contract")
        if (image.width, image.height) != (contract["width"], contract["height"]):
            raise ValueError("image dimensions differ from capture contract")
        # _image_array still converts several encodings so it stays testable,
        # but a production evidence capture is rgb8 and nothing else.
        if image.encoding != contract["encoding"]:
            raise ValueError(
                f"wire encoding {image.encoding!r} differs from capture contract "
                f"{contract['encoding']!r}"
            )
        if (info.width, info.height) != (image.width, image.height):
            raise ValueError("CameraInfo dimensions differ from Image")
        pixels = _image_array(image)
        relative_path = Path("frames") / f"frame_{key:019d}.png"
        frame_path = self.output_dir / relative_path
        frame_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(frame_path), pixels):
            raise OSError(f"failed to write {frame_path}")
        self.state.frames.append(
            {
                "stamp_ns": key,
                "image_path": relative_path.as_posix(),
                "encoding": image.encoding,
                "frame_id": image.header.frame_id,
                "width": int(image.width),
                "height": int(image.height),
                "step": int(image.step),
                "distortion_model": info.distortion_model,
                "d": [float(value) for value in info.d],
                "k": [float(value) for value in info.k],
                "r": [float(value) for value in info.r],
                "p": [float(value) for value in info.p],
            }
        )
        self.state.paired.add(key)
        self.state.last_pair_wall_s = time.monotonic()
        del self.state.images[key]
        del self.state.infos[key]

    def write_manifest(self) -> Path:
        frames = sorted(self.state.frames, key=lambda frame: int(frame["stamp_ns"]))
        output = self.output_dir / "capture.json"
        output.write_text(
            json.dumps(
                {
                    "schema_version": "1.0",
                    "kind": "production_ros_camera_capture",
                    "contract": CAPTURE_CONTRACT,
                    "clock": {
                        "samples": self.state.clock_count,
                        "first_ns": self.state.first_clock_ns,
                        "last_ns": self.state.latest_clock_ns,
                    },
                    "frames": frames,
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        return output


def main() -> int:
    args = arguments()
    output_dir = args.out_dir.resolve()
    if args.minimum_pairs < 1:
        raise ValueError("--minimum-pairs must be positive")
    if args.timeout <= 0.0 or args.idle_after_minimum <= 0.0:
        raise ValueError("timeouts must be positive")
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"capture output directory is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = ArucoCameraCapture(output_dir)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            state = node.state
            if (
                len(state.paired) >= args.minimum_pairs
                and state.last_pair_wall_s is not None
                and time.monotonic() - state.last_pair_wall_s >= args.idle_after_minimum
            ):
                break
        if len(node.state.paired) < args.minimum_pairs:
            raise AssertionError(
                f"captured {len(node.state.paired)} pairs; expected at least {args.minimum_pairs}"
            )
        capture_path = node.write_manifest()
        print(
            "ROS_ARUCO_CAPTURE_PASS",
            f"pairs={len(node.state.paired)}",
            f"path={capture_path}",
            flush=True,
        )
    except Exception as exc:
        print(
            "ROS_ARUCO_CAPTURE_FAIL",
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
