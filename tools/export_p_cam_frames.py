#!/usr/bin/env python3
"""Extract P's camera frames from a bag into PNGs the detector can read.

    source /opt/ros/jazzy/setup.bash
    PYTHONNOUSERSITE=1 python3 tools/export_p_cam_frames.py \\
        --bag out/evidence/.../rosbag --out out/evidence/.../frames

**Why this exists as a separate step.** `rosbag2_py` is a system-Jazzy
extension built against Python 3.12 and `torch` lives in Isaac's 3.11; Isaac's
interpreter cannot import the former (`rosbag2_py._compression_options` is
missing there, measured). Same rule as everywhere else in this repo: the two
ABIs meet over an artifact, never inside one process.

The index carries the **header stamp**, which is sim time. Wall time may
measure external latency but must never enter speed differentiation (ADR 0003),
and the bag's receive timestamp is wall time.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--image-topic", default="/p_cam/image_raw")
    ap.add_argument("--info-topic", default="/p_cam/camera_info")
    ap.add_argument("--stride", type=int, default=1,
                    help="keep every Nth frame; 1 keeps all of them")
    args = ap.parse_args(argv)

    import cv2
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, Image

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=[args.image_topic, args.info_topic]))

    args.out.mkdir(parents=True, exist_ok=True)
    frames, intrinsics, kept, seen = [], None, 0, 0
    while reader.has_next():
        topic, data, _receive_stamp = reader.read_next()
        if topic == args.info_topic:
            if intrinsics is None:
                info = deserialize_message(data, CameraInfo)
                intrinsics = {"k": list(info.k), "width": info.width,
                              "height": info.height,
                              "distortion_model": info.distortion_model,
                              "d": list(info.d)}
            continue
        message = deserialize_message(data, Image)
        seen += 1
        if (seen - 1) % args.stride:
            continue
        array = np.frombuffer(message.data, np.uint8).reshape(
            message.height, message.width, -1)[:, :, :3]
        if message.encoding == "rgb8":
            array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        name = f"frame_{kept:05d}.png"
        cv2.imwrite(str(args.out / name), array)
        frames.append({
            "file": name,
            # Sim time, from the header. ADR 0003.
            "stamp_s": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            "frame_id": message.header.frame_id,
        })
        kept += 1

    if intrinsics is None:
        raise SystemExit(f"no {args.info_topic} in {args.bag}; intrinsics are "
                         f"not optional and must not be assumed")

    (args.out / "index.json").write_text(json.dumps({
        "bag": str(args.bag), "image_topic": args.image_topic,
        "stride": args.stride, "images_in_bag": seen,
        "intrinsics": intrinsics, "frames": frames}, indent=2) + "\n",
        encoding="utf-8")
    span = (frames[-1]["stamp_s"] - frames[0]["stamp_s"]) if len(frames) > 1 else 0.0
    print(f"{kept} of {seen} frames -> {args.out}  "
          f"({intrinsics['width']}x{intrinsics['height']}, {span:.2f} s of sim time)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
