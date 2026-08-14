#!/usr/bin/env python3
"""Run the trained detector over P's camera frames and recover A's station.

ADR 0024's pipeline, end to end and offline: `/p_cam/image_raw` in, a world-X
station per frame out, using **nothing but pixels, the camera's own
calibration, and the surveyed ground plane**. No pose, no odometry, no TF, no
depth, no simulator truth reaches this file's estimate path -- truth appears
only in the comparison a caller does afterwards.

    box  ->  bottom-centre pixel  ->  ray through K  ->  ground plane  ->  X

**Why the bottom-centre pixel.** It is the only point of a 2-D box that
corresponds to a knowable 3-D point: where the robot touches the floor. A box
centre sits at an unknown height above the ground and its back-projection
depends on the robot's size; the bottom edge does not. The horizontal centre is
the body's mid-line under the mast's near-symmetric view.

Two runnable stages, because they fail differently and the first needs no GPU:

    --boxes truth      geometry only, on the dataset's labelled boxes. Isolates
                       the projection's own error from the detector's.
    --boxes detector   the full pipeline. The difference between the two IS the
                       detector's contribution to station error.

Usage:
    python tools/p_cam_infer.py --frames DIR --out stations.json
    python tools/p_cam_infer.py --dataset DIR --boxes truth --out bench.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

#: Offline, and it must stay offline: a hub lookup on a machine with no network
#: turns a two-minute bench into a stalled one.
os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CHECKPOINT = ROOT / "out/evidence/detector/rtdetr-lo/checkpoint"

#: The training run's own post-processing threshold. Not retuned here: a
#: threshold chosen against the frames being measured is a fitted parameter,
#: and this measurement has none.
SCORE_THRESHOLD = 0.3

#: The ground plane A drives on. z = 0 by construction -- the arena spawns the
#: twin there and the dataset rendered every frame with it there.
GROUND_Z_M = 0.0


def camera_pose(pose_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """-> (camera position in world metres, world_from_optical rotation).

    Read from the artifact `tools/export_camera_pose.py` writes out of the
    stage, because `pxr` and `torch` cannot share an interpreter here. The
    manifest's `camera` block is the sensor CONTRACT -- resolution, rate, FOV,
    mount height -- and carries no world pose, so it cannot answer this.

    Survey, not simulator truth: where P bolted its own instrument.
    """

    pose = json.loads(pose_path.read_text(encoding="utf-8"))
    position = np.array(pose["position_xyz_m"], dtype=np.float64)
    rotation = np.array(pose["world_from_optical"], dtype=np.float64).reshape(3, 3)
    return position, rotation


def ground_intersection(pixel, K, position, rotation, ground_z=GROUND_Z_M):
    """Back-project a pixel to where its ray meets the ground plane. -> (x, y).

    Returns None when the ray points at or above the horizon, which is not an
    error: a box whose bottom edge lands above the horizon is a detection of
    something that is not standing on this floor, and inventing a station for
    it would put a fabricated point into the fit.
    """

    u, v = pixel
    ray_camera = np.linalg.inv(K) @ np.array([u, v, 1.0])
    ray_world = rotation @ ray_camera
    if abs(ray_world[2]) < 1e-9 or (ground_z - position[2]) / ray_world[2] <= 0:
        return None
    t = (ground_z - position[2]) / ray_world[2]
    hit = position + t * ray_world
    return float(hit[0]), float(hit[1])


def station_from_box(box, K, position, rotation):
    """Station (world X) of the robot whose 2-D box this is. -> float | None."""

    bottom_centre = (0.5 * (box["x_min"] + box["x_max"]), box["y_max"])
    hit = ground_intersection(bottom_centre, K, position, rotation)
    return None if hit is None else hit[0]


def load_detector(checkpoint: Path, device: str):
    import torch
    from transformers import RTDetrForObjectDetection, RTDetrImageProcessor

    # The processor comes from the base model id: fine-tuning saved weights,
    # not preprocessing, and a mismatched processor silently rescales inputs.
    base = json.loads((checkpoint / "config.json").read_text(encoding="utf-8"))
    processor = RTDetrImageProcessor.from_pretrained(
        base.get("_name_or_path", "PekingU/rtdetr_r18vd"))
    model = RTDetrForObjectDetection.from_pretrained(checkpoint).to(device).eval()
    return processor, model, torch


def detect(images, processor, model, torch, device):
    """-> [best box or None] per image, highest score above the threshold."""

    out = []
    with torch.no_grad():
        for image in images:
            encoded = processor(images=image, return_tensors="pt").to(device)
            result = processor.post_process_object_detection(
                model(**encoded),
                target_sizes=torch.tensor([image.shape[:2]]).to(device),
                threshold=SCORE_THRESHOLD)[0]
            if not len(result["scores"]):
                out.append(None)
                continue
            best = int(result["scores"].argmax())
            b = result["boxes"][best].cpu().tolist()
            out.append({"x_min": b[0], "y_min": b[1], "x_max": b[2], "y_max": b[3],
                        "score": float(result["scores"][best])})
    return out


def read_frames(directory: Path):
    """-> ([HxWx3 uint8 RGB], [sim-time stamp_s], K).

    Written by `tools/export_p_cam_frames.py`, not read from the bag directly:
    `rosbag2_py` is a system-3.12 extension and this file needs Isaac's 3.11
    for `torch`. The stamps are HEADER stamps, so they are sim time (ADR 0003).
    """

    import cv2

    index = json.loads((directory / "index.json").read_text(encoding="utf-8"))
    images, stamps = [], []
    for frame in index["frames"]:
        bgr = cv2.imread(str(directory / frame["file"]), cv2.IMREAD_COLOR)
        if bgr is None:
            raise SystemExit(f"unreadable frame {frame['file']}")
        images.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        stamps.append(float(frame["stamp_s"]))
    K = np.array(index["intrinsics"]["k"], dtype=np.float64).reshape(3, 3)
    return images, stamps, K


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=Path,
                    help="directory from tools/export_p_cam_frames.py")
    ap.add_argument("--dataset", type=Path,
                    help="dataset directory with a labelled val split")
    ap.add_argument("--boxes", choices=("truth", "detector"), default="detector")
    ap.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    ap.add_argument("--camera-pose", type=Path,
                    default=ROOT / "out/p_cam_pose.json",
                    help="written by tools/export_camera_pose.py")
    ap.add_argument("--device", default=None)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)

    if (args.frames is None) == (args.dataset is None):
        ap.error("pass exactly one of --frames or --dataset")
    if args.dataset is not None and args.boxes == "detector":
        pass  # legal: run the detector over the labelled split as a control

    position, rotation = camera_pose(args.camera_pose)
    pose_doc = json.loads(args.camera_pose.read_text(encoding="utf-8"))
    print(f"camera at {position.round(4).tolist()} "
          f"(surveyed, from {pose_doc.get('stage')})")

    if args.frames is not None:
        frames, stamps, K = read_frames(args.frames)
        truth_boxes = [None] * len(frames)
        print(f"{len(frames)} frames from {args.frames.name}")
    else:
        frames, stamps, truth_boxes, K = load_dataset(args.dataset)
        print(f"{len(frames)} labelled frames from {args.dataset.name}")
    if K is None:
        raise SystemExit("no intrinsics with these frames; K is not optional")

    if args.boxes == "detector":
        device = args.device or ("cuda" if _cuda() else "cpu")
        processor, model, torch = load_detector(args.checkpoint, device)
        print(f"detector on {device} from {args.checkpoint}")
        boxes = detect(frames, processor, model, torch, device)
    else:
        boxes = truth_boxes

    rows = []
    for index, (box, stamp) in enumerate(zip(boxes, stamps, strict=True)):
        station = None if box is None else station_from_box(box, K, position, rotation)
        rows.append({"index": index, "stamp_s": round(stamp, 4),
                     "detected": box is not None,
                     "score": None if box is None else round(box.get("score", 1.0), 4),
                     # The box travels with the station so an overlay can show
                     # WHAT was measured beside the number, which is the only
                     # way a viewer can tell a good station from a lucky one.
                     "box": None if box is None else
                            {k: round(v, 2) for k, v in box.items()},
                     "station_m": None if station is None else round(station, 4)})

    found = [r for r in rows if r["station_m"] is not None]
    print(f"{len(found)}/{len(rows)} frames produced a station "
          f"({100.0 * len(found) / max(len(rows), 1):.1f}%)")
    if found:
        xs = [r["station_m"] for r in found]
        print(f"  station range {min(xs):.3f} .. {max(xs):.3f} m")

    doc = {"source": str(args.frames or args.dataset), "boxes": args.boxes,
           # THE STAGE THE POSE CAME FROM, carried downstream so a caller can
           # refuse a mismatch. Exporting the mast from the v1 stage and
           # applying it to arena frames put the camera 2.1 m away from where
           # it actually was, and the pipeline reported plausible stations for
           # all 208 frames with no error anywhere.
           "camera_pose_stage": pose_doc.get("stage"),
           "camera_pose_file": str(args.camera_pose),
           "checkpoint": str(args.checkpoint) if args.boxes == "detector" else None,
           "score_threshold": SCORE_THRESHOLD if args.boxes == "detector" else None,
           "camera_position_xyz_m": position.tolist(),
           "intrinsics_k": K.flatten().tolist(),
           "frames": rows}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.out}")
    return 0


def _cuda() -> bool:
    try:
        import torch
        return torch.cuda.is_available()
    except Exception:
        return False


def load_dataset(directory: Path):
    """-> (images, stamps, truth boxes, K) for the labelled val split."""

    from PIL import Image as PILImage

    index = json.loads((directory / "dataset.json").read_text(encoding="utf-8"))
    split = [f for f in index["frames"] if f.get("split") == "val"]
    images, stamps, boxes = [], [], []
    for order, frame in enumerate(split):
        images.append(np.asarray(PILImage.open(directory / frame["image"]).convert("RGB")))
        stamps.append(float(frame.get("stamp_s", order)))
        found = frame.get("boxes") or []
        boxes.append(found[0] if found else None)
    K = np.array(index["intrinsics_k"], dtype=np.float64).reshape(3, 3) \
        if "intrinsics_k" in index else None
    return images, stamps, boxes, K


if __name__ == "__main__":
    sys.exit(main())
