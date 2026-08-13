#!/usr/bin/env python3
"""Fine-tune RT-DETR to find robot A in P's frames.

    ~/isaac/env_isaaclab/bin/python tools/train_p_cam_detector.py \
        --dataset out/datasets/p_cam_v1 --resolution lo --epochs 12 \
        --out out/evidence/detector/run-lo

Runs under Isaac's Python because that is the only interpreter on this host
with torch, and it holds **no Isaac lock**: ADR 0024 requires the dataset to be
rendered once and training never to occupy the GPU session. It opens no
simulator and reads only files.

THE MODEL, AND THE LICENCE CHECK
--------------------------------
`PekingU/rtdetr_r18vd`, through `transformers.RTDetrForObjectDetection`.

* RT-DETR is **Apache-2.0** (the original Baidu PaddleDetection release and the
  HuggingFace port both), which satisfies ADR 0024 decision 2's
  permissive-only clause.
* `transformers` 4.57.6 is already installed in `~/isaac/env_isaaclab`, so
  nothing is pip-installed into Isaac's ABI to run this.
* **Ultralytics-YOLO is rejected on AGPL grounds** by ADR 0024, on licence
  alone and not on capability.

ADR 0024 decision 2 says the family pin lands as its own short record with the
spike evidence, never as an edit to that ADR. This script is the spike; the
record follows the evidence rather than preceding it.

WHAT IT MEASURES
----------------
Validation loss per epoch, and **detection rate at IoU >= 0.5** computed here
rather than through `pycocotools`, which is not installed and is not worth
installing into this ABI at this hour. That is a coarser metric than mAP and it
is labelled as one: it answers "does it find A", which is the question the
speed pipeline actually asks, and not "how well ranked are its confidences".

Per-profile validation is reported separately as well as pooled, because the
three corridor geometries are the only distribution shift this dataset has.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

ROOT = Path(os.path.abspath(__file__)).parent.parent

#: ADR 0024 decision 2: permissive licences only. Recorded in the run report so
#: the check is evidence rather than a claim in a docstring.
MODEL_ID = "PekingU/rtdetr_r18vd"
MODEL_LICENCE = "Apache-2.0"

#: IoU at which a prediction counts as having found A.
IOU_MATCH = 0.5


def iou(a: dict, b: dict) -> float:
    left = max(a["x_min"], b["x_min"])
    top = max(a["y_min"], b["y_min"])
    right = min(a["x_max"], b["x_max"])
    bottom = min(a["y_max"], b["y_max"])
    if right <= left or bottom <= top:
        return 0.0
    overlap = (right - left) * (bottom - top)
    area_a = (a["x_max"] - a["x_min"]) * (a["y_max"] - a["y_min"])
    area_b = (b["x_max"] - b["x_min"]) * (b["y_max"] - b["y_min"])
    union = area_a + area_b - overlap
    return overlap / union if union > 0 else 0.0


def load_split(dataset: Path, resolution: str, split: str) -> list[dict]:
    """Frames for one split, from the generator's own manifest.

    Read from `dataset.json` rather than by globbing the directory: the
    manifest is checkpointed per frame, so a partial dataset is described
    exactly, and a file on disk that the manifest does not list is a file the
    generator did not finish writing.
    """

    index = json.loads((dataset / "dataset.json").read_text(encoding="utf-8"))
    frames = []
    for record in index["frames"]:
        if record["split"] != split:
            continue
        entry = record["resolutions"].get(resolution)
        if not entry or not entry["boxes"]:
            continue
        frames.append({
            "image": dataset / entry["image"],
            "boxes": entry["boxes"],
            "profile": record["profile"],
            "range_m": record.get("range_m"),
        })
    return frames


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dataset", default="out/datasets/p_cam_v1")
    parser.add_argument("--resolution", default="lo", choices=("lo", "hi"))
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--limit", type=int, default=None,
                        help="cap the train set; for a smoke run")
    parser.add_argument("--out", default="out/evidence/detector/run")
    args = parser.parse_args()

    import torch
    from PIL import Image
    from transformers import RTDetrForObjectDetection, RTDetrImageProcessor

    dataset = Path(args.dataset)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    train = load_split(dataset, args.resolution, "train")
    val = load_split(dataset, args.resolution, "val")
    if args.limit:
        train = train[: args.limit]
    if not train or not val:
        print(f"FAIL: train={len(train)} val={len(val)} -- has the dataset rendered?")
        return 1

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"{MODEL_ID} ({MODEL_LICENCE}) on {device}; "
          f"train {len(train)}, val {len(val)}, resolution {args.resolution}")

    processor = RTDetrImageProcessor.from_pretrained(MODEL_ID)
    model = RTDetrForObjectDetection.from_pretrained(
        MODEL_ID,
        num_labels=1,
        ignore_mismatched_sizes=True,
    ).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr)

    def batches(frames, size):
        for start in range(0, len(frames), size):
            yield frames[start : start + size]

    def encode(chunk):
        images = [Image.open(frame["image"]).convert("RGB") for frame in chunk]
        annotations = []
        for position, frame in enumerate(chunk):
            objects = [
                {
                    "bbox": [
                        box["x_min"], box["y_min"],
                        box["x_max"] - box["x_min"], box["y_max"] - box["y_min"],
                    ],
                    "category_id": 0,
                    "area": (box["x_max"] - box["x_min"]) * (box["y_max"] - box["y_min"]),
                    "iscrowd": 0,
                }
                for box in frame["boxes"]
            ]
            annotations.append({"image_id": position, "annotations": objects})
        return processor(
            images=images, annotations=annotations, return_tensors="pt"
        ).to(device)

    history = []
    started = time.time()
    for epoch in range(args.epochs):
        model.train()
        total, seen = 0.0, 0
        for chunk in batches(train, args.batch_size):
            encoded = encode(chunk)
            loss = model(**encoded).loss
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 0.1)
            optimizer.step()
            optimizer.zero_grad()
            total += float(loss.item()) * len(chunk)
            seen += len(chunk)
        train_loss = total / max(seen, 1)

        model.eval()
        matched, truth_total = 0, 0
        per_profile: dict[str, list[int]] = {}
        val_total, val_seen = 0.0, 0
        with torch.no_grad():
            for chunk in batches(val, args.batch_size):
                encoded = encode(chunk)
                output = model(**encoded)
                val_total += float(output.loss.item()) * len(chunk)
                val_seen += len(chunk)
                sizes = torch.tensor(
                    [Image.open(f["image"]).size[::-1] for f in chunk]
                ).to(device)
                results = processor.post_process_object_detection(
                    output, target_sizes=sizes, threshold=0.3
                )
                for frame, result in zip(chunk, results, strict=True):
                    predicted = [
                        {"x_min": float(b[0]), "y_min": float(b[1]),
                         "x_max": float(b[2]), "y_max": float(b[3])}
                        for b in result["boxes"].cpu()
                    ]
                    bucket = per_profile.setdefault(frame["profile"], [0, 0])
                    for box in frame["boxes"]:
                        truth_total += 1
                        bucket[1] += 1
                        if any(iou(box, p) >= IOU_MATCH for p in predicted):
                            matched += 1
                            bucket[0] += 1

        row = {
            "epoch": epoch,
            "train_loss": round(train_loss, 5),
            "val_loss": round(val_total / max(val_seen, 1), 5),
            f"val_detection_rate_iou{IOU_MATCH}": round(matched / max(truth_total, 1), 4),
            "per_profile_detection_rate": {
                name: round(hit / total, 4) if total else None
                for name, (hit, total) in sorted(per_profile.items())
            },
            "elapsed_s": round(time.time() - started, 1),
        }
        history.append(row)
        print(json.dumps(row))

        # CHECKPOINT EVERY EPOCH: a fine-tune still running at the wall clock is
        # recoverable, and the curve is readable while it runs.
        (out / "history.json").write_text(
            json.dumps({
                "model": MODEL_ID,
                "licence": MODEL_LICENCE,
                "licence_check": "ADR 0024 decision 2: permissive only; "
                                 "Ultralytics-YOLO rejected on AGPL grounds",
                "metric_note": (
                    f"detection rate at IoU >= {IOU_MATCH}, computed in this "
                    "script; pycocotools is absent and mAP is NOT reported"
                ),
                "dataset": str(dataset),
                "resolution": args.resolution,
                "train_frames": len(train),
                "val_frames": len(val),
                "epochs_requested": args.epochs,
                "history": history,
            }, indent=2) + "\n",
            encoding="utf-8",
        )
        model.save_pretrained(out / "checkpoint")

    print(f"written: {out / 'history.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
