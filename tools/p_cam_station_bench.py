#!/usr/bin/env python3
"""How much of the station error is the projection, and how much is the detector?

The speed table measures the whole pipeline at once. When it reports a bias,
that number alone cannot say whether the detector is drawing the box in the
wrong place or the geometry is turning a correct box into the wrong metre.

This separates them. It runs the *same* `station_from_box` used in production
over the dataset's **labelled truth boxes**, so the detector is removed from
the loop entirely and whatever error remains belongs to the projection and its
conventions. Subtract this from the full pipeline's error and the remainder is
the detector's contribution.

No GPU, no model load, no Isaac: pure numpy over a JSON index, so it runs while
the simulator is busy and it can be re-run for free.

Usage:
    .venv/bin/python tools/p_cam_station_bench.py \\
        --dataset out/datasets/p_cam_v1 --resolution lo \\
        --camera-pose out/p_cam_pose.json --intrinsics out/.../index.json \\
        --out out/evidence/estimator/station-bench.json
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from p_cam_infer import camera_pose, station_from_box  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", type=Path, required=True)
    ap.add_argument("--resolution", default="lo", choices=("lo", "hi"))
    ap.add_argument("--split", default="val")
    ap.add_argument("--camera-pose", type=Path, default=ROOT / "out/p_cam_pose.json")
    ap.add_argument("--intrinsics", type=Path, required=True,
                    help="a frames index.json carrying the real CameraInfo K")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)

    index = json.loads((args.dataset / "dataset.json").read_text(encoding="utf-8"))
    frames = [f for f in index["frames"] if f.get("split") == args.split]
    if not frames:
        raise SystemExit(f"no {args.split} frames in {args.dataset}")

    position, rotation = camera_pose(args.camera_pose)
    pose_doc = json.loads(args.camera_pose.read_text(encoding="utf-8"))

    # The measured intrinsics the camera actually published, not a focal length
    # re-derived from the manifest's FOV -- the two agree only if nothing in
    # the render path adjusted the aperture.
    K = np.array(json.loads(args.intrinsics.read_text(encoding="utf-8"))
                 ["intrinsics"]["k"], dtype=np.float64).reshape(3, 3)

    # The dataset renders every frame from one eye; if that is not where the
    # pose says the camera is, every station below is measured from the wrong
    # place and none of them will look wrong.
    eyes = {tuple(round(v, 3) for v in f["camera_eye_xyz_m"]) for f in frames}
    if len(eyes) == 1:
        eye = np.array(next(iter(eyes)))
        if not np.allclose(eye, position, atol=0.01):
            raise SystemExit(
                f"the dataset rendered from {eye.tolist()} and the pose says "
                f"{position.round(3).tolist()}; re-export from the stage that "
                f"was rendered ({pose_doc.get('stage')} is the current one)")

    rows = []
    for frame in frames:
        boxes = (frame["resolutions"][args.resolution] or {}).get("boxes") or []
        if not boxes:
            continue
        estimated = station_from_box(boxes[0], K, position, rotation)
        if estimated is None:
            continue
        rows.append({
            "index": frame["index"], "profile": frame["profile"],
            "truth_x_m": frame["pose"]["x_m"],
            "estimated_x_m": round(estimated, 4),
            "error_m": round(estimated - frame["pose"]["x_m"], 4),
            "range_m": frame.get("range_m"),
        })

    if not rows:
        raise SystemExit("no labelled boxes produced a station")

    errors = [r["error_m"] for r in rows]
    ranges = [r["range_m"] for r in rows if r["range_m"] is not None]
    bias, sd = statistics.fmean(errors), statistics.stdev(errors)

    # Is the error a constant offset or does it scale? A constant cancels in a
    # speed; a scale does not, and the speed table's systematic underestimate
    # can only come from the second kind.
    slope = intercept = correlation = None
    if len(ranges) == len(rows) and len(rows) > 2:
        slope, intercept = np.polyfit(ranges, errors, 1)
        correlation = float(np.corrcoef(ranges, errors)[0, 1])

    print(f"{len(rows)} labelled {args.split} frames, {args.resolution} resolution")
    print("  convention         bottom-centre pixel -> ground plane z=0")
    print(f"  camera             {position.round(3).tolist()} "
          f"from {Path(str(pose_doc.get('stage'))).name}")
    print(f"  station bias       {bias:+.4f} m")
    print(f"  station sd         {sd:.4f} m")
    print(f"  |error| worst      {max(abs(e) for e in errors):.4f} m")
    if slope is not None:
        print(f"  error vs range     {slope:+.4f} m per m  "
              f"(intercept {intercept:+.4f} m, r = {correlation:+.3f})")
        print(f"  -> a range-proportional term of {slope:+.3f} is a SCALE error, "
              f"which does not cancel in a speed")

    doc = {"dataset": str(args.dataset), "split": args.split,
           "resolution": args.resolution, "n": len(rows),
           "camera_pose_stage": pose_doc.get("stage"),
           "camera_position_xyz_m": position.tolist(),
           "convention": "bottom-centre pixel, ground plane z=0",
           "station_bias_m": round(bias, 5), "station_sd_m": round(sd, 5),
           "worst_abs_error_m": round(max(abs(e) for e in errors), 5),
           "error_vs_range_slope": round(float(slope), 5) if slope is not None else None,
           "error_vs_range_intercept": round(float(intercept), 5)
           if intercept is not None else None,
           "error_vs_range_r": round(correlation, 4) if correlation is not None else None,
           "frames": rows}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
