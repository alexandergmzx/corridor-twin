#!/usr/bin/env python3
"""Render the enforcement run as a watchable video, from recorded evidence.

**A capture of the artifacts, not of a screen.** A screen recording of a live
replay depends on an X server, a window manager, RViz's startup timing and
whatever else the desktop is doing; it is the least reproducible artifact in a
repository whose whole discipline is reproducible artifacts. This reads the
committed frames, the committed stations and the committed table, and produces
the same video every time from the same inputs.

What the overlay shows is what **P** knows: the detector's box, the station
that box back-projects to, the local corridor width, the limit that width
selects, and the speed the window fit reports. Truth is drawn too, in a
separate colour and labelled EVAL, because a demonstration of a measurement is
not worth much without the thing it is measured against beside it.

    .venv/bin/python tools/render_enforcement_video.py \\
        --frames out/evidence/ship-day/f3.1-violation/png \\
        --stations .../stations.json --table .../speed-table.json \\
        --schedule .../commanded-pose-schedule.json --out enforcement.mp4
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent

#: Output scale. The camera contract is 640x360; 2x is legible on a projector
#: without inventing detail, and nearest-neighbour keeps the pixels honest.
SCALE = 2

WHITE = (245, 245, 245)
DIM = (150, 150, 150)
GREEN = (120, 220, 120)
RED = (70, 70, 245)
AMBER = (80, 200, 250)
EVAL = (220, 180, 90)


def draw_panel(frame, x, y, w, h, alpha=0.55):
    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y), (x + w, y + h), (25, 25, 25), -1)
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)


def text(frame, s, x, y, colour=WHITE, scale=0.5, weight=1):
    cv2.putText(frame, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, colour,
                weight, cv2.LINE_AA)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=Path, required=True)
    ap.add_argument("--stations", type=Path, required=True)
    ap.add_argument("--table", type=Path, required=True)
    ap.add_argument("--schedule", type=Path)
    ap.add_argument("--manifest", type=Path, default=ROOT / "out/corridor.manifest.json")
    ap.add_argument("--profile", default="nominal_m6_n3")
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)

    index = json.loads((args.frames / "index.json").read_text(encoding="utf-8"))
    stations = json.loads(args.stations.read_text(encoding="utf-8"))["frames"]
    table = json.loads(args.table.read_text(encoding="utf-8"))
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    profile = manifest["profiles"][args.profile]
    length = manifest["corridor_length_m"]

    truth = None
    if args.schedule and args.schedule.is_file():
        samples = json.loads(args.schedule.read_text(encoding="utf-8"))["samples"]
        truth = (np.array([s["sim_time_s"] for s in samples]),
                 np.array([s["x_m"] for s in samples]))

    def width_at(station):
        fraction = min(max(station / length, 0.0), 1.0)
        return profile["entry_width_m"] + fraction * (
            profile["corner_width_m"] - profile["entry_width_m"])

    gates = {row["station_m"]: row for row in table["gates"]}
    by_index = {row["index"]: row for row in stations}

    first = cv2.imread(str(args.frames / index["frames"][0]["file"]))
    height, width = first.shape[:2]
    size = (width * SCALE, height * SCALE)
    writer = cv2.VideoWriter(str(args.out), cv2.VideoWriter_fourcc(*"mp4v"),
                             args.fps, size)
    if not writer.isOpened():
        raise SystemExit(f"could not open {args.out} for writing")

    # Verdicts appear as the robot reaches each gate and then persist, so the
    # viewer can see the episode accumulate rather than one flash per gate.
    reached, violation_at = [], None
    written = 0

    for order, entry in enumerate(index["frames"]):
        raw = cv2.imread(str(args.frames / entry["file"]))
        frame = cv2.resize(raw, size, interpolation=cv2.INTER_NEAREST)
        row = by_index.get(order)
        station = row["station_m"] if row else None

        if row and row.get("box"):
            box = row["box"]
            seen_colour = GREEN
            cv2.rectangle(frame,
                          (int(box["x_min"] * SCALE), int(box["y_min"] * SCALE)),
                          (int(box["x_max"] * SCALE), int(box["y_max"] * SCALE)),
                          seen_colour, 2)
            text(frame, f"{row['score']:.2f}",
                 int(box["x_min"] * SCALE), int(box["y_min"] * SCALE) - 6,
                 seen_colour, 0.45)

        if station is not None:
            for gate, info in gates.items():
                if station >= gate and gate not in reached:
                    reached.append(gate)
                    if info.get("confirmed"):
                        violation_at = gate

        draw_panel(frame, 12, 12, 430, 128)
        text(frame, "P's roadside camera  -  pixels only", 24, 36, WHITE, 0.55)
        text(frame, "no pose, no odometry, no TF, no depth", 24, 56, DIM, 0.42)

        if station is None:
            text(frame, "A not in frame", 24, 88, DIM, 0.52)
        else:
            limit = gates[min(gates, key=lambda g: abs(g - station))]["limit_mps"]
            text(frame, f"station {station:6.3f} m", 24, 84, WHITE, 0.52)
            text(frame, f"width   {width_at(station):5.2f} m", 24, 104, WHITE, 0.52)
            text(frame, f"limit   {limit:5.2f} m/s", 210, 84,
                 RED if limit == min(g["limit_mps"] for g in gates.values()) else WHITE,
                 0.52)
            if truth is not None:
                want = float(np.interp(entry["stamp_s"], truth[0], truth[1]))
                text(frame, f"EVAL truth {want:5.3f} m", 210, 104, EVAL, 0.48)

        draw_panel(frame, 12, size[1] - 132, size[0] - 24, 120)
        text(frame, "gate    width   limit   measured   verdict",
             24, size[1] - 108, DIM, 0.44)
        for slot, (gate, info) in enumerate(sorted(gates.items())):
            y = size[1] - 88 + slot * 17
            passed = gate in reached
            colour = DIM if not passed else (
                RED if info.get("over_limit") else GREEN)
            measured = (f"{info['speed_window_mps']:.4f}"
                        if passed and info.get("speed_window_mps") else "  --  ")
            verdict = ""
            if passed:
                verdict = "OVER" if info.get("over_limit") else "compliant"
                if info.get("confirmed"):
                    verdict = "OVER  ** VIOLATION CONFIRMED **"
            text(frame,
                 f"{gate:4.1f}   {info['width_m']:5.2f}   {info['limit_mps']:5.2f}"
                 f"    {measured}   {verdict}",
                 24, y, colour, 0.44)

        if violation_at is not None:
            cv2.rectangle(frame, (2, 2), (size[0] - 3, size[1] - 3), RED, 4)
            text(frame, "SPEEDING", size[0] - 190, 44, RED, 0.8, 2)

        writer.write(frame)
        written += 1

    writer.release()
    seconds = written / args.fps
    print(f"{written} frames -> {args.out}  ({seconds:.1f} s at {args.fps} fps)")
    if seconds > 120:
        print(f"  **{seconds:.0f} s exceeds the 2-minute brief**")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
