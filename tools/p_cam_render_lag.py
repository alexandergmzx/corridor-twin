#!/usr/bin/env python3
"""Does the image show the scene its timestamp claims?

**The question the speed table cannot answer about itself.** A station-based
speed estimator divides a distance by a time, and it takes the time from the
image header. If the pixels in an image show the scene as it was some seconds
before the stamp on that image, the estimator is dividing a correct distance by
a wrong interval and there is nothing in its own output to say so.

CLAUDE.md has listed "the pose-to-render latency is uncharacterised" as an open
limit since v1, bounded there at one camera period. This measures it, and on
these runs it is three orders of magnitude larger than that bound.

The method needs no new instrumentation. The adapter drives A along a known
route at a known speed and records the commanded pose against sim time. The
detector recovers A's station from pixels. Both are indexed by the same clock,
so the times at which each says A crossed a given station can simply be
subtracted.

    lag(x) = (header stamp when the PIXELS first show A at x)
           - (sim time when the SCHEDULE put A at x)

A lag near zero means the stamp describes the content. A positive lag that
grows means the content is falling behind the clock, and every speed derived
from those stamps is scaled down by the rate at which it grows.

Usage:
    .venv/bin/python tools/p_cam_render_lag.py --stations .../stations.json \\
        --schedule .../commanded-pose-schedule.json --out lag.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

#: Stations to test, in metres. The five enforcement gates plus the ends of
#: the visible stretch, so the trend is readable rather than a single number.
DEFAULT_PROBES = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)


def first_crossing(rows, target: float) -> float | None:
    """Header stamp at which the estimated station first rises through target."""

    for before, after in zip(rows, rows[1:], strict=False):
        if (before["station_m"] < target <= after["station_m"]
                and after["stamp_s"] > before["stamp_s"]):
            # Linear between the two samples, so the answer is not quantised
            # to the frame period the lag is being compared against.
            span = after["station_m"] - before["station_m"]
            fraction = (target - before["station_m"]) / span if span else 0.0
            return before["stamp_s"] + fraction * (after["stamp_s"] - before["stamp_s"])
    return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stations", type=Path, required=True)
    ap.add_argument("--schedule", type=Path, required=True)
    ap.add_argument("--probes", type=float, nargs="+", default=list(DEFAULT_PROBES))
    ap.add_argument("--label", default="")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)

    estimated = json.loads(args.stations.read_text(encoding="utf-8"))
    rows = [r for r in estimated["frames"] if r["station_m"] is not None]
    rows.sort(key=lambda r: r["stamp_s"])

    raw = json.loads(args.schedule.read_text(encoding="utf-8"))
    samples = raw["samples"]
    times = np.array([s["sim_time_s"] for s in samples])
    xs = np.array([s["x_m"] for s in samples])
    speed = raw.get("path_speed_mps")

    probes = []
    for target in args.probes:
        if not (xs >= target).any():
            continue
        commanded = float(times[int(np.argmax(xs >= target))])
        observed = first_crossing(rows, target)
        if observed is None:
            continue
        probes.append({"station_m": target,
                       "schedule_t_s": round(commanded, 3),
                       "pixels_t_s": round(observed, 3),
                       "lag_s": round(observed - commanded, 3),
                       "lag_m": round((observed - commanded) * speed, 4)
                       if speed else None})

    print(f"\n{args.label or args.stations.parent.name}"
          f"   commanded {speed} m/s\n")
    print(f"{'station':>8} {'schedule':>9} {'pixels':>9} {'lag s':>8} {'lag m':>8}")
    for probe in probes:
        print(f"{probe['station_m']:>8.1f} {probe['schedule_t_s']:>9.2f} "
              f"{probe['pixels_t_s']:>9.2f} {probe['lag_s']:>+8.2f} "
              f"{(probe['lag_m'] if probe['lag_m'] is not None else 0):>+8.3f}")

    growth = None
    if len(probes) > 2:
        growth = float(np.polyfit([p["schedule_t_s"] for p in probes],
                                  [p["lag_s"] for p in probes], 1)[0])
        print(f"\nlag grows {growth:+.3f} s per second of sim time")
        print(f"  -> content advances at {1.0 - growth:.3f} x the clock, so any "
              f"speed read off these stamps is low by ~{100 * growth:.0f}%")

    doc = {"label": args.label, "stations": str(args.stations),
           "schedule": str(args.schedule), "commanded_mps": speed,
           "probes": probes, "lag_growth_s_per_s": round(growth, 5)
           if growth is not None else None,
           "content_rate_vs_clock": round(1.0 - growth, 5)
           if growth is not None else None}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
