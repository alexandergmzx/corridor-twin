#!/usr/bin/env python3
"""What speed does A actually reach at each enforcement gate?

ADR 0023 re-pins the speed policy to robot scale and says the numbers come
from *a measured profile run*, not from a choice. This reads that profile out
of the session bags the corridor runs already record.

**Ground truth, and only for this.** `/sim/ground_truth` is an evaluation
input (invariant 1): it may derive the policy the observer is later judged
against, and it may never reach the observer. Nothing here runs at demo time.

Station is world x. The gate markers sit on the taper's straight approach --
marker 0 at station 0.6 has its corners at x = 0.508-0.607 -- so for the five
gates the corridor's station coordinate and the world x-axis are the same
number, and the arc does not begin until x = 3.48.

Speed is reported two ways because they answer different questions:

* `twist` is what the simulator says the body is doing at that instant. It is
  the honest instantaneous speed and it is what a perfect observer would see.
* `secant` differences position over +/-`--window` metres of travel. It is
  what any *station-based* estimator can recover -- including the one this
  policy will be enforced by -- so a limit pinned against `twist` alone could
  be unenforceable by construction.

Usage:
    python tools/measure_speed_profile.py BAG [BAG ...] --out summary.json
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

#: The five enforcement gates of the nominal profile, in metres of station.
#: Read from the manifest rather than typed, so a profile change moves them.
DEFAULT_MANIFEST = Path(__file__).resolve().parent.parent / "out/corridor.manifest.json"

#: Half-width of the secant window, in metres of travel.
DEFAULT_WINDOW_M = 0.30


def gate_stations(manifest_path: Path, profile: str) -> list[float]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    markers = manifest["profiles"][profile]["markers"]
    return sorted({m["station_m"] for m in markers if m.get("role") == "gate"})


def width_at(manifest_path: Path, profile: str, station: float) -> float:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    prof = manifest["profiles"][profile]
    length = manifest["corridor_length_m"]
    fraction = min(max(station / length, 0.0), 1.0)
    return prof["entry_width_m"] + fraction * (
        prof["corner_width_m"] - prof["entry_width_m"])


def read_truth(bag: Path) -> list[tuple[float, float, float, float]]:
    """-> [(t_s, x, y, twist_vx)], in bag order."""

    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/sim/ground_truth"]))

    out = []
    while reader.has_next():
        _topic, data, _stamp = reader.read_next()
        msg = deserialize_message(data, Odometry)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        out.append((t, p.x, p.y, msg.twist.twist.linear.x))
    return out


def travelled(samples) -> list[float]:
    """Cumulative path length, so the secant window is metres of travel."""

    s, acc = [0.0], 0.0
    for (_, x0, y0, _), (_, x1, y1, _) in zip(samples, samples[1:], strict=False):
        acc += math.hypot(x1 - x0, y1 - y0)
        s.append(acc)
    return s


def at_gate(samples, dist, station, window):
    """Speeds at the FIRST crossing of `station` on the outbound leg.

    First crossing, not nearest sample: the delivery route passes some gates
    once and the departure leg can revisit them, and a run that reverses would
    otherwise report the return trip's speed.
    """

    cross = None
    for i in range(1, len(samples)):
        if samples[i - 1][1] < station <= samples[i][1]:
            cross = i
            break
    if cross is None:
        return None

    t_c, x_c, _y, twist_c = samples[cross]
    s_c = dist[cross]

    lo = hi = cross
    while lo > 0 and s_c - dist[lo] < window:
        lo -= 1
    while hi < len(dist) - 1 and dist[hi] - s_c < window:
        hi += 1
    span_s = samples[hi][0] - samples[lo][0]
    secant = (dist[hi] - dist[lo]) / span_s if span_s > 0 else None

    return {"t_s": round(t_c, 3), "x_m": round(x_c, 4),
            "twist_mps": round(abs(twist_c), 4),
            "secant_mps": round(secant, 4) if secant else None,
            "window_travel_m": round(dist[hi] - dist[lo], 4),
            "window_span_s": round(span_s, 3)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+", type=Path)
    ap.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    ap.add_argument("--profile", default="nominal_m6_n3")
    ap.add_argument("--window", type=float, default=DEFAULT_WINDOW_M)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args(argv)

    stations = gate_stations(args.manifest, args.profile)
    runs, rows = [], {s: [] for s in stations}

    for bag in args.bags:
        try:
            samples = read_truth(bag)
        except Exception as exc:                       # pragma: no cover
            print(f"{bag.name}: unreadable ({exc})", file=sys.stderr)
            continue
        if len(samples) < 10:
            print(f"{bag.name}: only {len(samples)} truth samples; skipped",
                  file=sys.stderr)
            continue
        dist = travelled(samples)
        gates = {}
        for station in stations:
            got = at_gate(samples, dist, station, args.window)
            if got:
                gates[station] = got
                rows[station].append(got)
        runs.append({"bag": bag.name, "truth_samples": len(samples),
                     "travelled_m": round(dist[-1], 3),
                     "gates": {str(k): v for k, v in gates.items()}})

    summary = []
    for station in stations:
        got = rows[station]
        if not got:
            summary.append({"station_m": station, "n": 0})
            continue
        twist = [g["twist_mps"] for g in got]
        secant = [g["secant_mps"] for g in got if g["secant_mps"] is not None]
        summary.append({
            "station_m": station,
            "width_m": round(width_at(args.manifest, args.profile, station), 4),
            "n": len(got),
            "twist_mean": round(statistics.fmean(twist), 4),
            "twist_min": round(min(twist), 4),
            "twist_max": round(max(twist), 4),
            "twist_sd": round(statistics.stdev(twist), 4) if len(twist) > 1 else None,
            "secant_mean": round(statistics.fmean(secant), 4) if secant else None,
            "secant_min": round(min(secant), 4) if secant else None,
            "secant_max": round(max(secant), 4) if secant else None,
        })

    print(f"{'station':>8} {'width':>7} {'n':>3} "
          f"{'twist mean':>11} {'min':>7} {'max':>7} {'sd':>7} "
          f"{'secant mean':>12} {'min':>7} {'max':>7}")
    for row in summary:
        if not row["n"]:
            print(f"{row['station_m']:>8.2f} {'-':>7} {0:>3}   (never crossed)")
            continue
        sd = f"{row['twist_sd']:.4f}" if row["twist_sd"] is not None else "-"
        print(f"{row['station_m']:>8.2f} {row['width_m']:>7.3f} {row['n']:>3} "
              f"{row['twist_mean']:>11.4f} {row['twist_min']:>7.4f} "
              f"{row['twist_max']:>7.4f} {sd:>7} "
              f"{row['secant_mean']:>12.4f} {row['secant_min']:>7.4f} "
              f"{row['secant_max']:>7.4f}")

    doc = {"profile": args.profile, "window_m": args.window,
           "truth_topic": "/sim/ground_truth", "summary": summary, "runs": runs}
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
        print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
