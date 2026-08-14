#!/usr/bin/env python3
"""Speed at each enforcement gate, from P's camera, against truth.

**This is correction 3's number.** The 2026-08-04 interview asked for active
AI/ML use; ADR 0024 answered with a learned detector on P's own camera. What
was missing was the measurement: how well does the thing actually estimate
speed? This produces that table, and it ships whatever it says.

    stations.json (pixels only)  +  commanded schedule (truth)  ->  per-gate row

**Zero fitted parameters.** The window half-width is a stated constant, the
detector's score threshold is the training run's own, and nothing here is
tuned against the answer. There is no target: a row is reported, not passed.

**Two estimators, side by side, because they differ and the difference matters.**

  `window`  Least-squares fit of station against time over +/-WINDOW_M of
            station around the gate. Uses every frame in the window, so its
            error falls with coverage and it degrades gracefully when frames
            are dropped.
  `secant`  Station difference between the two nearest gate crossings over
            their time difference -- what `GateSpeedEstimator` does. Two
            samples, so one bad station is half the estimate.

Truth is the **commanded pose schedule** the adapter wrote (`--drive-out`),
which is an evaluation input and never reaches the estimate path. For a
scripted pass it is exact by construction: the adapter set that pose. What it
does NOT capture is pose-to-render latency, which is unmeasured and is stated
as a limit rather than corrected for.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent

#: Half-width of the fitting window, in metres of station. Chosen before any
#: result was seen, from the geometry: +/-0.30 m is a fifth of the gate spacing,
#: so windows never overlap, and at A's ~0.2 m/s it spans ~3 s, which is ~45
#: frames at the 15 Hz contract -- enough to fit a line through.
WINDOW_M = 0.30

#: Fewer frames than this in a window and no speed is reported for that gate.
#: Two points define a line exactly and tell you nothing about its error.
MIN_FRAMES = 4


def gate_stations(manifest: Path, profile: str) -> list[float]:
    doc = json.loads(manifest.read_text(encoding="utf-8"))
    markers = doc["profiles"][profile]["markers"]
    return sorted({m["station_m"] for m in markers if m.get("role") == "gate"})


def path_axis_fraction(manifest: Path, profile: str) -> float:
    """The X component of the approach heading, as `MarkerMap` uses it.

    Authored geometry from the manifest, not a fitted constant: the shipped
    observer divides an axis speed by this to get the path speed the policy is
    written about. Quoted here so the table reports what the observer would.
    """

    doc = json.loads(manifest.read_text(encoding="utf-8"))
    heading = doc["profiles"][profile]["delivery_trajectory"]["approach_heading"]
    return float(heading[0])


def policy(manifest: Path, profile: str):
    doc = json.loads(manifest.read_text(encoding="utf-8"))
    prof, length = doc["profiles"][profile], doc["corridor_length_m"]
    rules = sorted((r["maximum_width_m"], r["limit_mps"])
                   for r in doc["speed_policy"]["rules"])
    sigma = float(doc["speed_policy"]["confidence_sigma"])
    consecutive = int(doc["speed_policy"]["consecutive_estimates"])

    def width_at(station: float) -> float:
        fraction = min(max(station / length, 0.0), 1.0)
        return prof["entry_width_m"] + fraction * (
            prof["corner_width_m"] - prof["entry_width_m"])

    def limit_at(station: float) -> float:
        width = width_at(station)
        for maximum, limit in rules:
            # The same 1 nm tolerance the observer uses; a bare `<=` put gate
            # 2.4 in the wrong zone by 2.2e-16 m (ADR 0038).
            if width <= maximum + 1e-9:
                return limit
        raise SystemExit(f"policy does not cover width {width}")

    return width_at, limit_at, sigma, consecutive


def truth_at(schedule, when_s: float) -> float | None:
    """Commanded world X at a sim time, linearly interpolated between updates."""

    times = schedule["t"]
    if when_s < times[0] or when_s > times[-1]:
        return None
    return float(np.interp(when_s, times, schedule["x"]))


def truth_speed(schedule, t0: float, t1: float, key: str) -> float | None:
    """Commanded speed over an interval. `key` is "s" (path) or "x" (axis).

    **Both, because they are different quantities and mixing them is an error
    the comparison invents rather than measures.** Pixels give a world-X speed.
    The policy is about how fast the robot is travelling along its path. On a
    one-sided taper the path runs at an angle to X, and through the corner arc
    it turns away from X entirely, so dX/dt is strictly less than ds/dt and the
    gap grows toward the corner.

    Reporting the axis comparison beside the path one separates the estimator's
    own error from the axis conversion, instead of charging the estimator for
    both.
    """

    if t1 <= t0:
        return None
    times, series = schedule["t"], schedule[key]
    if t0 < times[0] or t1 > times[-1]:
        return None
    return float((np.interp(t1, times, series) - np.interp(t0, times, series)) / (t1 - t0))


def window_speed(rows, gate: float, window: float):
    """LSQ fit of station against time over the window. -> dict | None."""

    inside = [r for r in rows if abs(r["station_m"] - gate) <= window]
    if len(inside) < MIN_FRAMES:
        return None
    t = np.array([r["stamp_s"] for r in inside])
    x = np.array([r["station_m"] for r in inside])
    if t.max() - t.min() <= 0:
        return None
    slope, intercept = np.polyfit(t, x, 1)
    residual = x - (slope * t + intercept)
    # Standard error of the slope, the honest uncertainty of this estimate.
    denominator = ((t - t.mean()) ** 2).sum()
    dof = max(len(inside) - 2, 1)
    slope_se = math.sqrt((residual ** 2).sum() / dof / denominator) if denominator else None
    return {"speed_mps": float(slope), "n": len(inside),
            "t0": float(t.min()), "t1": float(t.max()),
            "station_residual_sd_m": float(residual.std(ddof=1)) if len(inside) > 1 else None,
            "speed_stddev_mps": slope_se}


def secant_speed(rows, gate: float, window: float):
    """What GateSpeedEstimator does: two crossings, one difference."""

    inside = sorted((r for r in rows if abs(r["station_m"] - gate) <= window),
                    key=lambda r: r["stamp_s"])
    if len(inside) < 2:
        return None
    first, last = inside[0], inside[-1]
    span = last["stamp_s"] - first["stamp_s"]
    if span <= 0:
        return None
    return {"speed_mps": (last["station_m"] - first["station_m"]) / span, "n": 2}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stations", type=Path, required=True)
    ap.add_argument("--schedule", type=Path, required=True)
    ap.add_argument("--manifest", type=Path, default=ROOT / "out/corridor.manifest.json")
    ap.add_argument("--profile", default="nominal_m6_n3")
    ap.add_argument("--window", type=float, default=WINDOW_M)
    ap.add_argument("--lag", type=Path,
                    help="render-lag artifact; enables the error ATTRIBUTION")
    ap.add_argument("--label", default="")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)

    estimated = json.loads(args.stations.read_text(encoding="utf-8"))
    rows = [r for r in estimated["frames"] if r["station_m"] is not None]
    raw = json.loads(args.schedule.read_text(encoding="utf-8"))

    # THE POSE AND THE FRAMES MUST COME FROM ONE STAGE.
    #
    # The v1 stage and the composed arena BOTH carry a mast prim at the same
    # path, 2.1 m apart and aimed differently. Exporting the pose from the
    # wrong one produced a station for all 208 frames, a full five-gate table,
    # a confirmed violation, and a 13.8 cm bias that read as estimator error.
    # Nothing anywhere failed. This is the check that would have caught it.
    pose_stage = estimated.get("camera_pose_stage")
    if pose_stage and raw.get("stage") and Path(pose_stage).name != Path(raw["stage"]).name:
        raise SystemExit(
            f"the camera pose came from {Path(pose_stage).name} and the frames "
            f"from {Path(raw['stage']).name}. Re-export the pose from the stage "
            f"that was rendered, or every station is measured from the wrong "
            f"place.")
    samples = raw["samples"]
    schedule = {"t": [s["sim_time_s"] for s in samples],
                "x": [s["x_m"] for s in samples],
                "s": [s["route_s_m"] for s in samples]}

    # Per-frame station error: the geometry's own accuracy, before any speed.
    errors = []
    for row in rows:
        want = truth_at(schedule, row["stamp_s"])
        if want is not None:
            errors.append(row["station_m"] - want)

    stations = gate_stations(args.manifest, args.profile)
    width_at, limit_at, sigma, consecutive = policy(args.manifest, args.profile)
    axis_fraction = path_axis_fraction(args.manifest, args.profile)

    table, over_run = [], 0
    for gate in stations:
        limit = limit_at(gate)
        window = window_speed(rows, gate, args.window)
        secant = secant_speed(rows, gate, args.window)
        entry = {"station_m": round(gate, 3), "width_m": round(width_at(gate), 3),
                 "limit_mps": limit, "covered": window is not None}
        if window is None:
            entry["note"] = f"fewer than {MIN_FRAMES} frames in the window"
            table.append(entry)
            over_run = 0
            continue
        want = truth_speed(schedule, window["t0"], window["t1"], "s")
        want_axis = truth_speed(schedule, window["t0"], window["t1"], "x")
        # What the observer would report: an axis speed converted to a path
        # speed by the authored heading, exactly as `MarkerMap` does it.
        measured = window["speed_mps"] / axis_fraction
        deviation = (window["speed_stddev_mps"] / axis_fraction
                     if window["speed_stddev_mps"] else None)
        # The observer's own conservative rule: discount by confidence_sigma
        # before judging, so uncertainty can only excuse, never accuse.
        conservative = measured - sigma * (deviation or 0.0)
        exceeds = conservative > limit
        over_run = over_run + 1 if exceeds else 0
        entry.update({
            "n_frames": window["n"],
            "window_span_s": round(window["t1"] - window["t0"], 3),
            "speed_axis_mps": round(window["speed_mps"], 4),
            "speed_window_mps": round(measured, 4),
            "speed_stddev_mps": round(deviation, 5) if deviation else None,
            "station_residual_sd_m": round(window["station_residual_sd_m"], 4)
            if window["station_residual_sd_m"] else None,
            "speed_secant_mps": round(secant["speed_mps"], 4) if secant else None,
            "truth_path_mps": round(want, 4) if want else None,
            "truth_axis_mps": round(want_axis, 4) if want_axis else None,
            "error_mps": round(measured - want, 4) if want else None,
            "error_pct": round(100.0 * (measured - want) / want, 2) if want else None,
            # The estimator judged on the quantity it actually measures.
            "axis_error_pct": round(
                100.0 * (window["speed_mps"] - want_axis) / want_axis, 2)
            if want_axis else None,
            "secant_error_pct": round(
                100.0 * (secant["speed_mps"] / axis_fraction - want) / want, 2)
            if (want and secant) else None,
            "conservative_mps": round(conservative, 4),
            "over_limit": exceeds,
            "confirmed": exceeds and over_run >= consecutive,
        })
        table.append(entry)

    covered = sum(1 for row in table if row["covered"])
    confirmed = [row for row in table if row.get("confirmed")]

    print(f"\n{args.label or args.stations.parent.name}  "
          f"window +/-{args.window} m, {len(rows)} frames with a station\n")
    print(f"{'gate':>5} {'width':>6} {'limit':>6} {'n':>4} {'measured':>9} "
          f"{'truth':>8} {'err':>8} {'err%':>7} {'axis%':>7} {'sd':>8} "
          f"{'secant%':>8} verdict")
    for row in table:
        if not row["covered"]:
            print(f"{row['station_m']:>5.1f} {row['width_m']:>6.2f} "
                  f"{row['limit_mps']:>6.2f}    -   (not covered: {row['note']})")
            continue
        print(f"{row['station_m']:>5.1f} {row['width_m']:>6.2f} {row['limit_mps']:>6.2f} "
              f"{row['n_frames']:>4} {row['speed_window_mps']:>9.4f} "
              f"{row['truth_path_mps']:>8.4f} {row['error_mps']:>+8.4f} "
              f"{row['error_pct']:>+7.2f} {row['axis_error_pct']:>+7.2f} "
              f"{(row['speed_stddev_mps'] or 0):>8.5f} "
              f"{(row['secant_error_pct'] if row['secant_error_pct'] is not None else 0):>+8.2f} "
              f"{'OVER' if row['over_limit'] else 'compliant'}"
              f"{' **CONFIRMED**' if row.get('confirmed') else ''}")

    # ERROR ATTRIBUTION, not correction.
    #
    # If the pixels in a frame show the scene as it was before the stamp on
    # that frame, and the gap grows at g seconds per second, then a speed read
    # off those stamps is low by g -- arithmetic, independent of the estimator.
    # Reporting measured, predicted and residual side by side says how much of
    # the error is the recording path and how much is the estimator. Nothing
    # is subtracted from the table: a correction derived from the run it
    # corrects is a fitted parameter wearing a mechanism's clothes.
    attribution = None
    if args.lag and args.lag.is_file():
        lag = json.loads(args.lag.read_text(encoding="utf-8"))
        growth = lag.get("lag_growth_s_per_s")
        if growth is not None:
            measured = [row["error_pct"] for row in table
                        if row.get("error_pct") is not None]
            predicted = -100.0 * growth
            attribution = {
                "lag_growth_s_per_s": growth,
                "content_rate_vs_clock": lag.get("content_rate_vs_clock"),
                "predicted_error_pct": round(predicted, 2),
                "mean_measured_error_pct": round(statistics.fmean(measured), 2)
                if measured else None,
                "residual_pct": round(statistics.fmean(measured) - predicted, 2)
                if measured else None,
            }

    magnitudes = [abs(row["error_pct"]) for row in table
                  if row.get("error_pct") is not None]
    print(f"\ngate coverage      {covered}/{len(stations)}")
    if magnitudes:
        print(f"worst |error|      {max(magnitudes):.2f}%")
        print(f"mean |error|       {statistics.fmean(magnitudes):.2f}%")
    if errors:
        print(f"per-frame station  bias {statistics.fmean(errors):+.4f} m, "
              f"sd {statistics.stdev(errors):.4f} m, n {len(errors)}")
    if attribution:
        print("\nerror attribution  (reported, NOT applied)")
        print(f"  render lag grows {attribution['lag_growth_s_per_s']:+.3f} s/s, so "
              f"content runs at {attribution['content_rate_vs_clock']:.3f}x the clock")
        print(f"  predicted from the lag alone   {attribution['predicted_error_pct']:+.2f}%")
        print(f"  measured, mean over the gates  {attribution['mean_measured_error_pct']:+.2f}%")
        print(f"  RESIDUAL, the estimator's own  {attribution['residual_pct']:+.2f}%")
    print(f"violations         {len(confirmed)} confirmed"
          + (f" (first at gate {confirmed[0]['station_m']})" if confirmed else ""))

    doc = {"label": args.label, "stations": str(args.stations),
           "schedule": str(args.schedule), "profile": args.profile,
           "window_m": args.window, "min_frames": MIN_FRAMES,
           "truth_source": "commanded pose schedule (--drive-out); "
                           "evaluation input, never an observer input",
           "confidence_sigma": sigma, "consecutive_estimates": consecutive,
           "path_axis_fraction": axis_fraction,
           "frames_with_station": len(rows),
           "gate_coverage": f"{covered}/{len(stations)}",
           "per_frame_station_bias_m": round(statistics.fmean(errors), 5) if errors else None,
           "per_frame_station_sd_m": round(statistics.stdev(errors), 5)
           if len(errors) > 1 else None,
           "worst_abs_error_pct": round(max(magnitudes), 3) if magnitudes else None,
           "confirmed_violations": len(confirmed),
           "error_attribution": attribution,
           "gates": table}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
