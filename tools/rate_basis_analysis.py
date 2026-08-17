#!/usr/bin/env python3
"""Rung 1: is the adapter's rate shortfall a clock-basis artifact?

    python3 tools/rate_basis_analysis.py --out out/evidence/crossing/rate-basis.json

Pure analysis of already-captured evidence. It starts no simulator and needs no
GPU, which is the point: if the 12.93 Hz recorded against a declared 15 Hz is
simply wall-clock time being compared against a contract stated in simulation
time, that is answerable from the JSONs already committed, and no Isaac session
should be spent on it.

TWO QUESTIONS, NOT ONE
----------------------
1. **Clock basis.** Isaac cameras publish per render tick, so a simulation
   running below real time emits fewer frames per WALL second while remaining
   exactly on rate per SIMULATION second. The real-time factor separates these:
   it is the span of `/clock` observed on A's plane divided by the wall-clock
   window the capture ran for.

2. **What the image counts actually measure.** `CameraInfo` and `Image` leave
   the same render product on the same tick, so the contract rate governs both.
   They differ enormously in size -- a 640x360 rgb8 frame is ~691 kB against a
   handful of bytes -- and every subscriber in this measurement is BEST_EFFORT
   by necessity, because the publisher offers BEST_EFFORT and a RELIABLE
   subscriber would match nothing at all. So `CameraInfo` arrivals are a much
   better estimator of the EMIT rate than `Image` arrivals are: both are lower
   bounds, but only one is depressed by large-message loss.

If those two estimators disagree, the recorded "publisher rate" is a property of
the measuring instrument rather than of the publisher, and no amount of renderer
tuning (rung 2) or re-declaring (rung 3) would be addressing the real thing.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

DECLARED_HZ = 15.0


def analyse(capture: dict) -> dict:
    frames = capture["frames"]
    wall_window_s = capture["seconds"]
    robot_clock = capture["clock"]["robot_plane"]
    police_clock = capture["clock"]["police_plane"]

    sim_span_s = robot_clock["span_s"]
    police_span_s = police_clock["span_s"]
    real_time_factor = sim_span_s / wall_window_s

    published = frames["published_on_robot_plane"]
    camera_info = frames["camera_info_delivered"]

    image_rate_wall = published / wall_window_s
    image_rate_sim = published / sim_span_s if sim_span_s else 0.0
    # CameraInfo is counted in P's plane (it is what the measurement subscribed
    # to there), so it is divided by P's own clock span for consistency.
    info_rate_sim = camera_info / police_span_s if police_span_s else 0.0

    return {
        "label": capture["label"],
        "wall_window_s": wall_window_s,
        "sim_span_s": sim_span_s,
        "real_time_factor": round(real_time_factor, 4),
        "declared_hz": DECLARED_HZ,
        "image_rate_hz_wall_basis": round(image_rate_wall, 3),
        "image_rate_hz_sim_basis": round(image_rate_sim, 3),
        "camera_info_rate_hz_sim_basis": round(info_rate_sim, 3),
        "image_fraction_of_declared_sim_basis": round(image_rate_sim / DECLARED_HZ, 4),
        "camera_info_fraction_of_declared_sim_basis": round(info_rate_sim / DECLARED_HZ, 4),
        "estimators_disagree_by_hz": round(info_rate_sim - image_rate_sim, 3),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--captures",
        nargs="+",
        default=[
            "docs/evidence/crossing/crossing-640x360.json",
            "docs/evidence/crossing/crossing-1280x720.json",
        ],
    )
    parser.add_argument("--out", required=True)
    arguments = parser.parse_args()

    analyses = [
        analyse(json.loads(Path(path).read_text(encoding="utf-8")))
        for path in arguments.captures
    ]

    # Rung 1 resolves ONLY if simulation time explains the gap: the image rate on
    # a simulation-time basis has to reach the declared figure.
    clock_basis_explains = all(
        entry["image_fraction_of_declared_sim_basis"] >= 0.95 for entry in analyses
    )
    # The confound: same render product, same tick, two message sizes.
    instrument_confounded = any(
        entry["estimators_disagree_by_hz"] > 0.5 for entry in analyses
    )

    if clock_basis_explains:
        verdict = "RUNG_1_RESOLVES"
        reason = (
            "on a simulation-time basis the adapter meets the declared rate; the "
            "shortfall was a wall-clock comparison against a sim-time contract"
        )
    elif instrument_confounded:
        verdict = "RUNG_1_INCONCLUSIVE_INSTRUMENT_CONFOUNDED"
        reason = (
            "the real-time factor does not explain the gap, but CameraInfo and "
            "Image -- same render product, same tick, different sizes -- give "
            "materially different rate estimates, so the recorded publisher rate "
            "is a property of the BEST_EFFORT measuring subscriber, not of the "
            "publisher. Attributable accounting (step B) must settle it before "
            "any renderer work or re-declaration"
        )
    else:
        verdict = "RUNG_1_DOES_NOT_RESOLVE"
        reason = (
            "the rate is short on a simulation-time basis too, and the two "
            "estimators agree, so the publisher really is under-running"
        )

    result = {
        "question": "is the adapter rate shortfall a clock-basis artifact?",
        "declared_hz": DECLARED_HZ,
        "captures": analyses,
        "clock_basis_explains_shortfall": clock_basis_explains,
        "estimators_disagree": instrument_confounded,
        "verdict": verdict,
        "reason": reason,
    }

    destination = Path(arguments.out)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    print(f"\nwritten: {destination}")
    print("VERDICT:", verdict)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
