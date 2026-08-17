#!/usr/bin/env python3
"""Correlate matcher acquisition against corridor geometry, from committed runs.

    python3 tools/degeneracy_analysis.py --out out/evidence/robot-a-gate/degeneracy-analysis.json

Pure re-analysis of `docs/evidence/robot-a-gate/gate-*.json` plus the scenario
manifest. No simulator, no GPU, no ROS.

WHAT THIS CAN AND CANNOT ESTABLISH
----------------------------------
Robot2's three corridor runs share one signature: the scan matcher publishes
nothing until the robot is ~5 m in, and the along-corridor covariance sits ~18x
the cross-corridor value once it does. The obvious hypothesis is that the
matcher acquires when enough along-axis structure enters lidar range -- the
corridor's end wall being the strongest such feature.

**Three profiles cannot establish that.** They differ in taper, and taper
covaries with local width, with the distance at which the end wall subtends a
usable angle, and with how much of the far wall is occluded by the near walls.
This tool computes the candidate predictors and reports them side by side; it
deliberately does not fit anything, rank hypotheses, or report a correlation
coefficient over n=3, which would dress three numbers up as a trend.

There is also a **hard confound with the robot swap**: robot2 carries a C1 with
a 12.0 m maximum range and robot1 carries an MS200 with 8.0 m
(`yahboomcar-ros2/tools/build_arena.py:54`). The corridor's end wall stands
11.50 m from A's spawn on every profile, so C1 can range it from station 0
while MS200 cannot see it at all until station 3.50 m. Any robot1-vs-robot2
comparison of acquisition station is therefore a comparison of two sensors as
well as two odometry architectures, and this file states that rather than
letting a later reader assume otherwise.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

PROFILES = ("nominal_m6_n3", "wide_corner_m6_n4_5", "uniform_m6_n6")

#: Maximum range of each robot's lidar, for the range-confound column.
C1_MAX_RANGE_M = 12.0        # robot2, build_rasptank_arena.py:46
MS200_MAX_RANGE_M = 8.0      # robot1, build_arena.py:54


def end_wall_x_m(profile_manifest: dict) -> float:
    """West face of the mass the corridor runs into (the along-axis feature)."""

    return min(point[0] for point in profile_manifest["walls"]["CornerBuilding"])


def local_clear_width_m(profile_manifest: dict, station_m: float, length_m: float) -> float:
    """Linear taper from entry width to corner width over the corridor length."""

    fraction = min(1.0, max(0.0, station_m / length_m))
    entry = profile_manifest["entry_width_m"]
    corner = profile_manifest["corner_width_m"]
    return entry + (corner - entry) * fraction


def analyse(manifest: dict, gate: dict, profile: str) -> dict:
    entry = manifest["profiles"][profile]
    length_m = manifest["corridor_length_m"]
    acquisition_m = gate["first_odom_laser_station_m"]

    start_x, start_y, _ = entry["a_start_xyz_m"]
    heading_x, heading_y = entry["delivery_trajectory"]["approach_heading"]
    position = (
        start_x + heading_x * acquisition_m,
        start_y + heading_y * acquisition_m,
    )
    wall_x = end_wall_x_m(entry)

    covariance = gate["midpoint_covariance"]
    return {
        "profile": profile,
        "acquisition_station_m": acquisition_m,
        "max_consecutive_withheld": gate["max_consecutive_withheld_updates"],
        "entry_width_m": entry["entry_width_m"],
        "corner_width_m": entry["corner_width_m"],
        "taper_m_per_m": (entry["corner_width_m"] - entry["entry_width_m"]) / length_m,
        "end_wall_x_m": wall_x,
        "end_wall_distance_at_spawn_m": round(wall_x - start_x, 3),
        "end_wall_distance_at_acquisition_m": round(wall_x - position[0], 3),
        "local_clear_width_at_acquisition_m": round(
            local_clear_width_m(entry, acquisition_m, length_m), 3
        ),
        "station_end_wall_enters_c1_range_m": round(
            max(0.0, (wall_x - start_x) - C1_MAX_RANGE_M), 3
        ),
        "station_end_wall_enters_ms200_range_m": round(
            max(0.0, (wall_x - start_x) - MS200_MAX_RANGE_M), 3
        ),
        "midpoint_cov_xx": covariance["cov_xx"],
        "midpoint_cov_yy": covariance["cov_yy"],
        "midpoint_anisotropy": round(covariance["cov_xx"] / covariance["cov_yy"], 2),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--manifest", default="out/corridor.manifest.json")
    parser.add_argument("--evidence", default="docs/evidence/robot-a-gate")
    parser.add_argument("--out", required=True)
    arguments = parser.parse_args()

    manifest = json.loads(Path(arguments.manifest).read_text(encoding="utf-8"))
    rows = []
    for profile in PROFILES:
        gate_path = Path(arguments.evidence) / f"gate-{profile}.json"
        gate = json.loads(gate_path.read_text(encoding="utf-8"))
        rows.append(analyse(manifest, gate, profile))

    # Ordering checks, not fits. "Do these two quantities order the same way?"
    # is answerable with three points; "how strongly are they related?" is not.
    def orders_with(key: str) -> str:
        ordered = sorted(rows, key=lambda r: r["acquisition_station_m"])
        by_acquisition = [row[key] for row in ordered]
        if by_acquisition == sorted(by_acquisition):
            return "increases with acquisition station"
        if by_acquisition == sorted(by_acquisition, reverse=True):
            return "decreases with acquisition station"
        return "no monotonic ordering"

    result = {
        "question": "does corridor geometry order the matcher's acquisition station?",
        "sample_size": len(rows),
        "profiles": rows,
        "orderings": {
            key: orders_with(key)
            for key in (
                "taper_m_per_m",
                "local_clear_width_at_acquisition_m",
                "end_wall_distance_at_acquisition_m",
                "max_consecutive_withheld",
                "midpoint_anisotropy",
            )
        },
        "range_confound": {
            "end_wall_distance_from_spawn_m": rows[0]["end_wall_distance_at_spawn_m"],
            "c1_max_range_m": C1_MAX_RANGE_M,
            "ms200_max_range_m": MS200_MAX_RANGE_M,
            "c1_can_range_end_wall_from_spawn": (
                rows[0]["end_wall_distance_at_spawn_m"] <= C1_MAX_RANGE_M
            ),
            "ms200_can_range_end_wall_from_spawn": (
                rows[0]["end_wall_distance_at_spawn_m"] <= MS200_MAX_RANGE_M
            ),
            "note": (
                "robot1 (MS200, 8.0 m) cannot range the end wall until station "
                f"{rows[0]['station_end_wall_enters_ms200_range_m']} m; robot2 (C1, 12.0 m) "
                "can from station 0. A robot1-vs-robot2 acquisition comparison is "
                "therefore two sensors as well as two odometry architectures."
            ),
        },
        "claim_limit": (
            "n=3, and taper covaries with local width and with end-wall geometry. "
            "Orderings are reported; no mechanism, fit, or correlation coefficient "
            "is claimed. ADR 0027 refuses that claim and this analysis does not "
            "quietly acquire it."
        ),
    }

    destination = Path(arguments.out)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    width = max(len(row["profile"]) for row in rows)
    print(f"{'profile':{width}} {'acq_m':>6} {'endwall@acq':>12} {'width@acq':>10} {'aniso':>7}")
    for row in rows:
        print(
            f"{row['profile']:{width}} {row['acquisition_station_m']:6.2f} "
            f"{row['end_wall_distance_at_acquisition_m']:12.2f} "
            f"{row['local_clear_width_at_acquisition_m']:10.2f} "
            f"{row['midpoint_anisotropy']:7.1f}"
        )
    print()
    for key, ordering in result["orderings"].items():
        print(f"  {key:38} {ordering}")
    print(f"\nwritten: {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
