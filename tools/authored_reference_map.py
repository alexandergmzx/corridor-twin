#!/usr/bin/env python3
"""Render the AUTHORED corridor as the map a perfect SLAM would have produced.

    python3 tools/authored_reference_map.py \\
        --profile nominal_m6_n3 --resolution 0.02 --out out/evidence/authored-map.yaml

WHY THIS EXISTS
---------------
`score_slam_map.py` scores a saved map for `duplicate wall extent` -- two occupied
runs within 0.40 m of the outermost occupied cell, split by a gap. On a smeared
map that is the same wall drawn twice, and it is the right instrument for
catching divergence.

It was written for a CONVEX 4x4 m room. This corridor is neither: it TAPERS, so
its two side walls converge along their length, and it turns a CORNER, where the
north-wall extension and the east building face are two separate surfaces a
column can cross within one 0.40 m band. Both are authored geometry. A metric
that reads them as doubling would fail a perfectly good map, and -- much worse --
would let a real divergence hide inside a threshold raised to accommodate them.

So the metric is calibrated against the geometry it will be applied to. This
renders the scene's own free-space oracle (`scene.geometry.is_clear`, the same
function the route validator and the standoff test use) into map_saver's format,
which can then be scored by the SAME instrument. Whatever `duplicate wall extent`
that scores is the AUTHORED floor: the part of the reading that is the corridor's
shape rather than the run's error. A real map is only convicted of divergence by
the amount it exceeds that floor.

WHAT A PERFECT SLAM WOULD SEE
-----------------------------
Lidar returns come from wall SURFACES, not from wall interiors, and nothing
behind a wall is observed at all. So a cell is:

  * free      -- `is_clear` says drivable;
  * occupied  -- not drivable, but orthogonally adjacent to something drivable,
                 i.e. the surface a beam would strike;
  * unknown   -- not drivable and not adjacent to anything drivable: the inside
                 of a wall and the world beyond it.

Marking whole wall masses occupied instead would inflate every thickness and
doubling measurement, and would make the control useless in the direction that
matters -- it would excuse real smear.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "src/corridor_scene"))

from scene.geometry import is_clear  # noqa: E402
from scene.model import load_scenario  # noqa: E402

#: map_saver's trinary palette. Not invented here: `score_slam_map.py:33-35`
#: classifies 205 as unknown BEFORE applying the YAML thresholds, so a control
#: written with any other unknown value would be scored as free and would
#: report a wall-free world as perfectly explored.
FREE, OCCUPIED, UNKNOWN = 254, 0, 205


def render(scenario, profile, resolution: float, margin_m: float = 0.5):
    """-> (rows of palette values, origin_x, origin_y). Row 0 is the TOP (max y)."""

    # Bounds are found by probing rather than by reading geometry internals:
    # the oracle is the authority on what is drivable, and a bounds computation
    # that disagreed with it would silently clip the map.
    probe = resolution * 2.0
    xs, ys = [], []
    x = -scenario.west_margin_m - margin_m
    while x <= scenario.corridor_length_m + scenario.next_street.length_m + margin_m * 4:
        y = -(scenario.corridor_length_m + margin_m * 4)
        while y <= scenario.corridor_length_m + margin_m * 4:
            if is_clear(scenario, profile, x, y):
                xs.append(x)
                ys.append(y)
            y += probe
        x += probe
    if not xs:
        raise SystemExit("the oracle reports no drivable space anywhere in the probe box")

    origin_x, origin_y = min(xs) - margin_m, min(ys) - margin_m
    width = int((max(xs) + margin_m - origin_x) / resolution) + 1
    height = int((max(ys) + margin_m - origin_y) / resolution) + 1

    clear = [
        [
            is_clear(scenario, profile,
                     origin_x + (col + 0.5) * resolution,
                     origin_y + (row + 0.5) * resolution)
            for col in range(width)
        ]
        for row in range(height)
    ]

    grid = []
    for row in range(height):
        line = []
        for col in range(width):
            if clear[row][col]:
                line.append(FREE)
                continue
            touches_free = any(
                clear[row + dr][col + dc]
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
                if 0 <= row + dr < height and 0 <= col + dc < width
            )
            line.append(OCCUPIED if touches_free else UNKNOWN)
        grid.append(line)

    # PGM rows run top-down; the map origin is the BOTTOM-left corner.
    grid.reverse()
    return grid, origin_x, origin_y


def write_map(grid, origin_x: float, origin_y: float, resolution: float, out: Path) -> None:
    pgm = out.with_suffix(".pgm")
    height, width = len(grid), len(grid[0])
    body = bytearray()
    for row in grid:
        body.extend(bytes(row))
    pgm.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + bytes(body))
    out.write_text(
        f"image: {pgm.name}\n"
        f"mode: trinary\n"
        f"resolution: {resolution:.3f}\n"
        f"origin: [{origin_x:.3f}, {origin_y:.3f}, 0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    # Default None -> scene.model.default_config_path(), which IS the scenario
    # as run. Hardcoding the path here made this tool a second place that had
    # to be told which scenario was current.
    parser.add_argument("--config", default=None)
    parser.add_argument("--profile", default="nominal_m6_n3")
    parser.add_argument("--resolution", type=float, default=0.02)
    parser.add_argument("--out", required=True)
    arguments = parser.parse_args()

    scenario = load_scenario(Path(arguments.config) if arguments.config else None)
    profile = next(p for p in scenario.profiles if p.name == arguments.profile)

    grid, origin_x, origin_y = render(scenario, profile, arguments.resolution)
    destination = Path(arguments.out)
    destination.parent.mkdir(parents=True, exist_ok=True)
    write_map(grid, origin_x, origin_y, arguments.resolution, destination)

    counts = {name: 0 for name in ("free", "occupied", "unknown")}
    for row in grid:
        for value in row:
            counts["free" if value == FREE else
                   "occupied" if value == OCCUPIED else "unknown"] += 1
    print(f"{destination}: {len(grid[0])}x{len(grid)} @ {arguments.resolution} m, "
          f"origin ({origin_x:.3f}, {origin_y:.3f})")
    print(f"  free {counts['free']}  occupied {counts['occupied']}  "
          f"unknown {counts['unknown']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
