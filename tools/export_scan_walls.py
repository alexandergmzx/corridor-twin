#!/usr/bin/env python3
"""Write the corridor's walls in the form the twin's scan filter validates against.

    python3 tools/export_scan_walls.py --manifest out/corridor.manifest.json \
        --profile nominal_m6_n3 --out out/scan-walls-nominal_m6_n3.json

WHY
---
`_scan_frame_relay` drops phase-corrupted lidar revolutions by asking whether a
beam returns from BEYOND the wall it should have hit. That needs a wall model,
and it had exactly one: `segments_room()`, the stock 4 x 4 m yahboom test arena,
called with no arguments.

On the corridor -- 6 x 7 m, L-shaped, walls nowhere near a 4 m box -- essentially
every beam looks impossible. Measured across **56 of 62** `-isaac-d67` sessions
on this box: `/scan` publishes **nothing at all** for the ~21 s it takes to fill
the 300-sample fail-open window, and then the node disables itself and passes
raw scans for the remainder of the run.

Both halves are silent. The blackout reads as a slow twin -- it is the same
window in which SLAM has no scans and Nav2's costmaps are empty, which is where
the startup circle is hunted -- and the passthrough reads as a working filter
while the corrupted revolutions reach slam_toolbox, both costmaps and the
governor.

THE MANIFEST IS THE SOURCE
--------------------------
Same source as the arena, per profile. Nothing here is hand-authored, so the
model the filter validates against and the geometry the simulator loads cannot
become two descriptions of different scenes -- which is the fault this
repository has already spent a day on.

Wall polygons become their four edges. The filter raycasts against segments and
takes the nearest hit, so a closed rectangle per building is exactly right: an
interior edge is simply never the nearest one from outside.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def wall_segments(manifest: dict, profile: str) -> list[list[list[float]]]:
    walls = manifest["profiles"][profile]["walls"]
    segments: list[list[list[float]]] = []
    for corners in walls.values():
        points = [(float(x), float(y)) for x, y in corners]
        for start, end in zip(points, points[1:] + points[:1], strict=True):
            segments.append([[start[0], start[1]], [end[0], end[1]]])
    return segments


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--out", type=Path, required=True)
    arguments = parser.parse_args()

    manifest = json.loads(arguments.manifest.read_text(encoding="utf-8"))
    segments = wall_segments(manifest, arguments.profile)
    arguments.out.parent.mkdir(parents=True, exist_ok=True)
    arguments.out.write_text(json.dumps(segments) + "\n", encoding="utf-8")

    buildings = len(manifest["profiles"][arguments.profile]["walls"])
    print(
        f"  {len(segments)} wall segments from {buildings} buildings "
        f"({arguments.profile}) -> {arguments.out}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
