#!/usr/bin/env python3
"""Hide the corridor's own double surface from a duplicate-wall scorer.

    python3 tools/mask_authored_double_surface.py --map run/map.yaml \
        --manifest out/corridor.manifest.json --profile nominal_m6_n3 \
        --frame robot_start --out run/map-masked.yaml --json run/mask.json

WHY
---
`duplicate wall extent` is the ghosting signature: the same wall drawn twice.
It asks, per column and per row, whether there are two separate occupied runs
within `band_m = 0.40 m` of the OUTERMOST occupied cell, and reports the longest
consecutive stretch of lines where there are.

That question was posed of a plain 4 x 4 m room, which has no internal
structure. This corridor has two, both inside the band, both authored on
purpose: ADR 0019's corner screen stands 0.33 m west of the east wall and
parallel to it, and ADR 0018's east wall stub is a 0.318 m-thick block
protruding from that same wall.

Measured on the authored "perfect SLAM" oracle -- the scene with no sensor and
no drift in it at all -- the metric reads **0.340 m** against a 0.20 m limit. A
perfect map failed. Neither feature is negotiable: shortening the screen is what
made the occlusion certificate fail with P visible along the entire approach,
and the stub is the drawing's own geometry.

MASK, NOT SUBTRACT
------------------
Subtracting a floor keeps the blind spot AND moves the threshold, so a run's
number stops being comparable with every number recorded before it. Masking
removes exactly the authored regions and leaves the threshold where it was
measured. The cost is stated rather than hidden: **ghosting inside the masked
polygons is not detected.** Measured: 0.117% of the map's cells for the screen,
and the two together take the oracle to exactly 0.000 m -- masking a THIRD wall
changes nothing, which is how the list is known to be complete.

THE POLYGONS COME FROM THE MANIFEST
-----------------------------------
Same source as the arena, so the mask and the geometry cannot drift into two
descriptions of different scenes -- which is the fault this repository spent a
day on. Nothing here is hand-authored: `profiles[<p>].walls` is read straight
out of the manifest and padded.

FRAMES
------
The oracle is rendered in WORLD coordinates. A SLAM map is in the MAP frame,
which starts at A's spawn pose, so the same polygon has to be rotated and
translated by `a_start_xyz_m` and the profile's `approach_heading` before it
means anything there. `--frame` picks which, and it defaults to nothing: a
silent wrong frame would mask empty space and quietly change nothing.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

#: Padding around the authored polygon, in metres. The wall is rendered with
#: thickness and a real map smears it by a cell or two, so the mask has to be
#: slightly larger than the geometry or it leaves a rim behind that scores.
PAD_M = 0.06

#: Trinary map values, matching authored_reference_map.py.
UNKNOWN = 205

#: The walls this masks, NAMED rather than inferred. A tool that decides for
#: itself which walls are inconvenient is not an instrument -- so this list is
#: closed, each entry carries the ADR that authored it and the reason the metric
#: mistakes it, and the test asserts that masking them takes the oracle to
#: exactly 0.000 while a THIRD wall changes nothing.
#:
#: Both are internal structures standing within the scorer's 0.40 m band of an
#: outer wall, which is the situation its "outermost occupied run, doubled?"
#: test was never meant to read: the room it was tuned in has no internal
#: structure at all.
MASKED_WALLS = (
    # ADR 0019. Runs north-south at x 4.95-5.07, parallel to the east
    # building's inner face at x 5.40 and overlapping it in y from 0.57 to
    # 0.90. Two separate surfaces 0.33 m apart. Measured contribution: 0.100 m.
    "CornerScreen",
    # ADR 0018. A 0.318 m-thick block protruding west from the east wall; its
    # two long faces are parallel and inside the band. Measured contribution:
    # 0.240 m, and it only shows once the screen is masked -- the metric
    # reports the LONGEST doubled stretch, so features hide behind each other.
    "EastWallStub",
)


def read_map(path: Path) -> tuple[list[bytearray], float, float, float, str]:
    meta = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        meta[key.strip()] = value.strip()
    resolution = float(meta["resolution"])
    origin = json.loads(meta["origin"].replace("'", '"'))
    pgm = path.parent / meta["image"]

    data = pgm.read_bytes()
    fields, offset = [], 0
    while len(fields) < 4:
        end = data.index(b"\n", offset)
        token = data[offset:end]
        offset = end + 1
        if token.startswith(b"#"):
            continue
        fields.extend(token.split())
    magic, width, height, _maxval = fields[0], int(fields[1]), int(fields[2]), int(fields[3])
    if magic != b"P5":
        raise ValueError(f"{pgm} is not a binary PGM")
    body = data[offset:]
    grid = [bytearray(body[row * width:(row + 1) * width]) for row in range(height)]
    return grid, resolution, float(origin[0]), float(origin[1]), meta["image"]


def write_map(grid, resolution: float, origin_x: float, origin_y: float, out: Path) -> None:
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


def polygons_from_manifest(
    manifest: dict, profile: str, frame: str
) -> list[list[tuple[float, float]]]:
    walls = manifest["profiles"][profile]["walls"]
    missing = [name for name in MASKED_WALLS if name not in walls]
    if missing:
        raise ValueError(f"profile {profile!r} authors no {', '.join(missing)}")
    polygons = [
        [(float(x), float(y)) for x, y in walls[name]] for name in MASKED_WALLS
    ]

    if frame == "world":
        return polygons
    # A SLAM map's frame starts at A's spawn pose, so the world polygon is
    # expressed relative to it: translate by -a_start, then rotate by -heading.
    entry = manifest["profiles"][profile]
    start_x, start_y, _ = entry["a_start_xyz_m"]
    heading_x, heading_y = entry["delivery_trajectory"]["approach_heading"]
    angle = -math.atan2(heading_y, heading_x)
    cos, sin = math.cos(angle), math.sin(angle)
    return [
        [
            (
                (x - start_x) * cos - (y - start_y) * sin,
                (x - start_x) * sin + (y - start_y) * cos,
            )
            for x, y in polygon
        ]
        for polygon in polygons
    ]


def mask(
    map_path: Path, manifest_path: Path, profile: str, frame: str, out: Path
) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    polygons = polygons_from_manifest(manifest, profile, frame)
    grid, resolution, origin_x, origin_y, _image = read_map(map_path)
    height, width = len(grid), len(grid[0])

    # Axis-aligned by construction (both are rectangles), so a bounding box IS
    # the polygon. Stated rather than assumed: a rotated polygon would need a
    # real point-in-polygon test and this would silently over-mask.
    boxes = [
        (
            min(x for x, _ in polygon) - PAD_M,
            min(y for _, y in polygon) - PAD_M,
            max(x for x, _ in polygon) + PAD_M,
            max(y for _, y in polygon) + PAD_M,
        )
        for polygon in polygons
    ]

    masked = 0
    for row in range(height):
        # Row 0 is the TOP of the image, which is the highest y.
        world_y = origin_y + (height - 1 - row) * resolution
        for column in range(width):
            world_x = origin_x + column * resolution
            inside = any(
                low_x <= world_x <= high_x and low_y <= world_y <= high_y
                for low_x, low_y, high_x, high_y in boxes
            )
            if inside and grid[row][column] != UNKNOWN:
                grid[row][column] = UNKNOWN
                masked += 1

    write_map(grid, resolution, origin_x, origin_y, out)
    return {
        "map": str(map_path),
        "masked_map": str(out),
        "manifest": str(manifest_path),
        "profile": profile,
        "frame": frame,
        "walls": list(MASKED_WALLS),
        "pad_m": PAD_M,
        "polygons": [[[round(x, 4), round(y, 4)] for x, y in p] for p in polygons],
        "bounds": [[round(v, 4) for v in box] for box in boxes],
        "cells_masked": masked,
        "cells_total": width * height,
        "masked_fraction": round(masked / (width * height), 6),
        "blind_spot_note": (
            "ghosting INSIDE these polygons is not detected. They are the two "
            "authored internal structures -- ADR 0019's corner screen and "
            "ADR 0018's east wall stub -- which stand within the scorer's "
            "0.40 m band of an outer wall and which it therefore reads as "
            "doubled. Masked they take the perfect-map oracle to 0.000 m."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument(
        "--frame", required=True, choices=("world", "robot_start"),
        help="world for the authored oracle, robot_start for a SLAM map",
    )
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--json", type=Path)
    arguments = parser.parse_args()

    report = mask(
        arguments.map, arguments.manifest, arguments.profile, arguments.frame, arguments.out
    )
    if arguments.json:
        arguments.json.parent.mkdir(parents=True, exist_ok=True)
        arguments.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        f"  masked {report['cells_masked']} cells "
        f"({100 * report['masked_fraction']:.3f}% of the map) over "
        f"{', '.join(MASKED_WALLS)} in the {arguments.frame} frame -> {arguments.out}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
