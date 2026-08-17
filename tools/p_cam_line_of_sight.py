#!/usr/bin/env python3
"""Can P's camera see A, in 3-D, on the stage as composed?

    python3 tools/p_cam_line_of_sight.py \
        --stage out/corridor.usda --manifest out/corridor.manifest.json \
        --profile nominal_m6_n3 --out out/evidence/p_cam_candidates/los.json

WHY THIS EXISTS AS A TOOL
-------------------------
The 2026-08-12 decision memo reported that the mast clears all five enforcement
stations in 3-D, and that result decided where P's camera goes. It was produced
by an ad-hoc use of `scene.occlusion`'s raycaster and no committed script
reproduced it, so the load-bearing number for the whole Phase 3 pose was
unrepeatable. This is that script.

WHY THE 2-D ANSWER WAS WRONG, AND WHY THAT MATTERS HERE
-------------------------------------------------------
`p_cam_candidates.py` works in plan view and called the mast BLOCKED, because
ADR 0019's corner screen is in the way in plan -- which is exactly what a mast
is for. The screen and the corridor walls are finite in height, so a plan-view
test is structurally incapable of judging a raised camera. This one casts
against the composed triangles.

WHAT IT DOES NOT DO
-------------------
It chooses nothing, it renders nothing, and it opens no simulator. It reads the
stage the composer wrote, the pose the manifest carries, and reports.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

# Logical parent, never resolve(): D5 forbids realpath on checkout paths
# because it escapes the fleet symlink into ~/Development. Nothing here reaches
# a sibling repo, but the pattern is the one this repo bans, so it is not used.
ROOT = Path(os.path.abspath(__file__)).parent.parent
sys.path.insert(0, str(ROOT / "src" / "corridor_scene"))

from pxr import Usd  # noqa: E402
from scene.occlusion import (  # noqa: E402
    _mesh_triangles,
    _segment_hits_triangle,
    opaque_mesh_prims,
)
from scene.trajectory import trajectory_from_manifest  # noqa: E402

#: A's body centre above the ground. The subject is the chassis, not a point on
#: the floor: a ray to the floor grazes it and reports blocked where a camera
#: plainly sees the robot.
A_BODY_CENTRE_HEIGHT_M = 0.075

#: Where along A's approach enforcement is measured. Fractions of the approach
#: leg rather than absolute metres, so the stations follow a rescale instead of
#: silently walking off the end of a shorter corridor.
STATION_FRACTIONS = (0.2, 0.4, 0.6, 0.8, 1.0)


def _blocked(source, target, triangles) -> bool:
    """True if any opaque triangle strictly interrupts the segment."""

    for triangle in triangles:
        if _segment_hits_triangle(source, target, triangle) is not None:
            return True
    return False


def evaluate(stage_path: Path, manifest: dict, profile: str) -> dict:
    stage = Usd.Stage.Open(str(stage_path))
    if stage is None:
        raise SystemExit(f"could not open {stage_path}")

    prims = opaque_mesh_prims(stage)
    triangles = [t for prim in prims for t in _mesh_triangles(prim)]

    entry = manifest["profiles"][profile]
    pose = entry["p_cam"]
    eye = tuple(float(v) for v in pose["eye_xyz_m"])
    forward = tuple(float(v) for v in pose["forward_xyz"])
    half_fov = math.radians(float(manifest["camera"]["horizontal_fov_deg"])) / 2.0

    trajectory = trajectory_from_manifest(entry["delivery_trajectory"])
    approach = float(entry["delivery_trajectory"]["approach_length_m"])

    rows = []
    for fraction in STATION_FRACTIONS:
        station_s = approach * fraction
        at = trajectory.pose_at(station_s)
        target = (at.x_m, at.y_m, A_BODY_CENTRE_HEIGHT_M)

        ray = tuple(t - e for t, e in zip(target, eye, strict=True))
        distance = math.sqrt(sum(c * c for c in ray))
        unit = tuple(c / distance for c in ray)
        # Bearing off the optical axis, in 3-D: the frustum is declared
        # horizontally, so this is the conservative reading of it.
        cosine = max(-1.0, min(1.0, sum(u * f for u, f in zip(unit, forward, strict=True))))
        bearing_rad = math.acos(cosine)

        line_of_sight = not _blocked(eye, target, triangles)
        in_frustum = bearing_rad <= half_fov
        rows.append({
            "station_s_m": round(station_s, 4),
            "a_xy_m": [round(at.x_m, 4), round(at.y_m, 4)],
            "distance_m": round(distance, 4),
            "bearing_off_axis_deg": round(math.degrees(bearing_rad), 3),
            "line_of_sight": line_of_sight,
            "in_frustum": in_frustum,
            "usable": line_of_sight and in_frustum,
        })

    usable = [row for row in rows if row["usable"]]
    return {
        "profile": profile,
        "stage": str(stage_path),
        "camera_prim_pose_source": "manifest profiles.<profile>.p_cam",
        "eye_xyz_m": [round(v, 4) for v in eye],
        "forward_xyz": [round(v, 6) for v in forward],
        "horizontal_fov_deg": float(manifest["camera"]["horizontal_fov_deg"]),
        "a_body_centre_height_m": A_BODY_CENTRE_HEIGHT_M,
        "opaque_prims": len(prims),
        "opaque_triangles": len(triangles),
        "stations": rows,
        "usable_stations": f"{len(usable)}/{len(rows)}",
        "distance_range_m": (
            [min(r["distance_m"] for r in usable), max(r["distance_m"] for r in usable)]
            if usable else None
        ),
        "pass": len(usable) == len(rows),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--stage", default="out/corridor.usda")
    parser.add_argument("--manifest", default="out/corridor.manifest.json")
    parser.add_argument("--profile", default=None)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    manifest = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    profile = args.profile or str(manifest["selected_profile"])
    report = evaluate(Path(args.stage), manifest, profile)

    destination = Path(args.out)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(f"profile {profile}: {report['usable_stations']} usable, "
          f"{report['opaque_triangles']} opaque triangles")
    for row in report["stations"]:
        print(f"  s={row['station_s_m']:>6.3f} m  d={row['distance_m']:>5.2f} m  "
              f"bearing {row['bearing_off_axis_deg']:>5.1f} deg  "
              f"los={'y' if row['line_of_sight'] else 'n'} "
              f"frustum={'y' if row['in_frustum'] else 'n'}")
    print(f"written: {destination}")
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
