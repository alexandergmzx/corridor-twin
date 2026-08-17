#!/usr/bin/env python3
"""Export P's surveyed camera pose from the stage, in the optical convention.

    .venv/bin/python tools/export_camera_pose.py --stage out/corridor.usda \\
        --out out/p_cam_pose.json

**Why a file and not an import.** `pxr` lives in the system 3.12 venv and
`torch` lives in Isaac's 3.11; sourcing either into the other breaks the ABI
(CLAUDE.md, "Environment discipline"). The two halves meet over an artifact,
the same way `export_scan_walls.py` hands the scan filter its wall model.

**This is survey, not simulator truth.** Where P bolted its own camera is
something P knows by construction -- a real roadside installation is surveyed
the same way, and the enforcement pipeline is entitled to it. What P is not
entitled to is where *A* is, which is the thing being measured.

USD's camera convention is +Y up and the view down **-Z**. The optical
convention every ROS consumer expects is +X right, +Y down, +Z forward. They
differ by a 180-degree flip about X, and getting it wrong does not fail: it
back-projects every ray behind the camera, the ground intersection is refused
for every frame, and the pipeline reports zero coverage with no error.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from pxr import Usd, UsdGeom

DEFAULT_CAMERA_PRIM = "/World/Actors/PCameraMast/PCam"


def optical_pose(stage_path: Path, prim_path: str, profile: str | None = None):
    """-> (position, world_from_optical 3x3 as row-major lists)."""

    stage = Usd.Stage.Open(str(stage_path))
    if stage is None:
        raise SystemExit(f"could not open {stage_path}")
    if profile:
        variants = stage.GetPrimAtPath("/World").GetVariantSet("corridor_profile")
        if variants and profile in variants.GetVariantNames():
            variants.SetVariantSelection(profile)

    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        raise SystemExit(f"{prim_path} absent from {stage_path}")
    if not UsdGeom.Camera(prim):
        raise SystemExit(f"{prim_path} is not a UsdGeom.Camera")

    matrix = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    position = [float(v) for v in matrix.ExtractTranslation()]
    # Row-vector convention: the ROWS are the local axes in world coordinates.
    local_x = [float(matrix[0][i]) for i in range(3)]
    local_y = [float(matrix[1][i]) for i in range(3)]
    local_z = [float(matrix[2][i]) for i in range(3)]

    right = local_x
    down = [-v for v in local_y]
    forward = [-v for v in local_z]

    # Columns are the optical basis expressed in world, so R @ ray_optical is
    # the ray in world.
    rotation = [[right[r], down[r], forward[r]] for r in range(3)]
    return position, rotation, forward


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", type=Path, required=True)
    ap.add_argument("--prim", default=DEFAULT_CAMERA_PRIM)
    ap.add_argument("--profile")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)

    position, rotation, forward = optical_pose(args.stage, args.prim, args.profile)

    if position[2] <= 0.0:
        raise SystemExit(f"camera at z={position[2]}; it cannot see a ground plane")
    if forward[2] >= 0.0:
        # Every ground intersection would be refused, silently, on every frame.
        raise SystemExit(
            f"camera forward axis points up ({forward}); the optical convention "
            f"is wrong or the mast is aimed at the sky")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({
        "stage": str(args.stage), "prim": args.prim, "profile": args.profile,
        "convention": "optical: +x right, +y down, +z forward",
        "position_xyz_m": position,
        "world_from_optical": rotation,
        "forward_xyz": forward,
    }, indent=2) + "\n", encoding="utf-8")
    print(f"{args.prim} at {[round(v, 4) for v in position]} "
          f"looking {[round(v, 4) for v in forward]} -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
