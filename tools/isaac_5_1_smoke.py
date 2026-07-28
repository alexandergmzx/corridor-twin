#!/usr/bin/env python3
"""Load corridor USDA in the locally verified Isaac Sim 5.1 API surface."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from isaac_gpu import gpu_memory_snapshot

# The application must start before importing omni, pxr, or Isaac extensions.
from isaacsim import SimulationApp


def _manifest_walls(stage_path: Path) -> set[str]:
    """Return every wall the manifest declares for the selected profile.

    The smoke test used to hardcode four building names in two places. When
    ADR 0018 added the east-wall stub those lists did not fail -- they silently
    stopped covering it, which is the failure mode a hardcoded enumeration
    always has. Reading the manifest means a new wall is checked the moment it
    is authored.
    """

    import json

    manifest = json.loads(
        stage_path.with_suffix(".manifest.json").read_text(encoding="utf-8")
    )
    profile = manifest["profiles"][manifest["selected_profile"]]
    return set(profile["walls"])


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", type=Path)
    parser.add_argument("--updates", type=int, default=10)
    parser.add_argument(
        "--gui",
        action="store_true",
        help="Open a visible viewport while running the same stage validation.",
    )
    parser.add_argument(
        "--report-gpu-memory",
        action="store_true",
        help="Report a loaded-stage nvidia-smi memory snapshot before shutdown.",
    )
    return parser.parse_args()


def main() -> int:
    args = arguments()
    stage_path = args.stage.resolve()
    if not stage_path.is_file():
        raise FileNotFoundError(stage_path)
    # SimulationApp forwards unconsumed argv to Kit. Keep project arguments out
    # of Kit's command line after argparse has handled them.
    sys.argv = [sys.argv[0]]
    print("ISAAC_SMOKE_START", f"stage={stage_path}", flush=True)
    app = SimulationApp(
        {
            "headless": not args.gui,
            "width": 640,
            "height": 360,
            "renderer": "RaytracedLighting",
            "anti_aliasing": 2,
            "create_new_stage": False,
            "disable_viewport_updates": not args.gui,
            "fast_shutdown": True,
            "open_usd": str(stage_path),
        }
    )
    print("ISAAC_SMOKE_APP_READY", flush=True)
    try:
        # SimulationApp.open_usd is part of the installed 5.1.0 API and avoids
        # a synchronous context-open deadlock when viewport updates are disabled.
        import omni.usd
        from pxr import UsdGeom, UsdPhysics

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError(f"Isaac Sim did not open {stage_path}")
        print(
            "ISAAC_SMOKE_STAGE",
            f"root_layer={stage.GetRootLayer().identifier}",
            flush=True,
        )
        for _ in range(args.updates):
            app.update()
        print("ISAAC_SMOKE_UPDATED", f"updates={args.updates}", flush=True)

        # Derived from the manifest rather than listed here. Two hardcoded
        # tuples used to enumerate four buildings; when ADR 0018 added the
        # east-wall stub they did not fail, they silently stopped covering it.
        wall_names = sorted(_manifest_walls(stage_path))
        required = (
            "/World/PhysicsScene",
            "/World/Environment/Ground",
            *[f"/World/Environment/Corridor/{name}" for name in wall_names],
            "/World/Actors/A/CameraMount/FrontCamera",
        )
        missing = [path for path in required if not stage.GetPrimAtPath(path)]
        if missing:
            raise RuntimeError(f"missing prims after Isaac composition: {missing}")
        print("ISAAC_SMOKE_PRIMS_OK", flush=True)
        camera_paths = [
            prim.GetPath().pathString for prim in stage.Traverse() if prim.IsA(UsdGeom.Camera)
        ]
        sensor_path = "/World/Actors/A/CameraMount/FrontCamera"
        authored_camera_count = camera_paths.count(sensor_path)
        if authored_camera_count != 1:
            raise RuntimeError(f"expected authored camera {sensor_path}; found {camera_paths}")
        print(
            "ISAAC_SMOKE_CAMERA_OK",
            f"authored_cameras={authored_camera_count}",
            f"context_cameras={','.join(camera_paths)}",
            flush=True,
        )
        for name in wall_names:
            prim = stage.GetPrimAtPath(f"/World/Environment/Corridor/{name}")
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                raise RuntimeError(f"{name} lost its collision schema")
        # The variant set sits on the default prim: a variant only contributes
        # opinions inside its own namespace, and the actors move with (m,n).
        world = stage.GetPrimAtPath("/World")
        variants = world.GetVariantSets().GetVariantSet("corridorProfile")
        if args.report_gpu_memory:
            gpu_name, used_mib, total_mib = gpu_memory_snapshot()
            print(
                "ISAAC_SMOKE_GPU",
                f"name={gpu_name}",
                f"used_mib={used_mib}",
                f"total_mib={total_mib}",
                flush=True,
            )
        print(
            "ISAAC_SMOKE_PASS",
            f"stage={stage_path}",
            f"root_layer={stage.GetRootLayer().identifier}",
            f"profile={variants.GetVariantSelection()}",
            f"variants={','.join(variants.GetVariantNames())}",
            f"cameras={authored_camera_count}",
            f"updates={args.updates}",
            flush=True,
        )
    except Exception as exc:
        # With fast_shutdown enabled, app.close() terminates Kit with success.
        # Exit directly here so a failed validation remains machine-detectable.
        print(
            "ISAAC_SMOKE_FAIL",
            f"error={type(exc).__name__}: {exc}",
            file=sys.stderr,
            flush=True,
        )
        os._exit(1)
    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
