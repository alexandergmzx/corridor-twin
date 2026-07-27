#!/usr/bin/env python3
"""Load corridor USDA in the locally verified Isaac Sim 5.1 API surface."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

# The application must start before importing omni, pxr, or Isaac extensions.
from isaacsim import SimulationApp


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


def gpu_memory_snapshot() -> tuple[str, int, int]:
    output = subprocess.check_output(
        [
            "nvidia-smi",
            "--query-gpu=name,memory.used,memory.total",
            "--format=csv,noheader,nounits",
            "--id=0",
        ],
        text=True,
    ).strip()
    name, used_mib, total_mib = (item.strip() for item in output.split(","))
    return name, int(used_mib), int(total_mib)


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

        required = (
            "/World/PhysicsScene",
            "/World/Environment/Ground",
            "/World/Environment/Corridor/LeftBuilding",
            "/World/Environment/Corridor/RightBuilding",
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
        for name in ("LeftBuilding", "RightBuilding"):
            prim = stage.GetPrimAtPath(f"/World/Environment/Corridor/{name}")
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                raise RuntimeError(f"{name} lost its collision schema")
        corridor = stage.GetPrimAtPath("/World/Environment/Corridor")
        variants = corridor.GetVariantSets().GetVariantSet("corridorProfile")
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
