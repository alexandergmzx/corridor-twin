#!/usr/bin/env python3
"""Render P's enforcement dataset: robot A, seen from P's mast, labelled.

    ~/isaac/env_isaaclab/bin/python tools/replicator_p_cam_dataset.py \
        --profile nominal_m6_n3 --frames 20 --out out/datasets/p_cam_smoke

Runs under ISAAC's Python (3.11), never the system venv. Holds the Isaac lock
like any other GPU session.

WHAT THIS IS
------------
The generator for [docs/DATASET-SPEC.md]. Every count, split, range and label
in that document is read from `dataset_spec.py` here, so the specification and
the artifact cannot drift into two descriptions.

THE RENDER-PRODUCT BUDGET
-------------------------
CLAUDE.md invariant 3 permits exactly one render product, and the ADR 0009
adapter enforces it at runtime. **That law governs the demonstration scene.**
This is an offline authoring tool: it runs in its own process, writes a dataset
to disk and exits, and nothing it creates exists while the demonstration runs.

It attaches TWO products, both to the SAME camera prim, because the paired
resolution comparison ADR 0024 decision 5 asks for needs identical scene state
rather than a shared seed. Stated in the spec, and stated again here.

WHY THE ARENA AND NOT THE AUTHORED STAGE
----------------------------------------
The subject has to be the robot the detector will actually see. `corridor.usda`
carries A as a v1 stand-in cube; the arena carries the yahboom twin at
`/World/Robot`, which IS A (ADR 0027). A detector trained on the cube would
learn a box.

Physics is deliberately never played. A is placed by writing its root transform
and the frame is rendered from that pose, so the dataset samples the route
envelope uniformly instead of wherever a drive happened to go.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import random
import sys
from pathlib import Path

ROOT = Path(os.path.abspath(__file__)).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from dataset_spec import (  # noqa: E402
    DOME_INTENSITY,
    DOME_TEMPERATURE_K,
    FRAMES_PER_PROFILE,
    KEY_LIGHT_YAW_DEG,
    LATERAL_FRACTION,
    PROFILES,
    RESOLUTIONS,
    TRAIN_FRACTION,
    YAW_JITTER_DEG,
)

#: The one camera. Same prim the ADR 0009 adapter targets and the occlusion
#: certificate certifies, so all three consumers agree about where P looks.
P_CAM_PRIM = "/World/Actors/PCameraMast/PCam"

#: A, in the arena: the yahboom twin, not the v1 stand-in cube.
ROBOT_PRIM = "/World/Robot"

#: The only labelled object in the scene. A box is then an unambiguous claim.
A_LABEL = "robot_a"


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 16), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sample_pose(rng: random.Random, trajectory, route_m: float, half_width_m: float):
    """One draw from the route envelope: station, lateral offset, yaw jitter."""

    station = rng.uniform(0.0, route_m)
    at = trajectory.pose_at(station)
    lateral = rng.uniform(-LATERAL_FRACTION, LATERAL_FRACTION) * half_width_m
    # Offset perpendicular to the route tangent, so "lateral" means lateral to
    # the corridor rather than to world Y.
    normal = (-math.sin(at.yaw_rad), math.cos(at.yaw_rad))
    yaw = at.yaw_rad + math.radians(rng.uniform(-YAW_JITTER_DEG, YAW_JITTER_DEG))
    return {
        "station_m": round(station, 5),
        "x_m": round(at.x_m + normal[0] * lateral, 5),
        "y_m": round(at.y_m + normal[1] * lateral, 5),
        "yaw_rad": round(yaw, 6),
        "lateral_offset_m": round(lateral, 5),
        "route_yaw_rad": round(at.yaw_rad, 6),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--profile", action="append", choices=PROFILES, default=None,
                        help="repeatable; defaults to all three")
    parser.add_argument("--frames", type=int, default=None,
                        help="frames per profile; defaults to the spec's count")
    parser.add_argument("--manifest", default="out/corridor.manifest.json")
    parser.add_argument("--arena-dir", default="out")
    parser.add_argument("--out", default="out/datasets/p_cam_v1")
    parser.add_argument("--seed", type=int, default=20260813)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--overlays", type=int, default=0,
                        help="write N label-overlay PNGs for the acceptance check")
    args = parser.parse_args()

    profiles = args.profile or list(PROFILES)
    frames_each = args.frames or FRAMES_PER_PROFILE
    out_root = Path(args.out)

    manifest = json.loads(Path(args.manifest).read_text(encoding="utf-8"))

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": not args.gui})
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.usd
        from PIL import Image, ImageDraw
        from pxr import Gf, UsdGeom

        try:
            from isaacsim.core.utils.semantics import add_labels
        except ImportError:  # pre-5.x spelling, kept as a fallback only
            from isaacsim.core.utils.semantics import (
                add_update_semantics as _legacy,
            )

            def add_labels(prim, labels, instance_name="class"):
                _legacy(prim, labels[0], instance_name)

        sys.path.insert(0, str(ROOT / "src" / "corridor_scene"))
        from scene.trajectory import trajectory_from_manifest

        out_root.mkdir(parents=True, exist_ok=True)
        index_path = out_root / "dataset.json"
        records: list[dict] = []
        overlays_written = 0

        for profile in profiles:
            arena = Path(args.arena_dir) / f"arena_corridor_robot1_{profile}.usd"
            if not arena.is_file():
                print(f"SKIP {profile}: no arena at {arena}")
                continue
            print(f"=== {profile}: {arena}")

            if not omni.usd.get_context().open_stage(str(arena)):
                print(f"FAIL: could not open {arena}")
                return 1
            stage = omni.usd.get_context().get_stage()

            camera = stage.GetPrimAtPath(P_CAM_PRIM)
            if not camera:
                print(f"FAIL: {P_CAM_PRIM} absent -- is this arena pre-ADR-0031?")
                return 1
            robot = stage.GetPrimAtPath(ROBOT_PRIM)
            if not robot:
                print(f"FAIL: {ROBOT_PRIM} absent from {arena}")
                return 1

            # A is the ONLY labelled object, so a box is unambiguous.
            add_labels(robot, [A_LABEL], "class")

            entry = manifest["profiles"][profile]
            legs = entry["delivery_trajectory"]
            trajectory = trajectory_from_manifest(legs)
            route_m = (
                float(legs["approach_length_m"])
                + float(legs["arc_radius_m"]) * float(legs["arc_sweep_rad"])
                + float(legs["delivery_arc_radius_m"]) * float(legs["delivery_arc_sweep_rad"])
                + float(legs["delivery_length_m"])
            )
            half_width = float(entry["corner_width_m"]) / 2.0

            # TWO products, ONE camera: identical scene state at both
            # resolutions, which is what makes the pair a pair.
            products = {
                name: rep.create.render_product(P_CAM_PRIM, (w, h))
                for name, (w, h) in RESOLUTIONS.items()
            }
            annotators = {}
            for name, product in products.items():
                rgb = rep.AnnotatorRegistry.get_annotator("rgb")
                box = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
                rgb.attach(product)
                box.attach(product)
                annotators[name] = (rgb, box)

            xform = UsdGeom.Xformable(robot)
            rng = random.Random(f"{args.seed}:{profile}")

            for index in range(frames_each):
                pose = sample_pose(rng, trajectory, route_m, half_width)
                lighting = {
                    "dome_intensity": round(rng.uniform(*DOME_INTENSITY), 1),
                    "dome_temperature_k": round(rng.uniform(*DOME_TEMPERATURE_K), 1),
                    "key_light_yaw_deg": round(rng.uniform(*KEY_LIGHT_YAW_DEG), 1),
                }

                xform.ClearXformOpOrder()
                xform.AddTranslateOp().Set(Gf.Vec3d(pose["x_m"], pose["y_m"], 0.0))
                xform.AddRotateZOp().Set(math.degrees(pose["yaw_rad"]))

                for light in stage.Traverse():
                    if light.GetTypeName() == "DomeLight":
                        light.GetAttribute("inputs:intensity").Set(
                            lighting["dome_intensity"]
                        )
                        temperature = light.GetAttribute("inputs:colorTemperature")
                        if temperature:
                            temperature.Set(lighting["dome_temperature_k"])

                rep.orchestrator.step(rt_subframes=4)

                split = "train" if rng.random() < TRAIN_FRACTION else "val"
                paired_ok = True
                per_resolution = {}
                for name, (rgb, box) in annotators.items():
                    image = rgb.get_data()
                    boxes = box.get_data()
                    array = np.asarray(image)[:, :, :3]

                    directory = out_root / name / split
                    directory.mkdir(parents=True, exist_ok=True)
                    stem = f"{profile}_{index:05d}"
                    image_path = directory / f"rgb_{stem}.png"
                    Image.fromarray(array).save(image_path)

                    data = boxes["data"] if isinstance(boxes, dict) else boxes
                    found = [
                        {
                            "x_min": int(row["x_min"]), "y_min": int(row["y_min"]),
                            "x_max": int(row["x_max"]), "y_max": int(row["y_max"]),
                        }
                        for row in data
                    ]
                    (directory / f"bbox_{stem}.json").write_text(
                        json.dumps({"label": A_LABEL, "boxes": found}, indent=2) + "\n",
                        encoding="utf-8",
                    )
                    per_resolution[name] = {
                        "image": str(image_path.relative_to(out_root)),
                        "sha256": sha256_of(image_path),
                        "boxes": found,
                    }
                    if not found:
                        paired_ok = False

                    if overlays_written < args.overlays and name == "lo":
                        overlay_dir = out_root / "overlays"
                        overlay_dir.mkdir(parents=True, exist_ok=True)
                        picture = Image.fromarray(array).convert("RGB")
                        draw = ImageDraw.Draw(picture)
                        for entry_box in found:
                            draw.rectangle(
                                [entry_box["x_min"], entry_box["y_min"],
                                 entry_box["x_max"], entry_box["y_max"]],
                                outline=(255, 93, 143), width=2,
                            )
                        picture.save(overlay_dir / f"overlay_{stem}.png")
                        overlays_written += 1

                records.append({
                    "profile": profile,
                    "index": index,
                    "split": split,
                    "pose": pose,
                    "lighting": lighting,
                    "camera_eye_xyz_m": entry["p_cam"]["eye_xyz_m"],
                    "range_m": round(math.dist(
                        (pose["x_m"], pose["y_m"]),
                        (entry["p_cam"]["eye_xyz_m"][0], entry["p_cam"]["eye_xyz_m"][1]),
                    ), 4),
                    "resolutions": per_resolution,
                    "a_visible_in_both": paired_ok,
                })

                # CHECKPOINT EVERY FRAME. A run interrupted at any point leaves
                # a usable, self-describing partial dataset.
                index_path.write_text(
                    json.dumps({
                        "spec": "docs/DATASET-SPEC.md",
                        "camera_prim": P_CAM_PRIM,
                        "resolutions": RESOLUTIONS,
                        "seed": args.seed,
                        "frames_per_profile_requested": frames_each,
                        "frames": records,
                    }, indent=2) + "\n",
                    encoding="utf-8",
                )
                if (index + 1) % 25 == 0:
                    visible = sum(1 for r in records if r["a_visible_in_both"])
                    print(f"  {profile} {index + 1}/{frames_each} "
                          f"({visible}/{len(records)} with A in both)")

            for rgb, box in annotators.values():
                rgb.detach()
                box.detach()

        visible = sum(1 for r in records if r["a_visible_in_both"])
        print(f"\n{len(records)} frames, {visible} with A boxed at both resolutions")
        print(f"written: {index_path}")
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
