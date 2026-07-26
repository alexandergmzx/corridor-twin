from __future__ import annotations

import json
from pathlib import Path

import pytest
from pxr import Usd, UsdGeom, UsdPhysics
from scene.build import build_scene
from scene.occlusion import continuous_certificate, verify


@pytest.fixture()
def generated(tmp_path: Path) -> tuple[Path, Path]:
    return build_scene(None, tmp_path / "corridor.usda", 6.0, 3.0)


def _inner_width(stage: Usd.Stage, station: float) -> float:
    values: dict[str, float] = {}
    for building in ("LeftBuilding", "RightBuilding"):
        mesh = UsdGeom.Mesh(stage.GetPrimAtPath(f"/World/Environment/Corridor/{building}"))
        points = mesh.GetPointsAttr().Get()
        candidates = [float(point[1]) for point in points if abs(float(point[0]) - station) < 1e-6]
        if building == "LeftBuilding":
            values[building] = min(candidates)
        else:
            values[building] = max(candidates)
    return values["LeftBuilding"] - values["RightBuilding"]


def test_stage_contract_and_every_variant_width(generated: tuple[Path, Path]) -> None:
    stage_path, _ = generated
    stage = Usd.Stage.Open(str(stage_path))
    assert stage.GetDefaultPrim().GetPath().pathString == "/World"
    assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z
    assert UsdGeom.GetStageMetersPerUnit(stage) == pytest.approx(1.0)
    required = [
        "/World/PhysicsScene",
        "/World/Environment/Ground",
        "/World/Environment/Corridor/LeftBuilding",
        "/World/Environment/Corridor/RightBuilding",
        "/World/Actors/A/CameraMount/FrontCamera",
        "/World/Actors/B",
        "/World/Actors/P",
        "/World/Paths/DeliveryPath",
    ]
    assert all(stage.GetPrimAtPath(path) for path in required)

    corridor = stage.GetPrimAtPath("/World/Environment/Corridor")
    variants = corridor.GetVariantSets().GetVariantSet("corridorProfile")
    assert len(variants.GetVariantNames()) >= 3
    for name in variants.GetVariantNames():
        assert variants.SetVariantSelection(name)
        entry = corridor.GetAttribute("corridor:entryWidthM").Get()
        corner = corridor.GetAttribute("corridor:cornerWidthM").Get()
        assert _inner_width(stage, 0.0) == pytest.approx(entry, abs=1e-6)
        assert _inner_width(stage, 12.0) == pytest.approx(corner, abs=1e-6)
        for building in ("LeftBuilding", "RightBuilding"):
            prim = stage.GetPrimAtPath(f"/World/Environment/Corridor/{building}")
            assert prim.HasAPI(UsdPhysics.CollisionAPI)
            assert UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
            assert prim.HasAPI(UsdPhysics.MeshCollisionAPI)
            assert UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() == "convexHull"

    ground = stage.GetPrimAtPath("/World/Environment/Ground")
    assert ground.HasAPI(UsdPhysics.CollisionAPI)
    cameras = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Camera)]
    assert len(cameras) == 1


def test_requested_dimensions_become_selected_variant(tmp_path: Path) -> None:
    stage_path, manifest_path = build_scene(None, tmp_path / "custom.usda", 5.5, 3.2)
    stage = Usd.Stage.Open(str(stage_path))
    corridor = stage.GetPrimAtPath("/World/Environment/Corridor")
    variants = corridor.GetVariantSets().GetVariantSet("corridorProfile")
    assert variants.GetVariantSelection() == "requested_m5_5_n3_2"
    assert _inner_width(stage, 0.0) == pytest.approx(5.5, abs=1e-6)
    assert _inner_width(stage, 12.0) == pytest.approx(3.2, abs=1e-6)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["selected_profile"] == variants.GetVariantSelection()


def test_output_is_readable_usda_and_has_marker_assets(generated: tuple[Path, Path]) -> None:
    stage_path, manifest_path = generated
    assert stage_path.read_text(encoding="utf-8").startswith("#usda 1.0")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    marker_ids = {
        marker["id"] for profile in manifest["profiles"].values() for marker in profile["markers"]
    }
    assert marker_ids
    assert all(
        (stage_path.parent / "markers" / f"marker_{i:03d}.png").is_file() for i in marker_ids
    )


def test_continuous_occlusion_and_composed_mesh_audit(generated: tuple[Path, Path]) -> None:
    stage_path, manifest_path = generated
    result = verify(stage_path, manifest_path)
    assert result.passed
    assert not result.uncertified_intervals
    assert {item.method for item in result.coverage} == {"south_wall", "frustum_excluded"}
    assert result.usd_audit_rays > 0
    assert result.usd_audit_failures == 0


def test_visible_negative_control_fails(generated: tuple[Path, Path]) -> None:
    _, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    # Put P inside the clear corridor and in front of A: no wall or frustum plane
    # can certify this deliberately visible control.
    manifest["actors"]["p_bounds_min_xyz_m"] = [4.8, -0.2, 0.2]
    manifest["actors"]["p_bounds_max_xyz_m"] = [5.2, 0.2, 1.4]
    result = continuous_certificate(manifest, manifest["selected_profile"])
    assert not result.passed
    assert result.uncertified_intervals
