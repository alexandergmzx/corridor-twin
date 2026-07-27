from __future__ import annotations

import json
from pathlib import Path

import pytest
from pxr import Usd, UsdGeom, UsdPhysics
from scene.build import build_scene
from scene.geometry import corridor_faces, is_clear, police_bounds
from scene.model import load_scenario
from scene.occlusion import verify

BUILDINGS = ("NorthBuilding", "SouthBuilding", "CornerBuilding", "EastBuilding")


@pytest.fixture()
def generated(tmp_path: Path) -> tuple[Path, Path]:
    return build_scene(None, tmp_path / "corridor.usda", 6.0, 3.0)


def _points(stage: Usd.Stage, building: str) -> list[tuple[float, float]]:
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(f"/World/Environment/Corridor/{building}"))
    return [(float(point[0]), float(point[1])) for point in mesh.GetPointsAttr().Get()]


def _north_face(stage: Usd.Stage) -> float:
    """The north inner face is straight, so it has no vertex per station."""

    return min(y for _, y in _points(stage, "NorthBuilding"))


def _south_face(stage: Usd.Stage, station: float) -> float:
    """Measure the sloping south inner face at one of its authored stations."""

    candidates = [y for x, y in _points(stage, "SouthBuilding") if abs(x - station) < 1e-6]
    assert candidates, f"SouthBuilding has no vertex at station {station}"
    return max(candidates)


def _inner_width(stage: Usd.Stage, station: float) -> float:
    return _north_face(stage) - _south_face(stage, station)


def _variants(stage: Usd.Stage) -> Usd.VariantSet:
    return stage.GetPrimAtPath("/World").GetVariantSets().GetVariantSet("corridorProfile")


def test_stage_contract_and_every_variant_width(generated: tuple[Path, Path]) -> None:
    stage_path, _ = generated
    stage = Usd.Stage.Open(str(stage_path))
    assert stage.GetDefaultPrim().GetPath().pathString == "/World"
    assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z
    assert UsdGeom.GetStageMetersPerUnit(stage) == pytest.approx(1.0)
    required = [
        "/World/PhysicsScene",
        "/World/Environment/Ground",
        *[f"/World/Environment/Corridor/{name}" for name in BUILDINGS],
        "/World/Actors/A/CameraMount/FrontCamera",
        "/World/Actors/B",
        "/World/Actors/P",
        "/World/Paths/DeliveryPath",
    ]
    assert all(stage.GetPrimAtPath(path) for path in required)

    corridor = stage.GetPrimAtPath("/World/Environment/Corridor")
    variants = _variants(stage)
    assert len(variants.GetVariantNames()) >= 3
    for name in variants.GetVariantNames():
        assert variants.SetVariantSelection(name)
        entry = corridor.GetAttribute("corridor:entryWidthM").Get()
        corner = corridor.GetAttribute("corridor:cornerWidthM").Get()
        assert _inner_width(stage, 0.0) == pytest.approx(entry, abs=1e-6)
        assert _inner_width(stage, 12.0) == pytest.approx(corner, abs=1e-6)

    ground = stage.GetPrimAtPath("/World/Environment/Ground")
    assert ground.HasAPI(UsdPhysics.CollisionAPI)
    cameras = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Camera)]
    assert len(cameras) == 1


def test_taper_is_one_sided_with_a_straight_north_face(generated: tuple[Path, Path]) -> None:
    """The supplied diagram draws one straight face and one sloping face."""

    stage_path, _ = generated
    stage = Usd.Stage.Open(str(stage_path))
    variants = _variants(stage)
    for name in variants.GetVariantNames():
        variants.SetVariantSelection(name)
        # Every north-wall vertex sits on one straight line of constant Y.
        north_ys = {round(y, 9) for _, y in _points(stage, "NorthBuilding")}
        assert len(north_ys) == 2  # inner face and outer face only

        prim = stage.GetPrimAtPath("/World/Environment/Corridor")
        entry_width = prim.GetAttribute("corridor:entryWidthM").Get()
        corner_width = prim.GetAttribute("corridor:cornerWidthM").Get()
        entry_south = _south_face(stage, 0.0)
        corner_south = _south_face(stage, 12.0)
        if entry_width > corner_width:
            # The whole taper is carried by the south face rising toward north.
            assert corner_south > entry_south
            assert corner_south - entry_south == pytest.approx(
                entry_width - corner_width, abs=1e-9
            )


def test_every_flanking_building_is_a_static_collider(generated: tuple[Path, Path]) -> None:
    stage_path, _ = generated
    stage = Usd.Stage.Open(str(stage_path))
    variants = _variants(stage)
    for name in variants.GetVariantNames():
        variants.SetVariantSelection(name)
        for building in BUILDINGS:
            prim = stage.GetPrimAtPath(f"/World/Environment/Corridor/{building}")
            assert prim, f"{building} missing in variant {name}"
            assert prim.HasAPI(UsdPhysics.CollisionAPI)
            assert UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
            assert prim.HasAPI(UsdPhysics.MeshCollisionAPI)
            assert UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() == "convexHull"


def test_actors_follow_the_selected_profile(generated: tuple[Path, Path]) -> None:
    """P must move with (m,n); a frozen P would end up in a wall or the road."""

    stage_path, _ = generated
    stage = Usd.Stage.Open(str(stage_path))
    variants = _variants(stage)
    cache = UsdGeom.XformCache()
    seen: set[float] = set()
    for name in variants.GetVariantNames():
        variants.SetVariantSelection(name)
        cache.Clear()
        police = cache.GetLocalToWorldTransform(
            stage.GetPrimAtPath("/World/Actors/P")
        ).ExtractTranslation()
        seen.add(round(float(police[1]), 6))
    assert len(seen) == len(variants.GetVariantNames())


def test_actor_topology_matches_the_supplied_diagram() -> None:
    """A approaches down the corridor, B is along the next street, P is outside both."""

    scenario = load_scenario()
    length = scenario.corridor_length_m
    for profile in scenario.profiles:
        police_min, police_max = police_bounds(scenario, profile)
        # P stays west of the corner mass that hides it.
        assert police_max[0] < length - scenario.wall_thickness_m
        # P stays south of the corridor's south wall, outside the road.
        for corner_x in (police_min[0], police_max[0]):
            south_face = corridor_faces(profile, corner_x, length)[1]
            assert police_max[1] < south_face - scenario.wall_thickness_m
        # P's body enters no drivable space at any footprint corner.
        for corner_x in (police_min[0], police_max[0]):
            for corner_y in (police_min[1], police_max[1]):
                assert not is_clear(scenario, profile, corner_x, corner_y)
        # B stands down the next street, past the corner.
        b_y = -scenario.next_street.b_distance_m
        assert is_clear(scenario, profile, scenario.street_center_x_m, b_y)
        assert b_y < corridor_faces(profile, length, length)[1]


def test_requested_dimensions_become_selected_variant(tmp_path: Path) -> None:
    stage_path, manifest_path = build_scene(None, tmp_path / "custom.usda", 5.5, 3.2)
    stage = Usd.Stage.Open(str(stage_path))
    variants = _variants(stage)
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


def test_markers_sit_on_the_actual_wall_faces(generated: tuple[Path, Path]) -> None:
    """Marker survey must come from the shared faces, not a symmetric guess."""

    _, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    scenario = load_scenario()
    for name, block in manifest["profiles"].items():
        profile = next(p for p in scenario.profiles if p.name == name)
        for marker in block["markers"]:
            north, south = corridor_faces(profile, marker["station_m"], scenario.corridor_length_m)
            face = north if marker["side"] == "north" else south
            centre_y = sum(corner[1] for corner in marker["corners_xyz_m"]) / 4.0
            # Plates are canted and stand slightly off the wall.
            assert abs(centre_y - face) < 0.05


def test_walls_manifest_and_checker_share_one_geometry_source(
    generated: tuple[Path, Path],
) -> None:
    """Composed USD, manifest occluders, and corridor_faces() must agree.

    Duplicating the taper equation across the author, the manifest, and the
    checker is exactly what let an earlier visibility proof assume a symmetric
    corridor, so pin the three against each other.
    """

    stage_path, manifest_path = generated
    stage = Usd.Stage.Open(str(stage_path))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    scenario = load_scenario()
    length = scenario.corridor_length_m
    variants = _variants(stage)

    for name, block in manifest["profiles"].items():
        variants.SetVariantSelection(name)
        profile = next(p for p in scenario.profiles if p.name == name)
        for station in (0.0, length):
            north, south = corridor_faces(profile, station, length)
            assert _north_face(stage) == pytest.approx(north, abs=1e-9)
            assert _south_face(stage, station) == pytest.approx(south, abs=1e-9)

        slab = next(
            item
            for item in block["occluders"]
            if item["prim_path"].endswith("SouthBuilding")
        )
        for station in (0.0, length / 2.0, length):
            expected = corridor_faces(profile, station, length)[1]
            top = slab["y_high_intercept"] + slab["y_high_slope"] * station
            assert top == pytest.approx(expected, abs=1e-9)
            bottom = slab["y_low_intercept"] + slab["y_low_slope"] * station
            assert top - bottom == pytest.approx(scenario.wall_thickness_m, abs=1e-9)


def test_manifest_records_diagram_provenance(generated: tuple[Path, Path]) -> None:
    _, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    provenance = manifest["provenance"]
    assert provenance["source_document"] == "docs/ROBO_TASK.pdf"
    assert provenance["topology"] == "reconciled_with_supplied_diagram"
    # The drawing carries no scale bar, so metric lengths must not be presented
    # as recovered survey values.
    assert provenance["metric_scale"] == "demo_assumption"


def test_camera_cannot_see_police_anywhere_on_the_route(generated: tuple[Path, Path]) -> None:
    stage_path, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for name in manifest["profiles"]:
        result = verify(stage_path, manifest_path, name)
        assert result.passed, f"profile {name}: {result.camera_visible_intervals}"
        assert result.camera_visible_intervals == ()
        # The stronger, reciprocal claim: an opaque wall does the hiding.
        assert result.line_of_sight_blocked_everywhere
        assert result.frustum_only_intervals == ()
        assert result.usd_audit_rays > 0
        assert result.usd_audit_failures == 0
        assert result.nearest_blocking_distance_m is not None


def test_certificate_names_the_blocking_prim(generated: tuple[Path, Path]) -> None:
    """The interview overlay quotes this, so it must be real evidence."""

    stage_path, manifest_path = generated
    result = verify(stage_path, manifest_path)
    prims = {item.blocking_prim for item in result.coverage}
    assert prims
    assert all(prim is not None and prim.startswith("/World/Environment/") for prim in prims)
    assert all(item.witness_axis in {"x", "y"} for item in result.coverage)
    assert all(item.witness_coordinate_m is not None for item in result.coverage)
    # The reconciled scene genuinely needs both plane orientations. Recording
    # the axis prevents a Y coordinate from masquerading as witness_x_m.
    assert {item.witness_axis for item in result.coverage} == {"x", "y"}
    audited = set(result.usd_audit_prims)
    assert {f"/World/Environment/Corridor/{name}" for name in BUILDINGS} <= audited


def test_visible_negative_control_fails(generated: tuple[Path, Path], tmp_path: Path) -> None:
    """Prove the checker can still fail, so a pass means something."""

    stage_path, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    selected = manifest["selected_profile"]
    # Stand P in the clear corridor directly ahead of A.
    manifest["profiles"][selected]["police_bounds_min_xyz_m"] = [7.8, -0.3, 0.0]
    manifest["profiles"][selected]["police_bounds_max_xyz_m"] = [8.4, 0.3, 1.8]
    broken = tmp_path / "visible.manifest.json"
    broken.write_text(json.dumps(manifest), encoding="utf-8")

    result = verify(stage_path, broken, selected)
    assert not result.passed
    assert result.camera_visible_intervals
    assert not result.line_of_sight_blocked_everywhere
    # The independent composed-mesh audit must agree, not just the analytic proof.
    assert result.usd_audit_failures > 0
