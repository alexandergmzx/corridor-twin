from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest
from police_observer.estimator import ArucoStationEstimator, MarkerMap
from police_observer.synthetic import SyntheticCamera
from pxr import Usd, UsdGeom, UsdPhysics
from scene.build import build_scene
from scene.geometry import (
    MARKER_BACKING_OFFSET_M,
    MARKER_BACKING_SCALE,
    MARKER_WALL_CLEARANCE_M,
    corridor_faces,
    is_clear,
    police_bounds,
)
from scene.model import load_scenario
from scene.occlusion import _mesh_triangles, _segment_hits_triangle, opaque_mesh_prims, verify
from scene.trajectory import delivery_trajectory

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


@pytest.mark.parametrize(("m", "n"), [(6.0, 3.0), (5.5, 3.2), (5.0, 3.0), (8.0, 4.5)])
def test_reference_backings_stay_inside_their_host_face(tmp_path: Path, m: float, n: float) -> None:
    """A requested profile must not mount a plate off the end of its wall.

    The east face spans y only up to the north wall at m/2, so it shortens with
    a narrower entry width, and reference plates were validated against the
    widest *configured* profile before ``build`` had even appended the
    requested one. At m = 5.5 that admitted marker 83's backing at y = 2.8429
    against an east face ending at y = 2.75 — 0.09 m inside the adjoining
    building. m = 5.0 is the declared support floor and m = 8.0 is wider than
    any configured profile, so both ends of the range are exercised here. That
    wide case pairs with n = 4.5: with a narrow corner the corner mass reaches
    north over the east face, which the separate envelope test below covers.
    """

    stage_path, manifest_path = build_scene(None, tmp_path / f"m{m}.usda", m, n)
    stage = Usd.Stage.Open(str(stage_path))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    scenario = load_scenario()
    variants = _variants(stage)

    checked = 0
    for name, block in manifest["profiles"].items():
        assert variants.SetVariantSelection(name)
        north_face = _north_face(stage)
        for marker in block["markers"]:
            if marker["role"] != "reference":
                continue
            backing = UsdGeom.Mesh(
                stage.GetPrimAtPath(
                    f"/World/Environment/Corridor/Fiducials/Marker_{marker['id']:03d}_Backing"
                )
            )
            assert backing, f"{name}: marker {marker['id']} has no backing"
            points = np.asarray(backing.GetPointsAttr().Get(), dtype=np.float64)
            if marker["side"] == "north_wall":
                axis, low, high = 0, -scenario.west_margin_m, scenario.street_east_m
            else:
                axis, low, high = 1, scenario.street_south_m, north_face
            assert points[:, axis].min() >= low - 1e-9, f"{name}: marker {marker['id']} runs short"
            assert points[:, axis].max() <= high + 1e-9, f"{name}: marker {marker['id']} overhangs"
            checked += 1
    assert checked == len(manifest["profiles"]) * len(scenario.fiducials.references.plates)


def test_a_profile_too_narrow_for_its_reference_plates_is_rejected(tmp_path: Path) -> None:
    """The check is a real gate, not a bound so loose nothing can trip it."""

    with pytest.raises(ValueError, match="leaves the east face"):
        build_scene(None, tmp_path / "narrow.usda", 4.0, 3.0)


def test_east_face_plates_must_clear_the_corner_mass(tmp_path: Path) -> None:
    """Being on the east face is not enough; it must be visible from the corridor.

    The corner mass reaches north to the south face at x = L, which is
    ``m/2 - n`` -- exactly y = 0.0 on the default profile. Marker 84 was
    centred there, so half of it sat behind the corner building. SyntheticCamera
    projects without raycasting and rendered it whole, so the defect was
    invisible in synthetic runs while a real render would show it cut in half.

    Both corridor faces are straight, so the mouth at x = L is the only binding
    plane and a single comparison decides it.
    """

    scenario = load_scenario()
    profile = scenario.profile("nominal_m6_n3")
    length = scenario.corridor_length_m
    _, corner_south_face = corridor_faces(profile, length, length)
    assert corner_south_face == pytest.approx(0.0), "the default profile is the sharpest case"

    stage_path, manifest_path = build_scene(None, tmp_path / "corridor.usda", 6.0, 3.0)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    stage = Usd.Stage.Open(str(stage_path))
    variants = _variants(stage)

    for name, block in manifest["profiles"].items():
        assert variants.SetVariantSelection(name)
        this_profile = scenario.profile(name)
        _, edge = corridor_faces(
            this_profile, scenario.corridor_length_m, scenario.corridor_length_m
        )
        for marker in block["markers"]:
            if marker["side"] != "east_face":
                continue
            backing = UsdGeom.Mesh(
                stage.GetPrimAtPath(
                    f"/World/Environment/Corridor/Fiducials/Marker_{marker['id']:03d}_Backing"
                )
            )
            points = np.asarray(backing.GetPointsAttr().Get(), dtype=np.float64)
            assert points[:, 1].min() > edge, (
                f"{name}: marker {marker['id']} reaches y={points[:, 1].min():.4f}, "
                f"behind the corner mass at y={edge:.4f}"
            )


def test_a_profile_whose_corner_mass_swallows_the_east_face_is_rejected(tmp_path: Path) -> None:
    """The supported (m, n) envelope is bounded, and the bound is real geometry.

    The corner mass reaches north to ``m/2 - n``, so a wide entry with a narrow
    corner walls off the part of the east face the references are mounted on.
    Configured plates admit ``m/2 - n < 0.349``; ``m = 8.0`` with ``n = 3.0``
    puts that edge at y = 1.0 and is refused. Before this check the same build
    succeeded and simply rendered a half-buried marker.
    """

    with pytest.raises(ValueError, match="behind the corner mass"):
        build_scene(None, tmp_path / "swallowed.usda", 8.0, 3.0)

    # Just inside the envelope still builds, so this is a bound and not a wall.
    build_scene(None, tmp_path / "edge.usda", 6.5, 3.0)


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


def test_marker_plates_stay_on_the_corridor_side_of_actual_walls(
    generated: tuple[Path, Path],
) -> None:
    """A canted plate and its quiet zone must not be buried in a wall mesh."""

    stage_path, manifest_path = generated
    stage = Usd.Stage.Open(str(stage_path))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    scenario = load_scenario()
    variants = _variants(stage)
    for name, block in manifest["profiles"].items():
        assert variants.SetVariantSelection(name)
        profile = next(p for p in scenario.profiles if p.name == name)
        north_face, _ = corridor_faces(profile, 0.0, scenario.corridor_length_m)
        for marker in block["markers"]:
            root = "/World/Environment/Corridor/Fiducials"
            backing = UsdGeom.Mesh(
                stage.GetPrimAtPath(f"{root}/Marker_{marker['id']:03d}_Backing")
            )
            assert backing
            for point in backing.GetPointsAttr().Get():
                # The clearance rule is shared, but each plate is measured
                # against its own host surface. Reference plates live on the
                # north wall extension and the east building face, not on the
                # tapered corridor walls.
                if marker["side"] == "north":
                    north, _ = corridor_faces(profile, point[0], scenario.corridor_length_m)
                    signed_clearance = north - point[1]
                elif marker["side"] == "south":
                    _, south = corridor_faces(profile, point[0], scenario.corridor_length_m)
                    slope = (
                        profile.entry_width_m - profile.corner_width_m
                    ) / scenario.corridor_length_m
                    signed_clearance = (point[1] - south) / math.hypot(slope, 1.0)
                elif marker["side"] == "north_wall":
                    signed_clearance = north_face - point[1]
                elif marker["side"] == "east_face":
                    signed_clearance = scenario.street_east_m - point[0]
                else:
                    raise AssertionError(f"unknown marker surface {marker['side']!r}")
                assert signed_clearance >= MARKER_WALL_CLEARANCE_M - 1e-5
                assert 0.0 < point[2] < scenario.building_height_m


def test_marker_codes_have_a_geometric_white_quiet_zone(
    generated: tuple[Path, Path],
) -> None:
    """The black code border needs white backing beyond its surveyed corners."""

    stage_path, manifest_path = generated
    stage = Usd.Stage.Open(str(stage_path))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    variants = _variants(stage)
    for profile_name, block in manifest["profiles"].items():
        assert variants.SetVariantSelection(profile_name)
        for marker in block["markers"]:
            root = "/World/Environment/Corridor/Fiducials"
            code = UsdGeom.Mesh(stage.GetPrimAtPath(f"{root}/Marker_{marker['id']:03d}"))
            backing = UsdGeom.Mesh(
                stage.GetPrimAtPath(f"{root}/Marker_{marker['id']:03d}_Backing")
            )
            assert code and backing
            code_points = np.asarray(code.GetPointsAttr().Get(), dtype=np.float64)
            backing_points = np.asarray(backing.GetPointsAttr().Get(), dtype=np.float64)
            code_center = np.mean(code_points, axis=0)
            backing_center = np.mean(backing_points, axis=0)
            code_radius = np.max(np.linalg.norm(code_points - code_center, axis=1))
            backing_radius = np.max(np.linalg.norm(backing_points - backing_center, axis=1))
            assert backing_radius / code_radius == pytest.approx(
                MARKER_BACKING_SCALE,
                abs=1e-5,
            )
            assert np.linalg.norm(backing_center - code_center) == pytest.approx(
                MARKER_BACKING_OFFSET_M,
                abs=1e-6,
            )
            assert backing.GetDoubleSidedAttr().Get()


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


def test_reference_plates_never_become_enforcement_gates(
    generated: tuple[Path, Path],
) -> None:
    """A reference plate at x=18 would be a gate the robot never crosses.

    Without the role split the observer's gate list becomes
    (2, 4, 6, 8, 10, 13, 15, 17, 18), which corrupts every crossing
    interpolation.
    """

    _, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for block in manifest["profiles"].values():
        roles = {marker["id"]: marker["role"] for marker in block["markers"]}
        assert set(roles.values()) == {"gate", "reference"}
        gates = sorted({m["station_m"] for m in block["markers"] if m["role"] == "gate"})
        assert gates == [2.0, 4.0, 6.0, 8.0, 10.0]
        references = [m for m in block["markers"] if m["role"] == "reference"]
        assert references
        assert all(m["station_m"] not in gates for m in references)
        # Ids must stay unique across both roles.
        assert len(roles) == len(block["markers"])

    marker_map = MarkerMap.from_manifest(manifest_path)
    assert marker_map.gate_stations_m == (2.0, 4.0, 6.0, 8.0, 10.0)
    # References are still valid pose evidence.
    assert len(marker_map.marker_corners) == len(
        manifest["profiles"][manifest["selected_profile"]]["markers"]
    )


def test_role_less_markers_stay_gates_and_unknown_roles_fail(
    generated: tuple[Path, Path], tmp_path: Path
) -> None:
    """Schema-0.2 manifests carry no role and must keep working."""

    _, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    selected = manifest["selected_profile"]
    block = manifest["profiles"][selected]

    legacy = json.loads(json.dumps(manifest))
    legacy_block = legacy["profiles"][selected]
    legacy_block["markers"] = [
        {key: value for key, value in marker.items() if key != "role"}
        for marker in block["markers"]
        if marker["role"] == "gate"
    ]
    legacy_path = tmp_path / "schema-0.2.json"
    legacy_path.write_text(json.dumps(legacy), encoding="utf-8")
    assert MarkerMap.from_manifest(legacy_path).gate_stations_m == (2.0, 4.0, 6.0, 8.0, 10.0)

    broken = json.loads(json.dumps(manifest))
    broken["profiles"][selected]["markers"][0]["role"] = "landmark"
    broken_path = tmp_path / "unknown-role.json"
    broken_path.write_text(json.dumps(broken), encoding="utf-8")
    with pytest.raises(ValueError, match="unknown marker roles"):
        MarkerMap.from_manifest(broken_path)


@pytest.mark.parametrize(
    "widths", [(6.0, 3.0), (6.0, 4.5), (6.0, 6.0)], ids=["nominal", "wide", "uniform"]
)
def test_corner_coverage_uses_unoccluded_non_coplanar_references(
    tmp_path: Path, widths: tuple[float, float]
) -> None:
    """Coverage must not depend on a plate the buildings hide, or on one plane.

    SyntheticCamera only projects; it never raycasts against geometry, so the
    accepted markers are re-checked here against the composed meshes. Centres
    *and* every corner are cast, because a plate whose centre is clear can still
    have a corner buried. The accepted correspondences must also span rank 3:
    two plates on one building face are as ambiguous as one, and no marker count
    detects that.

    Coverage is required through the last enforcement gate plus the margin
    needed to bracket its crossing, not to the end of the corridor. Past that
    only the coplanar east-face pair remains in view, and the estimator is
    expected to reject those frames rather than emit an ambiguous pose.
    """

    stage_path, manifest_path = build_scene(None, tmp_path / "corridor.usda", *widths)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    scenario = load_scenario()
    profile = scenario.profile(manifest["selected_profile"])
    trajectory = delivery_trajectory(scenario, profile)
    stage = Usd.Stage.Open(str(stage_path))
    triangles = [t for prim in opaque_mesh_prims(stage) for t in _mesh_triangles(prim)]

    block = manifest["profiles"][manifest["selected_profile"]]
    surveyed = {
        marker["id"]: np.asarray(marker["corners_xyz_m"], dtype=np.float64)
        for marker in block["markers"]
    }
    planes = {
        marker["id"]: marker["side"]
        for marker in block["markers"]
        if marker["role"] == "reference"
    }
    camera = SyntheticCamera(manifest_path)
    estimator = ArucoStationEstimator(
        MarkerMap.from_manifest(manifest_path), camera.dictionary_name
    )

    last_gate = max(
        marker["station_m"] for marker in block["markers"] if marker["role"] == "gate"
    )
    stations = [8.0, 9.0, 10.0, last_gate + 0.4]
    for station_x_m in stations:
        pose = trajectory.camera_pose_at(trajectory.approach_s_at_x(station_x_m))
        origin = (pose.x_m, pose.y_m, pose.z_m)
        observation = estimator.estimate(
            camera.render(station_x_m), camera.calibration, timestamp_s=1.0 + station_x_m
        )
        assert observation is not None, f"no estimate at x={station_x_m}"
        accepted = list(observation.marker_ids)

        for marker_id in accepted:
            corners = surveyed[marker_id]
            for target in [corners.mean(axis=0), *corners]:
                blocked = any(
                    _segment_hits_triangle(origin, tuple(target), triangle) is not None
                    for triangle in triangles
                )
                assert not blocked, f"marker {marker_id} is occluded at x={station_x_m}"

        combined = np.concatenate([surveyed[marker_id] for marker_id in accepted])
        rank = np.linalg.matrix_rank(combined - combined.mean(axis=0), tol=1e-6)
        assert rank == 3, (
            f"x={station_x_m} accepted a rank-{rank} set from planes "
            f"{ {planes.get(i, 'corridor') for i in accepted} }"
        )


@pytest.mark.parametrize(
    "profile_name",
    ["nominal_m6_n3", "wide_corner_m6_n4_5", "uniform_m6_n6"],
    ids=["nominal", "wide", "uniform"],
)
def test_coplanar_reference_pair_alone_is_rejected(
    generated: tuple[Path, Path], profile_name: str
) -> None:
    """The estimator must refuse rank-deficient sets, not rely on the layout.

    Past the covered window only the two east-face plates remain, and they are
    coplanar. Counting markers would accept that pair, since two is already the
    minimum; the rank rule is what does not.

    This drives ``estimate()`` on a real rendered frame rather than asserting
    that the corners are rank 2 and the property defaults to 3, which would
    keep passing if the rank check were deleted from the estimator. The
    permissive control differs from production in exactly one parameter, so if
    it accepts the same frame production rejects, the rank rule is the only
    thing that can be responsible.
    """

    _, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    marker_map = MarkerMap.from_manifest(manifest_path, profile_name)
    camera = SyntheticCamera(manifest_path, profile_name)
    surveyed = {
        marker["id"]: np.asarray(marker["corners_xyz_m"], dtype=np.float64)
        for marker in manifest["profiles"][profile_name]["markers"]
    }

    production = ArucoStationEstimator(marker_map, camera.dictionary_name)
    permissive = ArucoStationEstimator(
        marker_map, camera.dictionary_name, minimum_correspondence_rank=2
    )
    assert production.minimum_markers == permissive.minimum_markers
    assert production.minimum_correspondence_rank == 3

    # The first station past the covered window on every profile: the corridor
    # wall gates and the north-wall references have left the frustum and only
    # the two east-face plates still decode.
    station_x_m = 11.5
    image = camera.render(station_x_m)

    accepted = permissive.estimate(image, camera.calibration, timestamp_s=1.0)
    assert accepted is not None, "the control must accept the frame for the test to mean anything"
    assert len(accepted.marker_ids) >= permissive.minimum_markers
    corners = np.concatenate([surveyed[marker_id] for marker_id in accepted.marker_ids])
    assert np.linalg.matrix_rank(corners - corners.mean(axis=0), tol=1e-6) == 2

    assert production.estimate(image, camera.calibration, timestamp_s=1.0) is None


def test_generated_manifests_declare_the_new_schema(generated: tuple[Path, Path]) -> None:
    """Roles and reference plates are new fields, so the schema moves with them."""

    _, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == "0.3.0"
