from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
import pytest
from police_observer.estimator import ArucoStationEstimator, MarkerMap
from police_observer.synthetic import SyntheticCamera
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from scene.build import build_scene
from scene.geometry import (
    MARKER_BACKING_OFFSET_M,
    MARKER_BACKING_SCALE,
    MARKER_WALL_CLEARANCE_M,
    building_footprints,
    corner_screen_bounds,
    corridor_faces,
    is_clear,
    police_bounds,
)
from scene.model import CorridorProfile, load_scenario
from scene.occlusion import (
    CAMERA_PRIM_PATH,
    Occluder,
    _mesh_triangles,
    _segment_hits_triangle,
    continuous_certificate,
    opaque_mesh_prims,
    verify,
)
from scene.trajectory import delivery_trajectory, trajectory_from_manifest

BUILDINGS = (
    "NorthBuilding",
    "SouthBuilding",
    "CornerBuilding",
    "EastBuilding",
    "EastWallStub",
    "CornerScreen",
)


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


def _police_y(stage: Usd.Stage) -> float:
    cache = UsdGeom.XformCache()
    cache.Clear()
    return float(
        cache.GetLocalToWorldTransform(
            stage.GetPrimAtPath("/World/Actors/P")
        ).ExtractTranslation()[1]
    )


def test_p_is_derived_from_the_geometry_and_not_frozen(tmp_path: Path) -> None:
    """A frozen P would end up in a wall or the road on some profile.

    Before ADR 0017, P sat behind the corner mass and its Y came off the south
    face, which varies with both m and n -- so it differed across all three
    configured profiles. P now stands east of the junction and its Y comes off
    the north face at ``m/2``, which varies with m alone. All three configured
    profiles share ``m = 6.0``, so P is deliberately in the *same* place in all
    of them; asserting that it moves between them would now be asserting a
    coincidence of the old anchor.

    What must still hold is that the placement is derived rather than frozen,
    so this varies the thing it actually depends on.
    """

    stage_path, _ = build_scene(None, tmp_path / "six.usda", 6.0, 3.0)
    stage = Usd.Stage.Open(str(stage_path))
    variants = _variants(stage)
    configured = {}
    for name in variants.GetVariantNames():
        variants.SetVariantSelection(name)
        configured[name] = round(_police_y(stage), 6)
    assert len(set(configured.values())) == 1, (
        f"the configured profiles share m = 6.0, so P should not move: {configured}"
    )

    # Change the one dimension P is anchored to, and it must follow.
    wider_path, _ = build_scene(None, tmp_path / "eight.usda", 8.0, 4.5)
    wider = Usd.Stage.Open(str(wider_path))
    _variants(wider).SetVariantSelection("requested_m8_n4_5")
    assert _police_y(wider) - next(iter(configured.values())) == pytest.approx(1.0, abs=1e-6), (
        "P's Y is measured from the north face at m/2, so a 2 m wider entry moves it 1 m north"
    )


def test_actor_topology_matches_the_supplied_diagram() -> None:
    """A approaches down the corridor, B is along the next street, P is inside it.

    Since ADR 0019 P stands inside the next street's east side, on the near
    (inner) face of its east wall -- the side the supplied diagram measures
    P's label on -- shielded by the corner screen rather than by standing
    beyond the wall's far face.
    """

    scenario = load_scenario()
    length = scenario.corridor_length_m
    for profile in scenario.profiles:
        police_min, police_max = police_bounds(scenario, profile)
        # P stays west of the east wall's inner face, inside the channel.
        assert police_max[0] < scenario.street_east_m
        # P stays within the span of that wall, so it is covered end to end.
        north_face = corridor_faces(profile, 0.0, length)[0]
        assert police_max[1] < north_face
        assert police_min[1] > scenario.street_south_m
        # P's body does not overlap the corner screen that hides it.
        screen_x_min, screen_x_max, screen_y_low, screen_y_high = corner_screen_bounds(
            scenario, profile
        )
        overlaps_screen = (
            police_min[0] < screen_x_max
            and police_max[1] > screen_y_low
            and police_min[1] < screen_y_high
        )
        assert not overlaps_screen
        # B stands down the next street, past the corner.
        b_y = -scenario.next_street.b_distance_m
        assert is_clear(scenario, profile, scenario.street_center_x_m, b_y)
        assert b_y < corridor_faces(profile, length, length)[1]


def test_p_stands_on_the_source_drawing_side_of_the_east_wall() -> None:
    """A6-H1: P's body must sit inside/west of the east wall's inner face.

    Measured in docs/evidence/source-diagram/NOTES.md: the next street's east
    wall spans x=1786-1850 px in the 300 dpi render, and P's label sits at
    x=1651-1758 px -- entirely inside the clear channel, 28 px west of the
    wall's near face. ADR 0017 placed P's body on the wall's *far* (outer,
    east) side, which is the opposite side from the one the source measures.

    This is a topology test, derived from the measured pixels, independent of
    ADR 0017's placement code: it fails against the ADR 0017 geometry (P east
    of ``street_east_m``) and must pass for every profile once P moves to the
    source-faithful side.
    """

    scenario = load_scenario()
    for profile in scenario.profiles:
        _, police_max = police_bounds(scenario, profile)
        assert police_max[0] <= scenario.street_east_m, (
            f"profile {profile.name}: P's body reaches x={police_max[0]:.3f}, east of the "
            f"street's east kerb at x={scenario.street_east_m:.3f}. The source measures P's "
            "label 28 px west of the east wall, inside the channel -- not beyond the wall's "
            "outer face."
        )


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
    """The check is a real gate, not a bound so loose nothing can trip it.

    ADR 0019 moved the east-face plates much lower (via the same band-floor
    clamp that already protects them from the corner mass), so they now fit
    comfortably inside the east face down to a much narrower entry width than
    m=4.0 was. m=2.0 is well below the declared m>=5.0 support floor either
    way, so this remains a check on an unsupported profile, just a more
    unsupported one.
    """

    with pytest.raises(ValueError, match="leaves the east face"):
        build_scene(None, tmp_path / "narrow.usda", 2.0, 1.0)


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


def test_east_face_plates_follow_the_visible_band(tmp_path: Path) -> None:
    """A plate's absolute coordinate must not decide whether a profile builds.

    The corner mass reaches north to ``m/2 - n``, so the usable strip of east
    face *shifts* north with a wide entry and a narrow corner. Its height is
    ``n`` regardless of ``m`` -- the band is the same size, just somewhere else
    -- so a plate whose ``along_m`` is absolute falls out of a band that could
    hold it perfectly well. That is what made ``m = 8.0, n = 3.0`` unbuildable,
    and it was a property of one hard-coded number rather than of the geometry.

    Placement now clamps to the band floor. This test pins both halves of that:
    the arithmetic that says the band is ``n`` tall, and the clamp acting only
    when the floor rises above the configured coordinate.
    """

    scenario = load_scenario()
    length = scenario.corridor_length_m

    # The band is n tall wherever it sits, which is why the old refusal could
    # not have been about running out of face.
    for entry, corner in ((6.0, 3.0), (8.0, 3.0), (10.0, 3.0)):
        profile = CorridorProfile(name="probe", entry_width_m=entry, corner_width_m=corner)
        _, corner_edge = corridor_faces(profile, length, length)
        assert profile.entry_width_m / 2.0 - corner_edge == pytest.approx(corner)

    # m = 8.0, n = 3.0 was refused before the clamp and builds now.
    stage_path, manifest_path = build_scene(None, tmp_path / "wide_entry.usda", 8.0, 3.0)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    block = manifest["profiles"]["requested_m8_n3"]
    profile = CorridorProfile(name="requested_m8_n3", entry_width_m=8.0, corner_width_m=3.0)
    _, corner_edge = corridor_faces(profile, length, length)

    stage = Usd.Stage.Open(str(stage_path))
    variants = _variants(stage)
    assert variants.SetVariantSelection("requested_m8_n3")
    east = [marker for marker in block["markers"] if marker["side"] == "east_face"]
    assert east, "the wide profile must still carry its east-face references"
    for marker in east:
        backing = UsdGeom.Mesh(
            stage.GetPrimAtPath(
                f"/World/Environment/Corridor/Fiducials/Marker_{marker['id']:03d}_Backing"
            )
        )
        points = np.asarray(backing.GetPointsAttr().Get(), dtype=np.float64)
        assert points[:, 1].min() > corner_edge, "clamped plate is still behind the corner mass"
        assert points[:, 1].max() < profile.entry_width_m / 2.0, "clamped plate ran off the top"

    # The clamp is a floor, not a free pass: a band too short for the plate is
    # still refused rather than squeezed in.
    with pytest.raises(ValueError, match="leaves the east face"):
        build_scene(None, tmp_path / "too_short.usda", 6.0, 1.0)


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
    """The written requirement -- A cannot see P, anywhere -- must hold outright.

    The stronger reciprocal claim (a wall does *all* of the hiding) is
    reported but no longer required end to end since ADR 0019: the corner
    screen covers the approach and the risky part of the turn, where P is
    genuinely in frustum, but departure, delivery-arc and delivery are
    excluded by camera frustum instead -- A is driving away from P with its
    camera facing forward, a materially different and much wider-margin way
    of not being seen, not a shortcut standing in for a missing wall.
    """

    stage_path, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for name in manifest["profiles"]:
        result = verify(stage_path, manifest_path, name)
        assert result.passed, f"profile {name}: {result.camera_visible_intervals}"
        assert result.camera_visible_intervals == ()
        # Frustum-only coverage is expected once A has turned enough to be
        # driving away from P -- the tail of the turn and everything after.
        # It must not appear on the approach, where nothing but the corner
        # screen stands between A's camera and P for the whole leg.
        assert {label for label, _, _ in result.frustum_only_intervals} <= {
            "arc",
            "departure",
            "delivery_arc",
            "delivery",
        }
        assert result.usd_audit_rays > 0
        assert result.usd_audit_failures == 0
        assert result.nearest_blocking_distance_m is not None


def test_certificate_names_the_blocking_prim(generated: tuple[Path, Path]) -> None:
    """The interview overlay quotes this, so it must be real evidence."""

    stage_path, manifest_path = generated
    result = verify(stage_path, manifest_path)
    wall_blocked = [item for item in result.coverage if item.wall_blocked]
    assert wall_blocked
    prims = {item.blocking_prim for item in wall_blocked}
    assert prims
    assert all(prim is not None and prim.startswith("/World/Environment/") for prim in prims)
    assert all(item.witness_axis in {"x", "y"} for item in wall_blocked)
    assert all(item.witness_coordinate_m is not None for item in wall_blocked)
    # Since ADR 0019 the approach and the turn are separated by the corner
    # screen, a single plane of constant X, so the default scene no longer
    # exercises both orientations. `test_a_crosswise_witness_is_still_required`
    # keeps that machinery covered.
    assert {item.witness_axis for item in wall_blocked} == {"x"}
    assert prims == {"/World/Environment/Corridor/CornerScreen"}
    audited = set(result.usd_audit_prims)
    assert {f"/World/Environment/Corridor/{name}" for name in BUILDINGS} <= audited


def test_manifest_only_police_substitution_is_rejected(
    generated: tuple[Path, Path], tmp_path: Path
) -> None:
    """A6-H2: a manifest-only mutation must not pass silently.

    ``verify()`` used to take P's bounds straight from the manifest and never
    checked them against ``/World/Actors/P`` in the composed stage, so editing
    only the manifest's numbers -- without touching the USD it claims to
    describe -- produced a certificate computed against numbers the scene
    disagrees with. Binding the verifier to the stage turns any such
    divergence into a rejection instead of a proof about the wrong P.
    """

    stage_path, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    selected = manifest["selected_profile"]
    # Stand P in the clear corridor directly ahead of A -- in the manifest
    # only. The composed stage still authors the real, hidden P.
    manifest["profiles"][selected]["police_bounds_min_xyz_m"] = [7.8, -0.3, 0.0]
    manifest["profiles"][selected]["police_bounds_max_xyz_m"] = [8.4, 0.3, 1.8]
    broken = tmp_path / "manifest_only_p_moved.manifest.json"
    broken.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(ValueError, match="diverged"):
        verify(stage_path, broken, selected)

    # The unmutated pair must still certify, so the rejection above is
    # attributable to the substitution rather than to a broken verifier.
    assert verify(stage_path, manifest_path, selected).passed


def test_stage_only_police_substitution_is_rejected(
    generated: tuple[Path, Path], tmp_path: Path
) -> None:
    """A6-H2: moving only the composed USD's P prim must not pass silently.

    This is the exact substitution the audit named: before binding the
    verifier to the stage, ``verify()`` never read ``/World/Actors/P`` at all,
    so translating it in the USD into an open, camera-visible spot changed
    nothing about the certificate -- the analytic proof still ran on the
    untouched manifest bounds and passed (confirmed by running this same
    mutation through the pre-fix verifier, which reported ``passed=True``).
    The stage here is copied to a fresh layer identifier before mutation, so
    the shared ``generated`` stage the other assertion below re-opens is never
    touched.
    """

    stage_path, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    selected = manifest["selected_profile"]

    original = Usd.Stage.Open(str(stage_path))
    mutated_path = tmp_path / "stage_only_p_moved.usda"
    original.GetRootLayer().Export(str(mutated_path))

    mutated = Usd.Stage.Open(str(mutated_path))
    _variants(mutated).SetVariantSelection(selected)
    xform = UsdGeom.Xformable(mutated.GetPrimAtPath("/World/Actors/P"))
    translate_op = next(
        op for op in xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate
    )
    # Same open-corridor spot the old manifest-only control used, this time
    # authored directly on the USD prim instead of in the manifest.
    translate_op.Set(Gf.Vec3d(8.1, 0.0, 0.9))
    mutated.GetRootLayer().Save()

    with pytest.raises(ValueError, match="diverged"):
        verify(mutated_path, manifest_path, selected)

    # The original stage, untouched, still certifies against its own manifest.
    assert verify(stage_path, manifest_path, selected).passed


def test_stage_only_camera_rotation_is_rejected(
    generated: tuple[Path, Path], tmp_path: Path
) -> None:
    """Camera orientation, not just position, must be bound to the stage.

    Before this test, ``stage_camera_facts`` read the camera's world
    position and FOV but never its rotation, so the analytic certificate and
    the mesh raycast audit both computed every heading along the route from
    the manifest-derived trajectory alone -- the composed stage's actual
    camera orientation was never consulted. A camera rolled 180 degrees about
    its own local Z axis keeps the same position, aperture and even the same
    forward axis (only its up axis flips), so it reproduces the exact
    negative control that first exposed the gap: the pre-fix verifier
    reported ``passed=True`` with zero mesh-audit failures for a camera that
    plainly does not author the pose the certificate assumes.
    """

    stage_path, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    selected = manifest["selected_profile"]

    original = Usd.Stage.Open(str(stage_path))
    mutated_path = tmp_path / "stage_only_camera_rolled.usda"
    original.GetRootLayer().Export(str(mutated_path))

    mutated = Usd.Stage.Open(str(mutated_path))
    _variants(mutated).SetVariantSelection(selected)
    xform = UsdGeom.Xformable(mutated.GetPrimAtPath(CAMERA_PRIM_PATH))
    transform_op = next(
        op for op in xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTransform
    )
    roll_180_about_local_z = Gf.Matrix4d(
        -1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
    )
    transform_op.Set(roll_180_about_local_z * transform_op.Get())
    mutated.GetRootLayer().Save()

    with pytest.raises(ValueError, match="diverged"):
        verify(mutated_path, manifest_path, selected)

    # The original stage, untouched, still certifies against its own manifest.
    assert verify(stage_path, manifest_path, selected).passed


def test_a_genuinely_visible_placement_fails_promptly(generated: tuple[Path, Path]) -> None:
    """A6-M1: a real visible/unresolved region must not exhaust the recursion budget.

    Unlike the two substitution tests above, this drives the analytic
    certificate directly with a stage-and-manifest-*consistent* placement --
    there is no divergence to reject, so this exercises the actual recursive
    search on a target that is genuinely, unresolvably visible. Before the
    call budget was added this took 40.7 s and produced 327,719 coverage
    entries for one profile; it must now resolve well under a second.
    """

    _, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    selected = manifest["selected_profile"]
    block = manifest["profiles"][selected]
    trajectory = trajectory_from_manifest(block["delivery_trajectory"])
    slabs = tuple(Occluder(**slab) for slab in block["occluders"])
    fov = float(manifest["camera"]["horizontal_fov_deg"])

    start = time.monotonic()
    result = continuous_certificate(
        trajectory, (7.8, -0.3, 0.0), (8.4, 0.3, 1.8), slabs, fov, selected
    )
    elapsed_s = time.monotonic() - start

    assert elapsed_s < 5.0, f"a visible negative control must terminate promptly, took {elapsed_s}s"
    assert not result.passed
    assert result.camera_visible_intervals
    assert not result.line_of_sight_blocked_everywhere


def test_removing_the_corner_screen_fails_the_certificate(
    generated: tuple[Path, Path], tmp_path: Path
) -> None:
    """The wall that hides P must be load-bearing, not incidentally present.

    Under ADR 0019 P stands inside the channel, on the near side of the next
    street's east wall, so that wall no longer separates it from A's route --
    the corner screen does, for the approach and the risky part of the turn.
    `occluders()` keeps the east wall in the list anyway, now incidentally
    rather than because the proof leans on it; removing the corner screen
    instead is what must fail the proof.

    Note the mesh audit still passes: it discovers prims from the composed
    stage by collision schema, so it does not care what the manifest lists.
    That the two halves disagree here is the point -- they are independent,
    and this exercises the analytic one.
    """

    stage_path, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    selected = manifest["selected_profile"]
    block = manifest["profiles"][selected]

    kept = [
        slab for slab in block["occluders"] if not slab["prim_path"].endswith("CornerScreen")
    ]
    assert len(kept) == len(block["occluders"]) - 1, "the corner screen must be in the slab list"
    block["occluders"] = kept
    mutated = tmp_path / "no_corner_screen.manifest.json"
    mutated.write_text(json.dumps(manifest), encoding="utf-8")

    result = verify(stage_path, mutated, selected)
    assert not result.passed

    # Fail for the right reason. A mutation test that passes because the code
    # crashed is a false green, so require the specific symptom: P becomes
    # genuinely camera-visible, over named route intervals, rather than the
    # proof merely failing to find a witness.
    assert result.camera_visible_intervals, "P should be exposed once its wall is gone"
    assert {label for label, _, _ in result.camera_visible_intervals} <= {"approach", "arc"}

    # And the mesh audit must still pass. It discovers prims from the composed
    # stage, which the mutation did not touch, so a failure here would mean
    # something structural broke instead of the analytic proof losing its
    # witness -- exactly the confusion this assertion exists to prevent.
    assert result.usd_audit_rays > 0
    assert result.usd_audit_failures == 0
    assert any("CornerScreen" in prim for prim in result.usd_audit_prims)

    # And the unmutated manifest passes with that wall doing the work, so the
    # failure above is attributable to the removal rather than to anything else.
    intact = verify(stage_path, manifest_path, selected)
    assert intact.passed
    assert {item.blocking_prim for item in intact.coverage if item.wall_blocked} == {
        "/World/Environment/Corridor/CornerScreen"
    }


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
    assert manifest["schema_version"] == "0.5.0"


# Captured from the build immediately before east-face placement began clamping
# to the visible band. The clamp was only safe to make because it provably moves
# no profile that has published accuracy figures; this is that proof, and it
# fails loudly if a future placement change quietly invalidates a measured run.
# ADR 0019 baseline. Both plates now use a deliberately low, even negative,
# nominal `along_m` (see corridor.yaml) so the existing band-floor clamp --
# `max(along_m, band_floor)` -- places them, rather than a value tuned to one
# profile's corner screen. wide_corner and uniform share a result because
# neither's floor is high enough to bind against the requested value; nominal's
# floor (0.673) does bind, which is why it differs from the other two.
EAST_FACE_SURVEY_AFTER_ADR_0019 = {
    "nominal_m6_n3": {"83": (17.983, 1.172857, 2.1), "84": (17.983, 0.715714, 0.7)},
    "wide_corner_m6_n4_5": {"83": (17.983, -0.1, 2.1), "84": (17.983, 0.35, 0.7)},
    "uniform_m6_n6": {"83": (17.983, -0.1, 2.1), "84": (17.983, 0.35, 0.7)},
}


def test_the_band_clamp_moved_no_configured_profile(generated: tuple[Path, Path]) -> None:
    """Pin where the band-floor clamp actually places each configured profile.

    Every configured profile carrying a measured figure must land at a known,
    checked position -- clamped or not -- so a future change to the clamp, the
    corner screen, or these plates' own `along_m` cannot silently invalidate a
    published accuracy figure without a test noticing.
    """

    _, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    checked = 0
    for name, block in manifest["profiles"].items():
        expected_by_id = EAST_FACE_SURVEY_AFTER_ADR_0019[name]
        for marker in block["markers"]:
            if marker["side"] != "east_face":
                continue
            expected = expected_by_id[str(marker["id"])]
            first_corner = tuple(round(value, 6) for value in marker["corners_xyz_m"][0])
            assert first_corner == pytest.approx(expected, abs=1e-6), (
                f"{name}: marker {marker['id']} moved to {first_corner}; a configured "
                "profile shifting invalidates its published accuracy figures"
            )
            checked += 1
    assert checked == sum(len(v) for v in EAST_FACE_SURVEY_AFTER_ADR_0019.values())


def test_a_crosswise_witness_is_still_required(generated: tuple[Path, Path]) -> None:
    """Keep the constant-Y witness covered after ADR 0017 moved P east.

    ADR 0011 argued the crosswise witness was "not an optimisation but a
    necessity": where A drew level with P, no plane of constant X separated
    them at all. That was a property of the *west* placement. With P east of
    the junction the east wall separates every interval on its own, so the
    default scene now uses X planes exclusively and would no longer notice if
    the Y solver broke.

    This drives the previous west placement, verbatim, against the current
    occluders. It still needs both orientations, so the machinery ADR 0011
    justified keeps its regression instead of quietly losing it when the
    default scene stopped needing it.
    """

    _, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    selected = manifest["selected_profile"]
    block = manifest["profiles"][selected]

    trajectory = trajectory_from_manifest(block["delivery_trajectory"])
    slabs = tuple(Occluder(**slab) for slab in block["occluders"])
    fov = float(manifest["camera"]["horizontal_fov_deg"])

    # P as it stood before ADR 0017: south of the corridor, behind the corner
    # mass, on the nominal profile.
    result = continuous_certificate(
        trajectory, (10.6, -2.275, 0.0), (11.2, -1.675, 1.8), slabs, fov, selected
    )
    assert result.passed, "the superseded placement was valid; this is a solver check"
    assert {item.witness_axis for item in result.coverage} == {"x", "y"}


def test_manifest_publishes_every_authored_wall(generated: tuple[Path, Path]) -> None:
    """A wall that is not published is a wall no consumer can see.

    The manifest used to carry only `occluders` -- the analytic proof's slab
    list -- so anything reading it was structurally unable to know about a wall
    the proof does not reference. That is why the east-wall stub was invisible
    in RViz and unchecked by the Isaac smoke test while being solid in the stage
    and audited by the mesh raycast. See ADR 0018.
    """

    stage_path, manifest_path = generated
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    scenario = load_scenario()
    stage = Usd.Stage.Open(str(stage_path))
    variants = _variants(stage)

    for name, block in manifest["profiles"].items():
        assert variants.SetVariantSelection(name)
        expected = set(building_footprints(scenario, scenario.profile(name)))
        assert set(block["walls"]) == expected, f"{name}: manifest walls disagree with geometry"
        assert set(block["walls"]) == set(BUILDINGS), f"{name}: BUILDINGS is out of date"

        # Every occluder is one of those walls; they are the same surfaces.
        occluding = {slab["prim_path"].rsplit("/", 1)[-1] for slab in block["occluders"]}
        assert occluding <= set(block["walls"])

        # And each published footprint matches the authored prim.
        for wall, footprint in block["walls"].items():
            mesh = UsdGeom.Mesh(stage.GetPrimAtPath(f"/World/Environment/Corridor/{wall}"))
            assert mesh, f"{name}: {wall} is published but not authored"
            authored = {(round(p[0], 6), round(p[1], 6)) for p in mesh.GetPointsAttr().Get()}
            assert {(round(x, 6), round(y, 6)) for x, y in footprint} <= authored
