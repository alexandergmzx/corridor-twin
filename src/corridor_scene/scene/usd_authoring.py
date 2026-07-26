"""Pure-pxr authoring of the corridor stage."""

from __future__ import annotations

import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

from .geometry import building_footprints, marker_surveys
from .model import CorridorProfile, Scenario


def _color_material(stage: Usd.Stage, name: str, color: tuple[float, float, float]):
    material = UsdShade.Material.Define(stage, f"/World/Looks/{name}")
    shader = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _marker_material(stage: Usd.Stage, marker_id: int) -> UsdShade.Material:
    root = f"/World/Looks/Marker_{marker_id:03d}"
    material = UsdShade.Material.Define(stage, root)
    surface = UsdShade.Shader.Define(stage, f"{root}/PreviewSurface")
    surface.CreateIdAttr("UsdPreviewSurface")
    texture = UsdShade.Shader.Define(stage, f"{root}/Texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(f"markers/marker_{marker_id:03d}.png")
    )
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
    reader = UsdShade.Shader.Define(stage, f"{root}/PrimvarReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result"
    )
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(), "rgb"
    )
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    return material


def _cube(
    stage: Usd.Stage,
    path: str,
    size_xyz: tuple[float, float, float],
    center_xyz: tuple[float, float, float],
    material: UsdShade.Material,
    collision: bool = False,
) -> UsdGeom.Cube:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    transform = UsdGeom.Xformable(cube)
    transform.AddTranslateOp().Set(Gf.Vec3d(*center_xyz))
    transform.AddScaleOp().Set(Gf.Vec3f(*size_xyz))
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
    return cube


def _prism_mesh(
    stage: Usd.Stage,
    path: str,
    footprint: list[tuple[float, float]],
    height: float,
    material: UsdShade.Material,
) -> UsdGeom.Mesh:
    count = len(footprint)
    points = [Gf.Vec3f(x, y, 0.0) for x, y in footprint]
    points.extend(Gf.Vec3f(x, y, height) for x, y in footprint)
    face_counts: list[int] = [count, count]
    face_indices: list[int] = list(reversed(range(count))) + list(range(count, 2 * count))
    for index in range(count):
        following = (index + 1) % count
        face_counts.append(4)
        face_indices.extend((index, following, following + count, index + count))

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr(face_counts)
    mesh.CreateFaceVertexIndicesAttr(face_indices)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(False)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    return mesh


def _road_mesh(
    stage: Usd.Stage,
    path: str,
    profile: CorridorProfile,
    length: float,
    material: UsdShade.Material,
) -> None:
    entry = profile.entry_width_m / 2.0
    corner = profile.corner_width_m / 2.0
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(
        [
            Gf.Vec3f(0.0, -entry, 0.002),
            Gf.Vec3f(length, -corner, 0.002),
            Gf.Vec3f(length, corner, 0.002),
            Gf.Vec3f(0.0, entry, 0.002),
        ]
    )
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


def _marker_mesh(
    stage: Usd.Stage,
    path: str,
    corners: tuple[tuple[float, float, float], ...],
    material: UsdShade.Material,
) -> None:
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in corners])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    texcoords = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
    )
    texcoords.Set([Gf.Vec2f(0.0, 0.0), Gf.Vec2f(1.0, 0.0), Gf.Vec2f(1.0, 1.0), Gf.Vec2f(0.0, 1.0)])
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


def _author_profile(
    stage: Usd.Stage,
    scenario: Scenario,
    profile: CorridorProfile,
    building_material: UsdShade.Material,
    road_material: UsdShade.Material,
    marker_materials: dict[int, UsdShade.Material],
) -> None:
    corridor = stage.GetPrimAtPath("/World/Environment/Corridor")
    corridor.CreateAttribute("corridor:entryWidthM", Sdf.ValueTypeNames.Double).Set(
        profile.entry_width_m
    )
    corridor.CreateAttribute("corridor:cornerWidthM", Sdf.ValueTypeNames.Double).Set(
        profile.corner_width_m
    )
    corridor.CreateAttribute("corridor:lengthM", Sdf.ValueTypeNames.Double).Set(
        scenario.corridor_length_m
    )
    _road_mesh(
        stage,
        "/World/Environment/Corridor/RoadSurface",
        profile,
        scenario.corridor_length_m,
        road_material,
    )
    for name, footprint in building_footprints(scenario, profile).items():
        _prism_mesh(
            stage,
            f"/World/Environment/Corridor/{name}",
            footprint,
            scenario.building_height_m,
            building_material,
        )
    UsdGeom.Xform.Define(stage, "/World/Environment/Corridor/Fiducials")
    for survey in marker_surveys(scenario, profile):
        _marker_mesh(
            stage,
            f"/World/Environment/Corridor/Fiducials/Marker_{survey.marker_id:03d}",
            survey.corners_xyz_m,
            marker_materials[survey.marker_id],
        )


def _camera_aperture(camera: UsdGeom.Camera, scenario: Scenario) -> None:
    focal_length_mm = 24.0
    horizontal_aperture_mm = (
        2.0 * focal_length_mm * math.tan(math.radians(scenario.camera.horizontal_fov_deg) / 2.0)
    )
    vertical_aperture_mm = horizontal_aperture_mm * (
        scenario.camera.height_px / scenario.camera.width_px
    )
    camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)
    camera.CreateFocalLengthAttr(focal_length_mm)
    camera.CreateHorizontalApertureAttr(horizontal_aperture_mm)
    camera.CreateVerticalApertureAttr(vertical_aperture_mm)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))


def _author_actors(stage: Usd.Stage, scenario: Scenario, actor_material: UsdShade.Material) -> None:
    UsdGeom.Xform.Define(stage, "/World/Actors")
    ax, ay, az = scenario.a_start_xyz_m
    actor_a = UsdGeom.Xform.Define(stage, "/World/Actors/A")
    actor_a.AddTranslateOp().Set(Gf.Vec3d(ax, ay, az))
    _cube(stage, "/World/Actors/A/Visual", (0.65, 0.45, 0.5), (0.0, 0.0, 0.25), actor_material)
    mount = UsdGeom.Xform.Define(stage, "/World/Actors/A/CameraMount")
    mount.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, scenario.camera.mount_height_m))
    camera = UsdGeom.Camera.Define(stage, "/World/Actors/A/CameraMount/FrontCamera")
    # USD cameras look down local -Z with +Y up. This maps forward to world +X
    # and image-up to world +Z at A's initial corridor pose.
    camera.AddTransformOp().Set(
        Gf.Matrix4d(
            0.0,
            -1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            -1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        )
    )
    _camera_aperture(camera, scenario)

    bx, by, bz = scenario.b_xyz_m
    _cube(stage, "/World/Actors/B", (0.45, 0.45, 1.7), (bx, by, bz + 0.85), actor_material)
    pmin = scenario.p_bounds_min_xyz_m
    pmax = scenario.p_bounds_max_xyz_m
    psize = tuple(high - low for low, high in zip(pmin, pmax, strict=True))
    pcenter = tuple((low + high) / 2.0 for low, high in zip(pmin, pmax, strict=True))
    _cube(stage, "/World/Actors/P", psize, pcenter, actor_material)


def _author_path(stage: Usd.Stage, scenario: Scenario) -> None:
    UsdGeom.Xform.Define(stage, "/World/Paths")
    curve = UsdGeom.BasisCurves.Define(stage, "/World/Paths/DeliveryPath")
    curve.CreateTypeAttr(UsdGeom.Tokens.linear)
    curve.CreateWrapAttr(UsdGeom.Tokens.nonperiodic)
    curve.CreateCurveVertexCountsAttr([len(scenario.delivery_path_xyz_m)])
    curve.CreatePointsAttr([Gf.Vec3f(*point) for point in scenario.delivery_path_xyz_m])
    curve.CreateWidthsAttr([0.04])
    curve.SetWidthsInterpolation(UsdGeom.Tokens.constant)


def _author_lighting(stage: Usd.Stage) -> None:
    """Add one inexpensive environment light for real-time rendering."""

    dome = UsdLux.DomeLight.Define(stage, "/World/Lighting/DomeLight")
    dome.CreateIntensityAttr(500.0)
    dome.CreateColorAttr(Gf.Vec3f(0.85, 0.90, 1.0))


def author_stage(
    path: Path,
    scenario: Scenario,
    profiles: tuple[CorridorProfile, ...],
    selected_profile: str,
) -> None:
    """Author and save a complete human-readable USDA stage."""

    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, "/World/Looks")
    ground_material = _color_material(stage, "Ground", (0.18, 0.20, 0.22))
    road_material = _color_material(stage, "Road", (0.10, 0.11, 0.12))
    building_material = _color_material(stage, "Building", (0.52, 0.50, 0.47))
    actor_material = _color_material(stage, "Actors", (0.10, 0.45, 0.90))

    marker_ids = {
        survey.marker_id for profile in profiles for survey in marker_surveys(scenario, profile)
    }
    marker_materials = {marker_id: _marker_material(stage, marker_id) for marker_id in marker_ids}

    physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics.CreateGravityMagnitudeAttr(9.81)
    UsdGeom.Xform.Define(stage, "/World/Environment")
    _cube(
        stage,
        "/World/Environment/Ground",
        (32.0, 24.0, 0.2),
        (8.0, 1.0, -0.1),
        ground_material,
        collision=True,
    )
    _cube(
        stage,
        "/World/Environment/CrossStreet",
        (scenario.cross_street_width_m, 20.0, 0.01),
        (scenario.corridor_length_m + scenario.cross_street_width_m / 2.0, 1.5, 0.001),
        road_material,
    )
    corridor = UsdGeom.Xform.Define(stage, "/World/Environment/Corridor")
    variants = corridor.GetPrim().GetVariantSets().AddVariantSet("corridorProfile")
    for profile in profiles:
        variants.AddVariant(profile.name)
        variants.SetVariantSelection(profile.name)
        with variants.GetVariantEditContext():
            _author_profile(
                stage,
                scenario,
                profile,
                building_material,
                road_material,
                marker_materials,
            )
    variants.SetVariantSelection(selected_profile)
    _author_actors(stage, scenario, actor_material)
    _author_path(stage, scenario)
    _author_lighting(stage)
    stage.GetRootLayer().Save()
