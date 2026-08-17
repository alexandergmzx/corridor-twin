"""Pure-pxr authoring of the corridor stage."""

from __future__ import annotations

import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

from .geometry import (
    all_surveys,
    building_footprints,
    corridor_faces,
    p_cam_pose,
    person_b_xyz,
    plate_backing_corners,
    police_bounds,
)
from .model import CorridorProfile, Scenario
from .trajectory import DeliveryTrajectory, delivery_trajectory


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


def _cylinder(
    stage: Usd.Stage,
    path: str,
    radius: float,
    height: float,
    center_xyz: tuple[float, float, float],
    material: UsdShade.Material,
) -> UsdGeom.Cylinder:
    """A vertical post. Circular in section, which is the whole point.

    A cylinder returns an arc of KNOWN RADIUS from every bearing, which is what
    makes it separable from a flat wall and from a convex corner in a single
    scan -- and separable without ever consulting return intensity, whose
    sim-to-real fidelity nobody in this project owns.
    """

    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(height)
    cylinder.CreateAxisAttr("Z")
    cylinder.CreateExtentAttr(
        [(-radius, -radius, -height / 2.0), (radius, radius, height / 2.0)]
    )
    UsdGeom.Xformable(cylinder).AddTranslateOp().Set(Gf.Vec3d(*center_xyz))
    UsdShade.MaterialBindingAPI(cylinder).Bind(material)
    # SOLID, because the delivery is now a contact.
    #
    # Every wall in this scene has carried a collider since the beginning
    # (`_cube(..., collision=True)`), and B never did. It did not matter while
    # arrival was a distance: A's lidar sees B either way, because the RTX lidar
    # traces RENDER geometry rather than physics, and Nav2 kept clear of B
    # because the scan made it a lethal costmap cell. So B was visible,
    # avoidable, and utterly intangible -- A would have driven straight through
    # it, and nothing in the scenario would have noticed.
    #
    # ADR 0033 makes the bump the arrival, and a bump needs something to bump.
    # Static, like the walls: no `RigidBodyAPI`, so B does not move, topple or
    # get pushed down the street when a 0.2 m robot leans on it. A stalls
    # against it, and the stall is what the encoders report.
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim()).CreateCollisionEnabledAttr(True)
    return cylinder


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


def _quad_mesh(
    stage: Usd.Stage,
    path: str,
    corners_xy: tuple[tuple[float, float], ...],
    height_m: float,
    material: UsdShade.Material,
) -> None:
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(x, y, height_m) for x, y in corners_xy])
    mesh.CreateFaceVertexCountsAttr([len(corners_xy)])
    mesh.CreateFaceVertexIndicesAttr(list(range(len(corners_xy))))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


def _corridor_road(
    stage: Usd.Stage,
    path: str,
    scenario: Scenario,
    profile: CorridorProfile,
    material: UsdShade.Material,
) -> None:
    """Author the asymmetric corridor road surface between the two faces."""

    length = scenario.corridor_length_m
    north, south_entry = corridor_faces(profile, 0.0, length)
    _, south_corner = corridor_faces(profile, length, length)
    _quad_mesh(
        stage,
        path,
        (
            (-scenario.west_margin_m, south_entry),
            (0.0, south_entry),
            (length, south_corner),
            (length, north),
            (-scenario.west_margin_m, north),
        ),
        0.002,
        material,
    )


def _next_street_road(
    stage: Usd.Stage,
    path: str,
    scenario: Scenario,
    profile: CorridorProfile,
    material: UsdShade.Material,
) -> None:
    """Author the perpendicular street A turns onto to reach B."""

    north, _ = corridor_faces(profile, 0.0, scenario.corridor_length_m)
    _quad_mesh(
        stage,
        path,
        (
            (scenario.street_west_m, scenario.street_south_m),
            (scenario.street_east_m, scenario.street_south_m),
            (scenario.street_east_m, north),
            (scenario.street_west_m, north),
        ),
        0.002,
        material,
    )


def _marker_mesh(
    stage: Usd.Stage,
    path: str,
    corners: tuple[tuple[float, float, float], ...],
    normal: tuple[float, float, float],
    material: UsdShade.Material,
    backing_material: UsdShade.Material,
) -> None:
    backing_corners = plate_backing_corners(corners, normal)
    backing = UsdGeom.Mesh.Define(stage, f"{path}_Backing")
    backing.CreatePointsAttr([Gf.Vec3f(*point) for point in backing_corners])
    backing.CreateFaceVertexCountsAttr([4])
    backing.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    backing.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    backing.CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI.Apply(backing.GetPrim()).Bind(backing_material)

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
    marker_backing_material: UsdShade.Material,
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
    _corridor_road(
        stage,
        "/World/Environment/Corridor/RoadSurface",
        scenario,
        profile,
        road_material,
    )
    _next_street_road(
        stage,
        "/World/Environment/Corridor/NextStreetSurface",
        scenario,
        profile,
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
    for survey in all_surveys(scenario, profile):
        _marker_mesh(
            stage,
            f"/World/Environment/Corridor/Fiducials/Marker_{survey.marker_id:03d}",
            survey.corners_xyz_m,
            survey.normal_xyz,
            marker_materials[survey.marker_id],
            marker_backing_material,
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


def _author_shared_actors(
    stage: Usd.Stage, scenario: Scenario, actor_material: UsdShade.Material
) -> None:
    """Author the actors whose placement does not depend on the profile."""

    UsdGeom.Xform.Define(stage, "/World/Actors")

    # B IS THE CYLINDER (ADR 0031). One prim: the thing the viewer sees and the
    # thing A's lidar fits a circle to are the same object, so there is no
    # second place for them to disagree about where B is.
    bx, by, bz = person_b_xyz(scenario)
    actors = scenario.actors
    _cylinder(
        stage,
        "/World/Actors/B",
        actors.b_radius_m,
        actors.b_height_m,
        (bx, by, bz + actors.b_height_m / 2.0),
        actor_material,
    )


def _author_profile_actors(
    stage: Usd.Stage,
    scenario: Scenario,
    profile: CorridorProfile,
    actor_material: UsdShade.Material,
) -> None:
    """Author A, its camera, P, and the delivery path for one corridor profile.

    A's start pose, P's standoff from the corner mass, and the whole route are
    all derived from the corridor faces, so they are authored inside the
    ``corridorProfile`` variant. Selecting a different ``(m,n)`` then moves them
    together and keeps P behind the opaque corner instead of stranding it in a
    wall or in the road.
    """

    trajectory = delivery_trajectory(scenario, profile)
    start = trajectory.pose_at(0.0)
    actor_a = UsdGeom.Xform.Define(stage, "/World/Actors/A")
    actor_a.AddTranslateOp().Set(Gf.Vec3d(start.x_m, start.y_m, start.z_m))
    actor_a.AddRotateZOp().Set(math.degrees(start.yaw_rad))
    a_size = scenario.actors.a_size_xyz_m
    _cube(stage, "/World/Actors/A/Visual", a_size, (0.0, 0.0, a_size[2] / 2.0), actor_material)
    # A's v1 eye point, and NOT a camera. ADR 0021 moved the render product to
    # P and ADR 0024 made A camera-less; this Xform survives because the
    # geometric visibility gate -- "does an opaque wall block the segment from
    # A's eye to P's body" -- is scenario realism this project does not disavow
    # (CLAUDE.md invariant 2). It carries no UsdGeom.Camera, so the stage holds
    # exactly one camera and it is P's.
    mount = UsdGeom.Xform.Define(stage, "/World/Actors/A/CameraMount")
    mount.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, scenario.camera.mount_height_m))

    pmin, pmax = police_bounds(scenario, profile)
    psize = tuple(high - low for low, high in zip(pmin, pmax, strict=True))
    pcenter = tuple((low + high) / 2.0 for low, high in zip(pmin, pmax, strict=True))
    _cube(stage, "/World/Actors/P", psize, pcenter, actor_material)

    # P's enforcement camera: the stage's ONE UsdGeom.Camera, and the one render
    # product the adapter attaches (CLAUDE.md invariant 3, as ADR 0021 recast
    # it). A sibling of P rather than a child, because `_cube` scales P's prim
    # and a child would inherit that scale into the camera's basis.
    pose = p_cam_pose(scenario, profile)
    mast = UsdGeom.Xform.Define(stage, "/World/Actors/PCameraMast")
    mast.AddTranslateOp().Set(Gf.Vec3d(*pose["eye_xyz_m"]))
    p_cam = UsdGeom.Camera.Define(stage, "/World/Actors/PCameraMast/PCam")
    right, up, forward = pose["right_xyz"], pose["up_xyz"], pose["forward_xyz"]
    # Rows are the camera's local axes in world: (right, image-up, -forward),
    # because a USD camera looks down local -Z with +Y up. Derived in
    # `geometry.p_cam_pose` rather than written as a literal matrix, so a mast
    # that moves takes its orientation with it.
    p_cam.AddTransformOp().Set(
        Gf.Matrix4d(
            right[0], right[1], right[2], 0.0,
            up[0], up[1], up[2], 0.0,
            -forward[0], -forward[1], -forward[2], 0.0,
            0.0, 0.0, 0.0, 1.0,
        )
    )
    _camera_aperture(p_cam, scenario)

    _author_path(stage, trajectory)


def _author_path(stage: Usd.Stage, trajectory: DeliveryTrajectory) -> None:
    """Author the route as a sampled polyline for visual inspection.

    The certificate consumes the analytic trajectory, not this curve; the curve
    exists so the turn is visible in the viewport.
    """

    UsdGeom.Xform.Define(stage, "/World/Paths")
    points = trajectory.polyline()
    curve = UsdGeom.BasisCurves.Define(stage, "/World/Paths/DeliveryPath")
    curve.CreateTypeAttr(UsdGeom.Tokens.linear)
    curve.CreateWrapAttr(UsdGeom.Tokens.nonperiodic)
    curve.CreateCurveVertexCountsAttr([len(points)])
    curve.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
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
    marker_backing_material = _color_material(stage, "MarkerBacking", (1.0, 1.0, 1.0))

    marker_ids = {
        survey.marker_id for profile in profiles for survey in all_surveys(scenario, profile)
    }
    marker_materials = {marker_id: _marker_material(stage, marker_id) for marker_id in marker_ids}

    physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics.CreateGravityMagnitudeAttr(9.81)
    UsdGeom.Xform.Define(stage, "/World/Environment")
    # Size the ground from the authored extent so it still covers the scene
    # after the next street was added.
    pad = 1.0
    west = -scenario.west_margin_m - pad
    east = scenario.street_east_m + scenario.wall_thickness_m + pad
    south = scenario.street_south_m - pad
    north = max(profile.entry_width_m for profile in profiles) / 2.0
    north += scenario.wall_thickness_m + pad
    _cube(
        stage,
        "/World/Environment/Ground",
        (east - west, north - south, 0.2),
        ((east + west) / 2.0, (north + south) / 2.0, -0.1),
        ground_material,
        collision=True,
    )
    UsdGeom.Xform.Define(stage, "/World/Environment/Corridor")
    _author_shared_actors(stage, scenario, actor_material)
    # The variant set lives on the default prim rather than on the corridor,
    # because a variant only contributes opinions inside its owning prim's
    # namespace. A, P, and the delivery path all move with (m,n), and they sit
    # under /World/Actors and /World/Paths.
    variants = stage.GetPrimAtPath("/World").GetVariantSets().AddVariantSet("corridorProfile")
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
                marker_backing_material,
            )
            _author_profile_actors(stage, scenario, profile, actor_material)
    variants.SetVariantSelection(selected_profile)
    _author_lighting(stage)
    stage.GetRootLayer().Save()
