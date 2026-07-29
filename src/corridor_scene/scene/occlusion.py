"""Continuous camera-visibility certificate and composed-USD raycast audit.

The supplied task states that the robot cannot see the traffic police. This
module treats that as a hard geometric acceptance gate rather than an assertion,
and keeps two distinct claims separate:

``direct_line_of_sight_blocked``
    An opaque wall lies between A's camera optical centre and P's body. This is
    a reciprocal, orientation-independent property.
``camera_visible``
    Some part of P's body lies inside A's camera frustum *and* is unoccluded.
    This is directional and is the property the written requirement forbids.

An off-screen P is never relabelled as wall-occluded. Coverage is continuous
over arc-length intervals of the shared delivery trajectory, and the turn is
swept as a yaw *range* rather than one heading per polyline segment, because an
instantaneous heading change can otherwise hide a visibility window that a real
rotating camera would pass through.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from .geometry import Occluder
from .trajectory import ARC, DELIVERY_ARC, DeliveryTrajectory, trajectory_from_manifest

Vec3 = tuple[float, float, float]

MAX_DEPTH = 18

# A genuinely visible (or genuinely in-frustum-but-unresolved) region can never
# be resolved by subdividing further -- no witness or exclusion appears no
# matter how small the pieces get, so the search would otherwise run every
# such region to MAX_DEPTH. At binary branching that is up to 2**18 leaf calls
# per region; measured against the visible negative control this took 40.7 s
# and 327,719 coverage entries. A genuine scene resolves in a handful of calls
# (the default profile needs zero subdivision at all), so this budget is slack
# for real geometry and a hard backstop for a region with nothing to find.
DEFAULT_CALL_BUDGET = 4096


@dataclass(frozen=True)
class Coverage:
    """One certified (trajectory interval, sub-volume of P) pair."""

    segment: str
    start_s_m: float
    end_s_m: float
    target_min_xyz_m: Vec3
    target_max_xyz_m: Vec3
    wall_blocked: bool
    frustum_excluded: bool
    blocking_prim: str | None = None
    witness_axis: str | None = None
    witness_coordinate_m: float | None = None


@dataclass(frozen=True)
class WallWitness:
    """One opaque-plane witness, including the coordinate system it uses."""

    prim_path: str
    axis: str
    coordinate_m: float


@dataclass(frozen=True)
class Certificate:
    """Machine-readable result of the visibility proof."""

    passed: bool
    profile: str
    camera_visible_intervals: tuple[tuple[str, float, float], ...]
    line_of_sight_blocked_everywhere: bool
    frustum_only_intervals: tuple[tuple[str, float, float], ...]
    coverage: tuple[Coverage, ...]
    usd_audit_rays: int
    usd_audit_failures: int
    usd_audit_prims: tuple[str, ...]
    nearest_blocking_distance_m: float | None


def _merge_intervals(
    intervals: Any,
) -> tuple[tuple[str, float, float], ...]:
    """Coalesce touching arc-length intervals so reports stay readable.

    Recursive subdivision can emit thousands of adjacent slivers for one real
    region; merging keeps the certificate about the geometry rather than about
    the search.
    """

    merged: list[list[Any]] = []
    for kind, start, end in sorted(intervals):
        if merged and merged[-1][0] == kind and start <= merged[-1][2] + 1e-9:
            merged[-1][2] = max(merged[-1][2], end)
        else:
            merged.append([kind, start, end])
    return tuple((kind, start, end) for kind, start, end in merged)


def _target_vertices(minimum: Vec3, maximum: Vec3) -> tuple[Vec3, ...]:
    """Return all eight corners of P's body volume."""

    return tuple(
        (x, y, z)
        for x in (minimum[0], maximum[0])
        for y in (minimum[1], maximum[1])
        for z in (minimum[2], maximum[2])
    )


def _frustum_excluded_sources(
    sources: tuple[Vec3, ...],
    targets: tuple[Vec3, ...],
    yaw_min: float,
    yaw_max: float,
    tan_half_fov: float,
    epsilon: float = 1e-8,
) -> bool:
    """Prove P lies outside the camera frustum for every yaw in a range.

    ``sources`` are vertices of a convex enclosure of every possible camera
    position in the interval. Each exclusion half-space is linear in the
    source-to-target offset, so enumerating source and target vertices is exact
    for a fixed yaw. For a yaw *range* the same condition must hold at both
    extremes: the half-space constraint is a shifted cosine in yaw, so it is
    positive across an interval shorter than pi exactly when it is positive at
    both ends. The turn here sweeps well under pi.
    """

    def offsets(yaw: float) -> list[tuple[float, float]]:
        forward_x, forward_y = math.cos(yaw), math.sin(yaw)
        right_x, right_y = forward_y, -forward_x
        values: list[tuple[float, float]] = []
        for source in sources:
            for target in targets:
                dx = target[0] - source[0]
                dy = target[1] - source[1]
                values.append(
                    (dx * forward_x + dy * forward_y, dx * right_x + dy * right_y)
                )
        return values

    low, high = offsets(yaw_min), offsets(yaw_max)

    # Entirely behind the image plane.
    if max(f for f, _ in low) < -epsilon and max(f for f, _ in high) < -epsilon:
        return True
    # Entirely beyond the right frustum plane.
    if (
        min(r - tan_half_fov * f for f, r in low) > epsilon
        and min(r - tan_half_fov * f for f, r in high) > epsilon
    ):
        return True
    # Entirely beyond the left frustum plane.
    return (
        min(-r - tan_half_fov * f for f, r in low) > epsilon
        and min(-r - tan_half_fov * f for f, r in high) > epsilon
    )


def _frustum_excluded(
    source_start: Vec3,
    source_end: Vec3,
    targets: tuple[Vec3, ...],
    yaw_min: float,
    yaw_max: float,
    tan_half_fov: float,
    epsilon: float = 1e-8,
) -> bool:
    """Compatibility wrapper for a straight source segment."""

    return _frustum_excluded_sources(
        (source_start, source_end),
        targets,
        yaw_min,
        yaw_max,
        tan_half_fov,
        epsilon,
    )


def _clip(low: float, high: float, coefficient: float, bound: float) -> tuple[float, float]:
    """Narrow ``[low, high]`` by the linear constraint ``coefficient*q <= bound``."""

    if abs(coefficient) < 1e-12:
        return (low, high) if bound >= 0.0 else (1.0, -1.0)
    if coefficient > 0.0:
        return low, min(high, bound / coefficient)
    return max(low, bound / coefficient), high


def _wall_witness_sources(
    sources: tuple[Vec3, ...],
    targets: tuple[Vec3, ...],
    slab: Occluder,
    margin: float = 1e-6,
) -> float | None:
    """Find an X plane where every possible sight ray is inside one opaque slab.

    Solved in closed form rather than sampled. Where a ray crosses the plane
    ``x = q`` its Y and Z are linear in ``q``, and the slab's own bounds are
    linear in ``q`` too, so every containment condition is a linear inequality
    and the feasible planes form an interval. A sampled search misses this: at
    the corridor entry the feasible window is only about 8 mm wide.

    Enumerating the vertices of convex source and target enclosures is exact,
    not sampled. At a separating plane the perspective crossing is
    linear-fractional in each source coordinate with a denominator of fixed
    sign, so its extrema over the enclosure occur at vertices.

    Works in either direction, because on the next street A looks back west
    toward P while in the corridor it looks east.
    """

    source_x = [point[0] for point in sources]
    target_x = [point[0] for point in targets]

    # The witness plane must separate every source point from every target
    # point, in whichever order they happen to lie along X.
    low = max(max(source_x), slab.x_min)
    high = min(min(target_x), slab.x_max)
    if low >= high:
        low = max(max(target_x), slab.x_min)
        high = min(min(source_x), slab.x_max)
        if low >= high:
            return None
    low += margin
    high -= margin

    for source in sources:
        for target in targets:
            span_x = target[0] - source[0]
            if abs(span_x) < 1e-9:
                return None
            # y(q) = y_intercept + y_slope*q, and likewise for z.
            y_slope = (target[1] - source[1]) / span_x
            y_intercept = source[1] - y_slope * source[0]
            z_slope = (target[2] - source[2]) / span_x
            z_intercept = source[2] - z_slope * source[0]

            # y(q) <= y_high(q) - margin
            low, high = _clip(
                low,
                high,
                y_slope - slab.y_high_slope,
                slab.y_high_intercept - y_intercept - margin,
            )
            # y(q) >= y_low(q) + margin
            low, high = _clip(
                low,
                high,
                slab.y_low_slope - y_slope,
                y_intercept - slab.y_low_intercept - margin,
            )
            # 0 + margin <= z(q) <= height - margin
            low, high = _clip(low, high, z_slope, slab.height_m - z_intercept - margin)
            low, high = _clip(low, high, -z_slope, z_intercept - margin)
            if low >= high:
                return None
    return (low + high) / 2.0


def _wall_witness(
    source_start: Vec3,
    source_end: Vec3,
    targets: tuple[Vec3, ...],
    slab: Occluder,
    margin: float = 1e-6,
) -> float | None:
    """Compatibility wrapper for a straight source segment."""

    return _wall_witness_sources((source_start, source_end), targets, slab, margin)


def _wall_witness_crosswise_sources(
    sources: tuple[Vec3, ...],
    targets: tuple[Vec3, ...],
    slab: Occluder,
    margin: float = 1e-6,
) -> float | None:
    """Find a Y plane where every possible sight ray is inside one opaque slab.

    The X-plane witness cannot work where A draws level with P, because no
    plane of constant X separates them there. At those stations A is north of
    the wall and P is south of it, so the wall is crossed side-on and a plane
    of constant Y is the natural witness. Every condition is again linear in
    the plane coordinate, so the feasible planes form an interval.
    """

    source_y = [point[1] for point in sources]
    target_y = [point[1] for point in targets]

    low = max(source_y)
    high = min(target_y)
    if low >= high:
        low = max(target_y)
        high = min(source_y)
        if low >= high:
            return None
    low += margin
    high -= margin

    for source in sources:
        for target in targets:
            span_y = target[1] - source[1]
            if abs(span_y) < 1e-9:
                return None
            # At plane y = q the crossing is at x(q) and z(q), both linear.
            x_slope = (target[0] - source[0]) / span_y
            x_intercept = source[0] - x_slope * source[1]
            z_slope = (target[2] - source[2]) / span_y
            z_intercept = source[2] - z_slope * source[1]

            # The crossing must fall within the slab's X extent.
            low, high = _clip(low, high, x_slope, slab.x_max - x_intercept - margin)
            low, high = _clip(low, high, -x_slope, x_intercept - slab.x_min - margin)
            # y_low(x(q)) + margin <= q <= y_high(x(q)) - margin
            low, high = _clip(
                low,
                high,
                slab.y_low_slope * x_slope - 1.0,
                -slab.y_low_intercept - slab.y_low_slope * x_intercept - margin,
            )
            low, high = _clip(
                low,
                high,
                1.0 - slab.y_high_slope * x_slope,
                slab.y_high_intercept + slab.y_high_slope * x_intercept - margin,
            )
            # margin <= z(q) <= height - margin
            low, high = _clip(low, high, z_slope, slab.height_m - z_intercept - margin)
            low, high = _clip(low, high, -z_slope, z_intercept - margin)
            if low >= high:
                return None
    return (low + high) / 2.0


def _wall_witness_crosswise(
    source_start: Vec3,
    source_end: Vec3,
    targets: tuple[Vec3, ...],
    slab: Occluder,
    margin: float = 1e-6,
) -> float | None:
    """Compatibility wrapper for a straight source segment."""

    return _wall_witness_crosswise_sources(
        (source_start, source_end), targets, slab, margin
    )


def _camera_source_vertices(
    trajectory: DeliveryTrajectory,
    kind: str,
    start_s_m: float,
    end_s_m: float,
) -> tuple[Vec3, ...]:
    """Enclose every camera position in an interval by convex vertices.

    Straight route pieces are represented exactly by their endpoints. A
    circular arc is not contained by its endpoint chord, so the turn uses its
    exact axis-aligned bounds: endpoint angles plus every enclosed cardinal
    angle determine the extrema. The resulting rectangle is deliberately
    conservative and contains the full arc.
    """

    start = trajectory.camera_pose_at(start_s_m)
    end = trajectory.camera_pose_at(end_s_m)
    endpoints = (
        (start.x_m, start.y_m, start.z_m),
        (end.x_m, end.y_m, end.z_m),
    )
    if kind not in {ARC, DELIVERY_ARC}:
        return endpoints

    # Both turns are enclosed the same way; they differ only in where they
    # start along the route, which way the polar angle runs, and which centre
    # they turn about. The delivery turn is left-handed, so its angle rises
    # where the first turn's falls.
    if kind == ARC:
        piece_start_s = trajectory.approach_length_m
        piece_length = trajectory.arc_length_m
        radius = trajectory.arc_radius_m
        center_x, center_y = trajectory.arc_center_xy_m
        start_angle = trajectory.arc_start_angle_rad
        direction = -1.0
    else:
        piece_start_s = (
            trajectory.approach_length_m
            + trajectory.arc_length_m
            + trajectory.departure_length_m
        )
        piece_length = trajectory.delivery_arc_length_m
        radius = trajectory.delivery_arc_radius_m
        center_x, center_y = trajectory.delivery_arc_center_xy_m
        start_angle = trajectory.delivery_arc_start_angle_rad
        direction = 1.0
    if radius <= 0.0 or piece_length <= 0.0:
        return endpoints

    low_s, high_s = sorted((start_s_m, end_s_m))
    low_s = max(low_s, piece_start_s)
    high_s = min(high_s, piece_start_s + piece_length)
    edge_a = start_angle + direction * (low_s - piece_start_s) / radius
    edge_b = start_angle + direction * (high_s - piece_start_s) / radius
    angle_low, angle_high = sorted((edge_a, edge_b))
    angles = [angle_low, angle_high]
    quarter_turn = math.pi / 2.0
    first_cardinal = math.ceil((angle_low - 1e-12) / quarter_turn)
    last_cardinal = math.floor((angle_high + 1e-12) / quarter_turn)
    angles.extend(
        index * quarter_turn for index in range(first_cardinal, last_cardinal + 1)
    )

    xs = [center_x + radius * math.cos(angle) for angle in angles]
    ys = [center_y + radius * math.sin(angle) for angle in angles]
    vertices = tuple(
        (x_m, y_m, start.z_m)
        for x_m in (min(xs), max(xs))
        for y_m in (min(ys), max(ys))
    )
    return tuple(dict.fromkeys(vertices))


def _any_wall_witness(
    sources: tuple[Vec3, ...],
    targets: tuple[Vec3, ...],
    slabs: tuple[Occluder, ...],
) -> WallWitness | None:
    """Return the first slab that blocks every ray, with its witness plane."""

    for slab in slabs:
        coordinate = _wall_witness_sources(sources, targets, slab)
        if coordinate is not None:
            return WallWitness(slab.prim_path, "x", coordinate)
        coordinate = _wall_witness_crosswise_sources(sources, targets, slab)
        if coordinate is not None:
            return WallWitness(slab.prim_path, "y", coordinate)
    return None


def continuous_certificate(
    trajectory: DeliveryTrajectory,
    police_min: Vec3,
    police_max: Vec3,
    slabs: tuple[Occluder, ...],
    horizontal_fov_deg: float,
    profile_name: str,
    call_budget: int = DEFAULT_CALL_BUDGET,
) -> Certificate:
    """Cover every trajectory interval using conservative enclosures.

    ``call_budget`` bounds the total number of ``cover`` invocations across the
    whole search, not just the depth of any one branch. A region that is
    genuinely visible (or genuinely unresolved) never gains a witness or a
    frustum exclusion no matter how far it is subdivided, so without a total
    budget every such region would recurse to ``MAX_DEPTH`` on its own -- up to
    ``2**MAX_DEPTH`` leaves for that region alone. Exhausting the budget is
    treated the same as reaching maximum depth: conservatively unresolved, so
    it counts as visible rather than silently passing.
    """

    tan_half = math.tan(math.radians(horizontal_fov_deg) / 2.0)
    coverage: list[Coverage] = []
    visible: list[tuple[str, float, float]] = []
    calls = 0

    def cover(
        kind: str,
        start_s: float,
        end_s: float,
        target_min: Vec3,
        target_max: Vec3,
        depth: int,
    ) -> None:
        nonlocal calls
        calls += 1
        sources = _camera_source_vertices(trajectory, kind, start_s, end_s)
        yaw_min, yaw_max = trajectory.yaw_range(start_s, end_s)
        targets = _target_vertices(target_min, target_max)

        wall = _any_wall_witness(sources, targets, slabs)
        frustum = _frustum_excluded_sources(sources, targets, yaw_min, yaw_max, tan_half)

        def record(blocked: WallWitness | None) -> None:
            coverage.append(
                Coverage(
                    segment=kind,
                    start_s_m=start_s,
                    end_s_m=end_s,
                    target_min_xyz_m=target_min,
                    target_max_xyz_m=target_max,
                    wall_blocked=blocked is not None,
                    frustum_excluded=frustum,
                    blocking_prim=blocked.prim_path if blocked else None,
                    witness_axis=blocked.axis if blocked else None,
                    witness_coordinate_m=blocked.coordinate_m if blocked else None,
                )
            )

        # Pursue the wall witness even when P is already off-screen. Frustum
        # exclusion alone would satisfy the written requirement, but the
        # stronger reciprocal claim is what makes the scene explainable, so
        # settling for off-screen is a last resort rather than a shortcut:
        # frustum exclusion is only *acted on* once subdivision stops, exactly
        # as before. Stopping on it early once seemed like a safe speedup and
        # was not: it let a region that a little more subdivision would have
        # found a wall witness for settle for the weaker frustum-only answer
        # instead, which fails the stronger bar this certificate holds the
        # default scene to. The call budget below is the actual fix.
        if wall is not None:
            record(wall)
            return
        if depth >= MAX_DEPTH or calls >= call_budget:
            if frustum:
                record(None)
            else:
                visible.append((kind, start_s, end_s))
            return

        # Split whichever of the two enclosures is coarser. Subdividing P's
        # body matters as much as subdividing the route: a single plane cannot
        # contain rays to opposite corners of P inside one 0.5 m wall, even
        # though the wall does block each of them at its own depth.
        span_s = end_s - start_s
        extents = [high - low for low, high in zip(target_min, target_max, strict=True)]
        axis = max(range(3), key=lambda index: extents[index])
        if span_s >= extents[axis]:
            middle = (start_s + end_s) / 2.0
            cover(kind, start_s, middle, target_min, target_max, depth + 1)
            cover(kind, middle, end_s, target_min, target_max, depth + 1)
            return
        split = (target_min[axis] + target_max[axis]) / 2.0
        lower_max = tuple(split if i == axis else v for i, v in enumerate(target_max))
        upper_min = tuple(split if i == axis else v for i, v in enumerate(target_min))
        cover(kind, start_s, end_s, target_min, lower_max, depth + 1)  # type: ignore[arg-type]
        cover(kind, start_s, end_s, upper_min, target_max, depth + 1)  # type: ignore[arg-type]

    for segment in trajectory.segments():
        if segment.end_s_m - segment.start_s_m <= 1e-9:
            continue
        cover(segment.kind, segment.start_s_m, segment.end_s_m, police_min, police_max, 0)

    frustum_only = _merge_intervals(
        (item.segment, item.start_s_m, item.end_s_m)
        for item in coverage
        if not item.wall_blocked
    )
    merged_visible = _merge_intervals(visible)
    return Certificate(
        passed=not merged_visible and not frustum_only,
        profile=profile_name,
        camera_visible_intervals=merged_visible,
        line_of_sight_blocked_everywhere=not frustum_only and not merged_visible,
        frustum_only_intervals=frustum_only,
        coverage=tuple(coverage),
        usd_audit_rays=0,
        usd_audit_failures=0,
        usd_audit_prims=(),
        nearest_blocking_distance_m=None,
    )


def _mesh_triangles(prim: Usd.Prim) -> list[tuple[Gf.Vec3d, Gf.Vec3d, Gf.Vec3d]]:
    mesh = UsdGeom.Mesh(prim)
    points = mesh.GetPointsAttr().Get()
    counts = mesh.GetFaceVertexCountsAttr().Get()
    indices = mesh.GetFaceVertexIndicesAttr().Get()
    transform = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    world = [transform.Transform(Gf.Vec3d(point)) for point in points]
    triangles: list[tuple[Gf.Vec3d, Gf.Vec3d, Gf.Vec3d]] = []
    offset = 0
    for count in counts:
        face = indices[offset : offset + count]
        for index in range(1, count - 1):
            triangles.append((world[face[0]], world[face[index]], world[face[index + 1]]))
        offset += count
    return triangles


def opaque_mesh_prims(stage: Usd.Stage) -> list[Usd.Prim]:
    """Return every solid environment mesh, discovered from the composed stage.

    Selecting by applied collision schema rather than by a hard-coded name list
    means renaming or adding a building cannot silently shrink the audit.
    """

    root = stage.GetPrimAtPath("/World/Environment")
    found: list[Usd.Prim] = []
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI):
            found.append(prim)
    return found


def _segment_hits_triangle(
    origin: Vec3, target: Vec3, triangle: tuple[Gf.Vec3d, Gf.Vec3d, Gf.Vec3d]
) -> float | None:
    """Return the ray fraction of a strictly-interior hit, if any."""

    epsilon = 1e-9
    ray = Gf.Vec3d(*(target[index] - origin[index] for index in range(3)))
    edge1 = triangle[1] - triangle[0]
    edge2 = triangle[2] - triangle[0]
    h = Gf.Cross(ray, edge2)
    determinant = Gf.Dot(edge1, h)
    if abs(determinant) < epsilon:
        return None
    inverse = 1.0 / determinant
    s = Gf.Vec3d(*origin) - triangle[0]
    u = inverse * Gf.Dot(s, h)
    if u < 0.0 or u > 1.0:
        return None
    q = Gf.Cross(s, edge1)
    v = inverse * Gf.Dot(ray, q)
    if v < 0.0 or u + v > 1.0:
        return None
    distance_fraction = inverse * Gf.Dot(edge2, q)
    if epsilon < distance_fraction < 1.0 - epsilon:
        return distance_fraction
    return None


def usd_raycast_audit(
    stage: Usd.Stage,
    trajectory: DeliveryTrajectory,
    police_min: Vec3,
    police_max: Vec3,
    horizontal_fov_deg: float,
    samples: int = 96,
) -> tuple[int, int, tuple[str, ...], float | None]:
    """Sample the composed mesh independently of the analytic certificate."""

    prims = opaque_mesh_prims(stage)
    triangles: list[tuple[Gf.Vec3d, Gf.Vec3d, Gf.Vec3d]] = []
    for prim in prims:
        triangles.extend(_mesh_triangles(prim))

    targets = _target_vertices(police_min, police_max)
    tan_half = math.tan(math.radians(horizontal_fov_deg) / 2.0)
    tested = 0
    failures = 0
    nearest: float | None = None

    for index in range(samples + 1):
        s_m = trajectory.length_m * index / samples
        pose = trajectory.camera_pose_at(s_m)
        source = (pose.x_m, pose.y_m, pose.z_m)
        for target in targets:
            if _frustum_excluded(
                source, source, (target,), pose.yaw_rad, pose.yaw_rad, tan_half
            ):
                continue
            tested += 1
            hits = [
                fraction
                for triangle in triangles
                if (fraction := _segment_hits_triangle(source, target, triangle)) is not None
            ]
            if not hits:
                failures += 1
                continue
            span = math.dist(source, target)
            distance = min(hits) * span
            nearest = distance if nearest is None else min(nearest, distance)

    return tested, failures, tuple(str(prim.GetPath()) for prim in prims), nearest


def verify(stage_path: Path, manifest_path: Path, profile_name: str | None = None) -> Certificate:
    """Prove the visibility requirement for one corridor profile."""

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    profile = profile_name or str(manifest["selected_profile"])
    block = manifest["profiles"][profile]

    police_min = tuple(float(v) for v in block["police_bounds_min_xyz_m"])
    police_max = tuple(float(v) for v in block["police_bounds_max_xyz_m"])
    slabs = tuple(Occluder(**slab) for slab in block["occluders"])
    trajectory = trajectory_from_manifest(block["delivery_trajectory"])
    fov = float(manifest["camera"]["horizontal_fov_deg"])

    certificate = continuous_certificate(
        trajectory,
        police_min,  # type: ignore[arg-type]
        police_max,  # type: ignore[arg-type]
        slabs,
        fov,
        profile,
    )

    stage = Usd.Stage.Open(str(stage_path))
    variants = stage.GetPrimAtPath("/World").GetVariantSets().GetVariantSet("corridorProfile")
    variants.SetVariantSelection(profile)
    audited, failed, prims, nearest = usd_raycast_audit(
        stage,
        trajectory,
        police_min,  # type: ignore[arg-type]
        police_max,  # type: ignore[arg-type]
        fov,
    )

    return Certificate(
        passed=certificate.passed and failed == 0,
        profile=profile,
        camera_visible_intervals=certificate.camera_visible_intervals,
        line_of_sight_blocked_everywhere=certificate.line_of_sight_blocked_everywhere,
        frustum_only_intervals=certificate.frustum_only_intervals,
        coverage=certificate.coverage,
        usd_audit_rays=audited,
        usd_audit_failures=failed,
        usd_audit_prims=prims,
        nearest_blocking_distance_m=nearest,
    )


def parser() -> argparse.ArgumentParser:
    command = argparse.ArgumentParser(description=__doc__)
    command.add_argument("--stage", type=Path, required=True)
    command.add_argument("--manifest", type=Path, required=True)
    command.add_argument("--profile")
    command.add_argument("--out", type=Path, required=True)
    return command


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    result = verify(args.stage, args.manifest, args.profile)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(asdict(result), indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.out}: {'PASS' if result.passed else 'FAIL'}")
    if not result.passed:
        for kind, start, end in result.camera_visible_intervals:
            print(f"  camera can see P on {kind} s=[{start:.3f}, {end:.3f}]")
        for kind, start, end in result.frustum_only_intervals:
            print(
                f"  P is only off-screen, not wall-occluded, on {kind} "
                f"s=[{start:.3f}, {end:.3f}]"
            )
        if result.usd_audit_failures:
            print(f"  {result.usd_audit_failures} composed-mesh audit rays reached P")
    return 0 if result.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
