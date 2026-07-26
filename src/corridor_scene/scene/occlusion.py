"""Continuous camera-visibility certificate and composed-USD raycast audit."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from pxr import Gf, Usd, UsdGeom

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class Coverage:
    """One continuously certified source-path interval."""

    segment: int
    start_fraction: float
    end_fraction: float
    method: str
    witness_x_m: float | None = None


@dataclass(frozen=True)
class Certificate:
    """Machine-readable result of the visibility proof."""

    passed: bool
    profile: str
    coverage: tuple[Coverage, ...]
    uncertified_intervals: tuple[tuple[int, float, float], ...]
    usd_audit_rays: int
    usd_audit_failures: int


def _point(a: Vec3, b: Vec3, fraction: float) -> Vec3:
    result = tuple(a[i] + fraction * (b[i] - a[i]) for i in range(3))
    return result  # type: ignore[return-value]


def _target_vertices(minimum: Vec3, maximum: Vec3) -> tuple[Vec3, ...]:
    return tuple(
        (x, y, z)
        for x in (minimum[0], maximum[0])
        for y in (minimum[1], maximum[1])
        for z in (minimum[2], maximum[2])
    )


def _frustum_excluded(
    source_start: Vec3,
    source_end: Vec3,
    targets: tuple[Vec3, ...],
    heading: tuple[float, float],
    tan_half_fov: float,
    epsilon: float = 1e-8,
) -> bool:
    """Prove a convex source/target product lies outside one frustum plane."""

    forward_x, forward_y = heading
    right_x, right_y = forward_y, -forward_x
    values: list[tuple[float, float]] = []
    for source in (source_start, source_end):
        for target in targets:
            dx = target[0] - source[0]
            dy = target[1] - source[1]
            forward = dx * forward_x + dy * forward_y
            right = dx * right_x + dy * right_y
            values.append((forward, right))
    if max(forward for forward, _ in values) < -epsilon:
        return True
    if min(right - tan_half_fov * forward for forward, right in values) > epsilon:
        return True
    return min(-right - tan_half_fov * forward for forward, right in values) > epsilon


def _wall_witness(
    source_start: Vec3,
    source_end: Vec3,
    targets: tuple[Vec3, ...],
    entry_width_m: float,
    corner_width_m: float,
    corridor_length_m: float,
    wall_thickness_m: float,
    building_height_m: float,
) -> float | None:
    """Find an X plane where every possible sight ray is inside the south wall."""

    source_x_max = max(source_start[0], source_end[0])
    target_x_min = min(point[0] for point in targets)
    low = max(source_x_max + 1e-5, 0.0)
    high = min(target_x_min - 1e-5, corridor_length_m)
    if low >= high:
        return None

    source_y = (source_start[1], source_end[1])
    source_z = (source_start[2], source_end[2])
    target_x = (min(p[0] for p in targets), max(p[0] for p in targets))
    target_y = (min(p[1] for p in targets), max(p[1] for p in targets))
    target_z = (min(p[2] for p in targets), max(p[2] for p in targets))
    candidates = 241
    for index in range(1, candidates):
        q = low + (high - low) * index / candidates
        t_values = [
            (q - sx) / (px - sx)
            for sx in (source_start[0], source_end[0])
            for px in target_x
            if px - sx > 1e-8
        ]
        if not t_values:
            continue
        t_bounds = (min(t_values), max(t_values))
        y_values = [(1.0 - t) * sy + t * py for t in t_bounds for sy in source_y for py in target_y]
        z_values = [(1.0 - t) * sz + t * pz for t in t_bounds for sz in source_z for pz in target_z]
        fraction = q / corridor_length_m
        width = entry_width_m + fraction * (corner_width_m - entry_width_m)
        inner_y = -width / 2.0
        outer_y = inner_y - wall_thickness_m
        margin = 1e-7
        if (
            min(y_values) >= outer_y + margin
            and max(y_values) <= inner_y - margin
            and min(z_values) >= margin
            and max(z_values) <= building_height_m - margin
        ):
            return q
    return None


def continuous_certificate(manifest: dict[str, Any], profile_name: str) -> Certificate:
    """Cover every path parameter using conservative interval enclosures."""

    profile = manifest["profiles"][profile_name]
    actors = manifest["actors"]
    camera_height = float(manifest["camera"]["mount_height_m"])
    path = [
        (float(x), float(y), float(z) + camera_height) for x, y, z in actors["delivery_path_xyz_m"]
    ]
    pmin = tuple(float(value) for value in actors["p_bounds_min_xyz_m"])
    pmax = tuple(float(value) for value in actors["p_bounds_max_xyz_m"])
    targets = _target_vertices(pmin, pmax)  # type: ignore[arg-type]
    target_center = tuple((low + high) / 2.0 for low, high in zip(pmin, pmax, strict=True))
    half_fov = math.radians(float(manifest["camera"]["horizontal_fov_deg"])) / 2.0
    tan_half = math.tan(half_fov)
    coverage: list[Coverage] = []
    failures: list[tuple[int, float, float]] = []

    for segment, (path_start, path_end) in enumerate(zip(path[:-1], path[1:], strict=True)):
        dx = path_end[0] - path_start[0]
        dy = path_end[1] - path_start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            failures.append((segment, 0.0, 1.0))
            continue
        heading = (dx / length, dy / length)

        def cover(
            start_fraction: float,
            end_fraction: float,
            depth: int,
            segment_index: int = segment,
            segment_start: Vec3 = path_start,
            segment_end: Vec3 = path_end,
            segment_heading: tuple[float, float] = heading,
        ) -> None:
            source_start = _point(segment_start, segment_end, start_fraction)
            source_end = _point(segment_start, segment_end, end_fraction)
            if _frustum_excluded(source_start, source_end, targets, segment_heading, tan_half):
                coverage.append(
                    Coverage(segment_index, start_fraction, end_fraction, "frustum_excluded")
                )
                return
            witness = _wall_witness(
                source_start,
                source_end,
                targets,
                float(profile["entry_width_m"]),
                float(profile["corner_width_m"]),
                float(manifest["corridor_length_m"]),
                float(manifest["wall_thickness_m"]),
                float(manifest["building_height_m"]),
            )
            if witness is not None:
                coverage.append(
                    Coverage(segment_index, start_fraction, end_fraction, "south_wall", witness)
                )
                return
            # One demonstrably clear center ray is sufficient to refute the
            # universal no-visibility claim and makes negative controls cheap.
            source_middle = _point(source_start, source_end, 0.5)
            if (
                not _frustum_excluded(
                    source_middle, source_middle, (target_center,), segment_heading, tan_half
                )
                and _wall_witness(
                    source_middle,
                    source_middle,
                    (target_center,),
                    float(profile["entry_width_m"]),
                    float(profile["corner_width_m"]),
                    float(manifest["corridor_length_m"]),
                    float(manifest["wall_thickness_m"]),
                    float(manifest["building_height_m"]),
                )
                is None
            ):
                middle = (start_fraction + end_fraction) / 2.0
                failures.append((segment_index, middle, middle))
                return
            if depth >= 18:
                failures.append((segment_index, start_fraction, end_fraction))
                return
            middle = (start_fraction + end_fraction) / 2.0
            cover(start_fraction, middle, depth + 1)
            cover(middle, end_fraction, depth + 1)

        cover(0.0, 1.0, 0)

    return Certificate(
        passed=not failures,
        profile=profile_name,
        coverage=tuple(coverage),
        uncertified_intervals=tuple(failures),
        usd_audit_rays=0,
        usd_audit_failures=0,
    )


def _mesh_triangles(stage: Usd.Stage, path: str) -> list[tuple[Gf.Vec3d, Gf.Vec3d, Gf.Vec3d]]:
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(path))
    points = mesh.GetPointsAttr().Get()
    counts = mesh.GetFaceVertexCountsAttr().Get()
    indices = mesh.GetFaceVertexIndicesAttr().Get()
    transform = UsdGeom.XformCache().GetLocalToWorldTransform(mesh.GetPrim())
    world = [transform.Transform(Gf.Vec3d(point)) for point in points]
    triangles: list[tuple[Gf.Vec3d, Gf.Vec3d, Gf.Vec3d]] = []
    offset = 0
    for count in counts:
        face = indices[offset : offset + count]
        for index in range(1, count - 1):
            triangles.append((world[face[0]], world[face[index]], world[face[index + 1]]))
        offset += count
    return triangles


def _segment_hits_triangle(
    origin: Vec3, target: Vec3, triangle: tuple[Gf.Vec3d, Gf.Vec3d, Gf.Vec3d]
) -> bool:
    epsilon = 1e-9
    ray = Gf.Vec3d(*(target[index] - origin[index] for index in range(3)))
    edge1 = triangle[1] - triangle[0]
    edge2 = triangle[2] - triangle[0]
    h = Gf.Cross(ray, edge2)
    determinant = Gf.Dot(edge1, h)
    if abs(determinant) < epsilon:
        return False
    inverse = 1.0 / determinant
    s = Gf.Vec3d(*origin) - triangle[0]
    u = inverse * Gf.Dot(s, h)
    if u < 0.0 or u > 1.0:
        return False
    q = Gf.Cross(s, edge1)
    v = inverse * Gf.Dot(ray, q)
    if v < 0.0 or u + v > 1.0:
        return False
    distance_fraction = inverse * Gf.Dot(edge2, q)
    return epsilon < distance_fraction < 1.0 - epsilon


def usd_raycast_audit(
    stage_path: Path, manifest: dict[str, Any], profile_name: str
) -> tuple[int, int]:
    """Sample the composed mesh independently as a diagnostic audit."""

    stage = Usd.Stage.Open(str(stage_path))
    corridor = stage.GetPrimAtPath("/World/Environment/Corridor")
    corridor.GetVariantSets().GetVariantSet("corridorProfile").SetVariantSelection(profile_name)
    triangles: list[tuple[Gf.Vec3d, Gf.Vec3d, Gf.Vec3d]] = []
    for building in ("LeftBuilding", "RightBuilding"):
        triangles.extend(_mesh_triangles(stage, f"/World/Environment/Corridor/{building}"))
    actors = manifest["actors"]
    height = float(manifest["camera"]["mount_height_m"])
    path = [(float(x), float(y), float(z) + height) for x, y, z in actors["delivery_path_xyz_m"]]
    targets = _target_vertices(
        tuple(actors["p_bounds_min_xyz_m"]), tuple(actors["p_bounds_max_xyz_m"])
    )
    tan_half = math.tan(math.radians(float(manifest["camera"]["horizontal_fov_deg"])) / 2.0)
    tested = 0
    failures = 0
    for start, end in zip(path[:-1], path[1:], strict=True):
        dx, dy = end[0] - start[0], end[1] - start[1]
        norm = math.hypot(dx, dy)
        heading = (dx / norm, dy / norm)
        for sample in range(65):
            source = _point(start, end, sample / 64.0)
            for target in targets:
                if _frustum_excluded(source, source, (target,), heading, tan_half):
                    continue
                tested += 1
                if not any(
                    _segment_hits_triangle(source, target, triangle) for triangle in triangles
                ):
                    failures += 1
    return tested, failures


def verify(stage_path: Path, manifest_path: Path, profile_name: str | None = None) -> Certificate:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    profile = profile_name or str(manifest["selected_profile"])
    certificate = continuous_certificate(manifest, profile)
    audited, failed = usd_raycast_audit(stage_path, manifest, profile)
    return Certificate(
        passed=certificate.passed and failed == 0,
        profile=profile,
        coverage=certificate.coverage,
        uncertified_intervals=certificate.uncertified_intervals,
        usd_audit_rays=audited,
        usd_audit_failures=failed,
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
    return 0 if result.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
