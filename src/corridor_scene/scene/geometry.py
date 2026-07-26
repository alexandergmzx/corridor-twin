"""Pure geometric construction shared by USD, manifests, and tests."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .model import CorridorProfile, Scenario

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class MarkerSurvey:
    """World-space survey of one square marker."""

    marker_id: int
    station_m: float
    side: str
    corners_xyz_m: tuple[Vec3, Vec3, Vec3, Vec3]
    normal_xyz: Vec3


def corridor_width(profile: CorridorProfile, station_m: float, length_m: float) -> float:
    """Linearly interpolate clear corridor width along its station."""

    fraction = min(max(station_m / length_m, 0.0), 1.0)
    return profile.entry_width_m + fraction * (profile.corner_width_m - profile.entry_width_m)


def building_footprints(
    scenario: Scenario, profile: CorridorProfile
) -> dict[str, list[tuple[float, float]]]:
    """Return north and south wall-volume footprints."""

    length = scenario.corridor_length_m
    depth = scenario.wall_thickness_m
    entry = profile.entry_width_m / 2.0
    corner = profile.corner_width_m / 2.0
    west = -2.0
    north = [
        (west, entry),
        (0.0, entry),
        (length, corner),
        (length, corner + depth),
        (0.0, entry + depth),
        (west, entry + depth),
    ]
    south = [
        (west, -entry),
        (west, -entry - depth),
        (0.0, -entry - depth),
        (length, -corner - depth),
        (length, -corner),
        (0.0, -entry),
    ]
    return {"LeftBuilding": north, "RightBuilding": south}


def marker_surveys(scenario: Scenario, profile: CorridorProfile) -> tuple[MarkerSurvey, ...]:
    """Place paired, canted markers along both corridor walls."""

    spec = scenario.fiducials
    station = spec.first_station_m
    marker_id = 0
    surveys: list[MarkerSurvey] = []
    cant = math.radians(spec.wall_plate_cant_deg)
    half = spec.marker_size_m / 2.0
    while station < scenario.corridor_length_m:
        width = corridor_width(profile, station, scenario.corridor_length_m)
        for side_name, side_sign in (("north", 1.0), ("south", -1.0)):
            normal = (-math.sin(cant), -side_sign * math.cos(cant), 0.0)
            horizontal = (-normal[1], normal[0], 0.0)
            center = (
                station + normal[0] * 0.015,
                side_sign * width / 2.0 + normal[1] * 0.015,
                1.2,
            )
            corners = (
                (
                    center[0] - horizontal[0] * half,
                    center[1] - horizontal[1] * half,
                    center[2] - half,
                ),
                (
                    center[0] + horizontal[0] * half,
                    center[1] + horizontal[1] * half,
                    center[2] - half,
                ),
                (
                    center[0] + horizontal[0] * half,
                    center[1] + horizontal[1] * half,
                    center[2] + half,
                ),
                (
                    center[0] - horizontal[0] * half,
                    center[1] - horizontal[1] * half,
                    center[2] + half,
                ),
            )
            surveys.append(
                MarkerSurvey(
                    marker_id=marker_id,
                    station_m=station,
                    side=side_name,
                    corners_xyz_m=corners,
                    normal_xyz=normal,
                )
            )
            marker_id += 1
        station += spec.spacing_m
    return tuple(surveys)


def camera_path(scenario: Scenario) -> tuple[Vec3, ...]:
    """Lift the delivery path to the camera optical-center height."""

    height = scenario.camera.mount_height_m
    return tuple((x, y, z + height) for x, y, z in scenario.delivery_path_xyz_m)
