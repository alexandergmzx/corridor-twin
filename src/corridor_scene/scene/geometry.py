"""Pure geometric construction shared by USD, manifests, trajectory, and tests.

This module is the single source of truth for the corridor faces. The USD
author, the scenario manifest, the marker survey, the delivery trajectory, and
the visibility checker all derive their numbers from :func:`corridor_faces`, so
the taper equation exists in exactly one place.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .model import CorridorProfile, Scenario

Vec3 = tuple[float, float, float]
Vec2 = tuple[float, float]
Footprint = list[Vec2]


@dataclass(frozen=True)
class MarkerSurvey:
    """World-space survey of one square marker."""

    marker_id: int
    station_m: float
    side: str
    corners_xyz_m: tuple[Vec3, Vec3, Vec3, Vec3]
    normal_xyz: Vec3


@dataclass(frozen=True)
class Occluder:
    """An opaque slab expressed as linear Y bounds over an X interval.

    Between ``x_min`` and ``x_max`` the slab fills
    ``[y_low(x), y_high(x)] x [0, height_m]``. Every occluder used as a
    visibility witness has Y bounds that are linear in X, which lets the
    certificate search for a witness plane in closed form.
    """

    prim_path: str
    x_min: float
    x_max: float
    y_low_intercept: float
    y_low_slope: float
    y_high_intercept: float
    y_high_slope: float
    height_m: float

    def y_bounds(self, x_m: float) -> Vec2:
        """Return the (low, high) Y extent of the slab at plane ``x_m``."""

        return (
            self.y_low_intercept + self.y_low_slope * x_m,
            self.y_high_intercept + self.y_high_slope * x_m,
        )


def corridor_width(profile: CorridorProfile, station_m: float, length_m: float) -> float:
    """Linearly interpolate clear corridor width along its station."""

    fraction = min(max(station_m / length_m, 0.0), 1.0)
    return profile.entry_width_m + fraction * (profile.corner_width_m - profile.entry_width_m)


def corridor_faces(profile: CorridorProfile, station_m: float, length_m: float) -> Vec2:
    """Return the (north, south) inner-face Y coordinates at a station.

    The supplied diagram draws one straight face and one sloping face, so the
    north face is held constant and the south face carries the entire taper.
    """

    north = profile.entry_width_m / 2.0
    return north, north - corridor_width(profile, station_m, length_m)


def corridor_centerline(profile: CorridorProfile, station_m: float, length_m: float) -> float:
    """Return the Y of the corridor centreline at a station.

    Because the taper is one-sided the centreline is not straight: it drifts
    toward the fixed north face as the corridor narrows.
    """

    north, south = corridor_faces(profile, station_m, length_m)
    return (north + south) / 2.0


def _south_face_line(scenario: Scenario, profile: CorridorProfile) -> Vec2:
    """Return the (intercept, slope) of the south inner face over [0, L]."""

    north = profile.entry_width_m / 2.0
    intercept = north - profile.entry_width_m
    slope = (profile.entry_width_m - profile.corner_width_m) / scenario.corridor_length_m
    return intercept, slope


def building_footprints(scenario: Scenario, profile: CorridorProfile) -> dict[str, Footprint]:
    """Return every opaque building footprint, each a convex closed volume.

    Convexity matters: the walls carry ``convexHull`` collision approximations,
    so an L-shaped prim would silently fill the junction that A must drive
    through. The corridor's south wall and the next street's west wall are
    therefore authored as two overlapping convex prims rather than one L.

    Naming note: prior versions authored ``LeftBuilding``/``RightBuilding`` for
    a symmetric taper. Under the one-sided taper those names no longer describe
    the geometry; the mapping is ``LeftBuilding -> NorthBuilding`` and
    ``RightBuilding -> SouthBuilding`` plus the new ``CornerBuilding`` and
    ``EastBuilding``.
    """

    length = scenario.corridor_length_m
    depth = scenario.wall_thickness_m
    west = -scenario.west_margin_m
    north, south_entry = corridor_faces(profile, 0.0, length)
    _, south_corner = corridor_faces(profile, length, length)
    corner_west = length - depth
    _, south_corner_west = corridor_faces(profile, corner_west, length)
    east_inner = scenario.street_east_m
    east_outer = east_inner + depth
    street_south = scenario.street_south_m

    return {
        # Straight face, extended east so it also caps the next street.
        "NorthBuilding": [
            (west, north),
            (east_outer, north),
            (east_outer, north + depth),
            (west, north + depth),
        ],
        # The tapering face the diagram draws as sloping.
        "SouthBuilding": [
            (0.0, south_entry),
            (length, south_corner),
            (length, south_corner - depth),
            (0.0, south_entry - depth),
        ],
        # The opaque corner mass: the next street's west wall, and the volume
        # that hides P from A's camera through the turn.
        "CornerBuilding": [
            (corner_west, south_corner_west),
            (length, south_corner),
            (length, street_south),
            (corner_west, street_south),
        ],
        "EastBuilding": [
            (east_inner, north),
            (east_inner, street_south),
            (east_outer, street_south),
            (east_outer, north),
        ],
    }


def occluders(scenario: Scenario, profile: CorridorProfile) -> tuple[Occluder, ...]:
    """Return the opaque slabs that can block a camera-to-P sight ray.

    Only the corridor's south wall and the corner mass can ever lie between A's
    route and P; the north and east buildings are on the far side of both and
    are excluded here. They are still audited by the independent composed-mesh
    raycast, which reads every building prim from the stage.
    """

    intercept, slope = _south_face_line(scenario, profile)
    depth = scenario.wall_thickness_m
    root = "/World/Environment/Corridor"
    return (
        Occluder(
            prim_path=f"{root}/SouthBuilding",
            x_min=0.0,
            x_max=scenario.corridor_length_m,
            y_low_intercept=intercept - depth,
            y_low_slope=slope,
            y_high_intercept=intercept,
            y_high_slope=slope,
            height_m=scenario.building_height_m,
        ),
        Occluder(
            prim_path=f"{root}/CornerBuilding",
            x_min=scenario.corridor_length_m - depth,
            x_max=scenario.corridor_length_m,
            y_low_intercept=scenario.street_south_m,
            y_low_slope=0.0,
            # Conservative: the corner mass reaches only as high as the south
            # face at that plane, not as high as the face at the corner itself.
            y_high_intercept=intercept,
            y_high_slope=slope,
            height_m=scenario.building_height_m,
        ),
    )


def marker_surveys(scenario: Scenario, profile: CorridorProfile) -> tuple[MarkerSurvey, ...]:
    """Place paired, canted markers on the actual corridor wall faces."""

    spec = scenario.fiducials
    station = spec.first_station_m
    marker_id = 0
    surveys: list[MarkerSurvey] = []
    cant = math.radians(spec.wall_plate_cant_deg)
    half = spec.marker_size_m / 2.0
    while station < scenario.corridor_length_m:
        north_face, south_face = corridor_faces(profile, station, scenario.corridor_length_m)
        for side_name, side_sign, face_y in (
            ("north", 1.0, north_face),
            ("south", -1.0, south_face),
        ):
            normal = (-math.sin(cant), -side_sign * math.cos(cant), 0.0)
            horizontal = (-normal[1], normal[0], 0.0)
            center = (
                station + normal[0] * 0.015,
                face_y + normal[1] * 0.015,
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


def a_start_xyz(scenario: Scenario, profile: CorridorProfile) -> Vec3:
    """Return A's start pose on the corridor centreline at the entry."""

    return (0.0, corridor_centerline(profile, 0.0, scenario.corridor_length_m), 0.0)


def person_b_xyz(scenario: Scenario) -> Vec3:
    """Return B's position, standing on the next street's centreline."""

    return (scenario.street_center_x_m, -scenario.next_street.b_distance_m, 0.0)


def police_bounds(scenario: Scenario, profile: CorridorProfile) -> tuple[Vec3, Vec3]:
    """Return P's axis-aligned body volume, derived from the occluding faces.

    P is placed by offsets from the corner mass and the corridor's south wall
    rather than by absolute coordinates, so a different corridor profile moves
    P with the geometry instead of stranding it inside a wall or in the road.
    """

    police = scenario.police
    size_x, size_y, size_z = police.body_size_xyz_m
    center_x = scenario.corridor_length_m - scenario.wall_thickness_m - police.west_offset_m
    _, south_face = corridor_faces(profile, center_x, scenario.corridor_length_m)
    center_y = south_face - scenario.wall_thickness_m - police.south_offset_m
    return (
        (center_x - size_x / 2.0, center_y - size_y / 2.0, 0.0),
        (center_x + size_x / 2.0, center_y + size_y / 2.0, size_z),
    )


def is_clear(scenario: Scenario, profile: CorridorProfile, x_m: float, y_m: float) -> bool:
    """Return whether a point lies in drivable corridor or next-street space."""

    length = scenario.corridor_length_m
    north, south_entry = corridor_faces(profile, 0.0, length)
    if -scenario.west_margin_m <= x_m < 0.0:
        # The lead-in west of the entry, where only the north wall continues.
        return south_entry < y_m < north
    if 0.0 <= x_m <= length:
        _, south = corridor_faces(profile, x_m, length)
        return south < y_m < north
    if length < x_m <= scenario.street_east_m:
        return scenario.street_south_m < y_m < north
    return False


def validate_layout(scenario: Scenario, profile: CorridorProfile) -> None:
    """Reject a profile-dependent layout that breaks a project invariant.

    ``model.validate_scenario`` covers the profile-independent checks; these
    are the ones that can only be decided once a corridor profile is chosen.
    """

    length = scenario.corridor_length_m
    depth = scenario.wall_thickness_m
    clearance = scenario.police.minimum_clearance_m
    pmin, pmax = police_bounds(scenario, profile)

    # P must stay west of the corner mass, which is what hides it.
    if pmax[0] > length - depth - clearance:
        raise ValueError(
            f"profile {profile.name}: P is within {clearance} m of the corner wall's west face"
        )
    # P must stay south of the corridor's south wall, outside the road.
    for corner_x in (pmin[0], pmax[0]):
        _, south_face = corridor_faces(profile, corner_x, length)
        if pmax[1] > south_face - depth - clearance:
            raise ValueError(
                f"profile {profile.name}: P is within {clearance} m of the south wall at "
                f"x={corner_x:.3f}"
            )
    # P must not stand in drivable space at any of its footprint corners.
    for corner_x in (pmin[0], pmax[0]):
        for corner_y in (pmin[1], pmax[1]):
            if is_clear(scenario, profile, corner_x, corner_y):
                raise ValueError(
                    f"profile {profile.name}: P's body enters drivable space at "
                    f"({corner_x:.3f}, {corner_y:.3f})"
                )

    # B must stand in the authored next street.
    bx, by, _ = person_b_xyz(scenario)
    if not is_clear(scenario, profile, bx, by):
        raise ValueError(f"profile {profile.name}: B does not stand in the next street")
