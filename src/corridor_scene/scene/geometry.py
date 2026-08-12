"""Pure geometric construction shared by USD, manifests, trajectory, and tests.

This module is the single source of truth for the corridor faces. The USD
author, the scenario manifest, the marker survey, the delivery trajectory, and
the visibility checker all derive their numbers from :func:`corridor_faces`, so
the taper equation exists in exactly one place.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .model import (
    MARKER_BACKING_OFFSET_M as _MARKER_BACKING_OFFSET_M,
)
from .model import (
    MARKER_BACKING_SCALE as _MARKER_BACKING_SCALE,
)
from .model import (
    MARKER_WALL_CLEARANCE_M as _MARKER_WALL_CLEARANCE_M,
)
from .model import CorridorProfile, Scenario

Vec3 = tuple[float, float, float]
Vec2 = tuple[float, float]
Footprint = list[Vec2]

# Re-exported from model so scenario validation can use the same constants
# without importing geometry, which would be circular.
MARKER_BACKING_SCALE = _MARKER_BACKING_SCALE
MARKER_BACKING_OFFSET_M = _MARKER_BACKING_OFFSET_M
MARKER_WALL_CLEARANCE_M = _MARKER_WALL_CLEARANCE_M

# A gate marker defines an enforcement station. A reference marker is pose
# evidence only and must never become a gate the robot is measured against.
GATE_ROLE = "gate"
REFERENCE_ROLE = "reference"
MARKER_ROLES = frozenset({GATE_ROLE, REFERENCE_ROLE})


@dataclass(frozen=True)
class MarkerSurvey:
    """World-space survey of one square marker."""

    marker_id: int
    station_m: float
    side: str
    corners_xyz_m: tuple[Vec3, Vec3, Vec3, Vec3]
    normal_xyz: Vec3
    role: str = GATE_ROLE


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

    Because the taper is one-sided the straight centreline is not aligned with
    world X: it drifts toward the fixed north face as the corridor narrows.
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
    ``EastBuilding``. ``EastWallStub`` arrived with ADR 0018 and
    ``CornerScreen`` with ADR 0019.
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
    stub_west, _, stub_south, stub_north = east_wall_stub_bounds(scenario)
    screen_x_min, screen_x_max, screen_y_low, screen_y_high = corner_screen_bounds(
        scenario, profile
    )

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
        # The block the drawing puts on the east wall beside B. It makes the
        # pocket B stands in, and it is why A's route has a delivery turn.
        "EastWallStub": [
            (stub_west, stub_south),
            (east_inner, stub_south),
            (east_inner, stub_north),
            (stub_west, stub_north),
        ],
        # Hangs from the north wall down to just above A's own path, closing
        # the direct view from the approach and the start of the turn into
        # the corner where P now stands. See ADR 0019.
        "CornerScreen": [
            (screen_x_min, screen_y_low),
            (screen_x_max, screen_y_low),
            (screen_x_max, screen_y_high),
            (screen_x_min, screen_y_high),
        ],
    }


def occluders(scenario: Scenario, profile: CorridorProfile) -> tuple[Occluder, ...]:
    """Return the opaque slabs that can block a camera-to-P sight ray.

    Under ADR 0019 P stands *inside* the channel, on the near side of the next
    street's east wall — the same side as A's whole route — so that wall can no
    longer separate the two of them and is kept here only because it is still
    real, audited geometry, not because the analytic proof leans on it. The
    corner screen added by that ADR is what actually hides P from the approach
    and the start of the turn; the corridor's south wall and the corner mass
    stay for the same reason they always have, because they still lie between
    A's corridor run and the corner. The north building remains excluded: it
    caps the scene along the whole north edge, and neither A's route nor P is
    ever on its far side.

    Every building is audited regardless by the independent composed-mesh
    raycast, which discovers prims from the stage by collision schema. These
    slabs are the analytic half of the proof, not the whole of it.
    """

    intercept, slope = _south_face_line(scenario, profile)
    depth = scenario.wall_thickness_m
    north_face, _ = corridor_faces(profile, 0.0, scenario.corridor_length_m)
    screen_x_min, screen_x_max, screen_y_low, screen_y_high = corner_screen_bounds(
        scenario, profile
    )
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
        # The next street's east wall. Not load-bearing under ADR 0019 -- P
        # stands on its near side now -- but kept in the list because it is
        # still real, audited geometry and could yet separate a future
        # placement or profile from A's route.
        Occluder(
            prim_path=f"{root}/EastBuilding",
            x_min=scenario.street_east_m,
            x_max=scenario.street_east_m + depth,
            y_low_intercept=scenario.street_south_m,
            y_low_slope=0.0,
            y_high_intercept=north_face,
            y_high_slope=0.0,
            height_m=scenario.building_height_m,
        ),
        # Closes the direct view from the approach and the first part of the
        # turn into the corner where P stands. This is what hides P under
        # ADR 0019, so it is load-bearing: remove it and the certificate
        # fails on every profile. See ADR 0019.
        Occluder(
            prim_path=f"{root}/CornerScreen",
            x_min=screen_x_min,
            x_max=screen_x_max,
            y_low_intercept=screen_y_low,
            y_low_slope=0.0,
            y_high_intercept=screen_y_high,
            y_high_slope=0.0,
            height_m=scenario.building_height_m,
        ),
    )


def plate_survey(
    anchor_xyz: Vec3,
    wall_normal_xy: Vec2,
    cant_rad: float,
    size_m: float,
    ) -> tuple[Vec3, tuple[Vec3, Vec3, Vec3, Vec3], Vec3]:
    """Return (centre, corners, normal) for one canted plate on a flat surface.

    The standoff solves for the plate's *backing* clearing its host wall: the
    nearest backing edge sits exactly ``MARKER_WALL_CLEARANCE_M`` proud of the
    surface regardless of cant. Both gate and reference plates go through this,
    so one clearance rule exists rather than two that can drift apart.
    """

    half = size_m / 2.0
    cosine = math.cos(cant_rad)
    sine = math.sin(cant_rad)
    normal = (
        cosine * wall_normal_xy[0] - sine * wall_normal_xy[1],
        sine * wall_normal_xy[0] + cosine * wall_normal_xy[1],
        0.0,
    )
    horizontal = (-normal[1], normal[0], 0.0)
    wall_dot_normal = wall_normal_xy[0] * normal[0] + wall_normal_xy[1] * normal[1]
    wall_dot_horizontal = abs(
        wall_normal_xy[0] * horizontal[0] + wall_normal_xy[1] * horizontal[1]
    )
    plate_half = half * MARKER_BACKING_SCALE
    standoff = (
        plate_half * wall_dot_horizontal + MARKER_WALL_CLEARANCE_M
    ) / wall_dot_normal + MARKER_BACKING_OFFSET_M
    center = (
        anchor_xyz[0] + normal[0] * standoff,
        anchor_xyz[1] + normal[1] * standoff,
        anchor_xyz[2],
    )
    corners = tuple(
        (
            center[0] - horizontal[0] * half * along,
            center[1] - horizontal[1] * half * along,
            center[2] + half * up,
        )
        for along, up in ((1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0))
    )
    return center, corners, normal  # type: ignore[return-value]


def plate_backing_corners(
    corners: tuple[Vec3, Vec3, Vec3, Vec3], normal: Vec3
) -> tuple[Vec3, ...]:
    """Return the four world corners of a plate's white backing quad.

    The backing, not the code, is what can overhang the end of a wall, so
    validation and USD authoring have to agree on exactly where it is. Both go
    through here rather than each re-deriving the 9/7 expansion.
    """

    center = tuple(sum(point[axis] for point in corners) / 4.0 for axis in range(3))
    return tuple(
        tuple(
            center[axis]
            + MARKER_BACKING_SCALE * (point[axis] - center[axis])
            - normal[axis] * MARKER_BACKING_OFFSET_M
            for axis in range(3)
        )
        for point in corners
    )


def marker_surveys(scenario: Scenario, profile: CorridorProfile) -> tuple[MarkerSurvey, ...]:
    """Place paired, canted enforcement markers on the corridor wall faces."""

    spec = scenario.fiducials
    station = spec.first_station_m
    marker_id = 0
    surveys: list[MarkerSurvey] = []
    cant = math.radians(spec.wall_plate_cant_deg)
    _, south_slope = _south_face_line(scenario, profile)
    while station < scenario.corridor_length_m:
        north_face, south_face = corridor_faces(profile, station, scenario.corridor_length_m)
        for side_name, side_sign, face_y in (
            ("north", 1.0, north_face),
            ("south", -1.0, south_face),
        ):
            if side_name == "north":
                wall_normal_xy = (0.0, -1.0)
            else:
                magnitude = math.hypot(south_slope, 1.0)
                wall_normal_xy = (-south_slope / magnitude, 1.0 / magnitude)
            _, corners, normal = plate_survey(
                (station, face_y, 1.2),
                wall_normal_xy,
                -side_sign * cant,
                spec.marker_size_m,
            )
            surveys.append(
                MarkerSurvey(
                    marker_id=marker_id,
                    station_m=station,
                    side=side_name,
                    corners_xyz_m=corners,
                    normal_xyz=normal,
                    role=GATE_ROLE,
                )
            )
            marker_id += 1
        station += spec.spacing_m
    return tuple(surveys)


def reference_surveys(scenario: Scenario, profile: CorridorProfile) -> tuple[MarkerSurvey, ...]:
    """Place far-field reference plates that restore coverage near the corner.

    These are pose evidence only. Near the corner the corridor is `n` wide, so
    wall markers sit about n/2 from the centreline and anything two metres ahead
    subtends more than the 37.5 degree half-FOV. The limit is angular, not
    resolution, so coverage past the last wall gate needs targets three to eight
    metres ahead — which means surfaces beyond the corridor's end.

    Two properties matter and both were measured rather than assumed. The two
    host planes are perpendicular, so a frame combining them yields non-coplanar
    correspondences instead of reintroducing the planar-PnP ambiguity. And the
    plates are staggered in height: placed at one height they telescope along
    the receding wall into a contiguous image strip, where each nearer plate
    paints over the farther one's ArUco quiet zone and only one decodes.
    """

    references = scenario.fiducials.references
    length = scenario.corridor_length_m
    north_face, _ = corridor_faces(profile, 0.0, length)
    # The strip of east face visible through the corridor mouth runs from the
    # corner mass's north edge up to the north wall, so it is `n` tall and its
    # *position* shifts north with m/2 - n while its height does not depend on
    # m at all. An absolute `along_m` therefore falls out of the band on a wide
    # entry with a narrow corner even though the band is the same size, which
    # made m = 8.0, n = 3.0 unbuildable for no geometric reason.
    #
    # Clamping to the band floor rather than re-anchoring to it is what keeps
    # the configured profiles fixed: their floors all sit below the configured
    # coordinate, two of them negative, so nothing already measured moves.
    _, corner_south_face = corridor_faces(profile, length, length)
    surveys: list[MarkerSurvey] = []
    for index, spec in enumerate(references.plates):
        if spec.surface == "north_wall":
            anchor = (spec.along_m, north_face, spec.height_m)
            wall_normal_xy = (0.0, -1.0)
        elif spec.surface == "east_face":
            # Two clearances, not one. `validate_layout` requires the backing to
            # clear the corner mass by one; a clamp that lands exactly on that
            # threshold leaves the outcome to floating-point rounding, which is
            # not a thing to decide whether a scene builds.
            band_floor = (
                corner_south_face
                + spec.size_m / 2.0 * MARKER_BACKING_SCALE
                + 2.0 * MARKER_WALL_CLEARANCE_M
            )
            anchor = (scenario.street_east_m, max(spec.along_m, band_floor), spec.height_m)
            wall_normal_xy = (-1.0, 0.0)
        else:
            raise ValueError(f"unknown reference surface {spec.surface!r}")
        _, corners, normal = plate_survey(
            anchor, wall_normal_xy, -math.radians(spec.cant_deg), spec.size_m
        )
        surveys.append(
            MarkerSurvey(
                marker_id=references.id_base + index,
                station_m=anchor[0],
                side=spec.surface,
                corners_xyz_m=corners,
                normal_xyz=normal,
                role=REFERENCE_ROLE,
            )
        )
    return tuple(surveys)


def all_surveys(scenario: Scenario, profile: CorridorProfile) -> tuple[MarkerSurvey, ...]:
    """Return every surveyed plate, gates first, with unique marker ids."""

    surveys = marker_surveys(scenario, profile) + reference_surveys(scenario, profile)
    identifiers = [survey.marker_id for survey in surveys]
    if len(set(identifiers)) != len(identifiers):
        raise ValueError("marker ids must be unique across gate and reference plates")
    return surveys


def a_start_xyz(scenario: Scenario, profile: CorridorProfile) -> Vec3:
    """Return A's start pose on the corridor centreline at the entry."""

    return (0.0, corridor_centerline(profile, 0.0, scenario.corridor_length_m), 0.0)


def east_wall_stub_bounds(scenario: Scenario) -> tuple[float, float, float, float]:
    """Return the stub's ``(x_min, x_max, y_min, y_max)`` footprint.

    The drawing puts an unlabelled block on the street's east wall beside B.
    Its depth transfers as a share of the clear street width; its position
    along the street does not transfer at all, so it is pinned to B, which is
    the one relationship the drawing does fix -- B stands immediately south of
    it. See ADR 0018.
    """

    stub = scenario.next_street.east_wall_stub
    depth = scenario.next_street.clear_width_m * stub.depth_fraction
    south = -scenario.next_street.b_distance_m + stub.gap_north_of_b_m
    return (scenario.street_east_m - depth, scenario.street_east_m, south, south + stub.length_m)


def street_drive_center_x_m(scenario: Scenario) -> float:
    """Return the centreline of the lane the stub leaves clear.

    A drives the middle of what is actually drivable, not the street's
    geometric middle. With the stub occupying the eastern share of the street,
    those are different, and the geometric middle would put A into the wall.
    """

    stub_west, _, _, _ = east_wall_stub_bounds(scenario)
    return (scenario.street_west_m + stub_west) / 2.0


def landmark_xyz(scenario: Scenario) -> Vec3:
    """Where B's lidar-detectable post stands.

    BESIDE B, SOUTH along the street. Three constraints fix that, and every one
    of them was found by a wrong answer rather than by reasoning.

    **Not toward the street.** The post is lidar-visible for exactly the reason
    B is, so offsetting it that way puts an obstacle on A's approach line,
    between A and its goal.

    **Not north.** The east wall stub sits immediately north of B by
    construction -- that is what `gap_north_of_b_m` means -- so a post offset
    north lands INSIDE it. Measured: `is_clear` False at (5.038, -1.800)
    against a stub spanning y -2.085 to -1.767. South is the open side, and the
    street runs 6 m that way.

    **Far enough to be its own cluster.** The offset must exceed B's half-width
    plus the post's radius plus the detector's clustering gap, or the two merge
    and the circle fit sees a box with a bump instead of a cylinder. At the
    current scale that floor is 0.0675 + 0.12 + 0.36 = 0.548 m, and 0.6 clears
    it.

    `validate_layout` checks the result, because two of the three mistakes above
    shipped in a scene that nothing objected to.

    One function, so the authored prop and the manifest the detector reads can
    never describe two different places.
    """

    b_x, b_y, b_z = person_b_xyz(scenario)
    return (b_x, b_y - scenario.actors.landmark_offset_m, b_z)


def person_b_xyz(scenario: Scenario) -> Vec3:
    """Return B's position in the pocket behind the east-wall stub.

    The drawing centres B's label 0.80 of the way across the street, which is
    inside the stub's shadow rather than out in the lane, and one stub-length
    south of it. B therefore stands against the east wall and A has to turn in
    behind the stub to reach it -- which is why the route gained its delivery
    turn. See ADR 0018.
    """

    street = scenario.next_street
    lateral = scenario.street_west_m + street.clear_width_m * street.b_lateral_fraction
    return (lateral, -street.b_distance_m, 0.0)


def police_bounds(scenario: Scenario, profile: CorridorProfile) -> tuple[Vec3, Vec3]:
    """Return P's axis-aligned body volume inside the next street's east side.

    The supplied diagram places P's label in the open channel, roughly level
    with the corridor and close to the east wall — not beyond that wall's
    outer face. ADR 0019 records why the body now stands on the wall's *inner*
    side, superseding ADR 0017's placement on its far side.

    P is placed by offsets from wall faces rather than by absolute
    coordinates, so a different corridor profile moves P with the geometry
    instead of stranding it inside a wall or in the road.
    ``east_wall_clearance_m`` is clear air west of the east wall's inner face,
    so the body stands inside the channel; ``north_offset_m`` runs south from
    the north wall's inner face, which is the only one of the two that varies
    with the profile, because it sits at ``m/2``.
    """

    police = scenario.police
    size_x, size_y, size_z = police.body_size_xyz_m
    center_x = scenario.street_east_m - police.east_wall_clearance_m - size_x / 2.0
    north_face, _ = corridor_faces(profile, 0.0, scenario.corridor_length_m)
    center_y = north_face - police.north_offset_m
    return (
        (center_x - size_x / 2.0, center_y - size_y / 2.0, 0.0),
        (center_x + size_x / 2.0, center_y + size_y / 2.0, size_z),
    )


# How far above the corridor centreline's height at the corner the screen's
# south face sits. That centreline is a safe, closed-form proxy for the
# highest point A's own path reaches there -- not the true peak (measured
# 1.736 m including the driving margin, against a proxy of 1.5 m for the
# nominal profile), which needs the full trajectory solve this module cannot
# reach without an import cycle. This margin was picked to clear the measured
# margin envelope on every authored and requested profile checked (0.10-0.38 m
# of clearance up to m=10), and it is a proxy: ``validate_layout`` still
# checks the built profile's actual route against the screen via ``is_clear``
# rather than trusting this number alone.
# MOVED TO THE SCENARIO YAML (geometry.corner_screen.north_margin_m). It is a
# dimension of the scene, so it must scale with the scene; as a constant here
# it stayed 0.4 m in a corridor whose corner is 0.9 m wide.
# (see geometry.corner_screen in the scenario YAML)

# The screen only has to separate P from a source that is already close to it
# in X -- a ray from anywhere on the approach or the early turn crosses the
# screen's height band within well under a metre of P's own west edge, because
# the band sits close to P's own height and the crossing point is pulled
# toward whichever end of the ray is closer to that height. A source far to
# the west still gets caught: the geometry does not need the screen to reach
# back to meet it. Measured: 0.1 m already holds visible=() on the nominal
# profile; this is a wider, still-narrow margin, and it keeps clear of the
# north-wall reference plates at along_m 13 and 15 (and 17, moved to 15.6 by
# this same ADR), which a screen reaching further west would otherwise occlude.
# MOVED TO THE SCENARIO YAML (geometry.corner_screen.width_m), same reason.
# (see geometry.corner_screen in the scenario YAML)


def corner_screen_bounds(
    scenario: Scenario, profile: CorridorProfile
) -> tuple[float, float, float, float]:
    """Return the corner screen's ``(x_min, x_max, y_low, y_high)`` footprint.

    P now stands inside the channel (ADR 0019), so nothing separates it from
    A's approach and the first part of its turn into the street — both look
    straight at the corner P occupies. This wall closes that view: its south
    face sits just above the highest point A's own path reaches there (see
    ``CORNER_SCREEN_NORTH_MARGIN_M``), and its east face stops just short of
    P's own body.

    Its north face reaches the true north wall, so it reads as an attached
    partition rather than a floating panel. An earlier revision stopped 0.3 m
    short, at P's own north edge, to keep clear of the north-wall reference
    plates surveyed in this same x-range; those plates were relocated west of
    this screen's x-range entirely as part of the same ADR, so that clearance
    stopped doing any work and the gap to the wall was cosmetic debt rather
    than a live constraint. Confirmed empirically before closing it: every
    authored profile's occlusion certificate and reference-plate coverage are
    unaffected by reaching the wall.

    It does not need to reach the whole way to the east wall either: once A
    turns onto the lane it drives with its back to the corner, so the
    departure, delivery-arc and delivery legs are excluded by camera frustum
    instead -- a materially different, and separately reported, way of not
    being seen.
    """

    length = scenario.corridor_length_m
    centerline_at_corner = corridor_centerline(profile, length, length)
    north_face, _ = corridor_faces(profile, 0.0, length)
    police_min, _ = police_bounds(scenario, profile)
    x_max = police_min[0] - scenario.police.minimum_clearance_m
    return (
        max(0.0, x_max - scenario.corner_screen.width_m),
        x_max,
        centerline_at_corner + scenario.corner_screen.north_margin_m,
        north_face,
    )


def is_clear(scenario: Scenario, profile: CorridorProfile, x_m: float, y_m: float) -> bool:
    """Return whether a point lies in drivable corridor or next-street space."""

    length = scenario.corridor_length_m
    north, south_entry = corridor_faces(profile, 0.0, length)
    screen_x_min, screen_x_max, screen_y_low, screen_y_high = corner_screen_bounds(
        scenario, profile
    )
    if screen_x_min <= x_m <= screen_x_max and screen_y_low <= y_m <= screen_y_high:
        # The corner screen is solid too, so nothing -- the route validator
        # included -- may treat its footprint as open floor.
        return False
    if -scenario.west_margin_m <= x_m < 0.0:
        # The lead-in west of the entry, where only the north wall continues.
        return south_entry < y_m < north
    if 0.0 <= x_m <= length:
        _, south = corridor_faces(profile, x_m, length)
        return south < y_m < north
    if length < x_m <= scenario.street_east_m:
        if not (scenario.street_south_m < y_m < north):
            return False
        # The stub is solid, so the street is not clear through it.
        stub_west, stub_east, stub_south, stub_north = east_wall_stub_bounds(scenario)
        return not (stub_west <= x_m <= stub_east and stub_south <= y_m <= stub_north)
    return False


def validate_layout(scenario: Scenario, profile: CorridorProfile) -> None:
    """Reject a profile-dependent layout that breaks a project invariant.

    ``model.validate_scenario`` covers the profile-independent checks; these
    are the ones that can only be decided once a corridor profile is chosen.
    """

    length = scenario.corridor_length_m
    clearance = scenario.police.minimum_clearance_m
    pmin, pmax = police_bounds(scenario, profile)

    # P must stay west of the east wall's inner face, which is the side the
    # supplied diagram measures P's label on -- see ADR 0019.
    if pmax[0] > scenario.street_east_m - clearance:
        raise ValueError(
            f"profile {profile.name}: P reaches x={pmax[0]:.3f}, within {clearance} m of the "
            f"east wall's inner face at x={scenario.street_east_m:.3f}"
        )
    # P must stay south of the north wall's inner face, or the corner screen
    # (which hangs from that same face) would not cover its whole body.
    north_face, _ = corridor_faces(profile, 0.0, length)
    if pmax[1] > north_face - clearance:
        raise ValueError(
            f"profile {profile.name}: P reaches y={pmax[1]:.3f}, within {clearance} m of the "
            f"north face at y={north_face:.3f}"
        )
    # And south of the street's far end, for the same reason at the other end.
    if pmin[1] < scenario.street_south_m + clearance:
        raise ValueError(
            f"profile {profile.name}: P reaches y={pmin[1]:.3f}, past the street's south end"
        )

    # B's post must stand in FREE SPACE, footprint and all.
    #
    # This check exists because the post shipped inside a wall twice and nothing
    # objected: offset north it landed in the east wall stub (`is_clear` False at
    # (5.038, -1.800) against a stub spanning y -2.085 to -1.767), and the prop's
    # size stopped scaling with the scene while its neighbours kept scaling. A
    # landmark inside geometry is worse than no landmark -- the detector cannot
    # see it, and the delivery has a marker that is not there.
    post_x, post_y, _ = landmark_xyz(scenario)
    radius = scenario.actors.landmark_radius_m
    for index in range(12):
        bearing = index * math.pi / 6.0
        probe_x = post_x + radius * math.cos(bearing)
        probe_y = post_y + radius * math.sin(bearing)
        if not is_clear(scenario, profile, probe_x, probe_y):
            raise ValueError(
                f"profile {profile.name}: B's landmark post at "
                f"({post_x:.3f}, {post_y:.3f}) r={radius} is not in free space -- "
                f"blocked at bearing {math.degrees(bearing):.0f} deg"
            )

    # And far enough from B that the detector sees two objects rather than one.
    # Closer than this they merge into a single cluster and the circle fit is
    # given a box with a bump on it.
    half_width = max(scenario.actors.b_size_xyz_m[0], scenario.actors.b_size_xyz_m[1]) / 2.0
    cluster_gap = radius * 2.0 * 1.5
    floor = half_width + radius + cluster_gap
    separation = math.dist((post_x, post_y), person_b_xyz(scenario)[:2])
    if separation < floor:
        raise ValueError(
            f"profile {profile.name}: B's post is {separation:.3f} m from B, inside the "
            f"{floor:.3f} m the detector needs to cluster them apart"
        )
    # P's body must not overlap the corner screen that hides it. Guaranteed by
    # construction -- corner_screen_bounds derives its east face from P's own
    # west edge -- but checked directly rather than trusted, since the two are
    # independently configurable in principle.
    screen_x_min, screen_x_max, screen_y_low, screen_y_high = corner_screen_bounds(
        scenario, profile
    )
    if pmin[0] < screen_x_max and pmax[1] > screen_y_low and pmin[1] < screen_y_high:
        raise ValueError(
            f"profile {profile.name}: P's body overlaps the corner screen at "
            f"x<{screen_x_max:.3f}"
        )

    # Every reference plate must sit entirely inside the face it is mounted on,
    # for *this* profile. Two things make that profile-dependent and easy to get
    # wrong. The east face spans y from the next street's south edge up to the
    # north wall at m/2, so a narrower requested corridor shortens it. And what
    # overhangs first is the backing, MARKER_BACKING_SCALE larger than the code,
    # so checking the plate centre or even the code corners passes a plate whose
    # quiet zone is already buried in the adjoining building.
    north_face, _ = corridor_faces(profile, 0.0, length)
    plates = scenario.fiducials.references.plates
    for survey, spec in zip(reference_surveys(scenario, profile), plates, strict=True):
        if spec.surface == "north_wall":
            # Past the east face the north wall is inside EastBuilding.
            axis, low, high = 0, -scenario.west_margin_m, scenario.street_east_m
            span = "north wall"
        else:
            axis, low, high = 1, scenario.street_south_m, north_face
            span = "east face"
        backing = plate_backing_corners(survey.corners_xyz_m, survey.normal_xyz)
        reach_low = min(point[axis] for point in backing)
        reach_high = max(point[axis] for point in backing)
        if reach_low < low or reach_high > high:
            raise ValueError(
                f"profile {profile.name}: reference marker {survey.marker_id} backing spans "
                f"[{reach_low:.4f}, {reach_high:.4f}] and leaves the {span}, which spans "
                f"[{low:.4f}, {high:.4f}]"
            )
        if spec.surface != "east_face":
            continue
        # Being *on* the east face is not enough: it also has to be visible
        # through the corridor mouth. The corner mass reaches north to the south
        # face at x = L, so anything south of that line is behind it from every
        # position inside the corridor -- both faces are straight, so the mouth
        # is the only binding plane. This is profile-dependent in the sharpest
        # way available, because the edge sits at m/2 - n: on the default
        # profile it is exactly y = 0.0. A plate centred there renders complete
        # in a projection-only camera while a real view of it is cut in half.
        _, corner_south_face = corridor_faces(profile, length, length)
        if reach_low < corner_south_face + MARKER_WALL_CLEARANCE_M:
            raise ValueError(
                f"profile {profile.name}: reference marker {survey.marker_id} reaches south to "
                f"y={reach_low:.4f} and is behind the corner mass, whose north edge is at "
                f"y={corner_south_face:.4f}"
            )

    # B must stand in the authored next street, and specifically in the pocket
    # the stub makes rather than inside the stub itself.
    bx, by, _ = person_b_xyz(scenario)
    if not is_clear(scenario, profile, bx, by):
        raise ValueError(f"profile {profile.name}: B does not stand in the next street")
    stub_west, stub_east, stub_south, stub_north = east_wall_stub_bounds(scenario)
    if stub_south <= by <= stub_north and bx >= stub_west:
        raise ValueError(
            f"profile {profile.name}: B at ({bx:.3f}, {by:.3f}) is inside the east-wall stub"
        )
    if by > stub_south:
        raise ValueError(
            f"profile {profile.name}: B at y={by:.3f} is north of the stub's south face at "
            f"y={stub_south:.3f}; the drawing puts B in the pocket south of it"
        )
    # The stub must leave a lane, and that lane must reach past B's line so the
    # delivery turn has somewhere to happen.
    if stub_west <= scenario.street_west_m:
        raise ValueError(
            f"profile {profile.name}: the east-wall stub spans the whole street, leaving no lane"
        )
