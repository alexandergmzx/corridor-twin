"""Typed scenario configuration used by every generated artifact."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

Vec3 = tuple[float, float, float]

# A 5x5 ArUco code has one black border module. One additional white module on
# every side makes the physical plate 9/7 the code size. The backing sits just
# behind the textured quad, and its nearest edge retains this wall clearance.
# These live here so scenario validation and geometry share one definition.
MARKER_BACKING_SCALE = 9.0 / 7.0
MARKER_BACKING_OFFSET_M = 0.002
MARKER_WALL_CLEARANCE_M = 0.015


@dataclass(frozen=True)
class CorridorProfile:
    """One finite USD corridor variant."""

    name: str
    entry_width_m: float
    corner_width_m: float


@dataclass(frozen=True)
class CameraSpec:
    """Pinhole camera and publishing budget."""

    frame_id: str
    width_px: int
    height_px: int
    rate_hz: float
    horizontal_fov_deg: float
    mount_height_m: float


@dataclass(frozen=True)
class ReferencePlateSpec:
    """One far-field reference plate on a named flat surface."""

    surface: str
    along_m: float
    height_m: float
    size_m: float
    cant_deg: float


@dataclass(frozen=True)
class ReferenceFiducialSpec:
    """Reference plates and the id range reserved for them."""

    id_base: int
    plates: tuple[ReferencePlateSpec, ...]


@dataclass(frozen=True)
class FiducialSpec:
    """Surveyed marker layout parameters."""

    dictionary: str
    marker_size_m: float
    first_station_m: float
    spacing_m: float
    wall_plate_cant_deg: float
    references: ReferenceFiducialSpec


@dataclass(frozen=True)
class EastWallStubSpec:
    """The block the drawing puts on the street's east wall beside B.

    ``depth_fraction`` is the share of the clear street width it occupies,
    measured from the drawing. It is a fraction rather than a length because the
    drawing's own scale and the scene's chosen street width disagree, and the
    fraction is the part that decides whether A can still get past. See
    ADR 0018.
    """

    depth_fraction: float
    length_m: float
    gap_north_of_b_m: float


@dataclass(frozen=True)
class CornerScreenSpec:
    """ADR 0019's partition, which hides P from A's approach and early turn.

    Authored in the scenario YAML rather than as constants in `geometry.py`,
    because these are dimensions of the SCENE and must move when it is scaled.
    They did not: at the committed 0.30 factor a 0.4 m north margin was still
    0.4 m in a corridor whose corner is 0.9 m wide, which put the screen most of
    a half-width north of where it belonged. The occlusion certificate then
    failed on the scenario that runs -- P visible for the whole approach and the
    first metre of the arc -- while passing on the authored one, which was the
    only scene anything built.
    """

    north_margin_m: float
    width_m: float


@dataclass(frozen=True)
class NextStreetSpec:
    """The perpendicular street A turns onto to reach B."""

    clear_width_m: float
    length_m: float
    turn_radius_m: float
    b_distance_m: float
    b_lateral_fraction: float
    east_wall_stub: EastWallStubSpec


@dataclass(frozen=True)
class PoliceSpec:
    """P's body and its standoff from the walls that hide it.

    Offsets are measured from wall faces rather than stored as absolute
    coordinates so that P follows the geometry when a different corridor
    profile is selected. ``east_wall_clearance_m`` is measured west from the
    next street's east wall's **inner** face, so P stands inside the clear
    channel rather than beyond the wall's outer face, and
    ``north_offset_m`` south from the north wall's inner face at ``m/2``. See
    ADR 0019, which supersedes ADR 0017's placement on the opposite side of
    that same wall.
    """

    body_size_xyz_m: Vec3
    east_wall_clearance_m: float
    north_offset_m: float
    minimum_clearance_m: float


@dataclass(frozen=True)
class ActorSpec:
    """Body sizes of the scenario's actors.

    Config-driven rather than hardcoded so they move with the scenario scale.
    B in particular is not decoration: the RTX lidar sees render geometry, so
    B's footprint is an obstacle in the costmap and it sets the standoff the
    delivery goal has to keep.

    Since ADR 0031 B **is** the cylinder -- one object at the delivery point,
    rather than a box for the eye with a detectable post beside it.
    """

    #: B's radius, and the detector's single source of truth for the circle it
    #: fits: it reaches the detector through the manifest so the expected size
    #: and the authored size can never drift apart into two literals. ABSOLUTE
    #: metres -- sized for the MS200, not for the scenario (scale_scenario's
    #: NOT_LENGTHS).
    b_radius_m: float
    #: B's height, which describes a person and therefore DOES scale.
    b_height_m: float
    a_size_xyz_m: Vec3


@dataclass(frozen=True)
class Scenario:
    """Validated, simulator-independent scenario model."""

    schema_version: str
    provenance: dict[str, Any]
    taper_mode: str
    corridor_length_m: float
    building_height_m: float
    wall_thickness_m: float
    west_margin_m: float
    corner_screen: CornerScreenSpec
    next_street: NextStreetSpec
    profiles: tuple[CorridorProfile, ...]
    default_profile: str
    camera: CameraSpec
    fiducials: FiducialSpec
    actors: ActorSpec
    police: PoliceSpec
    speed_policy: dict[str, Any]

    def profile(self, name: str) -> CorridorProfile:
        """Return a profile by name or fail with a useful error."""

        for profile in self.profiles:
            if profile.name == name:
                return profile
        available = ", ".join(profile.name for profile in self.profiles)
        raise ValueError(f"unknown corridor profile {name!r}; available: {available}")

    @property
    def street_west_m(self) -> float:
        """X of the next street's west edge, which is the corridor's east end."""

        return self.corridor_length_m

    @property
    def street_east_m(self) -> float:
        """X of the next street's east kerb."""

        return self.corridor_length_m + self.next_street.clear_width_m

    @property
    def street_center_x_m(self) -> float:
        """X of the next street's centreline, which A drives down to reach B."""

        return self.corridor_length_m + self.next_street.clear_width_m / 2.0

    @property
    def street_south_m(self) -> float:
        """Y of the next street's far end."""

        return -self.next_street.length_m


def authored_config_path() -> Path:
    """The scenario as authored from the supplied drawing, at its own scale.

    The source of record for every RATIO in this project. It is not what runs:
    the corridor is 12 m long here, sized for nothing in particular, and robot1
    is 0.20 x 0.16 m.
    """

    return Path(__file__).resolve().parents[1] / "config" / "corridor.yaml"


def default_config_path() -> Path:
    """The scenario AS RUN: the authored one, scaled to the robot.

    This is the default because a default that nothing runs is a trap. It was
    the authored 12 m scene, so `scene.build` with no `--config` wrote a 12 m
    stage and manifest into `out/`, and the composer -- which defaults to those
    same paths -- built its arenas from them. The runner then defaulted to the
    same 12 m manifest, and only an exported CORRIDOR_MANIFEST made a run plan
    at the scale it was actually driving.

    It did not, on 2026-08-12: every corridor run since the rescale drove a
    0.30-scale plan inside a 1.0-scale arena. The goal sat about twelve metres
    short of B, which is the whole of that run's 5.754 m delivery error and of
    a landmark "confirmed" in a stage that contains no post.

    `authored_config_path()` is still there, still the source of record, and
    `--config` still takes either.
    """

    return Path(__file__).resolve().parents[1] / "config" / "corridor-robot-scale.yaml"


def _xyz(value: Any, name: str) -> Vec3:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must contain three coordinates")
    return (float(value[0]), float(value[1]), float(value[2]))


def load_scenario(path: Path | None = None) -> Scenario:
    """Load and validate the scenario YAML."""

    source = path or default_config_path()
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    geometry = raw["geometry"]
    profile_values = geometry["profiles"]
    profiles = tuple(
        CorridorProfile(
            name=name,
            entry_width_m=float(values["entry_width_m"]),
            corner_width_m=float(values["corner_width_m"]),
        )
        for name, values in profile_values.items()
    )
    default_names = [name for name, values in profile_values.items() if values.get("default")]
    if len(default_names) != 1:
        raise ValueError("exactly one corridor profile must be marked default")

    camera_raw = raw["camera"]
    if int(camera_raw["count"]) != 1:
        raise ValueError("the VRAM budget permits exactly one camera")
    camera = CameraSpec(
        frame_id=str(camera_raw["frame_id"]),
        width_px=int(camera_raw["width_px"]),
        height_px=int(camera_raw["height_px"]),
        rate_hz=float(camera_raw["rate_hz"]),
        horizontal_fov_deg=float(camera_raw["horizontal_fov_deg"]),
        mount_height_m=float(camera_raw["mount_height_m"]),
    )
    fiducial_raw = raw["fiducials"]
    reference_raw = fiducial_raw["references"]
    fiducials = FiducialSpec(
        dictionary=str(fiducial_raw["dictionary"]),
        marker_size_m=float(fiducial_raw["marker_size_m"]),
        first_station_m=float(fiducial_raw["first_station_m"]),
        spacing_m=float(fiducial_raw["spacing_m"]),
        wall_plate_cant_deg=float(fiducial_raw["wall_plate_cant_deg"]),
        references=ReferenceFiducialSpec(
            id_base=int(reference_raw["id_base"]),
            plates=tuple(
                ReferencePlateSpec(
                    surface=str(plate["surface"]),
                    along_m=float(plate["along_m"]),
                    height_m=float(plate["height_m"]),
                    size_m=float(plate["size_m"]),
                    cant_deg=float(plate["cant_deg"]),
                )
                for plate in reference_raw["plates"]
            ),
        ),
    )
    street_raw = geometry["next_street"]
    stub_raw = street_raw["east_wall_stub"]
    next_street = NextStreetSpec(
        clear_width_m=float(street_raw["clear_width_m"]),
        length_m=float(street_raw["length_m"]),
        turn_radius_m=float(street_raw["turn_radius_m"]),
        b_distance_m=float(street_raw["b_distance_m"]),
        b_lateral_fraction=float(street_raw["b_lateral_fraction"]),
        east_wall_stub=EastWallStubSpec(
            depth_fraction=float(stub_raw["depth_fraction"]),
            length_m=float(stub_raw["length_m"]),
            gap_north_of_b_m=float(stub_raw["gap_north_of_b_m"]),
        ),
    )
    actors_raw = raw["actors"]
    actors = ActorSpec(
        b_radius_m=float(actors_raw["b_radius_m"]),
        b_height_m=float(actors_raw["b_height_m"]),
        a_size_xyz_m=_xyz(actors_raw["a_size_xyz_m"], "actors.a_size_xyz_m"),
    )
    police_raw = raw["police"]
    police = PoliceSpec(
        body_size_xyz_m=_xyz(police_raw["body_size_xyz_m"], "police.body_size_xyz_m"),
        east_wall_clearance_m=float(police_raw["east_wall_clearance_m"]),
        north_offset_m=float(police_raw["north_offset_m"]),
        minimum_clearance_m=float(police_raw["minimum_clearance_m"]),
    )
    scenario = Scenario(
        schema_version=str(raw["schema_version"]),
        provenance=dict(raw["provenance"]),
        taper_mode=str(geometry["taper_mode"]),
        corridor_length_m=float(geometry["corridor_length_m"]),
        building_height_m=float(geometry["building_height_m"]),
        wall_thickness_m=float(geometry["wall_thickness_m"]),
        west_margin_m=float(geometry["west_margin_m"]),
        corner_screen=CornerScreenSpec(
            north_margin_m=float(geometry["corner_screen"]["north_margin_m"]),
            width_m=float(geometry["corner_screen"]["width_m"]),
        ),
        next_street=next_street,
        profiles=profiles,
        default_profile=default_names[0],
        camera=camera,
        fiducials=fiducials,
        actors=actors,
        police=police,
        speed_policy=dict(raw["speed_policy"]),
    )
    validate_scenario(scenario)
    return scenario


def _validate_speed_policy(scenario: Scenario) -> None:
    """Reject a policy that cannot describe a limit for this corridor."""

    rules = scenario.speed_policy.get("rules")
    if not isinstance(rules, list) or not rules:
        raise ValueError("speed policy must define at least one rule")

    thresholds: list[float] = []
    for index, rule in enumerate(rules):
        try:
            maximum_width = float(rule["maximum_width_m"])
            limit = float(rule["limit_mps"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"speed policy rule {index} needs numeric maximum_width_m and limit_mps"
            ) from error
        # Finiteness is checked before sign, and separately from it. `nan <= 0`
        # and `inf <= 0` are both False, so a sign test alone accepts them here
        # while `normalized_speed_rules` rejects them on the observer side --
        # the scene builds, the manifest is written, and the observer then
        # refuses to construct. YAML spells both directly as `.nan` and `.inf`.
        if not math.isfinite(maximum_width):
            raise ValueError(f"speed policy rule {index} has a non-finite maximum_width_m")
        if not math.isfinite(limit):
            raise ValueError(f"speed policy rule {index} has a non-finite limit_mps")
        if maximum_width <= 0.0:
            raise ValueError(f"speed policy rule {index} has a non-positive maximum_width_m")
        if limit <= 0.0:
            raise ValueError(f"speed policy rule {index} has a non-positive limit_mps")
        thresholds.append(maximum_width)

    duplicates = sorted({value for value in thresholds if thresholds.count(value) > 1})
    if duplicates:
        raise ValueError(f"speed policy repeats maximum_width_m {duplicates}")

    # Every width any authored profile can present must have a rule. The entry
    # width is the widest point of a tapering corridor, so it bounds the rest.
    widest_rule = max(thresholds)
    uncovered = sorted(
        {
            profile.entry_width_m
            for profile in scenario.profiles
            if profile.entry_width_m > widest_rule
        }
    )
    if uncovered:
        raise ValueError(
            f"speed policy does not cover corridor widths {uncovered}; "
            f"the widest rule stops at {widest_rule} m"
        )


def _require_finite(value: float, name: str) -> None:
    """Reject NaN and +/-inf before any sign or range check sees them.

    ``nan <= 0`` and ``inf <= 0`` are both ``False``, so a sign or range test
    alone accepts either where it means to reject a non-positive value, and a
    non-finite dimension then reaches USD authoring or the trajectory solve,
    which fail late with an error that does not name the field responsible
    (A6-L1). YAML spells both directly as ``.nan`` and ``.inf``.
    """

    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value}")


def validate_scenario(scenario: Scenario) -> None:
    """Reject geometry that cannot satisfy the project contract.

    These are the profile-independent checks. Layout checks that depend on a
    selected corridor profile live in ``geometry.validate_layout``.

    Every numeric field is checked finite before anything else about it is
    checked, so a NaN or infinity fails here by name rather than reaching USD
    authoring or the trajectory solve and failing there with a message that
    does not point back at the field responsible.
    """

    _require_finite(scenario.corridor_length_m, "geometry.corridor_length_m")
    _require_finite(scenario.building_height_m, "geometry.building_height_m")
    _require_finite(scenario.wall_thickness_m, "geometry.wall_thickness_m")
    _require_finite(scenario.west_margin_m, "geometry.west_margin_m")

    street = scenario.next_street
    _require_finite(street.clear_width_m, "geometry.next_street.clear_width_m")
    _require_finite(street.length_m, "geometry.next_street.length_m")
    _require_finite(street.turn_radius_m, "geometry.next_street.turn_radius_m")
    _require_finite(street.b_distance_m, "geometry.next_street.b_distance_m")
    _require_finite(street.b_lateral_fraction, "geometry.next_street.b_lateral_fraction")
    stub = street.east_wall_stub
    _require_finite(stub.depth_fraction, "geometry.next_street.east_wall_stub.depth_fraction")
    _require_finite(stub.length_m, "geometry.next_street.east_wall_stub.length_m")
    _require_finite(
        stub.gap_north_of_b_m, "geometry.next_street.east_wall_stub.gap_north_of_b_m"
    )

    _require_finite(scenario.camera.rate_hz, "camera.rate_hz")
    _require_finite(scenario.camera.horizontal_fov_deg, "camera.horizontal_fov_deg")
    _require_finite(scenario.camera.mount_height_m, "camera.mount_height_m")

    _require_finite(scenario.fiducials.marker_size_m, "fiducials.marker_size_m")
    _require_finite(scenario.fiducials.first_station_m, "fiducials.first_station_m")
    _require_finite(scenario.fiducials.spacing_m, "fiducials.spacing_m")
    _require_finite(scenario.fiducials.wall_plate_cant_deg, "fiducials.wall_plate_cant_deg")
    for index, plate in enumerate(scenario.fiducials.references.plates):
        _require_finite(plate.along_m, f"fiducials.references.plates[{index}].along_m")
        _require_finite(plate.height_m, f"fiducials.references.plates[{index}].height_m")
        _require_finite(plate.size_m, f"fiducials.references.plates[{index}].size_m")
        _require_finite(plate.cant_deg, f"fiducials.references.plates[{index}].cant_deg")

    police = scenario.police
    for axis, extent in zip("xyz", police.body_size_xyz_m, strict=True):
        _require_finite(extent, f"police.body_size_xyz_m.{axis}")
    _require_finite(police.east_wall_clearance_m, "police.east_wall_clearance_m")
    _require_finite(police.north_offset_m, "police.north_offset_m")
    _require_finite(police.minimum_clearance_m, "police.minimum_clearance_m")

    for profile in scenario.profiles:
        _require_finite(profile.entry_width_m, f"profile {profile.name}.entry_width_m")
        _require_finite(profile.corner_width_m, f"profile {profile.name}.corner_width_m")

    if scenario.taper_mode != "one_sided_south":
        raise ValueError(
            f"unsupported taper mode {scenario.taper_mode!r}; "
            "the supplied diagram shows a straight north face"
        )

    # The speed policy is the most hand-edited value in this configuration and
    # it used to be the least protected: it travelled from here to the manifest
    # to the observer as an opaque dictionary, so a reordered rule list silently
    # deleted the corner rule and a missing catch-all killed the observer from
    # inside a subscription callback. Reject both before a manifest exists.
    #
    # `police_observer.estimator.normalized_speed_rules` enforces the same
    # invariant on whatever the observer reads, because a manifest can be edited
    # after it is built. corridor_scene cannot import that -- the dependency
    # runs the other way -- so `test_speed_policy_validation_agrees_across_packages`
    # pins the two against the same cases.
    _validate_speed_policy(scenario)
    if scenario.corridor_length_m <= 0.0:
        raise ValueError("corridor length must be positive")
    if scenario.wall_thickness_m <= 0.0:
        raise ValueError("walls must have positive thickness")
    if scenario.west_margin_m < 0.0:
        raise ValueError("west margin must not be negative")

    street = scenario.next_street
    if street.clear_width_m <= 0.0 or street.length_m <= 0.0:
        raise ValueError("the next street must have positive width and length")
    if street.turn_radius_m <= 0.0:
        raise ValueError("turn radius must be positive")
    if not 0.0 < street.b_distance_m < street.length_m:
        raise ValueError("B must stand inside the authored next street")

    # Only the profile-independent reference checks belong here. Anything that
    # depends on where a wall actually is must wait for a resolved profile, so
    # it lives in geometry.validate_layout — load_scenario runs before build
    # appends a dynamically requested (m, n).
    references = scenario.fiducials.references
    if references.id_base < 0:
        raise ValueError("reference id base must not be negative")
    # Reserve a range above every gate id so the two never collide, and keep
    # both inside the ArUco dictionary.
    if references.id_base + len(references.plates) > 100:
        raise ValueError("reference ids must stay inside DICT_5X5_100")
    for index, plate in enumerate(references.plates):
        if plate.surface not in {"north_wall", "east_face"}:
            raise ValueError(f"reference plate {index} has unknown surface {plate.surface!r}")
        if plate.size_m <= 0.0:
            raise ValueError(f"reference plate {index} has a non-positive size")
        if not 0.0 <= plate.cant_deg < 90.0:
            raise ValueError(f"reference plate {index} has an unusable cant {plate.cant_deg}")
        # The plate plus its backing must clear the ground and stay under the
        # building. Heights do not vary with the corridor profile.
        reach = plate.size_m / 2.0 * MARKER_BACKING_SCALE
        if plate.height_m - reach <= 0.0:
            raise ValueError(f"reference plate {index} reaches below ground")
        if plate.height_m + reach >= scenario.building_height_m:
            raise ValueError(f"reference plate {index} reaches above its building")

    police = scenario.police
    if any(extent <= 0.0 for extent in police.body_size_xyz_m):
        raise ValueError("P must have a positive body volume")
    if police.minimum_clearance_m <= 0.0:
        raise ValueError("P needs a positive clearance margin from occluders")
    if police.east_wall_clearance_m < police.minimum_clearance_m:
        raise ValueError("P's east wall clearance is inside its own clearance margin")
    # The north offset is measured from a face P stands *below* rather than
    # behind, so the body must clear that face by its own half-depth as well as
    # the margin, or it would overlap the wall it is measured from.
    if police.north_offset_m < police.minimum_clearance_m + police.body_size_xyz_m[1] / 2.0:
        raise ValueError("P's north offset would put its body inside the north wall")

    # The stub narrows the street A drives down, so it has to leave a lane A
    # fits through. A lane thinner than the robot plus the trajectory margin
    # would fail validate_trajectory later with a message about the *route*,
    # which sends a reader looking in the wrong file.
    #
    # The arc's exact fit also depends on the corridor taper's heading (an
    # authored (m, n) profile, not a scenario-level fact this function has),
    # so no closed form here can be the tight bound -- validate_trajectory's
    # sampled check stays authoritative for that. This is instead a
    # deliberately conservative, profile-independent sufficient condition: a
    # turn cannot be driven through a lane no wider than its own radius. It
    # may reject a handful of configurations that a full route solve would
    # still have accepted; it exists to catch the unambiguous case early and
    # by name, not to replace the numeric check.
    street = scenario.next_street
    stub = street.east_wall_stub
    if not 0.0 < stub.depth_fraction < 1.0:
        raise ValueError("the east-wall stub must occupy part of the street, not none or all")
    if stub.length_m <= 0.0 or stub.gap_north_of_b_m <= 0.0:
        raise ValueError("the east-wall stub needs a positive length and gap north of B")
    lane_width = street.clear_width_m * (1.0 - stub.depth_fraction)
    if lane_width <= street.turn_radius_m:
        raise ValueError(
            f"the east-wall stub leaves a {lane_width:.3f} m lane, too narrow for the "
            f"{street.turn_radius_m} m turn radius"
        )
    if not 0.0 < street.b_lateral_fraction < 1.0:
        raise ValueError("B's lateral fraction must place it inside the street")

    # Occlusion needs buildings taller than both the observing camera and the
    # body being hidden behind them.
    tallest_hidden = max(scenario.camera.mount_height_m, police.body_size_xyz_m[2])
    if scenario.building_height_m <= tallest_hidden:
        raise ValueError("buildings must be taller than the camera and P's body")

    for profile in scenario.profiles:
        if profile.entry_width_m <= 0.0 or profile.corner_width_m <= 0.0:
            raise ValueError(f"profile {profile.name} has a non-positive width")
        if profile.corner_width_m > profile.entry_width_m:
            raise ValueError(f"profile {profile.name} widens instead of tapering")
