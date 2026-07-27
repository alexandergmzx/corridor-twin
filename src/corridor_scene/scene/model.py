"""Typed scenario configuration used by every generated artifact."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

Vec3 = tuple[float, float, float]


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
class NextStreetSpec:
    """The perpendicular street A turns onto to reach B."""

    clear_width_m: float
    length_m: float
    turn_radius_m: float
    b_distance_m: float


@dataclass(frozen=True)
class PoliceSpec:
    """P's body and its standoff from the occluding corner walls.

    Offsets are measured from wall faces rather than stored as absolute
    coordinates so that P follows the geometry when a different corridor
    profile is selected.
    """

    body_size_xyz_m: Vec3
    west_offset_m: float
    south_offset_m: float
    minimum_clearance_m: float


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
    next_street: NextStreetSpec
    profiles: tuple[CorridorProfile, ...]
    default_profile: str
    camera: CameraSpec
    fiducials: FiducialSpec
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


def default_config_path() -> Path:
    """Return the source-tree configuration path."""

    return Path(__file__).resolve().parents[1] / "config" / "corridor.yaml"


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
    next_street = NextStreetSpec(
        clear_width_m=float(street_raw["clear_width_m"]),
        length_m=float(street_raw["length_m"]),
        turn_radius_m=float(street_raw["turn_radius_m"]),
        b_distance_m=float(street_raw["b_distance_m"]),
    )
    police_raw = raw["police"]
    police = PoliceSpec(
        body_size_xyz_m=_xyz(police_raw["body_size_xyz_m"], "police.body_size_xyz_m"),
        west_offset_m=float(police_raw["west_offset_m"]),
        south_offset_m=float(police_raw["south_offset_m"]),
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
        next_street=next_street,
        profiles=profiles,
        default_profile=default_names[0],
        camera=camera,
        fiducials=fiducials,
        police=police,
        speed_policy=dict(raw["speed_policy"]),
    )
    validate_scenario(scenario)
    return scenario


def validate_scenario(scenario: Scenario) -> None:
    """Reject geometry that cannot satisfy the project contract.

    These are the profile-independent checks. Layout checks that depend on a
    selected corridor profile live in ``geometry.validate_layout``.
    """

    if scenario.taper_mode != "one_sided_south":
        raise ValueError(
            f"unsupported taper mode {scenario.taper_mode!r}; "
            "the supplied diagram shows a straight north face"
        )
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

    police = scenario.police
    if any(extent <= 0.0 for extent in police.body_size_xyz_m):
        raise ValueError("P must have a positive body volume")
    if police.minimum_clearance_m <= 0.0:
        raise ValueError("P needs a positive clearance margin from occluders")
    if police.west_offset_m < police.minimum_clearance_m:
        raise ValueError("P's west offset is inside its own clearance margin")
    if police.south_offset_m < police.minimum_clearance_m:
        raise ValueError("P's south offset is inside its own clearance margin")

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
