"""Typed scenario configuration used by every generated artifact."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


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
class FiducialSpec:
    """Surveyed marker layout parameters."""

    dictionary: str
    marker_size_m: float
    first_station_m: float
    spacing_m: float
    wall_plate_cant_deg: float


@dataclass(frozen=True)
class Scenario:
    """Validated, simulator-independent scenario model."""

    schema_version: str
    corridor_length_m: float
    building_height_m: float
    wall_thickness_m: float
    cross_street_width_m: float
    profiles: tuple[CorridorProfile, ...]
    default_profile: str
    camera: CameraSpec
    fiducials: FiducialSpec
    a_start_xyz_m: tuple[float, float, float]
    b_xyz_m: tuple[float, float, float]
    p_bounds_min_xyz_m: tuple[float, float, float]
    p_bounds_max_xyz_m: tuple[float, float, float]
    delivery_path_xyz_m: tuple[tuple[float, float, float], ...]
    speed_policy: dict[str, Any]

    def profile(self, name: str) -> CorridorProfile:
        """Return a profile by name or fail with a useful error."""

        for profile in self.profiles:
            if profile.name == name:
                return profile
        available = ", ".join(profile.name for profile in self.profiles)
        raise ValueError(f"unknown corridor profile {name!r}; available: {available}")


def default_config_path() -> Path:
    """Return the source-tree configuration path."""

    return Path(__file__).resolve().parents[1] / "config" / "corridor.yaml"


def _xyz(value: Any, name: str) -> tuple[float, float, float]:
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
    fiducials = FiducialSpec(
        dictionary=str(fiducial_raw["dictionary"]),
        marker_size_m=float(fiducial_raw["marker_size_m"]),
        first_station_m=float(fiducial_raw["first_station_m"]),
        spacing_m=float(fiducial_raw["spacing_m"]),
        wall_plate_cant_deg=float(fiducial_raw["wall_plate_cant_deg"]),
    )
    actors = raw["actors"]
    delivery_path = tuple(
        _xyz(point, f"delivery_path_xyz_m[{index}]")
        for index, point in enumerate(actors["delivery_path_xyz_m"])
    )
    scenario = Scenario(
        schema_version=str(raw["schema_version"]),
        corridor_length_m=float(geometry["corridor_length_m"]),
        building_height_m=float(geometry["building_height_m"]),
        wall_thickness_m=float(geometry["wall_thickness_m"]),
        cross_street_width_m=float(geometry["cross_street_width_m"]),
        profiles=profiles,
        default_profile=default_names[0],
        camera=camera,
        fiducials=fiducials,
        a_start_xyz_m=_xyz(actors["a_start_xyz_m"], "a_start_xyz_m"),
        b_xyz_m=_xyz(actors["b_xyz_m"], "b_xyz_m"),
        p_bounds_min_xyz_m=_xyz(actors["p_bounds_min_xyz_m"], "p_bounds_min_xyz_m"),
        p_bounds_max_xyz_m=_xyz(actors["p_bounds_max_xyz_m"], "p_bounds_max_xyz_m"),
        delivery_path_xyz_m=delivery_path,
        speed_policy=dict(raw["speed_policy"]),
    )
    validate_scenario(scenario)
    return scenario


def validate_scenario(scenario: Scenario) -> None:
    """Reject geometry that cannot satisfy the project contract."""

    if scenario.corridor_length_m <= 0.0:
        raise ValueError("corridor length must be positive")
    if scenario.building_height_m <= scenario.camera.mount_height_m:
        raise ValueError("buildings must be taller than the camera")
    if len(scenario.delivery_path_xyz_m) < 2:
        raise ValueError("delivery path needs at least two points")
    for profile in scenario.profiles:
        if profile.entry_width_m <= 0.0 or profile.corner_width_m <= 0.0:
            raise ValueError(f"profile {profile.name} has a non-positive width")
        if profile.corner_width_m > profile.entry_width_m:
            raise ValueError(f"profile {profile.name} widens instead of tapering")
    if any(
        minimum >= maximum
        for minimum, maximum in zip(
            scenario.p_bounds_min_xyz_m,
            scenario.p_bounds_max_xyz_m,
            strict=True,
        )
    ):
        raise ValueError("P bounds must have positive extent")
