"""Scenario-manifest serialization shared by every runtime consumer."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .geometry import marker_surveys
from .model import CorridorProfile, Scenario


def manifest_data(
    scenario: Scenario,
    profiles: tuple[CorridorProfile, ...],
    selected_profile: str,
    stage_path: Path,
) -> dict[str, Any]:
    """Build a JSON-compatible description of the generated scene."""

    profile_data: dict[str, Any] = {}
    for profile in profiles:
        profile_data[profile.name] = {
            "entry_width_m": profile.entry_width_m,
            "corner_width_m": profile.corner_width_m,
            "markers": [
                {
                    "id": marker.marker_id,
                    "station_m": marker.station_m,
                    "side": marker.side,
                    "corners_xyz_m": marker.corners_xyz_m,
                    # OpenCV detector order: top-left, top-right,
                    # bottom-right, bottom-left in marker-image space.
                    "aruco_corner_order_xyz_m": tuple(
                        marker.corners_xyz_m[index] for index in (3, 2, 1, 0)
                    ),
                    "normal_xyz": marker.normal_xyz,
                }
                for marker in marker_surveys(scenario, profile)
            ],
        }
    return {
        "schema_version": scenario.schema_version,
        "stage": stage_path.name,
        "selected_profile": selected_profile,
        "corridor_length_m": scenario.corridor_length_m,
        "building_height_m": scenario.building_height_m,
        "wall_thickness_m": scenario.wall_thickness_m,
        "profiles": profile_data,
        "camera": {
            "frame_id": scenario.camera.frame_id,
            "width_px": scenario.camera.width_px,
            "height_px": scenario.camera.height_px,
            "rate_hz": scenario.camera.rate_hz,
            "horizontal_fov_deg": scenario.camera.horizontal_fov_deg,
            "mount_height_m": scenario.camera.mount_height_m,
        },
        "actors": {
            "a_start_xyz_m": scenario.a_start_xyz_m,
            "b_xyz_m": scenario.b_xyz_m,
            "p_bounds_min_xyz_m": scenario.p_bounds_min_xyz_m,
            "p_bounds_max_xyz_m": scenario.p_bounds_max_xyz_m,
            "delivery_path_xyz_m": scenario.delivery_path_xyz_m,
        },
        "fiducials": {
            "dictionary": scenario.fiducials.dictionary,
            "marker_size_m": scenario.fiducials.marker_size_m,
        },
        "speed_policy": scenario.speed_policy,
    }


def write_manifest(path: Path, data: dict[str, Any]) -> None:
    """Write stable, readable JSON with a trailing newline."""

    path.write_text(
        json.dumps(data, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
