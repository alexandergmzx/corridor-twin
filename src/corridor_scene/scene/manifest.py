"""Scenario-manifest serialization shared by every runtime consumer.

The manifest is the only scenario input the police observer is permitted to
read, and it is also what the visibility certificate is computed from, so it
carries the surveyed markers, the derived actor volumes, the occluding slabs,
and the delivery trajectory for each corridor profile.
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from .geometry import (
    a_start_xyz,
    all_surveys,
    building_footprints,
    occluders,
    person_b_xyz,
    police_bounds,
)
from .model import CorridorProfile, Scenario
from .trajectory import delivery_trajectory


def manifest_data(
    scenario: Scenario,
    profiles: tuple[CorridorProfile, ...],
    selected_profile: str,
    stage_path: Path,
) -> dict[str, Any]:
    """Build a JSON-compatible description of the generated scene."""

    profile_data: dict[str, Any] = {}
    for profile in profiles:
        police_min, police_max = police_bounds(scenario, profile)
        profile_data[profile.name] = {
            "entry_width_m": profile.entry_width_m,
            "corner_width_m": profile.corner_width_m,
            "a_start_xyz_m": a_start_xyz(scenario, profile),
            "police_bounds_min_xyz_m": police_min,
            "police_bounds_max_xyz_m": police_max,
            # Every opaque wall, named, with its footprint. This is the scene.
            # `occluders` below is a different thing -- the analytic proof's
            # slab list -- and consumers that need to know what the street
            # contains were reading that and seeing only the subset the proof
            # references. See ADR 0018.
            "walls": {
                name: [[float(x), float(y)] for x, y in footprint]
                for name, footprint in building_footprints(scenario, profile).items()
            },
            "occluders": [asdict(slab) for slab in occluders(scenario, profile)],
            "delivery_trajectory": asdict(delivery_trajectory(scenario, profile)),
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
                    "role": marker.role,
                }
                for marker in all_surveys(scenario, profile)
            ],
        }
    return {
        "schema_version": scenario.schema_version,
        "provenance": scenario.provenance,
        "stage": stage_path.name,
        "selected_profile": selected_profile,
        "taper_mode": scenario.taper_mode,
        "corridor_length_m": scenario.corridor_length_m,
        "building_height_m": scenario.building_height_m,
        "wall_thickness_m": scenario.wall_thickness_m,
        "west_margin_m": scenario.west_margin_m,
        "next_street": {
            "clear_width_m": scenario.next_street.clear_width_m,
            "length_m": scenario.next_street.length_m,
            "turn_radius_m": scenario.next_street.turn_radius_m,
            "b_distance_m": scenario.next_street.b_distance_m,
            "west_x_m": scenario.street_west_m,
            "east_x_m": scenario.street_east_m,
            "center_x_m": scenario.street_center_x_m,
            "south_y_m": scenario.street_south_m,
        },
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
            "b_xyz_m": person_b_xyz(scenario),
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
