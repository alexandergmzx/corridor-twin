"""Command-line entry point for deterministic corridor scene generation."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

from .geometry import marker_surveys
from .manifest import manifest_data, write_manifest
from .marker_assets import generate_marker_images
from .model import CorridorProfile, load_scenario
from .usd_authoring import author_stage


def _slug(value: float) -> str:
    return re.sub(r"[^0-9a-z]+", "_", f"{value:g}".lower()).strip("_")


def resolve_profiles(
    configured: tuple[CorridorProfile, ...], m: float, n: float
) -> tuple[tuple[CorridorProfile, ...], str]:
    """Select a matching profile or append the requested finite profile."""

    if m <= 0.0 or n <= 0.0:
        raise ValueError("m and n must be positive")
    if n > m:
        raise ValueError("n must be less than or equal to m for a tapered corridor")
    for profile in configured:
        if abs(profile.entry_width_m - m) < 1e-9 and abs(profile.corner_width_m - n) < 1e-9:
            return configured, profile.name
    name = f"requested_m{_slug(m)}_n{_slug(n)}"
    return configured + (CorridorProfile(name, m, n),), name


def build_scene(config: Path | None, output: Path, m: float, n: float) -> tuple[Path, Path]:
    """Build the stage, marker assets, and sidecar manifest."""

    if output.suffix.lower() != ".usda":
        raise ValueError("--out must use the human-readable .usda extension")
    scenario = load_scenario(config)
    profiles, selected = resolve_profiles(scenario.profiles, m, n)
    output.parent.mkdir(parents=True, exist_ok=True)
    marker_ids = {
        marker.marker_id for profile in profiles for marker in marker_surveys(scenario, profile)
    }
    generate_marker_images(
        output.parent / "markers",
        scenario.fiducials.dictionary,
        marker_ids,
    )
    author_stage(output, scenario, profiles, selected)
    manifest_path = output.with_suffix(".manifest.json")
    write_manifest(
        manifest_path,
        manifest_data(scenario, profiles, selected, output),
    )
    return output, manifest_path


def parser() -> argparse.ArgumentParser:
    command = argparse.ArgumentParser(description=__doc__)
    command.add_argument("--m", type=float, required=True, help="entry clear width in metres")
    command.add_argument("--n", type=float, required=True, help="corner clear width in metres")
    command.add_argument("--out", type=Path, required=True, help="output .usda path")
    command.add_argument("--config", type=Path, help="scenario YAML override")
    return command


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    stage_path, manifest_path = build_scene(args.config, args.out, args.m, args.n)
    print(f"wrote {stage_path}")
    print(f"wrote {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
