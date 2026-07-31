from dataclasses import replace
from pathlib import Path

import pytest
from scene.build import resolve_profiles
from scene.model import load_scenario, validate_scenario

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_scene_package_has_no_isaac_imports() -> None:
    forbidden = ("import omni", "from omni", "import isaacsim", "from isaacsim")
    violations = []
    for path in (PACKAGE_ROOT / "scene").rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        if any(token in text for token in forbidden):
            violations.append(path.name)
    assert violations == []


def test_scenario_records_metric_z_up_stage() -> None:
    scenario = (PACKAGE_ROOT / "config/corridor.yaml").read_text(encoding="utf-8")
    assert "up_axis: Z" in scenario
    assert "meters_per_unit: 1.0" in scenario


# A6-L1: `nan <= 0` and `inf <= 0` are both False, so a sign or range check
# alone accepts a non-finite dimension where it means to reject a non-positive
# one, and the value then reaches USD authoring or the trajectory solve and
# fails there with a message that does not name the field responsible. These
# cover one field at each nesting level `validate_scenario` walks, not every
# field: the same `_require_finite` guard protects all of them identically, so
# one representative per level is what proves the boundary catches it, not an
# exhaustive sweep of a mechanical, uniformly-applied check.
@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_a_non_finite_top_level_dimension_is_rejected_by_name(value: float) -> None:
    scenario = load_scenario()
    candidate = replace(scenario, corridor_length_m=value)
    with pytest.raises(ValueError, match=r"geometry\.corridor_length_m must be finite"):
        validate_scenario(candidate)


def test_a_non_finite_next_street_field_is_rejected_by_name() -> None:
    scenario = load_scenario()
    candidate = replace(
        scenario, next_street=replace(scenario.next_street, turn_radius_m=float("nan"))
    )
    with pytest.raises(
        ValueError, match=r"geometry\.next_street\.turn_radius_m must be finite"
    ):
        validate_scenario(candidate)


def test_a_non_finite_police_field_is_rejected_by_name() -> None:
    scenario = load_scenario()
    candidate = replace(scenario, police=replace(scenario.police, north_offset_m=float("inf")))
    with pytest.raises(ValueError, match=r"police\.north_offset_m must be finite"):
        validate_scenario(candidate)


def test_a_non_finite_camera_field_is_rejected_by_name() -> None:
    scenario = load_scenario()
    candidate = replace(scenario, camera=replace(scenario.camera, rate_hz=float("nan")))
    with pytest.raises(ValueError, match=r"camera\.rate_hz must be finite"):
        validate_scenario(candidate)


def test_a_non_finite_profile_width_is_rejected_by_name() -> None:
    scenario = load_scenario()
    profiles = tuple(
        replace(profile, entry_width_m=float("nan")) if index == 0 else profile
        for index, profile in enumerate(scenario.profiles)
    )
    candidate = replace(scenario, profiles=profiles)
    with pytest.raises(ValueError, match=r"entry_width_m must be finite"):
        validate_scenario(candidate)


def test_an_impossibly_narrow_lane_is_rejected_with_a_specific_message() -> None:
    """The east-wall stub must leave room for A's turn, checked at load time.

    A stub deep enough to leave less lane than the turn radius needs would
    otherwise fail inside `validate_trajectory`, with a message about the
    *route* rather than the stub depth that actually caused it. This widens
    only `depth_fraction`, leaving the scenario's own default turn radius
    (2.0 m) untouched, to prove the check is reachable at a realistic radius
    and not only once the radius is also inflated to make room for it.
    """

    scenario = load_scenario()
    candidate = replace(
        scenario,
        next_street=replace(
            scenario.next_street,
            east_wall_stub=replace(scenario.next_street.east_wall_stub, depth_fraction=0.75),
        ),
    )
    assert candidate.next_street.turn_radius_m == scenario.next_street.turn_radius_m
    with pytest.raises(ValueError, match="leaves a .* m lane, too narrow"):
        validate_scenario(candidate)


def test_resolve_profiles_rejects_non_finite_command_line_dimensions() -> None:
    """--m/--n reach resolve_profiles before a Scenario exists to validate them."""

    scenario = load_scenario()
    with pytest.raises(ValueError, match=r"--m must be finite"):
        resolve_profiles(scenario.profiles, float("nan"), 3.0)
    with pytest.raises(ValueError, match=r"--n must be finite"):
        resolve_profiles(scenario.profiles, 6.0, float("inf"))
