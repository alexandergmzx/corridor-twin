"""The speed policy is the demonstration's most hand-edited value.

It used to be its least protected one. `speed_policy` travelled from YAML
through the manifest into `MarkerMap` as an opaque dictionary, and `limit_at`
returns the first rule whose threshold covers the width -- correct only on a
sorted list. Nothing sorted it and nothing checked it, so reversing the rules,
a semantically identical set, silently made every gate 1.5 m/s and deleted the
corner rule from the demonstration. A missing catch-all was worse: it built
cleanly and then raised from inside a subscription callback, killing the
observer partway through a run.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from police_observer.estimator import MarkerMap, normalized_speed_rules
from scene.build import build_scene
from scene.model import authored_config_path

# The limits ADR 0016 fixes for the shipped configuration.
EXPECTED_LIMITS = [(2.0, 1.5), (4.0, 1.2), (6.0, 1.2), (8.0, 0.8), (10.0, 0.8)]


@pytest.fixture(scope="module")
def manifest(tmp_path_factory: pytest.TempPathFactory) -> Path:
    output = tmp_path_factory.mktemp("speed_policy") / "corridor.usda"
    # THE AUTHORED SCENE, explicitly. These tests describe the v1 camera
    # enforcement program -- surveyed ArUco stations, the shipped speed policy's
    # width thresholds -- and all of it is stated in authored metres. The
    # default config is now the 0.30-scale scenario the robot drives, where
    # (6.0, 3.0) is not a profile at all and resolve_profiles would append it
    # as a 6 m entry inside a 3.6 m corridor.
    _, manifest_path = build_scene(authored_config_path(), output, 6.0, 3.0)
    return manifest_path


def _with_rules(manifest: Path, tmp_path: Path, rules: list[dict[str, float]]) -> Path:
    """Write a copy of the manifest carrying a different rule list."""

    raw = json.loads(manifest.read_text(encoding="utf-8"))
    raw["speed_policy"]["rules"] = rules
    edited = tmp_path / "edited.manifest.json"
    edited.write_text(json.dumps(raw), encoding="utf-8")
    return edited


def test_shipped_policy_produces_the_documented_limits(manifest: Path) -> None:
    marker_map = MarkerMap.from_manifest(manifest, "nominal_m6_n3")
    limits = [(station, marker_map.limit_at(station)) for station in marker_map.gate_stations_m]
    assert limits == EXPECTED_LIMITS


def test_rule_order_cannot_change_behaviour(manifest: Path, tmp_path: Path) -> None:
    """The reproduction that motivated this: reversing the rules deleted the corner rule.

    Asserting equality with the shipped limits is the point. A test that only
    checked "no exception raised" would have passed while every gate silently
    read 1.5 m/s.
    """

    raw = json.loads(manifest.read_text(encoding="utf-8"))
    reversed_rules = list(reversed(raw["speed_policy"]["rules"]))
    assert [rule["maximum_width_m"] for rule in reversed_rules] == [1000.0, 5.0, 4.0]

    marker_map = MarkerMap.from_manifest(_with_rules(manifest, tmp_path, reversed_rules))
    limits = [(station, marker_map.limit_at(station)) for station in marker_map.gate_stations_m]
    assert limits == EXPECTED_LIMITS


def test_a_policy_with_no_catch_all_fails_before_any_frame(manifest: Path, tmp_path: Path) -> None:
    """`limit_at` runs inside a subscription callback; it must not be where this surfaces."""

    raw = json.loads(manifest.read_text(encoding="utf-8"))
    truncated = [rule for rule in raw["speed_policy"]["rules"] if rule["maximum_width_m"] < 100.0]
    edited = _with_rules(manifest, tmp_path, truncated)

    with pytest.raises(ValueError, match="does not cover corridor widths"):
        MarkerMap.from_manifest(edited, "uniform_m6_n6")


@pytest.mark.parametrize(
    ("rules", "message"),
    [
        ([], "at least one rule"),
        ([{"maximum_width_m": 4.0, "limit_mps": 0.8}] * 2, "repeats maximum_width_m"),
        ([{"maximum_width_m": 4.0, "limit_mps": 0.0}], "non-positive limit_mps"),
        ([{"maximum_width_m": -1.0, "limit_mps": 0.8}], "non-positive maximum_width_m"),
        ([{"maximum_width_m": 4.0}], "numeric maximum_width_m and limit_mps"),
    ],
    ids=["empty", "duplicate", "zero-limit", "negative-width", "missing-field"],
)
def test_unusable_rule_sets_are_rejected(rules: list, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        normalized_speed_rules(rules)


def test_normalization_is_idempotent_and_ascending() -> None:
    scrambled = [
        {"maximum_width_m": 1000.0, "limit_mps": 1.5},
        {"maximum_width_m": 4.0, "limit_mps": 0.8},
        {"maximum_width_m": 5.0, "limit_mps": 1.2},
    ]
    once = normalized_speed_rules(scrambled)
    assert once == ((4.0, 0.8), (5.0, 1.2), (1000.0, 1.5))
    assert once == normalized_speed_rules(
        [{"maximum_width_m": width, "limit_mps": limit} for width, limit in once]
    )


def test_a_policy_boundary_is_not_decided_by_the_sixteenth_decimal() -> None:
    """**ADR 0016's two-gate floor had been void at robot scale since 0030.**

    The nominal profile's gate at station 2.4 sits at the corridor's midpoint
    on a linear taper, so its clear width is exactly 1.2 m by construction --
    the strict rule's threshold. Evaluated in floats it is 1.2000000000000002,
    and a bare `<=` put it in the permissive zone. The strict zone held ONE
    gate; `consecutive_estimates` is 2; a corner-confined violation could
    therefore never be confirmed, and the demonstration's central claim would
    have produced zero events with nothing to point at.

    Scale-dependent, which is why no test caught it: v1's own arithmetic,
    `6.0 + (8.0 / 12.0) * (3.0 - 6.0)`, is exactly 4.0.
    """

    from police_observer.estimator import covered_by

    robot_scale_width = 1.8 + (2.4 / 3.6) * (0.9 - 1.8)
    assert robot_scale_width != 1.2, "the float error is gone; this test is moot"
    assert covered_by(robot_scale_width, 1.2), (
        "gate 2.4 is outside the strict zone again, by 2.2e-16 m"
    )

    v1_width = 6.0 + (8.0 / 12.0) * (3.0 - 6.0)
    assert v1_width == 4.0
    assert covered_by(v1_width, 4.0), "v1's exact boundary stopped holding"


def test_the_boundary_tolerance_is_far_below_anything_physical() -> None:
    """A tolerance wide enough to move a real decision would be a moved
    threshold wearing a disguise. One nanometre is 8 orders of magnitude below
    the narrowest width this scenario distinguishes (0.15 m between tiers)."""

    from police_observer.estimator import POLICY_WIDTH_EPSILON_M, covered_by

    assert POLICY_WIDTH_EPSILON_M < 1e-6
    assert not covered_by(1.2 + 1e-6, 1.2), (
        "a micrometre over the threshold is now inside the zone"
    )
