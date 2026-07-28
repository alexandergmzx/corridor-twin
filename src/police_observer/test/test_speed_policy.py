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

# The limits ADR 0016 fixes for the shipped configuration.
EXPECTED_LIMITS = [(2.0, 1.5), (4.0, 1.2), (6.0, 1.2), (8.0, 0.8), (10.0, 0.8)]


@pytest.fixture(scope="module")
def manifest(tmp_path_factory: pytest.TempPathFactory) -> Path:
    output = tmp_path_factory.mktemp("speed_policy") / "corridor.usda"
    _, manifest_path = build_scene(None, output, 6.0, 3.0)
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
