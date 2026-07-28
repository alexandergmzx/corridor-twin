"""Direct tests for the viewport perspectives.

The adapter can only be exercised on a GPU, so the arithmetic that decides where
the camera goes lives in a module with no Isaac imports and is checked here --
including the quadrants a single live run down one route never visits.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from viewpoints import (  # noqa: E402
    CHASE_VIEW,
    VIEW_NAMES,
    VIEWPOINTS,
    chase_pose,
    format_vec3,
    parse_vec3,
    resolve,
)


@pytest.mark.parametrize("name", sorted(VIEWPOINTS))
def test_every_preset_is_a_usable_camera(name: str) -> None:
    """A preset must be finite, and must look somewhere other than itself."""

    eye, target = VIEWPOINTS[name]
    assert len(eye) == 3 and len(target) == 3
    assert all(math.isfinite(component) for component in (*eye, *target))
    separation = math.dist(eye, target)
    assert separation > 1.0, f"{name} puts the camera on top of its target"
    assert eye[2] > target[2], f"{name} looks up from below the scene"


def test_chase_is_named_but_not_a_static_preset() -> None:
    """Chase has no fixed eye, so it must not sit in the table as if it did."""

    assert CHASE_VIEW in VIEW_NAMES
    assert CHASE_VIEW not in VIEWPOINTS
    assert resolve(CHASE_VIEW) is None


def test_rviz_preset_matches_the_rviz_orbit_configuration() -> None:
    """The preset is derived from the RViz config, so it must still agree.

    An orbit camera sits at ``focal + distance * (cos y cos p, sin y cos p,
    sin p)``. If someone retunes the RViz view without recomputing this, the two
    windows quietly stop showing the same angle -- which is the entire point of
    the preset.
    """

    distance, pitch, yaw = 30.0, 1.1, 4.2
    focal = (8.0, -1.0, 0.0)
    expected = (
        focal[0] + distance * math.cos(yaw) * math.cos(pitch),
        focal[1] + distance * math.sin(yaw) * math.cos(pitch),
        focal[2] + distance * math.sin(pitch),
    )
    eye, target = VIEWPOINTS["rviz"]
    assert target == pytest.approx(focal)
    assert eye == pytest.approx(expected, abs=0.01)


def test_resolve_rejects_an_unknown_view_and_names_the_valid_ones() -> None:
    with pytest.raises(ValueError) as error:
        resolve("overhead")
    message = str(error.value)
    assert "overhead" in message
    for name in VIEW_NAMES:
        assert name in message


def test_explicit_eye_and_target_override_the_preset() -> None:
    """Free exploration must not require editing the table."""

    eye = (9.0, -3.5, 34.0)
    target = (9.0, -3.5, 0.0)
    assert resolve("rviz", eye, target) == (eye, target)
    # Even chase yields to an explicit pair, so a one-off angle always wins.
    assert resolve(CHASE_VIEW, eye, target) == (eye, target)


def test_resolve_requires_eye_and_target_together() -> None:
    with pytest.raises(ValueError):
        resolve("rviz", (1.0, 2.0, 3.0), None)
    with pytest.raises(ValueError):
        resolve("rviz", None, (1.0, 2.0, 3.0))


def test_resolve_rejects_a_camera_pointed_at_itself() -> None:
    point = (1.0, 2.0, 3.0)
    with pytest.raises(ValueError):
        resolve("rviz", point, point)


@pytest.mark.parametrize("text", ["1,2", "1,2,3,4", "", "a,2,3", "1,,3"])
def test_parse_vec3_rejects_malformed_input(text: str) -> None:
    with pytest.raises(ValueError):
        parse_vec3(text)


@pytest.mark.parametrize("text", ["nan,0,0", "inf,0,0", "0,-inf,0", "0,0,NaN"])
def test_parse_vec3_rejects_non_finite_components(text: str) -> None:
    """`nan` compares false against every bound, so it must die at the door.

    An unchecked one reaches the camera transform and produces a black viewport
    with no error -- the same shape of hole `f2e2504` closed in the speed policy.
    """

    with pytest.raises(ValueError):
        parse_vec3(text)


def test_parse_vec3_accepts_signed_and_spaced_values() -> None:
    assert parse_vec3(" 1.5, -2 ,3e0 ") == (1.5, -2.0, 3.0)


@pytest.mark.parametrize("yaw_rad", [0.0, math.pi / 2, math.pi, -math.pi / 2, 2.3])
def test_chase_sits_behind_the_robot_and_looks_ahead(yaw_rad: float) -> None:
    """Checked in every quadrant so a sign error cannot survive in just one."""

    x_m, y_m = 4.0, -1.5
    (eye, target) = chase_pose(x_m, y_m, yaw_rad)
    heading = (math.cos(yaw_rad), math.sin(yaw_rad))

    # The camera trails the robot: the vector from eye to robot points along the
    # heading, so their dot product is positive.
    to_robot = (x_m - eye[0], y_m - eye[1])
    assert to_robot[0] * heading[0] + to_robot[1] * heading[1] > 0.0

    # And it aims past the robot rather than at it.
    to_target = (target[0] - x_m, target[1] - y_m)
    assert to_target[0] * heading[0] + to_target[1] * heading[1] > 0.0

    assert eye[2] > 0.0, "the chase camera is underground"
    assert eye[2] > target[2], "the chase camera looks up rather than down"


def test_chase_keeps_the_configured_distances() -> None:
    eye, target = chase_pose(0.0, 0.0, 0.0, back_m=6.0, height_m=4.0, ahead_m=3.0)
    assert eye == pytest.approx((-6.0, 0.0, 4.0))
    assert target[0] == pytest.approx(3.0)
    assert target[1] == pytest.approx(0.0)


@pytest.mark.parametrize(
    "kwargs",
    [{"back_m": 0.0}, {"height_m": -1.0}, {"ahead_m": 0.0}],
)
def test_chase_rejects_non_positive_distances(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        chase_pose(0.0, 0.0, 0.0, **kwargs)


def test_format_vec3_is_compact_and_parses_back() -> None:
    """The marker line is evidence, so it has to round-trip."""

    vector = (1.33, -12.86, 26.74)
    assert parse_vec3(format_vec3(vector)) == pytest.approx(vector)
