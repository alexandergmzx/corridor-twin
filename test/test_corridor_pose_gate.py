"""The composed robot's pose gate, tested where it can fail cheaply.

The forward-sign gate in `build_corridor_arena.py` drives the robot and requires
displacement along its heading. On its own that gate is weaker than it looks:
the first live composition placed the robot at **yaw 0** because
`XformCommonAPI` silently refused the referenced prim's op stack, and the gate
passed anyway, because the corridor's approach heading is within 8 degrees of
+x and the robot still moved forward. A gate that passes with the robot facing
the wrong way is not measuring what it claims to.

So the placement is asserted against the profile's expected spawn -- position
AND yaw -- before the drive gate is allowed to mean anything. That comparison is
pure arithmetic, so it is tested here rather than only inside a GPU session,
including the exact failure that occurred.

Angle comparison is the subtle part. Subtracting two yaws and taking the
magnitude is wrong across the +/-180 degree seam: +179.9 and -179.9 are 0.2
degrees apart, not 359.8. The corridor's own headings are all small, so this
would never have bitten here -- which is precisely why it is worth pinning
before some later profile or a reversed route makes it live.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from build_corridor_arena import placement_error, placement_is_correct  # noqa: E402

SPAWN = (1.5, -2.0, 0.055)
YAW = math.radians(7.13)


def _rotation_for(yaw_rad: float) -> list[list[float]]:
    """The 3x3 rows a USD ExtractRotationMatrix yields for a Z rotation."""

    cos, sin = math.cos(yaw_rad), math.sin(yaw_rad)
    return [[cos, sin, 0.0], [-sin, cos, 0.0], [0.0, 0.0, 1.0]]


def test_an_exact_placement_reports_no_error() -> None:
    position_m, yaw_deg = placement_error(SPAWN, YAW, SPAWN, _rotation_for(YAW))

    assert position_m == pytest.approx(0.0, abs=1e-9)
    assert yaw_deg == pytest.approx(0.0, abs=1e-9)
    assert placement_is_correct(SPAWN, YAW, SPAWN, _rotation_for(YAW))


def test_a_silently_dropped_yaw_is_rejected() -> None:
    """The failure that actually happened, and that the drive gate did not catch.

    Position lands correctly and only the rotation is missing, which is exactly
    what an ignored `SetRotate` produces.
    """

    position_m, yaw_deg = placement_error(SPAWN, YAW, SPAWN, _rotation_for(0.0))

    assert position_m == pytest.approx(0.0, abs=1e-9)
    assert yaw_deg == pytest.approx(7.13, abs=1e-6)
    assert not placement_is_correct(SPAWN, YAW, SPAWN, _rotation_for(0.0))


def test_a_displaced_spawn_is_rejected() -> None:
    moved = (SPAWN[0] + 0.25, SPAWN[1], SPAWN[2])

    position_m, _yaw_deg = placement_error(SPAWN, YAW, moved, _rotation_for(YAW))

    assert position_m == pytest.approx(0.25, abs=1e-9)
    assert not placement_is_correct(SPAWN, YAW, moved, _rotation_for(YAW))


def test_yaw_error_is_measured_across_the_180_degree_seam() -> None:
    """+179.9 and -179.9 are 0.2 degrees apart, not 359.8.

    Note what is and is not claimed. 0.2 degrees is still far outside the
    authoring tolerance, so this placement is correctly REJECTED; what the seam
    handling buys is that the reported error is 0.2 rather than 359.8, so the
    number in the failure message is usable.
    """

    expected = math.radians(179.9)
    observed = _rotation_for(math.radians(-179.9))

    _position_m, yaw_deg = placement_error(SPAWN, expected, SPAWN, observed)

    assert yaw_deg == pytest.approx(0.2, abs=1e-6)
    assert not placement_is_correct(SPAWN, expected, SPAWN, observed)


def test_a_placement_that_only_straddles_the_seam_is_accepted() -> None:
    """The other half of the seam: a genuinely tiny error spanning +/-180.

    Without wrapping this reads as ~360 degrees of error and a correct
    composition would be rejected outright.
    """

    expected = math.radians(180.0)
    observed = _rotation_for(math.radians(-179.99995))

    _position_m, yaw_deg = placement_error(SPAWN, expected, SPAWN, observed)

    assert yaw_deg < 1e-3
    assert placement_is_correct(SPAWN, expected, SPAWN, observed)


def test_the_tolerances_are_tight_enough_to_catch_a_wrong_profile() -> None:
    """The three profiles' headings differ by degrees, so degrees must fail.

    nominal is +7.13 deg and wide_corner is +3.58 deg. Composing one profile's
    arena while placing the robot on another's heading is a plausible mistake,
    and the gate has to reject it.
    """

    wrong_profile_yaw = math.radians(3.58)

    assert not placement_is_correct(SPAWN, YAW, SPAWN, _rotation_for(wrong_profile_yaw))
