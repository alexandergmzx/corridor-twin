"""The corridor Nav2 gate's goal transform and its pinned tolerance.

Two things here can be wrong in a way a live run would not reveal.

The **tolerance** is the one the fleet original got wrong: it prints "tolerance
was 150 mm" and enforces `err < 0.30` two lines later, so a 250 mm miss passes a
gate that says it allows 150. Here both come from one constant, and this file
pins that constant to the value ADR 0022 actually pins.

The **goal transform** is worse, because a wrong goal still produces a confident
SUCCEEDED. SLAM's map frame is anchored at the robot's spawn, so B has to be
expressed relative to that spawn and rotated into its heading -- and the three
profiles spawn A on three different headings. A goal that forgot the rotation
would be quietly wrong on two profiles out of three and right on the third,
which is the worst possible failure signature.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from corridor_nav_gate import (  # noqa: E402
    DELIVERY_STANDOFF_M,
    GOAL_TOLERANCE_M,
    goal_in_map_frame,
)


#: The goal is the delivery STANDOFF, not B's centre -- B is a lidar-visible
#: obstacle, so its centre is unreachable (see test_delivery_standoff.py).
#: These fixtures put the lane centre west of B, so the standoff moves west by
#: DELIVERY_STANDOFF_M and the transform is exercised on that point.
def _manifest(a_start, heading, b_xyz=(10.0, 0.0, 0.0)) -> dict:
    return {
        "actors": {"b_xyz_m": list(b_xyz), "b_size_xyz_m": [0.45, 0.45, 1.7]},
        "next_street": {"center_x_m": 0.0},
        "profiles": {
            "p": {
                "a_start_xyz_m": list(a_start),
                "delivery_trajectory": {"approach_heading": list(heading)},
            }
        },
    }


def test_a_spawn_at_the_origin_facing_x_leaves_the_goal_unchanged() -> None:
    """Identity transform: only the standoff moves the point, not the frame."""

    goal = goal_in_map_frame(_manifest((0.0, 0.0, 0.0), (1.0, 0.0)), "p")

    assert goal == pytest.approx((10.0 - DELIVERY_STANDOFF_M, 0.0))


def test_the_spawn_offset_is_subtracted() -> None:
    goal = goal_in_map_frame(_manifest((4.0, 1.0, 0.0), (1.0, 0.0)), "p")

    assert goal == pytest.approx((6.0 - DELIVERY_STANDOFF_M, -1.0))


def test_the_spawn_heading_rotates_the_goal() -> None:
    """A 90-degree spawn heading puts a goal that is due +x on the map's -y axis."""

    goal = goal_in_map_frame(_manifest((0.0, 0.0, 0.0), (0.0, 1.0)), "p")

    assert goal == pytest.approx((0.0, -(10.0 - DELIVERY_STANDOFF_M)), abs=1e-9)


def test_the_transform_preserves_distance() -> None:
    """A rotation about the spawn cannot change how far B is from A."""

    manifest = _manifest((2.0, -3.0, 0.0), (0.6, 0.8), b_xyz=(9.0, 5.0, 0.0))

    goal = goal_in_map_frame(manifest, "p")

    standoff_x = 9.0 - DELIVERY_STANDOFF_M
    assert math.hypot(*goal) == pytest.approx(
        math.hypot(standoff_x - 2.0, 5.0 - (-3.0))
    )


def test_the_real_corridor_headings_give_materially_different_goals() -> None:
    """The reason this is computed per profile rather than written down once.

    nominal spawns on +7.13 deg and uniform on 0.00. Over the corridor's ~18 m
    delivery that heading difference moves the goal by more than two metres --
    an order of magnitude past the 0.15 m tolerance, so a shared literal would
    fail the gate for a reason that has nothing to do with the robot.
    """

    b = (16.7934, -8.0, 0.0)
    nominal = goal_in_map_frame(
        _manifest((0.0, 0.0, 0.0), (0.9922778767136677, 0.12403473458920847), b), "p"
    )
    uniform = goal_in_map_frame(_manifest((0.0, 0.0, 0.0), (1.0, 0.0), b), "p")

    assert math.dist(nominal, uniform) > 2.0


def test_the_tolerance_is_the_pinned_adr_0022_value() -> None:
    """The fleet original printed 150 mm and enforced 300; one constant, one number."""

    assert GOAL_TOLERANCE_M == 0.15
