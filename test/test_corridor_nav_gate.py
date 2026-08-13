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

import corridor_nav_gate as nav_gate  # noqa: E402
from corridor_nav_gate import (  # noqa: E402
    DELIVERY_STANDOFF_M,
    GOAL_TOLERANCE_M,
    delivery_facing_world,
    goal_in_map_frame,
    goal_yaw_in_map_frame,
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


def test_the_goal_yaw_faces_b_and_is_never_the_identity_by_default() -> None:
    """`orientation.w = 1.0` was an instruction, not a neutral default.

    The map frame is anchored on A's spawn POSE, so map yaw zero means "finish
    on the heading you started on". On a profile that spawns at +7.13 deg of
    world that is simply a wrong instruction, and it was never chosen -- it was
    the quaternion's zero value left in place.
    """

    # Spawned facing +x, B due +x: facing B IS the spawn heading, so zero here
    # is correct rather than accidental.
    assert goal_yaw_in_map_frame(_manifest((0.0, 0.0, 0.0), (1.0, 0.0)), "p") == (
        pytest.approx(0.0)
    )

    # Spawned 90 deg off. The goal yaw must undo it, or A finishes broadside.
    rotated = goal_yaw_in_map_frame(_manifest((0.0, 0.0, 0.0), (0.0, 1.0)), "p")
    assert rotated == pytest.approx(-math.pi / 2)


def test_the_goal_yaw_points_from_the_standoff_at_b() -> None:
    """The two are one derivation: stand off B, then look back at it."""

    manifest = _manifest((0.0, 0.0, 0.0), (1.0, 0.0), b_xyz=(10.0, 4.0, 0.0))
    stand_x, stand_y = nav_gate.delivery_standoff_world(manifest)
    facing = delivery_facing_world(manifest)

    assert facing == pytest.approx(math.atan2(4.0 - stand_y, 10.0 - stand_x))
    # Walking DELIVERY_STANDOFF_M along the facing direction lands on B.
    assert (
        stand_x + DELIVERY_STANDOFF_M * math.cos(facing),
        stand_y + DELIVERY_STANDOFF_M * math.sin(facing),
    ) == pytest.approx((10.0, 4.0))


def test_the_facing_is_not_read_from_the_authored_route() -> None:
    """ADR 0022:15-17 keeps the authored line out of A's navigation.

    The route's final heading numerically AGREES -- the standoff sits on B's
    approach ray, so it must -- and that agreement is the trap: it makes the
    forbidden source look like a valid derivation. Pinned by removing the
    trajectory entirely and requiring the facing to survive.
    """

    manifest = _manifest((0.0, 0.0, 0.0), (1.0, 0.0), b_xyz=(10.0, 4.0, 0.0))
    del manifest["profiles"]["p"]["delivery_trajectory"]

    assert delivery_facing_world(manifest) == pytest.approx(0.0)


def test_the_goal_yaw_is_not_what_closes_the_delivery() -> None:
    """Kept honest on purpose: this fix does not fix the run.

    A arrives mid-turn at -51.4 to -78.6 deg of world. Against the measured
    arrival band, correcting the goal yaw moves the error from 58.5-85.7 deg to
    51.4-78.6 -- both sides of a 34.4 deg tolerance. If someone later reads the
    W1 commit as "the yaw bug is fixed", this test says otherwise in the only
    place that cannot go stale.
    """

    tolerance_deg = 34.4
    arrival_band_world_deg = (-51.4, -78.6)
    # nominal_m6_n3: spawns +7.13 deg of world, B due +x from the standoff.
    nominal = _manifest(
        (0.0, 0.0, 0.0), (0.9922778767136677, 0.12403473458920847), (10.0, 0.0, 0.0)
    )
    corrected_world_deg = math.degrees(
        goal_yaw_in_map_frame(nominal, "p")
    ) + 7.1250163489
    assert corrected_world_deg == pytest.approx(0.0, abs=1e-6)

    for arrival in arrival_band_world_deg:
        assert abs(arrival - corrected_world_deg) > tolerance_deg


def test_the_tolerance_is_the_pinned_adr_0022_value() -> None:
    """The fleet original printed 150 mm and enforced 300; one constant, one number."""

    assert GOAL_TOLERANCE_M == 0.15


def test_every_target_names_the_ekf_topic_the_containment_integrates() -> None:
    """A missing key here killed the nav gate mid-run.

    P2 added EKF-integrated travel to the arming gate and read
    `target["ekf_topic"]` from a table that had never carried it: the goal was
    never sent, the run produced no delivery, and the failure surfaced as a
    KeyError in a log rather than as a gate verdict.

    robot1's EKF publishes /odom at root; robot2's is odometry/filtered inside
    its namespace, so the two cannot share one literal.
    """

    for name, target in nav_gate.ROBOT_TARGETS.items():
        assert "ekf_topic" in target, f"{name} has no ekf_topic"
        topic = target["ekf_topic"]
        resolved = topic if topic.startswith("/") else f"{target['namespace']}/{topic}"
        assert resolved.startswith("/"), f"{name}: {resolved} is not an absolute topic"

    assert nav_gate.ROBOT_TARGETS["robot1"]["ekf_topic"] == "/odom"
    assert nav_gate.ROBOT_TARGETS["robot2"]["ekf_topic"] == "odometry/filtered"
