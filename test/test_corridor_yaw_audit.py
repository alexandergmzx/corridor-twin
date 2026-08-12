"""The yaw audit's arithmetic, checkable without a GPU session.

`total_turned_rad` is the whole instrument: every ratio in the report is a
quotient of two of its outputs. Two ways it could be wrong would both produce a
confidently clean audit -- a net-difference implementation would under-report a
pivot that crosses +/-pi, and an unwrapped sum would charge a 2*pi jump as real
rotation. Either would make a lying odometry look faithful.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from corridor_yaw_audit import EKF_RATIO_TOLERANCE, total_turned_rad, yaw_from  # noqa: E402


class _Quaternion:
    def __init__(self, yaw: float) -> None:
        self.x = self.y = 0.0
        self.z = math.sin(yaw / 2.0)
        self.w = math.cos(yaw / 2.0)


class _Odometry:
    def __init__(self, yaw: float) -> None:
        self.pose = type("P", (), {"pose": type("Q", (), {"orientation": _Quaternion(yaw)})})


@pytest.mark.parametrize("yaw", [0.0, 0.5, -0.5, 2.0, -2.0, 3.0])
def test_yaw_round_trips_through_the_quaternion(yaw: float) -> None:
    assert yaw_from(_Odometry(yaw)) == pytest.approx(yaw, abs=1e-9)


def test_a_quarter_turn_measures_a_quarter_turn() -> None:
    track = [i * math.pi / 200.0 for i in range(51)]

    assert total_turned_rad(track) == pytest.approx(math.pi / 4.0)


def test_rotation_through_pi_is_not_lost() -> None:
    """The failure that would matter: a pivot that wraps must read as 1.0 rad.

    A net (last - first) implementation scores this 5.28 rad -- it reads the
    wrap itself as five radians of rotation that never happened. Whichever
    direction the error runs, every ratio computed from it is meaningless.
    """

    start = math.pi - 0.5
    track = [start + i * 0.1 for i in range(11)]
    wrapped = [(yaw + math.pi) % (2.0 * math.pi) - math.pi for yaw in track]

    assert total_turned_rad(wrapped) == pytest.approx(1.0, abs=1e-9)
    naive = abs(wrapped[-1] - wrapped[0])
    assert naive == pytest.approx(5.2832, abs=1e-3)  # the naive answer, and it is wrong


def test_a_full_revolution_is_a_full_revolution() -> None:
    track = [(i * 2.0 * math.pi / 100.0 + math.pi) % (2.0 * math.pi) - math.pi
             for i in range(101)]

    assert total_turned_rad(track) == pytest.approx(2.0 * math.pi, abs=1e-6)


def test_a_stationary_robot_turns_nothing() -> None:
    assert total_turned_rad([0.7] * 50) == 0.0


def test_reversing_direction_accumulates_rather_than_cancels() -> None:
    """Absolute deltas, deliberately: a pivot out and back travelled both ways.

    Signed accumulation would score an out-and-back pivot zero, and a zero
    denominator makes every ratio in the report undefined exactly when the
    robot moved the most.
    """

    out = [i * 0.1 for i in range(11)]
    back = [1.0 - i * 0.1 for i in range(1, 11)]

    assert total_turned_rad(out + back) == pytest.approx(2.0, abs=1e-9)


def test_the_tolerance_admits_noise_but_not_the_wheel_error() -> None:
    """One number separating "noisy" from "carrying the ~2.8x wheel-yaw error"."""

    assert EKF_RATIO_TOLERANCE < 1.0
    assert abs(2.8 - 1.0) > EKF_RATIO_TOLERANCE
