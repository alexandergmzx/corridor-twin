"""The wedge detector has to distinguish "blocked" from "correctly stopped".

Both look like a stationary robot. Only one is a defect, and they need opposite
responses: a robot that stops because Nav2 told it to is behaving, while a robot
that is commanded and does not move is against something -- and on this chassis
that means the wheels are turning anyway and the encoders are reporting a
rotation that never happened, which is the documented mechanism that fans the
map (docs/slam-research/near-wall-stability.md; simctl:641-644).
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from corridor_corner_probe import WEDGE_MIN_S, find_wedge  # noqa: E402


def _row(t, truth_mps, commanded_mps, ratio=None, min_scan=1.0, x=0.0, y=0.0):
    return {
        "t": float(t), "truth_mps": truth_mps, "truth_wz": 0.0,
        "commanded_mps": commanded_mps, "commanded_wz": 0.0, "wheel_wz": 0.0,
        "wheel_truth_ratio": ratio, "min_scan_m": min_scan, "x": x, "y": y,
    }


def test_a_commanded_robot_that_does_not_move_is_wedged() -> None:
    rows = [_row(t, 0.20, 0.22) for t in range(5)]
    rows += [_row(t, 0.001, 0.22, ratio=23.0, min_scan=0.27, x=5.4, y=-4.04)
             for t in range(5, 45)]

    verdict = find_wedge(rows)

    assert verdict["wedged"] is True
    assert verdict["longest_blocked_s"] == 40
    assert verdict["from_t"] == 5.0
    assert verdict["min_clearance_m"] == 0.27
    assert verdict["peak_wheel_truth_ratio"] == 23.0


def test_a_robot_that_was_told_to_stop_is_not_wedged() -> None:
    """The distinction the whole probe exists to make.

    Identical truth speed to the wedged case -- zero. What differs is that
    nothing was asking it to move, which is a robot obeying rather than a robot
    stuck.
    """

    rows = [_row(t, 0.0, 0.0) for t in range(60)]

    assert find_wedge(rows)["wedged"] is False


def test_an_acceleration_transient_is_not_a_wedge() -> None:
    """Two seconds of commanded-but-stopped is a robot starting to move.

    This failed when the threshold was 1.0: a single sample scored as a wedge,
    which would have reported an acceleration transient as the documented
    blocked-wheel defect.
    """

    rows = [_row(0, 0.0, 0.22), _row(1, 0.0, 0.22)]
    rows += [_row(t, 0.20, 0.22) for t in range(2, 30)]

    verdict = find_wedge(rows)

    assert verdict["wedged"] is False
    assert verdict["longest_blocked_s"] < WEDGE_MIN_S


def test_the_longest_stretch_wins_not_the_first() -> None:
    rows = [_row(t, 0.0, 0.22) for t in range(3)]
    rows += [_row(t, 0.2, 0.22) for t in range(3, 10)]
    rows += [_row(t, 0.0, 0.22, min_scan=0.2, x=9.9, y=-4.1) for t in range(10, 40)]

    verdict = find_wedge(rows)

    assert verdict["longest_blocked_s"] == 30
    assert verdict["from_t"] == 10.0
    assert verdict["at_position"] == [9.9, -4.1]


def test_a_slow_crawl_is_moving_not_blocked() -> None:
    """0.05 m/s is the governor crawling, not a robot against a wall."""

    rows = [_row(t, 0.05, 0.22) for t in range(60)]

    assert find_wedge(rows)["wedged"] is False
