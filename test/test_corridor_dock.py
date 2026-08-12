"""The docking state machine, and the bounds that keep it from being a loop.

The whole point of docking is that it works while the map is wrong, so the
tests put the robot at map poses that are wrong and check that the goal still
lands beside the landmark. The other half is boundedness: a refiner that can
re-issue forever is a control loop wearing a state machine's clothes.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from corridor_dock import (  # noqa: E402
    DOCK_STANDOFF_M,
    DockingMachine,
    dock_goal,
    landmark_in_map,
)


def _verdict(x, y, confirmed=True):
    return {
        "confirmed": confirmed,
        "candidate": {"x": x, "y": y, "range_m": math.hypot(x, y)},
    }


# --- geometry ----------------------------------------------------------------


def test_a_detection_dead_ahead_lands_in_front_of_the_robot() -> None:
    assert landmark_in_map({"x": 2.0, "y": 0.0}, (5.0, 1.0), 0.0) == pytest.approx((7.0, 1.0))


def test_the_robots_heading_rotates_the_detection() -> None:
    """A detection is laser-relative; forgetting the yaw puts B at 90 degrees."""

    got = landmark_in_map({"x": 2.0, "y": 0.0}, (5.0, 1.0), math.pi / 2.0)

    assert got == pytest.approx((5.0, 3.0), abs=1e-9)


def test_the_goal_stops_short_of_the_landmark_on_the_approach_side() -> None:
    goal = dock_goal((10.0, 0.0), (0.0, 0.0))

    assert goal == pytest.approx((10.0 - DOCK_STANDOFF_M, 0.0))
    assert math.dist(goal, (10.0, 0.0)) == pytest.approx(DOCK_STANDOFF_M)


def test_the_approach_side_comes_from_where_the_robot_IS() -> None:
    """Approaching from the far side must stop on the far side, not overshoot."""

    goal = dock_goal((10.0, 0.0), (20.0, 0.0))

    assert goal == pytest.approx((10.0 + DOCK_STANDOFF_M, 0.0))


def test_already_inside_the_standoff_holds_position() -> None:
    """Never reverse into space the robot has not sensed."""

    assert dock_goal((10.0, 0.0), (9.8, 0.0)) == (9.8, 0.0)


# --- the property that matters: a wrong map does not move the goal -----------


def test_the_goal_tracks_the_landmark_even_when_the_map_pose_is_wrong() -> None:
    """The reason docking exists.

    The same physical situation -- B two metres dead ahead -- is presented at
    three wildly different map poses, as a drifting map would report. The goal
    must land 0.6 m short of B *relative to the robot* every time, because it is
    computed from the current transform and never from the map's history.
    """

    for robot_xy in [(0.0, 0.0), (5.0, -3.0), (-40.0, 12.0)]:
        machine = DockingMachine(nominal_goal=robot_xy)
        goal = machine.step(robot_xy, 0.0, _verdict(2.0, 0.0))

        assert goal is not None
        assert math.dist(goal, robot_xy) == pytest.approx(2.0 - DOCK_STANDOFF_M)


# --- arming ------------------------------------------------------------------


def test_a_far_detection_does_not_arm() -> None:
    """Range-gated, in the LASER frame."""

    machine = DockingMachine(nominal_goal=(0.0, 0.0))

    assert machine.step((0.0, 0.0), 0.0, _verdict(9.0, 0.0)) is None
    assert machine.state == DockingMachine.TRANSIT


def test_arming_does_NOT_depend_on_the_map_pose() -> None:
    """The bug that made docking never fire on a real run.

    Arming used to compare the robot's MAP pose against the MAP goal. On a run
    where A came within 0.49 m of B physically, its drifted map pose never came
    within 3 m of the map goal, so the machine sat in TRANSIT with zero
    refinements while the detector was confirming B the whole time.

    Here the map pose is absurdly far from the nominal goal and docking must
    still arm, because the LASER says B is two metres away.
    """

    machine = DockingMachine(nominal_goal=(500.0, -500.0))

    goal = machine.step((0.0, 0.0), 0.0, _verdict(2.0, 0.0))

    assert goal is not None
    assert machine.refinements == 1


def test_an_unconfirmed_verdict_never_moves_the_goal() -> None:
    machine = DockingMachine(nominal_goal=(0.0, 0.0))

    assert machine.step((0.0, 0.0), 0.0, _verdict(2.0, 0.0, confirmed=False)) is None
    assert machine.refinements == 0


# --- boundedness -------------------------------------------------------------


def test_refinement_is_bounded_and_then_stops() -> None:
    """A refiner that never stops is a control loop, not a state machine."""

    machine = DockingMachine(nominal_goal=(0.0, 0.0), max_refinements=2)
    issued = []
    # Ranges stay inside the arm radius and outside the docked band, and each
    # moves the estimate 0.3 m -- past the 0.20 m re-issue threshold -- so every
    # step would re-issue if nothing bounded it.
    for step in range(6):
        goal = machine.step((0.0, 0.0), 0.0, _verdict(2.9 - step * 0.3, 0.0))
        if goal is not None:
            issued.append(goal)

    assert len(issued) == 2
    assert machine.refinements == 2
    assert machine.state == DockingMachine.UNREFINED


def test_a_landmark_that_has_not_moved_does_not_re_issue() -> None:
    """Re-sending the same goal interrupts an approach that is already working."""

    machine = DockingMachine(nominal_goal=(0.0, 0.0))

    first = machine.step((0.0, 0.0), 0.0, _verdict(3.0, 0.0))
    second = machine.step((0.0, 0.0), 0.0, _verdict(3.02, 0.0))

    assert first is not None
    assert second is None
    assert machine.refinements == 1


def test_arrival_is_decided_by_the_sensor_not_the_map() -> None:
    machine = DockingMachine(nominal_goal=(0.0, 0.0))

    goal = machine.step((0.0, 0.0), 0.0, _verdict(DOCK_STANDOFF_M, 0.0))

    assert goal is None
    assert machine.state == DockingMachine.DOCKED


def test_a_docked_machine_stops_issuing_goals() -> None:
    machine = DockingMachine(nominal_goal=(0.0, 0.0))
    machine.step((0.0, 0.0), 0.0, _verdict(DOCK_STANDOFF_M, 0.0))

    assert machine.step((0.0, 0.0), 0.0, _verdict(5.0, 0.0)) is None
    assert machine.state == DockingMachine.DOCKED


def test_the_report_records_every_refinement() -> None:
    machine = DockingMachine(nominal_goal=(0.0, 0.0))
    machine.step((0.0, 0.0), 0.0, _verdict(3.0, 0.0))
    machine.step((0.0, 0.0), 0.0, _verdict(1.5, 0.0))

    report = machine.report()

    assert report["refinements"] == 2
    assert [entry["event"] for entry in report["history"]] == ["refine", "refine"]
    assert report["landmark_map_frame"] is not None
