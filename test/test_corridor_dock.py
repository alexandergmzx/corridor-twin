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


# --- containment: the phantom near spawn -------------------------------------
# Numbers from the run that made this necessary (20260812-131600) and from the
# committed scale: route-to-delivery 5.750 m, window 0.900 m, goal (4.106,
# -2.932), phantom "confirmed" at 1.06 m while the robot stood near (0, 0).

ROUTE_M = 5.7497
WINDOW_M = 0.9
GOAL = (4.1061, -2.9319)


def _contained(**kwargs):
    from corridor_dock import DockingMachine

    return DockingMachine(GOAL, route_length_m=ROUTE_M, **kwargs)


def test_the_window_is_derived_from_the_route_not_copied() -> None:
    """3.0 m was 15.653% of the authored route. Here it is 0.900 m.

    Which is also 3.0 x the 0.30 scale factor, so the derivation checks against
    itself. A literal 3.0 would be more than half of this route.
    """

    machine = _contained()
    assert machine.window_m == pytest.approx(WINDOW_M, abs=0.005)
    assert machine.min_travel_m == pytest.approx(ROUTE_M - WINDOW_M, abs=0.005)
    assert machine.window_m < 3.0 / 2


def test_the_spawn_phantom_is_refused() -> None:
    """**The negative control.** The exact shape of the 13:16 failure.

    A confirmed detection, a plausible laser range, and a robot that has barely
    moved and is nowhere near the goal. Shape and 3-of-5 agreement both pass --
    they are about the object, not the place.
    """

    machine = _contained()
    verdict = _verdict(0.3, 1.0)           # confirmed, 1.06 m away
    assert not machine.armed(verdict, robot_xy=(0.0, 0.0), robot_yaw=0.0, travelled_m=0.58)
    assert machine.step((0.0, 0.0), 0.0, verdict, travelled_m=0.58) is None
    assert "too early in the route" in machine.rejections


def test_the_real_post_at_the_end_of_the_route_still_arms() -> None:
    """The guard must not reject the thing it guards.

    A control that only ever says no is indistinguishable from docking being
    switched off.
    """

    machine = _contained()
    robot = (4.3, -2.5)                     # 0.45 m from the goal, near route end
    bearing = math.atan2(GOAL[1] - robot[1], GOAL[0] - robot[0])
    verdict = _verdict(GOAL[0], GOAL[1])
    verdict["candidate"]["bearing_rad"] = bearing
    verdict["candidate"]["range_m"] = 0.6

    assert machine.armed(verdict, robot_xy=robot, robot_yaw=0.0, travelled_m=5.2)
    assert machine.rejections == {}


def test_each_containment_test_can_refuse_on_its_own() -> None:
    """Three conditions, three separable reasons, so a rejection is diagnosable."""

    robot = (4.3, -2.5)
    bearing = math.atan2(GOAL[1] - robot[1], GOAL[0] - robot[0])

    def at(xy, yaw, travelled, bearing_rad):
        machine = _contained()
        verdict = _verdict(GOAL[0], GOAL[1])
        verdict["candidate"]["range_m"] = 0.6
        verdict["candidate"]["bearing_rad"] = bearing_rad
        machine.armed(verdict, robot_xy=xy, robot_yaw=yaw, travelled_m=travelled)
        return machine.rejections

    assert "too early in the route" in at(robot, 0.0, 1.0, bearing)
    assert "too far from the goal in the map frame" in at((1.0, 0.0), 0.0, 5.2, bearing)
    # Looking backwards: the detection sits 180 deg from the goal direction.
    assert "detection is not where the goal is" in at(robot, 0.0, 5.2, bearing + math.pi)


def test_containment_fails_closed_when_it_is_not_supplied() -> None:
    """Configured but unsupplied is a caller bug, and arming anyway restores the
    exact failure this exists to prevent."""

    machine = _contained()
    verdict = _verdict(GOAL[0], GOAL[1])
    verdict["candidate"]["range_m"] = 0.6

    assert not machine.armed(verdict)
    assert "containment configured but not supplied" in machine.rejections


def test_an_unconfigured_machine_keeps_its_old_behaviour() -> None:
    """Transit-only callers -- and every existing test above -- are unchanged."""

    from corridor_dock import DockingMachine

    machine = DockingMachine(GOAL)
    assert machine.min_travel_m is None
    assert machine.armed(_verdict(0.3, 1.0))
