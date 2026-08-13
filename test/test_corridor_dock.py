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
    DockingMachine,
    dock_goal,
    final_approach_m,
    landmark_in_map,
)

#: The committed scenario's derived final approach: B's radius 0.12 and A's
#: length 0.195 through `final_approach_m`. Computed, never written down, so a
#: rescale that moves it moves these tests with it.
STANDOFF_M = final_approach_m(0.12, 0.195)


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


def test_the_final_approach_is_the_governors_floor_not_a_chosen_number() -> None:
    """ADR 0031's contact semantics, and the ordering they must not assume.

    At the committed scenario the governor's hard stop decides: 0.35 m of laser
    range plus B's 0.12 m radius is 0.470 m, against a geometric-contact term of
    0.0975 + 0.12 + 0.15 = 0.3675 m. **The governor is never bypassed** -- the
    demo win is declared at a distance the safety envelope permits.

    The second case is what the `max` is FOR, and it is worth stating exactly
    because the obvious guess is wrong: a bigger B does not flip the ordering.
    B's radius enters both terms identically, so it cancels. What flips it is a
    longer ROBOT -- contact overtakes the governor once A's half-length exceeds
    0.20 m, which is `stop_distance - tolerance`.
    """

    assert final_approach_m(0.12, 0.195) == pytest.approx(0.470)
    assert final_approach_m(0.12, 0.195) > 0.195 / 2 + 0.12 + 0.15

    # A B of half a metre radius: the governor still decides, because the
    # radius is in both terms.
    assert final_approach_m(0.5, 0.195) == pytest.approx(0.35 + 0.5)

    # A robot a metre long: geometric contact takes over, and the derivation
    # follows it rather than reporting a distance that would clip B.
    assert final_approach_m(0.12, 1.0) == pytest.approx(0.5 + 0.12 + 0.15)


def test_the_goal_stops_short_of_the_landmark_on_the_approach_side() -> None:
    goal = dock_goal((10.0, 0.0), (0.0, 0.0), STANDOFF_M)

    assert goal == pytest.approx((10.0 - STANDOFF_M, 0.0))
    assert math.dist(goal, (10.0, 0.0)) == pytest.approx(STANDOFF_M)


def test_the_approach_side_comes_from_where_the_robot_IS() -> None:
    """Approaching from the far side must stop on the far side, not overshoot."""

    goal = dock_goal((10.0, 0.0), (20.0, 0.0), STANDOFF_M)

    assert goal == pytest.approx((10.0 + STANDOFF_M, 0.0))


def test_already_inside_the_standoff_holds_position() -> None:
    """Never reverse into space the robot has not sensed."""

    assert dock_goal((10.0, 0.0), (9.8, 0.0), STANDOFF_M) == (9.8, 0.0)


# --- the property that matters: a wrong map does not move the goal -----------


def test_the_goal_tracks_the_landmark_even_when_the_map_pose_is_wrong() -> None:
    """The reason docking exists.

    The same physical situation -- B two metres dead ahead -- is presented at
    three wildly different map poses, as a drifting map would report. The goal
    must land 0.6 m short of B *relative to the robot* every time, because it is
    computed from the current transform and never from the map's history.
    """

    for robot_xy in [(0.0, 0.0), (5.0, -3.0), (-40.0, 12.0)]:
        machine = DockingMachine(nominal_goal=robot_xy, standoff_m=STANDOFF_M)
        goal = machine.step(robot_xy, 0.0, _verdict(2.0, 0.0))

        assert goal is not None
        assert math.dist(goal, robot_xy) == pytest.approx(2.0 - STANDOFF_M)


# --- arming ------------------------------------------------------------------


def test_a_far_detection_does_not_arm() -> None:
    """Range-gated, in the LASER frame."""

    machine = DockingMachine(nominal_goal=(0.0, 0.0), standoff_m=STANDOFF_M)

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

    machine = DockingMachine(nominal_goal=(500.0, -500.0), standoff_m=STANDOFF_M)

    goal = machine.step((0.0, 0.0), 0.0, _verdict(2.0, 0.0))

    assert goal is not None
    assert machine.refinements == 1


def test_an_unconfirmed_verdict_never_moves_the_goal() -> None:
    machine = DockingMachine(nominal_goal=(0.0, 0.0), standoff_m=STANDOFF_M)

    assert machine.step((0.0, 0.0), 0.0, _verdict(2.0, 0.0, confirmed=False)) is None
    assert machine.refinements == 0


# --- boundedness -------------------------------------------------------------


def test_refinement_is_bounded_and_then_stops() -> None:
    """A refiner that never stops is a control loop, not a state machine."""

    machine = DockingMachine(nominal_goal=(0.0, 0.0), standoff_m=STANDOFF_M, max_refinements=2)
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

    machine = DockingMachine(nominal_goal=(0.0, 0.0), standoff_m=STANDOFF_M)

    first = machine.step((0.0, 0.0), 0.0, _verdict(3.0, 0.0))
    second = machine.step((0.0, 0.0), 0.0, _verdict(3.02, 0.0))

    assert first is not None
    assert second is None
    assert machine.refinements == 1


def test_arrival_is_decided_by_the_sensor_not_the_map() -> None:
    machine = DockingMachine(nominal_goal=(0.0, 0.0), standoff_m=STANDOFF_M)

    goal = machine.step((0.0, 0.0), 0.0, _verdict(STANDOFF_M, 0.0))

    assert goal is None
    assert machine.state == DockingMachine.DOCKED


def test_a_docked_machine_stops_issuing_goals() -> None:
    machine = DockingMachine(nominal_goal=(0.0, 0.0), standoff_m=STANDOFF_M)
    machine.step((0.0, 0.0), 0.0, _verdict(STANDOFF_M, 0.0))

    assert machine.step((0.0, 0.0), 0.0, _verdict(5.0, 0.0)) is None
    assert machine.state == DockingMachine.DOCKED


def test_the_report_records_every_refinement() -> None:
    machine = DockingMachine(nominal_goal=(0.0, 0.0), standoff_m=STANDOFF_M)
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

    return DockingMachine(GOAL, standoff_m=STANDOFF_M, route_length_m=ROUTE_M, **kwargs)


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

    machine = DockingMachine(GOAL, standoff_m=STANDOFF_M)
    assert machine.min_travel_m is None
    assert machine.armed(_verdict(0.3, 1.0))


def test_the_bearing_gate_admits_the_REAL_B_from_the_real_manifest() -> None:
    """The guard-must-not-reject-the-guarded test, done properly, re-derived.

    `test_the_real_post_at_the_end_of_the_route_still_arms` above sets the
    detection's bearing TO the goal bearing, so it passes by construction and
    proves nothing about the scene.

    **ADR 0031 moved this geometry and the cone is not carried across it.**
    Before the merge the detectable post stood 0.8 m south of B while the
    delivery standoff sat 0.6 m west of it, putting the two **1.000 m apart** --
    comparable to the 0.9 m window itself, and the reason +/-60 deg was chosen.
    Now the detection is B, so the separation is the standoff alone: **0.600 m**.
    The cone is re-measured against that rather than inherited from it.
    """

    import json

    manifest_path = Path(__file__).parent.parent / "out/corridor.manifest.json"
    if not manifest_path.is_file():
        pytest.skip("out/corridor.manifest.json is a generated artifact")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    sys.path.insert(0, str(Path(__file__).parent.parent / "tools"))
    from corridor_dock import MAX_BEARING_ERROR_DEG
    from corridor_nav_gate import DELIVERY_STANDOFF_M, delivery_standoff_world

    goal = delivery_standoff_world(manifest)
    b_position = manifest["actors"]["b_xyz_m"]
    # One object: what the detector confirms is what the goal stands off from.
    assert math.dist(goal[:2], b_position[:2]) == pytest.approx(
        DELIVERY_STANDOFF_M, abs=0.05
    )

    worst = 0.0
    for standoff in (2.0, 1.5, 1.0, 0.7, 0.5, 0.3):
        # Approaching the goal down the street, which is how A arrives.
        robot = (goal[0], goal[1] + standoff)
        to_goal = math.atan2(goal[1] - robot[1], goal[0] - robot[0])
        to_b = math.atan2(b_position[1] - robot[1], b_position[0] - robot[0])
        error = abs(math.degrees((to_b - to_goal + math.pi) % (2 * math.pi) - math.pi))
        worst = max(worst, error)
        assert error <= MAX_BEARING_ERROR_DEG, (
            f"at {standoff} m from the goal B is {error:.1f} deg off the goal "
            f"bearing, and containment would refuse it"
        )
    # The merge does not make this easier in the way it first looks. B is nearer
    # the goal than the post was, but it is nearer along the SAME side A
    # approaches from, so the bearing swings harder at close range rather than
    # less. Recorded so a scenario change that erodes the margin is visible
    # rather than silent.
    assert worst < MAX_BEARING_ERROR_DEG, f"margin has eroded to {worst:.1f} deg"
