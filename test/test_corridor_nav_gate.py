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


def test_route_to_delivery_reaches_b_and_omits_nothing_before_it() -> None:
    """**The docstring was false and the sum was short by a whole leg.**

    It excluded `departure_length_m` on the stated grounds that "the departure
    leg runs PAST B". It does not. The five legs run approach -> corner arc ->
    departure -> delivery arc -> delivery, and the route ENDS at B: measured,
    the station closest to B is the full trajectory length, at a distance of
    0.0000 m. The departure leg is the third of five and lies entirely before
    the delivery.

    So the function under-reported the route by 1.631 m at the committed scale,
    and `min_travel_m` unlocked arming 2.531 m before B rather than the 0.900 m
    its own window implied.
    """

    import json
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).parent.parent / "src" / "corridor_scene"))
    from scene.model import default_config_path, load_scenario
    from scene.trajectory import delivery_trajectory

    manifest = json.loads(
        (Path(__file__).parent.parent / "out" / "corridor.manifest.json").read_text()
    )
    scenario = load_scenario(default_config_path())

    for profile in ("nominal_m6_n3", "wide_corner_m6_n4_5", "uniform_m6_n6"):
        entry = next(p for p in scenario.profiles if p.name == profile)
        authored = delivery_trajectory(scenario, entry)
        # The route the manifest describes must be the route the scenario
        # authors -- the whole of it, because the whole of it precedes B.
        assert nav_gate.route_to_delivery_m(manifest, profile) == pytest.approx(
            authored.length_m, abs=1e-3
        ), f"{profile}: the sum must not drop a leg"


def test_arming_still_unlocks_before_the_earliest_measured_arming() -> None:
    """The regression the correction would otherwise cause, pinned by name.

    Correcting the sum alone moves `min_travel_m` from 4.850 m to 6.480 m. Bag
    `20260813-113859-isaac-d67` armed on the REAL B at **5.699 m** of A's own
    odometry travel, so the corrected sum would have refused a good arming.

    The cause is that A does not drive the authored route: Nav2 plans its own,
    and the measured odometry distance at first arming is 5.699-6.695 m against
    an authored 7.380 m. The window therefore has to absorb that difference,
    which is what re-basing `ARM_WINDOW_ROUTE_FRACTION` does.
    """

    import json
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).parent.parent / "tools"))
    from corridor_dock import DockingMachine

    manifest = json.loads(
        (Path(__file__).parent.parent / "out" / "corridor.manifest.json").read_text()
    )

    #: Measured first-arming travel on the real B, seven bags, 2026-08-13,
    #: after the convexity fix. tools/diagnostics/arming_replay.py.
    EARLIEST_MEASURED_ARMING_M = 5.699
    #: The spawn negative control: a confirmed detection 1.06 m away after
    #: 0.58 m of travel must still be refused (the 2026-08-12 13:16 failure).
    SPAWN_CONTROL_TRAVEL_M = 0.58

    for profile in ("nominal_m6_n3", "wide_corner_m6_n4_5", "uniform_m6_n6"):
        machine = DockingMachine(
            nominal_goal=(0.0, 0.0), standoff_m=0.470,
            route_length_m=nav_gate.route_to_delivery_m(manifest, profile),
        )
        assert machine.min_travel_m < EARLIEST_MEASURED_ARMING_M, (
            f"{profile}: min_travel {machine.min_travel_m:.3f} would have refused "
            f"the arming measured at {EARLIEST_MEASURED_ARMING_M} m"
        )
        assert machine.min_travel_m > SPAWN_CONTROL_TRAVEL_M, (
            f"{profile}: min_travel {machine.min_travel_m:.3f} no longer excludes "
            f"the spawn region"
        )


def test_the_governor_topics_are_absolute_and_match_the_governor() -> None:
    """**A silent topic mismatch looks exactly like a mask that never applied.**

    The safety governor is not namespaced: `safety_launch.py` declares the node
    with no `namespace=`, and the node subscribes to a literal `/scan` and
    `/cmd_vel_raw`. So it lives at `/cmd_vel_governor` whatever robot it governs.

    Building these names from the gate's own namespace was correct for robot1
    only by coincidence -- robot1's namespace is the empty string -- and would
    have published into the void for robot2. Pinned as absolute so the
    coincidence cannot be mistaken for a derivation.
    """

    assert nav_gate.GOVERNOR_CMD_TOPIC == "/cmd_vel_raw"
    assert nav_gate.GOVERNOR_DOCKING_TOPIC == "/cmd_vel_governor/docking_approach"
    for topic in (nav_gate.GOVERNOR_CMD_TOPIC, nav_gate.GOVERNOR_DOCKING_TOPIC):
        assert topic.startswith("/"), "the governor is not namespaced"
        assert "{" not in topic and "robot" not in topic

    # And the creep must never reach the firmware without passing the filter.
    assert nav_gate.GOVERNOR_CMD_TOPIC != "/cmd_vel", (
        "publishing to /cmd_vel bypasses the governor -- the one thing it warns about"
    )


def test_the_states_that_excuse_a_cancel_all_exist() -> None:
    """**The cancel is the handoff now, not the arrival.**

    The gate excuses a CANCELED action status when it cancelled the goal
    itself. That check used to test for `DOCKED` -- a state ADR 0033 removed --
    so after the rename it would have matched nothing and reported every
    delivery as a navigation failure.

    Pinned against the machine's real state names so a future rename breaks
    here rather than in a run artifact.
    """

    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).parent.parent / "tools"))
    from corridor_dock import DockingMachine

    excused = {"DOCKING", "DELIVERED_CONFIRMED", "ARRIVED_UNPROVEN"}
    real = {DockingMachine.DOCKING, DockingMachine.DELIVERED_CONFIRMED,
            DockingMachine.ARRIVED_UNPROVEN}

    assert excused == real, "the excused set must name states that exist"
    assert not hasattr(DockingMachine, "DOCKED"), (
        "DOCKED was removed by ADR 0033; reaching the band is a handoff"
    )


# ------------------------------------------------ the laser stationarity witness
#
# This is the witness that decides whether a bump happened. It has to survive the
# scan matcher's re-registration jumps: bag 20260814-003844 puts the matcher's
# stationary p95 at 374 mm against a median of 16.8 mm, so a statistic sensitive
# to the tail reads a parked robot as moving and no contact is ever confirmed.

EPS = nav_gate.LASER_STATIONARY_EPS_MPS
WINDOW = nav_gate.LASER_WITNESS_WINDOW_S
PAIRS = nav_gate.LASER_WITNESS_MIN_PAIRS


def _track(speeds_mps, *, dt=0.1, t0=100.0):
    """A synthetic /odom_laser track advancing at the given per-step speeds."""

    samples, x, t = [(t0, 0.0, 0.0)], 0.0, t0
    for speed in speeds_mps:
        t += dt
        x += speed * dt
        samples.append((t, x, 0.0))
    return samples, t


def test_a_parked_robot_reads_stationary():
    track, now = _track([0.0168] * 20)          # the bag's stationary median
    assert nav_gate.laser_stationary_from_track(track, now) is True


def test_a_creeping_robot_reads_moving():
    track, now = _track([0.05] * 20)            # the creep clamp
    assert nav_gate.laser_stationary_from_track(track, now) is False


def test_re_registration_jumps_do_not_defeat_the_witness():
    """**The reason this is a median.** One in five samples jumps 374 mm.

    A mean over this track is ~0.76 m/s and a maximum is 3.74 m/s -- both call a
    parked robot moving, and the delivery is never confirmed. The median is
    unmoved.
    """

    speeds = [0.0168] * 20
    for i in range(0, 20, 5):
        speeds[i] = 3.74                        # 374 mm in a 0.1 s step
    track, now = _track(speeds)

    assert nav_gate.laser_stationary_from_track(track, now) is True


def test_a_majority_of_jumps_is_not_explained_away():
    """The median is robust, not blind. Past half, the robot really is moving."""

    speeds = [0.0168] * 20
    for i in range(0, 20, 2):
        speeds[i] = 3.74
    speeds[1] = 3.74
    track, now = _track(speeds)

    assert nav_gate.laser_stationary_from_track(track, now) is False


def test_a_silent_matcher_abstains_rather_than_guessing():
    """None, not False: the caller then falls back to the encoders.

    Returning False ("moving") would be defensible but wrong -- it would mean a
    dead matcher permanently blocks every arrival, and the operator would see a
    robot touching B and reporting ARRIVED_UNPROVEN with nothing in the log.
    """

    assert nav_gate.laser_stationary_from_track([], 100.0) is None
    thin, now = _track([0.0] * (PAIRS - 2))
    assert nav_gate.laser_stationary_from_track(thin, now) is None


def test_stale_samples_fall_out_of_the_window():
    """A matcher that stopped publishing 10 s ago must not still be a witness."""

    track, now = _track([0.0168] * 20)
    assert nav_gate.laser_stationary_from_track(track, now + WINDOW + 5.0) is None


def test_only_the_window_is_considered_not_the_whole_history():
    """Parked for a while, then moving: the verdict follows the recent samples."""

    parked, t = _track([0.0] * 30)
    moving, now = _track([0.05] * 20, t0=t)
    assert nav_gate.laser_stationary_from_track(parked + moving[1:], now) is False


def test_the_threshold_sits_between_the_two_measured_regimes():
    """A pinned number, printed and enforced from one place."""

    assert 0.0168 < EPS < 0.05


# ------------------------------------------- a run that never hands off says so
#
# Run 20260814-031922: Nav2 reported SUCCEEDED at 0.6621 m from B while the
# handoff only fires inside 0.620 m. The machine sat in REFINE with zero creep
# ticks, the dock loop exited, control fell through to reporting, and NOTHING
# said the terminal phase had been skipped. The run's only complaint was an
# unrelated map-frame goal error.
#
# Each case below names the run it was measured on.

from corridor_nav_gate import delivery_reconciliation  # noqa: E402


def test_a_run_that_delivered_is_excused_and_not_faulted() -> None:
    """Runs 023306, 030242, 031348 -- cancelled at the handoff, then crept."""

    for state in ("DOCKING", "DELIVERED_CONFIRMED", "ARRIVED_UNPROVEN"):
        excuse, failure = delivery_reconciliation(
            {"enabled": True, "state": state, "creep_ticks": 3416}, "CANCELED")

        assert excuse is True, f"{state} is reachable only through the handoff"
        assert failure is None


def test_a_transit_abort_is_left_to_the_status_check() -> None:
    """Run 025049: Nav2 ABORTED at 4.792 m, 57 mm short of the arming travel.

    Already reported as an action-status failure; saying it twice in different
    words would send the reader looking for a second fault.
    """

    excuse, failure = delivery_reconciliation(
        {"enabled": True, "state": "TRANSIT", "creep_ticks": 0}, "ABORTED")

    assert excuse is False
    assert failure is None


def test_succeeding_outside_the_handoff_radius_is_a_failure() -> None:
    """**Run 031922, the silent one.** This is the whole commit."""

    excuse, failure = delivery_reconciliation(
        {"enabled": True, "state": "REFINE", "creep_ticks": 0,
         "creep": {"last_seen_range_m": 0.6621,
                   "last_sighting_ceiling_m": 0.39}},
        "SUCCEEDED")

    assert excuse is False
    assert failure is not None, "the terminal phase was skipped and nothing said so"
    assert "handoff never fired" in failure
    assert "0.6621" in failure, "the failure must carry the measured range"
    assert "creep_ticks 0" in failure


def test_a_run_without_docking_is_not_faulted() -> None:
    """The negative control: --no-dock is transit-only by choice."""

    excuse, failure = delivery_reconciliation({"enabled": False}, "SUCCEEDED")

    assert excuse is False
    assert failure is None


def test_a_delivered_run_can_never_trip_the_new_branch() -> None:
    """Guarded on creep_ticks AND state. The three real deliveries recorded
    3416-3496 ticks, so the guard has four orders of magnitude of room."""

    excuse, failure = delivery_reconciliation(
        {"enabled": True, "state": "DELIVERED_CONFIRMED", "creep_ticks": 3496},
        "SUCCEEDED")

    assert failure is None
    assert excuse is True


def test_the_gate_records_whether_the_handoff_fired() -> None:
    """A field, not only prose: the F15 lesson."""

    source = (ROOT / "tools/corridor_nav_gate.py").read_text(encoding="utf-8")

    assert 'dock_report["handoff"]' in source
    assert '"fired"' in source


def test_the_runner_records_a_missed_handoff_in_run_json() -> None:
    """A reported-only profile returns 0 from the gate by design, so the gate's
    own failures list cannot reach the run's classification. Same shape as the
    existing "goal not accepted" reconciliation, and for the same reason."""

    runner = (ROOT / "tools/corridor_profile_run.sh").read_text(encoding="utf-8")

    assert "the docking handoff never fired" in runner
    assert 'manifest_error "the docking handoff never fired' in runner, (
        "a missed handoff must land in run.json, not only in the console"
    )
    # A result, not a rerun: the robot was asked and the robot drove.
    at = runner.index("the docking handoff never fired")
    assert "rerun " not in runner[at - 400:at + 400], (
        "a missed handoff is a result; classifying it a rerun would hide it"
    )
