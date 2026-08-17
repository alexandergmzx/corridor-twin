"""The corridor gate's arithmetic, tested away from ROS and the GPU.

ADR 0027 selects robot A on the numbers this gate produces, so those numbers
have to be checkable by someone who is not standing in a live simulation. The
node stays untested here -- it is subscriptions and a publisher -- but every
figure the report carries comes from a module-level function, and those are
pinned below.

The cases chosen are the ones that would silently produce a plausible-looking
pass: a drift computed against an empty track, a covariance trace read at the
wrong station, and a withheld-update count that misreads a healthy stream as
degenerate.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

import corridor_sim_gate  # noqa: E402
from corridor_sim_gate import (  # noqa: E402
    covariance_at_midpoint,
    max_consecutive_withheld,
    midpoint_drift,
    path_length_m,
    stream_rate,
)


def _track(points: list[tuple[float, float]], start_s: float = 0.0) -> list:
    return [(start_s + index, x, y) for index, (x, y) in enumerate(points)]


#: The run that exposed the defect, straight out of its committed artifact:
#: out/evidence/robot-a-gate/gate-robot1-nominal_m6_n3.json, 2026-08-12 13:16.
#: A 551.0 s window was REQUESTED; 256.11 s was observed, because the recorder
#: stops when the transit does. Both streams were healthy and both were failed.
TRUNCATED_RUN_REQUESTED_S = 551.0
TRUNCATED_RUN_OBSERVED_S = 256.11
TRUNCATED_RUN_ODOM_LASER_MSGS = 2946
TRUNCATED_RUN_EKF_MSGS = 2595


def test_a_rate_needs_two_stamps_to_have_a_span() -> None:
    """One message is ABSENT, not slow. Dividing it by anything invents a rate."""

    assert stream_rate(0, None, None) == {"msgs": 0, "span_s": None, "hz": None}
    assert stream_rate(1, 10.0, 10.0) == {"msgs": 1, "span_s": None, "hz": None}


def test_a_rate_divides_by_the_span_between_first_and_last_stamp() -> None:
    """Ten stamps one second apart bound NINE intervals, so the rate is 1.0 Hz."""

    measured = stream_rate(10, 100.0, 109.0)
    assert measured["span_s"] == pytest.approx(9.0)
    assert measured["hz"] == pytest.approx(1.0)


def test_a_truncated_run_reports_the_rate_it_actually_observed() -> None:
    """The 2026-08-12 13:16 regression, in the numbers that produced it.

    2946 matcher messages over the 256.11 s observed is 11.50 Hz against a
    12.0 Hz declared scan rate -- healthy. Divided by the 551.0 s REQUESTED it
    read 5.35 Hz and the gate failed it as "too slow or absent".
    """

    first = 1_786_561_800.0
    matcher = stream_rate(
        TRUNCATED_RUN_ODOM_LASER_MSGS, first, first + TRUNCATED_RUN_OBSERVED_S
    )
    assert matcher["hz"] == pytest.approx(11.50, abs=0.01)

    ekf = stream_rate(TRUNCATED_RUN_EKF_MSGS, first, first + TRUNCATED_RUN_OBSERVED_S)
    assert ekf["hz"] == pytest.approx(10.13, abs=0.01)

    # The requested basis is what the old code used. Kept as the NEGATIVE
    # CONTROL: if this ever stops differing from the observed reading, the
    # fixture has stopped exercising the defect.
    requested_basis_hz = TRUNCATED_RUN_ODOM_LASER_MSGS / TRUNCATED_RUN_REQUESTED_S
    assert requested_basis_hz == pytest.approx(5.35, abs=0.01)
    assert matcher["hz"] > requested_basis_hz * 2.0


def test_both_rate_floors_pass_the_truncated_run_on_the_observed_basis() -> None:
    """The floors themselves, not just the arithmetic.

    robot1 declares 12.0 Hz scan, so the matcher floor is 6.0 Hz; the EKF floor
    is its own configured 10.0 Hz. Both streams of the 13:16 run clear them --
    which is the point: two of that run's three gate failures were the
    instrument, and only the drift row was real.
    """

    scan_hz = corridor_sim_gate.ROBOT_TARGETS["robot1"]["scan_hz"]
    matcher_floor = scan_hz * corridor_sim_gate.MIN_ODOM_LASER_HZ_FRACTION
    first = 1_786_561_800.0

    matcher = stream_rate(
        TRUNCATED_RUN_ODOM_LASER_MSGS, first, first + TRUNCATED_RUN_OBSERVED_S
    )
    ekf = stream_rate(TRUNCATED_RUN_EKF_MSGS, first, first + TRUNCATED_RUN_OBSERVED_S)

    assert matcher["hz"] >= matcher_floor
    assert ekf["hz"] >= corridor_sim_gate.MIN_EKF_HZ

    # And the control: on the requested basis both floors reject them.
    assert matcher_floor > TRUNCATED_RUN_ODOM_LASER_MSGS / TRUNCATED_RUN_REQUESTED_S
    assert corridor_sim_gate.MIN_EKF_HZ > TRUNCATED_RUN_EKF_MSGS / TRUNCATED_RUN_REQUESTED_S


def test_the_ekf_rate_floor_leaves_room_for_the_filter_to_be_healthy() -> None:
    """A floor at nominal is arithmetic, not a gate.

    Set to 10.0 first, it failed three consecutive runs at 9.98-9.99 Hz: a
    filter running exactly at its configured rate measures a hair under it. 9.0
    still catches one missing a tenth of its updates.
    """

    assert corridor_sim_gate.MIN_EKF_HZ == 9.0
    assert corridor_sim_gate.MIN_EKF_HZ < 10.0, "the EKF's configured frequency"
    assert corridor_sim_gate.MIN_ODOM_LASER_HZ_FRACTION == 0.5


def test_path_length_sums_planar_segments() -> None:
    assert path_length_m(_track([(0.0, 0.0), (3.0, 4.0), (3.0, 8.0)])) == pytest.approx(9.0)


def test_path_length_of_a_stationary_track_is_zero() -> None:
    """A robot that never moved must not accumulate distance from jitter-free truth."""

    assert path_length_m(_track([(1.0, 1.0)] * 5)) == pytest.approx(0.0)


def test_midpoint_drift_is_zero_when_the_estimate_tracks_truth() -> None:
    """A perfect estimate must report zero drift, whatever the sampling.

    This failed when written. The function compared the estimate truncated at
    the midpoint TIME against half the total DISTANCE, and truth crosses the
    halfway mark partway through a sample interval -- so a perfectly tracking
    estimate was charged a whole sample's travel as drift. Both tracks are now
    truncated at the same instant.
    """

    truth = _track([(0.0, 0.0), (2.0, 0.0), (4.0, 0.0), (6.0, 0.0)])
    estimate = _track([(0.0, 0.0), (2.0, 0.0), (4.0, 0.0), (6.0, 0.0)])

    result = midpoint_drift(truth, estimate)

    assert result["available"]
    assert result["longitudinal_drift_m"] == pytest.approx(0.0)
    assert result["drift_fraction"] == pytest.approx(0.0)


def test_coarse_truth_sampling_does_not_manufacture_drift() -> None:
    """The regression guard for the bug above, on a deliberately coarse track."""

    truth = _track([(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)])
    estimate = _track([(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)])

    assert midpoint_drift(truth, estimate)["longitudinal_drift_m"] == pytest.approx(0.0)


def test_midpoint_drift_detects_a_short_estimate() -> None:
    """Under-reading distance is the corridor degeneracy failure mode.

    Scan matching along a featureless corridor tends to UNDER-estimate travel,
    because successive scans look alike and the matcher prefers no motion.
    """

    truth = _track([(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)])
    estimate = _track([(0.0, 0.0), (4.0, 0.0), (8.0, 0.0)])

    result = midpoint_drift(truth, estimate)

    assert result["available"]
    assert result["estimated_distance_m"] < result["truth_distance_m"]


def test_midpoint_drift_refuses_rather_than_guesses_without_tracks() -> None:
    """An unavailable measurement must not read as a drift of zero."""

    assert midpoint_drift([], []) == {
        "available": False,
        "reason": "no truth or estimate track",
    }
    assert not midpoint_drift(_track([(0.0, 0.0), (1.0, 0.0)]), [])["available"]


def test_covariance_is_read_at_the_station_nearest_the_midpoint() -> None:
    """The study's headline number is the midpoint sample, so it must be the right one."""

    trace = [(0.0, 1.0, 1.0, 1.0), (5.0, 9.0, 9.0, 9.0), (10.0, 2.0, 2.0, 2.0)]

    result = covariance_at_midpoint(trace, total_distance_m=10.0)

    assert result["station_m"] == pytest.approx(5.0)
    assert result["trace"] == pytest.approx(27.0)


def test_covariance_at_midpoint_is_none_without_a_trace() -> None:
    assert covariance_at_midpoint([], total_distance_m=10.0) is None


def test_a_healthy_stream_reports_no_withheld_updates() -> None:
    """One scan period per gap is the matcher publishing normally."""

    assert max_consecutive_withheld([0.1, 0.1, 0.1], scan_period_s=0.1) == 0


def test_a_withheld_run_is_counted_in_scan_periods() -> None:
    """A 0.6 s gap at 10 Hz is five skipped updates, which is the ADR 0022 limit."""

    assert max_consecutive_withheld([0.1, 0.6, 0.1], scan_period_s=0.1) == 5


def test_the_worst_gap_is_what_counts_not_the_last() -> None:
    assert max_consecutive_withheld([0.9, 0.1, 0.2], scan_period_s=0.1) == 8


def test_an_empty_gap_list_is_not_a_failure() -> None:
    """A run with a single update has no gaps; that is not eight withheld ones."""

    assert max_consecutive_withheld([], scan_period_s=0.1) == 0


def test_a_nonsense_scan_period_is_refused() -> None:
    with pytest.raises(ValueError, match="scan period must be positive"):
        max_consecutive_withheld([0.1], scan_period_s=0.0)


def test_the_pinned_thresholds_match_adr_0022() -> None:
    """These are the numbers ADR 0027 will be decided against."""

    from corridor_sim_gate import MAX_CONSECUTIVE_WITHHELD, MAX_MIDPOINT_DRIFT_FRACTION

    assert MAX_CONSECUTIVE_WITHHELD == 5
    assert MAX_MIDPOINT_DRIFT_FRACTION == 0.05


# --- per-robot targeting (X4) ------------------------------------------------
# The two robots differ in namespace, frames, odometry source AND in which
# criterion is load-bearing. Getting any of these wrong fails silently: a gate
# pointed at the wrong namespace simply records a robot that never moved.


def test_the_two_robots_target_different_wiring() -> None:
    from corridor_sim_gate import ROBOT_TARGETS

    robot2 = ROBOT_TARGETS["robot2"]
    robot1 = ROBOT_TARGETS["robot1"]

    # robot1 runs at ROOT (architecture.md:46-51), robot2 under /robot2.
    assert robot2["namespace"] == "/robot2"
    assert robot1["namespace"] == ""
    # robot1's EKF output is /odom, not odometry/filtered
    # (bringup_corrected_launch.py:82).
    assert robot1["ekf_topic"] == "/odom"
    assert robot2["ekf_topic"] == "odometry/filtered"
    # Nav2 does not namespace frames, so these are literal per robot.
    assert robot1["base_frame"] == "base_footprint"
    assert robot2["base_frame"] == "robot2/base_footprint"


def test_withholding_is_gated_only_where_the_matcher_is_the_odometry() -> None:
    """The criterion swap that ADR 0027's contrast rests on.

    robot2 has no wheel encoders (fleet D-05), so the matcher IS its odometry
    and withholding starves localization. robot1's EKF fuses encoders and IMU
    and does not consume the matcher at all (ekf_sim_pnfix.yaml:138-146), so
    the same measurement is study data rather than a gate.
    """

    from corridor_sim_gate import ROBOT_TARGETS

    assert ROBOT_TARGETS["robot2"]["gate_withholding"] is True
    assert ROBOT_TARGETS["robot1"]["gate_withholding"] is False


def test_the_ekf_continuity_limit_keeps_blind_travel_under_the_goal_tolerance() -> None:
    """ADR 0022's derivation, reapplied to the topic Nav2 actually consumes.

    The bound is not a preference: at robot1's 0.35 m/s governor cap, the
    permitted gap times that cap must stay under the 0.15 m goal tolerance, or
    the robot can overshoot its goal while blind.
    """

    from corridor_nav_gate import GOAL_TOLERANCE_M
    from corridor_sim_gate import MAX_EKF_GAP_S

    governor_cap_mps = 0.35
    assert MAX_EKF_GAP_S * governor_cap_mps < GOAL_TOLERANCE_M
    # And it must be loose enough to pass a healthy 10 Hz EKF: 0.4 s is four
    # missed updates, not one jittery period.
    assert MAX_EKF_GAP_S >= 4 * (1.0 / 10.0)


def test_the_scan_rate_default_follows_the_robot() -> None:
    """robot2's matcher runs ~10 Hz, robot1's scan is 12 Hz declared."""

    from corridor_sim_gate import ROBOT_TARGETS

    assert ROBOT_TARGETS["robot2"]["scan_hz"] == 10.0
    assert ROBOT_TARGETS["robot1"]["scan_hz"] == 12.0


# --- yaw scale ---------------------------------------------------------------
# Added after the gate PASSED a transit whose heading ended 138 deg wrong: its
# longitudinal drift was 4.9% (green) while the estimate believed it had turned
# 365 deg against truth's 227. Distance and heading are independent failure
# modes, and a map is destroyed by the second one first.


def _yaw_track(yaws, start=0.0):
    return [(float(i), start, 0.0, yaw) for i, yaw in enumerate(yaws)]


def test_a_faithful_estimate_scores_one() -> None:
    yaws = [i * 0.1 for i in range(120)]

    result = corridor_sim_gate.yaw_scale(_yaw_track(yaws), _yaw_track(yaws))

    assert result["available"] is True
    assert result["ratio"] == pytest.approx(1.0)


def test_the_measured_transit_error_is_caught() -> None:
    """The real numbers from 20260811-233949: 365.4 deg believed, 227.5 true."""

    truth = _yaw_track([math.radians(i * 227.45 / 100.0) for i in range(101)])
    estimate = _yaw_track([math.radians(i * 365.37 / 100.0) for i in range(101)])

    result = corridor_sim_gate.yaw_scale(truth, estimate)

    assert result["ratio"] == pytest.approx(1.606, abs=1e-3)
    assert abs(result["ratio"] - 1.0) > corridor_sim_gate.MAX_YAW_SCALE_ERROR


def test_the_pivot_sweep_result_still_passes() -> None:
    """+/-4% was measured on a healthy chain; the criterion must not fail it."""

    for ratio in (0.9577, 1.0408):
        truth = _yaw_track([math.radians(i * 2.0) for i in range(101)])
        estimate = _yaw_track([math.radians(i * 2.0 * ratio) for i in range(101)])

        result = corridor_sim_gate.yaw_scale(truth, estimate)

        assert abs(result["ratio"] - 1.0) <= corridor_sim_gate.MAX_YAW_SCALE_ERROR


def test_a_run_that_barely_turned_is_unavailable_not_a_pass() -> None:
    """A ratio of two small numbers is noise, and noise must not read green."""

    truth = _yaw_track([math.radians(i * 0.1) for i in range(101)])
    estimate = _yaw_track([math.radians(i * 0.3) for i in range(101)])

    result = corridor_sim_gate.yaw_scale(truth, estimate)

    assert result["available"] is False


def test_an_inverted_yaw_channel_is_caught() -> None:
    yaws = [math.radians(i * 2.0) for i in range(101)]

    result = corridor_sim_gate.yaw_scale(_yaw_track(yaws), _yaw_track([-yaw for yaw in yaws]))

    assert result["ratio"] == pytest.approx(-1.0)
    assert abs(result["ratio"] - 1.0) > corridor_sim_gate.MAX_YAW_SCALE_ERROR


# --- world-frame delivery ----------------------------------------------------
# Added after a run reported 6-7 m of map-frame goal error while the robot had
# physically come within 0.768 m of the standoff and then driven back to its
# spawn. The map-frame number is computed in a frame SLAM owns; when SLAM
# diverges it stops describing the robot at all.


def _truth(points):
    return [(float(i), x, y, 0.0) for i, (x, y) in enumerate(points)]


def test_the_closest_approach_is_not_the_final_position() -> None:
    """The measured signature: arrived, then left. Both numbers are needed."""

    track = _truth([(0.022, 0.003), (5.687, -3.303), (2.640, 0.924), (-1.053, 1.028)])

    result = corridor_sim_gate.world_frame_delivery(track, (6.453, -3.360))

    assert result["closest_approach_m"] == pytest.approx(0.768, abs=1e-3)
    assert result["final_error_m"] > 8.0
    assert result["walked_away_m"] > 7.0


def test_a_robot_that_stays_has_not_walked_away() -> None:
    track = _truth([(0.0, 0.0), (3.0, -2.0), (6.4, -3.35), (6.453, -3.360)])

    result = corridor_sim_gate.world_frame_delivery(track, (6.453, -3.360))

    assert result["walked_away_m"] == pytest.approx(0.0, abs=1e-3)
    assert result["final_error_m"] < 0.01


def test_a_robot_that_never_arrives_reports_its_best() -> None:
    track = _truth([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])

    result = corridor_sim_gate.world_frame_delivery(track, (6.453, -3.360))

    assert result["closest_approach_m"] > 4.0
    assert result["walked_away_m"] == pytest.approx(0.0, abs=1e-6)


def test_no_truth_track_is_unavailable_not_zero() -> None:
    assert corridor_sim_gate.world_frame_delivery([], (0.0, 0.0))["available"] is False
