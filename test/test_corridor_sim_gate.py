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

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from corridor_sim_gate import (  # noqa: E402
    covariance_at_midpoint,
    max_consecutive_withheld,
    midpoint_drift,
    path_length_m,
)


def _track(points: list[tuple[float, float]], start_s: float = 0.0) -> list:
    return [(start_s + index, x, y) for index, (x, y) in enumerate(points)]


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
