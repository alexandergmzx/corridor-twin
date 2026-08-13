"""The yaw-scale gate must compare one interval, not two.

**The A/B this pins is a real one, measured on real bags.** On 2026-08-12 the
gate reported `wide_corner_m6_n4_5` at ratio 1.1081 and failed it against a
1.0 +/- 0.1 bound. Read out of the session bag, that run's EKF rotation over the
whole bag is -75.95 deg -- which is the gate's `estimated_deg` to the
centidegree -- while its TRUTH rotation over the transit is -69.51 deg, which is
close to the gate's `truth_deg` of -68.54. The gate had accumulated its
numerator over one span and its denominator over another, because the two
arrive on independent subscriptions that neither start nor stop together.

Clipped to the span they share, the same samples read 1.0013.

These tests are synthetic and self-contained -- they do not need the bags -- but
the shape they encode is the shape of that failure: a robot that turns, then
keeps turning while only one of the two channels is still being heard.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from corridor_sim_gate import yaw_scale  # noqa: E402


def _track(start_s: float, count: int, rate_hz: float, yaw_rate: float):
    """A track turning at a constant rate, one sample per 1/rate_hz."""

    step = 1.0 / rate_hz
    return [
        (start_s + index * step, 0.0, 0.0, yaw_rate * index * step)
        for index in range(count)
    ]


def test_a_faithful_estimate_over_a_shared_span_reads_one() -> None:
    truth = _track(100.0, 200, 10.0, -0.5)
    estimate = _track(100.0, 200, 10.0, -0.5)

    result = yaw_scale(truth, estimate)

    assert result["available"]
    assert result["ratio"] == pytest.approx(1.0, abs=1e-6)


def test_an_estimate_that_outlives_truth_does_not_inflate_the_ratio() -> None:
    """**The wide_corner defect, in miniature.**

    Both channels are perfectly faithful. Truth simply stops being heard 10 s
    before the estimate does, and the robot keeps turning through those 10 s.
    Summed over their own extents that is a 1.5x "scale error" invented
    entirely by the windows. Clipped, it is 1.0.
    """

    truth = _track(100.0, 200, 10.0, -0.5)        # 20 s
    estimate = _track(100.0, 300, 10.0, -0.5)     # 30 s, same rotation rate

    result = yaw_scale(truth, estimate)

    assert result["ratio"] == pytest.approx(1.0, abs=1e-6)
    assert result["window_s"] == pytest.approx(19.9, abs=0.2)
    assert result["estimate_dropped_outside_window"] == 100
    assert result["truth_dropped_outside_window"] == 0


def test_the_unclipped_sum_is_what_this_replaces() -> None:
    """The negative control: prove the old method really would have failed.

    A test that only shows the new code passing cannot distinguish a fix from a
    tautology. This computes the OLD quantity -- each track summed over its own
    extent -- on the same samples and asserts it is badly wrong.
    """

    truth = _track(100.0, 200, 10.0, -0.5)
    estimate = _track(100.0, 300, 10.0, -0.5)

    def unclipped(track):
        return sum(
            (later[3] - earlier[3] + math.pi) % (2.0 * math.pi) - math.pi
            for earlier, later in zip(track, track[1:], strict=False)
        )

    old_ratio = unclipped(estimate) / unclipped(truth)

    assert old_ratio == pytest.approx(1.4975, abs=0.01)
    assert abs(old_ratio - 1.0) > 0.1, "the old method must fail the 1.0 +/- 0.1 bound"
    assert abs(yaw_scale(truth, estimate)["ratio"] - 1.0) < 0.1


def test_a_real_scale_error_still_reads_as_one() -> None:
    """Clipping must not launder a genuine fault into a pass.

    Same span on both sides, estimate turning 17% fast: the nominal profile's
    actual, unexplained red. It stays red.
    """

    truth = _track(100.0, 200, 10.0, -0.5)
    estimate = _track(100.0, 200, 10.0, -0.5 * 1.17)

    result = yaw_scale(truth, estimate)

    assert result["ratio"] == pytest.approx(1.17, abs=1e-3)
    assert abs(result["ratio"] - 1.0) > 0.1


def test_tracks_that_never_overlap_are_unavailable_not_a_ratio() -> None:
    truth = _track(100.0, 200, 10.0, -0.5)
    estimate = _track(500.0, 200, 10.0, -0.5)

    result = yaw_scale(truth, estimate)

    assert not result["available"]
    assert "never overlapped" in result["reason"]
