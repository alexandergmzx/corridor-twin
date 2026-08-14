"""`RateWindow`: message rates over a trailing window, not since process start.

This exists because of ADR 0035. Once the lens comes up BEFORE the simulator,
cumulative `count / (now - t0)` stops being honest: ~130 s of legitimate
pre-Isaac silence turns a healthy 14 Hz /scan into a displayed ~9 Hz for the
rest of the run -- in the one window where the footer is the only live signal,
and in contradiction of `check_isaac_contract`'s own rate table.

No ROS, no numpy fixtures, no clock. The window is fed explicit timestamps so
the arithmetic is the subject rather than the scheduler.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools/lens"))

from _lens_core import RateWindow  # noqa: E402


def _feed(window, *, start, seconds, hz, keys=("scan",), step=0.2, counts=None):
    """Advance `seconds` at `hz`, returning (end_time, cumulative counts)."""

    counts = dict(counts or {k: 0 for k in keys})
    t = start
    end = start + seconds
    while t < end:
        t = round(t + step, 6)
        for k in keys:
            counts[k] += hz * step
        window.feed(t, {k: int(v) for k, v in counts.items()})
    return t, counts


def test_a_steady_publisher_reads_its_own_rate():
    window = RateWindow()
    _feed(window, start=0.0, seconds=30.0, hz=14.0)

    assert window.rates()["scan"] == pytest.approx(14.0, rel=0.05)


def test_silence_before_the_data_does_not_deflate_the_rate():
    """**The bug this class exists to prevent, in its measured shape.**

    130 s of pre-Isaac silence then 150 s at 14 Hz. A cumulative average reads
    150*14/280 = 7.5 Hz and keeps reading low for the whole run. The window
    reads what the publisher is actually doing.
    """

    window = RateWindow()
    end, counts = _feed(window, start=0.0, seconds=130.0, hz=0.0)
    _feed(window, start=end, seconds=150.0, hz=14.0, counts=counts)

    cumulative = counts["scan"] / (end + 150.0)
    assert cumulative < 8.0, "the fixture must reproduce the deflation"
    assert window.rates()["scan"] == pytest.approx(14.0, rel=0.05)


def test_a_topic_that_dies_is_visible_within_the_window():
    """What a lifetime average structurally cannot show."""

    window = RateWindow()
    end, counts = _feed(window, start=0.0, seconds=60.0, hz=14.0)
    assert window.rates()["scan"] == pytest.approx(14.0, rel=0.05)

    _feed(window, start=end, seconds=15.0, hz=0.0, counts=counts)
    assert window.rates()["scan"] == pytest.approx(0.0, abs=0.5)


def test_a_slow_topic_still_gets_enough_samples():
    """/map at ~1 Hz is why the window is 10 s and not 5."""

    window = RateWindow()
    _feed(window, start=0.0, seconds=30.0, hz=1.0, keys=("map",), step=1.0)

    assert window.rates()["map"] == pytest.approx(1.0, rel=0.2)


def test_it_abstains_rather_than_dividing_by_zero():
    window = RateWindow()
    assert window.rates() == {}
    window.feed(0.0, {"scan": 0})
    assert window.rates() == {}, "one sample is not a rate"
    window.feed(0.0, {"scan": 5})
    assert window.rates() == {}, "zero elapsed is not a rate"


def test_a_counter_that_goes_backwards_never_reports_negative():
    """Defensive: a restarted counter must read 0, not a negative rate."""

    window = RateWindow()
    window.feed(0.0, {"scan": 500})
    window.feed(10.0, {"scan": 3})

    assert window.rates()["scan"] == 0.0


def test_the_window_is_bounded():
    """It runs for the life of a lens that now outlives its run."""

    window = RateWindow()
    _feed(window, start=0.0, seconds=600.0, hz=14.0)

    assert len(window._samples) < 100, "the window is accumulating without bound"


# ------------------------------------------------------- the freeze predicate
#
# ADR 0035. The lens outlives its run so the operator can look afterwards -- but
# one left serving until morning would append empty samples for hours, roll the
# entire run out of the history buffer, and overwrite a good dump with nothing.
# Freezing is what makes the linger safe rather than destructive.

from _lens_core import FREEZE_IDLE_S, is_frozen  # noqa: E402


def test_a_lens_that_has_never_seen_anything_is_waiting_not_frozen():
    """**The bring-up case, and the one worth getting right.**

    The lens now starts before the simulator, so "no messages yet" is the
    normal state for the first ~70 s. Calling that frozen would put "run ended"
    on the page during bring-up, which is the opposite of the truth.
    """

    assert is_frozen(False, None, now=1000.0) is False
    assert is_frozen(False, None, now=1e6) is False


def test_a_live_session_is_not_frozen():
    assert is_frozen(True, 1000.0, now=1000.5) is False
    assert is_frozen(True, 1000.0, now=1000.0 + FREEZE_IDLE_S - 1.0) is False


def test_a_finished_session_freezes():
    assert is_frozen(True, 1000.0, now=1000.0 + FREEZE_IDLE_S + 1.0) is True


def test_the_idle_threshold_outlasts_an_ordinary_gap():
    """/map at ~1 Hz and a busy teardown must not read as a finished run."""

    assert FREEZE_IDLE_S >= 30.0, "too tight; a slow topic would freeze the page"
    assert FREEZE_IDLE_S <= 300.0, "too loose; the dump keeps growing after the run"
