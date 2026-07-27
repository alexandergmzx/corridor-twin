"""Violation events must count speeding episodes, not measurements.

The detector previously rearmed the instant it emitted, so a robot holding one
speed through a long over-limit stretch produced a fresh event every
``consecutive_estimates`` measurements. The event count therefore tracked how
many gates happened to be measurable rather than how the robot behaved. See
ADR 0014.
"""

from __future__ import annotations

import pytest
from police_observer.estimator import MarkerMap, SpeedMeasurement, ViolationDetector


@pytest.fixture()
def marker_map(tmp_path):
    from scene.build import build_scene

    _, manifest = build_scene(None, tmp_path / "corridor.usda", 6.0, 3.0)
    return MarkerMap.from_manifest(manifest)


def _measure(
    timestamp_s: float,
    speed_mps: float,
    limit_mps: float = 1.2,
    station_m: float = 4.0,
    width_m: float = 5.0,
) -> SpeedMeasurement:
    return SpeedMeasurement(
        timestamp_s=timestamp_s,
        station_m=station_m,
        speed_mps=speed_mps,
        speed_stddev_mps=0.001,
        corridor_width_m=width_m,
        speed_limit_mps=limit_mps,
        gate_from_id=0,
        gate_to_id=1,
        observation_count=10,
    )


def _events(detector: ViolationDetector, measurements) -> list:
    return [event for m in measurements if (event := detector.update(m)) is not None]


def test_sustained_speeding_emits_exactly_one_event(marker_map) -> None:
    detector = ViolationDetector(marker_map)
    sustained = [_measure(1.0 + index, 1.8) for index in range(8)]
    events = _events(detector, sustained)
    assert len(events) == 1
    # Confirmation still needs the configured consecutive count, so the event
    # lands on the second measurement, not the first.
    assert events[0].estimate.timestamp_s == pytest.approx(1.0 + 1)


def test_a_compliant_measurement_rearms_the_detector(marker_map) -> None:
    detector = ViolationDetector(marker_map)
    assert detector.update(_measure(1.0, 1.8)) is None
    assert detector.update(_measure(2.0, 1.8)) is not None
    assert detector.episode_open
    assert detector.update(_measure(3.0, 0.9)) is None
    assert not detector.episode_open
    assert detector.consecutive == 0


def test_a_second_independent_episode_emits_a_second_event(marker_map) -> None:
    detector = ViolationDetector(marker_map)
    sequence = [
        _measure(1.0, 1.8),
        _measure(2.0, 1.8),  # first event
        _measure(3.0, 1.8),
        _measure(4.0, 0.9),  # compliant: rearms
        _measure(5.0, 1.8),
        _measure(6.0, 1.8),  # second event
        _measure(7.0, 1.8),
    ]
    events = _events(detector, sequence)
    assert len(events) == 2
    assert [event.event_id for event in events] == [1, 2]
    assert events[1].estimate.timestamp_s == pytest.approx(6.0)


def test_entering_a_stricter_zone_mid_episode_does_not_duplicate(marker_map) -> None:
    """A constant speed must not become a new offense when the limit tightens.

    The corridor narrows from 1.5 to 1.2 to 0.8 m/s, so a steady over-limit run
    crosses into stricter zones. That is the same episode, not a new one.
    """

    detector = ViolationDetector(marker_map)
    sequence = [
        _measure(1.0, 1.8, limit_mps=1.2, station_m=4.0, width_m=5.0),
        _measure(2.0, 1.8, limit_mps=1.2, station_m=6.0, width_m=4.5),  # event
        _measure(3.0, 1.8, limit_mps=1.2, station_m=8.0, width_m=4.0),
        _measure(4.0, 1.8, limit_mps=0.8, station_m=10.0, width_m=3.5),  # stricter
        _measure(5.0, 1.8, limit_mps=0.8, station_m=10.0, width_m=3.5),
    ]
    events = _events(detector, sequence)
    assert len(events) == 1
    assert events[0].estimate.speed_limit_mps == pytest.approx(1.2)


def test_reset_clears_an_open_episode(marker_map) -> None:
    """Temporal discontinuities cannot leave an episode open.

    After a clock epoch or profile change, continuity across the discontinuity
    cannot be asserted, so the next confirmed exceedance is a new episode.
    """

    detector = ViolationDetector(marker_map)
    assert detector.update(_measure(1.0, 1.8)) is None
    first = detector.update(_measure(2.0, 1.8))
    assert first is not None and detector.episode_open

    detector.reset()
    assert not detector.episode_open
    assert detector.consecutive == 0
    assert detector.first_time_s is None

    assert detector.update(_measure(3.0, 1.8)) is None
    second = detector.update(_measure(4.0, 1.8))
    assert second is not None
    assert second.event_id == 2


def test_confirmation_duration_measures_from_the_first_confirming_estimate(
    marker_map,
) -> None:
    """A longer episode shows in the event's own fields, not as repeat events."""

    detector = ViolationDetector(marker_map)
    detector.update(_measure(10.0, 1.8))
    event = detector.update(_measure(11.5, 1.8))
    assert event is not None
    assert event.confirmation_duration_s == pytest.approx(1.5)
