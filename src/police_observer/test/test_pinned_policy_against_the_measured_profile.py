"""The pinned policy, judged by the observer's own detector.

ADR 0038. The three numbers were chosen from a table of measured speeds, and a
table is not a verdict: what decides whether this demonstration has a violation
in it is `ViolationDetector`, running the real confirmation rule against the
real `limit_at`. So these tests feed robot1's measured profile through the
shipped pipeline and assert what comes out.

**The profile is ground truth, and it is an evaluation input.** It derives the
policy the observer is later judged against; it never reaches the observer.
Invariant 1.

Measured 2026-08-14 across six delivery runs, `/sim/ground_truth`, secant over
a +/-0.30 m window of travel — `out/evidence/speed-profile/measured-profile.json`
and `tools/measure_speed_profile.py`.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from police_observer.estimator import MarkerMap, SpeedMeasurement, ViolationDetector

#: (station_m, mean, min, max) per gate, from the six-run profile.
MEASURED = (
    (0.6, 0.1967, 0.1731, 0.2066),
    (1.2, 0.1960, 0.1891, 0.2023),
    (1.8, 0.1689, 0.1399, 0.1818),
    (2.4, 0.1285, 0.1125, 0.1420),
    (3.0, 0.0807, 0.0555, 0.1028),
)

MANIFEST = Path(__file__).resolve().parents[3] / "out/corridor.manifest.json"


@pytest.fixture
def marker_map() -> MarkerMap:
    """**The shipped manifest**, not a hand-built map.

    The pin travels config -> `scale_scenario` -> scenario YAML -> `scene.build`
    -> manifest -> `MarkerMap`, and every one of those is a place it could
    arrive wrong. Reading the artifact the observer actually loads is the only
    version of this test that covers the path.
    """

    if not MANIFEST.is_file():
        pytest.skip(f"{MANIFEST} not generated; run scene.build")
    marker_map = MarkerMap.from_manifest(MANIFEST, "nominal_m6_n3")
    # approx, not equality: the manifest carries gate 1.8 as 1.7999999999999998,
    # which is the taper arithmetic and not a moved gate.
    assert marker_map.gate_stations_m == pytest.approx(
        [station for station, *_ in MEASURED]), (
        "the gates moved; the measured profile no longer describes this scenario"
    )
    return marker_map


def test_the_pin_reached_the_manifest_the_observer_loads(marker_map) -> None:
    """One constant, printed and enforced from one place (gate discipline)."""

    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
    from scale_scenario import PINNED_LIMITS_MPS

    # speed_rules is normalised ascending by width; the pin is widest first.
    assert tuple(limit for _width, limit in reversed(marker_map.speed_rules)) == \
        PINNED_LIMITS_MPS, (
            "the manifest's limits are not the pinned ones -- regenerate the "
            "scenario and the manifest, or the demo enforces a stale policy"
        )


def _limits(marker_map: MarkerMap) -> dict[float, float]:
    return {station: marker_map.limit_at(station) for station, *_ in MEASURED}


def test_the_zone_membership_is_unchanged_by_the_pin(marker_map) -> None:
    """**Rider 1, first half.** A' differs from A only in the strict speed, so
    which gate sits in which zone must be identical -- and identical to what
    ADR 0016 decided, which the pin does not reopen."""

    limits = _limits(marker_map)
    strict = min(limits.values())

    assert [station for station, limit in limits.items() if limit == strict] == [2.4, 3.0], (
        "the strict zone no longer holds exactly gates 2.4 and 3.0; ADR 0016's "
        "boundary has moved, which this pin was chosen not to do"
    )
    assert limits[0.6] == 0.30 and limits[1.2] == 0.25 and limits[1.8] == 0.25


def test_every_permissive_gate_clears_the_fastest_run(marker_map) -> None:
    """The compliant stretch must be compliant on the WORST case for it, which
    is the fastest of the six runs, not the mean."""

    limits = _limits(marker_map)
    for station, _mean, _lo, hi in MEASURED:
        if limits[station] == min(limits.values()):
            continue
        assert hi < limits[station], (
            f"gate {station}: fastest measured {hi} m/s is not below its "
            f"{limits[station]} m/s limit, so the compliant stretch is not "
            f"reliably compliant"
        )


def test_the_two_gate_floor_holds_on_the_slowest_run(marker_map) -> None:
    """**Rider 1, second half, and the reason A' was picked over B'.**

    ADR 0016: a violation confined to the corner can only be confirmed if the
    strict zone holds two gates, because `consecutive_estimates` is 2. Both
    must therefore be over-limit on the SLOWEST of the six runs, or a run
    exists in which nothing is confirmed.
    """

    limits = _limits(marker_map)
    strict = min(limits.values())
    over = [station for station, _mean, lo, _hi in MEASURED
            if limits[station] == strict and lo > strict]

    assert over == [2.4, 3.0], (
        f"only {over} exceed the strict limit on the slowest run; the "
        f"confirmation rule needs two and would fire on no run at all"
    )


def test_the_strict_limit_avoids_the_governor_creep_clamp(marker_map) -> None:
    """**Rider 2, asserted rather than only written down.** 0.05 m/s is the
    governor's creep clamp on A's plane. Sharing the number across two planes
    invites a reader to think one causes the other, and puts a measurement at
    exact boundary equality with a commanded speed."""

    assert min(_limits(marker_map).values()) != 0.05, (
        "the strict limit is the creep clamp again: one constant, two meanings"
    )


@pytest.mark.parametrize("case", ["mean", "slowest", "fastest"])
def test_the_measured_profile_produces_exactly_one_violation(marker_map, case) -> None:
    """**The headline, from the shipped detector.** Five gate measurements in,
    one event out, first confirmed at gate 3.0 -- on all three cases, so the
    demonstration does not depend on which of the six runs is recorded.

    A perfect estimator is assumed here (`speed_stddev_mps` = 0). What the
    real uncertainty does to these margins is F4's measurement, not an
    assumption this test is allowed to make.
    """

    pick = {"mean": 1, "slowest": 2, "fastest": 3}[case]
    detector = ViolationDetector(marker_map)

    events = []
    for index, row in enumerate(MEASURED):
        station, speed = row[0], row[pick]
        measurement = SpeedMeasurement(
            timestamp_s=float(index),
            station_m=station,
            speed_mps=speed,
            speed_stddev_mps=0.0,
            speed_limit_mps=marker_map.limit_at(station),
            corridor_width_m=marker_map.width_at(station),
            gate_from_id=index, gate_to_id=index + 1,
            observation_count=2,
        )
        violation = detector.update(measurement)
        if violation:
            events.append((station, violation))

    assert len(events) == 1, f"{len(events)} violations, expected exactly one"
    station, violation = events[0]
    assert station == 3.0, "confirmation did not land on the second strict gate"
    assert violation.event_id == 1
    assert violation.exceedance_mps > 0


def test_an_episode_open_at_route_end_emits_nothing_further(marker_map) -> None:
    """**Rider 4.** The A' episode runs through the final gate, so it is still
    open when the route ends.

    ADR 0014's semantics have no close event by design: the episode IS the one
    emitted `Violation`, and closure exists only to rearm the detector. So an
    episode still open at route end is not a leak and not a dropped event --
    it is the recorded state of a robot that was still speeding when it ran
    out of corridor. Asserted rather than assumed, because "one event per
    episode" and "an episode that never closes" sound like they conflict.
    """

    detector = ViolationDetector(marker_map)
    for index, (station, mean, *_rest) in enumerate(MEASURED):
        detector.update(SpeedMeasurement(
            timestamp_s=float(index), station_m=station, speed_mps=mean,
            speed_stddev_mps=0.0, speed_limit_mps=marker_map.limit_at(station),
            corridor_width_m=marker_map.width_at(station),
            gate_from_id=index, gate_to_id=index + 1, observation_count=2))

    assert detector.episode_open, "the episode closed before the route ended"

    # Past the last gate, still over the strict limit: no second event.
    extra = detector.update(SpeedMeasurement(
        timestamp_s=99.0, station_m=3.0, speed_mps=0.0807,
        speed_stddev_mps=0.0, speed_limit_mps=marker_map.limit_at(3.0),
        corridor_width_m=marker_map.width_at(3.0),
        gate_from_id=4, gate_to_id=5, observation_count=2))
    assert extra is None, "an open episode emitted a second event"

    # And compliance still rearms it, so nothing is stuck.
    assert detector.update(SpeedMeasurement(
        timestamp_s=100.0, station_m=3.0, speed_mps=0.0,
        speed_stddev_mps=0.0, speed_limit_mps=marker_map.limit_at(3.0),
        corridor_width_m=marker_map.width_at(3.0),
        gate_from_id=4, gate_to_id=5, observation_count=2)) is None
    assert not detector.episode_open, "compliance no longer rearms the detector"
