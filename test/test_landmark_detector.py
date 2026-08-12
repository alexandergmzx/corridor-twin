"""The landmark detector, against the things that look like a landmark.

A detector that finds the post is easy. This file is mostly about the four
false positives that would matter, because a phantom landmark is worse than no
landmark: it would move the delivery goal to a place nothing is.

  * a flat WALL at the same range,
  * a convex CORNER at the same range -- the corridor is full of them,
  * a cylinder of the WRONG radius,
  * a single lucky frame, which must not confirm on its own.

Scans are synthesised here rather than replayed so each case is exactly the one
thing it claims to be. Ranges are built by raycasting into wall segments, which
is the same construction the fleet's own scan fixtures use.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from landmark_detector import (  # noqa: E402
    CONFIRM_K,
    LandmarkDetector,
    cluster,
    fit_circle,
    scan_to_xy,
)

RADIUS = 0.12           # the authored landmark; absolute, not scaled with the scene
RANGE_MIN, RANGE_MAX = 0.12, 8.0
BEAMS = 360             # MS200: 360 beams over 360 degrees


def _scan(hit_fn):
    """A 360-beam scan whose range at each bearing comes from `hit_fn`."""

    increment = 2.0 * math.pi / BEAMS
    angle_min = -math.pi
    ranges = []
    for index in range(BEAMS):
        angle = angle_min + index * increment
        ranges.append(hit_fn(angle))
    return ranges, angle_min, increment


def _circle_hit(cx, cy, radius):
    """Ray-circle intersection: the range a beam would read off a cylinder."""

    def hit(angle):
        dx, dy = math.cos(angle), math.sin(angle)
        # |t*d - c|^2 = r^2
        b = dx * cx + dy * cy
        c = cx * cx + cy * cy - radius * radius
        discriminant = b * b - c
        if discriminant < 0.0:
            return float("inf")
        t = b - math.sqrt(discriminant)
        return t if t > 0 else float("inf")
    return hit


def _wall_hit(distance, normal_angle):
    """A flat wall at `distance`, perpendicular to `normal_angle`."""

    def hit(angle):
        cosine = math.cos(angle - normal_angle)
        if cosine <= 1e-6:
            return float("inf")
        return distance / cosine
    return hit


def _combine(*hits):
    def hit(angle):
        return min(h(angle) for h in hits)
    return hit


def _points(hit_fn):
    ranges, angle_min, increment = _scan(hit_fn)
    return scan_to_xy(ranges, angle_min, increment, RANGE_MIN, RANGE_MAX)


# --- the geometry primitives -------------------------------------------------


def test_a_circle_fit_recovers_a_known_circle() -> None:
    circle = [(1.0 + RADIUS * math.cos(a), 0.4 + RADIUS * math.sin(a))
              for a in [i * 0.3 for i in range(12)]]

    cx, cy, radius, residual = fit_circle(circle)

    assert (cx, cy) == pytest.approx((1.0, 0.4), abs=1e-6)
    assert radius == pytest.approx(RADIUS, abs=1e-6)
    assert residual < 1e-6


def test_collinear_points_have_no_circle() -> None:
    """A wall must return None rather than an enormous circle."""

    assert fit_circle([(1.0, y * 0.05) for y in range(10)]) is None


def test_clustering_splits_on_a_gap() -> None:
    points = [(1.0, y * 0.01) for y in range(5)] + [(1.0, 1.0 + y * 0.01) for y in range(5)]

    assert len(cluster(points, 0.2)) == 2


# --- the real thing ----------------------------------------------------------


def test_the_landmark_is_found_and_located() -> None:
    detector = LandmarkDetector(RADIUS)
    truth = (1.2, 0.0)

    found = detector.candidates(_points(_circle_hit(*truth, RADIUS)))

    assert found, "the post was not detected at all"
    best = found[0]
    assert math.dist((best["x"], best["y"]), truth) < 0.05
    assert best["fitted_radius_m"] == pytest.approx(RADIUS, abs=RADIUS * 0.4)


# --- the false positives that would matter -----------------------------------


def test_a_flat_wall_at_the_same_range_is_not_a_landmark() -> None:
    found = LandmarkDetector(RADIUS).candidates(_points(_wall_hit(1.2, 0.0)))

    assert found == [], f"a wall was detected as a landmark: {found}"


def test_a_convex_corner_at_the_same_range_is_not_a_landmark() -> None:
    """The corridor is full of these, and it is where A actually is."""

    corner = _combine(_wall_hit(1.2, 0.0), _wall_hit(1.2, math.pi / 2.0))

    found = LandmarkDetector(RADIUS).candidates(_points(corner))

    assert found == [], f"a corner was detected as a landmark: {found}"


def test_a_cylinder_of_the_wrong_radius_is_rejected() -> None:
    """A bin, a bollard, a table leg. Circular, but not B's post."""

    found = LandmarkDetector(RADIUS).candidates(_points(_circle_hit(1.2, 0.0, RADIUS * 4)))

    assert found == [], f"a wrong-sized cylinder was accepted: {found}"


def test_the_landmark_is_still_found_beside_a_wall() -> None:
    """The realistic case: B stands near the street's edge, not in free space."""

    scene = _combine(_circle_hit(1.2, 0.0, RADIUS), _wall_hit(2.0, math.pi / 2.0))

    found = LandmarkDetector(RADIUS).candidates(_points(scene))

    assert found, "the post was lost once a wall was in the scene"
    assert math.dist((found[0]["x"], found[0]["y"]), (1.2, 0.0)) < 0.05


# --- confirmation ------------------------------------------------------------


def test_one_lucky_frame_does_not_confirm() -> None:
    """The k-of-n rule, which is what makes a phantom harmless."""

    detector = LandmarkDetector(RADIUS)
    ranges, angle_min, increment = _scan(_circle_hit(1.2, 0.0, RADIUS))

    verdict = detector.feed(ranges, angle_min, increment, RANGE_MIN, RANGE_MAX)

    assert verdict["candidate"] is not None
    assert verdict["confirmed"] is False
    assert verdict["frames_agreeing"] == 1


def test_k_consistent_frames_confirm() -> None:
    detector = LandmarkDetector(RADIUS)
    ranges, angle_min, increment = _scan(_circle_hit(1.2, 0.0, RADIUS))

    verdicts = [
        detector.feed(ranges, angle_min, increment, RANGE_MIN, RANGE_MAX)
        for _ in range(CONFIRM_K)
    ]

    assert verdicts[-1]["confirmed"] is True
    assert [v["confirmed"] for v in verdicts[:-1]] == [False] * (CONFIRM_K - 1)


def test_frames_that_disagree_on_position_do_not_confirm() -> None:
    """Detections that jump around are noise, however circle-like each one is."""

    detector = LandmarkDetector(RADIUS)
    for offset in (0.0, 1.0, 2.0, 3.0, 4.0):
        ranges, angle_min, increment = _scan(_circle_hit(1.2, offset, RADIUS))
        verdict = detector.feed(ranges, angle_min, increment, RANGE_MIN, RANGE_MAX)

    assert verdict["confirmed"] is False


def test_an_empty_scan_confirms_nothing() -> None:
    detector = LandmarkDetector(RADIUS)
    ranges, angle_min, increment = _scan(lambda _a: float("inf"))

    verdict = detector.feed(ranges, angle_min, increment, RANGE_MIN, RANGE_MAX)

    assert verdict["candidate"] is None
    assert verdict["confirmed"] is False


def test_out_of_range_returns_are_dropped_not_placed_at_the_origin() -> None:
    """Isaac reports no-return as a sentinel; a survivor is a phantom point."""

    ranges, angle_min, increment = _scan(lambda _a: -1.0)

    assert scan_to_xy(ranges, angle_min, increment, RANGE_MIN, RANGE_MAX) == []


def test_the_radius_must_be_positive() -> None:
    with pytest.raises(ValueError):
        LandmarkDetector(0.0)


def test_a_post_shaped_wall_edge_is_rejected_by_ISOLATION() -> None:
    """The phantom that hijacked two runs, as a unit test.

    A wall feature 0.874 m from the robot fitted a circle of radius 0.1276
    against an authored 0.12, with a good residual and a post-sized chord. It
    passed every SHAPE test the detector had and confirmed 3-of-5, while the
    real post stood five metres away.

    No curve-fitting can separate these: the feature genuinely is post-shaped.
    What separates them is context -- this arc is ATTACHED to a wall that runs
    on at similar range, where a real post has open space behind it.
    """

    detector = LandmarkDetector(RADIUS)
    # A post-shaped convex nub at 0.9 m, with wall continuing away on one side
    # at a similar range: exactly a corridor's wall edge.
    nub = [
        (0.9 + RADIUS * math.cos(math.radians(a)), RADIUS * math.sin(math.radians(a)))
        for a in range(-60, 61, 8)
    ]
    wall = [(0.95, y) for y in [0.16 + 0.02 * i for i in range(14)]]

    found = detector.candidates(nub + wall)

    assert found == [], f"a wall edge was accepted as the post: {found}"


def test_the_real_post_still_passes_the_extent_check() -> None:
    """The guard must not reject the thing it guards."""

    found = LandmarkDetector(RADIUS).candidates(_points(_circle_hit(1.2, 0.0, RADIUS)))

    assert found, "the extent check rejected the actual post"
    assert math.dist((found[0]["x"], found[0]["y"]), (1.2, 0.0)) < 0.05
