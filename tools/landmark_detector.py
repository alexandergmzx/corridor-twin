#!/usr/bin/env python3
"""Find B's post in a laser scan, by SHAPE, without consulting the map.

    from landmark_detector import LandmarkDetector
    detector = LandmarkDetector(radius_m=manifest["actors"]["landmark_radius_m"])
    hit = detector.feed(ranges, angle_min, angle_increment, range_min, range_max)

WHY A SHAPE, AND WHY NOT INTENSITY
----------------------------------
A cylinder is circular in section, so it returns an arc of KNOWN RADIUS from
every bearing. A flat wall returns a straight run; a convex corner returns two
straight runs meeting at a point. Those are separable by fitting a circle and
looking at the residual, and the separation holds in simulation and on hardware
because it is geometry.

Return INTENSITY would be easier and is rejected: sim-vs-real intensity
fidelity is an unowned contract question in this project, and a detector that
only works in Isaac proves nothing about the robot. Nothing here reads
`LaserScan.intensities`.

WHY IT MATTERS THAT THIS IGNORES THE MAP
---------------------------------------
Every other measurement of "where is B" in this system passes through the SLAM
map, and the map diverges. This one is taken in the LASER frame from a single
scan. It is wrong exactly when the lidar is wrong, and it does not care what the
map believes, which is why it survives a divergence that invalidates every
map-frame number.

WHAT THIS IS NOT
----------------
Not an arrival mechanism, and not a motion primitive. It reports a range and
bearing; deciding anything with that is the caller's business, and the caller
never uses it to search -- the MS200 is 360 deg, so acquisition needs no motion
at all.

CONFIRMATION, NOT DETECTION
---------------------------
A single frame is never trusted. `feed` returns a candidate per scan and the
detector confirms only after K of the last N frames agree on a position, which
is what stops a chance alignment of corner geometry from triggering once and
being believed forever.
"""

from __future__ import annotations

import math
from collections import deque

#: A cylinder's arc is only recoverable if enough beams land on it. Below this
#: the fit is under-determined and will happily "fit" a corner: three points
#: define a circle exactly, so three points ALWAYS fit with zero residual.
MIN_POINTS = 4

#: Fit residual, as a fraction of the landmark's radius. A wall's points fit a
#: circle of the right radius badly; this is the number that says so. Scaled by
#: radius rather than absolute so the criterion survives a scenario rescale.
MAX_RESIDUAL_FRACTION = 0.30

#: The fitted radius has to be the AUTHORED one. A corner can produce a
#: low-residual circle of the wrong size, and this is what rejects it.
MAX_RADIUS_ERROR_FRACTION = 0.40

#: Points further apart than this along the scan start a new cluster. It is
#: expressed in multiples of the landmark's diameter so that it, too, rescales.
CLUSTER_GAP_FACTOR = 1.5

#: k-of-n confirmation. One frame is never enough.
CONFIRM_K, CONFIRM_N = 3, 5

#: Two candidates are "the same landmark" if their centres are within this.
AGREEMENT_M = 0.25


def scan_to_xy(ranges, angle_min: float, angle_increment: float,
               range_min: float, range_max: float) -> list[tuple[float, float]]:
    """Valid beams only, as (x, y) in the laser frame.

    Isaac's RTX lidar reports no-return as a sentinel rather than as inf, so
    the range window is applied here rather than trusted from upstream -- a
    sentinel that survives into the clustering is a phantom point at the origin.
    """

    points = []
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            continue
        angle = angle_min + index * angle_increment
        points.append((distance * math.cos(angle), distance * math.sin(angle)))
    return points


def cluster(points, max_gap_m: float) -> list[list[tuple[float, float]]]:
    """Split an ANGULARLY ORDERED scan into runs of adjacent nearby points.

    Order is what makes this O(n) with no spatial index: consecutive beams are
    neighbours by construction. Beams dropped by the range filter break
    adjacency, which is correct -- a gap in the returns is a gap in the world.
    """

    clusters, current = [], []
    for point in points:
        if current and math.dist(current[-1], point) > max_gap_m:
            clusters.append(current)
            current = []
        current.append(point)
    if current:
        clusters.append(current)
    return clusters


def fit_circle(points) -> tuple[float, float, float, float] | None:
    """Algebraic (Kasa) circle fit. -> (cx, cy, r, rms residual) or None.

    Kasa rather than a geometric fit because the input is a short arc from one
    scan and the closed form is exact, cheap and dependency-free. Its known
    weakness -- biased radius on arcs under ~90 deg -- is acceptable here
    because the radius is CHECKED against the authored value rather than
    trusted, and a biased fit fails that check in the safe direction.
    """

    count = len(points)
    if count < 3:
        return None

    mean_x = sum(x for x, _ in points) / count
    mean_y = sum(y for _, y in points) / count
    # Centred, so the normal equations stay well conditioned at range: a cluster
    # 8 m from the origin otherwise squares to 64 and swamps the fit.
    us = [x - mean_x for x, _ in points]
    vs = [y - mean_y for _, y in points]

    suu = sum(u * u for u in us)
    svv = sum(v * v for v in vs)
    suv = sum(u * v for u, v in zip(us, vs, strict=False))
    suuu = sum(u ** 3 for u in us)
    svvv = sum(v ** 3 for v in vs)
    suvv = sum(u * v * v for u, v in zip(us, vs, strict=False))
    svuu = sum(v * u * u for u, v in zip(us, vs, strict=False))

    determinant = 2.0 * (suu * svv - suv * suv)
    if abs(determinant) < 1e-12:
        # Collinear points: a straight wall. There is no circle, and saying so
        # is the whole job.
        return None

    center_u = (svv * (suuu + suvv) - suv * (svvv + svuu)) / determinant
    center_v = (suu * (svvv + svuu) - suv * (suuu + suvv)) / determinant
    radius = math.sqrt(center_u ** 2 + center_v ** 2 + (suu + svv) / count)

    cx, cy = center_u + mean_x, center_v + mean_y
    residual = math.sqrt(
        sum((math.dist((x, y), (cx, cy)) - radius) ** 2 for x, y in points) / count
    )
    return cx, cy, radius, residual


class LandmarkDetector:
    """Stateful across frames only for the k-of-n confirmation."""

    def __init__(self, radius_m: float, *, confirm_k: int = CONFIRM_K,
                 confirm_n: int = CONFIRM_N) -> None:
        if radius_m <= 0.0:
            raise ValueError("landmark radius must be positive")
        self.radius_m = radius_m
        self.confirm_k = confirm_k
        self.recent: deque = deque(maxlen=confirm_n)

    # --- one frame ---------------------------------------------------------
    def candidates(self, points) -> list[dict]:
        """Every cluster in this frame that fits a circle of the right size."""

        found = []
        for group in cluster(points, self.radius_m * 2.0 * CLUSTER_GAP_FACTOR):
            if len(group) < MIN_POINTS:
                continue
            fit = fit_circle(group)
            if fit is None:
                continue
            cx, cy, radius, residual = fit
            if residual > self.radius_m * MAX_RESIDUAL_FRACTION:
                continue
            if abs(radius - self.radius_m) > self.radius_m * MAX_RADIUS_ERROR_FRACTION:
                continue
            found.append({
                "x": round(cx, 4), "y": round(cy, 4),
                "range_m": round(math.hypot(cx, cy), 4),
                "bearing_rad": round(math.atan2(cy, cx), 4),
                "fitted_radius_m": round(radius, 4),
                "residual_m": round(residual, 5),
                "points": len(group),
            })
        # Best fit first: lowest residual is the most circle-like thing seen.
        return sorted(found, key=lambda entry: entry["residual_m"])

    def feed(self, ranges, angle_min: float, angle_increment: float,
             range_min: float, range_max: float) -> dict:
        """One scan in, one verdict out. Confirmed only on k-of-n agreement."""

        points = scan_to_xy(ranges, angle_min, angle_increment, range_min, range_max)
        found = self.candidates(points)
        best = found[0] if found else None
        self.recent.append(best)

        agreeing = 0
        if best is not None:
            agreeing = sum(
                1 for entry in self.recent
                if entry is not None
                and math.dist((entry["x"], entry["y"]), (best["x"], best["y"])) <= AGREEMENT_M
            )

        return {
            "candidate": best,
            "candidates_this_frame": len(found),
            "frames_agreeing": agreeing,
            "frames_considered": len(self.recent),
            "confirmed": best is not None and agreeing >= self.confirm_k,
        }

    def reset(self) -> None:
        self.recent.clear()
