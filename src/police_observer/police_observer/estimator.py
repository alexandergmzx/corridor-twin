"""ROS-independent ArUco pose, gate-speed, and violation estimation."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class Calibration:
    """Camera intrinsics for the pixels delivered to the observer."""

    width: int
    height: int
    matrix: np.ndarray
    distortion: np.ndarray
    frame_id: str
    distortion_model: str


def calibration_materially_changed(previous: Calibration | None, current: Calibration) -> bool:
    """Return whether a new frame's calibration invalidates the observation window.

    A changed K, D, dimensions, distortion model, or frame reinterprets every
    pixel against a different model, so differencing a station computed under
    the old calibration against one computed under the new one would silently
    mix two pixel models into one speed. ``Calibration`` carries no timestamp,
    so a stamp-only change can never trigger this (A6-M3).
    """

    if previous is None:
        return False
    return (
        previous.width != current.width
        or previous.height != current.height
        or previous.frame_id != current.frame_id
        or previous.distortion_model != current.distortion_model
        or not np.array_equal(previous.matrix, current.matrix)
        or not np.array_equal(previous.distortion, current.distortion)
    )


@dataclass(frozen=True)
class PoseObservation:
    """Camera station inferred only from image pixels and the survey."""

    timestamp_s: float
    station_m: float
    station_stddev_m: float
    reprojection_rmse_px: float
    marker_ids: tuple[int, ...]


@dataclass(frozen=True)
class SpeedMeasurement:
    """Speed between two surveyed station gates."""

    timestamp_s: float
    station_m: float
    speed_mps: float
    speed_stddev_mps: float
    corridor_width_m: float
    speed_limit_mps: float
    gate_from_id: int
    gate_to_id: int
    observation_count: int


@dataclass(frozen=True)
class Violation:
    """Debounced, conservative policy exceedance."""

    event_id: int
    estimate: SpeedMeasurement
    exceedance_mps: float
    confirmation_duration_s: float


#: Metres of slack on a policy zone boundary. One nanometre: far below any
#: length this scenario means anything at, and far above the float error that
#: put a gate in the wrong zone.
POLICY_WIDTH_EPSILON_M = 1e-9


def covered_by(width_m: float, maximum_width_m: float) -> bool:
    """Is `width_m` inside a rule whose threshold is `maximum_width_m`?

    **A policy boundary is not decided by the 16th decimal.** The nominal
    profile's gate at station 2.4 has a clear width of exactly 1.2 m by
    construction -- it is the corridor's midpoint on a linear taper -- and the
    strict rule's threshold is exactly 1.2 m. ADR 0016 chose that boundary so
    the strict zone would hold TWO gates, because `consecutive_estimates` is 2
    and a corner-confined violation cannot otherwise be confirmed.

    Evaluated in floating point, `1.8 + (2.4 / 3.6) * (0.9 - 1.8)` is
    `1.2000000000000002`, and a bare `<=` therefore put that gate in the
    permissive zone. The strict zone held one gate, the confirmation rule could
    never fire from the corner, and the demonstration's central claim would
    have produced zero violations with nothing to point at.

    It went unnoticed because the defect is scale-dependent. At v1's authored
    metres the same expression, `6.0 + (8.0 / 12.0) * (3.0 - 6.0)`, is exactly
    `4.0` and the comparison holds -- so ADR 0016's own arithmetic was correct
    when it was written, and ADR 0030's 0.30 scaling silently broke it while
    every v1 test stayed green.

    A tolerance, rather than rounding the widths or nudging the threshold: the
    threshold is ADR 0016's decision and must not move, and rounding would put
    a different arbitrary precision in the same place with less to say for
    itself.
    """

    return width_m <= maximum_width_m + POLICY_WIDTH_EPSILON_M


def normalized_speed_rules(rules: object) -> tuple[tuple[float, float], ...]:
    """Return policy rules as ascending, validated (maximum_width_m, limit_mps).

    ``limit_at`` returns the first rule whose threshold covers the width, which
    is only correct on a sorted list. Nothing enforced that: the policy travels
    from YAML through the manifest as an opaque dictionary, so reversing the
    rules -- a semantically identical set -- silently made every gate 1.5 m/s
    and deleted the corner rule from the demonstration.

    A piecewise-by-threshold policy has no meaningful order, so the fix is to
    normalize rather than to reject. What *is* rejected is a set that cannot
    describe a policy at all: no rules, a repeated threshold (which would make
    the winning rule depend on input order again), or a non-positive limit.
    """

    if not isinstance(rules, list) or not rules:
        raise ValueError("speed policy must define at least one rule")
    parsed: list[tuple[float, float]] = []
    for index, rule in enumerate(rules):
        try:
            maximum_width = float(rule["maximum_width_m"])
            limit = float(rule["limit_mps"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"speed policy rule {index} needs numeric maximum_width_m and limit_mps"
            ) from error
        # Reported separately from the sign check so the message names the
        # actual fault: `inf` is positive, and calling it "non-positive" sends
        # a reader looking for the wrong thing in their config.
        if not math.isfinite(maximum_width):
            raise ValueError(f"speed policy rule {index} has a non-finite maximum_width_m")
        if not math.isfinite(limit):
            raise ValueError(f"speed policy rule {index} has a non-finite limit_mps")
        if maximum_width <= 0.0:
            raise ValueError(f"speed policy rule {index} has a non-positive maximum_width_m")
        if limit <= 0.0:
            raise ValueError(f"speed policy rule {index} has a non-positive limit_mps")
        parsed.append((maximum_width, limit))

    thresholds = [maximum_width for maximum_width, _ in parsed]
    duplicates = sorted({value for value in thresholds if thresholds.count(value) > 1})
    if duplicates:
        raise ValueError(f"speed policy repeats maximum_width_m {duplicates}")
    return tuple(sorted(parsed))


@dataclass(frozen=True)
class MarkerMap:
    """Survey and policy read from the generated manifest."""

    profile_name: str
    marker_corners: dict[int, np.ndarray]
    gate_stations_m: tuple[float, ...]
    corridor_length_m: float
    entry_width_m: float
    corner_width_m: float
    speed_rules: tuple[tuple[float, float], ...]
    confidence_sigma: float
    consecutive_estimates: int
    # Station is measured along world X, which is also how markers are
    # surveyed. Under a one-sided taper the delivery path runs at a small angle
    # to X, so an X displacement is shorter than the distance actually
    # travelled. This is the X component of the path's unit heading; dividing
    # by it converts an axis speed into the true speed the policy is about.
    path_axis_fraction: float = 1.0

    @classmethod
    def from_manifest(cls, path: Path, profile_name: str | None = None) -> MarkerMap:
        raw = json.loads(path.read_text(encoding="utf-8"))
        selected = profile_name or str(raw["selected_profile"])
        profile = raw["profiles"][selected]
        marker_corners = {
            int(marker["id"]): np.asarray(marker["aruco_corner_order_xyz_m"], dtype=np.float64)
            for marker in profile["markers"]
        }
        # A marker without an explicit role is a gate, so schema-0.2 manifests
        # and preserved historical evidence stay readable. An unknown role is a
        # hard error: silently treating it as a reference would drop a real
        # enforcement station, and silently treating it as a gate would invent
        # one the robot never crosses.
        roles = {marker["id"]: str(marker.get("role", "gate")) for marker in profile["markers"]}
        unknown = sorted({role for role in roles.values()} - {"gate", "reference"})
        if unknown:
            raise ValueError(f"manifest uses unknown marker roles: {unknown}")
        stations = tuple(
            sorted(
                {
                    float(marker["station_m"])
                    for marker in profile["markers"]
                    if roles[marker["id"]] == "gate"
                }
            )
        )
        policy = raw["speed_policy"]
        rules = normalized_speed_rules(policy.get("rules"))
        marker_map = cls(
            profile_name=selected,
            marker_corners=marker_corners,
            gate_stations_m=stations,
            corridor_length_m=float(raw["corridor_length_m"]),
            entry_width_m=float(profile["entry_width_m"]),
            corner_width_m=float(profile["corner_width_m"]),
            speed_rules=rules,
            confidence_sigma=float(policy["confidence_sigma"]),
            consecutive_estimates=int(policy["consecutive_estimates"]),
            path_axis_fraction=float(
                profile.get("delivery_trajectory", {}).get("approach_heading", (1.0, 0.0))[0]
            ),
        )
        marker_map.assert_policy_covers_the_corridor()
        return marker_map

    def assert_policy_covers_the_corridor(self) -> None:
        """Fail now if any reachable width has no rule, rather than mid-run.

        ``limit_at`` is called from ``_on_frame``, a subscription callback. A
        policy missing its catch-all built cleanly and then killed the observer
        partway through a demonstration, which is the worst possible time and
        place to discover a configuration error. Checking every width the
        corridor actually presents turns that into a node that refuses to start
        and says why.
        """

        widths = {self.entry_width_m, self.corner_width_m}
        widths.update(self.width_at(station) for station in self.gate_stations_m)
        widest_rule = self.speed_rules[-1][0] if self.speed_rules else 0.0
        # Same tolerance as `limit_at`, or this could refuse to start on a
        # width the runtime would happily have covered.
        uncovered = sorted(width for width in widths
                           if not covered_by(width, widest_rule))
        if uncovered:
            raise ValueError(
                f"speed policy does not cover corridor widths {uncovered} on profile "
                f"{self.profile_name!r}; the widest rule stops at {widest_rule} m"
            )

    def width_at(self, station_m: float) -> float:
        fraction = min(max(station_m / self.corridor_length_m, 0.0), 1.0)
        return self.entry_width_m + fraction * (self.corner_width_m - self.entry_width_m)

    def limit_at(self, station_m: float) -> float:
        width = self.width_at(station_m)
        for maximum_width, limit in self.speed_rules:
            if covered_by(width, maximum_width):
                return limit
        raise ValueError(f"speed policy does not cover corridor width {width}")


def _aruco_dictionary(name: str):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"OpenCV does not provide ArUco dictionary {name!r}")
    dictionary_id = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dictionary_id)
    return cv2.aruco.Dictionary_get(dictionary_id)


class ArucoStationEstimator:
    """Estimate camera station without access to robot or simulator pose."""

    def __init__(
        self,
        marker_map: MarkerMap,
        dictionary_name: str,
        maximum_reprojection_rmse_px: float = 3.0,
        minimum_markers: int = 2,
        minimum_correspondence_rank: int = 3,
    ) -> None:
        self.marker_map = marker_map
        self.dictionary = _aruco_dictionary(dictionary_name)
        self.maximum_reprojection_rmse_px = maximum_reprojection_rmse_px
        # A single square gives four coplanar correspondences, which planar PnP
        # can fit perfectly while recovering the wrong pose. Such a frame looks
        # excellent by reprojection error, so the residual filter cannot catch
        # it; requiring a second marker is what makes the solve well posed.
        self.minimum_markers = minimum_markers
        # Counting markers is not sufficient. Two markers lying on one plane —
        # for example both far-field plates on the same building face — are
        # exactly as ambiguous as one, and no marker count detects that. The
        # correspondence set itself must span three dimensions. Lowering this is
        # only for regressions that need to reproduce the ambiguity on purpose.
        self.minimum_correspondence_rank = minimum_correspondence_rank
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.parameters = cv2.aruco.DetectorParameters_create()
        else:
            self.parameters = cv2.aruco.DetectorParameters()
        self.detector = (
            cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
            if hasattr(cv2.aruco, "ArucoDetector")
            else None
        )

    def detect(self, image: np.ndarray) -> tuple[list[np.ndarray], np.ndarray | None]:
        if image.ndim == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        elif image.ndim == 2:
            gray = image
        else:
            raise ValueError("image must be mono8 or BGR8")
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.parameters
            )
        return corners, ids

    def estimate(
        self, image: np.ndarray, calibration: Calibration, timestamp_s: float
    ) -> PoseObservation | None:
        if not math.isfinite(timestamp_s) or timestamp_s <= 0.0:
            return None
        if image.shape[0] != calibration.height or image.shape[1] != calibration.width:
            raise ValueError("image dimensions do not match calibration")
        corners, identifiers = self.detect(image)
        if identifiers is None:
            return None
        object_points: list[np.ndarray] = []
        image_points: list[np.ndarray] = []
        used_ids: list[int] = []
        for detected_corners, identifier_array in zip(corners, identifiers, strict=True):
            marker_id = int(identifier_array[0])
            surveyed = self.marker_map.marker_corners.get(marker_id)
            if surveyed is None:
                continue
            pixels = np.asarray(detected_corners, dtype=np.float64).reshape(4, 2)
            if cv2.contourArea(pixels.astype(np.float32)) < 16.0:
                continue
            if np.any(pixels[:, 0] < 1.0) or np.any(pixels[:, 0] >= calibration.width - 1.0):
                continue
            if np.any(pixels[:, 1] < 1.0) or np.any(pixels[:, 1] >= calibration.height - 1.0):
                continue
            object_points.append(surveyed)
            image_points.append(pixels)
            used_ids.append(marker_id)
        if len(object_points) < self.minimum_markers:
            return None

        world = np.concatenate(object_points).astype(np.float64)
        pixels = np.concatenate(image_points).astype(np.float64)
        # Reject a rank-deficient correspondence set before solving. Two plates
        # on one building face are coplanar and reintroduce the planar-PnP
        # ambiguity that the marker-count rule was meant to remove, so the
        # geometry is checked directly rather than inferred from how many
        # markers were seen.
        if (
            np.linalg.matrix_rank(world - world.mean(axis=0), tol=1e-6)
            < self.minimum_correspondence_rank
        ):
            return None
        flag = cv2.SOLVEPNP_ITERATIVE if len(world) >= 6 else cv2.SOLVEPNP_IPPE
        success, rotation_vector, translation = cv2.solvePnP(
            world,
            pixels,
            calibration.matrix,
            calibration.distortion,
            flags=flag,
        )
        if not success:
            return None
        projected, _ = cv2.projectPoints(
            world,
            rotation_vector,
            translation,
            calibration.matrix,
            calibration.distortion,
        )
        residual = projected.reshape(-1, 2) - pixels
        rmse = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        if not math.isfinite(rmse) or rmse > self.maximum_reprojection_rmse_px:
            return None
        rotation, _ = cv2.Rodrigues(rotation_vector)
        camera_center = -rotation.T @ translation
        station = float(camera_center[0, 0])
        depths = (rotation @ world.T + translation).T[:, 2]
        if np.any(depths <= 0.0):
            return None
        median_depth = float(np.median(depths))
        focal = float(calibration.matrix[0, 0])
        station_sigma = max(0.005, rmse * max(median_depth, 1.0) / focal)
        return PoseObservation(
            timestamp_s=timestamp_s,
            station_m=station,
            station_stddev_m=station_sigma,
            reprojection_rmse_px=rmse,
            marker_ids=tuple(sorted(used_ids)),
        )


# A backward station step is evidence of a pose jump only when it is larger
# than the observations' own stated uncertainty. Strict monotonicity is not a
# satisfiable requirement here: at 0.6 m/s the camera advances 0.0397 m per
# frame against a p95 station error of 0.0414 m, so noise alone produces
# backward-looking pairs. Each one used to discard the gate history and
# silently drop the next speed measurement.
#
# Three sigma is the conventional significance threshold. Measured noise steps
# sit at or below 0.5 sigma of the combined uncertainty, while the ambiguous
# single-marker pose this guard was installed against sits at 3.4 sigma -- and
# is already rejected upstream on correspondence rank before it can reach here.
# The scaling also works in the right direction: an ambiguous planar fit
# reports a *low* reprojection residual, so its sigma is small and its ratio
# large.
STATION_REGRESSION_SIGMA = 3.0


class GateSpeedEstimator:
    """Interpolate surveyed gate crossings and differentiate their times."""

    def __init__(
        self,
        marker_map: MarkerMap,
        station_regression_sigma: float = STATION_REGRESSION_SIGMA,
    ) -> None:
        self.marker_map = marker_map
        self.station_regression_sigma = station_regression_sigma
        self.previous: PoseObservation | None = None
        self.last_crossing: tuple[int, float, float] | None = None
        self.observation_count = 0

    def reset(self) -> None:
        self.previous = None
        self.last_crossing = None
        self.observation_count = 0

    def is_discontinuous(self, observation: PoseObservation) -> bool:
        """Report whether this observation breaks temporal or spatial continuity.

        Exposed so a caller can mirror the reset onto state this class does not
        own. ``update`` resets itself internally, and that used to be invisible
        to the violation detector, which then carried an open episode across a
        clock jump and suppressed the next genuine offense.

        Time is judged strictly: a stalled or reversed stamp is always a break.
        Station is judged against the observations' own uncertainty, because a
        regression smaller than the estimator's stated noise is not evidence of
        anything. See ``STATION_REGRESSION_SIGMA``.
        """

        previous = self.previous
        if previous is None:
            return False
        if observation.timestamp_s <= previous.timestamp_s:
            return True
        regression_m = previous.station_m - observation.station_m
        if regression_m <= 0.0:
            return False
        tolerance_m = self.station_regression_sigma * math.hypot(
            previous.station_stddev_m, observation.station_stddev_m
        )
        return regression_m > tolerance_m

    def update(self, observation: PoseObservation) -> list[SpeedMeasurement]:
        previous = self.previous
        if self.is_discontinuous(observation):
            self.reset()
            previous = None
        self.previous = observation
        self.observation_count += 1
        if previous is None or observation.station_m <= previous.station_m:
            return []

        delta_station = observation.station_m - previous.station_m
        delta_time = observation.timestamp_s - previous.timestamp_s
        results: list[SpeedMeasurement] = []
        for gate_id, gate_station in enumerate(self.marker_map.gate_stations_m):
            if not (previous.station_m < gate_station <= observation.station_m):
                continue
            fraction = (gate_station - previous.station_m) / delta_station
            crossing_time = previous.timestamp_s + fraction * delta_time
            crossing_sigma = (
                1.0 - fraction
            ) * previous.station_stddev_m + fraction * observation.station_stddev_m
            if self.last_crossing is not None:
                from_id, from_time, from_sigma = self.last_crossing
                elapsed = crossing_time - from_time
                distance = gate_station - self.marker_map.gate_stations_m[from_id]
                if elapsed > 1e-6 and distance > 0.0:
                    # Convert the along-X gate spacing into distance actually
                    # travelled before differentiating, so a tapered corridor
                    # does not systematically under-report speed.
                    axis_fraction = self.marker_map.path_axis_fraction
                    speed = distance / elapsed / axis_fraction
                    speed_sigma = (
                        math.sqrt(from_sigma**2 + crossing_sigma**2) / elapsed / axis_fraction
                    )
                    results.append(
                        SpeedMeasurement(
                            timestamp_s=crossing_time,
                            station_m=gate_station,
                            speed_mps=speed,
                            speed_stddev_mps=speed_sigma,
                            corridor_width_m=self.marker_map.width_at(gate_station),
                            speed_limit_mps=self.marker_map.limit_at(gate_station),
                            gate_from_id=from_id,
                            gate_to_id=gate_id,
                            observation_count=self.observation_count,
                        )
                    )
            self.last_crossing = (gate_id, crossing_time, crossing_sigma)
        return results


def conservative_speed_mps(
    speed_mps: float, speed_stddev_mps: float, confidence_sigma: float
) -> float:
    """Return the confidence-discounted speed that compliance decisions use.

    Subtracting a confidence margin from the raw measurement is what makes a
    "this was over the limit" claim defensible under uncertainty; it is also
    what makes the reverse claim -- "this cleared the limit" -- defensible.
    Exactly one function computes it so a rearm/compliance decision made
    anywhere in the system uses the same number. RViz used to clear a
    displayed violation on the raw speed while ``ViolationDetector`` rearmed on
    this conservative one, which could leave the display green after the
    detector's own episode was still open, or vice versa near the margin (A6-M2).
    """

    return speed_mps - confidence_sigma * speed_stddev_mps


def is_conservatively_compliant(
    speed_mps: float, speed_stddev_mps: float, speed_limit_mps: float, confidence_sigma: float
) -> bool:
    """Return whether a measurement is compliant once confidence is discounted."""

    return conservative_speed_mps(speed_mps, speed_stddev_mps, confidence_sigma) <= speed_limit_mps


class ViolationDetector:
    """Emit one event per continuous speeding episode.

    An episode opens when the confirmation rule is first satisfied and stays
    open while the robot keeps exceeding the applicable limit, including across
    a transition into a stricter zone. Only a conservative measurement at or
    below the limit rearms the detector. See ADR 0014.

    The earlier implementation reset immediately after emitting, so a steady
    over-limit run produced a fresh event every ``consecutive_estimates``
    measurements. That made the event count a function of how many gates
    happened to be measurable rather than of the robot's behaviour.
    """

    def __init__(self, marker_map: MarkerMap) -> None:
        self.marker_map = marker_map
        self.consecutive = 0
        self.first_time_s: float | None = None
        self.event_id = 0
        self.episode_open = False

    def reset(self) -> None:
        """Clear all episode state.

        Called on temporal discontinuities, where continuity of an open episode
        can no longer be asserted, as well as on a compliant measurement.
        """

        self.consecutive = 0
        self.first_time_s = None
        self.episode_open = False

    def update(self, measurement: SpeedMeasurement) -> Violation | None:
        conservative_speed = conservative_speed_mps(
            measurement.speed_mps, measurement.speed_stddev_mps, self.marker_map.confidence_sigma
        )
        if conservative_speed <= measurement.speed_limit_mps:
            # Compliance is the only thing that rearms.
            self.reset()
            return None
        if self.episode_open:
            # Still the same episode, including after the limit tightened.
            return None
        if self.consecutive == 0:
            self.first_time_s = measurement.timestamp_s
        self.consecutive += 1
        if self.consecutive < self.marker_map.consecutive_estimates:
            return None
        self.event_id += 1
        first = self.first_time_s if self.first_time_s is not None else measurement.timestamp_s
        self.episode_open = True
        return Violation(
            event_id=self.event_id,
            estimate=measurement,
            exceedance_mps=conservative_speed - measurement.speed_limit_mps,
            confirmation_duration_s=max(0.0, measurement.timestamp_s - first),
        )


class ObserverPipeline:
    """Own the gate estimator and the violation detector as one unit.

    ``GateSpeedEstimator.update`` resets its own gate history when an
    observation breaks continuity. Nothing propagated that to the violation
    detector, so an episode opened before a clock jump stayed open across it and
    suppressed the next genuine offense indefinitely — the discontinuity also
    stops measurements flowing, so no compliant measurement ever arrived to
    rearm it.

    Routing both through one path is what keeps the two resets coupled. Callers
    should use this rather than driving the two objects separately.
    """

    def __init__(self, marker_map: MarkerMap) -> None:
        self.marker_map = marker_map
        self.speed_estimator = GateSpeedEstimator(marker_map)
        self.violation_detector = ViolationDetector(marker_map)

    def reset(self) -> None:
        """Clear both stages together."""

        self.speed_estimator.reset()
        self.violation_detector.reset()

    def update(
        self, observation: PoseObservation
    ) -> list[tuple[SpeedMeasurement, Violation | None]]:
        """Advance both stages, mirroring any continuity break onto the detector."""

        if self.speed_estimator.is_discontinuous(observation):
            self.violation_detector.reset()
        return [
            (measurement, self.violation_detector.update(measurement))
            for measurement in self.speed_estimator.update(observation)
        ]
