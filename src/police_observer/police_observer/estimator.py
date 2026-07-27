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
        rules = tuple(
            (float(rule["maximum_width_m"]), float(rule["limit_mps"])) for rule in policy["rules"]
        )
        return cls(
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

    def width_at(self, station_m: float) -> float:
        fraction = min(max(station_m / self.corridor_length_m, 0.0), 1.0)
        return self.entry_width_m + fraction * (self.corner_width_m - self.entry_width_m)

    def limit_at(self, station_m: float) -> float:
        width = self.width_at(station_m)
        for maximum_width, limit in self.speed_rules:
            if width <= maximum_width:
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
    ) -> None:
        self.marker_map = marker_map
        self.dictionary = _aruco_dictionary(dictionary_name)
        self.maximum_reprojection_rmse_px = maximum_reprojection_rmse_px
        # A single square gives four coplanar correspondences, which planar PnP
        # can fit perfectly while recovering the wrong pose. Such a frame looks
        # excellent by reprojection error, so the residual filter cannot catch
        # it; requiring a second marker is what makes the solve well posed.
        self.minimum_markers = minimum_markers
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


class GateSpeedEstimator:
    """Interpolate surveyed gate crossings and differentiate their times."""

    def __init__(self, marker_map: MarkerMap) -> None:
        self.marker_map = marker_map
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
        """

        previous = self.previous
        if previous is None:
            return False
        return (
            observation.timestamp_s <= previous.timestamp_s
            or observation.station_m < previous.station_m
        )

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
        conservative_speed = (
            measurement.speed_mps - self.marker_map.confidence_sigma * measurement.speed_stddev_mps
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
