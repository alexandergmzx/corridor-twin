from __future__ import annotations

from pathlib import Path

import pytest
from police_observer.estimator import (
    ArucoStationEstimator,
    GateSpeedEstimator,
    MarkerMap,
    PoseObservation,
    ViolationDetector,
)
from police_observer.synthetic import SyntheticCamera
from scene.build import build_scene


@pytest.fixture(scope="module")
def pipeline(tmp_path_factory: pytest.TempPathFactory):
    output = tmp_path_factory.mktemp("camera_pipeline") / "corridor.usda"
    _, manifest = build_scene(None, output, 6.0, 3.0)
    marker_map = MarkerMap.from_manifest(manifest)
    camera = SyntheticCamera(manifest)
    pose = ArucoStationEstimator(marker_map, camera.dictionary_name)
    return camera, marker_map, pose


def _run_constant_speed(pipeline, truth_speed_mps: float):
    """Drive the camera along the authored path at a known true speed.

    The camera is addressed by corridor station, which is world X, while the
    delivery path runs at a small angle to X under a one-sided taper. Scaling
    by that fraction makes ``truth_speed_mps`` the distance actually travelled
    per second, which is the quantity the speed policy is written about.
    """

    camera, marker_map, pose = pipeline
    speed = GateSpeedEstimator(marker_map)
    violations = ViolationDetector(marker_map)
    measurements = []
    events = []
    frame = 0
    while True:
        elapsed = frame / camera.rate_hz
        station = truth_speed_mps * elapsed * marker_map.path_axis_fraction
        if station > 7.2:
            break
        observation = pose.estimate(
            camera.render(station), camera.calibration, timestamp_s=1.0 + elapsed
        )
        if observation is not None:
            for measurement in speed.update(observation):
                measurements.append(measurement)
                event = violations.update(measurement)
                if event is not None:
                    events.append(event)
        frame += 1
    return measurements, events


def test_camera_only_estimator_accuracy_and_violation(pipeline) -> None:
    measurements, events = _run_constant_speed(pipeline, 1.8)
    assert len(measurements) >= 2
    assert max(abs(value.speed_mps - 1.8) for value in measurements) < 0.02
    assert all(value.speed_stddev_mps > 0.0 for value in measurements)
    assert len(events) == 1
    assert events[0].exceedance_mps > 0.0


def test_sub_limit_sequence_does_not_emit_violation(pipeline) -> None:
    measurements, events = _run_constant_speed(pipeline, 1.0)
    assert len(measurements) >= 2
    assert max(abs(value.speed_mps - 1.0) for value in measurements) < 0.02
    assert events == []


def test_single_marker_frames_are_rejected(pipeline) -> None:
    """Guard against planar PnP's pose ambiguity.

    Four coplanar points from one square can be fitted almost exactly while the
    recovered pose is wrong, so a low reprojection error is not evidence here.
    Such a frame produced a 0.21 m backward station jump, which then reset the
    gate history and silently dropped a speed measurement.
    """

    camera, marker_map, _ = pipeline
    permissive = ArucoStationEstimator(marker_map, camera.dictionary_name, minimum_markers=1)
    strict = ArucoStationEstimator(marker_map, camera.dictionary_name)
    assert strict.minimum_markers >= 2

    station = 5.5333
    image = camera.render(station)
    corners, identifiers = strict.detect(image)
    assert identifiers is not None and len(identifiers) == 1

    loose = permissive.estimate(image, camera.calibration, timestamp_s=1.0)
    assert loose is not None
    assert loose.reprojection_rmse_px < 0.1  # looks excellent, but is not
    assert abs(loose.station_m - station) > 0.1
    assert strict.estimate(image, camera.calibration, timestamp_s=1.0) is None


def test_measured_speed_is_path_speed_not_axis_speed(pipeline) -> None:
    """A tapered corridor must not systematically under-report speed."""

    _, marker_map, _ = pipeline
    assert marker_map.path_axis_fraction < 1.0
    measurements, _ = _run_constant_speed(pipeline, 1.8)
    assert measurements
    # Without the path correction every estimate would read low by this factor.
    axis_only = [value.speed_mps * marker_map.path_axis_fraction for value in measurements]
    assert max(abs(value.speed_mps - 1.8) for value in measurements) < 0.02
    assert min(abs(value - 1.8) for value in axis_only) > 0.01


def test_timestamp_and_backward_station_reset_history(pipeline) -> None:
    _, marker_map, _ = pipeline
    estimator = GateSpeedEstimator(marker_map)
    valid = PoseObservation(1.0, 1.0, 0.01, 0.2, (0, 1))
    assert estimator.update(valid) == []
    assert estimator.observation_count == 1
    assert estimator.update(PoseObservation(0.5, 1.5, 0.01, 0.2, (0, 1))) == []
    assert estimator.observation_count == 1
    assert estimator.update(PoseObservation(2.0, 0.5, 0.01, 0.2, (0, 1))) == []
    assert estimator.observation_count == 1


def test_observer_adapter_contains_no_truth_subscription() -> None:
    source = (Path(__file__).parents[1] / "police_observer" / "node.py").read_text(encoding="utf-8")
    assert "ground_truth" not in source
    assert "Odometry" not in source
