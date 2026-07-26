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
    camera, marker_map, pose = pipeline
    speed = GateSpeedEstimator(marker_map)
    violations = ViolationDetector(marker_map)
    measurements = []
    events = []
    frame = 0
    while True:
        elapsed = frame / camera.rate_hz
        station = truth_speed_mps * elapsed
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
