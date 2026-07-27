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

    The production estimator now rejects this on rank rather than marker count,
    which is strictly stronger. The permissive control has to opt out of both
    rules to reproduce the historical failure at all.
    """

    camera, marker_map, _ = pipeline
    permissive = ArucoStationEstimator(
        marker_map,
        camera.dictionary_name,
        minimum_markers=1,
        minimum_correspondence_rank=2,
    )
    strict = ArucoStationEstimator(marker_map, camera.dictionary_name)
    assert strict.minimum_markers >= 2
    assert strict.minimum_correspondence_rank == 3

    station = 5.5333
    rendered = camera.render(station)
    corners, identifiers = strict.detect(rendered)
    assert identifiers is not None

    # The production-sized plates deliberately make adjacent tags readable at
    # this station. Build the impairment under test by retaining only tag 9
    # and a quiet-zone margin from the otherwise unchanged synthetic frame.
    # This keeps the regression independent of future target-size changes.
    marker_index = next(
        index for index, identifier in enumerate(identifiers.reshape(-1)) if int(identifier) == 9
    )
    marker_corners = corners[marker_index].reshape(4, 2)
    margin_px = 8
    x_min = max(int(marker_corners[:, 0].min()) - margin_px, 0)
    x_max = min(int(marker_corners[:, 0].max()) + margin_px + 1, rendered.shape[1])
    y_min = max(int(marker_corners[:, 1].min()) - margin_px, 0)
    y_max = min(int(marker_corners[:, 1].max()) + margin_px + 1, rendered.shape[0])
    image = rendered.copy()
    image[:] = 210
    image[y_min:y_max, x_min:x_max] = rendered[y_min:y_max, x_min:x_max]

    _, isolated_identifiers = strict.detect(image)
    assert isolated_identifiers is not None
    assert isolated_identifiers.reshape(-1).tolist() == [9]

    loose = permissive.estimate(image, camera.calibration, timestamp_s=1.0)
    assert loose is not None
    reference = strict.estimate(rendered, camera.calibration, timestamp_s=1.0)
    assert reference is not None and len(reference.marker_ids) >= 2

    # State the trap directly instead of pinning incidental pixel values that
    # move whenever the fiducial geometry or calibration changes: the isolated
    # tag reports a *better* residual than the multi-tag fit while being *less*
    # accurate. Residual is anti-correlated with accuracy for a single planar
    # square, which is exactly why the rmse filter cannot screen these frames.
    assert loose.reprojection_rmse_px < reference.reprojection_rmse_px
    assert abs(loose.station_m - station) > abs(reference.station_m - station)
    # It is also wrong by more than the production render gate would accept.
    assert abs(loose.station_m - station) > 0.05
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


def _drive(pipeline_parts, truth_mps: float, until_x_m: float = 10.8):
    """Run the full pixel-to-violation stack at a constant true path speed."""

    from police_observer.estimator import ObserverPipeline

    camera, marker_map, _ = pipeline_parts
    pose = ArucoStationEstimator(marker_map, camera.dictionary_name)
    pipeline = ObserverPipeline(marker_map)
    measurements, events = [], []
    frame = 0
    while True:
        elapsed = frame / camera.rate_hz
        station_x_m = truth_mps * elapsed * marker_map.path_axis_fraction
        if station_x_m > until_x_m:
            break
        observation = pose.estimate(
            camera.render(station_x_m), camera.calibration, timestamp_s=1.0 + elapsed
        )
        if observation is not None:
            for measurement, violation in pipeline.update(observation):
                measurements.append(measurement)
                if violation is not None:
                    events.append((violation, measurement))
        frame += 1
    return measurements, events


def test_every_enforcement_gate_is_measured(pipeline) -> None:
    """Gates 8 and 10 were unreachable before the reference fiducials."""

    measurements, _ = _drive(pipeline, 1.0)
    assert [m.station_m for m in measurements] == [4.0, 6.0, 8.0, 10.0]
    assert any(m.speed_limit_mps == 0.8 for m in measurements)


def test_corner_speeding_alone_produces_one_violation(pipeline) -> None:
    """1.0 m/s is legal on the wide approach and illegal past the narrowing.

    Two gates now sit inside the 0.8 m/s zone, so the conservative
    two-estimate confirmation can actually be satisfied there. With only one
    such gate the corner rule could be evaluated but never confirmed.
    """

    measurements, events = _drive(pipeline, 1.0)
    assert len(events) == 1
    violation, measurement = events[0]
    assert measurement.speed_limit_mps == 0.8
    assert measurement.station_m == 10.0
    assert violation.exceedance_mps > 0.0
    # The wide approach was compliant, so nothing fired before the corner.
    assert all(m.speed_mps <= m.speed_limit_mps for m in measurements if m.station_m <= 6.0)


def test_compliant_run_produces_no_violation(pipeline) -> None:
    measurements, events = _drive(pipeline, 0.6)
    assert measurements
    assert events == []
    assert all(m.speed_mps < m.speed_limit_mps for m in measurements)


def test_sustained_speeding_stays_one_continuous_episode(pipeline) -> None:
    """Crossing into the stricter corner zone must not open a second offense."""

    measurements, events = _drive(pipeline, 1.8)
    assert len(events) == 1
    violation, measurement = events[0]
    # The episode opens on the wide approach, before the corner rule applies.
    assert measurement.speed_limit_mps == 1.2
    assert violation.event_id == 1
    assert any(m.speed_limit_mps == 0.8 for m in measurements)
