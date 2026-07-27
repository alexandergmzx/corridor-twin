"""Continuity and turn-fit tests for the shared delivery trajectory."""

from __future__ import annotations

import math

import pytest
from scene.geometry import (
    Occluder,
    a_start_xyz,
    corridor_centerline,
    is_clear,
    person_b_xyz,
    police_bounds,
)
from scene.model import load_scenario
from scene.occlusion import (
    _camera_source_vertices,
    _frustum_excluded,
    _target_vertices,
    _wall_witness,
    _wall_witness_sources,
    continuous_certificate,
)
from scene.trajectory import ARC, DeliveryTrajectory, delivery_trajectory


def _profiles():
    scenario = load_scenario()
    return [(scenario, profile) for profile in scenario.profiles]


@pytest.mark.parametrize("scenario,profile", _profiles())
def test_position_and_yaw_are_continuous(scenario, profile) -> None:
    trajectory = delivery_trajectory(scenario, profile)
    step = 1e-6
    for segment in trajectory.segments()[:-1]:
        before = trajectory.pose_at(segment.end_s_m - step)
        after = trajectory.pose_at(segment.end_s_m + step)
        assert math.hypot(before.x_m - after.x_m, before.y_m - after.y_m) < 1e-5
        assert abs(before.yaw_rad - after.yaw_rad) < 1e-5


@pytest.mark.parametrize("scenario,profile", _profiles())
def test_route_starts_at_a_and_ends_at_b(scenario, profile) -> None:
    trajectory = delivery_trajectory(scenario, profile)
    start = trajectory.pose_at(0.0)
    assert (start.x_m, start.y_m, start.z_m) == pytest.approx(a_start_xyz(scenario, profile))
    end = trajectory.pose_at(trajectory.length_m)
    assert (end.x_m, end.y_m) == pytest.approx(person_b_xyz(scenario)[:2], abs=1e-6)
    assert end.yaw_rad == pytest.approx(-math.pi / 2.0, abs=1e-9)


@pytest.mark.parametrize("scenario,profile", _profiles())
def test_approach_follows_the_tapered_centerline(scenario, profile) -> None:
    """Under a one-sided taper the centreline is not straight along y=0."""

    trajectory = delivery_trajectory(scenario, profile)
    length = scenario.corridor_length_m
    for station in (0.0, 3.0, 6.0, 9.0):
        travelled = station / trajectory.approach_heading[0]
        pose = trajectory.pose_at(travelled)
        assert pose.x_m == pytest.approx(station, abs=1e-9)
        assert pose.y_m == pytest.approx(corridor_centerline(profile, station, length), abs=1e-9)


@pytest.mark.parametrize("scenario,profile", _profiles())
def test_yaw_is_monotone_through_the_turn(scenario, profile) -> None:
    """yaw_range() depends on this, and the certificate depends on yaw_range()."""

    trajectory = delivery_trajectory(scenario, profile)
    samples = [
        trajectory.pose_at(trajectory.length_m * index / 400).yaw_rad for index in range(401)
    ]
    pairs = zip(samples, samples[1:], strict=False)
    assert all(later <= earlier + 1e-12 for earlier, later in pairs)
    low, high = trajectory.yaw_range(0.0, trajectory.length_m)
    assert low == pytest.approx(min(samples))
    assert high == pytest.approx(max(samples))


@pytest.mark.parametrize("scenario,profile", _profiles())
def test_turn_fits_inside_the_clear_junction(scenario, profile) -> None:
    trajectory = delivery_trajectory(scenario, profile)
    arc = next(segment for segment in trajectory.segments() if segment.kind == ARC)
    span = arc.end_s_m - arc.start_s_m
    assert span > 0.0
    for index in range(201):
        pose = trajectory.pose_at(arc.start_s_m + span * index / 200)
        assert is_clear(scenario, profile, pose.x_m, pose.y_m)
    assert trajectory.arc_radius_m == pytest.approx(scenario.next_street.turn_radius_m)


@pytest.mark.parametrize("scenario,profile", _profiles())
def test_mid_turn_yaw_is_covered_not_skipped(scenario, profile) -> None:
    """Regression guard for per-segment heading sampling.

    A polyline model would evaluate the corner at two headings only. If any
    intermediate yaw could expose P, sampling the endpoints would miss it, so
    assert that the interval bound used by the certificate is at least as
    conservative as a dense sweep of the real yaws.
    """

    trajectory = delivery_trajectory(scenario, profile)
    police_min, police_max = police_bounds(scenario, profile)
    targets = _target_vertices(police_min, police_max)
    tan_half = math.tan(math.radians(scenario.camera.horizontal_fov_deg) / 2.0)
    arc = next(segment for segment in trajectory.segments() if segment.kind == ARC)

    source_start = trajectory.camera_pose_at(arc.start_s_m)
    source_end = trajectory.camera_pose_at(arc.end_s_m)
    yaw_min, yaw_max = trajectory.yaw_range(arc.start_s_m, arc.end_s_m)
    interval_excluded = _frustum_excluded(
        (source_start.x_m, source_start.y_m, source_start.z_m),
        (source_end.x_m, source_end.y_m, source_end.z_m),
        targets,
        yaw_min,
        yaw_max,
        tan_half,
    )
    if not interval_excluded:
        return

    # If the interval bound claims exclusion, every real intermediate pose must
    # agree. A yaw range that under-covers the sweep would break this.
    span = arc.end_s_m - arc.start_s_m
    for index in range(101):
        pose = trajectory.camera_pose_at(arc.start_s_m + span * index / 100)
        point = (pose.x_m, pose.y_m, pose.z_m)
        assert _frustum_excluded(
            point, point, targets, pose.yaw_rad, pose.yaw_rad, tan_half
        )


@pytest.mark.parametrize("scenario,profile", _profiles())
def test_arc_source_enclosure_contains_the_real_turn(scenario, profile) -> None:
    trajectory = delivery_trajectory(scenario, profile)
    arc = next(segment for segment in trajectory.segments() if segment.kind == ARC)
    for chunk in range(4):
        start = arc.start_s_m + (arc.end_s_m - arc.start_s_m) * chunk / 4.0
        end = arc.start_s_m + (arc.end_s_m - arc.start_s_m) * (chunk + 1) / 4.0
        enclosure = _camera_source_vertices(trajectory, ARC, start, end)
        x_bounds = (min(point[0] for point in enclosure), max(point[0] for point in enclosure))
        y_bounds = (min(point[1] for point in enclosure), max(point[1] for point in enclosure))
        for index in range(21):
            pose = trajectory.camera_pose_at(start + (end - start) * index / 20.0)
            assert x_bounds[0] - 1e-12 <= pose.x_m <= x_bounds[1] + 1e-12
            assert y_bounds[0] - 1e-12 <= pose.y_m <= y_bounds[1] + 1e-12


def test_curved_source_interval_cannot_be_replaced_by_its_chord(monkeypatch) -> None:
    """Regression for a certificate false-pass found during review.

    The arc endpoints both see the target through a short wall, but the real
    mid-arc camera sits outside their chord and sees over it. A proof using only
    the endpoints accepts this geometry; the conservative arc enclosure must
    reject it.
    """

    trajectory = DeliveryTrajectory(
        start_xyz_m=(math.cos(1.7), math.sin(1.7), 0.0),
        approach_heading=(1.0, 0.0),
        approach_length_m=0.0,
        arc_center_xy_m=(0.0, 0.0),
        arc_radius_m=1.0,
        arc_start_angle_rad=1.7,
        arc_sweep_rad=1.7,
        departure_length_m=0.0,
        camera_height_m=1.0,
    )
    slab = Occluder("/wall", 1.5, 1.55, -0.01, 0.0, 0.22, 0.0, 2.0)
    target_min = (2.0, -0.001, 0.99)
    target_max = (2.01, 0.001, 1.01)
    targets = _target_vertices(target_min, target_max)
    start = trajectory.camera_pose_at(0.0)
    end = trajectory.camera_pose_at(trajectory.arc_length_m)
    endpoints = (
        (start.x_m, start.y_m, start.z_m),
        (end.x_m, end.y_m, end.z_m),
    )

    assert _wall_witness(*endpoints, targets, slab) is not None
    enclosure = _camera_source_vertices(trajectory, ARC, 0.0, trajectory.arc_length_m)
    midpoint = trajectory.camera_pose_at(trajectory.arc_length_m / 2.0)
    assert min(point[0] for point in enclosure) <= midpoint.x_m <= max(
        point[0] for point in enclosure
    )
    assert min(point[1] for point in enclosure) <= midpoint.y_m <= max(
        point[1] for point in enclosure
    )
    assert min(point[0] for point in enclosure) == pytest.approx(math.cos(1.7))
    assert max(point[0] for point in enclosure) == pytest.approx(1.0)
    assert min(point[1] for point in enclosure) == pytest.approx(0.0)
    assert max(point[1] for point in enclosure) == pytest.approx(1.0)
    assert _wall_witness_sources(enclosure, targets, slab) is None

    # Keep the deliberately broken fixture bounded while still exercising the
    # public recursive certificate path.
    monkeypatch.setattr("scene.occlusion.MAX_DEPTH", 8)
    certificate = continuous_certificate(
        trajectory,
        target_min,
        target_max,
        (slab,),
        horizontal_fov_deg=179.0,
        profile_name="curved-source-negative-control",
    )
    assert not certificate.passed
    assert not certificate.line_of_sight_blocked_everywhere


def test_turn_radius_that_cannot_fit_is_rejected(tmp_path) -> None:
    import dataclasses

    scenario = load_scenario()
    profile = scenario.profile(scenario.default_profile)
    oversized = dataclasses.replace(
        scenario,
        next_street=dataclasses.replace(scenario.next_street, turn_radius_m=40.0),
    )
    with pytest.raises(ValueError, match="too large to fit"):
        delivery_trajectory(oversized, profile)
