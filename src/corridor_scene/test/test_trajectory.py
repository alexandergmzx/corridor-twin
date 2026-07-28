"""Continuity and turn-fit tests for the shared delivery trajectory."""

from __future__ import annotations

import math
from dataclasses import asdict, replace

import pytest
from scene.geometry import (
    Occluder,
    a_start_xyz,
    corridor_centerline,
    east_wall_stub_bounds,
    is_clear,
    person_b_xyz,
    police_bounds,
    street_drive_center_x_m,
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
from scene.trajectory import (
    ARC,
    DeliveryTrajectory,
    delivery_trajectory,
    trajectory_from_manifest,
    validate_trajectory,
)


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
    # B stands in the pocket behind the east-wall stub, so A arrives heading
    # east into it rather than still running south down the lane (ADR 0018).
    assert end.yaw_rad == pytest.approx(0.0, abs=1e-9)


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
def test_manifest_route_round_trip_preserves_coordinate_semantics(scenario, profile) -> None:
    authored = delivery_trajectory(scenario, profile)
    restored = trajectory_from_manifest(asdict(authored))
    assert restored == authored

    route_s_m = 6.0
    pose = restored.pose_at(route_s_m)
    assert pose.x_m == pytest.approx(route_s_m * restored.approach_heading[0])
    assert restored.approach_s_at_x(pose.x_m) == pytest.approx(route_s_m)
    if restored.approach_heading[0] < 1.0:
        assert pose.x_m != pytest.approx(route_s_m)


def test_world_x_to_route_station_rejects_non_approach_values() -> None:
    scenario = load_scenario()
    trajectory = delivery_trajectory(scenario, scenario.profile(scenario.default_profile))
    with pytest.raises(ValueError, match="outside the approach"):
        trajectory.approach_s_at_x(100.0)


@pytest.mark.parametrize("scenario,profile", _profiles())
def test_yaw_range_is_exact_although_yaw_is_not_monotone(scenario, profile) -> None:
    """The certificate depends on yaw_range(), which used to depend on monotonicity.

    Yaw fell monotonically over the old line-arc-line route, so yaw_range could
    read the interval's two endpoints and be right. ADR 0018 added a left-handed
    delivery turn, so yaw now falls through the first arc and rises through the
    second: an interval spanning both has its extremes in the *interior*.
    Endpoint sampling would under-report the sweep and bound the camera over a
    narrower cone than it actually traverses, which is a silent false pass.

    This pins the property that replaced monotonicity: yaw_range is exact
    against a dense sample, over the whole route and over every sub-interval
    that spans the two turns.
    """

    trajectory = delivery_trajectory(scenario, profile)
    count = 2000
    samples = [
        trajectory.pose_at(trajectory.length_m * index / count).yaw_rad
        for index in range(count + 1)
    ]

    # Yaw genuinely is not monotone any more, so the old assumption is dead.
    assert not all(
        later <= earlier + 1e-12 for earlier, later in zip(samples, samples[1:], strict=False)
    )

    low, high = trajectory.yaw_range(0.0, trajectory.length_m)
    assert low == pytest.approx(min(samples), abs=1e-6)
    assert high == pytest.approx(max(samples), abs=1e-6)

    # And on sub-intervals, including ones whose extremes are interior.
    for start_frac, end_frac in ((0.0, 0.6), (0.4, 1.0), (0.45, 0.95), (0.5, 0.75)):
        start_s = trajectory.length_m * start_frac
        end_s = trajectory.length_m * end_frac
        window = [
            trajectory.pose_at(start_s + (end_s - start_s) * index / count).yaw_rad
            for index in range(count + 1)
        ]
        low, high = trajectory.yaw_range(start_s, end_s)
        window_name = f"[{start_frac}, {end_frac}]"
        assert low <= min(window) + 1e-6, f"yaw_range missed a minimum on {window_name}"
        assert high >= max(window) - 1e-6, f"yaw_range missed a maximum on {window_name}"


@pytest.mark.parametrize("scenario,profile", _profiles())
def test_yaw_is_monotone_within_each_piece(scenario, profile) -> None:
    """What replaced whole-route monotonicity, and what makes yaw_range exact."""

    trajectory = delivery_trajectory(scenario, profile)
    for segment in trajectory.segments():
        span = segment.end_s_m - segment.start_s_m
        if span <= 0.0:
            continue
        samples = [
            trajectory.pose_at(segment.start_s_m + span * index / 200).yaw_rad
            for index in range(201)
        ]
        deltas = [later - earlier for earlier, later in zip(samples, samples[1:], strict=False)]
        assert all(d <= 1e-12 for d in deltas) or all(d >= -1e-12 for d in deltas), (
            f"yaw is not monotone within {segment.kind}, so yaw_range cannot read its ends"
        )


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


def test_the_old_street_centreline_would_drive_through_the_stub() -> None:
    """The reroute is required, not cosmetic.

    Before ADR 0018 the lane line was the street's geometric centre at
    x = 15.0. The stub's west face lands at x = 15.218 and A's body is 0.45 m
    wide, so that centreline puts A into the wall. `validate_trajectory` applies
    a 0.3 m margin, which is what would have caught it -- this drives that
    directly rather than trusting the arithmetic.
    """

    scenario = load_scenario()
    profile = scenario.profile("nominal_m6_n3")
    stub_west, _, _, _ = east_wall_stub_bounds(scenario)
    # A's visual body is 0.45 m wide, so its half-width is 0.225 m. The stub's
    # west face at 15.218 is 0.218 m east of the old centreline, which is inside
    # that half-width: the robot itself would clip the wall, before the
    # trajectory margin is considered at all.
    half_width = 0.225
    overlap = (scenario.street_center_x_m + half_width) - stub_west
    assert overlap > 0.0, (
        f"the stub must actually foul the old centreline for this to mean anything; "
        f"stub west face {stub_west:.3f}, A's flank {scenario.street_center_x_m + half_width:.3f}"
    )

    # The old route: lane and B both on the street's geometric centre.
    honest = delivery_trajectory(scenario, profile)
    superseded = replace(
        honest,
        departure_length_m=honest.departure_length_m + honest.delivery_arc_length_m,
        delivery_arc_radius_m=0.0,
        delivery_arc_sweep_rad=0.0,
        delivery_length_m=0.0,
    )
    shifted = replace(
        superseded,
        arc_center_xy_m=(
            superseded.arc_center_xy_m[0]
            + (scenario.street_center_x_m - street_drive_center_x_m(scenario)),
            superseded.arc_center_xy_m[1],
        ),
    )
    with pytest.raises(ValueError, match="leaves drivable space"):
        validate_trajectory(scenario, profile, shifted)


def test_route_legs_are_pinned() -> None:
    """A silent route change would invalidate every recorded live figure.

    The live evidence is measured against this route's length and timing, so
    the legs are pinned rather than merely asserted positive. If a geometry
    change moves them, this fails and the evidence has to be re-recorded with
    it -- which is the coupling that was missing when the route last moved.
    """

    scenario = load_scenario()
    trajectory = delivery_trajectory(scenario, scenario.profile("nominal_m6_n3"))
    lengths = {
        segment.kind: segment.end_s_m - segment.start_s_m for segment in trajectory.segments()
    }
    assert lengths["approach"] == pytest.approx(11.449, abs=5e-3)
    assert lengths["arc"] == pytest.approx(3.390, abs=5e-3)
    assert lengths["departure"] == pytest.approx(5.436, abs=5e-3)
    assert lengths["delivery_arc"] == pytest.approx(3.142, abs=5e-3)
    assert lengths["delivery"] == pytest.approx(1.184, abs=5e-3)
    assert trajectory.length_m == pytest.approx(24.601, abs=5e-3)


def test_manifest_round_trip_preserves_all_five_pieces() -> None:
    """Isaac drives the route from the manifest, so the parse must be lossless."""

    scenario = load_scenario()
    profile = scenario.profile("nominal_m6_n3")
    original = delivery_trajectory(scenario, profile)
    restored = trajectory_from_manifest(asdict(original))
    assert restored == original
    assert [segment.kind for segment in restored.segments()] == [
        "approach",
        "arc",
        "departure",
        "delivery_arc",
        "delivery",
    ]
