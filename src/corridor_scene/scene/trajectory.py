"""Continuous delivery trajectory shared by the USD path and the visibility gate.

A polyline with one heading per segment is not sufficient evidence for the
"A cannot see P" requirement: an instantaneous heading change at a corner can
hide a visibility window that a real rotating camera would sweep through. This
module therefore models the route as a chain of lines and circular arcs and
exposes both position and yaw continuously, so the certificate can bound the
camera over whole intervals of each turn.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from .geometry import (
    a_start_xyz,
    corridor_centerline,
    is_clear,
    person_b_xyz,
    street_drive_center_x_m,
)
from .model import CorridorProfile, Scenario

Vec3 = tuple[float, float, float]

APPROACH = "approach"
ARC = "arc"
DEPARTURE = "departure"
DELIVERY_ARC = "delivery_arc"
DELIVERY = "delivery"


@dataclass(frozen=True)
class Pose:
    """A planar pose on the route."""

    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float

    @property
    def heading(self) -> tuple[float, float]:
        """Return the unit forward direction implied by the yaw."""

        return (math.cos(self.yaw_rad), math.sin(self.yaw_rad))


@dataclass(frozen=True)
class Segment:
    """One continuously differentiable piece of the route."""

    kind: str
    start_s_m: float
    end_s_m: float


@dataclass(frozen=True)
class DeliveryTrajectory:
    """Five-piece route from A's start to B, parameterised by arc length.

    Line, right-hand arc into the street, straight run down the drivable lane,
    left-hand arc in behind the east-wall stub, then a short delivery run east
    to B. B stands in the pocket the stub makes against the east wall (ADR
    0018), which a line-arc-line route cannot reach.

    **Yaw is not monotonic.** It is constant on the approach, decreases through
    the first arc, holds on the lane, *increases* through the second arc, and
    holds again on the delivery run. The earlier three-piece route was
    monotonically non-increasing and :meth:`yaw_range` exploited that by reading
    only the interval's endpoints. That shortcut is now wrong, so the method
    takes extremes piece by piece instead. The visibility certificate leans on
    this bound being correct, so it is exact rather than sampled.
    """

    start_xyz_m: Vec3
    approach_heading: tuple[float, float]
    approach_length_m: float
    arc_center_xy_m: tuple[float, float]
    arc_radius_m: float
    arc_start_angle_rad: float
    arc_sweep_rad: float
    departure_length_m: float
    camera_height_m: float
    # The turn in behind the stub, and the run east to B. Both are zero-length
    # for a route that reaches B without them, which keeps this dataclass able
    # to describe the earlier line-arc-line geometry.
    delivery_arc_center_xy_m: tuple[float, float] = (0.0, 0.0)
    delivery_arc_radius_m: float = 0.0
    delivery_arc_start_angle_rad: float = 0.0
    delivery_arc_sweep_rad: float = 0.0
    delivery_length_m: float = 0.0

    @property
    def length_m(self) -> float:
        """Total route length."""

        return (
            self.approach_length_m
            + self.arc_length_m
            + self.departure_length_m
            + self.delivery_arc_length_m
            + self.delivery_length_m
        )

    @property
    def arc_length_m(self) -> float:
        """Arc-length of the turn into the street."""

        return self.arc_radius_m * self.arc_sweep_rad

    @property
    def delivery_arc_length_m(self) -> float:
        """Arc-length of the turn in behind the stub."""

        return self.delivery_arc_radius_m * self.delivery_arc_sweep_rad

    def segments(self) -> tuple[Segment, ...]:
        """Return the route's pieces as arc-length intervals.

        Zero-length pieces are still reported, so a consumer iterating segments
        sees the same shape whether or not the delivery turn is present.
        """

        first = self.approach_length_m
        second = first + self.arc_length_m
        third = second + self.departure_length_m
        fourth = third + self.delivery_arc_length_m
        return (
            Segment(APPROACH, 0.0, first),
            Segment(ARC, first, second),
            Segment(DEPARTURE, second, third),
            Segment(DELIVERY_ARC, third, fourth),
            Segment(DELIVERY, fourth, self.length_m),
        )

    def pose_at(self, s_m: float) -> Pose:
        """Return the ground pose at arc length ``s_m``, clamped to the route."""

        s_m = min(max(s_m, 0.0), self.length_m)
        forward_x, forward_y = self.approach_heading
        approach_yaw = math.atan2(forward_y, forward_x)

        if s_m <= self.approach_length_m:
            return Pose(
                self.start_xyz_m[0] + forward_x * s_m,
                self.start_xyz_m[1] + forward_y * s_m,
                self.start_xyz_m[2],
                approach_yaw,
            )

        arc_s = s_m - self.approach_length_m
        if arc_s <= self.arc_length_m:
            # The turn is right-handed, so the polar angle decreases.
            angle = self.arc_start_angle_rad - arc_s / self.arc_radius_m
            return Pose(
                self.arc_center_xy_m[0] + self.arc_radius_m * math.cos(angle),
                self.arc_center_xy_m[1] + self.arc_radius_m * math.sin(angle),
                self.start_xyz_m[2],
                angle - math.pi / 2.0,
            )

        exit_angle = self.arc_start_angle_rad - self.arc_sweep_rad
        lane_x = self.arc_center_xy_m[0] + self.arc_radius_m * math.cos(exit_angle)
        lane_y = self.arc_center_xy_m[1] + self.arc_radius_m * math.sin(exit_angle)
        lane_yaw = exit_angle - math.pi / 2.0
        lane_s = arc_s - self.arc_length_m
        if lane_s <= self.departure_length_m:
            return Pose(
                lane_x + math.cos(lane_yaw) * lane_s,
                lane_y + math.sin(lane_yaw) * lane_s,
                self.start_xyz_m[2],
                lane_yaw,
            )

        # The turn in behind the stub is left-handed, so this polar angle
        # increases where the first one decreased.
        delivery_s = lane_s - self.departure_length_m
        if delivery_s <= self.delivery_arc_length_m and self.delivery_arc_radius_m > 0.0:
            angle = self.delivery_arc_start_angle_rad + delivery_s / self.delivery_arc_radius_m
            return Pose(
                self.delivery_arc_center_xy_m[0] + self.delivery_arc_radius_m * math.cos(angle),
                self.delivery_arc_center_xy_m[1] + self.delivery_arc_radius_m * math.sin(angle),
                self.start_xyz_m[2],
                lane_yaw + delivery_s / self.delivery_arc_radius_m,
            )

        exit2 = self.delivery_arc_start_angle_rad + self.delivery_arc_sweep_rad
        run_x = self.delivery_arc_center_xy_m[0] + self.delivery_arc_radius_m * math.cos(exit2)
        run_y = self.delivery_arc_center_xy_m[1] + self.delivery_arc_radius_m * math.sin(exit2)
        run_yaw = lane_yaw + self.delivery_arc_sweep_rad
        remaining = delivery_s - self.delivery_arc_length_m
        return Pose(
            run_x + math.cos(run_yaw) * remaining,
            run_y + math.sin(run_yaw) * remaining,
            self.start_xyz_m[2],
            run_yaw,
        )

    def camera_pose_at(self, s_m: float) -> Pose:
        """Return the pose of the camera optical centre at arc length ``s_m``."""

        pose = self.pose_at(s_m)
        return Pose(pose.x_m, pose.y_m, pose.z_m + self.camera_height_m, pose.yaw_rad)

    def approach_s_at_x(self, station_x_m: float) -> float:
        """Convert a surveyed world-X station to approach arc length.

        The observer's station coordinate is world X, whereas this trajectory
        is parameterized by traveled distance. Keeping the conversion here
        prevents static and motion tools from quietly treating them as equal.
        """

        forward_x = self.approach_heading[0]
        if abs(forward_x) <= 1e-12:
            raise ValueError("approach does not advance along world X")
        route_s_m = (station_x_m - self.start_xyz_m[0]) / forward_x
        if route_s_m < -1e-9 or route_s_m > self.approach_length_m + 1e-9:
            raise ValueError(f"world-X station {station_x_m} is outside the approach")
        return min(max(route_s_m, 0.0), self.approach_length_m)

    def yaw_range(self, start_s_m: float, end_s_m: float) -> tuple[float, float]:
        """Return the (minimum, maximum) yaw over an arc-length interval.

        Exact, and deliberately not a two-endpoint read. Yaw used to be
        monotonically non-increasing over the whole route, so the extremes were
        simply the interval's ends. The delivery turn is left-handed and yaw
        rises through it, so an interval spanning both arcs has its extremes in
        the interior and endpoint sampling would under-report the sweep — the
        visibility certificate would then bound the camera over a narrower cone
        than it actually traverses, which is a silent false pass.

        Yaw is monotonic *within* each piece, so taking each piece's clipped
        ends and combining is exact without sampling.
        """

        low, high = sorted((start_s_m, end_s_m))
        low = min(max(low, 0.0), self.length_m)
        high = min(max(high, 0.0), self.length_m)

        yaws = [self.pose_at(low).yaw_rad, self.pose_at(high).yaw_rad]
        for segment in self.segments():
            clipped_start = max(segment.start_s_m, low)
            clipped_end = min(segment.end_s_m, high)
            if clipped_start > clipped_end:
                continue
            yaws.append(self.pose_at(clipped_start).yaw_rad)
            yaws.append(self.pose_at(clipped_end).yaw_rad)
        return (min(yaws), max(yaws))

    def polyline(self, samples_per_segment: int = 24) -> tuple[Vec3, ...]:
        """Sample the route as a ground polyline for USD inspection."""

        points: list[Vec3] = []
        for segment in self.segments():
            span = segment.end_s_m - segment.start_s_m
            if span <= 0.0:
                continue
            count = 1 if segment.kind not in {ARC, DELIVERY_ARC} else max(samples_per_segment, 2)
            for index in range(count + 1):
                pose = self.pose_at(segment.start_s_m + span * index / count)
                point = (pose.x_m, pose.y_m, pose.z_m)
                if not points or point != points[-1]:
                    points.append(point)
        return tuple(points)


def trajectory_from_manifest(data: dict[str, Any]) -> DeliveryTrajectory:
    """Rebuild the authored route from one manifest trajectory block.

    This parser is intentionally independent of USD and ROS so installed Isaac
    tools can command the exact route that the author and visibility gate use.
    """

    return DeliveryTrajectory(
        start_xyz_m=tuple(float(value) for value in data["start_xyz_m"]),  # type: ignore[arg-type]
        approach_heading=tuple(  # type: ignore[arg-type]
            float(value) for value in data["approach_heading"]
        ),
        approach_length_m=float(data["approach_length_m"]),
        arc_center_xy_m=tuple(  # type: ignore[arg-type]
            float(value) for value in data["arc_center_xy_m"]
        ),
        arc_radius_m=float(data["arc_radius_m"]),
        arc_start_angle_rad=float(data["arc_start_angle_rad"]),
        arc_sweep_rad=float(data["arc_sweep_rad"]),
        departure_length_m=float(data["departure_length_m"]),
        camera_height_m=float(data["camera_height_m"]),
        delivery_arc_center_xy_m=tuple(  # type: ignore[arg-type]
            float(value) for value in data.get("delivery_arc_center_xy_m", (0.0, 0.0))
        ),
        delivery_arc_radius_m=float(data.get("delivery_arc_radius_m", 0.0)),
        delivery_arc_start_angle_rad=float(data.get("delivery_arc_start_angle_rad", 0.0)),
        delivery_arc_sweep_rad=float(data.get("delivery_arc_sweep_rad", 0.0)),
        delivery_length_m=float(data.get("delivery_length_m", 0.0)),
    )


def delivery_trajectory(scenario: Scenario, profile: CorridorProfile) -> DeliveryTrajectory:
    """Build the route from the shared corridor geometry.

    The approach follows the corridor centreline, which under a one-sided taper
    drifts toward the fixed north face. The arc is tangent to that centreline
    and to the next street's centreline, so heading is continuous at both joins.
    """

    length = scenario.corridor_length_m
    radius = scenario.next_street.turn_radius_m
    start = a_start_xyz(scenario, profile)

    # The centreline is linear in x, so one sample gives its slope exactly.
    rise = corridor_centerline(profile, length, length) - corridor_centerline(profile, 0.0, length)
    norm = math.hypot(length, rise)
    heading = (length / norm, rise / norm)

    # Place the arc centre one radius to the right of both tangent lines. The
    # lane line is x = the drivable centreline, so the centre's x follows
    # directly. That is the centre of what the stub leaves clear, not the
    # street's geometric middle -- see ADR 0018.
    right_normal = (heading[1], -heading[0])
    lane_center_x = street_drive_center_x_m(scenario)
    center_x = lane_center_x - radius
    approach_length = (center_x - start[0] - radius * right_normal[0]) / heading[0]
    if approach_length <= 0.0:
        raise ValueError(
            f"turn radius {radius} m is too large to fit before the corner on profile "
            f"{profile.name}"
        )
    tangent_x = start[0] + heading[0] * approach_length
    tangent_y = start[1] + heading[1] * approach_length
    center_y = tangent_y + radius * right_normal[1]

    start_angle = math.atan2(tangent_y - center_y, tangent_x - center_x)
    # The departure tangent point is due east of the centre, so it is angle 0.
    sweep = start_angle
    if sweep <= 0.0:
        raise ValueError(f"profile {profile.name}: the corner turn does not sweep forward")

    # B stands in the pocket the east-wall stub makes, so the lane run stops one
    # radius short of B's line and a left-hand quarter turn brings A round to
    # face east. The second arc is tangent to the lane at its entry and to B's
    # line at its exit, so heading stays continuous at both joins exactly as it
    # does at the first turn.
    body_x, body_y, _ = person_b_xyz(scenario)
    delivery_center = (lane_center_x + radius, body_y + radius)
    departure_length = center_y - delivery_center[1]
    if departure_length <= 0.0:
        raise ValueError(
            f"profile {profile.name}: B is not far enough down the street for the delivery turn"
        )

    delivery_length = body_x - delivery_center[0]
    if delivery_length < 0.0:
        raise ValueError(
            f"profile {profile.name}: B is west of the delivery turn's exit, so the route "
            "would have to reverse to reach it"
        )

    return DeliveryTrajectory(
        start_xyz_m=start,
        approach_heading=heading,
        approach_length_m=approach_length,
        arc_center_xy_m=(center_x, center_y),
        arc_radius_m=radius,
        arc_start_angle_rad=start_angle,
        arc_sweep_rad=sweep,
        departure_length_m=departure_length,
        camera_height_m=scenario.camera.mount_height_m,
        delivery_arc_center_xy_m=delivery_center,
        delivery_arc_radius_m=radius,
        # Entry is due west of the centre; the left-hand quarter turn sweeps
        # forward to the exit due south of it.
        delivery_arc_start_angle_rad=math.pi,
        delivery_arc_sweep_rad=math.pi / 2.0,
        delivery_length_m=delivery_length,
    )


def validate_trajectory(
    scenario: Scenario,
    profile: CorridorProfile,
    trajectory: DeliveryTrajectory,
    margin_m: float = 0.3,
    samples: int = 400,
) -> None:
    """Reject a route that leaves drivable space.

    The margin is applied perpendicular to the heading, standing in for a
    vehicle half-width. The turn is the case that matters: a radius that does
    not fit would put the arc inside the corner mass or through a wall.
    """

    for index in range(samples + 1):
        pose = trajectory.pose_at(trajectory.length_m * index / samples)
        forward_x, forward_y = pose.heading
        lateral = (forward_y * margin_m, -forward_x * margin_m)
        for sign in (1.0, -1.0):
            probe_x = pose.x_m + sign * lateral[0]
            probe_y = pose.y_m + sign * lateral[1]
            if not is_clear(scenario, profile, probe_x, probe_y):
                raise ValueError(
                    f"profile {profile.name}: the delivery route leaves drivable space near "
                    f"({pose.x_m:.3f}, {pose.y_m:.3f})"
                )
