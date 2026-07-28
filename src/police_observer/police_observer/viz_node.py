"""Render the enforcement picture for RViz from the observer's own output.

This node is a *consumer*. It subscribes to nothing but the two topics the
police observer publishes, and it reads the surveyed manifest that P
legitimately holds. It never subscribes to pose, odometry, TF, the configured
speed, or any other simulator truth, and it is not a sensor: no image is
produced and no render product is created.

That constraint is also what makes the picture worth showing. A's marker on the
plan view is placed at the station the observer *measured from pixels*, not at
where the simulator put the robot. An interviewer watching the dot track the
corridor is watching camera-derived localisation, which is the claim being
demonstrated. Drawing simulator truth here would make the display prettier and
the demonstration meaningless.

The scene is drawn from the manifest's own wall set, so what a viewer sees is
what the street contains, not a second hand-drawn approximation of it. The walls
the occlusion certificate uses as witnesses are emphasised, so the concealment
story is still legible -- but a wall the proof never references is still drawn,
because it is still there. See ADR 0018.
"""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from corridor_interfaces.msg import SpeedEstimate, SpeedViolation

# Static geometry is latched so RViz shows the scene even when it connects long
# after the node started, which it always does when a human opens it mid-run.
LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

COMPLIANT = ColorRGBA(r=0.25, g=0.85, b=0.35, a=1.0)
VIOLATING = ColorRGBA(r=0.95, g=0.20, b=0.20, a=1.0)
WALL = ColorRGBA(r=0.55, g=0.57, b=0.62, a=1.0)
# Opaque, but never between A and P: drawn so the scene is legible without
# implying the certificate uses it as a witness.
SCENERY = ColorRGBA(r=0.38, g=0.40, b=0.44, a=1.0)
ROUTE = ColorRGBA(r=0.30, g=0.60, b=1.00, a=0.9)
GATE = ColorRGBA(r=1.00, g=0.80, b=0.10, a=1.0)
REFERENCE = ColorRGBA(r=0.70, g=0.45, b=0.95, a=1.0)
POLICE = ColorRGBA(r=1.00, g=0.35, b=0.10, a=1.0)
PERSON = ColorRGBA(r=0.20, g=0.90, b=0.85, a=1.0)


def _point(x: float, y: float, z: float) -> Point:
    return Point(x=float(x), y=float(y), z=float(z))


class EnforcementViewNode(Node):
    """Publish one MarkerArray describing what P can measure and what it cannot see."""

    def __init__(self) -> None:
        super().__init__("enforcement_view")
        self.declare_parameter("manifest_path", "")
        self.declare_parameter("corridor_profile", "")
        self.declare_parameter("estimate_topic", "police/speed_estimate")
        self.declare_parameter("violation_topic", "police/speed_violation")
        self.declare_parameter("marker_topic", "police/enforcement_view")
        self.declare_parameter("frame_id", "world")

        manifest_value = str(self.get_parameter("manifest_path").value)
        if not manifest_value:
            raise ValueError("manifest_path is required")
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.manifest = json.loads(Path(manifest_value).read_text(encoding="utf-8"))
        requested = str(self.get_parameter("corridor_profile").value)
        self.profile_name = requested or str(self.manifest["selected_profile"])
        if self.profile_name not in self.manifest["profiles"]:
            raise ValueError(f"manifest has no profile {self.profile_name!r}")
        self.profile = self.manifest["profiles"][self.profile_name]

        # Reuse the authored trajectory rather than re-deriving the route here;
        # a second geometry model is exactly what drifts out of agreement.
        from scene.trajectory import trajectory_from_manifest

        self.trajectory = trajectory_from_manifest(self.profile["delivery_trajectory"])

        self.latest_estimate: SpeedEstimate | None = None
        self.latest_violation: SpeedViolation | None = None
        self.violation_count = 0

        self.markers = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), LATCHED_QOS
        )
        self.create_subscription(
            SpeedEstimate,
            str(self.get_parameter("estimate_topic").value),
            self._on_estimate,
            10,
        )
        self.create_subscription(
            SpeedViolation,
            str(self.get_parameter("violation_topic").value),
            self._on_violation,
            10,
        )
        # Republish on a timer as well as on message, so the scene is present
        # before the first estimate arrives and survives an RViz restart.
        self.create_timer(0.5, self._publish)
        self._publish()

    def _on_estimate(self, message: SpeedEstimate) -> None:
        self.latest_estimate = message
        # A compliant measurement closes the displayed episode, matching the
        # detector's own rearm rule rather than inventing a second one.
        if message.valid and message.speed_mps <= message.speed_limit_mps:
            self.latest_violation = None
        self._publish()

    def _on_violation(self, message: SpeedViolation) -> None:
        self.latest_violation = message
        self.violation_count = max(self.violation_count, int(message.event_id))
        self._publish()

    # ------------------------------------------------------------------ markers

    def _marker(self, namespace: str, identifier: int, kind: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = identifier
        marker.type = kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _wall_names_that_hide_p(self) -> set[str]:
        """Return the walls the occlusion proof actually reasons about."""

        return {
            str(slab["prim_path"]).rsplit("/", maxsplit=1)[-1]
            for slab in self.profile["occluders"]
        }

    def _occluder_markers(self) -> list[Marker]:
        """Outline every wall, distinguishing the ones that hide P.

        This used to draw the manifest's occluder list, on the argument that
        what a viewer sees hiding P should be what the proof is about. That
        argument holds for P's concealment and stopped being sufficient the
        moment a wall existed that the proof does not reference: the east-wall
        stub is solid, A turns in behind it to reach B, and it was invisible
        here because it can never lie between A and P. A viewer watching the
        robot drive around a wall that is not drawn is being misled by a display
        that was only ever a map of the proof.

        The scene is now drawn from the manifest's `walls`, which is every
        opaque footprint. Walls the certificate uses as witnesses keep the
        emphasis, so the concealment story survives without the display
        pretending the other walls are absent.
        """

        blocking = self._wall_names_that_hide_p()
        markers = []
        for index, (name, footprint) in enumerate(sorted(self.profile["walls"].items())):
            marker = self._marker("occluders", index, Marker.LINE_STRIP)
            hides_p = name in blocking
            marker.scale.x = 0.08 if hides_p else 0.05
            marker.color = WALL if hides_p else SCENERY
            marker.points = [_point(x, y, 0.02) for x, y in footprint]
            if marker.points:
                marker.points.append(marker.points[0])
            markers.append(marker)
        return markers

    def _corridor_markers(self) -> list[Marker]:
        """The north face and the next street, so the corridor reads as a corridor."""

        north = float(self.profile["entry_width_m"]) / 2.0
        street = self.manifest["next_street"]
        west = -float(self.manifest["west_margin_m"])
        marker = self._marker("corridor", 0, Marker.LINE_LIST)
        marker.scale.x = 0.06
        marker.color = WALL
        segments = [
            # Straight north face, running past the corner to cap the street.
            ((west, north), (float(street["east_x_m"]), north)),
            # Next street's east face and its far end.
            (
                (float(street["east_x_m"]), north),
                (float(street["east_x_m"]), float(street["south_y_m"])),
            ),
        ]
        for start, end in segments:
            marker.points.append(_point(start[0], start[1], 0.02))
            marker.points.append(_point(end[0], end[1], 0.02))
        return [marker]

    def _route_marker(self) -> Marker:
        marker = self._marker("route", 0, Marker.LINE_STRIP)
        marker.scale.x = 0.05
        marker.color = ROUTE
        marker.points = [_point(x, y, 0.03) for x, y, _ in self.trajectory.polyline()]
        return marker

    def _fiducial_markers(self) -> list[Marker]:
        """Show the surveyed plates, split by the role they actually play."""

        markers = []
        for role, color in (("gate", GATE), ("reference", REFERENCE)):
            marker = self._marker("fiducials", 0 if role == "gate" else 1, Marker.CUBE_LIST)
            marker.scale.x = marker.scale.y = marker.scale.z = 0.22
            marker.color = color
            for plate in self.profile["markers"]:
                if plate.get("role", "gate") != role:
                    continue
                corners = plate["corners_xyz_m"]
                centre = [sum(point[axis] for point in corners) / 4.0 for axis in range(3)]
                marker.points.append(_point(*centre))
            if marker.points:
                markers.append(marker)
        return markers

    def _actor_markers(self) -> list[Marker]:
        """P behind the corner mass, and B down the next street."""

        low = self.profile["police_bounds_min_xyz_m"]
        high = self.profile["police_bounds_max_xyz_m"]
        police = self._marker("actors", 0, Marker.CUBE)
        police.scale.x = float(high[0] - low[0])
        police.scale.y = float(high[1] - low[1])
        police.scale.z = float(high[2] - low[2])
        police.pose.position = _point(
            (low[0] + high[0]) / 2.0, (low[1] + high[1]) / 2.0, (low[2] + high[2]) / 2.0
        )
        police.color = POLICE

        label = self._marker("actors", 1, Marker.TEXT_VIEW_FACING)
        label.scale.z = 0.45
        label.color = POLICE
        label.pose.position = _point(
            (low[0] + high[0]) / 2.0, (low[1] + high[1]) / 2.0, float(high[2]) + 0.5
        )
        label.text = "P (hidden by the corner mass)"

        bx, by, _ = self.manifest["actors"]["b_xyz_m"]
        person = self._marker("actors", 2, Marker.CYLINDER)
        person.scale.x = person.scale.y = 0.5
        person.scale.z = 1.8
        person.pose.position = _point(bx, by, 0.9)
        person.color = PERSON
        return [police, label, person]

    def _robot_marker(self) -> Marker | None:
        """A, placed at the station the camera measured -- never at truth."""

        estimate = self.latest_estimate
        if estimate is None or not estimate.valid:
            return None
        # approach_s_at_x raises outside the approach leg. Today a
        # SpeedMeasurement always carries a surveyed gate station, so this
        # cannot fire -- but this runs inside a subscription callback, and one
        # refactor that published a raw observation station would take the whole
        # display down. Drop the marker and say so instead.
        try:
            route_s_m = self.trajectory.approach_s_at_x(estimate.station_m)
        except ValueError:
            self.get_logger().warning(
                f"station {estimate.station_m:.3f} m is off the approach; "
                "not drawing the robot marker for this estimate"
            )
            return None
        pose = self.trajectory.pose_at(route_s_m)
        marker = self._marker("robot", 0, Marker.SPHERE)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.5
        marker.pose.position = _point(pose.x_m, pose.y_m, 0.35)
        marker.color = VIOLATING if self.latest_violation is not None else COMPLIANT
        return marker

    def _readout_marker(self) -> Marker:
        marker = self._marker("readout", 0, Marker.TEXT_VIEW_FACING)
        marker.scale.z = 0.55
        entry = float(self.profile["entry_width_m"])
        corner = float(self.profile["corner_width_m"])
        marker.pose.position = _point(4.0, entry / 2.0 + 2.5, 3.0)

        estimate = self.latest_estimate
        header = f"{self.profile_name}   m={entry:.1f} m   n={corner:.1f} m"
        if estimate is None or not estimate.valid:
            marker.color = WALL
            marker.text = f"{header}\nwaiting for a camera-derived estimate"
            return marker

        violation = self.latest_violation
        marker.color = VIOLATING if violation is not None else COMPLIANT
        lines = [
            header,
            f"station {estimate.station_m:6.2f} m    width {estimate.corridor_width_m:5.2f} m",
            f"speed   {estimate.speed_mps:6.2f} +/- {estimate.speed_stddev_mps:.2f} m/s",
            f"limit   {estimate.speed_limit_mps:6.2f} m/s",
        ]
        if violation is not None:
            lines.append(
                f"VIOLATION #{violation.event_id}   +{violation.exceedance_mps:.2f} m/s"
            )
        else:
            margin = estimate.speed_limit_mps - estimate.speed_mps
            lines.append(f"compliant   {margin:+.2f} m/s margin")
        marker.text = "\n".join(lines)
        return marker

    def _publish(self) -> None:
        array = MarkerArray()
        array.markers.extend(self._corridor_markers())
        array.markers.extend(self._occluder_markers())
        array.markers.append(self._route_marker())
        array.markers.extend(self._fiducial_markers())
        array.markers.extend(self._actor_markers())
        array.markers.append(self._readout_marker())
        robot = self._robot_marker()
        if robot is not None:
            array.markers.append(robot)
        self.markers.publish(array)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = EnforcementViewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


__all__ = ["EnforcementViewNode", "main"]
