"""ROS 2 publisher for deterministic, simulator-free camera playback."""

from __future__ import annotations

from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import CameraInfo, Image

from .synthetic import SyntheticCamera

# Match Jazzy's TimeSource subscription and rclcpp::ClockQoS explicitly.
CLOCK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class SyntheticPublisherNode(Node):
    """Publish calibrated frames and isolate truth on a test-only topic."""

    def __init__(self) -> None:
        super().__init__("synthetic_camera_publisher")
        self.declare_parameter("manifest_path", "")
        self.declare_parameter("corridor_profile", "")
        self.declare_parameter("speed_mps", 1.8)
        self.declare_parameter("start_station_m", 0.0)
        self.declare_parameter("end_station_m", 7.2)
        self.declare_parameter("publish_clock", False)
        self.declare_parameter("image_topic", "robot/front_camera/image_raw")
        self.declare_parameter("camera_info_topic", "robot/front_camera/camera_info")
        self.declare_parameter("truth_topic", "test/ground_truth/speed")
        manifest_value = str(self.get_parameter("manifest_path").value)
        if not manifest_value:
            raise ValueError("manifest_path is required")
        profile_value = str(self.get_parameter("corridor_profile").value)
        self.camera = SyntheticCamera(
            Path(manifest_value), profile_value if profile_value else None
        )
        self.speed_mps = float(self.get_parameter("speed_mps").value)
        self.start_station_m = float(self.get_parameter("start_station_m").value)
        self.end_station_m = float(self.get_parameter("end_station_m").value)
        self.publish_clock = bool(self.get_parameter("publish_clock").value)
        self.frame_index = 0
        self.finished = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        truth_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_publisher = self.create_publisher(
            Image, str(self.get_parameter("image_topic").value), sensor_qos
        )
        self.info_publisher = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            sensor_qos,
        )
        self.truth_publisher = self.create_publisher(
            TwistStamped, str(self.get_parameter("truth_topic").value), truth_qos
        )
        self.clock_publisher = (
            self.create_publisher(ClockMessage, "/clock", CLOCK_QOS)
            if self.publish_clock
            else None
        )
        self.period_s = 1.0 / self.camera.rate_hz
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(self.period_s, self._publish_frame, clock=steady_clock)

    def _publish_frame(self) -> None:
        if self.finished:
            return
        elapsed = self.frame_index * self.period_s
        # speed_mps is the distance actually travelled per second. Station is
        # world X, and under a one-sided taper the path runs at an angle to X,
        # so the station advances by only its X component.
        axis_fraction = self.camera.approach_heading[0]
        station = self.start_station_m + self.speed_mps * axis_fraction * elapsed
        if station > self.end_station_m:
            self.finished = True
            self.get_logger().info("synthetic sequence complete")
            return
        if self.publish_clock:
            timestamp_s = 1.0 + elapsed
            stamp = Time(nanoseconds=round(timestamp_s * 1e9)).to_msg()
            clock_message = ClockMessage()
            clock_message.clock = stamp
            assert self.clock_publisher is not None
            self.clock_publisher.publish(clock_message)
        else:
            stamp = self.get_clock().now().to_msg()

        pixels = self.camera.render(station)
        calibration = self.camera.calibration
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = calibration.frame_id
        image.height = calibration.height
        image.width = calibration.width
        image.encoding = "bgr8"
        image.is_bigendian = False
        image.step = calibration.width * 3
        image.data = pixels.tobytes()

        info = CameraInfo()
        info.header = image.header
        info.height = calibration.height
        info.width = calibration.width
        info.distortion_model = "plumb_bob"
        info.d = calibration.distortion.tolist()
        info.k = calibration.matrix.reshape(-1).tolist()
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        projection = [
            calibration.matrix[0, 0],
            0.0,
            calibration.matrix[0, 2],
            0.0,
            0.0,
            calibration.matrix[1, 1],
            calibration.matrix[1, 2],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        info.p = projection

        truth = TwistStamped()
        truth.header.stamp = stamp
        truth.header.frame_id = "corridor_map"
        truth.twist.linear.x = self.speed_mps
        self.info_publisher.publish(info)
        self.image_publisher.publish(image)
        self.truth_publisher.publish(truth)
        self.frame_index += 1


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SyntheticPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
