"""ROS 2 adapter for the camera-only police observer."""

from __future__ import annotations

from pathlib import Path

import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from corridor_interfaces.msg import SpeedEstimate, SpeedViolation

from .estimator import (
    ArucoStationEstimator,
    Calibration,
    GateSpeedEstimator,
    MarkerMap,
    SpeedMeasurement,
    Violation,
    ViolationDetector,
)


class PoliceObserverNode(Node):
    """Consume only image/calibration evidence and publish speed decisions."""

    def __init__(self) -> None:
        super().__init__("police_observer")
        self.declare_parameter("manifest_path", "")
        self.declare_parameter("corridor_profile", "")
        self.declare_parameter("marker_dictionary", "DICT_5X5_100")
        self.declare_parameter("pose_reprojection_error_px", 3.0)
        self.declare_parameter("image_topic", "robot/front_camera/image_raw")
        self.declare_parameter("camera_info_topic", "robot/front_camera/camera_info")
        self.declare_parameter("estimate_topic", "police/speed_estimate")
        self.declare_parameter("violation_topic", "police/speed_violation")

        manifest_value = self.get_parameter("manifest_path").value
        if not manifest_value:
            raise ValueError("manifest_path is required")
        profile_value = str(self.get_parameter("corridor_profile").value)
        marker_map = MarkerMap.from_manifest(
            Path(str(manifest_value)), profile_value if profile_value else None
        )
        self.station_estimator = ArucoStationEstimator(
            marker_map,
            str(self.get_parameter("marker_dictionary").value),
            float(self.get_parameter("pose_reprojection_error_px").value),
        )
        self.speed_estimator = GateSpeedEstimator(marker_map)
        self.violation_detector = ViolationDetector(marker_map)
        self.bridge = CvBridge()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.estimate_publisher = self.create_publisher(
            SpeedEstimate, str(self.get_parameter("estimate_topic").value), output_qos
        )
        self.violation_publisher = self.create_publisher(
            SpeedViolation, str(self.get_parameter("violation_topic").value), output_qos
        )
        self.image_subscriber = message_filters.Subscriber(
            self,
            Image,
            str(self.get_parameter("image_topic").value),
            qos_profile=sensor_qos,
        )
        self.info_subscriber = message_filters.Subscriber(
            self,
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            qos_profile=sensor_qos,
        )
        self.synchronizer = message_filters.TimeSynchronizer(
            [self.image_subscriber, self.info_subscriber], queue_size=10
        )
        self.synchronizer.registerCallback(self._on_frame)
        self.get_logger().info(
            f"camera-only observer ready for profile {marker_map.profile_name!r}"
        )

    def _on_frame(self, image_message: Image, info_message: CameraInfo) -> None:
        if image_message.header.frame_id != info_message.header.frame_id:
            self.get_logger().warning("discarding frame with mismatched CameraInfo frame")
            return
        if image_message.width != info_message.width or image_message.height != info_message.height:
            self.get_logger().warning("discarding frame with mismatched CameraInfo dimensions")
            return
        timestamp_s = Time.from_msg(image_message.header.stamp).nanoseconds / 1e9
        if timestamp_s <= 0.0:
            self.speed_estimator.reset()
            self.violation_detector.reset()
            return
        if image_message.encoding not in {"bgr8", "rgb8", "mono8"}:
            self.get_logger().warning(
                f"discarding unsupported image encoding {image_message.encoding!r}"
            )
            return
        image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
        image = np.asarray(image)
        if image_message.encoding == "rgb8":
            image = image[:, :, ::-1]
        calibration = Calibration(
            width=int(info_message.width),
            height=int(info_message.height),
            matrix=np.asarray(info_message.k, dtype=np.float64).reshape(3, 3),
            distortion=np.asarray(info_message.d, dtype=np.float64),
            frame_id=info_message.header.frame_id,
        )
        observation = self.station_estimator.estimate(image, calibration, timestamp_s)
        if observation is None:
            return
        for measurement in self.speed_estimator.update(observation):
            estimate_message = self._estimate_message(measurement)
            self.estimate_publisher.publish(estimate_message)
            violation = self.violation_detector.update(measurement)
            if violation is not None:
                self.violation_publisher.publish(
                    self._violation_message(violation, estimate_message)
                )

    def _estimate_message(self, value: SpeedMeasurement) -> SpeedEstimate:
        message = SpeedEstimate()
        message.header.stamp = Time(nanoseconds=round(value.timestamp_s * 1e9)).to_msg()
        message.header.frame_id = "corridor_map"
        message.corridor_profile = self.speed_estimator.marker_map.profile_name
        message.station_m = value.station_m
        message.speed_mps = value.speed_mps
        message.speed_stddev_mps = value.speed_stddev_mps
        message.corridor_width_m = value.corridor_width_m
        message.speed_limit_mps = value.speed_limit_mps
        message.gate_from_id = value.gate_from_id
        message.gate_to_id = value.gate_to_id
        message.observation_count = value.observation_count
        message.valid = True
        return message

    @staticmethod
    def _violation_message(value: Violation, estimate_message: SpeedEstimate) -> SpeedViolation:
        message = SpeedViolation()
        message.event_id = value.event_id
        message.estimate = estimate_message
        message.exceedance_mps = value.exceedance_mps
        message.confirmation_duration_s = value.confirmation_duration_s
        message.reason = "conservative camera-derived speed exceeds demonstration limit"
        return message


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PoliceObserverNode()
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
