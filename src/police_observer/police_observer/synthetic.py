"""Deterministic calibrated ArUco frame generation without a simulator."""

from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np

from .estimator import Calibration


def _aruco_dictionary(name: str):
    dictionary_id = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dictionary_id)
    return cv2.aruco.Dictionary_get(dictionary_id)


def _marker_image(dictionary, marker_id: int, size_px: int = 256) -> np.ndarray:
    image = np.zeros((size_px, size_px), dtype=np.uint8)
    if hasattr(cv2.aruco, "generateImageMarker"):
        cv2.aruco.generateImageMarker(dictionary, marker_id, size_px, image, 1)
    else:
        cv2.aruco.drawMarker(dictionary, marker_id, size_px, image, 1)
    return image


class SyntheticCamera:
    """Render surveyed marker planes through a ROS-convention pinhole camera."""

    def __init__(self, manifest_path: Path, profile_name: str | None = None) -> None:
        raw = json.loads(manifest_path.read_text(encoding="utf-8"))
        profile = profile_name or str(raw["selected_profile"])
        camera = raw["camera"]
        width = int(camera["width_px"])
        height = int(camera["height_px"])
        focal = (width / 2.0) / math.tan(math.radians(float(camera["horizontal_fov_deg"])) / 2.0)
        matrix = np.array(
            [[focal, 0.0, (width - 1) / 2.0], [0.0, focal, (height - 1) / 2.0], [0, 0, 1]],
            dtype=np.float64,
        )
        self.calibration = Calibration(
            width=width,
            height=height,
            matrix=matrix,
            distortion=np.zeros(5, dtype=np.float64),
            frame_id=str(camera["frame_id"]),
        )
        self.camera_height_m = float(camera["mount_height_m"])
        self.rate_hz = float(camera["rate_hz"])
        self.dictionary_name = str(raw["fiducials"]["dictionary"])
        self.dictionary = _aruco_dictionary(self.dictionary_name)
        block = raw["profiles"][profile]
        self.markers = {
            int(marker["id"]): np.asarray(marker["aruco_corner_order_xyz_m"], dtype=np.float64)
            for marker in block["markers"]
        }
        self.images = {
            marker_id: _marker_image(self.dictionary, marker_id) for marker_id in self.markers
        }
        # Follow the authored corridor centreline rather than assuming it runs
        # along y = 0. Under a one-sided taper the centreline drifts toward the
        # straight north face, and a camera left on the old axis would view
        # every marker off-centre.
        route = block.get("delivery_trajectory", {})
        heading = route.get("approach_heading", (1.0, 0.0))
        self.approach_heading = (float(heading[0]), float(heading[1]))
        start = route.get("start_xyz_m", (0.0, 0.0, 0.0))
        self.approach_start_xy = (float(start[0]), float(start[1]))

    def centerline_y(self, station_m: float) -> float:
        """Return the centreline Y at a corridor station (world X)."""

        forward_x, forward_y = self.approach_heading
        origin_x, origin_y = self.approach_start_xy
        return origin_y + (forward_y / forward_x) * (station_m - origin_x)

    def _world_to_camera(self, world: np.ndarray, station_m: float) -> np.ndarray:
        forward_x, forward_y = self.approach_heading
        origin = np.array([station_m, self.centerline_y(station_m), self.camera_height_m])
        relative = world - origin
        # ROS optical frame: X right, Y down, Z forward along the heading. With
        # a heading of +X this reduces to the axis-aligned case of X = world -Y
        # and Y = world -Z.
        return np.column_stack(
            (
                relative[:, 0] * forward_y - relative[:, 1] * forward_x,
                -relative[:, 2],
                relative[:, 0] * forward_x + relative[:, 1] * forward_y,
            )
        )

    def render(
        self,
        station_m: float,
        noise_stddev: float = 0.0,
        blur_kernel: int = 0,
    ) -> np.ndarray:
        canvas = np.full((self.calibration.height, self.calibration.width), 210, dtype=np.uint8)
        visible: list[tuple[float, int, np.ndarray]] = []
        for marker_id, world in self.markers.items():
            camera = self._world_to_camera(world, station_m)
            if np.any(camera[:, 2] <= 0.05):
                continue
            pixels = np.column_stack(
                (
                    self.calibration.matrix[0, 0] * camera[:, 0] / camera[:, 2]
                    + self.calibration.matrix[0, 2],
                    self.calibration.matrix[1, 1] * camera[:, 1] / camera[:, 2]
                    + self.calibration.matrix[1, 2],
                )
            ).astype(np.float32)
            if cv2.contourArea(pixels) < 12.0:
                continue
            if (
                np.max(pixels[:, 0]) < 0
                or np.min(pixels[:, 0]) >= self.calibration.width
                or np.max(pixels[:, 1]) < 0
                or np.min(pixels[:, 1]) >= self.calibration.height
            ):
                continue
            visible.append((float(np.mean(camera[:, 2])), marker_id, pixels))

        source_size = 256
        source = np.array(
            [
                [0, 0],
                [source_size - 1, 0],
                [source_size - 1, source_size - 1],
                [0, source_size - 1],
            ],
            dtype=np.float32,
        )
        for _, marker_id, pixels in sorted(visible, reverse=True):
            transform = cv2.getPerspectiveTransform(source, pixels)
            warped = cv2.warpPerspective(
                self.images[marker_id],
                transform,
                (self.calibration.width, self.calibration.height),
                flags=cv2.INTER_NEAREST,
                borderValue=255,
            )
            mask_source = np.full((source_size, source_size), 255, dtype=np.uint8)
            mask = cv2.warpPerspective(
                mask_source,
                transform,
                (self.calibration.width, self.calibration.height),
                flags=cv2.INTER_NEAREST,
                borderValue=0,
            )
            canvas[mask > 0] = warped[mask > 0]
        if blur_kernel:
            if blur_kernel % 2 == 0:
                raise ValueError("blur kernel must be odd")
            canvas = cv2.GaussianBlur(canvas, (blur_kernel, blur_kernel), 0)
        if noise_stddev > 0.0:
            random = np.random.default_rng(0)
            noise = random.normal(0.0, noise_stddev, canvas.shape)
            canvas = np.clip(canvas.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
