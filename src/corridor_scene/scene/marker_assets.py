"""Generate compact ArUco texture assets for the authored stage."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


def aruco_dictionary(name: str):
    """Resolve a configured OpenCV predefined dictionary."""

    if not hasattr(cv2.aruco, name):
        raise ValueError(f"OpenCV does not provide ArUco dictionary {name!r}")
    dictionary_id = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dictionary_id)
    return cv2.aruco.Dictionary_get(dictionary_id)


def generate_marker_images(
    directory: Path,
    dictionary_name: str,
    marker_ids: set[int],
    size_px: int = 256,
) -> dict[int, Path]:
    """Write deterministic grayscale marker PNG files."""

    directory.mkdir(parents=True, exist_ok=True)
    dictionary = aruco_dictionary(dictionary_name)
    paths: dict[int, Path] = {}
    for marker_id in sorted(marker_ids):
        image = np.zeros((size_px, size_px), dtype=np.uint8)
        if hasattr(cv2.aruco, "generateImageMarker"):
            cv2.aruco.generateImageMarker(dictionary, marker_id, size_px, image, 1)
        else:
            cv2.aruco.drawMarker(dictionary, marker_id, size_px, image, 1)
        path = directory / f"marker_{marker_id:03d}.png"
        if not cv2.imwrite(str(path), image):
            raise OSError(f"failed to write marker texture {path}")
        paths[marker_id] = path
    return paths
