"""Apply a fisheye lens intrinsic calibration (see lens_calib.py) to frames.

A wide-FOV lens (your 150 deg USB camera included) has enough barrel
distortion that a single flat homography -- which assumes a plain
perspective/pinhole projection -- is only accurate near the points you
calibrated it with, and drifts increasingly toward the frame edges. The fix
is to undistort every frame with the lens's actual intrinsics *before*
computing or applying the field homography, so everything downstream
(calibration_tool.py and pose_analyzer.py) works in an already-rectified,
pinhole-like image.
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np


class LensCalibration:
    """Camera intrinsics (K, D) for OpenCV's fisheye model, plus cached undistort maps."""

    def __init__(self, K, D, image_size, balance: float = 0.0):
        self.K = np.asarray(K, dtype=np.float64)
        self.D = np.asarray(D, dtype=np.float64)
        self.image_size = tuple(image_size)  # (width, height) at calibration time
        self.balance = balance  # 0 = crop toward valid pixels, 1 = keep full FOV (black corners)
        self._map1 = None
        self._map2 = None
        self._map_size = None
        self.new_K = None
        self._build_maps(self.image_size)

    def _build_maps(self, size):
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.K, self.D, size, np.eye(3), balance=self.balance)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K, self.D, np.eye(3), new_K, size, cv2.CV_16SC2)
        self._map1, self._map2, self._map_size, self.new_K = map1, map2, size, new_K

    def undistort(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        if self._map_size != (w, h):
            self._build_maps((w, h))
            print("build maps - " + str(frame.shape))
        return cv2.remap(frame, self._map1, self._map2, interpolation=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT)

    def save(self, path: str | Path) -> None:
        data = {
            "K": self.K.tolist(), "D": self.D.tolist(),
            "image_size": list(self.image_size), "balance": self.balance,
        }
        Path(path).write_text(json.dumps(data, indent=2))

    @classmethod
    def load(cls, path: str | Path) -> "LensCalibration":
        data = json.loads(Path(path).read_text())
        return cls(data["K"], data["D"], data["image_size"], data.get("balance", 0.0))
