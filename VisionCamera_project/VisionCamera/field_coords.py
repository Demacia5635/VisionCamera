"""Pixel <-> field coordinate mapping and 2D pose/kinematics math.

Coordinate convention follows WPILib's field coordinate system so results can
be compared directly against robot odometry: X/Y in meters, heading in
radians, counter-clockwise positive, 0 rad = facing +X.

The pixel->field mapping is a planar homography: it assumes every marked
point lies on the floor (z=0), which is the correct assumption for a robot's
footprint viewed by a camera fixed above the field. The homography is
computed once from >=4 known (pixel, field) point correspondences -- see
calibration_tool.py -- and reused for every frame from that camera setup.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np


@dataclass
class Pose2d:
    x: float        # field X, meters
    y: float        # field Y, meters
    heading: float  # radians, CCW from +X (WPILib convention)

    def __str__(self) -> str:
        return f"({self.x:+.3f}, {self.y:+.3f}) m  heading={math.degrees(self.heading):+.1f} deg"


@dataclass
class Kinematics:
    dt: float         # seconds, frame B time - frame A time
    distance: float   # meters, straight-line distance between positions
    direction: float  # radians, bearing of the A->B translation vector
    velocity: float   # m/s, distance / dt
    omega: float      # rad/s, wrapped heading change / dt

    def __str__(self) -> str:
        return (
            f"dt={self.dt:.3f} s  distance={self.distance:.3f} m  "
            f"direction={math.degrees(self.direction):+.1f} deg  "
            f"velocity={self.velocity:.3f} m/s  omega={math.degrees(self.omega):+.1f} deg/s"
        )


def wrap_angle(angle: float) -> float:
    """Wrap an angle (radians) to [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class FieldCalibration:
    """A pixel->field homography for one fixed camera setup."""

    def __init__(self, homography: np.ndarray, meta: dict | None = None):
        self.H = np.asarray(homography, dtype=np.float64)
        self.meta = meta or {}

    @classmethod
    def from_points(
        cls,
        image_points,
        field_points,
        meta: dict | None = None,
    ) -> "FieldCalibration":
        image_points = np.asarray(image_points, dtype=np.float64)
        field_points = np.asarray(field_points, dtype=np.float64)
        if len(image_points) < 4:
            raise ValueError("Need at least 4 point correspondences for a homography")
        if len(image_points) != len(field_points):
            raise ValueError("image_points and field_points must be the same length")
        method = cv2.RANSAC if len(image_points) > 4 else 0
        H, _ = cv2.findHomography(image_points, field_points, method)
        if H is None:
            raise ValueError("Homography computation failed (points may be collinear)")
        return cls(H, meta)

    def pixel_to_field(self, px: float, py: float) -> tuple[float, float]:
        pt = np.array([[[px, py]]], dtype=np.float64)
        out = cv2.perspectiveTransform(pt, self.H)
        return float(out[0, 0, 0]), float(out[0, 0, 1])

    def field_to_pixel(self, fx: float, fy: float) -> tuple[float, float]:
        pt = np.array([[[fx, fy]]], dtype=np.float64)
        out = cv2.perspectiveTransform(pt, np.linalg.inv(self.H))
        return float(out[0, 0, 0]), float(out[0, 0, 1])

    def save(self, path: str | Path) -> None:
        data = {"homography": self.H.tolist(), "meta": self.meta}
        Path(path).write_text(json.dumps(data, indent=2))

    @classmethod
    def load(cls, path: str | Path) -> "FieldCalibration":
        data = json.loads(Path(path).read_text())
        return cls(np.array(data["homography"], dtype=np.float64), data.get("meta", {}))


def pose_from_marks(
    calib: FieldCalibration,
    position_px: tuple[float, float],
    heading_px: tuple[float, float],
) -> Pose2d:
    """Build a field-frame Pose2d from two pixel clicks.

    position_px: pixel under the robot's reference point (e.g. bumper/frame
    center, projected onto the floor).
    heading_px: a second pixel further along the direction the robot faces
    (e.g. a point on the front of the robot). Only its direction from
    position_px matters, not its distance.
    """
    x, y = calib.pixel_to_field(*position_px)
    hx, hy = calib.pixel_to_field(*heading_px)
    heading = math.atan2(hy - y, hx - x)
    return Pose2d(x, y, heading)


def kinematics_between(pose_a: Pose2d, t_a: float, pose_b: Pose2d, t_b: float) -> Kinematics:
    dt = t_b - t_a
    if dt == 0:
        raise ValueError("The two frames have identical timestamps")
    dx = pose_b.x - pose_a.x
    dy = pose_b.y - pose_a.y
    distance = math.hypot(dx, dy)
    direction = math.atan2(dy, dx)
    velocity = distance / dt
    omega = wrap_angle(pose_b.heading - pose_a.heading) / dt
    return Kinematics(dt, distance, direction, velocity, omega)
