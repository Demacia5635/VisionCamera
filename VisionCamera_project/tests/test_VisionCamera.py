import math

import numpy as np
import pytest

from VisionCamera.field_coords import (
    FieldCalibration,
    kinematics_between,
    pose_from_marks,
    wrap_angle,
)
from VisionCamera.undistort import LensCalibration

# A simple axis-aligned mapping: pixel (100*x, 100*y) <-> field (x, y) meters.
# Pixel Y is flipped relative to field Y, like a camera looking straight down
# with the field's +Y pointing "up" in the image.
IMAGE_POINTS = [(0, 400), (400, 400), (400, 0), (0, 0)]
FIELD_POINTS = [(0, 0), (4, 0), (4, 4), (0, 4)]


@pytest.fixture
def calib():
    return FieldCalibration.from_points(IMAGE_POINTS, FIELD_POINTS)


def test_pixel_to_field_round_trip(calib):
    fx, fy = calib.pixel_to_field(200, 200)
    assert fx == pytest.approx(2.0, abs=1e-6)
    assert fy == pytest.approx(2.0, abs=1e-6)

    px, py = calib.field_to_pixel(2.0, 2.0)
    assert px == pytest.approx(200, abs=1e-3)
    assert py == pytest.approx(200, abs=1e-3)


def test_pose_from_marks_heading(calib):
    # Position at field (1,1); heading mark one pixel further "right" in the
    # image, i.e. toward increasing field X -> heading should be ~0 rad.
    pose = pose_from_marks(calib, position_px=(100, 300), heading_px=(200, 300))
    assert pose.x == pytest.approx(1.0, abs=1e-6)
    assert pose.y == pytest.approx(1.0, abs=1e-6)
    assert pose.heading == pytest.approx(0.0, abs=1e-6)


def test_pose_from_marks_heading_up_field(calib):
    # Heading mark straight "up" in the image = increasing field Y -> pi/2.
    pose = pose_from_marks(calib, position_px=(100, 300), heading_px=(100, 100))
    assert pose.heading == pytest.approx(math.pi / 2, abs=1e-6)


def test_wrap_angle():
    assert wrap_angle(0) == pytest.approx(0)
    assert wrap_angle(math.pi) == pytest.approx(-math.pi)
    assert wrap_angle(3 * math.pi) == pytest.approx(-math.pi)
    assert wrap_angle(-3 * math.pi) == pytest.approx(-math.pi)
    assert wrap_angle(2 * math.pi + 0.1) == pytest.approx(0.1)


def test_kinematics_between_straight_line(calib):
    pose_a = pose_from_marks(calib, (100, 300), (200, 300))  # (1,1), heading 0
    pose_b = pose_from_marks(calib, (300, 300), (400, 300))  # (3,1), heading 0

    k = kinematics_between(pose_a, 0.0, pose_b, 2.0)
    assert k.dt == pytest.approx(2.0)
    assert k.distance == pytest.approx(2.0)
    assert k.direction == pytest.approx(0.0, abs=1e-6)
    assert k.velocity == pytest.approx(1.0)
    assert k.omega == pytest.approx(0.0, abs=1e-6)


def test_kinematics_between_rotation(calib):
    pose_a = pose_from_marks(calib, (100, 300), (200, 300))  # heading 0
    pose_b = pose_from_marks(calib, (100, 300), (100, 100))  # heading pi/2

    k = kinematics_between(pose_a, 0.0, pose_b, 1.0)
    assert k.distance == pytest.approx(0.0, abs=1e-6)
    assert k.omega == pytest.approx(math.pi / 2, abs=1e-6)


def test_kinematics_rejects_zero_dt(calib):
    pose_a = pose_from_marks(calib, (100, 300), (200, 300))
    with pytest.raises(ValueError):
        kinematics_between(pose_a, 1.0, pose_a, 1.0)


def test_calibration_needs_four_points():
    with pytest.raises(ValueError):
        FieldCalibration.from_points(IMAGE_POINTS[:3], FIELD_POINTS[:3])


def test_lens_calibration_round_trip(tmp_path):
    K = [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]]
    D = [0.01, -0.005, 0.0, 0.0]
    lens = LensCalibration(K, D, (640, 480), balance=0.5)

    path = tmp_path / "lens_calibration.json"
    lens.save(path)
    loaded = LensCalibration.load(path)
    assert loaded.image_size == (640, 480)
    assert loaded.balance == pytest.approx(0.5)
    assert np.allclose(loaded.K, lens.K)
    assert np.allclose(loaded.D, lens.D)


def test_lens_calibration_undistort_zero_distortion_is_near_identity():
    # With D=0 the fisheye model has nothing to correct, so undistort()
    # should leave a frame essentially unchanged (up to the balance-driven
    # new camera matrix / interpolation).
    K = [[300.0, 0.0, 160.0], [0.0, 300.0, 120.0], [0.0, 0.0, 1.0]]
    D = [0.0, 0.0, 0.0, 0.0]
    lens = LensCalibration(K, D, (320, 240), balance=1.0)

    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    frame[100:140, 140:180] = 255  # a small white square near the center

    out = lens.undistort(frame)
    assert out.shape == frame.shape
    # The bright square should still be roughly centered and roughly the
    # same brightness total, i.e. not lost off-frame or blacked out.
    assert out[100:140, 140:180].mean() > 200
