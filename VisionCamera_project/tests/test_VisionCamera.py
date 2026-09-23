import math

import pytest

from VisionCamera.field_coords import (
    FieldCalibration,
    kinematics_between,
    pose_from_marks,
    wrap_angle,
)

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
