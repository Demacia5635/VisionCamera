"""Calibrate the USB camera's fisheye lens intrinsics from a checkerboard.

Usage:  python -m VisionCamera.lens_calib --source 0 --cols 9 --rows 6 [-o lens_calibration.json]

This is a one-time step per camera (redo it if you change focus/zoom or
swap cameras). It fits OpenCV's fisheye distortion model, appropriate for a
~150 deg USB camera where the standard pinhole model isn't accurate enough.
The saved lens_calibration.json is then used by calibration_tool.py and
pose_analyzer.py to undistort frames before mapping pixels to field
coordinates.

--cols/--rows are the number of *interior corners* of your checkerboard
(one less than the number of squares per side), e.g. a 10x7-square board
has --cols 9 --rows 6.

Controls: point the checkerboard at the camera, tilted/moved to cover
different parts of the frame (center, each corner, some tilt) -- fisheye
distortion is worst at the edges, so samples there matter most. Press 'c'
to capture a sample once green corners are shown, 'q' when you have enough
(15-20+) to compute and save.
"""
from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np

from VideoWriter import parse_source

MIN_SAMPLES = 8


def open_camera(source):
    if isinstance(source, int) and sys.platform == "win32":
        cap = cv2.VideoCapture(source, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(source)  
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera/stream: {source}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
    cap.set(cv2.CAP_PROP_FPS, 50.0)
    return cap


def calibrate(source, cols: int, rows: int, square_size: float, out_path: str, min_samples: int = MIN_SAMPLES):
    board_size = (cols, rows)
    objp = np.zeros((cols * rows, 3), dtype=np.float64)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size

    cap = open_camera(source)
    obj_points = []
    img_points = []
    image_size = None

    print("Move the checkerboard to cover the whole frame (center, corners, some tilt).\n"
          "Press 'c' to capture a sample, 'q' to finish and compute.")

    find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_NORMALIZE_IMAGE
    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Frame grab failed (camera disconnected?)")
                break
            image_size = (frame.shape[1], frame.shape[0])
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, board_size, flags=find_flags)

            display = frame.copy()
            if found:
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), subpix_criteria)
                cv2.drawChessboardCorners(display, board_size, corners, found)

            cv2.putText(display, f"samples: {len(obj_points)}  (need >= {min_samples})",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if found else (0, 0, 255), 2)
            cv2.imshow("Lens calibration - checkerboard", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("c") and found:
                obj_points.append(objp.reshape(1, -1, 3))
                img_points.append(corners.reshape(1, -1, 2))
                print(f"Captured sample {len(obj_points)} {image_size}")
            elif key == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

    if len(obj_points) < min_samples:
        raise RuntimeError(f"Only {len(obj_points)} samples captured, need at least {min_samples}")

    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    flags = cv2.CALIB_RECOMPUTE_EXTRINSIC | cv2.CALIB_CHECK_COND | cv2.CALIB_FIX_SKEW
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

    rms, K, D, _rvecs, _tvecs = cv2.fisheye.calibrate(
        obj_points, img_points, image_size, K, D, None, None, flags, criteria)

    print(f"Calibrated from {len(obj_points)} samples, RMS reprojection error = {rms:.3f} px")

    from undistort import LensCalibration
    LensCalibration(K, D, image_size).save(out_path)
    print(f"Saved {out_path}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", default="2", help="USB camera device index or a stream URL")
    parser.add_argument("--cols", type=int, required=True, help="Interior corners per row (squares_per_row - 1)")
    parser.add_argument("--rows", type=int, required=True, help="Interior corners per column (squares_per_col - 1)")
    parser.add_argument("--square-size", type=float, default=1.0,
                         help="Checkerboard square size (any consistent unit; only affects scale, not distortion)")
    parser.add_argument("--min-samples", type=int, default=MIN_SAMPLES, help="Minimum samples required")
    parser.add_argument("-o", "--out", default="lens_calibration.json", help="Output calibration file")
    args = parser.parse_args(argv)

    calibrate(parse_source(args.source), args.cols, args.rows, args.square_size, args.out, args.min_samples)


if __name__ == "__main__":
    main(sys.argv[1:])
