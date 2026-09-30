"""Build a pixel->field homography calibration for a fixed overhead camera.

Usage:  python -m VisionCamera.calibration_tool <reference_image> [-o calibration.json]

Pick a reference frame (e.g. one exported from a recording) that shows at
least 4 points whose real field coordinates you know -- field tape
intersections, AprilTag bases, or markers you placed and measured by hand.
Click each one on the image, enter its field X/Y in meters, and repeat for
all of them (more points spread across the visible area = a more accurate,
more RANSAC-robust fit). "Compute & save" writes calibration.json, which
pose_analyzer.py loads to convert every future pixel click from this camera
into field coordinates.

The points do not need to be square/rectangular or evenly spaced -- any 4+
non-collinear points work, since a homography is fit to them directly.
"""
from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np
from undistort import LensCalibration

def test():
    cap = cv2.VideoCapture('rec1.mp4')
    ret, f = cap.read()
    if not ret:
        print('no data')
        exit(1)
    #f  = cv2.resize(f, (1600,900), fx=0, fy=0, interpolation=cv2.INTER_LINEAR)
    print(f.shape)
    cv2.imshow('base', f)
    l = LensCalibration.load('lens_calibration.json')
    uf = l.undistort(f)
    cv2.imshow('uf', uf)
    cv2.waitKey(0)

if __name__ == "__main__":
    test()
