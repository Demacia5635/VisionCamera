# VisionCamera

Overhead Limelight 4 camera used as a pose/velocity **ground truth** reference for
Demacia 2027 robot testing. Analysis is offline (post-run), not real-time.

## Requirements

- Camera: Limelight 4, mounted overhead above part of the field at ~3.5 m.
- Recording workflow: start a recorder of the camera stream, run a command to
  move the robot, then stop the recording.
- Review workflow: play back the recording and select 2 frames from it.
- Annotation: on each of the 2 frames, mark the robot's position and heading.
- Output: the robot pose in field coordinates for each frame, plus between
  the two points:
  - distance
  - direction
  - velocity
  - omega (angular velocity)

## What was built

The project had an existing scaffold (`VideoWriter.py` recorder,
`viewCaptures.py` frame browser). Added the missing pieces to cover the full
workflow above:

| File | Purpose |
|---|---|
| `VisionCamera/VideoWriter.py` | Records the camera stream to MP4 + periodic JPEG snapshots. Refactored into a proper CLI (`--url`, `--save-dir`, `--save-interval`, `--jpeg-quality`, `--out`); default URL updated to a Limelight-style MJPEG endpoint. *(existing file, refactored)* |
| `VisionCamera/viewCaptures.py` | Browses/blends periodic snapshots to eyeball motion. Added a `main()` entry point. *(existing file, minor update)* |
| `VisionCamera/calibration_tool.py` | **New.** Click ≥4 known field points on a reference frame, enter each one's real field X/Y (meters); fits a pixel→field homography (`cv2.findHomography`), shows reprojection error, saves `calibration.json`. Run once per camera mount. Optionally undistorts the reference image first via `--lens-calibration`. |
| `VisionCamera/pose_analyzer.py` | **New.** Scrub the recorded video frame-by-frame with a live preview, capture two frames (A/B). On each, click once for robot position and once more for a heading point. Converts both to field coordinates via the calibration, computes distance/direction/velocity/omega between A and B, and can append results to `results.csv`. Optionally undistorts every frame first via `--lens-calibration`. |
| `VisionCamera/lens_calib.py` | **New.** Checkerboard-based fisheye intrinsic calibration (`cv2.fisheye.calibrate`) for a wide-FOV/fisheye USB camera. Live-captures checkerboard samples from the camera, saves `lens_calibration.json`. One-time step per camera. |
| `VisionCamera/checkerboard.py` | **New.** Generates a print-ready checkerboard PNG (sized in mm, at 300 dpi) matching `lens_calib.py`'s `--cols`/`--rows`, so there's no need to source one externally. Verified OpenCV's `findChessboardCorners` detects it. |
| `VisionCamera/undistort.py` | **New.** `LensCalibration`: loads `lens_calibration.json`, builds/caches `cv2.fisheye` remap tables, and undistorts frames. Used by `calibration_tool.py` and `pose_analyzer.py`. |
| `VisionCamera/field_coords.py` | **New.** Core math: `FieldCalibration` (homography wrapper), `Pose2d`, `kinematics_between()`. No GUI dependency — unit tested. |
| `tests/test_VisionCamera.py` | **New.** 10 unit tests covering the homography round-trip, heading computation, angle wrapping, kinematics, and lens undistortion. |
| `pyproject.toml` | Added runtime deps (`opencv-python`, `numpy`, `Pillow`) and 6 console-script entry points. |

## Coordinate convention

Meters, WPILib field convention: origin per your calibration points, heading
in radians counter-clockwise from +X. Chosen so results compare directly
against the robot's own odometry/`Pose2d`.

## Usage

```
# 0. Lens calibration (one-time per camera; required for a fisheye/wide-FOV
#    camera like a 150 deg USB cam -- see "Fisheye / wide-FOV lens" below).
#    --cols/--rows are interior corners of your checkerboard.
python -m VisionCamera.lens_calib --source 0 --cols 9 --rows 6 -o lens_calibration.json

# 1. Record (run your robot-move command while this is running, 'q' to stop)
python -m VisionCamera.VideoWriter --source 0
# --source also takes a stream URL (e.g. http://limelight.local:5800/stream.mjpg)
# instead of a USB device index, and --width/--height/--cam-fps request a
# capture resolution/rate from the camera if the default is too low.

# 2. Calibrate the field homography once per camera mount, using a frame that
#    shows >=4 known field points. Pass --lens-calibration if you did step 0.
python -m VisionCamera.calibration_tool reference_frame.jpg -o calibration.json \
    --lens-calibration lens_calibration.json

# 3. Mark two frames from a recording and read off distance/direction/velocity/omega.
#    Use the SAME --lens-calibration here as in step 2, or marks won't line up.
python -m VisionCamera.pose_analyzer recording_20260919_120000.mp4 -c calibration.json \
    --lens-calibration lens_calibration.json -r results.csv
```

## Fisheye / wide-FOV lens

`calibration_tool.py`'s field homography assumes a plain perspective
(pinhole) projection. A ~150 deg USB camera has enough barrel distortion
that a homography fit directly to its raw pixels is only accurate near the
points you clicked, and drifts further off toward the edges. `lens_calib.py`
fixes this by fitting OpenCV's fisheye distortion model
(`cv2.fisheye.calibrate`) from checkerboard samples, and `undistort.py`
applies it (via `--lens-calibration`) to rectify frames *before* the field
homography ever sees them. Steps:

1. Generate and print a target: `python -m VisionCamera.checkerboard -o checkerboard.png`
   (defaults to 10x7 squares @ 25 mm; matches `lens_calib.py`'s default
   `--cols 9 --rows 6`). Print at 100%/actual size -- not "fit to page" --
   and verify a square measures 25 mm with a ruler before trusting it.
   Mount on something flat and rigid (cardboard, foam board) so it doesn't
   bow; a warped board hurts calibration accuracy.
2. Run `lens_calib.py`, moving the board to cover the whole frame -- center,
   all four corners, some tilt -- since distortion is worst at the edges
   and that's exactly where samples matter most. Capture 15-20+ samples.
3. Use the resulting `lens_calibration.json` with `--lens-calibration` in
   *both* `calibration_tool.py` and `pose_analyzer.py` -- they must agree,
   since the field homography is only valid for the undistorted image it
   was calibrated against.

## Open items / caveats

- `VideoWriter.py` now captures from a USB camera by device index (default
  `0`) instead of a network stream; if `0` isn't the right camera, try `1`,
  `2`, etc. via `--source`. A stream URL still works too (`--source
  http://limelight.local:5800/stream.mjpg`) for setups that record over the
  network instead of USB.
- Frame seeking uses OpenCV's `CAP_PROP_POS_FRAMES`, which is generally
  accurate for our own mp4v recordings but not guaranteed frame-exact for
  all codecs.
- Not yet verified against a real camera or real recording — the math/GUI
  logic is unit tested, but end-to-end use with actual hardware is still
  pending.
- `calibration_tool.py` had picked up two breaking edits from outside this
  session (an absolute `from field_coords import ...` that made the module
  fail to import at all under `python -m`, and `self.calib` declared as a
  bare annotation with no `= None`, which could raise `AttributeError` from
  `redraw()`). Both were fixed while wiring in lens-calibration support,
  since the module has to import correctly for that to work at all.
- `lens_calib.py`/`undistort.py` are unit-tested for the math (save/load,
  a zero-distortion sanity check) but not yet run against a real fisheye
  USB camera or checkerboard — `cv2.fisheye.calibrate` can be finicky about
  sample coverage/count, so expect to iterate on capture quality.
