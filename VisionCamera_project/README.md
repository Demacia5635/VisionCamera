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
| `VisionCamera/calibration_tool.py` | **New.** Click ≥4 known field points on a reference frame, enter each one's real field X/Y (meters); fits a pixel→field homography (`cv2.findHomography`), shows reprojection error, saves `calibration.json`. Run once per camera mount. |
| `VisionCamera/pose_analyzer.py` | **New.** Scrub the recorded video frame-by-frame with a live preview, capture two frames (A/B). On each, click once for robot position and once more for a heading point. Converts both to field coordinates via the calibration, computes distance/direction/velocity/omega between A and B, and can append results to `results.csv`. |
| `VisionCamera/field_coords.py` | **New.** Core math: `FieldCalibration` (homography wrapper), `Pose2d`, `kinematics_between()`. No GUI dependency — unit tested. |
| `tests/test_VisionCamera.py` | **New.** 8 unit tests covering the homography round-trip, heading computation, angle wrapping, and kinematics. |
| `pyproject.toml` | Added runtime deps (`opencv-python`, `numpy`, `Pillow`) and 4 console-script entry points. |

## Coordinate convention

Meters, WPILib field convention: origin per your calibration points, heading
in radians counter-clockwise from +X. Chosen so results compare directly
against the robot's own odometry/`Pose2d`.

## Usage

```
# 1. Record (run your robot-move command while this is running, 'q' to stop)
python -m VisionCamera.VideoWriter --url http://limelight.local:5800/stream.mjpg

# 2. Calibrate once per camera mount, using a frame that shows >=4 known field points
python -m VisionCamera.calibration_tool reference_frame.jpg -o calibration.json

# 3. Mark two frames from a recording and read off distance/direction/velocity/omega
python -m VisionCamera.pose_analyzer recording_20260919_120000.mp4 -c calibration.json -r results.csv
```

## Open items / caveats

- Default Limelight stream URL (`http://limelight.local:5800/stream.mjpg`) is
  a best guess — confirm against your Limelight 4's actual web UI/firmware
  and override with `--url` if different.
- Frame seeking uses OpenCV's `CAP_PROP_POS_FRAMES`, which is generally
  accurate for our own mp4v recordings but not guaranteed frame-exact for
  all codecs.
- Not yet verified against a real Limelight stream or real recording — the
  math/GUI logic is unit tested, but end-to-end use with actual hardware is
  still pending.
