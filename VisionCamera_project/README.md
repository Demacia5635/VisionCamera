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
| `VisionCamera/__main__.py` | **New.** `python -m VisionCamera` opens a launcher window listing every tool below (one button each, launched as its own process) plus a one-click checkerboard generator. *(existing file, was a placeholder)* |
| `VisionCamera/VideoWriter.py` | Headless/CLI recorder: camera stream to MP4 + periodic JPEG snapshots. `open_camera()` (shared camera-opening logic) extracted for reuse by `recorder_gui.py`. *(existing file, refactored)* |
| `VisionCamera/recorder_gui.py` | **New.** GUI recorder: source picker (detected USB indices or a typed device/URL), auto/manual exposure slider, capture-interval field, live view scaled to the window, and a Start/Stop recording button -- each Start begins a fresh `recording_<timestamp>.mp4` without restarting the app. Built on top of `VideoWriter.py`'s `open_camera`/`image_worker`/`video_worker`. |
| `VisionCamera/viewCaptures.py` | Browses/blends periodic snapshots to eyeball motion. Added a `main()` entry point. *(existing file, minor update)* |
| `VisionCamera/calibration_tool.py` | **New.** Click ≥4 known field points on a reference frame, enter each one's real field X/Y (meters); fits a pixel→field homography (`cv2.findHomography`), shows reprojection error, saves `calibration.json`. Run once per camera mount. Optionally undistorts the reference image first via `--lens-calibration`. The reference image is now optional on the command line -- omit it (or use "Open image..." to switch mid-session) and a file picker opens instead. |
| `VisionCamera/pose_analyzer.py` | **New.** Scrub the recorded video frame-by-frame with a live preview, capture two frames (A/B). On each, click once for robot position and once more for a heading point. Converts both to field coordinates via the calibration, computes distance/direction/velocity/omega between A and B, and can append results to `results.csv`. Optionally undistorts every frame first via `--lens-calibration`. The video is now optional on the command line -- omit it (or use "Open video..." to switch mid-session) and a file picker opens instead. |
| `VisionCamera/lens_calib.py` | **New.** Checkerboard-based fisheye intrinsic calibration (`cv2.fisheye.calibrate`) for a wide-FOV/fisheye USB camera. Live-captures checkerboard samples from the camera, saves `lens_calibration.json`. One-time step per camera. |
| `VisionCamera/checkerboard.py` | **New.** Generates a print-ready checkerboard PNG (sized in mm, at 300 dpi) matching `lens_calib.py`'s `--cols`/`--rows`, so there's no need to source one externally. Verified OpenCV's `findChessboardCorners` detects it. |
| `VisionCamera/undistort.py` | **New.** `LensCalibration`: loads `lens_calibration.json`, builds/caches `cv2.fisheye` remap tables, and undistorts frames. Used by `calibration_tool.py` and `pose_analyzer.py`. |
| `VisionCamera/field_coords.py` | **New.** Core math: `FieldCalibration` (homography wrapper), `Pose2d`, `kinematics_between()`. No GUI dependency — unit tested. |
| `tests/test_VisionCamera.py` | **New.** 10 unit tests covering the homography round-trip, heading computation, angle wrapping, kinematics, and lens undistortion. |
| `pyproject.toml` | Added runtime deps (`opencv-python`, `numpy`, `Pillow`) and 7 console-script entry points. |

## Coordinate convention

Meters, WPILib field convention: origin per your calibration points, heading
in radians counter-clockwise from +X. Chosen so results compare directly
against the robot's own odometry/`Pose2d`.

## Usage

```
# Launcher: a window listing every tool below as a button (plus a one-click
# checkerboard generator); each button starts that tool as its own process.
python -m VisionCamera

# 0. Lens calibration (one-time per camera; required for a fisheye/wide-FOV
#    camera like a 150 deg USB cam -- see "Fisheye / wide-FOV lens" below).
#    --cols/--rows are interior corners of your checkerboard.
python -m VisionCamera.lens_calib --source 0 --cols 9 --rows 6 -o lens_calibration.json

# 1. Record (run your robot-move command while this is running, 'q' to stop)
python -m VisionCamera.VideoWriter --source 0
# --source also takes a stream URL (e.g. http://limelight.local:5800/stream.mjpg)
# instead of a USB device index, and --width/--height/--cam-fps request a
# capture resolution/rate from the camera if the default is too low.
#
# Or use the GUI version -- source picker, exposure slider, scaled live
# view, Start/Stop recording (each Start = a new recording_<timestamp>.mp4):
python -m VisionCamera.recorder_gui --source 0

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
- `lens_calib.py` had picked up the same absolute-import regression as
  `calibration_tool.py` before it (`from VideoWriter import ...` / `from
  undistort import ...`). Separately, its switch from `cv2.fisheye.CALIB_*`
  flag constants to the plain `cv2.CALIB_*` ones was a real, correct fix on
  your part: this OpenCV build (5.0.0) doesn't expose those flags under
  `cv2.fisheye` at all, so the original code would have raised
  `AttributeError` the first time `calibrate()` ran.
- **Why the import kept flip-flopping:** `calibration_tool.py` had its
  sibling imports changed from relative back to absolute twice, and
  `lens_calib.py` once — each time because absolute imports (`from
  field_coords import ...`) are what work when a file is run directly
  (e.g. an IDE's "Run Python File"), while relative imports (`from
  .field_coords import ...`) are what work under `python -m
  VisionCamera.x` / the installed console scripts. Rather than keep
  reverting each other, `calibration_tool.py`, `lens_calib.py`,
  `pose_analyzer.py`, and `recorder_gui.py` now all use a `try: from
  .sibling import X / except ImportError: from sibling import X` fallback,
  verified to work both ways — run them however's convenient.
- `VideoWriter.py`'s `record()` had picked up a hardcoded override
  (`source = 2; width = 1440; height = 810; cam_fps = 50`) that silently
  ignored whatever `--source`/`--width`/`--height`/`--cam-fps` were passed
  in — removed while extracting `open_camera()` for reuse by
  `recorder_gui.py`, since that override would otherwise have propagated
  into the GUI too.
- `recorder_gui.py`'s exposure slider range (-13 to -1, a log2(seconds)
  DirectShow convention) and the auto/manual `CAP_PROP_AUTO_EXPOSURE`
  values (0.75/0.25) are a commonly-used but informally-documented
  OpenCV/DirectShow convention, not something reliably queryable per
  camera — if the slider does nothing or clamps before a usable exposure,
  the actual range/convention for your specific camera may differ.
- The recording pipeline (`CameraReader` start/stop, separate files per
  session, periodic snapshots) was smoke-tested with a paced fake camera
  source standing in for real hardware, but not against an actual USB
  camera or its exposure control yet.
- `calibration_tool.py`'s `--lens-calibration` default had changed (outside
  this session) from `None` to `"lens_calibration.json"` -- a reasonable
  auto-pick-up default, kept as-is -- but loading it was unguarded, so the
  tool would crash on startup for anyone who hasn't run `lens_calib.py` yet
  and has no such file in their working directory. Made the load tolerant:
  a missing/invalid file now shows a warning and continues without fisheye
  correction instead of crashing.
- `__main__.py` launches each tool via `subprocess.Popen([sys.executable,
  "-m", "VisionCamera.<tool>"])`, non-blocking, so multiple tools can run
  at once (e.g. recording while reviewing an older clip). It only confirmed
  each tool's process *starts*; it doesn't capture or surface errors a tool
  raises after that point (those show up in that tool's own window/console,
  same as running it directly would).
- `pose_analyzer.py` *does* apply lens undistortion (`goto_frame()` already
  called `self.lens_calib.undistort(frame)` before anything else saw the
  frame) -- but unlike `calibration_tool.py`, its `--lens-calibration`
  defaulted to `None` instead of auto-picking-up `lens_calibration.json`,
  so it silently skipped correction unless you remembered to pass the flag
  every time. That's a real mismatch: `calibration.json` is built from an
  undistorted reference image, so marks on un-corrected frames are
  systematically off. Fixed to match `calibration_tool.py`'s default and
  graceful-missing-file warning.
