"""Record the overhead camera to an MP4, plus periodic JPEG snapshots.

Usage:  python -m VisionCamera.VideoWriter [--source 0] [options]

Workflow this supports: start this recorder, run whatever robot command
moves the robot, then press 'q' (or Ctrl+C) to stop and finalize the MP4.
Afterwards, pick two frames from the recording and use pose_analyzer.py to
mark the robot's position/heading and compute ground-truth kinematics.

Source: by default this opens a local USB camera by device index (0 is
usually the first camera Windows sees; try 1, 2, ... if that's the wrong
one, or check "Camera" in Device Manager). --source also accepts a network
stream URL (e.g. http://<host>:5800/stream.mjpg) if you're capturing over
the network instead of USB. USB webcams commonly default to a low
resolution/fps until requested otherwise -- use --width/--height/--cam-fps
to ask the camera for something better; if it doesn't support the exact
values, the driver picks the closest it can do.
"""
from __future__ import annotations

import argparse
import os
import queue
import sys
import threading
import time
from datetime import datetime

import cv2

DEFAULT_SOURCE = 0  # first USB camera


def parse_source(value: str):
    """--source is a USB device index (int) if it parses as one, else a URL string."""
    try:
        return int(value)
    except ValueError:
        return value


def fourcc_code(c1, c2, c3, c4):
    return (ord(c1) & 255) | ((ord(c2) & 255) << 8) | \
           ((ord(c3) & 255) << 16) | ((ord(c4) & 255) << 24)


def image_worker(image_q: queue.Queue, jpeg_quality: int):
    while True:
        item = image_q.get()
        if item is None:
            break
        path, frame = item
        cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])


def video_worker(video_q: queue.Queue, writer: cv2.VideoWriter):
    while True:
        frame = video_q.get()
        if frame is None:
            break
        writer.write(frame)
    writer.release()  # finalizes the MP4 so it's playable


def open_camera(source, width: int | None = None, height: int | None = None, cam_fps: float | None = None):
    """Open a camera/stream and return (cap, frame_width, frame_height, fps, first_frame).

    Grabs one frame immediately: both because many USB cameras/DirectShow
    drivers report width/height as 0 until a frame has actually been read
    (a 0x0 size is why VideoWriter.isOpened() would otherwise come back
    False), and so callers don't have to special-case "the first frame was
    already consumed" -- it's returned here to use or discard.
    """
    # DSHOW opens USB cameras faster and more reliably than the default MSMF
    # backend on Windows; it doesn't apply to (and is ignored for) URL sources.
    if isinstance(source, int) and sys.platform == "win32":
        cap = cv2.VideoCapture(source, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera/stream: {source}")

    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if cam_fps:
        cap.set(cv2.CAP_PROP_FPS, cam_fps)

    ok, first_frame = cap.read()
    if not ok:
        cap.release()
        raise RuntimeError(f"Could not grab a frame from source: {source}")
    frame_height, frame_width = first_frame.shape[:2]

    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps != fps or fps > 120:
        fps = 25.0

    return cap, frame_width, frame_height, fps, first_frame


def record(source, save_dir: str, save_interval: float, jpeg_quality: int, video_file: str,
           width: int | None = None, height: int | None = None, cam_fps: float | None = None):
    os.makedirs(save_dir, exist_ok=True)

    image_q: queue.Queue = queue.Queue(maxsize=20)
    video_q: queue.Queue = queue.Queue(maxsize=500)  # ~10 s of buffer at 50 fps

    cap, frame_width, frame_height, fps, first_frame = open_camera(source, width, height, cam_fps)

    writer = cv2.VideoWriter(video_file, fourcc_code('m', 'p', '4', 'v'), fps, (frame_width, frame_height))
    if not writer.isOpened():
        cap.release()
        raise RuntimeError(
            f"Could not open video writer for {video_file} ({frame_width}x{frame_height} @ {fps:g} fps)")

    threads = [
        threading.Thread(target=image_worker, args=(image_q, jpeg_quality), daemon=True),
        threading.Thread(target=video_worker, args=(video_q, writer), daemon=True),
    ]
    for t in threads:
        t.start()

    print(f"Recording source={source} -> {video_file}  ({frame_width}x{frame_height} @ {fps:g} fps). "
          f"Press 'q' in the preview window (or Ctrl+C) to stop.")

    last_save = 0.0
    frame = first_frame  # the frame already grabbed above to size the writer
    ret = True
    nframes = 0
    try:
        while True:
            if not ret:
                print("Frame grab failed (stream ended or dropped)")
                break

            # every frame -> video
            try:
                video_q.put_nowait(frame)
                nframes = nframes+1
            except queue.Full:
                print("Video queue full, dropping frame")

            # every save_interval seconds -> jpg
            now = time.monotonic()
            if now - last_save >= save_interval:
                interval = now - last_save
                print(f'{nframes} - {nframes/interval}fps')
                nframes = 0
                last_save = now
                ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                try:
                    image_q.put_nowait((os.path.join(save_dir, f"frame_{ts}.jpg"), frame.copy()))
                except queue.Full:
                    print("Image queue full, dropping frame")

            cv2.imshow("Overhead camera", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            ret, frame = cap.read()
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        image_q.put(None)
        video_q.put(None)
        for t in threads:
            t.join()  # flushes queues and closes the MP4
        print(f"Saved {video_file}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", default=str(DEFAULT_SOURCE),
                         help="USB camera device index (0, 1, ...) or a stream URL")
    parser.add_argument("--width", type=int, default=None, help="Requested capture width (px)")
    parser.add_argument("--height", type=int, default=None, help="Requested capture height (px)")
    parser.add_argument("--cam-fps", type=float, default=None, help="Requested capture frame rate")
    parser.add_argument("--save-dir", default="captures", help="Directory for periodic JPEG snapshots")
    parser.add_argument("--save-interval", type=float, default=5, help="Seconds between JPEG snapshots")
    parser.add_argument("--jpeg-quality", type=int, default=90, help="JPEG quality (1-100)")
    parser.add_argument("--out", default=None, help="Output MP4 path (default: recording_<timestamp>.mp4)")
    args = parser.parse_args(argv)

    video_file = args.out or f"recording_{datetime.now():%Y%m%d_%H%M%S}.mp4"
    record(parse_source(args.source), args.save_dir, args.save_interval, args.jpeg_quality, video_file,
           width=args.width, height=args.height, cam_fps=args.cam_fps)


if __name__ == "__main__":
    main(sys.argv[1:])
