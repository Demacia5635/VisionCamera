"""Record the overhead camera stream to an MP4, plus periodic JPEG snapshots.

Usage:  python -m VisionCamera.VideoWriter --url <stream-url> [options]

Workflow this supports: start this recorder, run whatever robot command
moves the robot, then press 'q' (or Ctrl+C) to stop and finalize the MP4.
Afterwards, pick two frames from the recording and use pose_analyzer.py to
mark the robot's position/heading and compute ground-truth kinematics.

Stream URL: a Limelight typically serves its camera view over HTTP MJPEG,
e.g. http://<limelight-ip-or-hostname>:5800/stream.mjpg (OpenCV's
VideoCapture handles this like any other stream URL). Some setups instead
expose RTSP -- check your Limelight's web UI / firmware docs for what's
enabled, and pass whichever URL actually works with --url.
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

DEFAULT_URL = "http://limelight-pdh.local:5800/stream.mjpg"


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


def record(url: str, save_dir: str, save_interval: float, jpeg_quality: int, video_file: str):
    os.makedirs(save_dir, exist_ok=True)

    image_q: queue.Queue = queue.Queue(maxsize=20)
    video_q: queue.Queue = queue.Queue(maxsize=300)  # ~10 s of buffer at 30 fps

    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open stream: {url}")

    # Stream properties (many IP streams report fps as 0, so fall back to a default)
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps != fps or fps > 120:
        fps = 25.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    writer = cv2.VideoWriter(video_file, fourcc_code('m', 'p', '4', 'v'), fps, (width, height))
    if not writer.isOpened():
        cap.release()
        raise RuntimeError("Could not open video writer")

    threads = [
        threading.Thread(target=image_worker, args=(image_q, jpeg_quality), daemon=True),
        threading.Thread(target=video_worker, args=(video_q, writer), daemon=True),
    ]
    for t in threads:
        t.start()

    print(f"Recording {url} -> {video_file}  ({width}x{height} @ {fps:g} fps). "
          f"Press 'q' in the preview window (or Ctrl+C) to stop.")

    last_save = 0.0
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Frame grab failed (stream ended or dropped)")
                break

            # every frame -> video
            try:
                video_q.put_nowait(frame)
            except queue.Full:
                print("Video queue full, dropping frame")

            # every save_interval seconds -> jpg
            now = time.monotonic()
            if now - last_save >= save_interval:
                last_save = now
                ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                try:
                    image_q.put_nowait((os.path.join(save_dir, f"frame_{ts}.jpg"), frame.copy()))
                except queue.Full:
                    print("Image queue full, dropping frame")

            cv2.imshow("Overhead camera", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
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
    parser.add_argument("--url", default=DEFAULT_URL, help="Camera stream URL")
    parser.add_argument("--save-dir", default="captures", help="Directory for periodic JPEG snapshots")
    parser.add_argument("--save-interval", type=float, default=0.5, help="Seconds between JPEG snapshots")
    parser.add_argument("--jpeg-quality", type=int, default=90, help="JPEG quality (1-100)")
    parser.add_argument("--out", default=None, help="Output MP4 path (default: recording_<timestamp>.mp4)")
    args = parser.parse_args(argv)

    video_file = args.out or f"recording_{datetime.now():%Y%m%d_%H%M%S}.mp4"
    record(args.url, args.save_dir, args.save_interval, args.jpeg_quality, video_file)


if __name__ == "__main__":
    main(sys.argv[1:])

