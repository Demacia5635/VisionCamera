"""GUI front-end for recording the overhead camera.

Usage:  python -m VisionCamera.recorder_gui [--source 0] [options]

A live-view window with:
  - a Source box (pick a detected camera index or type a device index /
    stream URL), a Resolution preset (640x480 @ 50fps up to 4K @ 50fps --
    requests only, the camera's actual negotiated size/fps is always what's
    used), and "Open" to (re)connect with both,
  - an exposure slider ("Auto exposure" toggles manual control),
  - a capture interval field (seconds between JPEG snapshots while
    recording, same idea as VideoWriter.py's --save-interval),
  - the live camera view scaled to fit the window,
  - a Start/Stop recording button. Each "Start" begins a new
    recording_<timestamp>.mp4 (plus periodic JPEG snapshots into
    --save-dir) -- stop and start again to get a separate file per take,
    without closing the app or losing the live view in between.

This builds on VideoWriter.py's camera-opening and background-writer logic
(open_camera, image_worker, video_worker) rather than duplicating it; use
VideoWriter.py directly instead if you want a plain headless/CLI recorder.
"""
from __future__ import annotations

import argparse
import os
import queue
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import messagebox, ttk

import cv2
from PIL import Image, ImageTk

try:  # python -m VisionCamera.recorder_gui
    from .VideoWriter import DEFAULT_SOURCE, fourcc_code, image_worker, open_camera, parse_source, video_worker
except ImportError:  # run directly, e.g. `python recorder_gui.py`
    from VideoWriter import DEFAULT_SOURCE, fourcc_code, image_worker, open_camera, parse_source, video_worker

# Typical DirectShow/UVC exposure is a log2(seconds) value, e.g. -6 = 1/64 s.
# The actual supported range is camera-specific and not reliably queryable
# through OpenCV, so this is a practical default -- widen it if your camera's
# slider clamps before reaching the exposure you want.
EXPOSURE_MIN = -13.0
EXPOSURE_MAX = -1.0

# DSHOW's CAP_PROP_AUTO_EXPOSURE convention (0.75 = auto, 0.25 = manual) is a
# long-standing OpenCV/DirectShow quirk, not a documented API; some cameras
# ignore it entirely and stay in whatever mode their own driver UI set.
AUTO_EXPOSURE_ON = 0.75
AUTO_EXPOSURE_OFF = 0.25

# Resolution/fps presets offered in the GUI, from VGA up to 4K. These are
# *requests* -- open_camera() always reads back the actual negotiated size
# from a grabbed frame and the actual fps from the camera, so a camera that
# can't do a given preset (e.g. 4K @ 50fps) just falls back to whatever it
# can actually deliver instead of failing.
RESOLUTION_PRESETS = [
    (640, 480, 50),
    (1280, 720, 30),
    (1280, 720, 50),
    (1600, 900, 50),
    (1920, 1080, 50),
    (2560, 1440, 50),
    (3840, 2160, 50),  # 4K
]


def _preset_label(width: int, height: int, fps: float) -> str:
    tag = " (4K)" if (width, height) == (3840, 2160) else ""
    return f"{width}x{height} @ {fps:g}fps{tag}"


class CameraReader(threading.Thread):
    """Continuously grabs frames in the background and optionally fans them
    out to a recording session (video + periodic JPEG snapshots)."""

    def __init__(self, source, width=None, height=None, cam_fps=None):
        super().__init__(daemon=True)
        self.cap, self.frame_width, self.frame_height, self.fps, first_frame = open_camera(
            source, width, height, cam_fps)

        self._frame_lock = threading.Lock()
        self._latest = first_frame

        self._rec_lock = threading.Lock()
        self._video_q: queue.Queue | None = None
        self._image_q: queue.Queue | None = None
        self._save_dir = "captures"
        self._save_interval = 1.0
        self._last_save = 0.0
        self.frames_written = 0

        self._running = True

    def run(self):
        while self._running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self._frame_lock:
                self._latest = frame

            with self._rec_lock:
                video_q, image_q, save_dir, save_interval = (
                    self._video_q, self._image_q, self._save_dir, self._save_interval)
            if video_q is None:
                continue

            try:
                video_q.put_nowait(frame)
                self.frames_written += 1
            except queue.Full:
                pass

            now = time.monotonic()
            if now - self._last_save >= save_interval:
                self._last_save = now
                ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                try:
                        assert image_q is not None
                        image_q.put_nowait((os.path.join(save_dir, f"frame_{ts}.jpg"), frame.copy()))
                except queue.Full:
                    pass

    def latest_frame(self):
        with self._frame_lock:
            return None if self._latest is None else self._latest.copy()

    def start_recording(self, video_q: queue.Queue, image_q: queue.Queue, save_dir: str, save_interval: float):
        with self._rec_lock:
            self._video_q, self._image_q = video_q, image_q
            self._save_dir, self._save_interval = save_dir, save_interval
            self._last_save = time.monotonic()
            self.frames_written = 0

    def stop_recording(self):
        with self._rec_lock:
            self._video_q, self._image_q = None, None

    def set_save_interval(self, save_interval: float):
        with self._rec_lock:
            self._save_interval = save_interval

    def set_exposure(self, auto: bool, value: float):
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, AUTO_EXPOSURE_ON if auto else AUTO_EXPOSURE_OFF)
        if not auto:
            self.cap.set(cv2.CAP_PROP_EXPOSURE, value)

    def stop(self):
        self._running = False
        self.join(timeout=2)
        self.cap.release()


def probe_sources(max_index: int = 5) -> list[str]:
    found = []
    for i in range(max_index + 1):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW) if sys.platform == "win32" else cv2.VideoCapture(i)
        if cap.isOpened():
            found.append(str(i))
        cap.release()
    return found


class RecorderGUI(tk.Tk):
    def __init__(self, source, save_dir: str, save_interval: float, jpeg_quality: int,
                 width=None, height=None, cam_fps=None):
        super().__init__()
        self.title("VisionCamera recorder")
        self.geometry("1100x800")

        self.save_dir = save_dir
        self.jpeg_quality = jpeg_quality
        self.reader: CameraReader | None = None
        self.recording = False
        self.rec_threads: list[threading.Thread] = []
        self.rec_video_q: queue.Queue | None = None
        self.rec_image_q: queue.Queue | None = None
        self.rec_file: str | None = None
        self._photo = None

        self._build_ui()
        self.interval_var.set(str(save_interval))
        self.source_var.set(str(source))

        default_width, default_height, default_fps = RESOLUTION_PRESETS[0]
        width = default_width if width is None else width
        height = default_height if height is None else height
        cam_fps = default_fps if cam_fps is None else cam_fps
        label = _preset_label(width, height, cam_fps)
        if label not in self._resolution_by_label:
            self._resolution_by_label[label] = (width, height, cam_fps)
            self.resolution_combo["values"] = (label, *self.resolution_combo["values"])
        self.resolution_var.set(label)

        self._open(parse_source(source), width, height, cam_fps)

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._tick()

    # ---------- UI ----------
    def _build_ui(self):
        top = ttk.Frame(self, padding=6)
        top.pack(fill="x")

        ttk.Label(top, text="Source:").pack(side="left")
        self.source_var = tk.StringVar(value=str(DEFAULT_SOURCE))
        self.source_combo = ttk.Combobox(top, textvariable=self.source_var,
                                          values=probe_sources() or [str(DEFAULT_SOURCE)], width=22)
        self.source_combo.pack(side="left", padx=(4, 4))

        ttk.Label(top, text="Resolution:").pack(side="left", padx=(8, 0))
        self.resolution_var = tk.StringVar(value="")
        self._resolution_by_label = {_preset_label(w, h, f): (w, h, f) for w, h, f in RESOLUTION_PRESETS}
        self.resolution_combo = ttk.Combobox(top, textvariable=self.resolution_var,
                                              values=list(self._resolution_by_label), state="readonly", width=18)
        self.resolution_combo.pack(side="left", padx=(4, 0))

        ttk.Button(top, text="Open", command=self._on_open_clicked).pack(side="left", padx=(8, 0))

        ttk.Label(top, text="   Capture interval (s):").pack(side="left")
        self.interval_var = tk.StringVar(value="0.5")
        interval_box = ttk.Spinbox(top, from_=0.1, to=60, increment=0.1, textvariable=self.interval_var, width=6,
                                    command=self._on_interval_changed)
        interval_box.pack(side="left")
        interval_box.bind("<Return>", lambda e: self._on_interval_changed())
        interval_box.bind("<FocusOut>", lambda e: self._on_interval_changed())

        exp = ttk.Frame(self, padding=(6, 0))
        exp.pack(fill="x")
        self.auto_exposure = tk.BooleanVar(value=True)
        ttk.Checkbutton(exp, text="Auto exposure", variable=self.auto_exposure,
                         command=self._on_exposure_changed).pack(side="left")
        ttk.Label(exp, text="Exposure:").pack(side="left", padx=(12, 4))
        self.exposure_var = tk.DoubleVar(value=(EXPOSURE_MIN + EXPOSURE_MAX) / 2)
        self.exposure_scale = ttk.Scale(exp, from_=EXPOSURE_MIN, to=EXPOSURE_MAX, variable=self.exposure_var,
                                         command=lambda v: self._on_exposure_changed())
        self.exposure_scale.pack(side="left", fill="x", expand=True, padx=4)
        self.exposure_label = ttk.Label(exp, text="", width=6)
        self.exposure_label.pack(side="left")

        self.canvas = tk.Canvas(self, bg="#202020", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=6, pady=6)

        bottom = ttk.Frame(self, padding=6)
        bottom.pack(fill="x")
        self.record_button = ttk.Button(bottom, text="Start recording", command=self._toggle_recording)
        self.record_button.pack(side="left")
        self.status_label = ttk.Label(bottom, text="Not connected")
        self.status_label.pack(side="left", padx=(12, 0))

        self._on_exposure_changed()  # sync label/slider enabled-state with initial auto=True

    # ---------- camera open/close ----------
    def _on_open_clicked(self):
        width, height, cam_fps = self._resolution_by_label[self.resolution_var.get()]
        self._open(parse_source(self.source_var.get()), width, height, cam_fps)

    def _open(self, source, width=None, height=None, cam_fps=None):
        if self.recording:
            self._stop_recording()
        if self.reader is not None:
            self.reader.stop()
            self.reader = None

        try:
            reader = CameraReader(source, width, height, cam_fps)
        except RuntimeError as e:
            messagebox.showerror("Could not open camera", str(e))
            self.status_label.config(text="Not connected")
            return
        reader.start()
        self.reader = reader
        self._on_exposure_changed()
        self.status_label.config(
            text=f"source={source}  {reader.frame_width}x{reader.frame_height} @ {reader.fps:g} fps")

    # ---------- exposure / interval ----------
    def _on_exposure_changed(self):
        auto = self.auto_exposure.get()
        value = self.exposure_var.get()
        self.exposure_label.config(text=f"{value:.1f}")
        self.exposure_scale.state(["disabled"] if auto else ["!disabled"])
        if self.reader is not None:
            self.reader.set_exposure(auto, value)

    def _on_interval_changed(self):
        try:
            interval = max(0.05, float(self.interval_var.get()))
        except ValueError:
            return
        if self.reader is not None:
            self.reader.set_save_interval(interval)

    # ---------- recording ----------
    def _toggle_recording(self):
        self._stop_recording() if self.recording else self._start_recording()

    def _start_recording(self):
        if self.reader is None:
            messagebox.showerror("No camera", "Open a camera source first.")
            return
        os.makedirs(self.save_dir, exist_ok=True)
        self.rec_file = f"recording_{datetime.now():%Y%m%d_%H%M%S}.mp4"
        writer = cv2.VideoWriter(self.rec_file, fourcc_code('m', 'p', '4', 'v'), self.reader.fps,
                                  (self.reader.frame_width, self.reader.frame_height))
        if not writer.isOpened():
            messagebox.showerror("Recording failed", f"Could not open video writer for {self.rec_file}")
            return

        self.rec_video_q = queue.Queue(maxsize=500)
        self.rec_image_q = queue.Queue(maxsize=20)
        self.rec_threads = [
            threading.Thread(target=image_worker, args=(self.rec_image_q, self.jpeg_quality), daemon=True),
            threading.Thread(target=video_worker, args=(self.rec_video_q, writer), daemon=True),
        ]
        for t in self.rec_threads:
            t.start()

        try:
            interval = max(0.05, float(self.interval_var.get()))
        except ValueError:
            interval = 0.5
        self.reader.start_recording(self.rec_video_q, self.rec_image_q, self.save_dir, interval)
        self.recording = True
        self.record_button.config(text="Stop recording")

    def _stop_recording(self):
        if not self.recording:
            return
        assert self.reader is not None
        self.reader.stop_recording()
        assert self.rec_video_q is not None
        self.rec_video_q.put(None)
        assert self.rec_image_q is not None
        self.rec_image_q.put(None)
        for t in self.rec_threads:
            t.join()
        self.recording = False
        self.record_button.config(text="Start recording")
        self.status_label.config(text=f"Saved {self.rec_file}  ({self.reader.frames_written} frames)")
        self.rec_threads = []
        self.rec_video_q = self.rec_image_q = None

    # ---------- live view ----------
    def _tick(self):
        if self.reader is not None:
            frame = self.reader.latest_frame()
            if frame is not None:
                self._draw(frame)
            if self.recording:
                self.status_label.config(
                    text=f"Recording -> {self.rec_file}  ({self.reader.frames_written} frames)")
        self.after(33, self._tick)

    def _draw(self, frame_bgr):
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        if cw < 2 or ch < 2:
            return
        img = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        scale = min(cw / img.width, ch / img.height)
        size = (max(1, int(img.width * scale)), max(1, int(img.height * scale)))
        self._photo = ImageTk.PhotoImage(img.resize(size, Image.Resampling.LANCZOS))
        self.canvas.delete("all")
        self.canvas.create_image(cw // 2, ch // 2, image=self._photo, anchor="center")

    def _on_close(self):
        if self.recording:
            self._stop_recording()
        if self.reader is not None:
            self.reader.stop()
        self.destroy()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", default=str(DEFAULT_SOURCE),
                         help="USB camera device index (0, 1, ...) or a stream URL to open at startup")
    parser.add_argument("--width", type=int, default=None, help="Requested capture width (px)")
    parser.add_argument("--height", type=int, default=None, help="Requested capture height (px)")
    parser.add_argument("--cam-fps", type=float, default=None, help="Requested capture frame rate")
    parser.add_argument("--save-dir", default="captures", help="Directory for periodic JPEG snapshots")
    parser.add_argument("--save-interval", type=float, default=0.5, help="Initial seconds between JPEG snapshots")
    parser.add_argument("--jpeg-quality", type=int, default=90, help="JPEG quality (1-100)")
    args = parser.parse_args(argv)

    RecorderGUI(args.source, args.save_dir, args.save_interval, args.jpeg_quality,
                width=args.width, height=args.height, cam_fps=args.cam_fps).mainloop()


if __name__ == "__main__":
    main(sys.argv[1:])
