"""Mark robot pose on two video frames and compute ground-truth kinematics.

Usage:  python -m VisionCamera.pose_analyzer <recording.mp4> [-c calibration.json] [-r results.csv]

Workflow:
  1. Scrub the recording with the slider / step buttons and click
     "Use as frame A" at the moment just before the robot moves, then scrub
     forward and click "Use as frame B" at the moment just after it stops.
  2. On each frame panel, click once on the robot's reference point
     (e.g. bumper/frame center) -- that's its position -- then click a
     second time on a point further along the direction the robot is
     facing (e.g. the front of the robot) -- that sets its heading. Click
     again to redo both marks for that frame.
  3. Once both frames are marked, the field-frame pose of each and the
     distance / direction / velocity / omega between them are shown below,
     using calibration.json (see calibration_tool.py) to convert pixels to
     field coordinates (meters, WPILib convention).
  4. "Save result" appends a row to the results CSV for later comparison
     against the robot's own odometry.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

import cv2
from PIL import Image, ImageTk

from .field_coords import FieldCalibration, kinematics_between, pose_from_marks
from .undistort import LensCalibration

MARK_RADIUS = 5
RESULTS_FIELDS = [
    "saved_at", "video", "frame_a_index", "frame_a_t", "frame_b_index", "frame_b_t",
    "dt_s", "pose_a_x_m", "pose_a_y_m", "pose_a_heading_deg",
    "pose_b_x_m", "pose_b_y_m", "pose_b_heading_deg",
    "distance_m", "direction_deg", "velocity_mps", "omega_deg_s",
]


class FramePanel(ttk.Frame):
    """A canvas showing one captured frame, with click-to-mark position+heading."""

    def __init__(self, parent, label: str, on_change):
        super().__init__(parent, padding=4)
        self.label = label
        self.on_change = on_change

        ttk.Label(self, text=label, font=("Segoe UI", 10, "bold")).pack(anchor="w")
        self.info = ttk.Label(self, text="(not set)")
        self.info.pack(anchor="w")

        self.canvas = tk.Canvas(self, bg="#202020", highlightthickness=0, height=360)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<Configure>", lambda e: self.redraw())

        self.frame_index: int | None = None
        self.t: float | None = None
        self.image: Image.Image | None = None
        self.photo = None
        self.scale = 1.0
        self.offset = (0, 0)
        self.position_px: tuple[float, float] | None = None
        self.heading_px: tuple[float, float] | None = None

    def set_frame(self, frame_bgr, frame_index: int, t: float):
        self.image = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        self.frame_index = frame_index
        self.t = t
        self.position_px = None
        self.heading_px = None
        self.info.config(text=f"frame #{frame_index}   t={t:.3f} s")
        self.redraw()
        self.on_change()

    @property
    def marked(self) -> bool:
        return self.position_px is not None and self.heading_px is not None

    def _on_click(self, event):
        if self.image is None:
            return
        ix, iy = self._canvas_to_image(event.x, event.y)
        if not (0 <= ix < self.image.width and 0 <= iy < self.image.height):
            return
        if self.position_px is None:
            self.position_px = (ix, iy)
        elif self.heading_px is None:
            self.heading_px = (ix, iy)
        else:
            self.position_px = (ix, iy)
            self.heading_px = None
        self.redraw()
        self.on_change()

    def _canvas_to_image(self, cx, cy):
        ox, oy = self.offset
        return (cx - ox) / self.scale, (cy - oy) / self.scale

    def _image_to_canvas(self, ix, iy):
        ox, oy = self.offset
        return ix * self.scale + ox, iy * self.scale + oy

    def redraw(self):
        self.canvas.delete("all")
        if self.image is None:
            return
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        if cw < 2 or ch < 2:
            return
        self.scale = min(cw / self.image.width, ch / self.image.height)
        size = (max(1, int(self.image.width * self.scale)), max(1, int(self.image.height * self.scale)))
        ox, oy = (cw - size[0]) // 2, (ch - size[1]) // 2
        self.offset = (ox, oy)
        self.photo = ImageTk.PhotoImage(self.image.resize(size, Image.Resampling.LANCZOS))
        self.canvas.create_image(ox, oy, image=self.photo, anchor="nw")

        if self.position_px:
            cx, cy = self._image_to_canvas(*self.position_px)
            self.canvas.create_oval(cx - MARK_RADIUS, cy - MARK_RADIUS, cx + MARK_RADIUS, cy + MARK_RADIUS,
                                     outline="#00ff88", width=2)
        if self.position_px and self.heading_px:
            ca = self._image_to_canvas(*self.position_px)
            cb = self._image_to_canvas(*self.heading_px)
            self.canvas.create_line(*ca, *cb, fill="#ffaa00", width=2, arrow=tk.LAST)


class PoseAnalyzer(tk.Tk):
    def __init__(self, video_path: str, calib_path: str, results_path: str, lens_calib_path: str | None = None):
        super().__init__()
        self.title(f"Pose analyzer - {video_path}")
        self.geometry("1300x900")
        self.video_path = video_path
        self.results_path = results_path
        self.lens_calib = LensCalibration.load(lens_calib_path) if lens_calib_path else None

        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open video: {video_path}")
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

        self.calib = self._load_calibration(calib_path)
        self._last_pose_a = self._last_pose_b = self._last_kinematics = None

        self._build_ui()
        self.goto_frame(0)

    # ---------- calibration ----------
    def _load_calibration(self, calib_path: str) -> FieldCalibration | None:
        if os.path.isfile(calib_path):
            return FieldCalibration.load(calib_path)
        messagebox.showwarning(
            "No calibration found",
            f"Could not find {calib_path}.\nPick a calibration.json (see calibration_tool.py), "
            "or cancel to view pixel coordinates only.",
        )
        path = filedialog.askopenfilename(title="Select calibration.json", filetypes=[("JSON", "*.json")])
        if path:
            return FieldCalibration.load(path)
        return None

    # ---------- UI ----------
    def _build_ui(self):
        scrub = ttk.Frame(self, padding=6)
        scrub.pack(fill="x")

        ttk.Button(scrub, text="|<", width=3, command=lambda: self.goto_frame(0)).pack(side="left")
        ttk.Button(scrub, text="-10", width=4, command=lambda: self.step(-10)).pack(side="left")
        ttk.Button(scrub, text="-1", width=3, command=lambda: self.step(-1)).pack(side="left")

        self.slider_var = tk.IntVar(value=0)
        self.slider = ttk.Scale(scrub, from_=0, to=max(0, self.frame_count - 1), orient="horizontal",
                                 variable=self.slider_var, command=self._on_slider)
        self.slider.pack(side="left", fill="x", expand=True, padx=6)

        ttk.Button(scrub, text="+1", width=3, command=lambda: self.step(1)).pack(side="left")
        ttk.Button(scrub, text="+10", width=4, command=lambda: self.step(10)).pack(side="left")
        ttk.Button(scrub, text=">|", width=3, command=lambda: self.goto_frame(self.frame_count - 1)).pack(side="left")

        self.frame_label = ttk.Label(scrub, text="", width=26)
        self.frame_label.pack(side="left", padx=(10, 0))

        capture = ttk.Frame(self, padding=(6, 0))
        capture.pack(fill="x")
        ttk.Button(capture, text="Use as frame A", command=lambda: self.capture_into("A")).pack(side="left")
        ttk.Button(capture, text="Use as frame B", command=lambda: self.capture_into("B")).pack(side="left", padx=6)
        ttk.Label(capture, text="  Click a frame panel: 1st click = position, 2nd click = heading, "
                                 "3rd click = redo.").pack(side="left")

        preview_frame = ttk.Frame(self, padding=(6, 0))
        preview_frame.pack(fill="x")
        ttk.Label(preview_frame, text="Live preview (scrub above, then capture into A/B):").pack(anchor="w")
        self.preview_canvas = tk.Canvas(preview_frame, bg="#202020", highlightthickness=0, height=180)
        self.preview_canvas.pack(fill="x")
        self.preview_canvas.bind("<Configure>", lambda e: self._draw_preview())
        self._preview_photo = None

        panels = ttk.Frame(self, padding=6)
        panels.pack(fill="both", expand=True)
        self.panel_a = FramePanel(panels, "Frame A", self.update_results)
        self.panel_a.pack(side="left", fill="both", expand=True)
        self.panel_b = FramePanel(panels, "Frame B", self.update_results)
        self.panel_b.pack(side="left", fill="both", expand=True)

        results = ttk.Frame(self, padding=6)
        results.pack(fill="x")
        self.results_text = tk.Text(results, height=6, font=("Consolas", 10), state="disabled")
        self.results_text.pack(fill="x")
        ttk.Button(results, text=f"Save result -> {self.results_path}", command=self.save_result).pack(
            anchor="e", pady=(4, 0))

    # ---------- scrubbing ----------
    def _on_slider(self, _value):
        self.goto_frame(int(round(self.slider_var.get())), from_slider=True)

    def step(self, delta):
        self.goto_frame(int(self.slider_var.get()) + delta)

    def goto_frame(self, index: int, from_slider: bool = False):
        index = max(0, min(self.frame_count - 1, index))
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, frame = self.cap.read()
        if not ok:
            return
        if self.lens_calib is not None:
            frame = self.lens_calib.undistort(frame)
        self._current_frame = frame
        self._current_index = index
        t_ms = self.cap.get(cv2.CAP_PROP_POS_MSEC)
        self._current_t = t_ms / 1000.0 if t_ms else index / self.fps
        if not from_slider:
            self.slider_var.set(index)
        self.frame_label.config(text=f"frame {index}/{self.frame_count - 1}   t={self._current_t:.3f} s")
        self._preview_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        self._draw_preview()

    def _draw_preview(self):
        if getattr(self, "_preview_image", None) is None:
            return
        cw, ch = self.preview_canvas.winfo_width(), self.preview_canvas.winfo_height()
        if cw < 2 or ch < 2:
            return
        img = self._preview_image
        scale = min(cw / img.width, ch / img.height)
        size = (max(1, int(img.width * scale)), max(1, int(img.height * scale)))
        self._preview_photo = ImageTk.PhotoImage(img.resize(size, Image.Resampling.LANCZOS))
        self.preview_canvas.delete("all")
        self.preview_canvas.create_image(cw // 2, ch // 2, image=self._preview_photo, anchor="center")

    def capture_into(self, which: str):
        panel = self.panel_a if which == "A" else self.panel_b
        panel.set_frame(self._current_frame, self._current_index, self._current_t)

    # ---------- results ----------
    def update_results(self):
        text = []
        pose_a = pose_b = None
        if self.panel_a.marked and self.calib is not None:
            pose_a = pose_from_marks(self.calib, self.panel_a.position_px, self.panel_a.heading_px)
            text.append(f"Frame A  t={self.panel_a.t:.3f}s   {pose_a}")
        elif self.panel_a.marked:
            text.append(f"Frame A  t={self.panel_a.t:.3f}s   (no calibration: pixel pos={self.panel_a.position_px})")

        if self.panel_b.marked and self.calib is not None:
            pose_b = pose_from_marks(self.calib, self.panel_b.position_px, self.panel_b.heading_px)
            text.append(f"Frame B  t={self.panel_b.t:.3f}s   {pose_b}")
        elif self.panel_b.marked:
            text.append(f"Frame B  t={self.panel_b.t:.3f}s   (no calibration: pixel pos={self.panel_b.position_px})")

        self._last_pose_a = pose_a
        self._last_pose_b = pose_b

        if pose_a is not None and pose_b is not None:
            k = kinematics_between(pose_a, self.panel_a.t, pose_b, self.panel_b.t)
            text.append("")
            text.append(str(k))
            self._last_kinematics = k
        else:
            self._last_kinematics = None

        self.results_text.config(state="normal")
        self.results_text.delete("1.0", "end")
        self.results_text.insert("1.0", "\n".join(text) if text else "Mark position + heading on both frames.")
        self.results_text.config(state="disabled")

    def save_result(self):
        if self._last_pose_a is None or self._last_pose_b is None or self._last_kinematics is None:
            messagebox.showerror("Nothing to save", "Mark position + heading on both frames first.")
            return
        pa, pb, k = self._last_pose_a, self._last_pose_b, self._last_kinematics
        row = {
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "video": self.video_path,
            "frame_a_index": self.panel_a.frame_index, "frame_a_t": self.panel_a.t,
            "frame_b_index": self.panel_b.frame_index, "frame_b_t": self.panel_b.t,
            "dt_s": k.dt,
            "pose_a_x_m": pa.x, "pose_a_y_m": pa.y, "pose_a_heading_deg": math.degrees(pa.heading),
            "pose_b_x_m": pb.x, "pose_b_y_m": pb.y, "pose_b_heading_deg": math.degrees(pb.heading),
            "distance_m": k.distance, "direction_deg": math.degrees(k.direction),
            "velocity_mps": k.velocity, "omega_deg_s": math.degrees(k.omega),
        }
        new_file = not os.path.isfile(self.results_path)
        with open(self.results_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=RESULTS_FIELDS)
            if new_file:
                writer.writeheader()
            writer.writerow(row)
        messagebox.showinfo("Saved", f"Appended to {self.results_path}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("video", help="Recorded video file (.mp4) from VideoWriter.py")
    parser.add_argument("-c", "--calibration", default="calibration.json", help="Calibration file to use")
    parser.add_argument("-r", "--results", default="results.csv", help="CSV file to append saved results to")
    parser.add_argument("--lens-calibration", default=None,
                         help="lens_calibration.json (see lens_calib.py) to undistort every frame before "
                              "marking -- required for a fisheye/wide-FOV camera; must match the calibration "
                              "used to build --calibration")
    args = parser.parse_args(argv)
    PoseAnalyzer(args.video, args.calibration, args.results, args.lens_calibration).mainloop()


if __name__ == "__main__":
    main(sys.argv[1:])
