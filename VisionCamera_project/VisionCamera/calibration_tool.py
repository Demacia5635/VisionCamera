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
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

import cv2
import numpy as np
from PIL import Image, ImageTk

try:  # python -m VisionCamera.calibration_tool
    from .field_coords import FieldCalibration
    from .undistort import LensCalibration
except ImportError:  # run directly, e.g. `python calibration_tool.py`
    from field_coords import FieldCalibration
    from undistort import LensCalibration

GRID_STEP_M = 1.0  # meters between preview gridlines


class CalibrationTool(tk.Tk):
    def __init__(self, image_path: str, out_path: str, lens_calib_path: str | None = None):
        super().__init__()
        self.title(f"Field calibration - {image_path}")
        self.geometry("1100x750")
        self.out_path = out_path

        self.image = Image.open(image_path).convert("RGB")
        if lens_calib_path:
            lens = LensCalibration.load(lens_calib_path)
            undistorted = lens.undistort(cv2.cvtColor(np.array(self.image), cv2.COLOR_RGB2BGR))
            self.image = Image.fromarray(cv2.cvtColor(undistorted, cv2.COLOR_BGR2RGB))
        self.photo = None
        self.scale = 1.0
        self.offset = (0, 0)

        # Each entry: {"px": (x, y), "field": (fx, fy)}
        self.points: list[dict] = []
        self.calib: FieldCalibration | None = None
        self.show_grid = tk.BooleanVar(value=False)

        self._build_ui()
        self.bind("<Configure>", lambda e: self.redraw())

    # ---------- UI ----------
    def _build_ui(self):
        left = ttk.Frame(self, padding=6)
        left.pack(side="left", fill="y")

        ttk.Label(left, text="Click a known point on the image,\nthen enter its field X/Y (m).",
                  justify="left").pack(anchor="w")

        self.listbox = tk.Listbox(left, width=34, height=20, font=("Consolas", 10))
        self.listbox.pack(pady=6, fill="y", expand=True)

        ttk.Button(left, text="Delete selected", command=self.delete_selected).pack(fill="x")
        ttk.Checkbutton(left, text="Preview grid", variable=self.show_grid,
                         command=self.redraw).pack(fill="x", pady=(6, 0))
        ttk.Button(left, text="Compute && save", command=self.compute_and_save).pack(fill="x", pady=(12, 0))

        self.status = ttk.Label(left, text="0 points (need >= 4)", wraplength=240)
        self.status.pack(fill="x", pady=(10, 0))

        right = ttk.Frame(self, padding=6)
        right.pack(side="left", fill="both", expand=True)
        self.canvas = tk.Canvas(right, bg="#202020", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Button-1>", self.on_click)

    # ---------- geometry helpers ----------
    def canvas_to_image(self, cx, cy):
        ox, oy = self.offset
        return (cx - ox) / self.scale, (cy - oy) / self.scale

    def image_to_canvas(self, ix, iy):
        ox, oy = self.offset
        return ix * self.scale + ox, iy * self.scale + oy

    # ---------- events ----------
    def on_click(self, event):
        ix, iy = self.canvas_to_image(event.x, event.y)
        if not (0 <= ix < self.image.width and 0 <= iy < self.image.height):
            return
        fx = simpledialog.askfloat("Field X", "Field X coordinate (meters):", parent=self)
        if fx is None:
            return
        fy = simpledialog.askfloat("Field Y", "Field Y coordinate (meters):", parent=self)
        if fy is None:
            return
        self.points.append({"px": (ix, iy), "field": (fx, fy)})
        self.listbox.insert("end", f"#{len(self.points)}  px=({ix:.0f},{iy:.0f})  field=({fx:.3f},{fy:.3f})")
        self.status.config(text=f"{len(self.points)} points (need >= 4)")
        self.redraw()

    def delete_selected(self):
        sel = self.listbox.curselection()
        if not sel:
            return
        idx = sel[0]
        del self.points[idx]
        self.listbox.delete(idx)
        for i in range(idx, len(self.points)):
            self.listbox.delete(i)
            self.listbox.insert(i, self._point_label(i))
        self.status.config(text=f"{len(self.points)} points (need >= 4)")
        self.redraw()

    def _point_label(self, i):
        p = self.points[i]
        ix, iy = p["px"]
        fx, fy = p["field"]
        return f"#{i + 1}  px=({ix:.0f},{iy:.0f})  field=({fx:.3f},{fy:.3f})"

    def compute_and_save(self):
        if len(self.points) < 4:
            messagebox.showerror("Not enough points", "Need at least 4 point correspondences.")
            return
        image_points = [p["px"] for p in self.points]
        field_points = [p["field"] for p in self.points]
        try:
            self.calib = FieldCalibration.from_points(
                image_points, field_points,
                meta={"image_size": [self.image.width, self.image.height],
                      "num_points": len(self.points)},
            )
        except ValueError as e:
            messagebox.showerror("Calibration failed", str(e))
            return

        errors = []
        for p in self.points:
            fx, fy = self.calib.pixel_to_field(*p["px"])
            ex, ey = fx - p["field"][0], fy - p["field"][1]
            errors.append((ex ** 2 + ey ** 2) ** 0.5)
        rms = (sum(e ** 2 for e in errors) / len(errors)) ** 0.5

        self.calib.save(self.out_path)
        self.show_grid.set(True)
        self.redraw()
        messagebox.showinfo(
            "Saved",
            f"Saved {self.out_path}\n\nReprojection error: rms={rms:.3f} m, max={max(errors):.3f} m\n"
            "(large errors usually mean a mis-typed coordinate or a mis-clicked point)",
        )

    # ---------- drawing ----------
    def redraw(self):
        self.canvas.delete("all")
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        if cw < 2 or ch < 2:
            return
        self.scale = min(cw / self.image.width, ch / self.image.height)
        size = (max(1, int(self.image.width * self.scale)), max(1, int(self.image.height * self.scale)))
        ox = (cw - size[0]) // 2
        oy = (ch - size[1]) // 2
        self.offset = (ox, oy)
        self.photo = ImageTk.PhotoImage(self.image.resize(size, Image.Resampling.LANCZOS))
        self.canvas.create_image(ox, oy, image=self.photo, anchor="nw")

        for i, p in enumerate(self.points):
            cx, cy = self.image_to_canvas(*p["px"])
            self.canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, outline="#00ff88", width=2)
            self.canvas.create_text(cx + 8, cy - 8, text=str(i + 1), fill="#00ff88", anchor="w",
                                     font=("Consolas", 11, "bold"))

        if self.show_grid.get() and self.calib is not None:
            self._draw_grid()

    def _draw_grid(self):
        fxs = [p["field"][0] for p in self.points]
        fys = [p["field"][1] for p in self.points]
        margin = 2.0
        x0, x1 = min(fxs) - margin, max(fxs) + margin
        y0, y1 = min(fys) - margin, max(fys) + margin

        def grid_range(lo, hi, step):
            n0 = int(lo // step) - 1
            n1 = int(hi // step) + 1
            return [n * step for n in range(n0, n1 + 1)]

        for gx in grid_range(x0, x1, GRID_STEP_M):
            pts = [self.calib.field_to_pixel(gx, gy) for gy in (y0, y1)]
            self._draw_line(pts[0], pts[1])
        for gy in grid_range(y0, y1, GRID_STEP_M):
            pts = [self.calib.field_to_pixel(gx, gy) for gx in (x0, x1)]
            self._draw_line(pts[0], pts[1])

    def _draw_line(self, p_img_a, p_img_b):
        ca = self.image_to_canvas(*p_img_a)
        cb = self.image_to_canvas(*p_img_b)
        self.canvas.create_line(*ca, *cb, fill="#ffaa00", width=1)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", help="Reference frame image showing known field points")
    parser.add_argument("-o", "--out", default="calibration.json", help="Output calibration file")
    parser.add_argument("--lens-calibration", default="lens_calibration.json",
                         help="lens_calibration.json (see lens_calib.py) to undistort the "
                              "reference image before marking points -- required for a fisheye/wide-FOV camera")
    args = parser.parse_args(argv)
    CalibrationTool(args.image, args.out, args.lens_calibration).mainloop()


if __name__ == "__main__":
    main(sys.argv[1:])
