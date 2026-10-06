"""VisionCamera launcher: pick which tool to run.

Usage:  python -m VisionCamera

Each button below launches one tool as its own process (so you can run
several at once, e.g. the recorder while reviewing an older clip in the pose
analyzer), using whatever defaults that tool ships with -- calibration_tool.py
and pose_analyzer.py will prompt you to pick an image/video file if you don't
pass one on the command line, same as clicking their buttons here does.
"""
from __future__ import annotations

import subprocess
import sys
import tkinter as tk
from datetime import datetime
from tkinter import messagebox, ttk

try:  # python -m VisionCamera
    from . import checkerboard
except ImportError:  # run directly, e.g. `python __main__.py`
    import checkerboard

# (button label, module name, one-line description)
TOOLS = [
    ("Record (GUI)", "recorder_gui",
     "Source/resolution/exposure controls, live view, Start/Stop recording."),
    ("Record (headless)", "VideoWriter",
     "Plain CLI recorder -- press 'q' in its preview window to stop."),
    ("View captures", "viewCaptures",
     "Browse/blend the periodic JPEG snapshots saved while recording."),
    ("Lens calibration", "lens_calib",
     "One-time fisheye/wide-FOV calibration from a checkerboard (needs a camera)."),
    ("Field calibration", "calibration_tool",
     "Click known field points on a reference frame to build calibration.json."),
    ("Pose analyzer", "pose_analyzer",
     "Mark 2 frames, read off distance/direction/velocity/omega."),
]


class Launcher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("VisionCamera")
        self.geometry("560x420")
        self._build_ui()

    def _build_ui(self):
        ttk.Label(self, text="VisionCamera", font=("Segoe UI", 14, "bold")).pack(anchor="w", padx=12, pady=(12, 0))
        ttk.Label(self, text="Overhead camera pose/velocity ground truth -- pick a tool:",
                  wraplength=520, justify="left").pack(anchor="w", padx=12, pady=(0, 8))

        for label, module, description in TOOLS:
            row = ttk.Frame(self, padding=(12, 4))
            row.pack(fill="x")
            ttk.Button(row, text=label, width=18,
                       command=lambda m=module, l=label: self.launch(m, l)).pack(side="left")
            ttk.Label(row, text=description, wraplength=360, justify="left").pack(side="left", padx=(10, 0))

        ttk.Separator(self).pack(fill="x", padx=12, pady=(8, 4))
        gen_row = ttk.Frame(self, padding=(12, 4))
        gen_row.pack(fill="x")
        ttk.Button(gen_row, text="Generate checkerboard", width=18,
                   command=self.generate_checkerboard).pack(side="left")
        ttk.Label(gen_row, text="Print-ready target for lens calibration (640x480 default -> checkerboard.png).",
                  wraplength=360, justify="left").pack(side="left", padx=(10, 0))

        self.log = tk.Text(self, height=6, font=("Consolas", 9), state="disabled")
        self.log.pack(fill="both", expand=True, padx=12, pady=(8, 12))

    def _log(self, message: str):
        self.log.config(state="normal")
        self.log.insert("end", f"[{datetime.now():%H:%M:%S}] {message}\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def launch(self, module: str, label: str):
        try:
            subprocess.Popen([sys.executable, "-m", f"VisionCamera.{module}"])
        except OSError as e:
            messagebox.showerror("Launch failed", f"Could not start {label}:\n{e}")
            return
        self._log(f"Launched {label} (python -m VisionCamera.{module})")

    def generate_checkerboard(self):
        path = "checkerboard.png"
        img = checkerboard.generate(10, 7, 25.0)
        img.save(path, dpi=(checkerboard.DPI, checkerboard.DPI))
        self._log(f"Saved {path} (10x7 squares @ 25mm, --cols 9 --rows 6)")
        messagebox.showinfo(
            "Checkerboard saved",
            f"Saved {path}.\n\nPrint at 100%/actual size (not \"fit to page\"), then verify a square "
            "measures 25 mm with a ruler before using it for lens calibration.",
        )


def main() -> None:
    Launcher().mainloop()


if __name__ == "__main__":
    main()
