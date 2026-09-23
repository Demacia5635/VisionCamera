"""Browse captured frames and merge two of them a chosen duration apart.

Usage:  python view_captures.py [captures_folder]

Filenames are expected as frame_YYYYMMDD_HHMMSS_mmm.jpg (as written by the
capture script). If a name doesn't parse, the file's modified time is used.
"""
import bisect
import os
import re
import sys
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageChops, ImageTk

DEFAULT_DIR = "captures"
EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp")
NAME_RE = re.compile(r"(\d{8})_(\d{6})_(\d{3})")
MODES = ("Blend", "Side by side", "Difference")


def parse_time(path):
    """Return the capture time (seconds since epoch) for an image file."""
    m = NAME_RE.search(os.path.basename(path))
    if m:
        date, clock, ms = m.groups()
        dt = datetime.strptime(date + clock, "%Y%m%d%H%M%S")
        return dt.timestamp() + int(ms) / 1000.0
    return os.path.getmtime(path)


def load_folder(folder):
    """Return a list of (relative_seconds, path) sorted by time."""
    paths = [
        os.path.join(folder, f)
        for f in os.listdir(folder)
        if f.lower().endswith(EXTENSIONS)
    ]
    items = sorted((parse_time(p), p) for p in paths)
    if not items:
        return []
    t0 = items[0][0]
    return [(t - t0, p) for t, p in items]


def merge_images(a, b, mode, alpha):
    """Combine two PIL images. `alpha` is the weight of b in Blend mode (0..1)."""
    a = a.convert("RGB")
    b = b.convert("RGB")
    if b.size != a.size:
        b = b.resize(a.size)
    if mode == "Side by side":
        out = Image.new("RGB", (a.width * 2, a.height))
        out.paste(a, (0, 0))
        out.paste(b, (a.width, 0))
        return out
    if mode == "Difference":
        return ImageChops.difference(a, b)
    return Image.blend(a, b, alpha)


class Viewer(tk.Tk):
    def __init__(self, folder):
        super().__init__()
        self.title("Capture viewer")
        self.geometry("1100x700")

        self.items = []      # [(relative_seconds, path)]
        self.times = []      # relative seconds only, for bisect
        self.merged = None   # last merged PIL image
        self.photo = None    # keep a reference so Tk doesn't discard it

        self._build_ui()
        self.load(folder)

    # ---------- UI ----------
    def _build_ui(self):
        left = ttk.Frame(self, padding=6)
        left.pack(side="left", fill="y")

        ttk.Button(left, text="Open folder...", command=self.choose_folder).pack(fill="x")
        self.folder_label = ttk.Label(left, text="", wraplength=240)
        self.folder_label.pack(fill="x", pady=(4, 6))

        list_frame = ttk.Frame(left)
        list_frame.pack(fill="y", expand=True)
        self.listbox = tk.Listbox(list_frame, width=34, exportselection=False,
                                  font=("Consolas", 10))
        scroll = ttk.Scrollbar(list_frame, orient="vertical", command=self.listbox.yview)
        self.listbox.configure(yscrollcommand=scroll.set)
        self.listbox.pack(side="left", fill="y", expand=True)
        scroll.pack(side="left", fill="y")
        self.listbox.bind("<<ListboxSelect>>", lambda e: self.update_view())

        right = ttk.Frame(self, padding=6)
        right.pack(side="left", fill="both", expand=True)

        controls = ttk.Frame(right)
        controls.pack(fill="x")

        ttk.Label(controls, text="Duration (s):").grid(row=0, column=0, sticky="w")
        self.duration = tk.StringVar(value="1.0")
        spin = ttk.Spinbox(controls, from_=0, to=100000, increment=0.5,
                           textvariable=self.duration, width=8, command=self.update_view)
        spin.grid(row=0, column=1, padx=(4, 16))
        spin.bind("<Return>", lambda e: self.update_view())
        spin.bind("<FocusOut>", lambda e: self.update_view())

        ttk.Label(controls, text="Merge:").grid(row=0, column=2, sticky="w")
        self.mode = tk.StringVar(value=MODES[0])
        combo = ttk.Combobox(controls, values=MODES, textvariable=self.mode,
                             state="readonly", width=14)
        combo.grid(row=0, column=3, padx=(4, 16))
        combo.bind("<<ComboboxSelected>>", lambda e: self.update_view())

        ttk.Label(controls, text="Blend:").grid(row=0, column=4, sticky="w")
        self.alpha = tk.DoubleVar(value=0.5)
        ttk.Scale(controls, from_=0, to=1, variable=self.alpha, length=160,
                  command=lambda v: self.update_view()).grid(row=0, column=5, padx=4)

        self.info = ttk.Label(right, text="", anchor="w")
        self.info.pack(fill="x", pady=6)

        self.canvas = tk.Canvas(right, bg="#202020", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda e: self.redraw())

    # ---------- data ----------
    def choose_folder(self):
        folder = filedialog.askdirectory(title="Select captures folder")
        if folder:
            self.load(folder)

    def load(self, folder):
        if not os.path.isdir(folder):
            messagebox.showerror("Error", f"Folder not found: {folder}")
            return
        self.items = load_folder(folder)
        self.times = [t for t, _ in self.items]
        self.folder_label.config(text=os.path.abspath(folder))
        self.listbox.delete(0, "end")
        for t, p in self.items:
            self.listbox.insert("end", f"+{t:9.3f} s  {os.path.basename(p)}")
        self.merged = None
        self.redraw()
        if self.items:
            self.listbox.selection_set(0)
            self.update_view()
        else:
            self.info.config(text="No images found in this folder.")

    # ---------- merging ----------
    def find_partner(self, index, duration):
        """Index of the image closest to items[index] time + duration, or None."""
        target = self.times[index] + duration
        if target > self.times[-1] + 1e-9:
            return None
        j = bisect.bisect_left(self.times, target)
        if j > 0 and (j == len(self.times) or
                      target - self.times[j - 1] <= self.times[j] - target):
            j -= 1
        return j

    def update_view(self):
        sel = self.listbox.curselection()
        if not sel or not self.items:
            return
        try:
            duration = float(self.duration.get())
            if duration < 0:
                raise ValueError
        except ValueError:
            self.info.config(text="Duration must be a non-negative number.")
            return

        i = sel[0]
        j = self.find_partner(i, duration)
        if j is None:
            self.merged = None
            self.info.config(
                text=f"Image at +{self.times[i]:.3f} s plus {duration:g} s is past the last "
                     f"image (+{self.times[-1]:.3f} s).")
            self.redraw()
            return

        try:
            with Image.open(self.items[i][1]) as a, Image.open(self.items[j][1]) as b:
                self.merged = merge_images(a, b, self.mode.get(), self.alpha.get())
        except OSError as e:
            self.merged = None
            self.info.config(text=f"Could not read image: {e}")
            self.redraw()
            return

        actual = self.times[j] - self.times[i]
        self.info.config(
            text=f"A: +{self.times[i]:.3f} s   B: +{self.times[j]:.3f} s   "
                 f"(actual gap {actual:.3f} s, requested {duration:g} s)")
        self.redraw()

    def redraw(self):
        self.canvas.delete("all")
        if self.merged is None:
            return
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        if cw < 2 or ch < 2:
            return
        scale = min(cw / self.merged.width, ch / self.merged.height)
        size = (max(1, int(self.merged.width * scale)), max(1, int(self.merged.height * scale)))
        self.photo = ImageTk.PhotoImage(self.merged.resize(size, Image.Resampling.LANCZOS))
        self.canvas.create_image(cw // 2, ch // 2, image=self.photo, anchor="center")


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    Viewer(argv[0] if argv else DEFAULT_DIR).mainloop()


if __name__ == "__main__":
    main()
