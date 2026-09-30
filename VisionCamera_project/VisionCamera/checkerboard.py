"""Generate a printable checkerboard calibration target for lens_calib.py.

Usage:  python -m VisionCamera.checkerboard [--squares-x 10] [--squares-y 7] [--square-mm 25] [-o checkerboard.png]

Print the output at 100% / "actual size" -- NOT "fit to page" or "shrink to
fit" -- then measure one square with a ruler to confirm it printed at the
intended size (printer margins/scaling can silently shrink it otherwise).
Mount it on something flat and rigid (cardboard, foam board, a clipboard)
so it doesn't bow when you hold it -- a warped board hurts calibration
accuracy. Matte paper is better than glossy, which can glare/reflect under
the field lighting.

squares_x/squares_y are full squares per side. lens_calib.py's --cols/--rows
are *interior corners*, i.e. one less than the squares per side, so a
10x7-square board (the default here) is --cols 9 --rows 6 there.
"""
from __future__ import annotations

import argparse
import sys

from PIL import Image, ImageDraw

DPI = 300


def mm_to_px(mm: float, dpi: int = DPI) -> int:
    return round(mm / 25.4 * dpi)


def generate(squares_x: int, squares_y: int, square_mm: float, margin_mm: float = 15.0,
             dpi: int = DPI) -> Image.Image:
    square_px = mm_to_px(square_mm, dpi)
    margin_px = mm_to_px(margin_mm, dpi)
    board_w, board_h = squares_x * square_px, squares_y * square_px
    footer_px = mm_to_px(10, dpi)

    img = Image.new("L", (board_w + 2 * margin_px, board_h + 2 * margin_px + footer_px), 255)
    draw = ImageDraw.Draw(img)
    for row in range(squares_y):
        for col in range(squares_x):
            if (row + col) % 2 == 0:
                x0 = margin_px + col * square_px
                y0 = margin_px + row * square_px
                draw.rectangle([x0, y0, x0 + square_px, y0 + square_px], fill=0)

    text = (f"{squares_x}x{squares_y} squares @ {square_mm:g} mm  "
            f"(lens_calib.py: --cols {squares_x - 1} --rows {squares_y - 1})  "
            "-- print at 100%, verify with a ruler")
    draw.text((margin_px, board_h + 2 * margin_px + footer_px // 3), text, fill=0)
    return img


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--squares-x", type=int, default=10, help="Full squares across (columns)")
    parser.add_argument("--squares-y", type=int, default=7, help="Full squares down (rows)")
    parser.add_argument("--square-mm", type=float, default=25.0, help="Physical size of one square, mm")
    parser.add_argument("-o", "--out", default="checkerboard.png", help="Output image path")
    args = parser.parse_args(argv)

    img = generate(args.squares_x, args.squares_y, args.square_mm)
    img.save(args.out, dpi=(DPI, DPI))
    print(f"Saved {args.out} ({img.width}x{img.height}px @ {DPI} dpi).")
    print(f"Print at 100%/actual size, then verify a square measures {args.square_mm:g} mm with a ruler.")
    print(f"Use with lens_calib.py: --cols {args.squares_x - 1} --rows {args.squares_y - 1}")


if __name__ == "__main__":
    main(sys.argv[1:])
