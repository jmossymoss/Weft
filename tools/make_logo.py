#!/usr/bin/env python3
"""Regenerate app/weft_logo_data.h from the Weft weave mark.

The mark: an interwoven W of orange threads — a top arch, two
interlocked bottom U's, and five dot-bar-dot warp stitches — every
thread end separated from its dot by a thin white gap (the woven look).
Drawn as vectors at 1024px with 4x supersampling, then downsampled to
the icon sizes and the Settings badge. Transparent background.

    python3 tools/make_logo.py   # rewrites app/weft_logo_data.h
"""
import math
from pathlib import Path

from PIL import Image, ImageDraw

ORANGE = (245, 124, 10, 255)
S = 4                      # supersample factor
C = 1024 * S               # canvas
W = int(46 * S)            # stroke width
R = int(47 * S)            # dot radius
GAP = int(14 * S)          # white separation ring around dots

# Column x positions (five warp columns) and the interlock offset.
CX = [int(x * S) for x in (184, 350, 512, 674, 840)]
OFF = int(26 * S)          # half-gap between the two U legs at center


def stroke_arc(d, cx, cy, r, a0, a1, width):
    # Round-capped arc: PIL arc + cap dots.
    bbox = [cx - r, cy - r, cx + r, cy + r]
    d.arc(bbox, a0, a1, fill=ORANGE, width=width)
    for a in (a0, a1):
        x = cx + r * math.cos(math.radians(a))
        y = cy + r * math.sin(math.radians(a))
        d.ellipse([x - width / 2, y - width / 2, x + width / 2,
                   y + width / 2], fill=ORANGE)


def stroke_line(d, x0, y0, x1, y1, width):
    d.line([x0, y0, x1, y1], fill=ORANGE, width=width)
    for x, y in ((x0, y0), (x1, y1)):
        d.ellipse([x - width / 2, y - width / 2, x + width / 2,
                   y + width / 2], fill=ORANGE)


def dot(d, x, y, r=None, color=ORANGE):
    r = R if r is None else r
    d.ellipse([x - r, y - r, x + r, y + r], fill=color)


def render():
    img = Image.new("RGBA", (C, C), (255, 255, 255, 0))
    d = ImageDraw.Draw(img)
    c1, c2, c3, c4, c5 = CX
    # --- threads -------------------------------------------------------
    # Top arch: c2 -> c4.
    arch_y = int(340 * S)
    stroke_arc(d, (c2 + c4) // 2, arch_y, (c4 - c2) // 2, 180, 360, W)
    # Bottom U's, interlocked at center: left U c1 -> c3-OFF, right U
    # c3+OFF -> c5. Legs rise to mid height; the right U's left leg
    # continues up as the long center thread.
    u_y = int(700 * S)
    # Left U: c1 .. c3-2*OFF (its right leg caps mid-height, just left
    # of the center thread); right U: c3 .. c5 (its left leg IS the long
    # center thread, colinear with the center dot).
    il = c3 - 2 * OFF
    u_r_l = (il - c1) // 2
    u_r_r = (c5 - c3) // 2
    stroke_arc(d, c1 + u_r_l, u_y, u_r_l, 0, 180, W)
    stroke_arc(d, c3 + u_r_r, u_y, u_r_r, 0, 180, W)
    # U legs.
    leg_top_outer = int(700 * S)          # outer legs stop at the arc
    leg_top_inner = int(620 * S)          # left U right leg cap height
    stroke_line(d, il, u_y, il, leg_top_inner, W)
    center_top = int(430 * S)             # long center thread
    stroke_line(d, c3, u_y, c3, center_top, W)
    stroke_line(d, c5, u_y, c5, leg_top_outer, W)
    stroke_line(d, c1, u_y, c1, leg_top_outer, W)
    # --- white gaps under every dot, then the dots ----------------------
    stitches = []  # (x, dot_y, bar_y0, bar_y1, lower_dot_y)
    stitches.append((c1, int(388 * S), int(455 * S), int(608 * S),
                     int(675 * S)))
    stitches.append((c5, int(388 * S), int(455 * S), int(608 * S),
                     int(675 * S)))
    stitches.append((c2, int(330 * S), int(400 * S), int(560 * S),
                     int(628 * S)))
    stitches.append((c4, int(330 * S), int(400 * S), int(560 * S),
                     int(628 * S)))
    dots = []
    for x, dy, b0, b1, ly in stitches:
        dots.append((x, dy))
        dots.append((x, ly))
    dots.append((c3, int(360 * S)))       # center top dot
    # Gap rings first (punch white/transparent), then bars, then dots.
    for x, y in dots:
        dot(d, x, y, R + GAP, (255, 255, 255, 0))
    for x, dy, b0, b1, ly in stitches:
        stroke_line(d, x, b0, x, b1, W)
    # Re-punch gaps around dot sites that bars/threads crossed.
    for x, y in dots:
        dot(d, x, y, R + GAP, (255, 255, 255, 0))
    for x, y in dots:
        dot(d, x, y)
    return img.resize((1024, 1024), Image.LANCZOS)


def emit(img):
    sizes = [16, 32, 48, 64]
    out = ["// Auto-generated weft logo RGBA bitmaps (row-major, RGBA8,"
           " top-left origin).",
           "// Regenerate with tools/make_logo.py. Do not edit by hand.",
           "#pragma once", ""]

    def arr(name, size):
        im = img.resize((size, size), Image.LANCZOS)
        px = list(im.getdata())
        vals = []
        for r, g, b, a in px:
            vals += [r, g, b, a]
        body = ",".join(str(v) for v in vals)
        lines = []
        while len(body) > 116:
            cut = body.rfind(",", 0, 116) + 1
            lines.append("  " + body[:cut])
            body = body[cut:]
        lines.append("  " + body)
        out.append(f"static const unsigned char {name}[{size}*{size}*4]"
                   " = {")
        out.extend(lines)
        out.append("};")
        out.append("")

    for s in sizes:
        arr(f"weft_icon_{s}", s)
    arr("weft_badge", 96)
    out.append("static const int weft_badge_w = 96;")
    out.append("static const int weft_badge_h = 96;")
    out.append("")
    Path("app/weft_logo_data.h").write_text("\n".join(out))


if __name__ == "__main__":
    emit(render())
    print("wrote app/weft_logo_data.h")
