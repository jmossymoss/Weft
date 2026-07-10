#!/usr/bin/env python3
"""Embed the Weft logo (assets/weft_logo.png) into the app.

The logo image is used verbatim — no cropping, no redrawing. It is only
LANCZOS-downscaled to the sizes the app needs:

  app/weft_logo_data.h   RGBA arrays for the GLFW window icon
                         (16/32/48/64) and the Settings badge (96).
  app/weft.ico           multi-size Windows icon compiled into the .exe
                         via app/weft.rc.

    python3 tools/make_logo.py                 # uses assets/weft_logo.png
    python3 tools/make_logo.py --from path.png # uses another source image
"""
import sys
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SRC = ROOT / "assets" / "weft_logo.png"


def emit_header(img):
    sizes = [16, 32, 48, 64]
    out = ["// Auto-generated weft logo RGBA bitmaps (row-major, RGBA8,"
           " top-left origin).",
           "// Source image: assets/weft_logo.png (downscaled only, never"
           " redrawn).",
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
    Path(ROOT / "app" / "weft_logo_data.h").write_text("\n".join(out))


def emit_ico(img):
    sizes = [(s, s) for s in (16, 24, 32, 48, 64, 128, 256)]
    base = img.resize((256, 256), Image.LANCZOS)
    base.save(ROOT / "app" / "weft.ico", format="ICO", sizes=sizes)


if __name__ == "__main__":
    src = DEFAULT_SRC
    if len(sys.argv) == 3 and sys.argv[1] == "--from":
        src = Path(sys.argv[2])
    img = Image.open(src).convert("RGBA")
    if img.width != img.height:
        sys.exit(f"logo must be square, got {img.width}x{img.height}")
    emit_header(img)
    emit_ico(img)
    print(f"wrote app/weft_logo_data.h and app/weft.ico from {src}")
