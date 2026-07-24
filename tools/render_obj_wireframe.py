"""Render OBJ face groups as an orthographic wireframe PNG.

A dependency-light companion to render_obj_faces.py (which needs Blender):
enough to inspect and compare topology on specific CAD faces when reviewing
an intentional count change.

  python3 tools/render_obj_wireframe.py mesh.obj out.png --faces 43,50 \
      [--label TEXT] [--elev 18] [--azim -60] [--size 900]

Polygon interiors are filled flat and edges drawn on top, so quads, n-gons
and triangles are told apart by their outlines. Faces given with --faces are
drawn highlighted; everything else is dropped.
"""

import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw

QUAD_FILL = (86, 96, 112)
NGON_FILL = (150, 96, 60)
TRI_FILL = (110, 78, 110)
EDGE = (232, 236, 242)
BG = (24, 26, 30)


def read_obj(path, wanted):
    verts = []
    polys = []
    face_id = 0
    with Path(path).open("r", errors="ignore") as stream:
        for line in stream:
            if line.startswith("v "):
                _, x, y, z, *_ = line.split()
                verts.append((float(x), float(y), float(z)))
            elif line.startswith("g face_") or line.startswith("o face_"):
                try:
                    face_id = int(line.strip().split("_", 1)[1])
                except ValueError:
                    face_id = 0
            elif line.startswith("f "):
                if wanted and face_id not in wanted:
                    continue
                idx = [int(p.split("/", 1)[0]) - 1 for p in line.split()[1:]]
                polys.append(idx)
    return verts, polys


def project(verts, polys, elev, azim, size, pad=40):
    ce, se = math.cos(math.radians(elev)), math.sin(math.radians(elev))
    ca, sa = math.cos(math.radians(azim)), math.sin(math.radians(azim))
    used = sorted({i for poly in polys for i in poly})
    cam = {}
    for i in used:
        x, y, z = verts[i]
        # yaw about Z, then pitch
        rx = ca * x + sa * y
        ry = -sa * x + ca * y
        cam[i] = (rx, ce * ry + se * z, -se * ry + ce * z)
    if not cam:
        return {}, 1.0
    xs = [p[0] for p in cam.values()]
    ys = [p[1] for p in cam.values()]
    spanx = max(xs) - min(xs) or 1.0
    spany = max(ys) - min(ys) or 1.0
    scale = (size - 2 * pad) / max(spanx, spany)
    cx = 0.5 * (max(xs) + min(xs))
    cy = 0.5 * (max(ys) + min(ys))
    screen = {}
    for i, (x, y, depth) in cam.items():
        screen[i] = (
            size / 2 + (x - cx) * scale,
            size / 2 - (y - cy) * scale,
            depth,
        )
    return screen, scale


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("obj")
    ap.add_argument("out")
    ap.add_argument("--faces", default="")
    ap.add_argument("--label", default="")
    ap.add_argument("--elev", type=float, default=18.0)
    ap.add_argument("--azim", type=float, default=-60.0)
    ap.add_argument("--size", type=int, default=900)
    args = ap.parse_args()

    wanted = {int(t) for t in args.faces.split(",") if t.strip()}
    verts, polys = read_obj(args.obj, wanted)
    if not polys:
        raise SystemExit(f"no polygons for faces {sorted(wanted)} in {args.obj}")
    screen, _ = project(verts, polys, args.elev, args.azim, args.size)

    img = Image.new("RGB", (args.size, args.size), BG)
    draw = ImageDraw.Draw(img)
    ordered = sorted(
        polys, key=lambda p: sum(screen[i][2] for i in p) / len(p)
    )
    counts = {"quad": 0, "tri": 0, "ngon": 0}
    for poly in ordered:
        pts = [(screen[i][0], screen[i][1]) for i in poly]
        if len(poly) == 4:
            fill, kind = QUAD_FILL, "quad"
        elif len(poly) == 3:
            fill, kind = TRI_FILL, "tri"
        else:
            fill, kind = NGON_FILL, "ngon"
        counts[kind] += 1
        draw.polygon(pts, fill=fill)
        draw.line(pts + [pts[0]], fill=EDGE, width=1)

    caption = args.label or Path(args.obj).name
    draw.text(
        (12, 10),
        f"{caption}   {counts['quad']} quads  {counts['tri']} tris  "
        f"{counts['ngon']} n-gons",
        fill=(240, 240, 240),
    )
    img.save(args.out)
    print(f"{args.out}: {counts}")


if __name__ == "__main__":
    main()
