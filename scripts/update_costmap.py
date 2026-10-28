#!/home/hansm/anaconda3/bin/python3
"""
update_costmap.py — regenerate map.pgm and costmap_preview.png.

Default arena (world frame, metres): the 10/27/26 OptiTrack marker survey,
/mnt/e/mlss/costmaps/10_27_26/marker_locations.csv (see extract_markers.py there):
  1 SE   ( 3.099, -2.278)
  2 SSW  (-2.455, -3.120)
  3 SWW  (-4.207, -0.892)
  4 W    (-4.051,  1.453)
  5 NW   (-3.290,  3.372)
  6 N    (-0.098,  3.235)
  7 NE   ( 2.999,  3.020)   not measured (its bag was empty): where the N edge line
                            meets x = SE's x, then moved 5 in (0.127 m) towards the
                            arena centroid
Edges connect 1→2→…→7→1. Default obstacle: the crater, centred on its marker
(-0.159, -3.695), diameter 2.0 m (1.5 m measured, inflated).

Usage examples:
  # Default arena and crater
  python3 scripts/update_costmap.py

  # Without the default crater
  python3 scripts/update_costmap.py --no-default-obstacles

  # Add two circular obstacles (x, y, radius in metres)
  python3 scripts/update_costmap.py --obstacle 1.0 0.5 0.3 --obstacle -1.0 -1.0 0.4

  # Override one corner (corner index 1-based, then x y)
  python3 scripts/update_costmap.py --corner 3 -4.0 0.1

  # Override all corners as "x,y" pairs (any number >= 3, in order around the arena)
  python3 scripts/update_costmap.py --corners "2.31,-3.16" "-2.2,-3.07" "-3.86,0.28" "-3.453,3.02" "2.316,1.123"
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    print("[warn] scipy not found — inflation preview will be skipped")

try:
    from PIL import Image, ImageDraw
    HAS_PIL = True
except ImportError:
    HAS_PIL = False
    print("[warn] Pillow not found — preview image will not be generated")

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR   = Path(__file__).parent
REPO_ROOT    = SCRIPT_DIR.parent
MAP_PGM      = REPO_ROOT / "src/nav2_stack/maps/map.pgm"
PREVIEW_PNG  = SCRIPT_DIR / "costmap_preview.png"

# ── Map parameters (must match map.yaml) ─────────────────────────────────────
RESOLUTION  = 0.2    # m/pixel
ORIGIN_X    = -5.0
ORIGIN_Y    = -5.0
WIDTH_PX    = 50
HEIGHT_PX   = 50

# ── Nav2 inflation params (from nav2_param2.yaml, local_costmap/global_costmap's
# actual InflationLayer -- NOT the disabled MPPI ObstaclesCritic's own
# cost_scaling_factor, which is a separate, unrelated 10.0) ───────────────────
INFLATION_RADIUS = 0.75
COST_SCALING     = 2.58

# ── Default arena corners (x, y) in world frame ───────────────────────────────
# 10/27/26 marker survey (see the docstring). The previous arena, for reference:
# SE (2.28, -3.467), SW (-2.709, -2.305), W (-3.777, -0.367), NW (-2.85, 2.727),
# NE (2.2425, 1.9317)
DEFAULT_CORNERS = [
    ( 3.0988, -2.2780),   # 1 SE
    (-2.4545, -3.1198),   # 2 SSW
    (-4.2065, -0.8922),   # 3 SWW
    (-4.0505,  1.4526),   # 4 W
    (-3.2896,  3.3718),   # 5 NW
    (-0.0976,  3.2353),   # 6 N
    ( 2.9994,  3.0196),   # 7 NE (not measured, see the docstring)
]
DEFAULT_LABELS = ["1 SE", "2 SSW", "3 SWW", "4 W", "5 NW", "6 N", "7 NE"]
# (x, y, radius): the crater, 1.5 m measured, inflated to 2.0 m diameter
DEFAULT_OBSTACLES = [(-0.1587, -3.6945, 1.0)]
# Every arena edge is moved this far towards the inside before rasterizing (10/28/26:
# 7 in tighter than the surveyed edges). --inset overrides it, 0 = the surveyed polygon.
DEFAULT_INSET = 7 * 0.0254


# ── Geometry helpers ──────────────────────────────────────────────────────────

def point_in_polygon(x, y, poly):
    """Ray-casting point-in-polygon test."""
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def world_to_px(wx, wy):
    col = int((wx - ORIGIN_X) / RESOLUTION)
    row = int(HEIGHT_PX - 1 - (wy - ORIGIN_Y) / RESOLUTION)
    return col, row


def px_to_world(col, row):
    """World coordinates of the centre of pixel (col, row)."""
    wx = ORIGIN_X + (col + 0.5) * RESOLUTION
    wy = ORIGIN_Y + (HEIGHT_PX - 1 - row + 0.5) * RESOLUTION
    return wx, wy


# ── PGM generation ────────────────────────────────────────────────────────────

def inset_polygon(corners, d):
    """Move every edge of the polygon d metres inwards (d < 0: outwards); each new corner
    is where the two moved edges next to it meet."""
    n = len(corners)
    area2 = sum(corners[i][0] * corners[(i + 1) % n][1] - corners[(i + 1) % n][0] * corners[i][1]
                for i in range(n))
    sign = 1.0 if area2 > 0 else -1.0             # counter-clockwise: inside is to the left
    lines = []
    for i in range(n):
        (x1, y1), (x2, y2) = corners[i], corners[(i + 1) % n]
        L = math.hypot(x2 - x1, y2 - y1)
        nx, ny = -sign * (y2 - y1) / L, sign * (x2 - x1) / L      # inward unit normal
        lines.append(((x1 + d * nx, y1 + d * ny), (x2 - x1, y2 - y1)))
    out = []
    for i in range(n):
        (p, r), (q, t) = lines[i - 1], lines[i]   # edge into corner i, edge out of it
        den = r[0] * t[1] - r[1] * t[0]
        u = ((q[0] - p[0]) * t[1] - (q[1] - p[1]) * t[0]) / den
        out.append((p[0] + u * r[0], p[1] + u * r[1]))
    return out


def build_pgm(corners, obstacles):
    """
    Returns a (HEIGHT_PX, WIDTH_PX) uint8 array.
      254 = free space
        0 = occupied (outside arena or inside an obstacle circle)
    A pixel is classified by its centre (until 10/27/26 by its lower-left corner,
    which shifted the free area half a pixel towards +x/+y).
    """
    pixels = np.zeros((HEIGHT_PX, WIDTH_PX), dtype=np.uint8)

    for row in range(HEIGHT_PX):
        for col in range(WIDTH_PX):
            wx, wy = px_to_world(col, row)
            if point_in_polygon(wx, wy, corners) and \
                    not any(math.hypot(wx - ox, wy - oy) <= r for ox, oy, r in obstacles):
                pixels[row, col] = 254

    return pixels


def save_pgm(pixels, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'wb') as f:
        f.write(f'P5\n{WIDTH_PX} {HEIGHT_PX}\n255\n'.encode('ascii'))
        f.write(pixels.tobytes())
    free = int((pixels == 254).sum())
    total = WIDTH_PX * HEIGHT_PX
    print(f"[map]  Written {path}")
    print(f"       {free}/{total} free pixels ({100*free/total:.1f}%)")


# ── Preview generation ────────────────────────────────────────────────────────

def nav2_cost(d_m):
    """Nav2 inflation cost at distance d_m from nearest obstacle."""
    if d_m <= 0:
        return 254   # lethal
    cost = 253.0 * math.exp(-COST_SCALING * d_m)
    return max(1, math.ceil(cost))


def inflation_color(cost):
    """Map inflation cost [1..253] to an RGB colour (red→orange→yellow)."""
    t = (cost - 1) / 252.0   # 1.0 = inscribed (hottest), 0.0 = edge
    r = 255
    g = int(60 + (1 - t) * 170)
    b = int((1 - t) * 60)
    return (r, g, b)


def build_preview(pixels, corners, obstacles, labels):
    if not HAS_PIL:
        return None

    SCALE = 4
    W, H = WIDTH_PX * SCALE, HEIGHT_PX * SCALE
    LEGEND_W = 190   # dedicated side panel so the legend never overlaps the map

    if HAS_SCIPY:
        occupied  = (pixels == 0)
        dist_px   = distance_transform_edt(~occupied)
        dist_m    = dist_px * RESOLUTION
    else:
        dist_m = None

    img = Image.new('RGB', (W + LEGEND_W, H), color=(255, 255, 255))
    px  = img.load()

    for row in range(HEIGHT_PX):
        for col in range(WIDTH_PX):
            if pixels[row, col] == 0:
                color = (20, 20, 20)
            elif dist_m is not None and dist_m[row, col] <= INFLATION_RADIUS:
                cost  = nav2_cost(dist_m[row, col])
                color = inflation_color(cost)
            else:
                color = (245, 245, 245)

            for dr in range(SCALE):
                for dc in range(SCALE):
                    px[col * SCALE + dc, row * SCALE + dr] = color

    draw = ImageDraw.Draw(img)

    # Grid lines every 1 m
    for v in range(int(ORIGIN_X), int(-ORIGIN_X) + 1):
        col, _ = world_to_px(v, 0)
        x = col * SCALE + SCALE // 2
        draw.line([(x, 0), (x, H)], fill=(80, 80, 200), width=1)
        draw.text((x + 2, 2), f"{v}m", fill=(60, 60, 180))

        _, row = world_to_px(0, v)
        y = row * SCALE + SCALE // 2
        draw.line([(0, y), (W, y)], fill=(80, 80, 200), width=1)
        draw.text((2, y + 2), f"{v}m", fill=(60, 60, 180))

    # Arena polygon outline
    pts_s = [(world_to_px(x, y)[0] * SCALE + SCALE // 2,
              world_to_px(x, y)[1] * SCALE + SCALE // 2)
             for x, y in corners]
    draw.line(pts_s + [pts_s[0]], fill=(200, 0, 0), width=2)
    for (cx_s, cy_s), label in zip(pts_s, labels):
        draw.ellipse([cx_s - 5, cy_s - 5, cx_s + 5, cy_s + 5], fill=(200, 0, 0))
        draw.text((cx_s + 7, cy_s - 8), label, fill=(180, 0, 0))

    # Obstacle circles
    for (ox, oy, radius) in obstacles:
        cx_px, cy_px = world_to_px(ox, oy)
        r_px = radius / RESOLUTION
        cx_s = cx_px * SCALE + SCALE // 2
        cy_s = cy_px * SCALE + SCALE // 2
        rs   = r_px * SCALE
        draw.ellipse([cx_s - rs, cy_s - rs, cx_s + rs, cy_s + rs],
                     outline=(60, 0, 200), width=2)
        draw.text((cx_s + rs + 4, cy_s - 8),
                  f"r={radius}m", fill=(60, 0, 180))

    # Legend — drawn in the dedicated side panel to the right of the map so it
    # never overlays map content.
    lx, ly = W + 14, 14
    legend_entries = [
        ((20, 20, 20),    "Occupied / wall"),
        ((255, 60, 80),   "Inscribed zone"),
        ((255, 160, 60),  "Inflation gradient"),
        ((245, 245, 245), "Free space"),
        ((60, 0, 200),    "Added obstacle"),
    ]
    box_h = len(legend_entries) * 18 + 8
    draw.rectangle([lx - 4, ly - 4, lx + (LEGEND_W - 24), ly + box_h],
                   fill=(255, 255, 255), outline=(180, 180, 180))
    for i, (swatch, label) in enumerate(legend_entries):
        y = ly + 2 + i * 18
        draw.rectangle([lx, y, lx + 14, y + 12],
                       fill=swatch, outline=(120, 120, 120))
        draw.text((lx + 18, y), label, fill=(30, 30, 30))

    return img


# ── Argument parsing ──────────────────────────────────────────────────────────

def parse_corners(args_corners, args_corner_overrides):
    """Build the final corners list from defaults + CLI overrides."""
    corners = list(DEFAULT_CORNERS)

    if args_corners:
        if len(args_corners) < 3:
            sys.exit(f"[error] --corners expects at least 3 'x,y' pairs, got {len(args_corners)}")
        try:
            corners = [tuple(float(v) for v in s.split(',')) for s in args_corners]
        except ValueError:
            sys.exit("[error] --corners: each value must be 'x,y' (e.g. '2.31,-3.16')")

    for override in (args_corner_overrides or []):
        idx, x, y = override
        idx = int(idx)
        if not (1 <= idx <= len(corners)):
            sys.exit(f"[error] --corner index must be 1-{len(corners)}, got {idx}")
        corners[idx - 1] = (x, y)
        print(f"[args] Corner {idx} overridden to ({x}, {y})")

    return corners


def main():
    ap = argparse.ArgumentParser(
        description="Regenerate costmap map.pgm and costmap_preview.png.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument(
        "--corners", nargs="+", metavar="X,Y",
        help="Override ALL arena corners as 'x,y' pairs, in order around the arena "
             "(default: the 7 corners of the 10/27/26 survey)"
    )
    ap.add_argument(
        "--corner", nargs=3, metavar=("IDX", "X", "Y"), action="append",
        type=float, dest="corner_overrides",
        help="Override ONE corner by 1-based index. Can be repeated. "
             "Example: --corner 3 -4.0 0.1"
    )
    ap.add_argument(
        "--obstacle", nargs=3, metavar=("X", "Y", "R"), action="append",
        type=float, default=[],
        help="Add a circular obstacle at world coords (X, Y) with radius R metres. "
             "Can be repeated. Example: --obstacle 1.0 0.5 0.3"
    )
    ap.add_argument(
        "--inset", type=float, default=DEFAULT_INSET,
        help=f"Move every arena edge this many metres inwards (default {DEFAULT_INSET:.4f} = 7 in; "
             f"0 = the surveyed edges)"
    )
    ap.add_argument(
        "--no-default-obstacles", action="store_true",
        help="Leave out the default obstacles (the crater)"
    )
    ap.add_argument(
        "--out-map", default=str(MAP_PGM),
        help=f"Output map.pgm path (default: {MAP_PGM})"
    )
    ap.add_argument(
        "--out-preview", default=str(PREVIEW_PNG),
        help=f"Output preview PNG path (default: {PREVIEW_PNG})"
    )
    ap.add_argument(
        "--no-preview", action="store_true",
        help="Skip generating the preview PNG"
    )

    args = ap.parse_args()

    corners   = parse_corners(args.corners, args.corner_overrides)
    if args.inset:
        print(f"[arena] Edges moved {args.inset:.4f} m ({args.inset / 0.0254:.1f} in) inwards")
        corners = inset_polygon(corners, args.inset)
    labels    = DEFAULT_LABELS if len(corners) == len(DEFAULT_LABELS) and not args.corners \
        else [str(i + 1) for i in range(len(corners))]
    obstacles = ([] if args.no_default_obstacles else list(DEFAULT_OBSTACLES)) + \
        [tuple(o) for o in args.obstacle]

    print(f"\n[arena] Corners:")
    for label, (x, y) in zip(labels, corners):
        print(f"  {label:6s}  ({x:+.3f}, {y:+.3f})")

    if obstacles:
        print(f"[obs]  Obstacles:")
        for ox, oy, r in obstacles:
            print(f"  ({ox:+.3f}, {oy:+.3f})  r={r} m")

    print()
    pixels = build_pgm(corners, obstacles)
    save_pgm(pixels, args.out_map)

    if not args.no_preview:
        img = build_preview(pixels, corners, obstacles, labels)
        if img is not None:
            img.save(args.out_preview)
            print(f"[preview] Written {args.out_preview}")
        else:
            print("[preview] Skipped (Pillow not available)")


if __name__ == "__main__":
    main()
