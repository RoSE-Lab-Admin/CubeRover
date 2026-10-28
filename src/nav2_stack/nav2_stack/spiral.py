"""Spiral waypoints for autonomous_trials' waypoint_mode "spiral" (instead of
gp_explorer's information-based goals).

An Archimedean spiral around a centre: it starts on a circle of radius r_min
(the turning radius the rover should drive), grows by GROWTH per loop until the
next point would come closer than CLEARANCE to a wall of the Nav2 map ("the
outside is reached"), then shrinks back to r_min, then grows again, always
turning the same way (counter-clockwise). The spiral is cut into legs, one per
trial (bag): outward legs (r_min -> outside) and inward legs (outside -> r_min).
A leg also ends early where the spiral passes too close to a wall; the next leg
starts where it is clear again.

Deterministic for a given map and parameters, so leg i is the same on a
resumed run.
"""
import math
from pathlib import Path
from typing import List, Tuple

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt

GROWTH = 0.5          # m of radius per loop
CLEARANCE = 0.75      # m from walls (the costmap's inflation radius)
DENSE_STEP = 0.05     # m between points of a leg's path
WAYPOINT_SPACING = 1.0  # m between waypoints along a leg (option A)
MIN_LEG_LENGTH = 1.0  # m; shorter clear stretches are skipped
MAX_LOOPS = 3         # never grows past r_min + MAX_LOOPS * GROWTH


def read_pgm(path: Path) -> np.ndarray:
    data = Path(path).read_bytes()
    parts, i = [], 0
    while len(parts) < 4:                     # magic, width, height, maxval
        while data[i:i + 1].isspace():
            i += 1
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        parts.append(data[i:j])
        i = j
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(data[i + 1:i + 1 + w * h], np.uint8).reshape(h, w)


class Clearance:
    """Distance (m) from a world point to the nearest non-free cell of the Nav2 map."""

    def __init__(self, map_yaml: Path):
        meta = yaml.safe_load(Path(map_yaml).read_text())
        img = read_pgm(Path(map_yaml).parent / meta["image"])
        free = img > 250 if not meta.get("negate", 0) else img < 5
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        self.h, self.w = img.shape
        self.dist = distance_transform_edt(free) * self.res

    def __call__(self, x: float, y: float) -> float:
        c = int((x - self.ox) / self.res)
        r = self.h - 1 - int((y - self.oy) / self.res)
        return float(self.dist[r, c]) if 0 <= r < self.h and 0 <= c < self.w else 0.0


class Leg:
    def __init__(self, direction: str, points: List[Tuple[float, float]], r0: float, r1: float):
        self.direction = direction            # "outward" or "inward"
        self.points = points                  # dense, DENSE_STEP apart
        self.r0, self.r1 = r0, r1

    @property
    def length(self) -> float:
        p = np.asarray(self.points)
        return float(np.hypot(*np.diff(p, axis=0).T).sum()) if len(p) > 1 else 0.0

    def waypoints(self, spacing: float = WAYPOINT_SPACING) -> List[Tuple[float, float]]:
        """Every `spacing` m along the leg, always ending on its last point."""
        p = np.asarray(self.points)
        s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(p, axis=0).T))])
        targets = np.arange(spacing, s[-1], spacing)
        idx = [int(np.searchsorted(s, t)) for t in targets]
        if s[-1] - (targets[-1] if len(targets) else 0.0) < 0.5 * spacing and idx:
            idx = idx[:-1]                    # don't put a waypoint right before the end
        return [tuple(p[i]) for i in idx] + [tuple(p[-1])]


def spiral_legs(r_min: float, centre: Tuple[float, float], clearance: Clearance, n_legs: int,
                growth: float = GROWTH, clr: float = CLEARANCE) -> List[Leg]:
    """The first n_legs legs of the spiral (see module docstring)."""
    cx, cy = centre
    k = growth / (2 * math.pi)                # dr / dtheta
    r_max = r_min + MAX_LOOPS * growth
    th, r, sign = 0.0, r_min, +1
    legs: List[Leg] = []
    cur: List[Tuple[float, float]] = []
    r_start = r

    def close():
        nonlocal cur
        if len(cur) > 1 and Leg("", cur, 0, 0).length >= MIN_LEG_LENGTH:
            legs.append(Leg("outward" if sign > 0 else "inward", cur, r_start, r))
        cur = []

    while len(legs) < n_legs and th < 400 * math.pi:
        x, y = cx + r * math.cos(th), cy + r * math.sin(th)
        if clearance(x, y) >= clr:
            if not cur:
                r_start = r
            cur.append((x, y))
        else:
            close()
            if sign > 0 and r > r_min + 1e-6:  # the outside is reached: turn inward
                sign = -1
        dth = DENSE_STEP / math.hypot(r, k)
        th += dth
        r += sign * k * dth
        if sign < 0 and r <= r_min:            # back at the inner circle: turn outward
            r = r_min
            close()
            sign = +1
        elif sign > 0 and r >= r_max:
            close()
            sign = -1
    return legs[:n_legs]


def route_remaining(route: np.ndarray, cum: np.ndarray, xy, start_idx: int, window_m: float = 3.0):
    """(remaining length along route, index) for the point of route nearest to xy,
    searched only from start_idx to window_m further on, so progress along a
    spiral never jumps to a neighbouring loop."""
    hi = int(np.searchsorted(cum, cum[start_idx] + window_m)) + 1
    seg = route[start_idx:hi]
    j = start_idx + int(np.argmin(np.hypot(seg[:, 0] - xy[0], seg[:, 1] - xy[1])))
    return float(cum[-1] - cum[j]), j
