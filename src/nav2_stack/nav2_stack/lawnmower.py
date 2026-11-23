"""Lawnmower legs for autonomous_trials' waypoint_mode "lawnmower" (lawnmower_drive path: S-curve
lanes driven in path mode; zigzag / zigzag_heading: one planned goal per lane, see zigzag_legs).

Lanes run along x, `spacing` apart in y. Each lane is the longest stretch at its height that keeps
CLEARANCE from everything occupied in the Nav2 map (walls and craters), at least MIN_LANE long; only
the largest block of consecutive lanes is used. Lane changes are S-curves of two R_TURN arcs that
shift the rover one lane sideways while it keeps its heading, so with path mode's one-gear-per-leg
rule the lanes alternate forward / reverse and there are no turnarounds.

One leg (one trial / bag) = the S-curve onto a lane + the lane. Passes alternate up and down; every
pass shifts its lanes by spacing / PASS_SHIFTS against the previous one (cycling through PASS_SHIFTS
positions), so consecutive passes drive the strips in between instead of the same lanes again. The
number of lanes per pass is whatever fits at its positions. Between passes the rover changes onto the
next pass's lane nearest to it with a shorter S-curve. Deterministic for a given map and spacing, so
leg i is the same on a resumed run.
"""
import math
from typing import List, Tuple

import numpy as np

from nav2_stack.spiral import CLEARANCE, DENSE_STEP

SPACING = 0.8      # m between lanes
R_TURN = 1.5       # m, S-curve arcs (the planner's minimum turning radius)
MIN_LANE = 3.0     # m, shorter lanes are left out
PASS_SHIFTS = 3    # lane positions per spacing: each pass is shifted by spacing / PASS_SHIFTS
# zigzag lane ends (the planner's goals) must fit the whole rover lying along the lane: the end point's
# CLEARANCE alone let ends sit ~0.4 m from a crater edge, where no E-W pose is collision-free and the
# planner fails ("no valid path found", 11/23 bag_05 / bag_07 at CraterNE's x = 1.00 ends). Each end is
# pulled in along the lane in END_STEP steps until the footprint fits at every heading within
# END_YAW_TOL of E-W, END_MARGIN clear of occupied map cells.
ROVER_HALF = (0.37, 0.23)   # m, half length / width of the costmaps' footprint (nav2_param2.yaml)
END_MARGIN = 0.05           # m (the global costmap's resolution)
END_YAW_TOL = 20.0          # deg, = lawnmower_heading_tolerance
END_STEP = 0.05             # m


def _footprint_fits(clearance, x: float, y: float, yaw_tol_deg: float = END_YAW_TOL) -> bool:
    """The rover footprint (+ END_MARGIN) centred at (x, y) touches no non-free map cell at any
    heading within yaw_tol_deg of E-W (5 deg steps; the rectangle is symmetric, so W is the same)."""
    hl, hw = ROVER_HALF[0] + END_MARGIN, ROVER_HALF[1] + END_MARGIN
    u, v = np.meshgrid(np.linspace(-hl, hl, int(2 * hl / 0.02) + 1),
                       np.linspace(-hw, hw, int(2 * hw / 0.02) + 1))
    u, v = u.ravel(), v.ravel()
    n = int(math.ceil(yaw_tol_deg / 5.0))
    for yaw in np.radians(np.linspace(-yaw_tol_deg, yaw_tol_deg, 2 * n + 1) if n else [0.0]):
        px = x + math.cos(yaw) * u - math.sin(yaw) * v
        py = y + math.sin(yaw) * u + math.cos(yaw) * v
        c = np.floor((px - clearance.ox) / clearance.res).astype(int)
        r = clearance.h - 1 - np.floor((py - clearance.oy) / clearance.res).astype(int)
        inside = (c >= 0) & (c < clearance.w) & (r >= 0) & (r < clearance.h)
        if not inside.all() or (clearance.dist[r, c] <= 0).any():
            return False
    return True


def _fit_ends(clearance, y: float, a: float, b: float, yaw_tol_deg: float):
    """(a, b) pulled in until the footprint fits at both ends (see ROVER_HALF). Applied to the
    lanes already chosen, so the lanes / passes themselves stay as without it."""
    while b - a > 1.0 and not _footprint_fits(clearance, a, y, yaw_tol_deg):
        a += END_STEP
    while b - a > 1.0 and not _footprint_fits(clearance, b, y, yaw_tol_deg):
        b -= END_STEP
    return a, b


class LaneLeg:
    def __init__(self, points, lane: int, n_lanes: int, direction: int, going_up: bool,
                 pass_no: int = 1, shift: float = 0.0):
        self.points = points                  # dense, DENSE_STEP apart
        self.lane, self.n_lanes = lane, n_lanes
        self.direction, self.going_up = direction, going_up
        self.pass_no, self.shift = pass_no, shift

    @property
    def length(self) -> float:
        p = np.asarray(self.points)
        return float(np.hypot(*np.diff(p, axis=0).T).sum()) if len(p) > 1 else 0.0

    @property
    def label(self) -> str:
        return (f"pass {self.pass_no} ({'up' if self.going_up else 'down'}, lanes shifted "
                f"+{self.shift:.2f} m) lane {self.lane + 1}/{self.n_lanes} towards "
                f"{'+x' if self.direction > 0 else '-x'}")


def s_curve(x0: float, y0: float, direction: int, dy: float, r: float = R_TURN) -> List[Tuple[float, float]]:
    """S-curve from (x0, y0) travelling along x in `direction` (+1/-1), shifting dy sideways."""
    theta = math.acos(1.0 - abs(dy) / (2.0 * r))
    n = int(math.ceil(r * theta / DENSE_STEP))
    ds = r * theta / n
    x, y, th = x0, y0, 0.0 if direction > 0 else math.pi
    first = math.copysign(1.0, dy) * direction       # +1: turn left first (towards the new lane)
    pts = []
    for turn in (first, -first):
        for _ in range(n):
            mid = th + turn * ds / (2.0 * r)
            x += ds * math.cos(mid); y += ds * math.sin(mid); th += turn * ds / r
            pts.append((x, y))
    return pts


def _lanes_at(clearance, spacing, clr, off):
    """Largest block of consecutive lanes at y = map bottom + off + k * spacing."""
    ok = lambda x, y: clearance(x, y) >= clr
    xs = np.arange(clearance.ox, clearance.ox + clearance.w * clearance.res, DENSE_STEP)
    y_lo, y_hi = clearance.oy, clearance.oy + clearance.h * clearance.res
    lanes = []
    for y in np.arange(y_lo + off, y_hi, spacing):
        good = np.array([ok(x, y) for x in xs])
        runs, start = [], None
        for i, v in enumerate(np.r_[good, False]):
            if v and start is None:
                start = i
            if not v and start is not None:
                runs.append((xs[start], xs[i - 1])); start = None
        run = max(runs, key=lambda r: r[1] - r[0]) if runs else None
        lanes.append((float(y), *run) if run and run[1] - run[0] >= MIN_LANE else None)
    blocks, cur = [], []
    for lane in lanes + [None]:
        if lane:
            cur.append(lane)
        elif cur:
            blocks.append(cur); cur = []
    return max(blocks, key=lambda b: (len(b), sum(l[2] - l[1] for l in b))) if blocks else []


def _lanes(clearance, spacing, clr):
    """(offset, lanes) with the most lanes (then the most lane length), offsets in 0.1 m steps."""
    best = (0.0, [])
    for off in np.arange(0.0, spacing, 0.1):
        b = _lanes_at(clearance, spacing, clr, off)
        if (len(b), sum(l[2] - l[1] for l in b)) > (len(best[1]), sum(l[2] - l[1] for l in best[1])):
            best = (float(off), b)
    return best


def _sequence(clearance, spacing, clr, n, fit_yaw=None):
    """The lanes of the first n legs: (y, a, b, pass_no, shift, index in pass, lanes in pass, up).
    fit_yaw (deg, zigzag): every lane end pulled in until the rover fits there within fit_yaw of
    E-W (_fit_ends); None = as found."""
    base, first = _lanes(clearance, spacing, clr)
    if len(first) < 2:
        return []
    passes = {}
    seq, p, last_y = [], 0, None
    while len(seq) < n:
        k = p % PASS_SHIFTS
        if k not in passes:
            off = (base + k * spacing / PASS_SHIFTS) % spacing
            passes[k] = first if k == 0 else _lanes_at(clearance, spacing, clr, off)
            if fit_yaw is not None:
                passes[k] = [(y, *_fit_ends(clearance, y, a, b, fit_yaw)) for y, a, b in passes[k]]
        lanes = passes[k]
        if len(lanes) < 2:                      # a shift with too few lanes: skip it
            p += 1
            continue
        up = p % 2 == 0
        order = sorted(range(len(lanes)), key=lambda j: lanes[j][0], reverse=not up)
        if last_y is not None:                  # start at the lane nearest the rover, never the same one
            cand = [j for j in order if abs(lanes[j][0] - last_y) > 1e-6]
            j0 = min(cand, key=lambda j: abs(lanes[j][0] - last_y))
            order = order[order.index(j0):]
        for j in order:
            y, a, b = lanes[j]
            shift = (k * spacing / PASS_SHIFTS)
            seq.append((y, a, b, p + 1, shift, j, len(lanes), up))
            last_y = y
        p += 1
    return seq[:n]


def lawnmower_legs(clearance, n_legs: int, spacing: float = SPACING, clr: float = CLEARANCE) -> List[LaneLeg]:
    """The first n_legs legs (see module docstring); [] if fewer than 2 lanes fit."""
    seq = _sequence(clearance, spacing, clr, n_legs + 1)
    if len(seq) < 2:
        return []
    ok = lambda pts: all(clearance(x, y) >= clr for x, y in pts)
    legs, d, x_end = [], +1, None
    for i in range(min(n_legs, len(seq) - 1)):
        yc, ac, bc, pass_no, shift, j, n_in_pass, up = seq[i]
        yn, an, bn = seq[i + 1][:3]
        if i == 0:
            head, x0 = [], ac
        else:
            head = s_curve(x_end, seq[i - 1][0], d, yc - seq[i - 1][0])
            x0 = head[-1][0]
        far = bc if d > 0 else ac                       # lane end in the travel direction ...
        far = min(far, bn) if d > 0 else max(far, an)   # ... where the next lane is still there
        while not ok(s_curve(far, yc, -d, yn - yc)) and (far - x0) * d > 0.5:
            far -= 0.1 * d                              # pull in until the next S-curve fits
        lane = [(x, yc) for x in np.arange(x0, far + 1e-9 * d, DENSE_STEP * d)]
        legs.append(LaneLeg(head + lane, j, n_in_pass, d, up, pass_no, shift))
        x_end, d = far, -d
    return legs


class ZigzagLeg:
    """One zigzag trial: a single goal, the end of the next lane on the other side
    (planned and driven like a GP goal). points = [where the leg starts, the goal]."""

    def __init__(self, start, goal, lane: int, n_lanes: int, direction: int, going_up: bool,
                 pass_no: int, shift: float):
        self.points = [tuple(start), tuple(goal)]
        self.lane, self.n_lanes = lane, n_lanes
        self.direction, self.going_up = direction, going_up    # direction: +1 = goal on the east side
        self.pass_no, self.shift = pass_no, shift

    @property
    def length(self) -> float:
        (x0, y0), (x1, y1) = self.points
        return math.hypot(x1 - x0, y1 - y0)

    @property
    def yaw(self) -> float:
        """Heading along the lanes in the direction of travel (E = 0, W = pi)."""
        return 0.0 if self.direction > 0 else math.pi

    @property
    def label(self) -> str:
        return (f"zigzag pass {self.pass_no} ({'up' if self.going_up else 'down'}, lanes shifted "
                f"+{self.shift:.2f} m) to lane {self.lane + 1}/{self.n_lanes}'s "
                f"{'east' if self.direction > 0 else 'west'} end")


def zigzag_start(clearance, spacing: float = SPACING, clr: float = CLEARANCE,
                 end_yaw_tol: float = END_YAW_TOL):
    """Where the zigzag starts (the first lane's west end) and a point 1 m east of it."""
    seq = _sequence(clearance, spacing, clr, 1, end_yaw_tol)
    if not seq:
        return None
    y, a = seq[0][0], seq[0][1]
    return [(a, y), (a + 1.0, y)]


def zigzag_legs(clearance, n_legs: int, spacing: float = SPACING, clr: float = CLEARANCE,
                end_yaw_tol: float = END_YAW_TOL) -> List[ZigzagLeg]:
    """Zigzag across the lanes of lawnmower_legs (same passes and shifts, but every lane end
    pulled in until the rover fits there along E-W within end_yaw_tol, see ROVER_HALF): start
    at the first lane's west end, then every leg's goal is the next lane's end on the
    other side -- east, west, east, ... -- one lane further north (south on the way
    back). [] if fewer than 2 lanes fit."""
    seq = _sequence(clearance, spacing, clr, n_legs + 1, end_yaw_tol)
    if len(seq) < 2:
        return []
    legs, d = [], +1
    prev = (seq[0][1], seq[0][0])
    for i in range(1, min(n_legs + 1, len(seq))):
        y, a, b, pass_no, shift, j, n_in_pass, up = seq[i]
        goal = (b if d > 0 else a, y)
        legs.append(ZigzagLeg(prev, goal, j, n_in_pass, d, up, pass_no, shift))
        prev, d = goal, -d
    return legs
