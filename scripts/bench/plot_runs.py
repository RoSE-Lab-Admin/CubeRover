#!/usr/bin/env python3
"""Per-bag tracks of an autonomous_trials bag_dir (bench or live): driven track
(forward blue / reverse orange), the first (green) and last (purple) /plan,
driven gear changes (black x), goal (red star), and one summary line per bag:
duration, gear changes and the rover heading off E-W at each, reverse distance,
how many plans had cusps, plan cusp headings off E-W.

    python3 scripts/bench/plot_runs.py BAG_DIR OUT.png [--title T]

Needs ROS sourced (rosbag2_py) and matplotlib (system python3; the cuda_end venv has no matplotlib).
"""
import argparse
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.serialization import deserialize_message

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src" / "nav2_stack"))
from nav2_stack.spiral import Clearance  # noqa: E402
from nav2_stack import rover_geometry  # noqa: E402

GEO = rover_geometry.load(REPO / "src" / "nav2_stack" / "config" / "rover_geometry.yaml")

MAP = Clearance(REPO / "src" / "nav2_stack" / "maps" / "map.yaml")
FWD, REV = "#2a78d6", "#eb6834"


def ew(a):
    """Heading off the E-W axis, deg (0 = pointing E or W)."""
    d = abs(math.degrees(math.atan2(math.sin(a), math.cos(a))))
    return min(d, 180 - d)


def plan_cusps(p):
    """Poses of a plan where the gear flips (a step against the pose yaw is reverse)."""
    if len(p) < 3:
        return p[:0]
    d = np.diff(p[:, :2], axis=0)
    g = np.where(d[:, 0] * np.cos(p[:-1, 2]) + d[:, 1] * np.sin(p[:-1, 2]) >= 0, 1, -1)
    return p[np.nonzero(np.diff(g))[0] + 1]


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/FitRosey_passive/pose", "/roseybot_base_controller/cmd_vel_out", "/plan"]))
    P, C, PL = [], [], []
    while r.has_next():
        t, d, ts = r.read_next(); ts *= 1e-9
        if t == "/FitRosey_passive/pose":
            m = deserialize_message(d, PoseStamped); q = m.pose.orientation
            P.append((ts, m.pose.position.x, m.pose.position.y, q.x, q.y, q.z, q.w))
        elif t == "/roseybot_base_controller/cmd_vel_out":
            m = deserialize_message(d, TwistStamped); C.append((ts, m.twist.linear.x))
        else:
            m = deserialize_message(d, PathMsg)
            if m.poses:
                PL.append(np.array([[p.pose.position.x, p.pose.position.y,
                                     2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w)] for p in m.poses]))
    P = np.array(P)
    if len(P):
        # rigid body (rear-right marker) -> rover base_link: true position and heading
        pos, rot = rover_geometry.base_link_poses(GEO, np.column_stack([P[:, 1:3], np.zeros(len(P))]), P[:, 3:7])
        P = np.column_stack([P[:, 0], pos[:, :2], rover_geometry.yaw_pitch_roll(rot)[0]])
    return P, np.array(C), PL


def goal_of(bag_dir, bag):
    if bag.name == "initial_bag":
        f = bag_dir / "initial_goal.csv"
    else:
        f = next(iter(sorted(bag_dir.glob(f"traj_{bag.name.split('_')[1]}*.csv"))), None)
    if f is None or not f.exists():
        return None
    g = np.loadtxt(f, delimiter=",", skiprows=1, ndmin=2)
    return g[0, :2] if bag.name == "initial_bag" else g[-1, :2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_dir"); ap.add_argument("out"); ap.add_argument("--title", default="")
    a = ap.parse_args()
    D = Path(a.bag_dir)
    bags = sorted([p.parent for p in D.rglob("metadata.yaml")], key=lambda p: (p.name != "initial_bag", p.name))
    if not bags:
        print("no recorded bags"); return
    cols = min(6, len(bags)); rows = math.ceil(len(bags) / cols)
    fig, axs = plt.subplots(rows, cols, figsize=(cols * 3.4, rows * 3.1 + 0.4), squeeze=False)
    axs = axs.ravel(); lines = []
    for ax, bag in zip(axs, bags):
        P, C, PL = read(bag)
        ax.imshow(np.where(MAP.dist > 0, 1.0, 0.0), cmap="gray", vmin=-0.5, vmax=1, origin="upper",
                  extent=(MAP.ox, MAP.ox + MAP.w * MAP.res, MAP.oy, MAP.oy + MAP.h * MAP.res))
        ax.set_xlim(-4, 3); ax.set_ylim(-2.8, 3.2); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
        if len(P) < 10:
            ax.set_title(f"{bag.name}: no poses", fontsize=8); lines.append(f"{bag.name}: no poses"); continue
        vx = np.interp(P[:, 0], C[:, 0], C[:, 1]) if len(C) else np.zeros(len(P))
        s = np.sign(np.where(np.abs(vx) > 0.02, vx, 0)); nz = np.nonzero(s)[0]
        cusp = [nz[k] for k in range(1, len(nz)) if s[nz[k]] != s[nz[k - 1]]]
        dl = np.hypot(np.diff(P[:, 1]), np.diff(P[:, 2])); rev = vx[:-1] < -0.02
        ax.scatter(P[:, 1], P[:, 2], c=np.where(vx < -0.02, REV, FWD), s=1.5, zorder=3)
        for k in cusp:
            ax.plot(P[k, 1], P[k, 2], "kx", ms=6, zorder=5)
        for p, col in ((PL[0], "#1baf7a"), (PL[-1], "#9b3fd1")) if PL else ():
            ax.plot(p[:, 0], p[:, 1], color=col, lw=1, alpha=0.8, zorder=2)
        g = goal_of(D, bag)
        if g is not None:
            ax.plot(g[0], g[1], "r*", ms=9, zorder=6)
        hds = ",".join(f"{ew(P[k, 3]):.0f}" for k in cusp) or "-"
        pc = [ew(c[2]) for p in PL for c in plan_cusps(p)]
        ax.set_title(f"{bag.name}: {P[-1, 0] - P[0, 0]:.0f} s, {len(cusp)} gear changes\nheading off E-W at them: {hds}", fontsize=7.5)
        lines.append(f"{bag.name}: {P[-1, 0] - P[0, 0]:.0f} s, {len(cusp)} gear changes (heading off E-W: {hds}), "
                     f"reverse {dl[rev].sum():.1f} m, {len(PL)} plans ({sum(len(plan_cusps(p)) > 0 for p in PL)} with cusps"
                     + (f", plan cusps off E-W median {np.median(pc):.0f} / max {max(pc):.0f} deg" if pc else "")
                     + f"), ends heading {ew(P[-1, 3]):.0f} deg off E-W"
                     + (f", {math.hypot(P[-1, 1] - g[0], P[-1, 2] - g[1]):.2f} m from the goal" if g is not None else ""))
    for ax in axs[len(bags):]:
        ax.axis("off")
    if a.title:
        fig.suptitle(a.title, fontsize=10)
    fig.tight_layout(); fig.savefig(a.out, dpi=100)
    print("\n".join(lines))


if __name__ == "__main__":
    main()
