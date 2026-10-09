#!/usr/bin/env python3
"""Scores the runs recorded by arc_follow_test.py (reads <out>/runs.csv + bags).

Per run:
  plan_R      radius of the first planned arc (circle fit to the first /plan)
  driven_R    circle fit to the driven track while it was turning
  dev_mean/max  distance of the driven track from that first planned arc (m)
  turned/sweep  heading change driven vs planned (deg); turned is short of sweep
              even on a perfect run, since the goal counts as reached 0.5 m early
  rejects     planner "arc rejected" lines during the run (-> Hybrid-A*, 3-point turns)
  wz_mean/sat mean |wz| commanded while moving, fraction of time |wz| >= 0.55
  gears       forward/reverse switches
  held        THE VERDICT: the planner kept the arc (no "arc rejected" while the
              rover was still more than END_ZONE m from the goal) and the track
              stayed within MAX_DEV m of the first planned arc up to that point.
              "reached" alone is not enough: with the 0.5 m goal tolerance a
              missed arc plus half a 3-point turn still ends within 0.5 m
  held_frac   fraction of the arc length driven before the first rejection
              (1.0 if it held)
Writes <out>/summary.csv and <out>/arcs.png.

    source install/setup.bash
    python3 scripts/arc_test/arc_follow_analyze.py ~/AL_scripts/arc_tests_MMDD
"""
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
try:  # the cuda_end venv has no matplotlib: then just skip the figure
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    plt = None
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ("/FitRosey_V1/pose", "/cmd_vel", "/plan", "/rosout")
MAX_DEV = 0.25    # m, max distance from the first planned arc for a "held" run
END_ZONE = 1.0    # m from the goal: rejections this close to the end don't count
                  # (over the last ~1 m the needed radius swings wildly with tiny
                  # heading errors -- seen in the sim even on a 4 cm-accurate run)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=""),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    P, C, plans, logs = [], [], [], []
    while r.has_next():
        topic, raw, t = r.read_next()
        m = deserialize_message(raw, get_message(types[topic]))
        t *= 1e-9
        if topic == "/FitRosey_V1/pose":
            P.append((t, m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation)))
        elif topic == "/cmd_vel":
            C.append((t, m.twist.linear.x, m.twist.angular.z))
        elif topic == "/plan" and m.poses:
            plans.append((t, np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses])))
        elif topic == "/rosout" and m.name == "planner_server" and "GridBasedCustom: arc" in m.msg:
            # stamp, not receive time: /rosout is transient-local, so a new bag
            # first receives the planner's older lines (e.g. the previous run)
            logs.append((m.stamp.sec + m.stamp.nanosec * 1e-9, m.msg.split("GridBasedCustom: ", 1)[1]))
    return np.array(P), np.array(C), plans, logs


def fit_circle(xy):
    if len(xy) < 5:
        return math.nan, None
    x, y = xy[:, 0], xy[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]
    (cx, cy, c), *_ = np.linalg.lstsq(A, x ** 2 + y ** 2, rcond=None)
    return math.sqrt(max(c + cx ** 2 + cy ** 2, 0)), (cx, cy)


def dist_to_poly(pts, poly):
    a, b = poly[:-1], poly[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    out = []
    for p in pts:
        u = np.clip(((p - a) * ab).sum(1) / L2, 0, 1)
        out.append(np.hypot(*(a + u[:, None] * ab - p).T).min())
    return np.array(out)


def score(row, bag):
    P, C, plans, logs = read_bag(bag)
    t_start = float(row["t_start"])
    plans = [p for p in plans if p[0] >= t_start]
    logs = [l for l in logs if l[0] >= t_start]
    res = dict(run=row["run"], dynamics=row.get("dynamics") or "nn", time_steps=int(row["time_steps"]), radius=float(row["radius"]),
               side=row["side"], sweep=float(row["sweep_deg"]), outcome=row["outcome"],
               duration=float(row["t_end"]) - float(row["t_start"]))
    if len(P) < 10 or not plans or res["duration"] < 5.0:
        res["invalid"] = True   # e.g. ended at once on a stale goal result: nothing driven
        return res, None
    moving = np.r_[False, np.hypot(*np.diff(P[:, 1:3], axis=0).T) > 1e-4]
    Pm = P[moving] if moving.sum() > 10 else P
    plan0 = plans[0][1]
    res["plan_R"] = fit_circle(plan0)[0]
    # the turning part: while the heading is still changing (drop the final straight-ish tail)
    hd = np.unwrap(Pm[:, 3])
    turned = hd - hd[0]
    k = np.searchsorted(np.abs(turned) if turned[-1] >= 0 else -turned, 0.9 * abs(turned[-1])) + 1
    res["driven_R"] = fit_circle(Pm[:max(k, 10), 1:3])[0]
    dev = dist_to_poly(Pm[::5, 1:3], plan0)
    res["dev_mean"], res["dev_max"] = float(dev.mean()), float(dev.max())
    res["turned"] = float(abs(math.degrees(turned[-1])))
    res["rejects"] = sum("rejected" in m for _, m in logs)
    # verdict: arc kept until the rover was near the goal, and tracked closely until then
    goal = np.array([float(row["goal_x"]), float(row["goal_y"])])
    def dist_goal(t):
        i = min(np.searchsorted(P[:, 0], t), len(P) - 1)
        return float(np.hypot(*(P[i, 1:3] - goal)))
    t_fail = next((t for t, m in logs if "rejected" in m and dist_goal(t) > END_ZONE), None)
    upto = Pm[Pm[:, 0] <= t_fail] if t_fail is not None else Pm
    dev_held = dist_to_poly(upto[::5, 1:3], plan0) if len(upto) > 5 else np.array([np.inf])
    seg = np.hypot(*np.diff(plan0, axis=0).T)
    s_plan = np.r_[0, np.cumsum(seg)]
    if len(upto):
        last = upto[-1, 1:3]
        j = int(np.argmin(np.hypot(*(plan0 - last).T)))
        res["held_frac"] = float(s_plan[j] / s_plan[-1]) if t_fail is not None else 1.0
    else:
        res["held_frac"] = 0.0
    res["held"] = bool(t_fail is None and dev_held.max() <= MAX_DEV)
    if len(C):
        mv = np.abs(C[:, 1]) > 0.02
        wz = np.abs(C[mv, 2]) if mv.any() else np.zeros(1)
        res["wz_mean"], res["wz_sat"] = float(wz.mean()), float((wz >= 0.55).mean())
        g = np.sign(C[:, 1][np.abs(C[:, 1]) > 0.02])
        res["gears"] = int((np.diff(g) != 0).sum()) if len(g) > 1 else 0
    return res, (P, plans)


def main():
    out = Path(sys.argv[1]).expanduser()
    rows = list(csv.DictReader(open(out / "runs.csv")))
    results, tracks = [], []
    for row in rows:
        bag = out / row["run"]
        if not bag.exists():
            print(f"missing bag {bag}")
            continue
        r, tr = score(row, bag)
        if r.get("invalid"):
            print(f"invalid run (nothing driven, {r['duration']:.0f} s) -- excluded: {r['run']}")
            continue
        results.append(r)
        tracks.append((r, tr))
    cols = ["run", "dynamics", "time_steps", "radius", "outcome", "duration", "plan_R", "driven_R", "dev_mean",
            "dev_max", "sweep", "turned", "rejects", "held", "held_frac", "wz_mean", "wz_sat", "gears"]
    with open(out / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(results)
    fmt = lambda v: f"{v:.2f}" if isinstance(v, float) else str(v)
    print(f"{'run':40s} held  frac out      t   planR drivR devM devX sweep turn rej wz   sat  gears")
    for r in results:
        print(f"{r['run']:40s} {('YES' if r.get('held') else 'no'):4s} {r.get('held_frac', 0):5.2f} "
              f"{r['outcome'][:7]:7s} {r['duration']:4.0f} "
              + " ".join(fmt(r.get(k, math.nan)).rjust(5) for k in
                         ("plan_R", "driven_R", "dev_mean", "dev_max", "sweep", "turned", "rejects",
                          "wz_mean", "wz_sat", "gears")))
    print("\nper model, horizon and radius (means)")
    grp = defaultdict(list)
    for r in results:
        grp[(r["dynamics"], r["time_steps"], r["radius"])].append(r)
    print(f"{'model':10s} {'steps':>5} {'R':>5} {'n':>2} HELD  held_frac reached  drivenR  dev_mean dev_max  rejects  gears  wz_sat")
    for (dyn, ts, R), rs in sorted(grp.items()):
        m = lambda k: np.nanmean([x.get(k, math.nan) for x in rs])
        print(f"{dyn:10s} {ts:5d} {R:5.2f} {len(rs):2d} {sum(bool(x.get('held')) for x in rs)}/{len(rs)}"
              f"   {m('held_frac'):5.2f}    {sum(x['outcome'] == 'reached' for x in rs)}/{len(rs)}"
              f"     {m('driven_R'):5.2f}    {m('dev_mean'):5.2f}   {m('dev_max'):5.2f}"
              f"    {m('rejects'):5.1f}  {m('gears'):5.1f}   {m('wz_sat'):4.2f}")
    n = len(tracks)
    if n and plt is None:
        print("\n(no matplotlib in this Python -- figure skipped)")
    elif n:
        cols_ = min(n, 4)
        fig, axes = plt.subplots((n + cols_ - 1) // cols_, cols_, figsize=(4.2 * cols_, 4.2 * ((n + cols_ - 1) // cols_)),
                                 squeeze=False)
        for ax, (r, tr) in zip(axes.ravel(), tracks):
            if tr is None:
                ax.set_title(f"{r['run']}\n(no data)", fontsize=8)
                continue
            P, plans = tr
            ax.plot(plans[0][1][:, 0], plans[0][1][:, 1], "b-", lw=2, label="first plan")
            ax.plot(P[:, 1], P[:, 2], "k-", lw=1.5, label="driven")
            ax.plot(P[0, 1], P[0, 2], "go")
            ax.set_aspect("equal")
            ax.set_title(f"{r['run']}\n{'HELD' if r.get('held') else 'not held'} ({r.get('held_frac', 0):.2f})  {r['outcome']}  driven R {r.get('driven_R', math.nan):.2f}  "
                         f"dev {r.get('dev_mean', math.nan):.2f}/{r.get('dev_max', math.nan):.2f}  "
                         f"rej {r.get('rejects', 0)}", fontsize=8)
        for ax in axes.ravel()[n:]:
            ax.axis("off")
        axes[0, 0].legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(out / "arcs.png", dpi=70)
        print(f"\nfigure: {out / 'arcs.png'}")


if __name__ == "__main__":
    main()
