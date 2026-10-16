#!/usr/bin/env python3
"""Turning commitment from mppi_trial.py npz files.

Per controller step: heading change over the horizon of the model's optimal
trajectory vs. the heading change of the current plan over the same arc length.
ratio = model / plan (1 = MPPI plans to turn as much as the global path does).
Also the distance of the driven track from the FIRST plan (the arc the rover
was supposed to follow).

    python3 turn_analyze.py turn_results
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def plan_heading_change(plan, x, y, length):
    xy = plan[:, :2].astype(float)
    i0 = np.argmin(np.hypot(xy[:, 0] - x, xy[:, 1] - y))
    seg = xy[i0:]
    if len(seg) < 3:
        return np.nan
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(seg, axis=0).T))]
    j = np.searchsorted(s, length)
    seg = seg[:max(min(j, len(seg)), 3)]
    h = np.arctan2(np.diff(seg[:, 1]), np.diff(seg[:, 0]))
    return abs(np.sum(wrap(np.diff(h))))


def dist_to_poly(pts, poly):
    a, b = poly[:-1], poly[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    out = []
    for p in pts:
        u = np.clip(((p - a) * ab).sum(1) / L2, 0, 1)
        q = a + u[:, None] * ab
        out.append(np.hypot(*(q - p).T).min())
    return np.array(out)


def analyze(f):
    d = np.load(f, allow_pickle=True)
    m = json.loads(str(d['metrics']))
    plans, pt = d['plans'], d['plan_t']
    om, ot = d['opt_model'], d['opt_model_t']
    model_dh, plan_dh = [], []
    for t, traj in zip(ot, om):
        traj = np.asarray(traj, dtype=float)
        j = np.searchsorted(pt, t) - 1
        if j < 0 or len(traj) < 2:
            continue
        L = np.hypot(*np.diff(traj[:, :2], axis=0).T).sum()
        if L < 0.05:
            continue
        model_dh.append(abs(wrap(traj[-1, 2] - traj[0, 2])))
        plan_dh.append(plan_heading_change(np.asarray(plans[j], dtype=float), traj[0, 0], traj[0, 1], L))
    model_dh, plan_dh = np.degrees(model_dh), np.degrees(plan_dh)
    turning = plan_dh > 5.0  # only steps where the plan actually curves
    P = d['pose'][:, 1:3]
    off = dist_to_poly(P[::10], np.asarray(plans[0], dtype=float)[:, :2]) if len(plans) and len(P) else np.zeros(1)
    return dict(reached=m.get('status') == 'reached', time=m.get('time', np.nan),
                gears=m.get('gear_changes', 0),
                plan_deg=np.nanmean(plan_dh[turning]) if turning.any() else np.nan,
                model_deg=np.nanmean(model_dh[turning]) if turning.any() else np.nan,
                off_first=float(np.mean(off)), wz=m.get('mean_abs_wz_cmd', np.nan))


def main():
    out = Path(sys.argv[1] if len(sys.argv) > 1 else 'turn_results')
    rows = defaultdict(list)
    per = []
    for f in sorted(out.glob('*.npz')):
        cfg, scen, rep = f.stem.split('__')
        r = analyze(f)
        rows[cfg].append(r)
        per.append((cfg, scen, rep, r))
    print(f'{"config":16s} {"scen":10s} rep reached  time gears  plan°  model°  ratio  off1st |wz|')
    for cfg, scen, rep, r in per:
        print(f'{cfg:16s} {scen:10s} {rep:>3s} {str(r["reached"]):7s} {r["time"]:5.1f} {r["gears"]:5d} '
              f'{r["plan_deg"]:6.1f} {r["model_deg"]:6.1f} {r["model_deg"] / r["plan_deg"]:6.2f} '
              f'{r["off_first"]:6.2f} {r["wz"]:.2f}')
    print('\nper config (means over scenarios/repeats)')
    print(f'{"config":16s} reached  mean time(reached)  gears  plan°  model°  ratio  off1st |wz|')
    for cfg, rs in rows.items():
        rc = [r for r in rs if r['reached']]
        pd = np.nanmean([r['plan_deg'] for r in rs]); md = np.nanmean([r['model_deg'] for r in rs])
        print(f'{cfg:16s} {len(rc)}/{len(rs)}    {np.mean([r["time"] for r in rc]) if rc else np.nan:6.1f}'
              f'            {np.mean([r["gears"] for r in rs]):5.1f} {pd:6.1f} {md:6.1f} {md / pd:6.2f} '
              f'{np.mean([r["off_first"] for r in rs]):6.2f} {np.mean([r["wz"] for r in rs]):.2f}')


if __name__ == '__main__':
    main()
