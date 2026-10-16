#!/usr/bin/env python3
"""Rate-limiter comparison from mppi_trial.py npz files: outcome per config,
plus how well MPPI's own prediction (/optimal_trajectory_model, i.e. the
optimal sequence run through the configured dynamics) matches what the rover
then actually drove over the same 2.5 s, and how jittery the turn command is.

    python3 rl_analyze.py rl_results
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def prediction_error(d):
    P = d['pose']
    om, ot = d['opt_model'], d['opt_model_t']
    # predictions are stamped in ROS time, poses in the trial script's clock:
    # find the offset by matching each prediction's start to the track
    speed = np.r_[0, np.hypot(*np.diff(P[:, 1:3], axis=0).T)]
    offs = []
    for t, traj in list(zip(ot, om))[::5]:
        traj = np.asarray(traj, dtype=float)
        i = int(np.argmin(np.hypot(P[:, 1] - traj[0, 0], P[:, 2] - traj[0, 1])))
        if speed[i] > 1e-4:
            offs.append(t - P[i, 0])
    if not offs:
        return np.nan, np.nan
    ot = np.asarray(ot) - np.median(offs)
    eh, ep = [], []
    for t, traj in zip(ot, om):
        traj = np.asarray(traj, dtype=float)
        if len(traj) < 2:
            continue
        H = len(traj)
        a = np.searchsorted(P[:, 0], t)
        b = np.searchsorted(P[:, 0], t + 0.1 * H)
        if b >= len(P) or a >= len(P) or P[b, 0] - P[a, 0] < 0.1 * H - 0.2:
            continue
        # predicted end pose vs real pose 0.1*H s later
        eh.append(abs(np.degrees(wrap(traj[-1, 2] - P[b, 3]))))
        ep.append(np.hypot(traj[-1, 0] - P[b, 1], traj[-1, 1] - P[b, 2]))
    return (float(np.mean(eh)) if eh else np.nan, float(np.mean(ep)) if ep else np.nan)


def main():
    out = Path(sys.argv[1] if len(sys.argv) > 1 else 'rl_results')
    rows = defaultdict(list)
    for f in sorted(out.glob('*.npz')):
        cfg, scen, rep = f.stem.split('__')
        d = np.load(f, allow_pickle=True)
        m = json.loads(str(d['metrics']))
        C = d['cmd']
        jitter = float(np.abs(np.diff(C[:, 2])).mean()) if len(C) > 2 else np.nan
        eh, ep = prediction_error(d)
        r = dict(scen=scen, reached=m.get('status') == 'reached', time=m.get('time', np.nan),
                 gears=m.get('gear_changes', 0), jitter=jitter, pred_head=eh, pred_pos=ep)
        rows[cfg].append(r)
        print(f'{cfg:12s} {scen:10s} {rep}  {m.get("status", "?"):8s} t={r["time"]:5.1f}s gears={r["gears"]:3d} '
              f'wz jitter {jitter:.3f}  prediction err after 2.5 s: heading {eh:5.1f} deg, position {ep:.2f} m')
    print('\nper config (means)')
    print(f'{"config":12s} reached  time(reached)  gears  wz jitter  pred heading err  pred position err')
    for cfg, rs in rows.items():
        rc = [r for r in rs if r['reached']]
        mean = lambda k: np.nanmean([r[k] for r in rs])
        print(f'{cfg:12s} {len(rc)}/{len(rs)}    {np.mean([r["time"] for r in rc]) if rc else np.nan:6.1f} s'
              f'      {mean("gears"):5.1f}  {mean("jitter"):8.3f}   {mean("pred_head"):8.1f} deg      '
              f'{mean("pred_pos"):6.2f} m')


if __name__ == '__main__':
    main()
