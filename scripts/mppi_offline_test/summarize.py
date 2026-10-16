#!/usr/bin/env python3
"""Summarize mppi_trial.py runs: per (config, scenario) mean/spread table and
a track plot per run (rover track coloured by gear, first plan, goal).

    python3 summarize.py results/step0 results/s2_s4 ... --plot out.png [--scenarios bag14_t20 ...]
"""
import argparse
import json
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def load(dirs):
    runs = []
    for d in dirs:
        for f in sorted(Path(d).glob('*.npz')):
            z = np.load(f, allow_pickle=True)
            cfg, scen, rep = f.stem.split('__')
            runs.append(dict(config=cfg, scenario=scen, rep=int(rep), pose=z['pose'], cmd=z['cmd'],
                             goal=tuple(z['goal']), plans=z['plans'], m=json.loads(str(z['metrics']))))
    return runs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dirs', nargs='+')
    ap.add_argument('--plot', default='')
    ap.add_argument('--scenarios', nargs='*', default=None)
    ap.add_argument('--configs', nargs='*', default=None)
    a = ap.parse_args()
    runs = load(a.dirs)
    if a.scenarios:
        runs = [r for r in runs if r['scenario'] in a.scenarios]
    if a.configs:
        runs = [r for r in runs if r['config'] in a.configs]
    groups = defaultdict(list)
    for r in runs:
        groups[(r['config'], r['scenario'])].append(r['m'])
    print(f'{"config":28s} {"scenario":13s} reached  time[s] (each)          gear changes (each)   net heading')
    for (c, s), ms in groups.items():
        t = [m['time'] if m['reached'] else float('nan') for m in ms]
        print(f'{c:28s} {s:13s} {sum(m["reached"] for m in ms)}/{len(ms)}     '
              f'{" ".join(f"{m["time"]:5.1f}{"" if m["reached"] else "*"}" for m in ms):22s} '
              f'{" ".join(f"{m["gear_changes"]:3d}" for m in ms):20s} '
              f'{" ".join(f"{m["net_heading_deg"]:5.0f}" for m in ms)}')
    print('(* = timed out)')
    if not a.plot:
        return
    configs = list(dict.fromkeys(r['config'] for r in runs))
    scen_reps = list(dict.fromkeys((r['scenario'], r['rep']) for r in runs))
    fig, axes = plt.subplots(len(configs), len(scen_reps), figsize=(3.4 * len(scen_reps), 3.3 * len(configs)),
                             squeeze=False)
    for ax in axes.flat:
        ax.set_visible(False)
    for r in runs:
        ax = axes[configs.index(r['config'])][scen_reps.index((r['scenario'], r['rep']))]
        ax.set_visible(True)
        P, C = r['pose'], r['cmd']
        if len(r['plans']):
            p0 = r['plans'][0]
            ax.plot(p0[:, 0], p0[:, 1], color='0.75', lw=3, label='first plan')
        if len(P) > 1 and len(C):
            k = np.searchsorted(C[:, 0], P[:, 0]).clip(0, len(C) - 1)
            fwd = C[k, 1] >= 0
            ax.scatter(P[:, 1], P[:, 2], c=np.where(fwd, 'tab:blue', 'tab:orange'), s=1.5)
            ax.plot(P[0, 1], P[0, 2], 'go', ms=6)
        ax.plot(*r['goal'], 'r*', ms=12)
        m = r['m']
        ax.set_title(f'{r["config"]}\n{r["scenario"]} #{r["rep"]}: '
                     f'{m.get("status", "reached" if m["reached"] else "timeout")} {m["time"]:.0f}s, {m["gear_changes"]} gears',
                     fontsize=7.5)
        ax.set_aspect('equal'); ax.tick_params(labelsize=6)
        ax.set_xlim(-2.6, 1.9); ax.set_ylim(-2.5, 2.1)
    fig.suptitle('rover track: blue = commanded forward, orange = commanded reverse; grey = first plan', fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(a.plot, dpi=70)
    print('plot', a.plot)


if __name__ == '__main__':
    main()
