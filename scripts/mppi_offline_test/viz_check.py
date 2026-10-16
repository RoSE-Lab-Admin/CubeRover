#!/usr/bin/env python3
"""Compare /optimal_trajectory (commands integrated as commanded) with
/optimal_trajectory_model (same commands through the configured dynamics
model) recorded by mppi_trial.py, per controller cycle (paired by stamp).

    python3 viz_check.py results/viz/*.npz
"""
import math
import sys

import numpy as np


def main():
    for f in sys.argv[1:]:
        z = np.load(f, allow_pickle=True)
        ot, o, mt, m = z['opt_t'], z['opt'], z['opt_model_t'], z['opt_model']
        if not len(ot) or not len(mt):
            print(f'{f}: missing topics (opt {len(ot)}, model {len(mt)})')
            continue
        idx = {round(t, 4): i for i, t in enumerate(mt)}
        pos_diff, end_dist, dyaw_k, dyaw_m = [], [], [], []
        for i, t in enumerate(ot):
            j = idx.get(round(t, 4))
            if j is None:
                continue
            a, b = np.asarray(o[i], dtype=float), np.asarray(m[j], dtype=float)
            n = min(len(a), len(b))
            pos_diff.append(np.hypot(a[:n, 0] - b[:n, 0], a[:n, 1] - b[:n, 1]).max())
            end_dist.append(math.hypot(a[n - 1, 0] - b[n - 1, 0], a[n - 1, 1] - b[n - 1, 1]))
            dyaw_k.append(math.degrees(np.unwrap(a[:n, 2])[-1] - a[0, 2]))
            dyaw_m.append(math.degrees(np.unwrap(b[:n, 2])[-1] - b[0, 2]))
        pd = np.array(pos_diff)
        print(f'{f.split("/")[-1]}: {len(pd)} paired cycles (of {len(ot)} opt / {len(mt)} model msgs); '
              f'max point gap: median {np.median(pd):.4f} m, max {pd.max():.4f} m; '
              f'heading change over horizon: as-commanded median {np.median(dyaw_k):+.1f} deg, '
              f'model median {np.median(dyaw_m):+.1f} deg; '
              f'cycles where they turn opposite ways: '
              f'{sum(1 for k_, m_ in zip(dyaw_k, dyaw_m) if k_ * m_ < 0 and abs(k_) > 5 and abs(m_) > 5)}')


if __name__ == '__main__':
    main()
