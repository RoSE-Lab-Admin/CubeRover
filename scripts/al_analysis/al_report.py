"""Tables for al_comparison.ipynb from the al_metrics.py cache (numpy only)."""
import json
from pathlib import Path

import numpy as np

CACHE = Path.home() / 'AL_scripts' / 'al_analysis_cache'
N_BOOT = 5
# (key, label, unit, better)  better: 'low' or 'high'
METRICS = [
    ('step1_yaw_rmse', '1-step yaw-rate error', 'rad/s', 'low'),
    ('step1_speed_rmse', '1-step speed error', 'm/s', 'low'),
    ('pred25_head_deg', '2.5 s heading error (MPPI belief)', 'deg', 'low'),
    ('pred25_pos_m', '2.5 s position error (MPPI belief)', 'm', 'low'),
    ('plan10_xtrack_m', '10 s distance from plan (mean)', 'm', 'low'),
    ('plan10_end_m', '10 s end-point error vs plan', 'm', 'low'),
    ('path_ratio_plan', 'driven / first plan length', '', 'low'),
    ('path_ratio_straight', 'driven / straight distance', '', 'low'),
    ('gears', 'gear changes per trial', '', 'low'),
    ('rev_plan_share', 'replans with a reverse leg', '', 'low'),
    ('time', 'drive time', 's', 'low'),
]


def load(name):
    return json.load(open(CACHE / f'{name}.json'))


def usable(run, skip_bootstrap=True):
    """valid, not low-battery; NN runs without the 5 kinematics bootstrap trials."""
    out = []
    for t in run['trials']:
        if t.get('invalid') or t.get('low_batt'):
            continue
        if skip_bootstrap and run['mode'] == 'nn' and t['i'] <= N_BOOT:
            continue
        out.append(t)
    return out


def boot_ci(x, n=2000, seed=0):
    x = np.asarray([v for v in x if v is not None and np.isfinite(v)], float)
    if len(x) < 2:
        return (np.nan, np.nan, np.nan)
    rng = np.random.default_rng(seed)
    m = rng.choice(x, (n, len(x))).mean(1)
    return float(x.mean()), float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


def diff_ci(a, b, n=2000, seed=0):
    a = np.asarray([v for v in a if v is not None and np.isfinite(v)], float)
    b = np.asarray([v for v in b if v is not None and np.isfinite(v)], float)
    if len(a) < 2 or len(b) < 2:
        return (np.nan, np.nan, np.nan)
    rng = np.random.default_rng(seed)
    d = rng.choice(a, (n, len(a))).mean(1) - rng.choice(b, (n, len(b))).mean(1)
    return float(a.mean() - b.mean()), float(np.percentile(d, 2.5)), float(np.percentile(d, 97.5))


def summary_rows(runs_a, runs_b, name_a, name_b, metrics=METRICS):
    """rows: label, mean a [CI], mean b [CI], difference [CI], verdict"""
    rows = []
    for key, label, unit, better in metrics:
        va = [t.get(key) for t in runs_a]
        vb = [t.get(key) for t in runs_b]
        ma, mb, d = boot_ci(va), boot_ci(vb), diff_ci(va, vb)
        if np.isnan(d[0]):
            verdict = ''
        elif d[1] > 0 or d[2] < 0:
            a_better = (d[0] < 0) == (better == 'low')
            verdict = f'{name_a if a_better else name_b} better'
        else:
            verdict = 'no clear difference'
        rows.append((label + (f' [{unit}]' if unit else ''), ma, mb, d, verdict))
    return rows


def arc_rows(runs, names):
    out = []
    for r, n in zip(runs, names):
        arcs = [t for t in r if t.get('arc_trial')]
        held = sum(t['arc_held'] for t in arcs)
        dev = boot_ci([t['arc_dev_max'] for t in arcs])
        out.append((n, len(arcs), held, dev))
    return out


def outcome_rows(runs, names):
    out = []
    for r, n in zip(runs, names):
        out.append((n, len(r), sum(t['outcome'] == 'reached' for t in r)))
    return out


def print_summary(rows, name_a, name_b):
    print(f"{'metric':42s} {name_a:>22s} {name_b:>22s} {'difference (a-b)':>24s}  verdict")
    for label, ma, mb, d, v in rows:
        f = lambda m: f'{m[0]:7.3f} [{m[1]:6.3f},{m[2]:6.3f}]'
        print(f'{label:42s} {f(ma):>22s} {f(mb):>22s} {f(d):>24s}  {v}')
