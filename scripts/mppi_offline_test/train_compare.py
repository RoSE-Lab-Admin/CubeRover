#!/usr/bin/env python3
"""Offline comparison of dynamics retraining variants, as the real trial-14
retrain would run them (warm start from the previous from-scratch weights),
writing only into --out (never into from_scratch_weights/).

Variants:
  old      dynamics_retrain.py at git HEAD (random 20% bag split, no weights)
  split    current code, failure_weighting=False (new/failed bags always train)
  weighted current code, failure_weighting=True

Evaluation on real data:
  - per-step prediction error (unweighted) on bag 14's no-progress stretches
    vs on normal driving (all other samples)
  - the problem region (REGION_VX / REGION_WZ: slow and turning): mean predicted vs mean
    measured effective (vx, wz) per command bin

Run with ROS sourced and the cuda_end venv's site-packages available.
"""
import argparse
import importlib.util
import random
import subprocess
import sys
from pathlib import Path

import numpy as np
import torch

REGION_VX, REGION_WZ = 0.12, 0.15  # problem region: slow and turning
REPO = Path('/home/roselab/CubeRover_waypoint')
sys.path.insert(0, str(REPO / 'src/nav2_stack'))
from nav2_stack import dynamics_retrain as new_dr  # noqa: E402


def load_old(tmp: Path):
    src = subprocess.check_output(
        ['git', '-C', str(REPO), 'show', 'HEAD:src/nav2_stack/nav2_stack/dynamics_retrain.py'])
    p = tmp / 'dynamics_retrain_old.py'
    p.write_bytes(src)
    spec = importlib.util.spec_from_file_location('dynamics_retrain_old', p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def seeded(mod, seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    real = np.random.default_rng
    mod.np.random.default_rng = lambda *a, **k: real(seed)


def eval_model(pt, bag_dir, bag14, fmean, fstd):
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_JAZZY)
    m = torch.jit.load(str(pt), map_location='cpu').eval()
    fm, fs = torch.tensor(fmean), torch.tensor(fstd)

    def predict(cmd):  # (N,2) -> effective per-step [dfwd, dyaw]
        with torch.no_grad():
            corr = m((torch.as_tensor(cmd, dtype=torch.float32) - fm) / fs).numpy()
        return cmd * new_dr.DT + corr

    # samples with weights (weighting on) just to locate the no-progress stretches
    out = {}
    rows_np, rows_ok = [], []
    for b in new_dr.get_bags(bag_dir):
        segs, failed = new_dr.extract_trajectory(b, ts, None, weighting=True)
        for s in segs or []:
            d = s['data']
            cmd = d[:-1, [new_dr.COL_VX_CMD, new_dr.COL_WZ_CMD]]
            meas = d[1:, [new_dr.COL_DFWD, new_dr.COL_DYAW]]
            err = ((predict(cmd) - meas) ** 2).sum(-1)
            noprog = d[:-1, new_dr.COL_W] >= new_dr.NO_PROGRESS_WEIGHT - 1e-6
            if b == bag14:
                rows_np.append(err[noprog])
                rows_ok.append(err[~noprog])
            else:
                rows_ok.append(err)
            if b == bag14:
                out.setdefault('b14_cmd', []).append(cmd)
                out.setdefault('b14_meas', []).append(meas)
    res = {'err_bag14_noprog': float(np.concatenate(rows_np).mean()) if rows_np else float('nan'),
           'err_normal': float(np.concatenate(rows_ok).mean())}
    # problem region: slow and turning commands, binned by sign
    cmd = np.concatenate(out['b14_cmd'])
    meas = np.concatenate(out['b14_meas']) / new_dr.DT
    region = (np.abs(cmd[:, 0]) <= REGION_VX) & (np.abs(cmd[:, 1]) >= REGION_WZ)
    pred = predict(cmd) / new_dr.DT
    bins = []
    for sv in (1, -1):
        for sw in (1, -1):
            sel = region & (np.sign(cmd[:, 0]) == sv) & (np.sign(cmd[:, 1]) == sw)
            if sel.sum() >= 5:
                bins.append((sv, sw, int(sel.sum()), cmd[sel].mean(0), meas[sel].mean(0), pred[sel].mean(0)))
    res['region'] = bins
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag-dir', default='/home/roselab/AL_scripts/rosbags_10_04_26_test', type=Path)
    ap.add_argument('--warm', default='from_scratch_weights/mlp128_trial13.pt')
    ap.add_argument('--out', default='results/train_compare', type=Path)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--variants', nargs='+', default=['old', 'split', 'weighted'])
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    fmean = np.array([0.00372449, -0.03188930], dtype=np.float32)  # nav2_param2.yaml linear_model.fmean/fstd
    fstd = np.array([0.19342424, 0.31374899], dtype=np.float32)
    bag14 = a.bag_dir / 'bag_14'
    warm = a.bag_dir / a.warm
    old = load_old(a.out)
    results = {'trial13 (as deployed)': eval_model(warm, a.bag_dir, bag14, fmean, fstd)}
    for v in a.variants:
        mod = old if v == 'old' else new_dr
        seeded(mod, a.seed)
        save = a.out / f'mlp128_trial14_{v}.pt'
        kw = dict(bag_dir=a.bag_dir, new_bag_path=bag14, model_type='mlp', width=128,
                  warm_start=True, subset=False, subset_fraction=0.3, fmean=fmean, fstd=fstd,
                  warm_start_path=warm, save_path=save)
        if v != 'old':
            kw['failure_weighting'] = (v == 'weighted')
        print(f'=== training variant {v}', flush=True)
        mod.retrain(**kw)
        results[v] = eval_model(save, a.bag_dir, bag14, fmean, fstd)

    print('\nper-step squared error (unweighted; lower is better)')
    print(f'{"model":24s} {"bag14 no-progress":>18s} {"normal driving":>15s}')
    for k, r in results.items():
        print(f'{k:24s} {r["err_bag14_noprog"]:18.3e} {r["err_normal"]:15.3e}')
    print(f'\nproblem region in bag 14 (|vx_cmd|<={REGION_VX}, |wz_cmd|>={REGION_WZ}): mean effective [vx, wz]')
    for k, r in results.items():
        print(f'-- {k}')
        for sv, sw, n, c, meas, pred in r['region']:
            print(f'   cmd vx{"+" if sv > 0 else "-"} wz{"+" if sw > 0 else "-"} (n={n:4d}, mean cmd '
                  f'[{c[0]:+.3f},{c[1]:+.3f}]): measured [{meas[0]:+.3f},{meas[1]:+.3f}]  '
                  f'predicted [{pred[0]:+.3f},{pred[1]:+.3f}]')


if __name__ == '__main__':
    main()
