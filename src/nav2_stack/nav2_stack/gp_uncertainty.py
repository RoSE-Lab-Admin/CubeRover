#!/usr/bin/env python3
"""
Global uncertainty of the terrain GP (x, y -> z) after a trial.

Trains the same GP as gp_explorer_gpu.py (same bag loading -- the wheel contact points, 10 cm
binning, kernel and hyperparameter fit) on every bag in --bag-dir, then evaluates the GP's latent variance
var[f(x, y)] -- the uncertainty of the terrain height itself, without the measurement-noise
term, which never shrinks -- on a FIXED regular grid over the reachable arena (every point at
least INFLATION = 0.75 m from walls and craters in the map). The same grid every trial, so the
numbers are comparable across trials and across waypoint modes (gp / lawnmower / spiral).

Appends one row to <bag-dir>/gp_uncertainty.csv (rows are only ever appended, also on resume):

    time,bag,n_bags,n_train,lengthscale,outputscale,noise,integrated_var,integrated_var_fixed,
    n_grid,grid_res_m,clearance_m

integrated_var = sum of var[f] over the n_grid grid points (m^2), with the GP's hyperparameters
re-fitted on this run's data (as gp_explorer_gpu.py does; lengthscale, outputscale = its prior
variance, noise). It also moves when the hyperparameters move (e.g. a shorter lengthscale once
finer terrain structure shows up), so it need not fall monotonically.

integrated_var_fixed = the same sum with FIXED hyperparameters (FIXED_* below, the same for every
run): the posterior variance then depends only on WHERE the rover has driven, not on the heights
measured, so it falls monotonically with coverage and is directly comparable between runs and
waypoint modes.

autonomous_trials runs this as a subprocess after every trial (it exits afterwards, so it does
not hold GPU memory while the controller's network runs).

    python3 gp_uncertainty.py --bag-dir DIR --bag bag_07 --map .../maps/map.pgm
"""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import torch

try:                                    # run as a script next to gp_explorer_gpu.py
    import gp_explorer_gpu as gpx
except ImportError:                     # imported from the package
    from nav2_stack import gp_explorer_gpu as gpx

GRID_RES = 0.10     # m between evaluation points
# Fixed GP hyperparameters for integrated_var_fixed: fit once (300 Adam iterations) on all live
# lawnmower bags of 11/21 (V5b, V5c, V6, V7, V8 pooled: 127 bags), on the wheel-contact ground map
# (gp_explorer_gpu.MAP_POINTS "contacts", 11/22: 2074 10-cm cells). Keep them unchanged, or runs are
# no longer comparable. (Before 11/22: centre-track map, 0.72 m / 0.0049 / 1.0e-4 -- rows from then
# are not comparable with later ones.)
FIXED_LENGTHSCALE = 0.69    # m
FIXED_OUTPUTSCALE = 0.0050  # m^2 (prior std ~7 cm)
FIXED_NOISE = 1.0e-4        # m^2 (~1 cm)
CSV_NAME = "gp_uncertainty.csv"
COLUMNS = ("time,bag,n_bags,n_train,lengthscale,outputscale,noise,integrated_var,"
           "integrated_var_fixed,n_grid,grid_res_m,clearance_m")


def evaluation_grid(map_path: Path, res: float = GRID_RES, clearance: float = gpx.INFLATION):
    """Regular grid (world x, y) over the map cells at least `clearance` from obstacles."""
    mask = gpx.build_free_mask(map_path, clearance)
    H, W = mask.shape
    xs = np.arange(gpx.MAP_ORIG_X + res / 2, gpx.MAP_ORIG_X + W * gpx.MAP_RES, res)
    ys = np.arange(gpx.MAP_ORIG_Y + res / 2, gpx.MAP_ORIG_Y + H * gpx.MAP_RES, res)
    X, Y = np.meshgrid(xs, ys)
    cols = np.floor((X - gpx.MAP_ORIG_X) / gpx.MAP_RES).astype(int)
    rows = H - 1 - np.floor((Y - gpx.MAP_ORIG_Y) / gpx.MAP_RES).astype(int)
    keep = mask[rows, cols]
    return np.column_stack([X[keep], Y[keep]])


def latent_variance(model, grid: np.ndarray, device) -> np.ndarray:
    """var[f(x)] of the GP (eval mode) at the grid points, without likelihood noise. Exact
    (Cholesky) solves at any size -- GPyTorch's default switches to approximate iterative solves
    above 800 training points, which made the summed variance jitter (and even rise with more data
    at fixed hyperparameters)."""
    import gpytorch
    dtype = next(model.parameters()).dtype
    out = []
    with torch.no_grad(), gpytorch.settings.max_cholesky_size(1_000_000), \
            gpytorch.settings.fast_pred_var(False):
        for i in range(0, len(grid), 4096):
            g = torch.tensor(grid[i:i + 4096], dtype=dtype, device=device)
            out.append(model(g).variance.cpu().numpy())
    return np.concatenate(out)


def fixed_model(X: np.ndarray, z: np.ndarray, device):
    """The explorer's GP on this data, but with the FIXED_* hyperparameters (no fit)."""
    import gpytorch
    tx = torch.tensor(X, dtype=torch.float64, device=device)     # double: the variances
    ty = torch.tensor(z, dtype=torch.float64, device=device)     # here go down to ~1e-5
    likelihood = gpytorch.likelihoods.GaussianLikelihood().to(device).double()
    model = gpx.TerrainGP(tx, ty, likelihood).to(device).double()
    model.covar_module.base_kernel.lengthscale = FIXED_LENGTHSCALE
    model.covar_module.outputscale = FIXED_OUTPUTSCALE
    likelihood.noise = FIXED_NOISE
    model.mean_module.constant.data.fill_(float(np.mean(z)))
    model.eval(); likelihood.eval()
    return model


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag-dir", required=True, type=Path)
    ap.add_argument("--bag", required=True, help="the trial just finished (written to the row)")
    ap.add_argument("--map", required=True, type=Path, help="map.pgm (the installed one)")
    ap.add_argument("--n-iter", default=100, type=int, help="GP hyperparameter fit iterations")
    ap.add_argument("--upto", default=None,
                    help="only use bags whose name sorts <= this one (offline checks)")
    ap.add_argument("--out", default=None, type=Path,
                    help=f"CSV to append to (default: <bag-dir>/{CSV_NAME})")
    args = ap.parse_args()

    t0 = time.time()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    bags = sorted(p for p in args.bag_dir.iterdir() if p.is_dir() and (p / "metadata.yaml").exists())
    if args.upto is not None:
        bags = [b for b in bags if b.name == "initial_bag" or b.name <= args.upto]
    X, z = gpx.load_bags(args.bag_dir, bags=bags)
    model, _ = gpx.train_gp(X, z, device, args.n_iter)
    grid = evaluation_grid(args.map)
    var = latent_variance(model, grid, device)
    var_fixed = latent_variance(fixed_model(X, z, device), grid, device)

    row = (f"{datetime.now().isoformat(timespec='seconds')},{args.bag},{len(bags)},{len(X)},"
           f"{model.covar_module.base_kernel.lengthscale.item():.6g},"
           f"{model.covar_module.outputscale.item():.6g},{model.likelihood.noise.item():.6g},"
           f"{var.sum():.6g},{var_fixed.sum():.6g},{len(grid)},{GRID_RES:g},{gpx.INFLATION:g}")
    path = args.out or args.bag_dir / CSV_NAME
    new = not path.exists()
    with open(path, "a") as f:
        if new:
            f.write(COLUMNS + "\n")
        f.write(row + "\n")
    print(f"[gp_uncertainty] {args.bag}: integrated latent variance {var.sum():.6g} m^2 "
          f"(fixed GP settings: {var_fixed.sum():.6g} m^2) over "
          f"{len(grid)} grid points ({GRID_RES:g} m, >= {gpx.INFLATION:g} m from obstacles), "
          f"{len(X)} training cells from {len(bags)} bags, {time.time() - t0:.1f} s on {device} "
          f"-> {path}")


if __name__ == "__main__":
    sys.exit(main())
