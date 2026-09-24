#!/usr/bin/env python3
"""
Online retraining for the deployed dynamics-correction model (linear_ar_velocity
or mlp{8,16,32,64,128}_ar_velocity_teacher), used by autonomous_trials.py when
retrain_dynamics is enabled.

Vendored, lean, single-model (AR + velocity-style + teacher-forced only) port of
the relevant pieces of plotting_data/run_nn_dynamics_v11.py. plotting_data/ is
fully gitignored and never reaches the deployment workstation via git pull, so
this is a deliberate, self-contained copy rather than an import -- keep this in
mind if the offline training methodology in plotting_data/ changes and needs
porting here too.

Normalization: reuses the ALREADY-DEPLOYED shared normalizer (passed in by the
caller, loaded from the live linear_model.fmean/fstd) -- never refit here. Every
deployed model (linear + every MLP width) is normalized identically in the C++
controller, so refitting per retrain would silently break whichever model is
deployed.
"""

import random
import re
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import torch
import torch.nn as nn
from scipy.interpolate import interp1d
from torch.utils.data import DataLoader, Dataset

from ament_index_python.packages import get_package_share_directory

# ── Constants (match plotting_data/run_nn_dynamics_v11.py exactly) ────────────
POSE_TOPIC = "/FitRosey_V1/pose"
CMD_OUT_TOPIC = "/roseybot_base_controller/cmd_vel_out"

COL_X, COL_Y, COL_YAW = 0, 1, 2
COL_VX_CMD, COL_WZ_CMD = 3, 4
COL_DFWD, COL_DLAT, COL_DYAW = 5, 6, 7
COL_DX, COL_DY = 8, 9
N_COLS = 10

FUTURE_FEAT_COLS = [COL_VX_CMD, COL_WZ_CMD]
LOOKBACK = 0     # v11 family: no past features, no history window
HORIZON = 25     # matches Nav2 MPPI's time_steps=25
DT = 0.1
IN_DIM_AR = 2    # LOOKBACK*0 + 2 future features

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
MAX_EPOCHS = 300
PATIENCE = 40
LR = 1e-3
WD = 1e-4
BATCH = 256
GRAD_CLIP = 1.0
RIDGE_LAMBDA = 1.0

STAT_VX, STAT_WZ = 0.02, 0.02
MIN_SEG_LEN = LOOKBACK + HORIZON + 1
GAP_FACTOR = 3.0
MAX_POS_JUMP = 0.05
AVG_WINDOW_S = 0.1
MOTION_POS_THRESH = 0.01
LOOKBACK_PAD = 10

VALID_WIDTHS = {8, 16, 32, 64, 128}


# ── Quaternion / binning helpers (ported verbatim) ─────────────────────────────
def _quat_to_yaw(qx, qy, qz, qw):
    return np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))


def block_average_onto(ct: np.ndarray, pt: np.ndarray, *vals: np.ndarray) -> List[np.ndarray]:
    edges = np.concatenate([[ct[0] - AVG_WINDOW_S / 2],
                            (ct[:-1] + ct[1:]) / 2,
                            [ct[-1] + AVG_WINDOW_S / 2]])
    bin_idx = np.clip(np.searchsorted(edges, pt) - 1, 0, len(ct) - 1)
    out = []
    for v in vals:
        sums = np.bincount(bin_idx, weights=v, minlength=len(ct))
        counts = np.bincount(bin_idx, minlength=len(ct))
        avg = np.divide(sums, counts, out=np.full(len(ct), np.nan), where=counts > 0)
        if np.isnan(avg).any():
            good = ~np.isnan(avg)
            avg = interp1d(ct[good], avg[good], bounds_error=False,
                           fill_value=(avg[good][0], avg[good][-1]))(ct)
        out.append(avg)
    return out


def get_bags(bag_dir: Path) -> List[Path]:
    """All bag directories directly under bag_dir (flat layout: bag_NN, initial_bag)."""
    def sort_key(p):
        nums = re.findall(r"\d+", p.name)
        return int(nums[0]) if nums else -1  # initial_bag sorts first
    return sorted({p.parent for p in bag_dir.rglob("metadata.yaml")}, key=sort_key)


def extract_trajectory(bag_path: Path, typestore) -> Optional[List[Dict]]:
    from rosbags.rosbag2 import Reader

    pose_rows, cmdout_rows = [], []
    with Reader(str(bag_path)) as reader:
        topics = {c.topic for c in reader.connections}
        if not {POSE_TOPIC, CMD_OUT_TOPIC}.issubset(topics):
            return None
        conns = [c for c in reader.connections if c.topic in (POSE_TOPIC, CMD_OUT_TOPIC)]
        for conn, _ts, raw in reader.messages(connections=conns):
            m = typestore.deserialize_cdr(raw, conn.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if conn.topic == POSE_TOPIC:
                p, o = m.pose.position, m.pose.orientation
                pose_rows.append((t, p.x, p.y, o.x, o.y, o.z, o.w))
            else:
                cmdout_rows.append((t, float(m.twist.linear.x), float(m.twist.angular.z)))

    if len(pose_rows) < 30 or len(cmdout_rows) < MIN_SEG_LEN:
        return None

    pose = np.array(sorted(pose_rows, key=lambda r: r[0]))
    cmdout = np.array(sorted(cmdout_rows, key=lambda r: r[0]))
    pt = pose[:, 0]
    ct = cmdout[:, 0]

    t_start = max(pt[0], ct[0])
    t_end = min(pt[-1], ct[-1])
    mask = (ct >= t_start) & (ct <= t_end)
    ct = ct[mask]
    cmdout = cmdout[mask]
    if len(ct) < MIN_SEG_LEN:
        return None

    yaw_raw = np.unwrap(_quat_to_yaw(pose[:, 3], pose[:, 4], pose[:, 5], pose[:, 6]))
    x_avg, y_avg, yaw_avg = block_average_onto(ct, pt, pose[:, 1], pose[:, 2], yaw_raw)

    dx = np.concatenate([[0.0], np.diff(x_avg)])
    dy = np.concatenate([[0.0], np.diff(y_avg)])
    dyaw = np.concatenate([[0.0], np.diff(yaw_avg)])
    yaw_prev = np.concatenate([[yaw_avg[0]], yaw_avg[:-1]])
    dfwd = np.cos(yaw_prev) * dx + np.sin(yaw_prev) * dy
    dlat = -np.sin(yaw_prev) * dx + np.cos(yaw_prev) * dy

    data = np.column_stack([
        x_avg, y_avg, yaw_avg, cmdout[:, 1], cmdout[:, 2],
        dfwd, dlat, dyaw, dx, dy,
    ]).astype(np.float32)

    segs = _split_segments(data, ct, bag_path.name)
    return segs if segs else None


def _split_segments(data: np.ndarray, times: np.ndarray, name: str) -> List[Dict]:
    dt_arr = np.diff(times)
    med_dt = float(np.median(dt_arr)) if len(dt_arr) > 0 else DT
    gap_mask = np.concatenate([[False], dt_arr > GAP_FACTOR * med_dt])
    pos_step = np.sqrt(np.diff(data[:, COL_X])**2 + np.diff(data[:, COL_Y])**2)
    jump_mask = np.concatenate([[False], pos_step > MAX_POS_JUMP])
    hard_break = gap_mask | jump_mask

    cmd_moving = ((np.abs(data[:, COL_VX_CMD]) > STAT_VX) |
                 (np.abs(data[:, COL_WZ_CMD]) > STAT_WZ))
    pos_moving = np.concatenate([[False], pos_step > MOTION_POS_THRESH])
    moving = cmd_moving & pos_moving

    segments = []
    block_start = 0
    for i in range(1, len(data) + 1):
        if i == len(data) or hard_break[i]:
            block_end = i
            bm = moving[block_start:block_end]
            j = 0
            while j < len(bm):
                if bm[j]:
                    run_start = j
                    while j < len(bm) and bm[j]:
                        j += 1
                    run_end = j
                    abs_start = block_start + run_start
                    abs_end = block_start + run_end
                    pad = min(LOOKBACK_PAD, abs_start - block_start)
                    seg = data[abs_start - pad:abs_end]
                    if len(seg) >= MIN_SEG_LEN:
                        segments.append({"data": seg, "dt": DT, "name": name})
                else:
                    j += 1
            block_start = block_end
    return segments


# ── Normalizer (load-only -- never refit here, see module docstring) ──────────
class Normalizer:
    def __init__(self, fmean: np.ndarray, fstd: np.ndarray):
        self.fmean = fmean
        self.fstd = fstd

    def norm_future2(self, t: torch.Tensor) -> torch.Tensor:
        return (t - torch.as_tensor(self.fmean, dtype=t.dtype, device=t.device)) / \
               torch.as_tensor(self.fstd, dtype=t.dtype, device=t.device)


# ── Dataset / models (ported verbatim) ─────────────────────────────────────────
class WindowDataset(Dataset):
    def __init__(self, segments: List[Dict], stride: int = 1):
        win = LOOKBACK + HORIZON + 1
        self.windows = [d[s:s + win]
                        for seg in segments
                        for d in [seg["data"]]
                        for s in range(0, len(d) - win + 1, stride)]

    def __len__(self):
        return len(self.windows)

    def __getitem__(self, idx):
        return torch.from_numpy(self.windows[idx])


class MLP(nn.Module):
    def __init__(self, in_dim: int, out_dim: int, hidden: int, n_layers: int):
        super().__init__()
        layers: List[nn.Module] = [nn.Linear(in_dim, hidden), nn.LayerNorm(hidden), nn.ReLU()]
        for _ in range(n_layers - 1):
            layers += [nn.Linear(hidden, hidden), nn.LayerNorm(hidden), nn.ReLU()]
        layers.append(nn.Linear(hidden, out_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class LinearModel(nn.Module):
    def __init__(self, in_dim: int, out_dim: int):
        super().__init__()
        self.linear = nn.Linear(in_dim, out_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.linear(x)


def pointwise_velocity_target(window: torch.Tensor) -> torch.Tensor:
    """(B,H,2) [target_cf,target_cyaw] -- true consecutive theta already baked
    into COL_DFWD/DYAW."""
    L, H = LOOKBACK, HORIZON
    cf = window[:, L + 1:L + H + 1, COL_DFWD] - window[:, L:L + H, COL_VX_CMD] * DT
    cyaw = window[:, L + 1:L + H + 1, COL_DYAW] - window[:, L:L + H, COL_WZ_CMD] * DT
    return torch.stack([cf, cyaw], dim=2)


# ── Fitting (AR + velocity-style only) ─────────────────────────────────────────
def fit_linear_closed_form(train_segs: List[Dict], norm: Normalizer) -> LinearModel:
    ds = WindowDataset(train_segs, stride=1)
    ld = DataLoader(ds, batch_size=4096, shuffle=False)
    L, H = LOOKBACK, HORIZON
    X_list, Y_list = [], []
    for window in ld:
        fut_full = norm.norm_future2(window[:, :, FUTURE_FEAT_COLS])
        full_target = pointwise_velocity_target(window)
        rows_x, rows_y = [], []
        for k in range(H):
            x_k = fut_full[:, L + k]  # LOOKBACK=0 -> no past features to concat
            rows_x.append(x_k)
            rows_y.append(full_target[:, k, :])
        X_list.append(torch.cat(rows_x, dim=0).numpy())
        Y_list.append(torch.cat(rows_y, dim=0).numpy())

    X = np.concatenate(X_list, axis=0).astype(np.float64)
    Y = np.concatenate(Y_list, axis=0).astype(np.float64)
    X_aug = np.column_stack([X, np.ones(len(X))])
    XtX = X_aug.T @ X_aug
    XtX[:-1, :-1] += RIDGE_LAMBDA * np.eye(X.shape[1])
    XtY = X_aug.T @ Y
    W = np.linalg.solve(XtX, XtY)

    model = LinearModel(IN_DIM_AR, 2)
    with torch.no_grad():
        model.linear.weight.copy_(torch.from_numpy(W[:-1, :].T).float())
        model.linear.bias.copy_(torch.from_numpy(W[-1, :]).float())
    return model


def train_mlp_ar_teacher(hidden: int, n_layers: int, train_segs, val_segs, norm: Normalizer,
                         init_state_dict: Optional[dict] = None):
    """AR + velocity-style + teacher-forced only. init_state_dict, if given,
    warm-starts training from those weights instead of default init."""
    pin = DEVICE.type == "cuda"
    tr_ld = DataLoader(WindowDataset(train_segs, stride=1), BATCH, shuffle=True,
                       num_workers=2, pin_memory=pin, drop_last=True)
    va_ld = DataLoader(WindowDataset(val_segs, stride=HORIZON), BATCH, shuffle=False,
                       num_workers=2, pin_memory=pin)

    model = MLP(IN_DIM_AR, 2, hidden, n_layers).to(DEVICE)
    if init_state_dict is not None:
        model.load_state_dict(init_state_dict)
    opt = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=WD)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=MAX_EPOCHS)
    best_val, best_state, patience = float("inf"), None, 0

    L = LOOKBACK

    def compute_loss(window):
        fut_n = norm.norm_future2(window[:, :, FUTURE_FEAT_COLS])
        target = pointwise_velocity_target(window)
        rows_x, rows_y = [], []
        for k in range(HORIZON):
            rows_x.append(fut_n[:, L + k])
            rows_y.append(target[:, k, :])
        x = torch.cat(rows_x, dim=0)
        y = torch.cat(rows_y, dim=0)
        pred = model(x)
        return ((pred - y) ** 2).sum(-1).mean()

    for _epoch in range(MAX_EPOCHS):
        model.train()
        for window in tr_ld:
            window = window.to(DEVICE)
            loss = compute_loss(window)
            opt.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), GRAD_CLIP)
            opt.step()
        sched.step()

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for window in va_ld:
                window = window.to(DEVICE)
                val_loss += compute_loss(window).item()
        val_loss /= max(len(va_ld), 1)

        if val_loss < best_val:
            best_val = val_loss
            best_state = {k: v.cpu().clone() for k, v in model.state_dict().items()}
            patience = 0
        else:
            patience += 1
            if patience >= PATIENCE:
                break

    model.load_state_dict(best_state)
    return model, best_val


# ── Top-level entry point ──────────────────────────────────────────────────────
def _load_segments(bag_paths: List[Path], typestore) -> List[Dict]:
    segs = []
    for bag_path in bag_paths:
        s = extract_trajectory(bag_path, typestore)
        if s:
            segs.extend(s)
    return segs


def retrain(bag_dir: Path, new_bag_path: Path, model_type: str, width: Optional[int],
           warm_start: bool, subset: bool, subset_fraction: float,
           fmean: np.ndarray, fstd: np.ndarray) -> dict:
    """
    Retrains the deployed model (model_type="linear" or "mlp", width required for
    "mlp") on bags in bag_dir, always including new_bag_path plus either every
    other bag (subset=False) or a random subset_fraction of them (subset=True).

    Returns a dict:
      - model_type="mlp": {"exported_path": <str>} -- also writes the retrained,
        scripted model directly to the installed share directory the live
        controller reads from (found via ament_index, not the source tree --
        see module docstring).
      - model_type="linear": {"weight": [4 floats], "bias": [2 floats]} -- the
        caller (autonomous_trials.py) is responsible for pushing these into the
        live controller via `ros2 param set`, since linear weights are ROS2
        parameters, not a file.
    """
    from rosbags.typesys import Stores, get_typestore
    try:
        typestore = get_typestore(Stores.ROS2_JAZZY)
    except Exception:
        typestore = get_typestore(Stores.ROS2_HUMBLE)

    all_bags = get_bags(bag_dir)
    other_bags = [b for b in all_bags if b != new_bag_path]

    if subset and other_bags:
        k = max(1, round(subset_fraction * len(other_bags)))
        chosen_others = random.sample(other_bags, min(k, len(other_bags)))
    else:
        chosen_others = other_bags

    use_bags = chosen_others + ([new_bag_path] if new_bag_path in all_bags else [])
    print(f"[dynamics_retrain] training on {len(use_bags)} bags "
          f"({len(chosen_others)} existing{' (subset)' if subset else ''} + "
          f"{'1 new' if new_bag_path in all_bags else '0 new'})", flush=True)

    if len(use_bags) >= 4:
        rng = np.random.default_rng()
        idx = rng.permutation(len(use_bags))
        n_val = max(1, int(len(idx) * 0.2))
        val_bags = [use_bags[i] for i in idx[:n_val]]
        train_bags = [use_bags[i] for i in idx[n_val:]]
    else:
        # too few bags for a meaningful held-out split yet (e.g. early trials) --
        # reuse everything for both, rather than risking an empty val set that
        # would break early stopping.
        train_bags = val_bags = use_bags

    train_segs = _load_segments(train_bags, typestore)
    val_segs = _load_segments(val_bags, typestore)
    if not train_segs:
        raise RuntimeError(f"no usable segments extracted from {len(use_bags)} bags in {bag_dir}")

    norm = Normalizer(fmean, fstd)

    if model_type == "linear":
        model = fit_linear_closed_form(train_segs, norm)
        W = model.linear.weight.detach().numpy()
        b = model.linear.bias.detach().numpy()
        print(f"[dynamics_retrain] linear retrain done "
              f"(warm_start is a no-op for linear -- closed-form, not iterative)", flush=True)
        return {"weight": [float(W[0, 0]), float(W[0, 1]), float(W[1, 0]), float(W[1, 1])],
                "bias": [float(b[0]), float(b[1])]}

    if model_type == "mlp":
        if width not in VALID_WIDTHS:
            raise ValueError(f"width must be one of {sorted(VALID_WIDTHS)}, got {width}")
        deploy_dir = Path(get_package_share_directory("nav2_mppi_controller")) / "models" / "ar_mlp"
        deploy_path = deploy_dir / f"mlp{width}_ar_velocity_teacher.pt"

        init_state_dict = None
        if warm_start and deploy_path.exists():
            init_state_dict = torch.jit.load(str(deploy_path)).state_dict()

        model, val_loss = train_mlp_ar_teacher(width, 2, train_segs, val_segs, norm,
                                               init_state_dict=init_state_dict)
        print(f"[dynamics_retrain] mlp{width} retrain done, val_loss={val_loss:.6f}", flush=True)

        model = model.cpu().eval()
        scripted = torch.jit.script(model)
        x = torch.randn(37, IN_DIM_AR)
        with torch.no_grad():
            ref, got = model(x), scripted(x)
        assert (ref - got).abs().max().item() == 0.0, "scripted model diverges from eager -- aborting export"

        deploy_dir.mkdir(parents=True, exist_ok=True)
        scripted.save(str(deploy_path))
        return {"exported_path": str(deploy_path)}

    raise ValueError(f"model_type must be 'linear' or 'mlp', got {model_type!r}")
