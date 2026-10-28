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
POSE_TOPIC = "/FitRosey_passive/pose"
# bags before 2026-10-27 recorded the rigid body under its old Motive name
POSE_TOPICS = (POSE_TOPIC, "/FitRosey_V1/pose")
CMD_OUT_TOPIC = "/roseybot_base_controller/cmd_vel_out"

COL_X, COL_Y, COL_YAW = 0, 1, 2
COL_VX_CMD, COL_WZ_CMD = 3, 4
COL_DFWD, COL_DLAT, COL_DYAW = 5, 6, 7
COL_DX, COL_DY = 8, 9
COL_W = 10       # per-sample training weight (1.0 unless failure weighting is on)
COL_PITCH, COL_ROLL = 11, 12  # OptiTrack attitude, rad (ROS ZYX: pitch + = nose down); terrain only
N_COLS = 13

FUTURE_FEAT_COLS = [COL_VX_CMD, COL_WZ_CMD]
LOOKBACK = 0     # past cmd_vel_out steps fed to the MLP besides the current one (set_history();
                 # 0 = v11 family, no history window). Never used by the linear model.
HORIZON = 25     # matches Nav2 MPPI's time_steps=25
DT = 0.1
IN_DIM_AR = 2    # 2 * (LOOKBACK + 1): [vx, wz] of steps k-LOOKBACK .. k, oldest first

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


def set_history(steps: int):
    """Number of past cmd_vel_out steps the MLP sees besides the current one
    (nav2_param2.yaml FollowPath.nn_history_steps). Must match the controller."""
    global LOOKBACK, IN_DIM_AR, MIN_SEG_LEN
    LOOKBACK = max(0, int(steps))
    IN_DIM_AR = 2 * (LOOKBACK + 1)
    MIN_SEG_LEN = LOOKBACK + HORIZON + 1


# ── Terrain model (terrain_nn) ─────────────────────────────────────────────────
# The MLP also gets the rover's pitch and roll at the start of each step (current
# step only, no history) and predicts a lateral step besides the forward/yaw
# corrections. Training uses the OptiTrack attitude. In MPPI's rollouts step 0
# uses the measured attitude too; steps 1.. use the slope of the GP height map
# (refit here exactly as gp_explorer_gpu does) at the predicted pose, mapped onto
# what OptiTrack would read by a linear calibration fit on the training poses
# (absorbs the rigid body's mounting offset). The map and the calibration are
# stored in the exported model, see TerrainMLP.attitude().
TERRAIN = False
ATT_COLS = [COL_PITCH, COL_ROLL]
ATT_SCALE = 0.1          # rad; pitch/roll are divided by this before the terrain add-on
TERRAIN_HEAD_WIDTH = 16  # hidden units of the terrain add-on (TerrainMLP.head)
# training only: Gaussian noise on the pitch/roll inputs, ~ the GP map's error vs
# OptiTrack, so the network learns slope trends rather than exact attitudes (and
# copes with the map-derived attitude of MPPI's future steps)
ATT_NOISE = np.radians(2.0)

# ── Slip GP (terrain_gp, model_type "slip_gp") ─────────────────────────────────
# Slip ratios s = 1 - (moved along the command) / (commanded), per 1 deg x 1 deg
# bin of the slip features (summed over the bin's samples, so the many small
# per-step ratios average out), then one GP per ratio over the bins.
SLIP_MIN_CMD = (0.05, 0.05)        # |vx| m/s, |wz| rad/s for a sample to count
SLIP_BIN = np.radians(1.0)
SLIP_MIN_SAMPLES = 10              # per bin
SLIP_GRID_RES = np.radians(0.5)
SLIP_GRID_MAX = np.radians(25.0)   # climb -25..25 deg, side 0..25 deg
SLIP_CLAMP = (-1.0, 1.0)
TERRAIN_GRID_RES = 0.05  # m, slope grid sampled from the GP mean
# attitude that mirrored copies are reflected about (the calibration offset =
# what OptiTrack reads on level ground); set by retrain()
ATT_CENTER = np.zeros(2, dtype=np.float32)


def set_terrain(on: bool):
    global TERRAIN
    TERRAIN = bool(on)


def model_dims():
    """(input, output) size of the MLP: [vx, wz] history (+ pitch, roll) ->
    [corr_fwd, corr_yaw] (or [corr_fwd, lateral, corr_yaw] for the terrain model)."""
    return (IN_DIM_AR + 2, 3) if TERRAIN else (IN_DIM_AR, 2)
GAP_FACTOR = 3.0
MAX_POS_JUMP = 0.05
AVG_WINDOW_S = 0.1
MOTION_POS_THRESH = 0.01  # (no longer gates samples, see "commanded but stuck" below)
LOOKBACK_PAD = 10

# ── Which commanded samples are used ──────────────────────────────────────────
# Every sample where the base controller applied a non-zero command
# (cmd_vel_out) is used -- including when the rover did not move: small
# commands below the motors' deadband are exactly what the model must learn
# (10/05 bag_07: 0.04 m/s commands, wheels ~still, rover stalled). Excluded:
#  - blocked: wheels turning (ground speed >= BLOCKED_WHEEL_SPEED) while the
#    body, rotation included, barely moves (< BLOCKED_BODY_SPEED) -- pushing
#    against a wall is not the rover's dynamics
#  - mocap trouble: pose gaps > MOCAP_GAP_S or frozen (identical) poses
JOINT_TOPIC = "/dynamic_joint_states"
WHEEL_RADIUS = 0.158       # m, roseybot_controllers.yaml
WHEEL_SEPARATION = 0.3365  # m, roseybot_controllers.yaml
BLOCKED_WHEEL_SPEED = 0.10  # m/s of wheel ground speed
BLOCKED_BODY_SPEED = 0.02   # m/s, translation + rotation x half the track
MOCAP_GAP_S = 0.25

VALID_WIDTHS = {8, 16, 32, 64, 128}

# ── Failure weighting (see compute_sample_weights) ────────────────────────────
# Samples from trials that did not reach their goal, and stretches where the
# rover made no progress along its plan, are weighted up in the MLP loss so the
# model learns the dynamics of exactly the commands that went wrong.
PLAN_TOPIC = "/plan"
OUTCOMES_CSV = "trial_outcomes.csv"  # written by autonomous_trials.py, one row per trial
GOAL_TOL = 0.5             # m, matches the controller's xy goal tolerance
FAIL_WEIGHT = 2.0          # samples from a failed trial
NO_PROGRESS_WEIGHT = 3.0   # samples in a no-progress stretch
MAX_WEIGHT = 5.0           # cap on the product
PROGRESS_WINDOW_S = 5.0    # progress is measured over this centered window
PROGRESS_MIN_M = 0.10      # less remaining-plan reduction than this over the window = no progress

# A topic whose header stamps are more than this far from the bag's recording
# (receive) time is treated as stamped by an unsynced clock (e.g. the rover's
# computer booting without NTP: cmd_vel_out stamps ~20 h behind on 10/05) and
# its recording time is used instead. All topics are recorded on the same
# workstation, so recording times are mutually consistent.
CLOCK_OFFSET_TOL_S = 1.0


# ── Quaternion / binning helpers (ported verbatim) ─────────────────────────────
def _quat_to_yaw(qx, qy, qz, qw):
    return np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))


def _quat_to_pitch_roll(qx, qy, qz, qw):
    """ROS (ZYX) pitch and roll -- same formula as NNDynamics::integrateNeuralNetwork."""
    pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx**2 + qy**2))
    return pitch, roll


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


def read_outcomes(bag_dir: Path) -> Dict[str, str]:
    """bag name -> trial outcome ('reached', 'stalled', ...) from OUTCOMES_CSV, if present."""
    path = bag_dir / OUTCOMES_CSV
    out = {}
    if path.exists():
        for line in path.read_text().splitlines()[1:]:
            parts = line.split(",")
            if len(parts) >= 2:
                out[parts[0].strip()] = parts[1].strip()  # later rows (retries) win
    return out


def _remaining_plan_length(plan_xy: np.ndarray, plan_cum: np.ndarray, x: float, y: float) -> float:
    k = int(np.argmin((plan_xy[:, 0] - x) ** 2 + (plan_xy[:, 1] - y) ** 2))
    return float(plan_cum[-1] - plan_cum[k])


def compute_sample_weights(ct: np.ndarray, x: np.ndarray, y: np.ndarray, plans, failed: bool,
                           weighting: bool) -> np.ndarray:
    """Per-sample loss weight. 1.0 everywhere unless weighting is on; then
    FAIL_WEIGHT for a failed trial times NO_PROGRESS_WEIGHT where the remaining
    length of the latest /plan shrank by less than PROGRESS_MIN_M over a
    PROGRESS_WINDOW_S window (planned detours such as three-point turns still
    shrink it, shuffling in place does not), capped at MAX_WEIGHT."""
    w = np.ones(len(ct), dtype=np.float32)
    if not weighting:
        return w
    f_fail = FAIL_WEIGHT if failed else 1.0
    no_prog = np.zeros(len(ct), dtype=bool)
    if plans:
        plan_t = np.array([t for t, _ in plans])
        remaining = np.full(len(ct), np.nan)
        near_goal = np.zeros(len(ct), dtype=bool)
        for i, t in enumerate(ct):
            j = int(np.searchsorted(plan_t, t, side="right")) - 1
            if j < 0:
                continue
            xy, cum = plans[j][1]
            remaining[i] = _remaining_plan_length(xy, cum, x[i], y[i])
            near_goal[i] = np.hypot(xy[-1, 0] - x[i], xy[-1, 1] - y[i]) < GOAL_TOL
        valid = ~np.isnan(remaining)
        if valid.sum() > 1:
            half = PROGRESS_WINDOW_S / 2.0
            r_before = np.interp(ct - half, ct[valid], remaining[valid])
            r_after = np.interp(ct + half, ct[valid], remaining[valid])
            first_valid = ct[valid][0]
            full_window = (ct - half >= first_valid) & (ct + half <= ct[-1])
            no_prog = valid & full_window & ~near_goal & (r_before - r_after < PROGRESS_MIN_M)
    w = np.where(no_prog, f_fail * NO_PROGRESS_WEIGHT, f_fail)
    return np.minimum(w, MAX_WEIGHT).astype(np.float32)


def extract_trajectory(bag_path: Path, typestore, outcome: Optional[str] = None,
                       weighting: bool = False):
    """Returns (segments or None, failed). failed comes from the recorded
    outcome if given, else from the bag: final pose farther than GOAL_TOL from
    the last /plan's endpoint."""
    # AnyReader decodes types missing from the typestore (control_msgs'
    # DynamicJointState) from the definitions stored in the bag itself
    from rosbags.highlevel import AnyReader

    pose_rows, cmdout_rows, plans, joint_rows = [], [], [], []
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        topics = {c.topic for c in reader.connections}
        pose_topic = next((t for t in POSE_TOPICS if t in topics), None)
        if pose_topic is None or CMD_OUT_TOPIC not in topics:
            return None, False
        rec_t = {pose_topic: [], CMD_OUT_TOPIC: [], PLAN_TOPIC: []}  # recording times, same order
        conns = [c for c in reader.connections
                 if c.topic in (pose_topic, CMD_OUT_TOPIC, PLAN_TOPIC, JOINT_TOPIC)]
        for conn, ts_ns, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            if conn.topic == JOINT_TOPIC:
                # recording time: same clock as the (possibly corrected) others below
                speeds = [iv.values[list(iv.interface_names).index("velocity")]
                          for iv in m.interface_values if "velocity" in iv.interface_names]
                if speeds:
                    joint_rows.append((ts_ns * 1e-9, float(np.mean(np.abs(speeds)))))
                continue
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if conn.topic == pose_topic:
                p, o = m.pose.position, m.pose.orientation
                pose_rows.append((t, p.x, p.y, o.x, o.y, o.z, o.w))
            elif conn.topic == CMD_OUT_TOPIC:
                cmdout_rows.append((t, float(m.twist.linear.x), float(m.twist.angular.z)))
            elif m.poses:
                xy = np.array([[q.pose.position.x, q.pose.position.y] for q in m.poses])
                cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
                plans.append((t, (xy, cum)))
            else:
                continue
            rec_t[conn.topic].append(ts_ns * 1e-9)

    # unsynced publisher clocks: fall back to recording time (see CLOCK_OFFSET_TOL_S)
    for topic, rows in ((pose_topic, pose_rows), (CMD_OUT_TOPIC, cmdout_rows), (PLAN_TOPIC, plans)):
        if not rows:
            continue
        offset = float(np.median([r[0] for r in rows] - np.array(rec_t[topic])))
        if abs(offset) > CLOCK_OFFSET_TOL_S:
            off_s = f"{offset / 3600.0:+.2f} h" if abs(offset) >= 360.0 else f"{offset:+.1f} s"
            print(f"[dynamics_retrain] WARNING {bag_path.name}: {topic} header stamps are "
                  f"{off_s} off the recording time (unsynced clock?) -- "
                  f"using recording time for it", flush=True)
            rows[:] = [(rt,) + tuple(r[1:]) for rt, r in zip(rec_t[topic], rows)]
    plans.sort(key=lambda r: r[0])

    if outcome is not None:
        failed = outcome != "reached"
    elif plans and pose_rows:
        last_pose = max(pose_rows, key=lambda r: r[0])
        goal = plans[-1][1][0][-1]
        failed = bool(np.hypot(last_pose[1] - goal[0], last_pose[2] - goal[1]) > GOAL_TOL)
    else:
        failed = False

    if len(pose_rows) < 30 or len(cmdout_rows) < MIN_SEG_LEN:
        return None, failed

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
        return None, failed

    yaw_raw = np.unwrap(_quat_to_yaw(pose[:, 3], pose[:, 4], pose[:, 5], pose[:, 6]))
    pitch_raw, roll_raw = _quat_to_pitch_roll(pose[:, 3], pose[:, 4], pose[:, 5], pose[:, 6])
    x_avg, y_avg, yaw_avg, pitch_avg, roll_avg = block_average_onto(
        ct, pt, pose[:, 1], pose[:, 2], yaw_raw, pitch_raw, roll_raw)

    dx = np.concatenate([[0.0], np.diff(x_avg)])
    dy = np.concatenate([[0.0], np.diff(y_avg)])
    dyaw = np.concatenate([[0.0], np.diff(yaw_avg)])
    yaw_prev = np.concatenate([[yaw_avg[0]], yaw_avg[:-1]])
    dfwd = np.cos(yaw_prev) * dx + np.sin(yaw_prev) * dy
    dlat = -np.sin(yaw_prev) * dx + np.cos(yaw_prev) * dy

    w = compute_sample_weights(ct, x_avg, y_avg, plans, failed, weighting)
    data = np.column_stack([
        x_avg, y_avg, yaw_avg, cmdout[:, 1], cmdout[:, 2],
        dfwd, dlat, dyaw, dx, dy, w, pitch_avg, roll_avg,
    ]).astype(np.float32)

    # samples to leave out (see "Which commanded samples are used" at the top)
    step_t = np.maximum(np.concatenate([[DT], np.diff(ct)]), 1e-3)
    body_speed = (np.hypot(dx, dy) + np.abs(dyaw) * WHEEL_SEPARATION / 2.0) / step_t
    if joint_rows:
        joints = np.array(sorted(joint_rows))
        wheel_speed = np.interp(ct, joints[:, 0], joints[:, 1]) * WHEEL_RADIUS
        blocked = (wheel_speed >= BLOCKED_WHEEL_SPEED) & (body_speed < BLOCKED_BODY_SPEED)
    else:
        blocked = np.zeros(len(ct), dtype=bool)
    mocap_bad = np.zeros(len(ct), dtype=bool)
    frozen = np.all(np.diff(pose[:, 1:], axis=0) == 0.0, axis=1)
    for i in np.flatnonzero((np.diff(pt) > MOCAP_GAP_S) | frozen):
        mocap_bad |= (ct >= pt[i]) & (ct <= pt[i + 1])
    cmd_moving = (np.abs(cmdout[:, 1]) > STAT_VX) | (np.abs(cmdout[:, 2]) > STAT_WZ)
    stuck = cmd_moving & (body_speed < MOTION_POS_THRESH / DT) & ~blocked & ~mocap_bad
    if stuck.any() or blocked.any() or mocap_bad.any():
        print(f"[dynamics_retrain]   {bag_path.name}: {int(stuck.sum())} commanded-but-stuck "
              f"samples kept, excluded {int((cmd_moving & blocked).sum())} blocked (wheels "
              f"turning, body still) and {int(mocap_bad.sum())} with mocap gaps/freezes",
              flush=True)

    segs = _split_segments(data, ct, bag_path.name, usable=~blocked & ~mocap_bad)
    return (segs if segs else None), failed


def _split_segments(data: np.ndarray, times: np.ndarray, name: str,
                    usable: Optional[np.ndarray] = None) -> List[Dict]:
    dt_arr = np.diff(times)
    med_dt = float(np.median(dt_arr)) if len(dt_arr) > 0 else DT
    gap_mask = np.concatenate([[False], dt_arr > GAP_FACTOR * med_dt])
    pos_step = np.sqrt(np.diff(data[:, COL_X])**2 + np.diff(data[:, COL_Y])**2)
    jump_mask = np.concatenate([[False], pos_step > MAX_POS_JUMP])
    hard_break = gap_mask | jump_mask

    cmd_moving = ((np.abs(data[:, COL_VX_CMD]) > STAT_VX) |
                 (np.abs(data[:, COL_WZ_CMD]) > STAT_WZ))
    # every commanded sample, moving or not (the rover not responding to a
    # command is dynamics too), minus the excluded ones (see extract_trajectory)
    moving = cmd_moving if usable is None else cmd_moving & usable

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
        # past steps in the input (exported with the model; NNDynamics checks it
        # against FollowPath.nn_history_steps)
        self.history_steps: int = (in_dim // 2) - 1

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


def gp_attitude(grid: torch.Tensor, x0: float, y0: float, res: float,
                x: torch.Tensor, y: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
    """(B, 2) [pitch, roll] in rad of the GP surface under (x, y) for heading yaw:
    bilinear lookup of the surface gradient grid (2, ny, nx) = [dz/dx, dz/dy] with
    node (0, 0) at (x0, y0); outside the grid the surface counts as level. Must stay
    CUDA-graph safe (it runs inside NNDynamics' captured rollout): no host tensors,
    no syncs."""
    ny, nx = grid.shape[1], grid.shape[2]
    fx = (x - x0) / res
    fy = (y - y0) / res
    inside = ((fx >= 0) & (fx <= nx - 1) & (fy >= 0) & (fy <= ny - 1)).to(x.dtype)
    fx = fx.clamp(0.0, nx - 1.001)
    fy = fy.clamp(0.0, ny - 1.001)
    ix = fx.floor()
    iy = fy.floor()
    tx = fx - ix
    ty = fy - iy
    i00 = iy.long() * nx + ix.long()
    g = grid.reshape(2, ny * nx)
    g00 = g.index_select(1, i00)
    g01 = g.index_select(1, i00 + 1)
    g10 = g.index_select(1, i00 + nx)
    g11 = g.index_select(1, i00 + nx + 1)
    gr = ((g00 * (1 - tx) + g01 * tx) * (1 - ty) + (g10 * (1 - tx) + g11 * tx) * ty) * inside
    c = torch.cos(yaw)
    s = torch.sin(yaw)
    along = gr[0] * c + gr[1] * s     # rise per metre along the heading
    across = -gr[0] * s + gr[1] * c   # rise per metre towards the rover's left
    return torch.stack([-torch.atan(along), torch.atan(across)], dim=1)


class TerrainMLP(nn.Module):
    """[normalized vx, wz history, pitch, roll (rad)] -> [corr_fwd, lateral, corr_yaw]:
    the plain command MLP (net: commands only -> [corr_fwd, corr_yaw]) plus a small
    terrain add-on (head: commands + attitude -> all three, zero-initialized, so a
    new model starts as exactly the plain one). Keeping the attitude out of the big
    network stops it from memorizing trajectories by their pitch/roll (10/28 held-out
    test: 5.8 cm after 2.5 s vs 6.7 plain and 10.9 for one MLP over everything).
    Carries the GP slope map and the GP -> OptiTrack calibration that NNDynamics uses
    for the attitude of rollout steps 1.. (see attitude())."""

    def __init__(self, in_dim: int, out_dim: int, hidden: int, n_layers: int,
                 history_steps: int, tmap: Dict):
        super().__init__()
        layers: List[nn.Module] = [nn.Linear(in_dim - 2, hidden), nn.LayerNorm(hidden), nn.ReLU()]
        for _ in range(n_layers - 1):
            layers += [nn.Linear(hidden, hidden), nn.LayerNorm(hidden), nn.ReLU()]
        layers.append(nn.Linear(hidden, 2))
        self.net = nn.Sequential(*layers)
        self.head = nn.Sequential(nn.Linear(in_dim, TERRAIN_HEAD_WIDTH), nn.Tanh(),
                                  nn.Linear(TERRAIN_HEAD_WIDTH, out_dim))
        nn.init.zeros_(self.head[2].weight)
        nn.init.zeros_(self.head[2].bias)
        self.history_steps: int = history_steps
        self.terrain: bool = True   # NNDynamics switches to the terrain rollout on this
        self.att_scale: float = ATT_SCALE
        self.grid_x0: float = float(tmap["x0"])
        self.grid_y0: float = float(tmap["y0"])
        self.grid_res: float = float(tmap["res"])
        self.register_buffer("grid", torch.as_tensor(tmap["grid"], dtype=torch.float32))
        # rows pitch, roll: OptiTrack ~= calib @ [pitch_gp, roll_gp, 1]
        self.register_buffer("calib", torch.as_tensor(tmap["calib"], dtype=torch.float32))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        b = self.net(x[:, :-2])
        plain = torch.stack([b[:, 0], torch.zeros_like(b[:, 0]), b[:, 1]], dim=1)
        return plain + self.head(torch.cat([x[:, :-2], x[:, -2:] / self.att_scale], dim=1))

    @torch.jit.export
    def attitude(self, x: torch.Tensor, y: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
        """(B, 2) [pitch, roll] OptiTrack would read at world pose (x, y, yaw), from the
        GP surface through the calibration."""
        a = gp_attitude(self.grid, self.grid_x0, self.grid_y0, self.grid_res, x, y, yaw)
        return a @ self.calib[:, :2].t() + self.calib[:, 2]


def slip_features(pitch_gp: torch.Tensor, roll_gp: torch.Tensor,
                  vx: torch.Tensor) -> torch.Tensor:
    """(B, 2) slip GP input [climb, side] in rad from the GP surface's pitch/roll:
    climb = slope along the direction of travel (+ = uphill, also when reversing up
    a hill), side = side slope magnitude."""
    direction = 1.0 - 2.0 * (vx < 0).to(vx.dtype)
    return torch.stack([-pitch_gp * direction, roll_gp.abs()], dim=1)


class TerrainSlipGP(nn.Module):
    """Kinematics with slip learned from the slope (terrain_gp): the applied
    v = v_cmd (1 - s_x), w = w_cmd (1 - s_w), with [s_x, s_w] from a GP over
    slip_features() of the GP height map's surface under the rover. Both GPs are
    precomputed onto grids (the height map's slope as in TerrainMLP, the slip GP
    over [climb, side]), so a rollout step is two bilinear lookups. Same interface
    as TerrainMLP for NNDynamics: forward([normalized vx, wz history, pitch, roll])
    -> [corr_fwd, lateral (= 0), corr_yaw], attitude(x, y, yaw) -> OptiTrack-like
    [pitch, roll]; forward() maps the attitude back onto the GP surface through
    the inverse calibration, so step 0's measured attitude (mounting offset
    included) and the map's attitude of steps 1.. give the same slip."""

    def __init__(self, history_steps: int, tmap: Dict, slip: Dict,
                 fmean: np.ndarray, fstd: np.ndarray):
        super().__init__()
        self.history_steps: int = history_steps
        self.terrain: bool = True   # NNDynamics switches to the terrain rollout on this
        self.dt: float = DT
        self.grid_x0: float = float(tmap["x0"])
        self.grid_y0: float = float(tmap["y0"])
        self.grid_res: float = float(tmap["res"])
        self.register_buffer("grid", torch.as_tensor(tmap["grid"], dtype=torch.float32))
        calib = torch.as_tensor(tmap["calib"], dtype=torch.float32)
        self.register_buffer("calib", calib)
        self.register_buffer("calib_inv", torch.linalg.inv(calib[:, :2]))
        self.slip_c0: float = float(slip["c0"])
        self.slip_s0: float = float(slip["s0"])
        self.slip_res: float = float(slip["res"])
        self.slip_min: float = float(SLIP_CLAMP[0])
        self.slip_max: float = float(SLIP_CLAMP[1])
        self.register_buffer("slip_grid", torch.as_tensor(slip["grid"], dtype=torch.float32))
        self.register_buffer("fmean", torch.as_tensor(fmean, dtype=torch.float32))
        self.register_buffer("fstd", torch.as_tensor(fstd, dtype=torch.float32))

    def slip(self, f: torch.Tensor) -> torch.Tensor:
        """(B, 2) [s_x, s_w] at slip features f (B, 2); the grid's edge value beyond it."""
        ny, nx = self.slip_grid.shape[1], self.slip_grid.shape[2]
        fx = ((f[:, 0] - self.slip_c0) / self.slip_res).clamp(0.0, nx - 1.001)
        fy = ((f[:, 1] - self.slip_s0) / self.slip_res).clamp(0.0, ny - 1.001)
        ix = fx.floor()
        iy = fy.floor()
        tx = fx - ix
        ty = fy - iy
        i00 = iy.long() * nx + ix.long()
        g = self.slip_grid.reshape(2, ny * nx)
        s = ((g.index_select(1, i00) * (1 - tx) + g.index_select(1, i00 + 1) * tx) * (1 - ty) +
             (g.index_select(1, i00 + nx) * (1 - tx) + g.index_select(1, i00 + nx + 1) * tx) * ty)
        return s.t().clamp(self.slip_min, self.slip_max)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        cmd = x[:, -4:-2] * self.fstd + self.fmean      # this step's [vx, wz]
        a_gp = (x[:, -2:] - self.calib[:, 2]) @ self.calib_inv.t()
        s = self.slip(slip_features(a_gp[:, 0], a_gp[:, 1], cmd[:, 0]))
        corr_fwd = -s[:, 0] * cmd[:, 0] * self.dt
        corr_yaw = -s[:, 1] * cmd[:, 1] * self.dt
        return torch.stack([corr_fwd, torch.zeros_like(corr_fwd), corr_yaw], dim=1)

    @torch.jit.export
    def attitude(self, x: torch.Tensor, y: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
        """(B, 2) [pitch, roll] OptiTrack would read at world pose (x, y, yaw), from the
        GP surface through the calibration (as TerrainMLP.attitude)."""
        a = gp_attitude(self.grid, self.grid_x0, self.grid_y0, self.grid_res, x, y, yaw)
        return a @ self.calib[:, :2].t() + self.calib[:, 2]


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
    if TERRAIN:
        # kinematics never moves the rover sideways: the whole lateral step is the target
        lat = window[:, L + 1:L + H + 1, COL_DLAT]
        return torch.stack([cf, lat, cyaw], dim=2)
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
                         init_state_dict: Optional[dict] = None, tmap: Optional[Dict] = None):
    """AR + velocity-style + teacher-forced only. init_state_dict, if given,
    warm-starts training from those weights instead of default init. tmap: the
    terrain map (build_terrain_map) -- required with TERRAIN."""
    pin = DEVICE.type == "cuda"
    tr_ld = DataLoader(WindowDataset(train_segs, stride=1), BATCH, shuffle=True,
                       num_workers=2, pin_memory=pin, drop_last=True)
    va_ld = DataLoader(WindowDataset(val_segs, stride=HORIZON), BATCH, shuffle=False,
                       num_workers=2, pin_memory=pin)

    in_dim, out_dim = model_dims()
    if TERRAIN:
        model = TerrainMLP(in_dim, out_dim, hidden, n_layers, LOOKBACK, tmap).to(DEVICE)
    else:
        model = MLP(in_dim, out_dim, hidden, n_layers).to(DEVICE)
    if init_state_dict is not None:
        # only the network weights (net, terrain head): a terrain model's map and
        # calibration buffers come from this retrain
        own = {k: v for k, v in model.state_dict().items() if k not in ("grid", "calib")}
        init = {k: v for k, v in init_state_dict.items() if k not in ("grid", "calib")}
        if init.keys() != own.keys() or any(init[k].shape != own[k].shape for k in own):
            w0 = init.get('net.0.weight')
            print(f"[dynamics_retrain] warm-start model's layers differ from this one's "
                  f"(net input {None if w0 is None else w0.shape[1]} vs "
                  f"{model.net[0].weight.shape[1]}, terrain head "
                  f"{'yes' if 'head.0.weight' in init else 'no'} vs "
                  f"{'yes' if TERRAIN else 'no'}; nn_history_steps or terrain_nn changed?) "
                  f"-- training from a blank init", flush=True)
        else:
            model.load_state_dict(init, strict=False)
    opt = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=WD)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=MAX_EPOCHS)
    best_val, best_state, patience = float("inf"), None, 0

    L = LOOKBACK

    def compute_loss(window, weighted=True):
        fut_n = norm.norm_future2(window[:, :, FUTURE_FEAT_COLS])
        target = pointwise_velocity_target(window)
        rows_x, rows_y, rows_w = [], [], []
        for k in range(HORIZON):
            # [vx, wz] of steps k-L .. k (oldest first), flattened -- same order as NNDynamics
            x_k = fut_n[:, k:L + k + 1].reshape(fut_n.shape[0], -1)
            if TERRAIN:  # + OptiTrack pitch, roll at the start of step k (rad)
                a_k = window[:, L + k][:, ATT_COLS]
                if weighted and model.training:
                    a_k = a_k + ATT_NOISE * torch.randn_like(a_k)
                x_k = torch.cat([x_k, a_k], dim=1)
            rows_x.append(x_k)
            rows_y.append(target[:, k, :])
            rows_w.append(window[:, L + k, COL_W])
        x = torch.cat(rows_x, dim=0)
        y = torch.cat(rows_y, dim=0)
        err = ((model(x) - y) ** 2).sum(-1)
        if not weighted:
            return err.mean()
        w = torch.cat(rows_w, dim=0)
        # weighted mean; with all weights 1 this is exactly err.mean()
        return (w * err).sum() / w.sum().clamp_min(1e-12)

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
    model.eval()
    with torch.no_grad():
        unweighted = sum(compute_loss(win.to(DEVICE), weighted=False).item() for win in va_ld)
    print(f"[dynamics_retrain] best val_loss={best_val:.6f} "
          f"(unweighted {unweighted / max(len(va_ld), 1):.6f})", flush=True)
    return model, best_val


# ── Symmetry augmentation ──────────────────────────────────────────────────────
# Mirrored copies of the training data (10/09 analysis of all bags: the steady
# yaw gain matches for left/right turns and for forward/reverse; the only
# asymmetry was a ~0.004 rad/s drift, negligible for control, not modelled).
#   left/right    : wz -> -wz; yaw, lateral (and world y, dy, yaw) flip sign
#   forward/back  : the rover turned by 180 deg: vx -> -vx; forward and
#                   lateral motion flip sign, yaw change stays
# Copies get lower weights than the real data, so where real data disagrees
# it still wins; validation always stays real data only.
SYM_WEIGHTS = {"lr": 0.5, "fb": 0.25, "lr_fb": 0.25}


def _mirror(data: np.ndarray, lr: bool, fb: bool, weight: float) -> np.ndarray:
    d = data.copy()
    # attitude (terrain model): left/right mirrors the roll, the 180 deg turn both;
    # reflected about the mounting offset ATT_CENTER, not about 0, so the copies
    # read the way OptiTrack would
    if lr:
        for c in (COL_WZ_CMD, COL_DYAW, COL_DLAT, COL_Y, COL_YAW, COL_DY):
            d[:, c] = -d[:, c]
        d[:, COL_ROLL] = 2 * ATT_CENTER[1] - d[:, COL_ROLL]
    if fb:
        for c in (COL_VX_CMD, COL_DFWD, COL_DLAT):
            d[:, c] = -d[:, c]
        d[:, COL_PITCH] = 2 * ATT_CENTER[0] - d[:, COL_PITCH]
        d[:, COL_ROLL] = 2 * ATT_CENTER[1] - d[:, COL_ROLL]
    d[:, COL_W] *= weight
    return d


def augment_symmetry(segments: List[Dict], weights: Dict[str, float] = SYM_WEIGHTS) -> List[Dict]:
    """segments + their mirrored copies (see SYM_WEIGHTS); a weight <= 0 skips that copy."""
    out = list(segments)
    for key, (lr, fb) in (("lr", (True, False)), ("fb", (False, True)), ("lr_fb", (True, True))):
        w = weights.get(key, 0.0)
        if w > 0:
            out += [{**sg, "data": _mirror(sg["data"], lr, fb, w), "name": f"{sg['name']}~{key}"}
                    for sg in segments]
    return out


# ── Terrain map (terrain_nn) ───────────────────────────────────────────────────
def build_terrain_map(bag_paths: List[Path], segments: List[Dict]) -> Dict:
    """GP height map of every bag's mocap poses, fit exactly as gp_explorer_gpu does
    (10 cm cell means of z, ExactGP, its train_gp) but on the CPU; its gradient is
    sampled on a TERRAIN_GRID_RES grid over the Nav2 map. Then the linear
    calibration GP attitude -> OptiTrack attitude is fit on every sample of
    segments (real data, not mirrored)."""
    from nav2_stack import gp_explorer_gpu as gpx  # deferred: pulls in gpytorch
    try:
        X, z = gpx.load_bags(None, bags=bag_paths)
    except SystemExit as e:  # gp_explorer exits on "no pose data"
        raise RuntimeError(f"terrain map: {e}") from None
    gp, _ = gpx.train_gp(X, z, torch.device("cpu"))
    pgm = gpx.read_pgm(Path(get_package_share_directory("nav2_stack")) / "maps" / "map.pgm")
    ny = int(round(pgm.shape[0] * gpx.MAP_RES / TERRAIN_GRID_RES)) + 1
    nx = int(round(pgm.shape[1] * gpx.MAP_RES / TERRAIN_GRID_RES)) + 1
    gx, gy = np.meshgrid(gpx.MAP_ORIG_X + np.arange(nx) * TERRAIN_GRID_RES,
                         gpx.MAP_ORIG_Y + np.arange(ny) * TERRAIN_GRID_RES)
    pts = torch.tensor(np.column_stack([gx.ravel(), gy.ravel()]), dtype=torch.float32)
    grads = []
    for chunk in pts.split(10000):
        chunk = chunk.clone().requires_grad_(True)
        g, = torch.autograd.grad(gp(chunk).mean.sum(), chunk)
        grads.append(g.detach())
    grid = torch.cat(grads).T.reshape(2, ny, nx).contiguous()
    tmap = {"grid": grid, "x0": gpx.MAP_ORIG_X, "y0": gpx.MAP_ORIG_Y, "res": TERRAIN_GRID_RES}

    data = np.concatenate([sg["data"] for sg in segments])
    t = torch.from_numpy(data)
    a_gp = gp_attitude(grid, tmap["x0"], tmap["y0"], tmap["res"],
                       t[:, COL_X], t[:, COL_Y], t[:, COL_YAW]).numpy().astype(np.float64)
    A = np.column_stack([a_gp, np.ones(len(a_gp))])
    calib = np.zeros((2, 3))
    for i, c in enumerate(ATT_COLS):
        calib[i], *_ = np.linalg.lstsq(A, data[:, c].astype(np.float64), rcond=None)
    tmap["calib"] = calib.astype(np.float32)
    resid = data[:, ATT_COLS] - A @ calib.T
    ls = gp.covar_module.base_kernel.lengthscale.item()
    print(f"[dynamics_retrain] terrain map: GP on {len(X)} cells (lengthscale {ls:.2f} m), "
          f"slope grid {nx}x{ny} at {TERRAIN_GRID_RES * 100:.0f} cm; calibration on "
          f"{len(data)} samples:", flush=True)
    for i, n in enumerate(("pitch", "roll")):
        print(f"[dynamics_retrain]   {n} = {calib[i, 0]:.3f} * pitch_gp + {calib[i, 1]:.3f} * "
              f"roll_gp {np.degrees(calib[i, 2]):+.2f} deg   (residual rms "
              f"{np.degrees(resid[:, i].std()):.2f} deg)", flush=True)
    return tmap


def fit_slip_gp(segments: List[Dict], tmap: Dict) -> Dict:
    """Slip GP grid (2, ny, nx) [s_x, s_w] over [climb, side] (see SLIP_*), fit on
    segments with the slip features of the height map's surface at the measured
    poses -- what the rollout sees at steps 1.."""
    from nav2_stack import gp_explorer_gpu as gpx  # deferred: pulls in gpytorch
    rows = []
    for sg in segments:
        d = torch.from_numpy(sg["data"])
        cur, nxt = d[:-1], d[1:]   # command and pose at step k, motion over step k
        a = gp_attitude(tmap["grid"], tmap["x0"], tmap["y0"], tmap["res"],
                        cur[:, COL_X], cur[:, COL_Y], cur[:, COL_YAW])
        f = slip_features(a[:, 0], a[:, 1], cur[:, COL_VX_CMD])
        rows.append(torch.cat([f, cur[:, [COL_VX_CMD, COL_WZ_CMD]],
                               nxt[:, [COL_DFWD, COL_DYAW]]], 1))
    R = torch.cat(rows).numpy().astype(np.float64)
    f, vx, wz, dfwd, dyaw = R[:, :2], R[:, 2], R[:, 3], R[:, 4], R[:, 5]

    nx = int(round(2 * SLIP_GRID_MAX / SLIP_GRID_RES)) + 1
    ny = int(round(SLIP_GRID_MAX / SLIP_GRID_RES)) + 1
    cg, sg_ = np.meshgrid(-SLIP_GRID_MAX + np.arange(nx) * SLIP_GRID_RES,
                          np.arange(ny) * SLIP_GRID_RES)
    pts = torch.tensor(np.column_stack([cg.ravel(), sg_.ravel()]), dtype=torch.float32)
    grids, report = [], []
    for name, cmd, moved, min_cmd in (("linear", vx, dfwd, SLIP_MIN_CMD[0]),
                                      ("angular", wz, dyaw, SLIP_MIN_CMD[1])):
        use = np.abs(cmd) > min_cmd
        keys = np.floor(f[use] / SLIP_BIN).astype(np.int64)
        _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
        inv = inv.ravel()
        along = np.bincount(inv, moved[use] * np.sign(cmd[use]))   # moved along the command
        commanded = np.bincount(inv, np.abs(cmd[use]) * DT)
        fb = np.stack([np.bincount(inv, f[use][:, i]) for i in range(2)], 1) / counts[:, None]
        keep = counts >= SLIP_MIN_SAMPLES
        if keep.sum() < 3:
            raise RuntimeError(f"slip GP: only {int(keep.sum())} {name} slope bins with "
                               f">= {SLIP_MIN_SAMPLES} samples")
        s = 1.0 - along[keep] / commanded[keep]
        gp, _ = gpx.train_gp(fb[keep], s, torch.device("cpu"))
        with torch.no_grad():
            grids.append(gp(pts).mean.reshape(ny, nx))
        flat = float(1.0 - along.sum() / commanded.sum())
        report.append(f"{name}: {int(keep.sum())} bins from {int(use.sum())} samples, "
                      f"overall slip {flat:+.2f}, GP at level {grids[-1][0, nx // 2]:+.2f} / "
                      f"climb +10 deg {grids[-1][0, nx // 2 + 20]:+.2f} / "
                      f"climb -10 deg {grids[-1][0, nx // 2 - 20]:+.2f} / "
                      f"side 10 deg {grids[-1][20, nx // 2]:+.2f}")
    for line in report:
        print(f"[dynamics_retrain] slip GP {line}", flush=True)
    return {"grid": torch.stack(grids), "c0": -SLIP_GRID_MAX, "s0": 0.0, "res": SLIP_GRID_RES}


# ── Top-level entry point ──────────────────────────────────────────────────────
def _load_bags(bag_paths: List[Path], typestore, outcomes: Dict[str, str], weighting: bool):
    """bag path -> (segments, failed) for every bag with usable data."""
    loaded = {}
    for bag_path in bag_paths:
        segs, failed = extract_trajectory(bag_path, typestore, outcomes.get(bag_path.name),
                                          weighting)
        loaded[bag_path] = (segs or [], failed)
        n = sum(len(sg["data"]) for sg in segs or [])
        print(f"[dynamics_retrain]   {bag_path.name}: {len(segs or [])} segments, {n} samples"
              f"{' (failed trial)' if failed else ''}"
              f"{'  <-- NO usable data' if not segs else ''}", flush=True)
    return loaded


def retrain(bag_dir: Path, new_bag_path: Path, model_type: str, width: Optional[int],
           warm_start: bool, subset: bool, subset_fraction: float,
           fmean: np.ndarray, fstd: np.ndarray,
           warm_start_path: Optional[Path] = None, save_path: Optional[Path] = None,
           failure_weighting: bool = False, symmetry_augmentation: bool = False,
           history_steps: int = 0, bags: Optional[List[Path]] = None,
           terrain: bool = False) -> dict:
    """
    Retrains a model (model_type="linear", "mlp" or "slip_gp", width required for "mlp")
    on bags in bag_dir, always including new_bag_path plus either every other
    bag (subset=False) or a random subset_fraction of them (subset=True).

    MLP weight locations (both optional, MLP only):
      - warm_start_path: read the warm-start init from here instead of the
        shared deployed .pt (ignored if warm_start=False).
      - save_path: write the retrained, scripted model here instead of the
        shared deployed .pt (used by --train-from-scratch to keep every
        iteration's weights isolated in bag_dir, never touching the shared
        deployed model other callers rely on).
      Both default to the shared installed path (found via ament_index, not
      the source tree) when not given -- unchanged behavior for the normal
      retrain_dynamics flow.

    bags: train on exactly these bag directories instead of every bag found
    under bag_dir. Used by autonomous_trials' async training, which runs while
    the next trial is still recording a (half-written) bag into bag_dir.

    terrain: train the terrain MLP (see TERRAIN above; MLP only, ignored with a
    warning for "linear"). Its GP map is refit on the same bags.

    Returns a dict:
      - model_type="mlp": {"exported_path": <str>}.
      - model_type="linear": {"weight": [4 floats], "bias": [2 floats]} -- the
        caller (autonomous_trials.py) is responsible for pushing these into the
        live controller via `ros2 param set`, since linear weights are ROS2
        parameters, not a file.
    """
    # history only for the MLP (the linear model's weights are a fixed 2x2 in the yaml)
    # (slip_gp only uses the newest step, but takes the controller's full input window)
    set_history(history_steps if model_type in ("mlp", "slip_gp") else 0)
    if LOOKBACK:
        print(f"[dynamics_retrain] history: the MLP sees cmd_vel_out of the current step and the "
              f"{LOOKBACK} before it ({IN_DIM_AR} inputs)", flush=True)
    if terrain and model_type != "mlp":
        print(f"[dynamics_retrain] WARNING: terrain only applies to the MLP -- training "
              f"{model_type} without it", flush=True)
    set_terrain(terrain and model_type == "mlp")
    ATT_CENTER[:] = 0.0
    if TERRAIN:
        print(f"[dynamics_retrain] terrain: + pitch, roll inputs ({model_dims()[0]} inputs), "
              f"+ lateral output", flush=True)
    from rosbags.typesys import Stores, get_typestore
    try:
        typestore = get_typestore(Stores.ROS2_JAZZY)
    except Exception:
        typestore = get_typestore(Stores.ROS2_HUMBLE)

    all_bags = list(bags) if bags is not None else get_bags(bag_dir)
    loaded = _load_bags(all_bags, typestore, read_outcomes(bag_dir), failure_weighting)
    failed_bags = [b for b in all_bags if loaded[b][1] and b != new_bag_path]
    other_bags = [b for b in all_bags if b != new_bag_path and b not in failed_bags]

    if subset and other_bags:
        k = max(1, round(subset_fraction * len(other_bags)))
        chosen_others = random.sample(other_bags, min(k, len(other_bags)))
    else:
        chosen_others = other_bags

    # the new bag and every failed trial always train (never only held out),
    # so failure data is learned from right away
    forced = ([new_bag_path] if new_bag_path in all_bags else []) + failed_bags
    use_bags = chosen_others + forced
    print(f"[dynamics_retrain] training on {len(use_bags)} bags "
          f"({len(chosen_others)} existing{' (subset)' if subset else ''} + "
          f"{'1 new' if new_bag_path in all_bags else '0 new'} + {len(failed_bags)} failed; "
          f"failure weighting {'on' if failure_weighting else 'off'})", flush=True)

    if len(use_bags) >= 4 and chosen_others:
        rng = np.random.default_rng()
        n_val = min(len(chosen_others), max(1, int(len(use_bags) * 0.2)))
        idx = rng.permutation(len(chosen_others))
        val_bags = [chosen_others[i] for i in idx[:n_val]]
        train_bags = [chosen_others[i] for i in idx[n_val:]] + forced
    else:
        # too few bags for a meaningful held-out split yet (e.g. early trials) --
        # reuse everything for both, rather than risking an empty val set that
        # would break early stopping.
        train_bags = val_bags = use_bags

    train_segs = [s for b in train_bags for s in loaded[b][0]]
    val_segs = [s for b in val_bags for s in loaded[b][0]]
    if not train_segs:
        raise RuntimeError(f"no usable segments extracted from {len(use_bags)} bags in {bag_dir}")
    tmap = None
    if TERRAIN:
        tmap = build_terrain_map(all_bags, [s for b in all_bags for s in loaded[b][0]])
        ATT_CENTER[:] = tmap["calib"][:, 2]
    if symmetry_augmentation:
        n_real = len(train_segs)
        train_segs = augment_symmetry(train_segs)
        print(f"[dynamics_retrain] symmetry augmentation: {n_real} -> {len(train_segs)} training "
              f"segments (weights {SYM_WEIGHTS}; validation stays real only)", flush=True)

    norm = Normalizer(fmean, fstd)

    if model_type == "slip_gp":
        # terrain_gp: no network -- refit both GPs on every bag used (real data only)
        segs = [s for b in use_bags for s in loaded[b][0]]
        tmap = build_terrain_map(all_bags, [s for b in all_bags for s in loaded[b][0]])
        model = TerrainSlipGP(LOOKBACK, tmap, fit_slip_gp(segs, tmap), fmean, fstd).eval()
        scripted = torch.jit.script(model)
        n_in = IN_DIM_AR + 2
        x = torch.randn(37, n_in) * 0.5
        p = torch.rand(37, 3) * torch.tensor([10.0, 10.0, 6.3]) - torch.tensor([5.0, 5.0, 3.1])
        with torch.no_grad():
            ref = torch.cat([model(x), model.attitude(p[:, 0], p[:, 1], p[:, 2])], 1)
            got = torch.cat([scripted(x), scripted.attitude(p[:, 0], p[:, 1], p[:, 2])], 1)
        assert (ref - got).abs().max().item() == 0.0, "scripted model diverges from eager -- aborting export"
        if save_path is None:
            raise ValueError("slip_gp needs a save_path (never the shared deployed model)")
        save_path.parent.mkdir(parents=True, exist_ok=True)
        scripted.save(str(save_path))
        print(f"[dynamics_retrain] slip GP model exported to {save_path}", flush=True)
        return {"exported_path": str(save_path)}

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
        default_dir = Path(get_package_share_directory("nav2_mppi_controller")) / "models" / "ar_mlp"
        default_path = default_dir / f"mlp{width}_ar_velocity_teacher.pt"
        load_path = warm_start_path if warm_start_path is not None else default_path
        out_path = save_path if save_path is not None else default_path

        init_state_dict = None
        if warm_start and load_path.exists():
            init_state_dict = torch.jit.load(str(load_path)).state_dict()

        model, val_loss = train_mlp_ar_teacher(width, 2, train_segs, val_segs, norm,
                                               init_state_dict=init_state_dict, tmap=tmap)
        print(f"[dynamics_retrain] mlp{width}{' terrain' if TERRAIN else ''} retrain done, "
              f"val_loss={val_loss:.6f}", flush=True)

        model = model.cpu().eval()
        scripted = torch.jit.script(model)
        x = torch.randn(37, model_dims()[0])
        with torch.no_grad():
            ref, got = model(x), scripted(x)
            if TERRAIN:
                p = torch.rand(37, 3) * torch.tensor([10.0, 10.0, 6.3]) - torch.tensor([5.0, 5.0, 3.1])
                ref = torch.cat([ref, model.attitude(p[:, 0], p[:, 1], p[:, 2])], 1)
                got = torch.cat([got, scripted.attitude(p[:, 0], p[:, 1], p[:, 2])], 1)
        assert (ref - got).abs().max().item() == 0.0, "scripted model diverges from eager -- aborting export"

        out_path.parent.mkdir(parents=True, exist_ok=True)
        scripted.save(str(out_path))
        return {"exported_path": str(out_path)}

    raise ValueError(f"model_type must be 'linear', 'mlp' or 'slip_gp', got {model_type!r}")
