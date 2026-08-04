#!/usr/bin/env python3
"""
GP Exploration — GPyTorch/CUDA alternative to gp_explorer.py

Key differences from gp_explorer.py:
  - Trains an ExactGP on GPU via GPyTorch (same O(n³) Cholesky, but CUDA-accelerated)
  - All Nav2 paths are collected first; then Cohn ALC is computed in a single
    batched GPU pass using the factored posterior covariance formula, avoiding
    the 500 sequential sklearn predict() calls that are the main bottleneck
  - Falls back transparently to CPU when CUDA is unavailable

Factored ALC formula (computed once for all stacked path points):

    L        = chol(K(X_train, X_train) + σ²I)       (n_train × n_train)
    A        = L⁻¹ K(X_train, X_ref)                 (n_train × n_ref)   — once
    B_chunk  = L⁻¹ K(X_train, chunk)                 (n_train × chunk)   — per chunk
    σ²_i     = outputscale − ||B_chunk[:,i]||² + σ²  (posterior obs variance)
    K_post   = K(chunk, X_ref) − B_chunk.T @ A       (chunk × n_ref)
    ALC_i    = ||K_post[i,:]||² / σ²_i

Usage (identical to gp_explorer.py, plus --n-iter):
    python3 gp_explorer_gpu.py --bag-dir /path/to/bags --name run1
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch
import gpytorch
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from scipy.ndimage import distance_transform_edt
from scipy.spatial.transform import Rotation
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


# ── Map config — must match map.yaml ──────────────────────────────────────────
MAP_RES    = 0.2
MAP_ORIG_X = -5.0
MAP_ORIG_Y = -5.0
INFLATION  = 0.75

# ── Nav2 settings ─────────────────────────────────────────────────────────────
GLOBAL_FRAME = 'world'
PLANNER_ID   = 'GridBased'

# ── Internal constants ─────────────────────────────────────────────────────────
POSE_TOPIC         = '/FitRosey_V1/pose'
N_REF_POINTS       = 200
MAX_PATH_PTS       = 100
DOWNSAMPLE_FACTOR  = 20
STATIONARY_THRESH  = 0.001


# ── GPyTorch model ─────────────────────────────────────────────────────────────

class TerrainGP(gpytorch.models.ExactGP):
    def __init__(self, train_x, train_y, likelihood):
        super().__init__(train_x, train_y, likelihood)
        self.mean_module  = gpytorch.means.ConstantMean()
        self.covar_module = gpytorch.kernels.ScaleKernel(
            gpytorch.kernels.RBFKernel()
        )

    def forward(self, x):
        return gpytorch.distributions.MultivariateNormal(
            self.mean_module(x), self.covar_module(x)
        )


def train_gp(X: np.ndarray, z: np.ndarray,
             device: torch.device, n_iter: int = 100):
    """Train ExactGP with Adam; return (model, likelihood) in eval mode."""
    train_x = torch.tensor(X, dtype=torch.float32, device=device)
    train_y = torch.tensor(z, dtype=torch.float32, device=device)

    likelihood = gpytorch.likelihoods.GaussianLikelihood().to(device)
    model      = TerrainGP(train_x, train_y, likelihood).to(device)

    model.train()
    likelihood.train()

    optimizer = torch.optim.Adam(model.parameters(), lr=0.1)
    mll       = gpytorch.mlls.ExactMarginalLogLikelihood(likelihood, model)

    t0 = time.time()
    for i in range(n_iter):
        optimizer.zero_grad()
        output = model(train_x)
        loss   = -mll(output, train_y)
        loss.backward()
        optimizer.step()
        if (i + 1) % 25 == 0:
            ls  = model.covar_module.base_kernel.lengthscale.item()
            out = model.covar_module.outputscale.item()
            nz  = model.likelihood.noise.item()
            print(f'  iter {i+1:3d}/{n_iter}  loss={loss.item():.4f}'
                  f'  ls={ls:.4f}  outputscale={out:.4f}  noise={nz:.6f}')

    elapsed = time.time() - t0
    print(f'  training done in {elapsed:.1f}s on {device}')
    model.eval()
    likelihood.eval()
    return model, likelihood


# ── Batched Cohn ALC on GPU ────────────────────────────────────────────────────

def batch_cohn_alc(model: TerrainGP,
                   device: torch.device,
                   all_paths: list,
                   X_ref: np.ndarray,
                   chunk_size: int = 8000) -> list:
    """
    Compute Cohn ALC for all paths in a single (chunked) GPU pass.

    Precomputes L and A once, then evaluates every stacked path point together
    rather than calling model.predict() 500 times.  Peak GPU memory scales with
    chunk_size × n_train (default: ~32 MB per chunk at n_train=1000).
    """
    if not all_paths:
        return []

    train_x   = model.train_inputs[0]                  # (n_train, 2) on device
    n_train   = train_x.shape[0]
    noise_var = model.likelihood.noise.item()           # scalar float

    with torch.no_grad():
        # ── K_train + σ²I and its Cholesky ─────────────────────────────────
        K_train = model.covar_module(train_x).to_dense()          # (n_train, n_train)
        # Add noise + small jitter for numerical stability
        K_noisy = K_train + (noise_var + 1e-5) * torch.eye(n_train, device=device)
        L       = torch.linalg.cholesky(K_noisy)                  # lower triangular

        # ── A = L⁻¹ K(X_train, X_ref)  ─────────────────────────────────────
        ref_t      = torch.tensor(X_ref, dtype=torch.float32, device=device)
        K_tr       = model.covar_module(train_x, ref_t).to_dense()  # (n_train, n_ref)
        A          = torch.linalg.solve_triangular(L, K_tr, upper=False)  # (n_train, n_ref)

        outputscale = model.covar_module.outputscale.item()        # k(x,x) for RBF

        # ── Stack all path points and allocate output ────────────────────────
        path_lengths = [len(p) for p in all_paths]
        path_cat     = np.concatenate(all_paths, axis=0)           # (total, 2)
        total        = len(path_cat)
        alc_pts      = torch.empty(total, dtype=torch.float32, device=device)

        # ── Chunk loop ───────────────────────────────────────────────────────
        for start in range(0, total, chunk_size):
            end     = min(start + chunk_size, total)
            chunk_t = torch.tensor(path_cat[start:end],
                                   dtype=torch.float32, device=device)
            chunk_n = end - start

            # K(X_train, chunk): (n_train, chunk_n)
            K_chunk = model.covar_module(train_x, chunk_t).to_dense()

            # B_chunk = L⁻¹ K_chunk: (n_train, chunk_n)
            B_chunk = torch.linalg.solve_triangular(L, K_chunk, upper=False)

            # Posterior obs variance = k(x*,x*) − ||b||² + σ²_noise
            sigma2 = (outputscale - (B_chunk ** 2).sum(dim=0) + noise_var
                      ).clamp_min(1e-12)                           # (chunk_n,)

            # Prior and posterior cross-covariance: (chunk_n, n_ref)
            K_cr_prior = model.covar_module(chunk_t, ref_t).to_dense()
            K_cr_post  = K_cr_prior - B_chunk.T @ A

            # ALC contribution per point: ||k_post_cross||² / σ²
            alc_pts[start:end] = (K_cr_post ** 2).sum(dim=1) / sigma2

    # Aggregate per path on CPU
    alc_cpu = alc_pts.cpu().numpy()
    scores  = []
    offset  = 0
    for length in path_lengths:
        scores.append(float(alc_cpu[offset:offset + length].sum()))
        offset += length
    return scores


# ── Bag loading ────────────────────────────────────────────────────────────────

def trim_stationary(X: np.ndarray, z: np.ndarray):
    if len(X) < 2:
        return X, z
    disp   = np.linalg.norm(np.diff(X, axis=0), axis=1)
    moving = np.where(disp > STATIONARY_THRESH)[0]
    if len(moving) == 0:
        return X[:0], z[:0]
    start, end = moving[0], moving[-1] + 1
    return X[start:end + 1], z[start:end + 1]


def load_bags(bag_dir: Path):
    try:
        typestore = get_typestore(Stores.ROS2_JAZZY)
    except AttributeError:
        typestore = get_typestore(Stores.ROS2_HUMBLE)

    bag_dirs = sorted(p for p in bag_dir.iterdir()
                      if p.is_dir() and (p / 'metadata.yaml').exists())
    if not bag_dirs:
        sys.exit(f'[error] No bags found in {bag_dir}')

    X_list, z_list = [], []
    for bag in bag_dirs:
        print(f'  reading {bag.name} …', flush=True)
        bag_X, bag_z = [], []
        with Reader(bag) as reader:
            if POSE_TOPIC not in reader.topics:
                print(f'    skipping — {POSE_TOPIC!r} not present')
                continue
            conns = [c for c in reader.connections if c.topic == POSE_TOPIC]
            for conn, _, rawdata in reader.messages(connections=conns):
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                bag_X.append([msg.pose.position.x, msg.pose.position.y])
                bag_z.append(msg.pose.position.z)
        if not bag_X:
            continue
        bag_X = np.array(bag_X, dtype=float)
        bag_z = np.array(bag_z, dtype=float)
        n_raw = len(bag_X)
        bag_X, bag_z = trim_stationary(bag_X, bag_z)
        print(f'    {n_raw} raw → {len(bag_X)} after trimming stationary ends')
        if len(bag_X) == 0:
            print(f'    skipping — no movement detected')
            continue
        X_list.append(bag_X)
        z_list.append(bag_z)

    if not X_list:
        sys.exit('[error] No pose data parsed from any bag')

    X = np.concatenate(X_list)
    z = np.concatenate(z_list)
    idx = np.arange(0, len(X), DOWNSAMPLE_FACTOR)
    X, z = X[idx], z[idx]
    print(f'  downsampled {DOWNSAMPLE_FACTOR}x → {len(X)} training points')
    return X, z


# ── Map utilities ──────────────────────────────────────────────────────────────

def read_pgm(path: Path) -> np.ndarray:
    with open(path, 'rb') as f:
        def next_token():
            buf = b''
            while True:
                c = f.read(1)
                if c == b'#':
                    f.readline()
                    continue
                if c in (b' ', b'\t', b'\n', b'\r'):
                    if buf:
                        return buf.decode()
                    continue
                buf += c
        assert next_token() == 'P5', 'Only binary P5 PGM supported'
        W, H = int(next_token()), int(next_token())
        int(next_token())
        data = np.frombuffer(f.read(H * W), dtype=np.uint8).reshape(H, W)
    return data


def build_free_mask(pgm_path: Path, clearance_m: float = INFLATION) -> np.ndarray:
    img      = read_pgm(pgm_path)
    occupied = img < 128
    dist_px  = distance_transform_edt(~occupied)
    return dist_px >= (clearance_m / MAP_RES)


def pixel_to_world(col: int, row: int, H: int):
    wx = MAP_ORIG_X + (col + 0.5) * MAP_RES
    wy = MAP_ORIG_Y + (H - 1 - row + 0.5) * MAP_RES
    return wx, wy


def sample_candidates(free_mask: np.ndarray, current_xy: np.ndarray,
                      current_yaw: float, n: int, min_dist: float) -> np.ndarray:
    H, W = free_mask.shape
    rows, cols = np.where(free_mask)
    perm = np.random.permutation(len(rows))
    out = []
    for i in perm:
        wx, wy = pixel_to_world(cols[i], rows[i], H)
        if np.hypot(wx - current_xy[0], wy - current_xy[1]) < min_dist:
            continue
        direction  = np.arctan2(wy - current_xy[1], wx - current_xy[0])
        angle_diff = abs(np.arctan2(np.sin(direction - current_yaw),
                                    np.cos(direction - current_yaw)))
        if np.deg2rad(60) < angle_diff < np.deg2rad(120):
            continue
        out.append([wx, wy])
        if len(out) == n:
            break
    if not out:
        sys.exit('[error] No valid candidates — check map, --min-dist, --current-yaw')
    return np.array(out)


def reference_grid(free_mask: np.ndarray, n: int) -> np.ndarray:
    H, W = free_mask.shape
    rows, cols = np.where(free_mask)
    idx = np.random.choice(len(rows), size=min(n, len(rows)), replace=False)
    return np.array([pixel_to_world(cols[i], rows[i], H) for i in idx])


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class GPExplorer(Node):
    def __init__(self):
        super().__init__('gp_explorer_gpu')
        self._ac   = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._pose = None
        self.create_subscription(PoseStamped, POSE_TOPIC, self._pose_cb, 10)

    def _pose_cb(self, msg: PoseStamped):
        self._pose = msg

    def get_current_pose(self):
        print(f'  waiting for pose on {POSE_TOPIC} …', flush=True)
        while self._pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        p   = self._pose.pose
        q   = p.orientation
        yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
        print(f'  pose received: x={p.position.x:.3f}  y={p.position.y:.3f}'
              f'  yaw={np.rad2deg(yaw):.1f}°')
        return p.position.x, p.position.y, yaw

    def compute_path(self, sx: float, sy: float,
                     gx: float, gy: float) -> np.ndarray | None:
        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('compute_path_to_pose action server not available')
            sys.exit(1)
        goal = ComputePathToPose.Goal()
        goal.use_start  = True
        goal.planner_id = PLANNER_ID
        for pose, (px, py) in ((goal.start, (sx, sy)), (goal.goal, (gx, gy))):
            pose.header.frame_id    = GLOBAL_FRAME
            pose.header.stamp       = self.get_clock().now().to_msg()
            pose.pose.position.x    = float(px)
            pose.pose.position.y    = float(py)
            pose.pose.orientation.w = 1.0
        future = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return None
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=10.0)
        res = res_future.result()
        if res is None:
            return None
        poses = res.result.path.poses
        if not poses:
            return None
        return np.array([[p.pose.position.x, p.pose.position.y] for p in poses])


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='GP exploration with GPyTorch/CUDA (GPU-accelerated alternative to gp_explorer.py)')
    parser.add_argument('--bag-dir',       required=True, type=Path,
                        help='Directory containing ros2 bag subdirectories')
    parser.add_argument('--map',
                        default=Path(__file__).resolve().parent.parent / 'maps' / 'map.pgm',
                        type=Path, help='Path to map.pgm')
    parser.add_argument('--n-samples',     default=500,  type=int,
                        help='Number of candidate goals to evaluate')
    parser.add_argument('--min-dist',      default=2.5,  type=float,
                        help='Minimum distance from current pose to any candidate (m)')
    parser.add_argument('--min-clearance', default=1.0,  type=float,
                        help='Minimum clearance from obstacles (m); >= inflation_radius')
    parser.add_argument('--name',          required=True, type=str,
                        help='Run name — saved as <bag-dir>/<name>_path.csv')
    parser.add_argument('--n-iter',        default=100,  type=int,
                        help='Adam iterations for GP hyperparameter training (default 100)')
    args = parser.parse_args()

    if args.min_clearance < INFLATION:
        print(f'[warn] --min-clearance {args.min_clearance} m < inflation {INFLATION} m; raising')
        args.min_clearance = INFLATION

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f'\n[gpu] device: {device}', end='')
    if device.type == 'cuda':
        print(f'  ({torch.cuda.get_device_name(0)})', end='')
    print()

    # ── 1. Train GP ───────────────────────────────────────────────────────────
    print('\n[1/5] Loading bags and training GP …')
    X_train, z_train = load_bags(args.bag_dir)
    model, likelihood = train_gp(X_train, z_train, device, args.n_iter)

    # ── 2. Pose + candidates ──────────────────────────────────────────────────
    print('\n[2/5] Getting current pose and sampling candidates …')
    rclpy.init()
    node = GPExplorer()
    cx, cy, cyaw = node.get_current_pose()
    current_xy = np.array([cx, cy])

    sample_mask = build_free_mask(args.map, args.min_clearance)
    ref_mask    = build_free_mask(args.map, INFLATION)
    candidates  = sample_candidates(sample_mask, current_xy, cyaw,
                                    args.n_samples, args.min_dist)
    X_ref = reference_grid(ref_mask, N_REF_POINTS)
    print(f'  {len(candidates)} candidates, {len(X_ref)} reference points')

    # ── 3. Collect all Nav2 paths (ALC deferred to batch GPU step) ────────────
    print('\n[3/5] Planning paths via Nav2 …')
    valid_goals    = []
    valid_paths    = []   # full paths for saving
    eval_paths     = []   # downsampled for ALC
    path_lengths_m = []

    for i, (gx, gy) in enumerate(candidates):
        pts = node.compute_path(cx, cy, gx, gy)
        if pts is None or len(pts) < 2:
            continue
        length = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
        if length < 0.05:
            continue

        if len(pts) > MAX_PATH_PTS:
            idx      = np.linspace(0, len(pts) - 1, MAX_PATH_PTS, dtype=int)
            pts_eval = pts[idx]
        else:
            pts_eval = pts

        valid_goals.append((gx, gy))
        valid_paths.append(pts)
        eval_paths.append(pts_eval)
        path_lengths_m.append(length)

        if (i + 1) % 20 == 0 or i == len(candidates) - 1:
            print(f'  [{i+1}/{len(candidates)}]  valid paths so far: {len(valid_goals)}',
                  flush=True)

    node.destroy_node()
    rclpy.shutdown()

    # ── 4. Batch ALC on GPU ───────────────────────────────────────────────────
    n_valid = len(valid_goals)
    print(f'\n[4/5] Computing Cohn ALC for {n_valid} paths on {device} …')
    t0         = time.time()
    alc_scores = batch_cohn_alc(model, device, eval_paths, X_ref)
    print(f'  batch ALC done in {time.time() - t0:.2f}s')

    scores = [alc / length
              for alc, length in zip(alc_scores, path_lengths_m)]

    # ── 5. Report ─────────────────────────────────────────────────────────────
    if not scores:
        print('\n[5/5] No valid path found for any candidate.')
        sys.exit(1)

    best   = int(np.argmax(scores))
    bx, by = valid_goals[best]

    out_path = args.bag_dir / f'{args.name}_path.csv'
    with open(out_path, 'w') as f:
        f.write('x,y\n')
        for px, py in valid_paths[best]:
            f.write(f'{px:.6f},{py:.6f}\n')

    pose_csv = Path(__file__).resolve().parent.parent / 'pose.csv'
    with open(pose_csv, 'w') as f:
        f.write('x,y,z\n')
        f.write(f'{bx:.6f},{by:.6f},0.0\n')

    print(f'\n[5/5] Best exploration goal:')
    print(f'  x = {bx:.4f}')
    print(f'  y = {by:.4f}')
    print(f'  ALC / distance score = {scores[best]:.6f}')
    print(f'  ({n_valid} of {len(candidates)} candidates had valid paths)')
    print(f'  planned path saved → {out_path}')
    print(f'  goal pose saved    → {pose_csv}')


if __name__ == '__main__':
    main()
