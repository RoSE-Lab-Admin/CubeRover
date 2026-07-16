#!/usr/bin/env python3
"""
GP Exploration — reads rosbag data, trains a GP on (x,y)->z,
samples candidate goals inside the navigable map, plans RS paths via Nav2,
and returns the goal with highest expected information gain per metre
using Cohn's Active Learning Criterion (ALC).

Usage (Nav2 must be running with planner_server active):

  python3 gp_explorer.py --bag-dir /path/to/bags \\
                         --current-x 0.0 --current-y 0.0

Optional flags:
  --map        /path/to/map.pgm   (default: ../maps/map.pgm relative to this file)
  --n-samples  200                (candidate goals to evaluate)
  --min-dist   1.0                (min metres from current pose to candidate)

Dependencies:
  pip3 install --user scikit-learn rosbags scipy
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from scipy.ndimage import distance_transform_edt
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


# ── Map config — must match map.yaml ─────────────────────────────────────────
MAP_RES    = 0.2     # metres per pixel
MAP_ORIG_X = -5.0    # world x of left (col=0) edge
MAP_ORIG_Y = -5.0    # world y of bottom edge
INFLATION  = 0.75    # metres — match inflation_radius in nav2_param2.yaml

# ── Nav2 settings ─────────────────────────────────────────────────────────────
GLOBAL_FRAME = 'world'
PLANNER_ID   = 'GridBased'

# ── Internal constants ────────────────────────────────────────────────────────
POSE_TOPIC    = '/FitRosey_V1/pose'
N_REF_POINTS  = 150   # uniform reference grid used for Cohn ALC integration
MAX_PATH_PTS  = 80    # downsample long paths to this count before ALC
DOWNSAMPLE_FACTOR = 100  # keep every Nth pose sample from bags


# ─────────────────────────────────────────────────────────────────────────────
# Bag loading
# ─────────────────────────────────────────────────────────────────────────────

def load_bags(bag_dir: Path):
    """
    Read all ros2 bag directories under bag_dir and return
    X (N,2) array of (x,y) and z (N,) from /FitRosey_V1/pose.
    """
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
        with Reader(bag) as reader:
            if POSE_TOPIC not in reader.topics:
                print(f'    skipping — {POSE_TOPIC!r} not present')
                continue
            conns = [c for c in reader.connections if c.topic == POSE_TOPIC]
            for conn, _, rawdata in reader.messages(connections=conns):
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                X_list.append([msg.pose.position.x, msg.pose.position.y])
                z_list.append(msg.pose.position.z)

    if not X_list:
        sys.exit('[error] No pose data parsed from any bag')

    X = np.array(X_list, dtype=float)
    z = np.array(z_list, dtype=float)

    idx = np.arange(0, len(X), DOWNSAMPLE_FACTOR)
    X, z = X[idx], z[idx]
    print(f'  downsampled by {DOWNSAMPLE_FACTOR}x → {len(X)} training points total')
    return X, z


# ─────────────────────────────────────────────────────────────────────────────
# GP training
# ─────────────────────────────────────────────────────────────────────────────

def train_gp(X: np.ndarray, z: np.ndarray) -> GaussianProcessRegressor:
    kernel = (RBF(length_scale=0.5, length_scale_bounds=(0.05, 5.0))
              + WhiteKernel(noise_level=1e-3, noise_level_bounds=(1e-6, 0.5)))
    gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=5,
                                  normalize_y=True)
    gp.fit(X, z)
    print(f'  fitted kernel: {gp.kernel_}')
    return gp


# ─────────────────────────────────────────────────────────────────────────────
# Map utilities
# ─────────────────────────────────────────────────────────────────────────────

def read_pgm(path: Path) -> np.ndarray:
    """Parse a binary P5 PGM file robustly (handles comments)."""
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
        int(next_token())  # maxval — unused
        data = np.frombuffer(f.read(H * W), dtype=np.uint8).reshape(H, W)
    return data


def build_free_mask(pgm_path: Path) -> np.ndarray:
    """
    Return boolean (H,W) mask: True = navigable (free AND outside inflation zone).
    Pixel < 128 = occupied; inflate by INFLATION metres via distance transform.
    """
    img      = read_pgm(pgm_path)
    occupied = img < 128
    dist_px  = distance_transform_edt(~occupied)
    return dist_px >= (INFLATION / MAP_RES)


def pixel_to_world(col: int, row: int, H: int):
    """Return world (x, y) at the centre of pixel (col, row)."""
    wx = MAP_ORIG_X + (col + 0.5) * MAP_RES
    wy = MAP_ORIG_Y + (H - 1 - row + 0.5) * MAP_RES
    return wx, wy


def sample_candidates(free_mask: np.ndarray, current_xy: np.ndarray,
                      n: int, min_dist: float) -> np.ndarray:
    """Sample up to n world (x,y) candidates that are free and >= min_dist away."""
    H, W = free_mask.shape
    rows, cols = np.where(free_mask)
    perm = np.random.permutation(len(rows))
    out = []
    for i in perm:
        wx, wy = pixel_to_world(cols[i], rows[i], H)
        if np.hypot(wx - current_xy[0], wy - current_xy[1]) >= min_dist:
            out.append([wx, wy])
            if len(out) == n:
                break
    if not out:
        sys.exit('[error] No valid candidates — check map path and --min-dist')
    return np.array(out)


def reference_grid(free_mask: np.ndarray, n: int) -> np.ndarray:
    """Random sample of n free-cell world positions used as Cohn ALC reference."""
    H, W = free_mask.shape
    rows, cols = np.where(free_mask)
    idx = np.random.choice(len(rows), size=min(n, len(rows)), replace=False)
    return np.array([pixel_to_world(cols[i], rows[i], H) for i in idx])


# ─────────────────────────────────────────────────────────────────────────────
# Cohn's Active Learning Criterion
# ─────────────────────────────────────────────────────────────────────────────

def cohn_alc(gp: GaussianProcessRegressor,
             path_xy: np.ndarray,
             X_ref: np.ndarray) -> float:
    """
    Estimate the total expected reduction in posterior variance over X_ref
    if the GP were to observe at every point along the path (greedy/additive).

    ALC(path) = Σ_{x* ∈ path}  ||k_post(X_ref, x*)||² / σ²_post(x*)

    where k_post(·,·) is the GP posterior cross-covariance and σ²_post(x*)
    is the posterior variance at x*.  A higher score means more uncertainty
    is resolved per observation.

    Implementation: concatenate path + X_ref into one predict call so sklearn
    returns the full posterior covariance matrix in a single pass.
    """
    n = len(path_xy)
    if n == 0:
        return 0.0

    all_pts = np.vstack([path_xy, X_ref])
    _, cov  = gp.predict(all_pts, return_cov=True)

    K_cross = cov[:n, n:]                           # (n_path, n_ref)
    sigma2  = np.maximum(np.diag(cov)[:n], 1e-12)  # (n_path,) posterior variance

    return float(np.sum(np.sum(K_cross ** 2, axis=1) / sigma2))


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 node — calls Nav2 planner
# ─────────────────────────────────────────────────────────────────────────────

class GPExplorer(Node):
    def __init__(self):
        super().__init__('gp_explorer')
        self._ac = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def compute_path(self, sx: float, sy: float,
                     gx: float, gy: float) -> np.ndarray | None:
        """
        Call Nav2 ComputePathToPose and return (N,2) array of path (x,y),
        or None if planning failed.
        """
        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('compute_path_to_pose action server not available')
            sys.exit(1)

        goal = ComputePathToPose.Goal()
        goal.use_start   = True
        goal.planner_id  = PLANNER_ID

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


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Select the most informative exploration goal using GP + Cohn ALC')
    parser.add_argument('--bag-dir',   required=True, type=Path,
                        help='Directory containing ros2 bag subdirectories')
    parser.add_argument('--current-x', required=True, type=float,
                        help='Current rover x position (world frame)')
    parser.add_argument('--current-y', required=True, type=float,
                        help='Current rover y position (world frame)')
    parser.add_argument('--map',
                        default=Path(__file__).resolve().parent.parent / 'maps' / 'map.pgm',
                        type=Path, help='Path to map.pgm')
    parser.add_argument('--n-samples', default=200, type=int,
                        help='Number of candidate goals to evaluate')
    parser.add_argument('--min-dist',  default=1.0, type=float,
                        help='Minimum distance from current pose to any candidate (m)')
    args = parser.parse_args()

    current_xy = np.array([args.current_x, args.current_y])

    # ── 1. Train GP ───────────────────────────────────────────────────────────
    print('\n[1/5] Loading bags and training GP …')
    X_train, z_train = load_bags(args.bag_dir)
    gp = train_gp(X_train, z_train)

    # ── 2. Sample candidates ──────────────────────────────────────────────────
    print('\n[2/5] Sampling candidate goals …')
    free_mask  = build_free_mask(args.map)
    candidates = sample_candidates(free_mask, current_xy,
                                   args.n_samples, args.min_dist)
    X_ref = reference_grid(free_mask, N_REF_POINTS)
    print(f'  {len(candidates)} candidates, {len(X_ref)} reference points')

    # ── 3. Plan paths ─────────────────────────────────────────────────────────
    print('\n[3/5] Planning paths via Nav2 …')
    rclpy.init()
    node = GPExplorer()

    scores, goals = [], []

    for i, (gx, gy) in enumerate(candidates):
        pts = node.compute_path(args.current_x, args.current_y, gx, gy)
        if pts is None or len(pts) < 2:
            continue

        length = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
        if length < 0.05:
            continue

        # ── 4. Evaluate Cohn ALC ──────────────────────────────────────────────
        if len(pts) > MAX_PATH_PTS:
            idx = np.linspace(0, len(pts) - 1, MAX_PATH_PTS, dtype=int)
            pts_eval = pts[idx]
        else:
            pts_eval = pts

        alc   = cohn_alc(gp, pts_eval, X_ref)
        score = alc / length
        scores.append(score)
        goals.append((gx, gy))

        if (i + 1) % 20 == 0 or i == len(candidates) - 1:
            print(f'  [{i+1}/{len(candidates)}]  valid paths: {len(goals)}'
                  + (f'  best score: {max(scores):.4f}' if scores else ''),
                  flush=True)

    node.destroy_node()
    rclpy.shutdown()

    # ── 5. Report ─────────────────────────────────────────────────────────────
    if not goals:
        print('\n[5/5] No valid path found for any candidate.')
        sys.exit(1)

    best  = int(np.argmax(scores))
    bx, by = goals[best]

    print(f'\n[5/5] Best exploration goal:')
    print(f'  x = {bx:.4f}')
    print(f'  y = {by:.4f}')
    print(f'  ALC / distance score = {scores[best]:.6f}')
    print(f'  ({len(goals)} of {len(candidates)} candidates had valid paths)')


if __name__ == '__main__':
    main()
