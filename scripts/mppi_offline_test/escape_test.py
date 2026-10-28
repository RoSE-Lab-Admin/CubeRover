#!/usr/bin/env python3
"""Offline test of the get-unstuck behaviour (nav2_stack/escape.py).

Needs, on an isolated domain: the real nav2.launch.py + rover_sim.py (no wall
physics, so the rover can be placed overlapping a wall). Per scenario this
resets the sim pose, starts the real path_follower (from source) with one
waypoint, and records the outcome, escape logs and track. Also checks the
"no safe way out" rule on a synthetic map with a pillar, and the
pre-explorer escape (no goal).

    python3 escape_test.py --out escape_results [--only 1 2]
"""
import argparse
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

REPO = Path('/home/roselab/CubeRover_waypoint')
sys.path.insert(0, str(REPO / 'src/nav2_stack'))
from nav2_stack import escape as esc  # noqa: E402  (source version)

# name, start (x, y, yaw deg), goal (x, y), escape on?
SCENARIOS = {
    1: ('bag 09 end, stuck in the bottom wall, escape OFF', (0.56, -2.76, 41.0), (-2.3, 0.9), False),
    2: ('bag 09 end, stuck in the bottom wall', (0.56, -2.76, 41.0), (-2.3, 0.9), True),
    3: ('same spot, facing the wall', (0.56, -2.76, -139.0), (-2.3, 0.9), True),
    4: ('top-right corner, nose into the corner', (2.05, 1.85, 45.0), (-1.0, -0.5), True),
    5: ('right wall, parallel to it', (2.25, 0.0, 90.0), (-1.0, 0.5), True),
    6: ('free space (regression: no escape)', (-1.0, -0.3, 0.0), (0.8, 0.5), True),
}


class Driver(Node):
    def __init__(self):
        super().__init__('escape_test_driver')
        self.reset_pub = self.create_publisher(PoseStamped, '/sim/reset_pose', 10)
        # latched with depth 1: a new path_follower gets only the current goal
        self.wp_pub = self.create_publisher(PathMsg, '/sim_waypoints',
                                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pose, self.track, self.result = None, [], None
        self.create_subscription(PoseStamped, '/FitRosey_passive/pose', self._pose, 20)
        self.create_subscription(String, '/trial_goal_result', self._result, 10)

    def _pose(self, m):
        q = m.pose.orientation
        self.pose = (m.pose.position.x, m.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        self.track.append((time.monotonic(),) + self.pose)

    def _result(self, m):
        self.result = m.data

    def spin_for(self, s):
        t = time.monotonic() + s
        while time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.05)


def ps(x, y, yaw, stamp, frame='world'):
    p = PoseStamped()
    p.header.frame_id, p.header.stamp = frame, stamp
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def run_scenario(d, k, outdir, timeout):
    name, (sx, sy, syaw), goal, on = SCENARIOS[k]
    d.result, d.track = None, []
    for _ in range(5):
        d.reset_pub.publish(ps(sx, sy, math.radians(syaw), d.get_clock().now().to_msg()))
        d.spin_for(0.1)
    d.spin_for(1.0)
    # publish the goal BEFORE starting path_follower so it never sees an old one
    path = PathMsg()
    path.header.frame_id = 'world'
    path.poses = [ps(goal[0], goal[1], 0.0, d.get_clock().now().to_msg())]
    d.wp_pub.publish(path)
    d.spin_for(0.5)
    d.track = []
    log = open(outdir / f'scenario{k}_path_follower.log', 'w')
    env = dict(os.environ, PYTHONPATH=f"{REPO / 'src/nav2_stack'}:{os.environ.get('PYTHONPATH', '')}")
    pf = subprocess.Popen([sys.executable, str(REPO / 'src/nav2_stack/nav2_stack/path_follower.py'),
                           '--ros-args', '-p', f'escape_when_stuck:={"true" if on else "false"}'],
                          stdout=log, stderr=subprocess.STDOUT, env=env)
    t0 = time.monotonic()
    while d.result is None and time.monotonic() - t0 < timeout:
        d.spin_for(0.2)
    elapsed = time.monotonic() - t0
    pf.send_signal(2)
    try:
        pf.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pf.kill()
    log.close()
    text = (outdir / f'scenario{k}_path_follower.log').read_text()
    esc_lines = [l.split(']: ', 1)[-1] for l in text.splitlines() if 'escape' in l and 'escape_when' not in l]
    tr = np.array(d.track) if d.track else np.zeros((0, 4))
    final = tr[-1, 1:3] if len(tr) else (math.nan, math.nan)
    reached = math.hypot(final[0] - goal[0], final[1] - goal[1]) < 0.5
    return dict(k=k, name=name, start=(sx, sy, syaw), goal=goal, result=d.result or 'TIMEOUT',
                reached=reached, time=elapsed, escape_log=esc_lines, track=tr)


def pillar_check():
    """Synthetic map: rover touching a pillar, with another pillar right
    behind and a wall ahead-left/right -- every candidate reaches a different
    obstacle or drives in further, so no escape must be chosen."""
    class _Log:
        def warn(self, m): print('   ', m)
        info = warn
    e = esc.Escaper.__new__(esc.Escaper)
    e.log = _Log()
    res = 0.05
    grid = np.zeros((80, 80), np.uint8)
    grid[38:42, 30:34] = 254          # pillar the rover is touching (just behind its rear)
    grid[38:42, 48:52] = 254          # second pillar just ahead
    grid[24:28, 30:52] = 254          # wall below
    grid[52:56, 30:52] = 254          # wall above
    from scipy.ndimage import binary_dilation
    infl = binary_dilation(grid == 254, iterations=int(0.23 / res)) & (grid != 254)
    grid[infl] = 253
    e.res, e.ox, e.oy, e.grid = res, 0.0, 0.0, grid
    from scipy.ndimage import distance_transform_edt, label
    e.lethal_dist = distance_transform_edt(grid != 254) * res
    e.labels, _ = label(grid == 254)
    step = res / 2
    xs = np.arange(-esc.FOOTPRINT_HALF_L, esc.FOOTPRINT_HALF_L + 1e-9, step)
    ys = np.arange(-esc.FOOTPRINT_HALF_W, esc.FOOTPRINT_HALF_W + 1e-9, step)
    gx, gy = np.meshgrid(xs, ys)
    e.fp_area = np.stack([gx.ravel(), gy.ravel()], 1)
    e.fp_edge = e.fp_area[(np.isclose(np.abs(gx), xs.max()) | np.isclose(np.abs(gy), ys.max())).ravel()]
    pose = (2.05, 2.0, 0.0)   # rear corners overlapping the first pillar
    best, results = e.choose(pose)
    return pose, best, results


def plot(results, outdir):
    e = esc.Escaper.__new__(esc.Escaper)
    n = len(results)
    fig, axes = plt.subplots(1, n, figsize=(4.4 * n, 4.6), squeeze=False)
    for ax, r in zip(axes[0], results):
        tr = r['track']
        if len(tr):
            ax.plot(tr[:, 1], tr[:, 2], 'k-', lw=1.3, label='driven')
        sx, sy, syaw = r['start']
        p0 = (sx, sy, math.radians(syaw))
        for gear, k, name in esc.CANDIDATES:
            pts = np.array([esc.Escaper._along(p0, gear, k, s) for s in np.linspace(0, esc.MAX_DIST, 30)])
            ax.plot(pts[:, 0], pts[:, 1], ':', color='tab:blue' if gear > 0 else 'tab:orange', lw=1)
        ax.annotate('', (sx + 0.4 * math.cos(p0[2]), sy + 0.4 * math.sin(p0[2])), (sx, sy),
                    arrowprops=dict(arrowstyle='-|>', lw=2, color='tab:green'))
        ax.plot(*r['goal'], 'r*', ms=12)
        ax.set_title(f"{r['k']}: {r['name']}\n{r['result']}, reached={r['reached']}", fontsize=8)
        ax.set_xlim(-3.0, 2.8); ax.set_ylim(-3.4, 2.8); ax.set_aspect('equal')
        from PIL import Image
        img = np.array(Image.open(REPO / 'src/nav2_stack/maps/map.pgm'))
        ax.imshow(img < 128, extent=[-5, 5, -5, 5], cmap='Greys', alpha=0.4, origin='upper')
    fig.suptitle('escape test: dotted = the six candidate motions (blue fwd, orange rev), black = driven',
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(outdir / 'escape_test.png', dpi=75)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='escape_results')
    ap.add_argument('--only', nargs='*', type=int)
    ap.add_argument('--timeout', type=float, default=120.0)
    a = ap.parse_args()
    outdir = Path(a.out)
    outdir.mkdir(parents=True, exist_ok=True)

    print('== no-safe-way-out rule (synthetic map with pillars)')
    pose, best, results = pillar_check()
    for r in results:
        print(f"   {r['name']:22s} " + (f"VALID free after {r['dist']:.2f} m, clearance {r['clearance']:.2f} m"
                                         if r['valid'] else r['reason']))
    print(f"   chosen: {best['name'] if best else 'NONE -> no escape'}")

    rclpy.init()
    d = Driver()
    d.spin_for(2.0)
    results = []
    for k in sorted(SCENARIOS):
        if a.only and k not in a.only:
            continue
        r = run_scenario(d, k, outdir, a.timeout)
        results.append(r)
        print(f"\n== scenario {k}: {r['name']}\n   result {r['result']}, goal reached {r['reached']}, "
              f"{r['time']:.0f} s")
        for l in r['escape_log']:
            print('   ', l[:300])

    # pre-explorer escape (autonomous_trials): stuck, no goal -> toward the arena centre
    print('\n== pre-explorer escape (no goal, like autonomous_trials before gp_explorer)')
    d.track = []
    for _ in range(5):
        d.reset_pub.publish(ps(0.56, -2.76, math.radians(41.0), d.get_clock().now().to_msg()))
        d.spin_for(0.1)
    d.spin_for(1.0)
    e = esc.Escaper(d, lambda: d.pose, esc.default_planner_id())
    ok, msg = e.run((0.0, 0.0))
    print(f'   escaped={ok}: {msg}; now stuck={e.is_stuck(d.pose)} at ({d.pose[0]:.2f}, {d.pose[1]:.2f})')
    plot(results, outdir)
    print(f'\nplot: {outdir / "escape_test.png"}')
    d.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
