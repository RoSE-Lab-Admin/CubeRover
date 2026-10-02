#!/usr/bin/env python3
"""Plan a set of hard (sharper-than-turning-radius) start->goal scenarios
against a running planner_test.launch.py, under several planner configs, and
report/plot direction changes per path.

Configs compared (same start/goal per scenario):
  stock    GridBased (stock Nav2), goal yaw = path_follower's arc heading
  custom   GridBasedCustom, ignore_goal_heading=false, same arc goal yaw
  free     GridBasedCustom, ignore_goal_heading=true (goal yaw ignored)

    python3 plan_scenarios.py [--out results.png]
"""
import argparse
import math
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node

FRAME = 'world'
IGNORE_PARAM = 'GridBasedCustom.ignore_goal_heading'

# name, (start x, y, yaw deg), (goal x, y)
SCENARIOS = [
    ('goal directly behind', (-0.4, -0.3, 0), (-1.6, -0.3)),
    ('hard left', (-0.6, -0.6, 0), (-0.4, 0.5)),
    ('back-left', (-0.2, -0.6, 0), (-1.2, 0.4)),
    ('behind-right', (0.0, 0.3, 0), (-0.6, -0.6)),
    ('facing wall, goal behind', (0.9, -0.3, 0), (-1.0, -0.3)),
    ('side step', (-0.6, -0.3, 0), (-0.1, 0.2)),
    ('U-turn near wall', (0.6, 1.2, 0), (0.0, 0.4)),
]

CONFIGS = [
    ('stock GridBased, arc yaw', 'GridBased', None),
    ('GridBasedCustom, arc yaw', 'GridBasedCustom', False),
    ('GridBasedCustom, heading free', 'GridBasedCustom', True),
]


def arc_arrival_heading(x0, y0, theta0, x1, y1):
    # Copy of path_follower.PathFollower.arc_arrival_heading
    dx, dy = x1 - x0, y1 - y0
    denom = dx * np.sin(theta0) - dy * np.cos(theta0)
    if abs(denom) < 1e-6:
        return np.arctan2(dy, dx)
    r = -(dx**2 + dy**2) / (2.0 * denom)
    cx = x0 - r * np.sin(theta0)
    cy = y0 + r * np.cos(theta0)
    rx, ry = x1 - cx, y1 - cy
    if r > 0:
        return np.arctan2(rx, -ry)
    return np.arctan2(-rx, ry)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def analyze(path_xyyaw):
    """Return (length, direction_changes, gear signs per segment)."""
    p = path_xyyaw
    if len(p) < 2:
        return 0.0, 0, np.array([])
    d = np.diff(p[:, :2], axis=0)
    seg_len = np.hypot(d[:, 0], d[:, 1])
    heading = np.stack([np.cos(p[:-1, 2]), np.sin(p[:-1, 2])], axis=1)
    sign = np.sign(np.einsum('ij,ij->i', d, heading))
    # carry previous gear through zero-length / perpendicular segments
    for i in range(len(sign)):
        if sign[i] == 0 or seg_len[i] < 1e-4:
            sign[i] = sign[i - 1] if i > 0 else 1
    changes = int(np.count_nonzero(np.diff(sign) != 0))
    return float(seg_len.sum()), changes, sign


def straight_after_resets(path_xyyaw, sign, turn_eps_deg=7.5):
    """Straight distance driven after the start and after each reversal,
    before the first heading change (inf if the leg never turns)."""
    p = path_xyyaw
    seg_len = np.hypot(*np.diff(p[:, :2], axis=0).T)
    dyaw = np.abs(np.rad2deg(np.angle(np.exp(1j * np.diff(p[:, 2])))))
    out, run, counting = [], 0.0, True
    for i in range(len(seg_len)):
        if i > 0 and sign[i] != sign[i - 1]:
            if counting:
                out.append(math.inf)
            run, counting = 0.0, True
        if counting:
            if dyaw[i] > turn_eps_deg:
                out.append(run)
                counting = False
            else:
                run += seg_len[i]
    if counting:
        out.append(math.inf)
    return out


class Bench(Node):
    def __init__(self):
        super().__init__('planner_offline_bench')
        self.ac = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.get_cli = self.create_client(GetParameters, '/planner_server/get_parameters')
        self.set_cli = self.create_client(SetParameters, '/planner_server/set_parameters')
        self.costmap_cli = self.create_client(GetCostmap, '/global_costmap/get_costmap')

    def call(self, cli, req, timeout=10.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {cli.srv_name} unavailable')
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def get_param(self, name):
        res = self.call(self.get_cli, GetParameters.Request(names=[name]))
        return res.values[0]

    def set_bool(self, name, value):
        p = Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=value))
        res = self.call(self.set_cli, SetParameters.Request(parameters=[p]))
        if not res.results[0].successful:
            raise RuntimeError(f'set {name} failed: {res.results[0].reason}')

    def plan(self, planner_id, start, goal):
        g = ComputePathToPose.Goal()
        g.use_start = True
        g.planner_id = planner_id
        for pose, (x, y, yaw) in ((g.start, start), (g.goal, goal)):
            pose.header.frame_id = FRAME
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
        t0 = time.monotonic()
        fut = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return None, time.monotonic() - t0, 'rejected'
        rfut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=15.0)
        res = rfut.result()
        dt = time.monotonic() - t0
        if res is None or res.status != GoalStatus.STATUS_SUCCEEDED or not res.result.path.poses:
            code = res.result.error_code if res is not None else 'timeout'
            return None, dt, f'failed (error {code})'
        arr = np.array([[p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation)]
                        for p in res.result.path.poses])
        return arr, res.result.planning_time.sec + res.result.planning_time.nanosec * 1e-9, 'ok'


def draw_costmap(ax, cm):
    # nav2_msgs/Costmap: 0 free .. 253 inscribed, 254 lethal, 255 unknown
    info = cm.metadata
    grid = np.array(cm.data, dtype=float).reshape(info.size_y, info.size_x)
    grid[grid == 255] = 128
    x0, y0 = info.origin.position.x, info.origin.position.y
    ext = [x0, x0 + info.size_x * info.resolution, y0, y0 + info.size_y * info.resolution]
    ax.imshow(grid, origin='lower', extent=ext, cmap='Greys', vmin=0, vmax=254,
              interpolation='nearest')


def draw_pose(ax, x, y, yaw, color, length=0.35, **kw):
    ax.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw), width=0.03,
             head_width=0.12, length_includes_head=True, color=color, **kw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='planner_scenarios.png')
    args = ap.parse_args()

    rclpy.init()
    node = Bench()
    print('waiting for compute_path_to_pose and costmap ...', flush=True)
    node.ac.wait_for_server()
    # static costmap only publishes once, so fetch it on demand instead
    costmap = node.call(node.costmap_cli, GetCostmap.Request()).map
    plugins = list(node.get_param('planner_plugins').string_array_value)
    print(f'planner_plugins: {plugins}  (navigation would use {plugins[0]})')
    original_ignore = node.get_param(IGNORE_PARAM).bool_value

    results = {}
    try:
        for ci, (cname, planner_id, ignore) in enumerate(CONFIGS):
            if ignore is not None:
                node.set_bool(IGNORE_PARAM, ignore)
            for si, (sname, start, goal_xy) in enumerate(SCENARIOS):
                sx, sy, syaw_deg = start
                syaw = math.radians(syaw_deg)
                gyaw = arc_arrival_heading(sx, sy, syaw, *goal_xy)
                path, dt, status = node.plan(planner_id, (sx, sy, syaw), (*goal_xy, gyaw))
                results[(si, ci)] = (path, dt, status, gyaw)
    finally:
        node.set_bool(IGNORE_PARAM, original_ignore)

    # ── table ──
    print()
    header = f'{"scenario":28s}' + ''.join(f'| {c[0]:32s}' for c in CONFIGS)
    print(header)
    print('-' * len(header))
    for si, (sname, _, _) in enumerate(SCENARIOS):
        row = f'{sname:28s}'
        for ci in range(len(CONFIGS)):
            path, dt, status, _ = results[(si, ci)]
            if path is None:
                cell = status
            else:
                length, changes, _ = analyze(path)
                _, _, sign = analyze(path)
                legs = straight_after_resets(path, sign)
                straight = min(legs)
                cell = (f'{changes} chg {length:3.1f}m '
                        f'straight {"-" if math.isinf(straight) else f"{straight:.2f}"}')
            row += f'| {cell:32s}'
        print(row)

    # ── plot ──
    nrow, ncol = len(SCENARIOS), len(CONFIGS)
    fig, axes = plt.subplots(nrow, ncol, figsize=(4.2 * ncol, 3.9 * nrow), squeeze=False)
    for si, (sname, start, goal_xy) in enumerate(SCENARIOS):
        sx, sy, syaw_deg = start
        xs = [sx, goal_xy[0]]
        ys = [sy, goal_xy[1]]
        for ci, (cname, _, ignore) in enumerate(CONFIGS):
            ax = axes[si][ci]
            draw_costmap(ax, costmap)
            path, dt, status, gyaw = results[(si, ci)]
            title = f'{sname}\n{cname}\n'
            if path is None:
                title += status
            else:
                length, changes, sign = analyze(path)
                title += f'{changes} direction change(s), {length:.1f} m, {dt * 1000:.0f} ms'
                for i, s in enumerate(sign):
                    ax.plot(path[i:i + 2, 0], path[i:i + 2, 1], '-',
                            color='tab:blue' if s > 0 else 'tab:orange', lw=2.2)
                xs += list(path[:, 0])
                ys += list(path[:, 1])
                # final heading the planner produced
                draw_pose(ax, path[-1, 0], path[-1, 1], path[-1, 2], 'tab:purple', alpha=0.9)
            draw_pose(ax, sx, sy, math.radians(syaw_deg), 'tab:green')
            ax.plot(*goal_xy, 'r*', ms=13)
            if not ignore:
                draw_pose(ax, *goal_xy, gyaw, 'red', alpha=0.45)  # requested goal yaw
            pad = 0.9
            ax.set_xlim(min(xs) - pad, max(xs) + pad)
            ax.set_ylim(min(ys) - pad, max(ys) + pad)
            ax.set_aspect('equal')
            ax.set_title(title, fontsize=9)
            ax.tick_params(labelsize=7)
    fig.legend(handles=[
        plt.Line2D([], [], color='tab:blue', lw=2.2, label='forward'),
        plt.Line2D([], [], color='tab:orange', lw=2.2, label='reverse'),
        plt.Line2D([], [], color='tab:green', lw=3, label='start pose'),
        plt.Line2D([], [], color='red', alpha=0.45, lw=3, label='requested goal yaw (arc)'),
        plt.Line2D([], [], color='tab:purple', lw=3, label='planned final heading'),
    ], loc='upper center', ncol=5, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 1 - 0.5 / (3.9 * nrow)))
    fig.savefig(args.out, dpi=90)
    print(f'\nplot written to {args.out}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
