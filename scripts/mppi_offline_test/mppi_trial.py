#!/usr/bin/env python3
"""Closed-loop MPPI trials against mppi_sim.launch.py.

For each (config, scenario, repeat): optionally apply controller parameter
overrides (set_parameters + lifecycle RESET/STARTUP, like
autonomous_trials.reload_controller), reset the simulated rover to the
scenario start, then replan with ComputePathToPose at 1 Hz (like the BT) and
send each plan to FollowPath until the goal is within tolerance or timeout.
Records pose, commands and plans per run to an .npz and prints metrics.

    python3 mppi_trial.py --configs B0 B1 --scenarios bag14_t20 --repeats 3 --out results/
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path as PathMsg
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.srv import GetCostmap, ManageLifecycleNodes
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node

FRAME = 'world'
GOAL_TOL = 0.5          # PositionGoalChecker xy_goal_tolerance in nav2_param2.yaml
REPLAN_PERIOD = 1.0     # BT replans at 1 Hz

# name: (start x, y, yaw deg), goal (x, y), timeout s
SCENARIOS = {
    'bag14_t20': ((-0.03, -0.26, 175.7), (1.5, -2.1), 90.0),
    'bag14_start': ((-1.49, 0.69, 110.8), (1.5, -2.1), 120.0),
    'easy_reverse': ((0.2, -0.3, 180.0), (1.3, -0.3), 40.0),
    'easy_fwd_arc': ((-1.5, -0.5, 0.0), (0.5, 0.3), 40.0),
    # 10/05 unprepped2 trial starts/goals (arcs the controller followed poorly)
    'u7_bag26': ((1.07, -1.55, 166.4), (-2.1, 1.7), 120.0),
    'u7_bag31': ((0.78, -1.01, 114.4), (-2.1, 1.7), 120.0),
    'u2_bag12': ((1.06, 0.62, 165.7), (-2.3, -0.9), 90.0),
    'u2_bag15': ((1.21, 0.94, -122.6), (1.5, -2.1), 90.0),
    'u2_bag16': ((1.20, -1.78, -58.8), (-2.7, -0.1), 90.0),
    'u2_bag17': ((-1.90, -1.02, 20.7), (1.5, 0.3), 90.0),
}

# name: FollowPath.* parameter overrides on top of the launched config
CONFIGS = {
    'B0': {},
    'B1': {'dynamics_mode': 'kinematics'},
    'B2': {'dynamics_mode': 'linear'},
}


def pval(v):
    if isinstance(v, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
    if isinstance(v, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v)
    if isinstance(v, float):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)
    if isinstance(v, str):
        return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=v)
    if isinstance(v, (list, tuple)):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                              double_array_value=[float(x) for x in v])
    raise TypeError(type(v))


def from_pval(p):
    return {ParameterType.PARAMETER_BOOL: p.bool_value, ParameterType.PARAMETER_INTEGER: p.integer_value,
            ParameterType.PARAMETER_DOUBLE: p.double_value, ParameterType.PARAMETER_STRING: p.string_value,
            ParameterType.PARAMETER_DOUBLE_ARRAY: list(p.double_array_value)}.get(p.type)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def make_pose(x, y, yaw, stamp):
    p = PoseStamped()
    p.header.frame_id = FRAME
    p.header.stamp = stamp
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class Driver(Node):
    def __init__(self):
        super().__init__('mppi_trial_driver')
        self.plan_ac = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.follow_ac = ActionClient(self, FollowPath, 'follow_path')
        self.set_cli = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.get_cli = self.create_client(GetParameters, '/controller_server/get_parameters')
        # planner params (keys starting with 'GridBasedCustom.') go to planner_server
        self.pset_cli = self.create_client(SetParameters, '/planner_server/set_parameters')
        self.pget_cli = self.create_client(GetParameters, '/planner_server/get_parameters')
        self.state_cli = self.create_client(GetState, '/controller_server/get_state')
        self.mgr_cli = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        self.reset_pub = self.create_publisher(PoseStamped, '/sim/reset_pose', 10)
        self.costmap_cli = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        self.costmap = None
        self.pose = None
        self.rec_pose, self.rec_cmd, self.rec_plan = [], [], []
        self.recording = False
        self.t0 = time.monotonic()
        self.create_subscription(PoseStamped, '/FitRosey_V1/pose', self.on_pose, 50)
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd, 50)
        # MPPI's chosen sequence: as commanded (kinematic) and as the configured model predicts
        self.rec_opt, self.rec_opt_model = [], []
        self.create_subscription(PathMsg, '/optimal_trajectory',
                                 lambda m: self.on_path(m, self.rec_opt), 10)
        self.create_subscription(PathMsg, '/optimal_trajectory_model',
                                 lambda m: self.on_path(m, self.rec_opt_model), 10)

    def now_s(self):
        return time.monotonic() - self.t0

    def on_pose(self, m):
        self.pose = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))
        if self.recording:
            self.rec_pose.append((self.now_s(), *self.pose))

    def on_path(self, m, store):
        if self.recording and m.poses:
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            store.append((t, np.array([[p.pose.position.x, p.pose.position.y,
                                        yaw_of(p.pose.orientation)] for p in m.poses])))

    def on_cmd(self, m):
        if self.recording:
            self.rec_cmd.append((self.now_s(), m.twist.linear.x, m.twist.angular.z))

    def wait(self, fut, timeout):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None

    def call(self, cli, req, timeout=30.0):
        if not cli.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f'{cli.srv_name} unavailable')
        return self.wait(cli.call_async(req), timeout)

    def load_costmap(self):
        r = self.call(self.costmap_cli, GetCostmap.Request(), 20.0)
        if r is None:
            raise RuntimeError('could not fetch the global costmap')
        md = r.map.metadata
        self.costmap = (np.array(r.map.data, dtype=np.uint8).reshape(md.size_y, md.size_x),
                        md.resolution, md.origin.position.x, md.origin.position.y)

    def in_collision(self, x, y):
        # the sim has no physics: treat the rover center in lethal/inscribed
        # cost (or off the map) as a crash and end the run
        grid, res, ox, oy = self.costmap
        i, j = int((x - ox) / res), int((y - oy) / res)
        if not (0 <= j < grid.shape[0] and 0 <= i < grid.shape[1]):
            return True
        return 253 <= grid[j, i] <= 254

    def controller_state(self):
        r = self.call(self.state_cli, GetState.Request(), 10.0)
        return r.current_state.label if r else 'unknown'

    def ensure_active(self):
        st = self.controller_state()
        if st != 'active':
            self.get_logger().warn(f'controller_server is {st}; RESET+STARTUP via lifecycle manager')
            self.reload()
            st = self.controller_state()
        if st != 'active':
            raise SystemExit(f'controller_server is {st!r}, not active -- results would be invalid')

    def _route(self, k):
        if k.startswith('GridBasedCustom.'):
            return self.pset_cli, self.pget_cli, k
        return self.set_cli, self.get_cli, 'FollowPath.' + k

    def get_params(self, names):
        out = {}
        for n in names:
            _, gcli, full = self._route(n)
            r = self.call(gcli, GetParameters.Request(names=[full]))
            out[n] = from_pval(r.values[0])
        return out

    def set_params(self, values):
        for k, v in values.items():
            scli, _, full = self._route(k)
            r = self.call(scli, SetParameters.Request(parameters=[Parameter(name=full, value=pval(v))]))
            if r is None or not r.results[0].successful:
                raise RuntimeError(f'set {full}={v} failed')

    def reload(self):
        for cmd in (ManageLifecycleNodes.Request.RESET, ManageLifecycleNodes.Request.STARTUP):
            r = self.call(self.mgr_cli, ManageLifecycleNodes.Request(command=cmd), 240.0)
            if r is None or not r.success:
                raise RuntimeError(f'lifecycle command {cmd} failed')

    def apply_config(self, overrides, baseline):
        """Set overrides (and restore every other key touched by any config to
        its launched value), then reload so Static params are re-read."""
        values = {**baseline, **overrides}
        self.set_params(values)
        self.reload()
        self.ensure_active()

    def cancel_follow(self, handle):
        if handle is not None:
            self.wait(handle.cancel_goal_async(), 5.0)

    def run(self, scenario, label):
        (sx, sy, syaw_deg), goal, timeout = SCENARIOS[scenario]
        stamp = self.get_clock().now().to_msg()
        # put the sim rover at the start and let the controller come to rest
        for _ in range(3):
            self.reset_pub.publish(make_pose(sx, sy, math.radians(syaw_deg), stamp))
            rclpy.spin_once(self, timeout_sec=0.1)
        t_end = time.monotonic() + 1.5
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.rec_pose, self.rec_cmd, self.rec_plan = [], [], []
        self.rec_opt, self.rec_opt_model = [], []
        self.recording = True
        start = self.now_s()
        handle, next_plan, reached, status = None, 0.0, False, 'timeout'
        while self.now_s() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.pose and math.hypot(self.pose[0] - goal[0], self.pose[1] - goal[1]) < GOAL_TOL:
                reached, status = True, 'reached'
                break
            if self.pose and self.in_collision(self.pose[0], self.pose[1]):
                status = 'collision'
                break
            if self.now_s() - start >= next_plan:
                next_plan += REPLAN_PERIOD
                g = ComputePathToPose.Goal()
                g.goal = make_pose(goal[0], goal[1], 0.0, self.get_clock().now().to_msg())
                g.planner_id = 'GridBasedCustom'
                g.use_start = False
                gh = self.wait(self.plan_ac.send_goal_async(g), 5.0)
                res = self.wait(gh.get_result_async(), 6.0) if gh and gh.accepted else None
                if res is None or res.status != GoalStatus.STATUS_SUCCEEDED or not res.result.path.poses:
                    continue
                path = res.result.path
                self.rec_plan.append((self.now_s(), np.array(
                    [[p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation)]
                     for p in path.poses])))
                fg = FollowPath.Goal()
                fg.path = path
                fg.controller_id = 'FollowPath'
                fg.goal_checker_id = 'goal_checker'
                fg.progress_checker_id = 'progress_checker'
                h = self.wait(self.follow_ac.send_goal_async(fg), 5.0)
                if h is not None and h.accepted:
                    handle = h  # a new goal preempts the previous one, like the BT
        self.recording = False
        self.cancel_follow(handle)
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.last_status = status
        return reached, status, start


def cross_track(P, plans):
    """Distance of each recorded pose from the plan in force at that time."""
    if not plans or not len(P):
        return np.zeros(0)
    pt = np.array([t for t, _ in plans])
    out = []
    for t, x, y, _ in P[::10]:
        j = np.searchsorted(pt, t) - 1
        if j < 0:
            continue
        pl = plans[j][1][:, :2]
        a, b = pl[:-1], pl[1:]
        ab = b - a
        L2 = np.maximum((ab ** 2).sum(1), 1e-12)
        u = np.clip(((np.array([x, y]) - a) * ab).sum(1) / L2, 0, 1)
        proj = a + u[:, None] * ab
        out.append(np.hypot(proj[:, 0] - x, proj[:, 1] - y).min() if len(pl) > 1 else 0.0)
    return np.array(out)


def metrics(rec_pose, rec_cmd, goal, start, reached, plans=None):
    P = np.array(rec_pose)
    C = np.array(rec_cmd) if rec_cmd else np.zeros((0, 3))
    m = {'reached': reached}
    if len(P):
        d = np.hypot(P[:, 1] - goal[0], P[:, 2] - goal[1])
        m['time'] = float(P[-1, 0] - start)
        m['dist_start'], m['dist_end'] = float(d[0]), float(d[-1])
        m['path_len'] = float(np.hypot(*np.diff(P[:, 1:3], axis=0).T).sum())
        m['net_heading_deg'] = float(np.degrees(np.unwrap(P[:, 3])[-1] - P[0, 3]))
    mv = C[np.abs(C[:, 1]) > 0.02, 1] if len(C) else np.zeros(0)
    m['gear_changes'] = int(np.count_nonzero(np.diff(np.sign(mv)))) if len(mv) > 1 else 0
    m['mean_abs_wz_cmd'] = float(np.abs(C[:, 2]).mean()) if len(C) else 0.0
    ct = cross_track(P, plans or [])
    m['cross_track_mean'] = float(ct.mean()) if len(ct) else float('nan')
    m['cross_track_p90'] = float(np.percentile(ct, 90)) if len(ct) else float('nan')
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--configs', nargs='+', default=['B0'])
    ap.add_argument('--scenarios', nargs='+', default=['bag14_t20'])
    ap.add_argument('--repeats', type=int, default=1)
    ap.add_argument('--extra', default='', help='JSON dict name -> overrides, merged into CONFIGS')
    ap.add_argument('--out', default='results')
    ap.add_argument('--no-reload', action='store_true', help='skip set/reload for the first config')
    args = ap.parse_args()
    if args.extra:
        CONFIGS.update(json.loads(Path(args.extra).read_text() if Path(args.extra).exists() else args.extra))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Driver()
    node.plan_ac.wait_for_server()
    node.follow_ac.wait_for_server()
    node.ensure_active()
    node.load_costmap()
    touched = sorted({k for c in args.configs for k in CONFIGS[c]})
    baseline = node.get_params(touched) if touched else {}
    print('launched values of touched params:', baseline, flush=True)
    rows = []
    try:
        for ci, cname in enumerate(args.configs):
            if not (ci == 0 and args.no_reload and not CONFIGS[cname]):
                node.apply_config(CONFIGS[cname], baseline)
            for scen in args.scenarios:
                for rep in range(args.repeats):
                    reached, status, start = node.run(scen, f'{cname}/{scen}/{rep}')
                    goal = SCENARIOS[scen][1]
                    m = metrics(node.rec_pose, node.rec_cmd, goal, start, reached, node.rec_plan)
                    m['status'] = status
                    rows.append((cname, scen, rep, m))
                    np.savez(out / f'{cname}__{scen}__{rep}.npz', pose=np.array(node.rec_pose),
                             cmd=np.array(node.rec_cmd), start=start, goal=goal,
                             plan_t=np.array([t for t, _ in node.rec_plan]),
                             plans=np.array([p for _, p in node.rec_plan], dtype=object),
                             opt_t=np.array([t for t, _ in node.rec_opt]),
                             opt=np.array([p for _, p in node.rec_opt], dtype=object),
                             opt_model_t=np.array([t for t, _ in node.rec_opt_model]),
                             opt_model=np.array([p for _, p in node.rec_opt_model], dtype=object),
                             metrics=json.dumps(m), config=json.dumps(CONFIGS[cname]))
                    print(f'{cname:10s} {scen:14s} #{rep} {status:8s} '
                          f't={m.get("time", 0):5.1f}s dist {m.get("dist_start", 0):.2f}->'
                          f'{m.get("dist_end", 0):.2f} gears={m["gear_changes"]:3d} '
                          f'dheading={m.get("net_heading_deg", 0):6.1f} |wz|={m["mean_abs_wz_cmd"]:.2f} '
                          f'off-plan mean {m["cross_track_mean"]:.2f} p90 {m["cross_track_p90"]:.2f} m',
                          flush=True)
    finally:
        if touched:
            try:
                node.apply_config({}, baseline)
            except Exception as e:  # noqa: BLE001
                print('restore failed:', e)
    (out / 'summary.json').write_text(json.dumps(
        [{'config': c, 'scenario': s, 'rep': r, **m} for c, s, r, m in rows], indent=1))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
