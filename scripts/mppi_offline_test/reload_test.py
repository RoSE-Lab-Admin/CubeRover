#!/usr/bin/env python3
"""Offline test of the controller-only model reload (lifecycle_manager_controller)
against the real nav2.launch.py + rover_sim.py on an isolated domain.

  1. wait until all six Nav2 nodes are active
  2. N pushes: set FollowPath.nn_model_path (alternating two .pt files), then
     RESET+STARTUP on /lifecycle_manager_controller/manage_nodes -- timed;
     check controller_server is active and the log shows the pushed file loaded
  3. check planner_server / bt_navigator were never reconfigured (log counts)
  4. drive one NavigateToPose goal through bt_navigator with the sim rover
  5. for reference, time one RESET+STARTUP of lifecycle_manager_navigation

    python3 reload_test.py --log LAUNCH_LOG --models A.pt B.pt [--n 20]
"""
import argparse
import math
import re
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node

NODES = ['map_server', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator',
         'waypoint_follower']


class T(Node):
    def __init__(self):
        super().__init__('reload_test')
        self.set_cli = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.ctrl_mgr = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_controller/manage_nodes')
        self.nav_mgr = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        self.reset_pub = self.create_publisher(PoseStamped, '/sim/reset_pose', 10)
        self.nav_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def wait(self, fut, timeout):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None

    def state(self, name):
        cli = self.create_client(GetState, f'/{name}/get_state')
        try:
            if not cli.wait_for_service(timeout_sec=5.0):
                return 'unavailable'
            r = self.wait(cli.call_async(GetState.Request()), 5.0)
            return r.current_state.label if r else 'no-reply'
        finally:
            self.destroy_client(cli)

    def manage(self, cli, cmd, timeout=240.0):
        cli.wait_for_service(timeout_sec=15.0)
        r = self.wait(cli.call_async(ManageLifecycleNodes.Request(command=cmd)), timeout)
        return r is not None and r.success

    def set_model(self, path):
        p = Parameter(name='FollowPath.nn_model_path',
                      value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=path))
        self.set_cli.wait_for_service(timeout_sec=10.0)
        r = self.wait(self.set_cli.call_async(SetParameters.Request(parameters=[p])), 10.0)
        return r is not None and r.results[0].successful


def pose(x, y, yaw, stamp):
    p = PoseStamped()
    p.header.frame_id = 'world'
    p.header.stamp = stamp
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def count(log, pattern):
    return len(re.findall(pattern, open(log, errors='ignore').read()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', required=True, help='launch output file (to verify loads)')
    ap.add_argument('--models', nargs=2, required=True)
    ap.add_argument('--n', type=int, default=20)
    a = ap.parse_args()
    rclpy.init()
    n = T()

    t0 = time.monotonic()
    while True:
        states = {k: n.state(k) for k in NODES}
        if all(v == 'active' for v in states.values()):
            break
        if time.monotonic() - t0 > 300:
            raise SystemExit(f'stack not active after 300 s: {states}')
        time.sleep(2.0)
    print(f'all six nodes active ({time.monotonic() - t0:.0f} s after test start)', flush=True)

    planner_cfg0 = count(a.log, r'\[planner_server\]: Configuring')
    bt_cfg0 = count(a.log, r'\[bt_navigator\]: Configuring')
    times, bad = [], 0
    for i in range(a.n):
        model = a.models[i % 2]
        loads0 = count(a.log, re.escape(f"loaded '{model}'"))
        ok_set = n.set_model(model)
        t = time.monotonic()
        ok = n.manage(n.ctrl_mgr, ManageLifecycleNodes.Request.RESET) and \
            n.manage(n.ctrl_mgr, ManageLifecycleNodes.Request.STARTUP)
        dt = time.monotonic() - t
        st = n.state('controller_server')
        time.sleep(0.5)  # let the log flush
        loaded = count(a.log, re.escape(f"loaded '{model}'")) > loads0
        good = ok_set and ok and st == 'active' and loaded
        bad += not good
        times.append(dt)
        print(f'push {i + 1:2d}: {dt:5.2f} s  manager ok={ok} controller={st} '
              f'loaded {model.split("/")[-1]}={loaded}', flush=True)
    others = {k: n.state(k) for k in NODES}
    planner_cfg = count(a.log, r'\[planner_server\]: Configuring') - planner_cfg0
    bt_cfg = count(a.log, r'\[bt_navigator\]: Configuring') - bt_cfg0
    print(f'\npushes: {a.n - bad}/{a.n} good; time min {min(times):.2f} s, median '
          f'{sorted(times)[len(times) // 2]:.2f} s, max {max(times):.2f} s')
    print(f'planner_server reconfigured {planner_cfg}x, bt_navigator {bt_cfg}x during pushes; '
          f'states now: {others}')

    # drive one goal through bt_navigator with the sim rover
    for _ in range(3):
        n.reset_pub.publish(pose(-1.0, -0.3, 0.0, n.get_clock().now().to_msg()))
        rclpy.spin_once(n, timeout_sec=0.1)
    time.sleep(1.5)
    n.nav_ac.wait_for_server(timeout_sec=20.0)
    g = NavigateToPose.Goal()
    g.pose = pose(0.6, 0.3, 0.0, n.get_clock().now().to_msg())
    t = time.monotonic()
    h = n.wait(n.nav_ac.send_goal_async(g), 10.0)
    res = n.wait(h.get_result_async(), 90.0) if h is not None and h.accepted else None
    ok = res is not None and res.status == GoalStatus.STATUS_SUCCEEDED
    print(f'NavigateToPose after the pushes: {"SUCCEEDED" if ok else "FAILED"} '
          f'in {time.monotonic() - t:.1f} s')

    # reference: one cycle of the navigation manager (planner, bt, ... -- no controller)
    t = time.monotonic()
    ok = n.manage(n.nav_mgr, ManageLifecycleNodes.Request.RESET) and \
        n.manage(n.nav_mgr, ManageLifecycleNodes.Request.STARTUP)
    print(f'reference: lifecycle_manager_navigation RESET+STARTUP (5 nodes) took '
          f'{time.monotonic() - t:.1f} s (ok={ok})')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
