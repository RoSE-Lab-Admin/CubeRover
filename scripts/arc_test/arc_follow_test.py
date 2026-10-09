#!/usr/bin/env python3
"""Automated arc-following test: drives the rover along single planned arcs of
given radii, one bag per arc, optionally for several MPPI horizons.

Nav2 must already be running (ros2 launch nav2_stack nav2.launch.py). Per arc
this does what autonomous_trials does per trial: start `ros2 bag record`, start
`waypoint.launch.py include_nav2:=false` with a one-row goal CSV, wait for the
outcome, stop both. The goal is the end of an arc of radius R that starts along
the rover's current heading, so the planner's arc mode plans exactly that arc.

  - left/right alternate; sweep 90 deg if it keeps >= --clearance m from the
    walls, else 75/60/105 deg; if no arc fits, an unrecorded Nav2 move to the
    arena centre first
  - --dynamics nn kinematics x --time-steps 25 30 40: one block per combination;
    each sets FollowPath.dynamics_mode / time_steps and reloads only
    controller_server (lifecycle_manager_controller). "nn" = the shared default
    model. The launched dynamics_mode, model path and time_steps are restored
    at the end
  - /rosout is bagged too, so the planner's "arc plan"/"arc rejected" lines are
    in each bag; runs.csv in --out lists every run

    python3 scripts/arc_test/arc_follow_test.py --out ~/AL_scripts/arc_tests_MMDD \
        --radii 1.3 1.5 1.7 --repeats 2 --dynamics nn kinematics --time-steps 25 30 40
"""
import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from PIL import Image
from rcl_interfaces.srv import GetParameters
from scipy.ndimage import distance_transform_edt

from nav2_stack import autonomous_trials as at

# repositioning targets, tried in turn when no arc fits (middle of the free
# arena first); the arrival heading is not controllable, so trying a few
# spots is what makes long arcs (e.g. 1.7 m x 90 deg) fit often enough
REPOSITION = [(-0.3, -0.2), (-0.9, 0.4), (0.4, -0.7), (-0.6, -1.0), (0.3, 0.5)]
MAX_REPOSITIONS = 4
SWEEPS = (90, 75, 60, 105)     # default preference order (--sweeps); planner arc mode allows <= 120
RADIUS_MARGIN = 0.02           # target radius must exceed arc_min_radius by this


def ctrl_default_model(width):
    return str(Path(get_package_share_directory("nav2_mppi_controller")) / "models" / "ar_mlp"
               / f"mlp{width}_ar_velocity_teacher.pt")


class Map:
    def __init__(self):
        share = Path(get_package_share_directory("nav2_stack")) / "maps"
        img = np.array(Image.open(share / "map.pgm"))
        self.img, self.res, self.ox, self.oy = img, 0.2, -5.0, -5.0   # map.yaml
        self.dist = distance_transform_edt(img >= 128) * self.res

    def clearance(self, x, y):
        c, r = int((x - self.ox) / self.res), self.img.shape[0] - 1 - int((y - self.oy) / self.res)
        if not (0 <= r < self.img.shape[0] and 0 <= c < self.img.shape[1]):
            return 0.0
        return float(self.dist[r, c])


def arc_points(pose, R, sign, sweep_deg, n=60):
    x0, y0, yaw = pose
    f = np.array([math.cos(yaw), math.sin(yaw)])
    nrm = sign * np.array([-f[1], f[0]])
    a = np.linspace(0.0, math.radians(sweep_deg), n)
    return np.array([x0, y0]) + R * (np.sin(a)[:, None] * f + (1 - np.cos(a))[:, None] * nrm)


def pick_arc(pose, R, side_order, amap, min_clear, sweeps=SWEEPS):
    """First (side, sweep) in preference order whose arc centreline stays
    >= min_clear from the walls. Returns (side, sweep, goal_xy, clearance) or None."""
    for sweep in sweeps:
        for side in side_order:
            pts = arc_points(pose, R, 1 if side == "left" else -1, sweep)
            cl = min(amap.clearance(*p) for p in pts)
            if cl >= min_clear:
                return side, sweep, tuple(pts[-1]), cl
    return None


def get_param(node, client, name):
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f"{client.srv_name} not available")
    fut = client.call_async(GetParameters.Request(names=[name]))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    v = fut.result().values[0]
    return {1: v.bool_value, 2: v.integer_value, 3: v.double_value, 4: v.string_value}.get(v.type)


def current_pose(node, timeout=10.0):
    t0 = time.monotonic()
    node.xy = None
    while node.pose() is None and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.pose() is None:
        raise RuntimeError("no /FitRosey_V1/pose")
    return node.pose()


def write_goal(path, xy):
    path.write_text(f"x,y,z\n{xy[0]:.4f},{xy[1]:.4f},0.0\n")


def drive(node, out, name, goal_xy, record):
    """One waypoint run; bagged into out/name if record. Returns the outcome."""
    csv_path = out / f"{name}_goal.csv"
    write_goal(csv_path, goal_xy)
    # drain a late /trial_goal_result from the previous run's path_follower
    # (it can arrive after its launch was stopped); otherwise wait_for_goal
    # takes it as this run's "succeeded" and the run ends after ~2 s
    t_drain = time.monotonic() + 1.5
    while time.monotonic() < t_drain:
        rclpy.spin_once(node, timeout_sec=0.1)
    bag = None
    if record:
        bag = at.start_process(["ros2", "bag", "record", *at.BAG_TOPICS, "/rosout",
                                "-o", str(out / name)])
        time.sleep(at.BAG_START_DELAY_S)
    wp = at.start_process(["ros2", "launch", "nav2_stack", "waypoint.launch.py",
                           "include_nav2:=false", f"pose_csv:={csv_path}"])
    try:
        return at.wait_for_goal(node, goal_xy, name)
    finally:
        at.stop_process(wp, "waypoint.launch.py")
        if bag is not None:
            at.stop_process(bag, "ros2 bag record")


DYNAMICS = {"nn": "neural_network", "kinematics": "kinematics"}


def configure_controller(node, mode, steps, model):
    at.log(f"controller: dynamics_mode={mode}, time_steps={steps}"
           + (f", model={model}" if mode == "neural_network" else ""))
    at.set_controller_param(node, "FollowPath.time_steps", int(steps))
    at.set_controller_param(node, "FollowPath.nn_model_path", model)
    at.set_controller_param(node, "FollowPath.dynamics_mode", mode)
    at.reload_controller(node)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="directory for bags + runs.csv")
    ap.add_argument("--radii", nargs="+", type=float, default=[1.3, 1.5])
    ap.add_argument("--repeats", type=int, default=2, help="arcs per radius per horizon")
    ap.add_argument("--time-steps", nargs="*", type=int, default=[],
                    help="MPPI horizons to test in blocks; empty = the launched value")
    ap.add_argument("--dynamics", nargs="+", choices=sorted(DYNAMICS), default=["nn"],
                    help="dynamics models to test in blocks: nn (shared default model), kinematics")
    ap.add_argument("--sweeps", nargs="+", type=float, default=list(SWEEPS),
                    help="arc sweeps (deg) in order of preference; the first that fits is used. "
                         "Keep <= ~110: the planner rejects > 120 and the needed sweep grows "
                         "when the rover lags")
    ap.add_argument("--goal-tolerance", type=float, default=None,
                    help="xy goal tolerance (m) for the test, e.g. 0.3; sets the controller's "
                         "goal_checker.xy_goal_tolerance and restores the launched value at the end")
    ap.add_argument("--clearance", type=float, default=0.9,
                    help="min wall clearance of the arc centreline (m)")
    a = ap.parse_args()
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    amap = Map()

    rclpy.init()
    node = at.PoseWatcher("/FitRosey_V1/pose")
    ctrl_get = node.create_client(GetParameters, "/controller_server/get_parameters")
    plan_get = node.create_client(GetParameters, "/planner_server/get_parameters")
    at.wait_for_nav2_active()

    width = get_param(node, ctrl_get, "FollowPath.nn_hidden_width") or 128
    launched_steps = get_param(node, ctrl_get, "FollowPath.time_steps")
    launched_mode = get_param(node, ctrl_get, "FollowPath.dynamics_mode")
    launched_model = get_param(node, ctrl_get, "FollowPath.nn_model_path")
    launched_tol = get_param(node, ctrl_get, "goal_checker.xy_goal_tolerance")
    if a.goal_tolerance is not None:
        at.log(f"goal tolerance {launched_tol} -> {a.goal_tolerance} m for the test")
        at.set_controller_param(node, "goal_checker.xy_goal_tolerance", float(a.goal_tolerance))
        at.XY_GOAL_TOLERANCE = a.goal_tolerance   # wait_for_goal's own distance check
    arc_min = get_param(node, plan_get, "GridBasedCustom.arc_min_radius")
    if arc_min is None or arc_min < 0:
        arc_min = get_param(node, plan_get, "GridBasedCustom.minimum_turning_radius")
    at.log(f"planner arc_min_radius={arc_min}, controller time_steps={launched_steps}")
    radii = [r for r in a.radii if r >= arc_min + RADIUS_MARGIN]
    for r in sorted(set(a.radii) - set(radii)):
        at.log(f"SKIPPING radius {r}: below the planner's arc_min_radius {arc_min} "
               f"(+{RADIUS_MARGIN}), it would plan Hybrid-A* instead of the arc")
    if not radii:
        raise SystemExit("no testable radius")

    model = ctrl_default_model(width)
    blocks = [(d, ts) for d in a.dynamics for ts in (a.time_steps or [launched_steps])]
    at.log(f"{len(blocks)} blocks x {len(radii)} radii x {a.repeats} repeats = "
           f"{len(blocks) * len(radii) * a.repeats} arcs: {blocks}")
    runs_csv = out / "runs.csv"
    new_file = not runs_csv.exists()
    fcsv = open(runs_csv, "a", newline="")
    w = csv.writer(fcsv)
    if new_file:
        w.writerow(["run", "time_steps", "radius", "side", "sweep_deg", "start_x", "start_y",
                    "start_yaw_deg", "goal_x", "goal_y", "clearance", "outcome", "t_start", "t_end", "dynamics", "goal_tol"])
    side_flip = 0
    try:
        for dyn, steps in blocks:
            configure_controller(node, DYNAMICS[dyn], steps, model)
            for rep in range(a.repeats):
                for R in radii:
                    order = ["left", "right"] if side_flip % 2 == 0 else ["right", "left"]
                    side_flip += 1
                    pose = current_pose(node)
                    choice = pick_arc(pose, R, order, amap, a.clearance, a.sweeps)
                    for k in range(MAX_REPOSITIONS):
                        if choice is not None:
                            break
                        # skip a target the rover is already at (it would end at once)
                        tgt = next(p for p in REPOSITION[k:] + REPOSITION
                                   if math.hypot(p[0] - pose[0], p[1] - pose[1]) > 0.8)
                        at.log(f"no {R} m arc fits from ({pose[0]:.2f}, {pose[1]:.2f}, "
                               f"{math.degrees(pose[2]):.0f} deg) -- repositioning to {tgt} "
                               f"(not recorded)")
                        drive(node, out, "reposition", tgt, record=False)
                        pose = current_pose(node)
                        choice = pick_arc(pose, R, order, amap, a.clearance, a.sweeps)
                    if choice is None:
                        at.log(f"still no {R} m arc fits -- skipping this run")
                        continue
                    side, sweep, goal, cl = choice
                    name = f"{datetime.now():%H%M%S}_{dyn}_ts{steps}_R{R:.2f}_{side}{sweep:g}"
                    at.log(f"=== {name}: from ({pose[0]:.2f}, {pose[1]:.2f}, "
                           f"{math.degrees(pose[2]):.0f} deg) to ({goal[0]:.2f}, {goal[1]:.2f}), "
                           f"clearance {cl:.2f} m")
                    t0 = time.time()
                    outcome = drive(node, out, name, goal, record=True)
                    w.writerow([name, steps, R, side, sweep, f"{pose[0]:.4f}", f"{pose[1]:.4f}",
                                f"{math.degrees(pose[2]):.1f}", f"{goal[0]:.4f}", f"{goal[1]:.4f}",
                                f"{cl:.2f}", outcome, f"{t0:.3f}", f"{time.time():.3f}", dyn,
                                a.goal_tolerance if a.goal_tolerance is not None else launched_tol])
                    fcsv.flush()
                    at.log(f"=== {name}: {outcome}")
                    if outcome == "safety_stop":
                        raise SystemExit("safety_watchdog stop -- aborting")
    finally:
        fcsv.close()
        if a.goal_tolerance is not None and launched_tol is not None:
            at.log(f"restoring goal tolerance {launched_tol} m")
            try:
                at.set_controller_param(node, "goal_checker.xy_goal_tolerance", float(launched_tol))
            except Exception as e:
                at.log(f"WARNING: could not restore the goal tolerance ({e})")
        if None not in (launched_mode, launched_steps, launched_model):
            at.log(f"restoring the launched controller settings")
            try:
                configure_controller(node, launched_mode, launched_steps, launched_model)
            except Exception as e:
                at.log(f"WARNING: could not restore the controller settings ({e})")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
