#!/usr/bin/env python3
"""
Orchestrates N autonomous exploration trials end-to-end, replacing the manual
4-terminal workflow (repo README): for each trial, calls gp_explorer_gpu.py to
pick + plan a goal (retrying up to 3 times total if a Nav2 service isn't up
yet), then records a bag while path_follower drives to that goal, stopping
the bag once the goal is reached or progress has stalled.

Does NOT reimplement any of gp_explorer's GP/ALC logic, Nav2's path planning,
or path_follower's path-following -- it only sequences and manages the
lifecycle of those existing, unchanged scripts/launch files exactly as you'd
run them manually.

Run via autonomous_trials.launch.py (which also brings up Nav2 exactly once,
persistently, for the whole run) -- not intended to be run standalone, though
it works standalone too if Nav2 is already up elsewhere.
"""

import argparse
import csv
import math
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator

# Every node lifecycle_manager_navigation brings up (nav2_param2.yaml). No amcl
# in this stack (ground-truth pose comes from OptiTrack instead) -- deliberately
# NOT using BasicNavigator.waitUntilNav2Active(), whose default localizer='amcl'
# would hang forever waiting for a node that's never launched. Matches the
# narrower per-node wait path_follower.py already does internally for a subset
# of these (planner_server/controller_server/bt_navigator); this waits for all
# 6, once, before any trial starts.
NAV2_MANAGED_NODES = ["map_server", "planner_server", "controller_server",
                      "behavior_server", "bt_navigator", "waypoint_follower"]

# dynamics_retrain (and therefore torch) is imported lazily, inside
# maybe_retrain(), so that retrain_dynamics=false (the default) never requires
# torch to be importable at all -- only opting into retraining does.

XY_GOAL_TOLERANCE = 0.5     # matches nav2_param2.yaml's goal_checker xy_goal_tolerance
STALL_WINDOW_S = 30.0       # no-progress-for-this-long => treat the trial as stuck
MIN_IMPROVEMENT_M = 0.05    # smaller than this doesn't count as "progress" (noise floor)
GP_EXPLORER_MAX_ATTEMPTS = 3
BAG_START_DELAY_S = 2.0     # let `ros2 bag record` actually start before commands flow

BAG_TOPICS = [
    "/FitRosey_V1/pose", "/dynamic_joint_states", "/cmd_vel",
    "/roseybot_base_controller/cmd_vel_out", "/plan", "/optimal_trajectory",
]


def log(msg: str):
    print(f"[autonomous_trials] {msg}", flush=True)


class PoseWatcher(Node):
    """Keeps the latest real OptiTrack pose available for goal-reached / stall checks."""

    def __init__(self, opti_topic: str):
        super().__init__("autonomous_trials_pose_watcher")
        self.xy = None
        self.create_subscription(PoseStamped, opti_topic, self._cb, 10)

    def _cb(self, msg: PoseStamped):
        self.xy = (msg.pose.position.x, msg.pose.position.y)


def wait_for_nav2_active():
    """Blocks until every lifecycle_manager_navigation-managed node reports
    itself active. Nav2 bringup -- especially controller_server, which now
    loads a TorchScript model and may capture a CUDA graph for the deployed
    dynamics model -- can take real time; without this, the very first trial
    can race a stack that isn't actually ready yet and look like a stall."""
    log(f"waiting for Nav2 to fully activate ({', '.join(NAV2_MANAGED_NODES)}) "
        f"-- this can take a while, especially controller_server loading/capturing "
        f"the dynamics model")
    nav = BasicNavigator()
    t0 = time.monotonic()
    for node_name in NAV2_MANAGED_NODES:
        try:
            nav._waitForNodeToActivate(node_name)
        except Exception as e:
            log(f"WARNING: error waiting for {node_name} to activate ({e}) -- continuing anyway")
        log(f"  {node_name}: active ({time.monotonic() - t0:.1f}s elapsed)")
    nav.destroy_node()
    log(f"Nav2 fully active after {time.monotonic() - t0:.1f}s")


def wait_for_goal(node: PoseWatcher, goal_xy, label: str) -> str:
    """Spins until the real pose is within XY_GOAL_TOLERANCE of goal_xy, or no
    progress has been made for STALL_WINDOW_S. Returns 'reached' or 'stalled'."""
    best_dist = math.inf
    last_improve_t = time.monotonic()
    log(f"{label}: waiting for goal ({goal_xy[0]:.3f}, {goal_xy[1]:.3f})  "
        f"tolerance={XY_GOAL_TOLERANCE}m  stall_window={STALL_WINDOW_S}s")
    while True:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.xy is not None:
            dist = math.hypot(node.xy[0] - goal_xy[0], node.xy[1] - goal_xy[1])
            if dist < XY_GOAL_TOLERANCE:
                log(f"{label}: goal reached (dist={dist:.3f}m)")
                return "reached"
            if dist < best_dist - MIN_IMPROVEMENT_M:
                best_dist = dist
                last_improve_t = time.monotonic()
        if time.monotonic() - last_improve_t > STALL_WINDOW_S:
            log(f"{label}: STALLED (best_dist={best_dist:.3f}m, no improvement for "
                f"{STALL_WINDOW_S}s) -- skipping this trial")
            return "stalled"


def start_process(cmd) -> subprocess.Popen:
    log(f"starting: {' '.join(cmd)}")
    return subprocess.Popen(cmd)


def stop_process(proc: subprocess.Popen, name: str, timeout: float = 15.0):
    if proc is None or proc.poll() is not None:
        return
    log(f"stopping {name} (pid={proc.pid})")
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        log(f"{name} did not exit after SIGINT within {timeout}s, killing")
        proc.kill()
        proc.wait()


def read_goal_xy(pose_csv: Path):
    with open(pose_csv) as f:
        reader = csv.reader(f)
        next(reader)  # header: x,y,z
        row = next(reader)
        return float(row[0]), float(row[1])


def run_one_trial(bag_dir: Path, bag_name: str, pose_csv: Path, goal_xy, pose_node, label: str) -> str:
    """Returns 'reached' or 'stalled' (see wait_for_goal)."""
    bag_path = bag_dir / bag_name
    bag_proc = start_process(["ros2", "bag", "record", *BAG_TOPICS, "-o", str(bag_path)])
    time.sleep(BAG_START_DELAY_S)

    wp_proc = start_process([
        "ros2", "launch", "nav2_stack", "waypoint.launch.py",
        "include_nav2:=false", f"pose_csv:={pose_csv}",
    ])

    try:
        outcome = wait_for_goal(pose_node, goal_xy, label)
    finally:
        stop_process(wp_proc, "waypoint.launch.py")
        stop_process(bag_proc, "ros2 bag record")
    return outcome


def parse_bool(s: str) -> bool:
    return str(s).strip().lower() in ("true", "1", "yes")


def parse_nav2_param_yaml(text: str) -> dict:
    """Extracts just the fields dynamics_retrain needs from nav2_param2.yaml:
    dynamics_mode/nn_hidden_width (to know what's currently deployed) and the
    linear/fmean/fstd values (always needed for the shared normalizer, and as
    the current linear weights)."""
    def grab_scalar(key, default=None):
        m = re.search(rf"^\s*{re.escape(key)}:\s*\"?([\w.]+)\"?\s*$", text, re.MULTILINE)
        return m.group(1) if m else default

    def grab_array(key, n):
        m = re.search(rf"{re.escape(key)}:\s*\[([^\]]+)\]", text)
        if not m:
            return None
        vals = [float(v) for v in m.group(1).split(",")]
        assert len(vals) == n, f"{key}: expected {n} values, got {len(vals)}"
        return vals

    return {
        "dynamics_mode": grab_scalar("dynamics_mode", "kinematics"),
        "nn_hidden_width": int(grab_scalar("nn_hidden_width", "64")),
        "weight": grab_array("linear_model.weight", 4),
        "bias": grab_array("linear_model.bias", 2),
        "fmean": grab_array("linear_model.fmean", 2),
        "fstd": grab_array("linear_model.fstd", 2),
    }


def push_linear_params_and_reload(weight, bias):
    log(f"pushing retrained linear weights into the live controller: "
        f"weight={weight} bias={bias}")
    subprocess.run(["ros2", "param", "set", "/controller_server",
                    "FollowPath.linear_model.weight", str(weight)], check=True)
    subprocess.run(["ros2", "param", "set", "/controller_server",
                    "FollowPath.linear_model.bias", str(bias)], check=True)
    reload_controller()


def reload_controller():
    """Cycles controller_server's lifecycle so it reconstructs NNDynamics fresh
    (re-reads the .pt file / just-pushed linear params). See plan doc for why
    this is needed -- weights are otherwise only ever loaded once at startup."""
    log("cycling controller_server lifecycle to reload the dynamics model")
    subprocess.run(["ros2", "lifecycle", "set", "/controller_server", "configure"], check=True)
    subprocess.run(["ros2", "lifecycle", "set", "/controller_server", "activate"], check=True)


def maybe_retrain(retrain_cfg: dict, bag_dir: Path, new_bag_path: Path):
    if retrain_cfg is None:
        return
    from nav2_stack import dynamics_retrain  # deferred -- see import comment near top of file
    log(f"retraining {retrain_cfg['model_type']}"
        f"{retrain_cfg.get('width', '')} on data in {bag_dir} "
        f"(warm_start={retrain_cfg['warm_start']}, subset={retrain_cfg['subset']})")
    try:
        result = dynamics_retrain.retrain(
            bag_dir=bag_dir, new_bag_path=new_bag_path,
            model_type=retrain_cfg["model_type"], width=retrain_cfg.get("width"),
            warm_start=retrain_cfg["warm_start"], subset=retrain_cfg["subset"],
            subset_fraction=retrain_cfg["subset_fraction"],
            fmean=retrain_cfg["fmean"], fstd=retrain_cfg["fstd"])
    except Exception as e:
        log(f"WARNING: retrain failed ({e}) -- keeping the currently deployed weights")
        return

    if retrain_cfg["model_type"] == "linear":
        push_linear_params_and_reload(result["weight"], result["bias"])
    else:
        log(f"exported retrained mlp to {result['exported_path']}")
        reload_controller()


def run_gp_explorer(gp_explorer_path: Path, bag_dir: Path, name: str) -> bool:
    for attempt in range(1, GP_EXPLORER_MAX_ATTEMPTS + 1):
        log(f"gp_explorer_gpu.py attempt {attempt}/{GP_EXPLORER_MAX_ATTEMPTS} "
            f"(name={name})")
        result = subprocess.run(
            [sys.executable, str(gp_explorer_path), "--bag-dir", str(bag_dir), "--name", name])
        if result.returncode == 0:
            return True
        log(f"attempt {attempt} failed (exit code {result.returncode})")
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-dir", required=True, type=Path,
                        help="Directory bags are stored in and loaded from")
    parser.add_argument("--n-trajectories", default=25, type=int,
                        help="Number of trials to run (default: 25)")
    parser.add_argument("--opti-topic", default="/FitRosey_V1/pose",
                        help="Ground-truth pose topic (default: /FitRosey_V1/pose)")
    parser.add_argument("--retrain-dynamics", default="false", type=parse_bool,
                        help="Retrain the deployed dynamics model after every trial "
                             "(default: false)")
    parser.add_argument("--warm-start", default="true", type=parse_bool,
                        help="Warm-start retraining from the currently deployed weights "
                             "(MLP only -- no-op for linear). Default: true")
    parser.add_argument("--retrain-subset", default="false", type=parse_bool,
                        help="Train on a random subset of prior bags + the new trajectory, "
                             "instead of every bag collected so far. Default: false")
    parser.add_argument("--retrain-subset-fraction", default=0.3, type=float,
                        help="Fraction of prior bags to sample when --retrain-subset is "
                             "true. Default: 0.3")
    # launch_ros.actions.Node always appends --ros-args (and would append any
    # remappings/params too) to the process's argv, since it assumes the
    # executable parses those via rclpy's standard handling. Strip them before
    # plain argparse sees argv, or it chokes on --ros-args as unrecognized.
    import rclpy.utilities
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    bag_dir = args.bag_dir
    bag_dir.mkdir(parents=True, exist_ok=True)

    retrain_cfg = None
    if args.retrain_dynamics:
        yaml_path = Path(get_package_share_directory("nav2_stack")) / "config" / "nav2_param2.yaml"
        parsed = parse_nav2_param_yaml(yaml_path.read_text())
        if parsed["dynamics_mode"] == "kinematics":
            log("retrain_dynamics=true but dynamics_mode=kinematics in nav2_param2.yaml "
                "-- nothing to retrain, disabling")
        else:
            model_type = "linear" if parsed["dynamics_mode"] == "linear" else "mlp"
            retrain_cfg = {
                "model_type": model_type,
                "width": parsed["nn_hidden_width"] if model_type == "mlp" else None,
                "warm_start": args.warm_start,
                "subset": args.retrain_subset,
                "subset_fraction": args.retrain_subset_fraction,
                "fmean": np.array(parsed["fmean"], dtype=np.float32),
                "fstd": np.array(parsed["fstd"], dtype=np.float32),
            }
            log(f"online retraining enabled: model={model_type}"
                f"{retrain_cfg['width'] or ''}  warm_start={args.warm_start}  "
                f"subset={args.retrain_subset} (fraction={args.retrain_subset_fraction})")

    # Same package directory as gp_explorer_gpu.py (source or installed -- this
    # script and gp_explorer_gpu.py are always siblings either way), so this is
    # robust regardless of how autonomous_trials.py itself was invoked.
    gp_explorer_path = Path(__file__).resolve().parent / "gp_explorer_gpu.py"
    if not gp_explorer_path.exists():
        sys.exit(f"FATAL: expected to find gp_explorer_gpu.py next to this script "
                  f"at {gp_explorer_path}, but it's not there")
    # gp_explorer_gpu.py writes pose.csv to Path(__file__).resolve().parent.parent /
    # 'pose.csv' -- computed here the identical way (same package, same __file__
    # resolution) so we pass waypoint.launch.py the EXACT path it was written to,
    # rather than relying on waypoint.launch.py's own default (which resolves via
    # the installed share directory and may not match where gp_explorer_gpu.py
    # actually wrote it, depending on whether it's run from source or installed).
    pose_csv = gp_explorer_path.resolve().parent.parent / "pose.csv"

    rclpy.init()
    wait_for_nav2_active()
    pose_node = PoseWatcher(args.opti_topic)

    try:
        initial_bag = bag_dir / "initial_bag"
        if not initial_bag.exists():
            log(f"no initial_bag found in {bag_dir} -- collecting one at the origin")
            origin_csv = bag_dir / "initial_goal.csv"
            with open(origin_csv, "w") as f:
                f.write("x,y,z\n0.0,0.0,0.0\n")
            outcome = run_one_trial(bag_dir, "initial_bag", origin_csv, (0.0, 0.0), pose_node,
                                    "initial_bag")
            if outcome == "reached":
                log("initial_bag: waypoint reached")
            else:
                log("initial_bag: FAILED to reach waypoint (stalled)")
            maybe_retrain(retrain_cfg, bag_dir, initial_bag)
        else:
            log(f"found existing {initial_bag}, skipping bootstrap")

        n_success = 0
        for i in range(1, args.n_trajectories + 1):
            traj_name = f"traj_{i:02d}"
            bag_name = f"bag_{i:02d}"
            log(f"=== trial {i}/{args.n_trajectories} ({traj_name}) ===")

            if not run_gp_explorer(gp_explorer_path, bag_dir, traj_name):
                sys.exit(f"FATAL: gp_explorer_gpu.py failed {GP_EXPLORER_MAX_ATTEMPTS} times "
                         f"for {traj_name} -- aborting entire run")

            goal_xy = read_goal_xy(pose_csv)
            outcome = run_one_trial(bag_dir, bag_name, pose_csv, goal_xy, pose_node, traj_name)
            if outcome == "reached":
                n_success += 1
                log(f"trial {i}/{args.n_trajectories} ({traj_name}): waypoint reached")
            else:
                log(f"trial {i}/{args.n_trajectories} ({traj_name}): FAILED to reach "
                    f"waypoint (stalled) -- planning a new trajectory")
            maybe_retrain(retrain_cfg, bag_dir, bag_dir / bag_name)

        log(f"all {args.n_trajectories} trials complete: "
            f"{n_success}/{args.n_trajectories} successful")
    finally:
        pose_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
