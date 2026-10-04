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
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
from nav2_msgs.srv import ManageLifecycleNodes
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
STALL_WINDOW_S = 60.0       # no-progress-for-this-long => treat the trial as stuck
MIN_IMPROVEMENT_M = 0.05    # smaller than this doesn't count as "progress" (noise floor)
GP_EXPLORER_MAX_ATTEMPTS = 3
NO_MOVEMENT_MAX_ATTEMPTS = 3  # retries for a trial that recorded a "no_movement" bag (see wait_for_goal)
BAG_START_DELAY_S = 2.0     # let `ros2 bag record` actually start before commands flow

BAG_TOPICS = [
    "/FitRosey_V1/pose", "/dynamic_joint_states", "/cmd_vel",
    "/roseybot_base_controller/cmd_vel_out", "/plan", "/optimal_trajectory",
    "/optimal_trajectory_model",
]


def log(msg: str):
    print(f"[autonomous_trials] {msg}", flush=True)


class PoseWatcher(Node):
    """Keeps the latest real OptiTrack pose available for goal-reached / stall
    checks, watches for a safety_watchdog-triggered emergency stop and
    path_follower's authoritative trajectory outcome, and owns persistent
    service clients for pushing live params / cycling the Nav2 lifecycle.

    The clients are owned here (one long-lived node for the whole run)
    rather than shelling out to `ros2 param set` / `ros2 service call` per
    call -- each of those CLI invocations spins up a brand-new DDS
    participant and pays fresh discovery from scratch, which was observed to
    occasionally hang indefinitely with zero diagnostic output right after a
    burst of node churn (bag recorder / waypoint launch / gp_explorer
    processes joining and leaving the ROS graph in quick succession). Reusing
    one already-discovered participant, with an explicit timeout on every
    call, turns that into a bounded, diagnosable failure instead."""

    def __init__(self, opti_topic: str):
        super().__init__("autonomous_trials_pose_watcher")
        self.xy = None
        self.safety_stop = False
        self.goal_result = None  # None | "succeeded" | "failed"
        self.create_subscription(PoseStamped, opti_topic, self._cb, 10)
        self.create_subscription(Bool, "/safety_stop", self._safety_stop_cb, 10)
        self.create_subscription(String, "/trial_goal_result", self._goal_result_cb, 10)
        self.set_params_client = self.create_client(
            SetParameters, "/controller_server/set_parameters")
        self.manage_nodes_client = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes")

    def _cb(self, msg: PoseStamped):
        self.xy = (msg.pose.position.x, msg.pose.position.y)

    def _safety_stop_cb(self, msg: Bool):
        if msg.data:
            self.safety_stop = True

    def _goal_result_cb(self, msg: String):
        self.goal_result = msg.data

    def reset_trial_state(self):
        self.goal_result = None


def wait_for_node_active(nav: BasicNavigator, node_name: str, timeout_sec: float = 5.0,
                          max_attempts: int = 60) -> bool:
    """Bounded-timeout, bounded-retry replacement for BasicNavigator's private
    _waitForNodeToActivate(), which loops forever with NO timeout at all
    (rclpy.spin_until_future_complete with no timeout_sec, inside a
    while-not-active loop with no exit condition) -- if a node's get_state
    call ever fails to resolve cleanly (observed live: an RMW
    response-delivery timeout on planner_server's side), the unbounded
    version gets stuck forever with no way to recover. Same fix as
    path_follower.py's _wait_for_node_active(), duplicated here since this
    runs in a separate process with no shared module. Returns True once
    node_name reports 'active', False if it never does within max_attempts."""
    from lifecycle_msgs.srv import GetState
    for _ in range(max_attempts):
        client = nav.create_client(GetState, f'{node_name}/get_state')
        try:
            if client.wait_for_service(timeout_sec=timeout_sec):
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(nav, future, timeout_sec=timeout_sec)
                result = future.result()
                if result is not None and result.current_state.label == 'active':
                    return True
        finally:
            nav.destroy_client(client)
    return False


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
        if not wait_for_node_active(nav, node_name):
            log(f"WARNING: {node_name} did not report active in time -- continuing anyway")
        log(f"  {node_name}: active ({time.monotonic() - t0:.1f}s elapsed)")
    nav.destroy_node()
    log(f"Nav2 fully active after {time.monotonic() - t0:.1f}s")


def wait_for_goal(node: PoseWatcher, goal_xy, label: str) -> str:
    """Spins until path_follower reports its authoritative trajectory
    outcome, the real pose is within XY_GOAL_TOLERANCE of goal_xy (a
    secondary/fallback "reached" signal -- see below), no progress has been
    made for STALL_WINDOW_S, or safety_watchdog fires.
    Returns 'reached', 'stalled', 'no_movement', or 'safety_stop'.

    'no_movement' is a stricter subset of 'stalled': not just no improvement
    for the last STALL_WINDOW_S, but no meaningful progress at all across the
    *entire* trial (distance-to-goal never dropped more than
    MIN_IMPROVEMENT_M below its first recorded value). Distinguishes a
    genuine "tried and got stuck partway" stall from "the rover never
    actually moved" (observed live: path_follower's own nav2-readiness wait
    hung -- see path_follower.py's _wait_for_node_active -- so it never even
    issued a goal, and the whole 60s was spent doing nothing). The caller
    uses this to discard the bag and retry the same trial instead of treating
    it as a normal failed-but-attempted trajectory.

    Trusts path_follower's own /trial_goal_result over the distance poll:
    Nav2's internal goToPose result and this function's independent
    re-derivation of "reached" from a coarser, staler pose-topic poll can
    disagree right at the tolerance boundary (observed live: Nav2 reported
    TaskResult.SUCCEEDED while this poll still saw 0.542m > 0.5m and the
    trial was wrongly marked STALLED after burning the full stall window).
    The distance check is kept as a fallback "reached" trigger too -- it can
    only ever fire *correctly* (same 0.5m tolerance Nav2's own goal checker
    uses), so keeping it can't reintroduce that bug, only catch a reached
    goal if /trial_goal_result were ever dropped."""
    node.reset_trial_state()
    best_dist = math.inf
    first_dist = None
    last_improve_t = time.monotonic()
    log(f"{label}: waiting for goal ({goal_xy[0]:.3f}, {goal_xy[1]:.3f})  "
        f"tolerance={XY_GOAL_TOLERANCE}m  stall_window={STALL_WINDOW_S}s")

    def _made_progress() -> bool:
        return first_dist is not None and best_dist < first_dist - MIN_IMPROVEMENT_M

    while True:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.safety_stop:
            log(f"{label}: SAFETY STOP triggered by safety_watchdog -- aborting")
            return "safety_stop"
        if node.goal_result == "succeeded":
            log(f"{label}: goal reached (path_follower reported TaskResult.SUCCEEDED)")
            return "reached"
        if node.goal_result == "failed":
            log(f"{label}: path_follower reported the trajectory FAILED")
            return "stalled" if _made_progress() else "no_movement"
        if node.xy is not None:
            dist = math.hypot(node.xy[0] - goal_xy[0], node.xy[1] - goal_xy[1])
            if first_dist is None:
                first_dist = dist
                best_dist = dist
            if dist < XY_GOAL_TOLERANCE:
                log(f"{label}: goal reached (dist={dist:.3f}m)")
                return "reached"
            if dist < best_dist - MIN_IMPROVEMENT_M:
                best_dist = dist
                last_improve_t = time.monotonic()
        if time.monotonic() - last_improve_t > STALL_WINDOW_S:
            made_progress = _made_progress()
            log(f"{label}: STALLED (best_dist={best_dist:.3f}m, no improvement for "
                f"{STALL_WINDOW_S}s) -- "
                f"{'skipping this trial' if made_progress else 'NO MOVEMENT AT ALL, will retry'}")
            return "stalled" if made_progress else "no_movement"


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


def next_trial_start_index(bag_dir: Path) -> int:
    """Scans bag_dir for existing bag_NN directories (from a previous,
    interrupted run) and returns the next unused trial index, so re-running
    against the same bag_dir resumes/continues numbering instead of
    restarting at bag_01 and overwriting what's already there. initial_bag
    doesn't count (it's bootstrap-only, not numbered). Returns 1 if none
    exist."""
    existing = []
    for p in bag_dir.glob("bag_*"):
        m = re.fullmatch(r"bag_(\d+)", p.name)
        if m:
            existing.append(int(m.group(1)))
    return max(existing, default=0) + 1


def run_one_trial(bag_dir: Path, bag_name: str, pose_csv: Path, goal_xy, pose_node, label: str) -> str:
    """Returns 'reached', 'stalled', 'no_movement', or 'safety_stop' (see wait_for_goal)."""
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


def with_retries(fn, attempts=3, backoff_s=5.0, label=""):
    last_exc = None
    for attempt in range(1, attempts + 1):
        try:
            fn()
            return
        except Exception as e:
            last_exc = e
            log(f"WARNING: {label} attempt {attempt}/{attempts} failed ({e})")
            if attempt < attempts:
                time.sleep(backoff_s)
    raise RuntimeError(f"{label} failed after {attempts} attempts") from last_exc


def _param_value(value) -> ParameterValue:
    if isinstance(value, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    if isinstance(value, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
    if isinstance(value, float):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
    if isinstance(value, str):
        return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
    if isinstance(value, (list, tuple)):
        return ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            double_array_value=[float(v) for v in value])
    raise TypeError(f"unsupported parameter value type: {type(value)}")


def set_controller_param(node: PoseWatcher, name: str, value, timeout_sec=15.0,
                          service_wait_sec=10.0):
    """Sets a /controller_server parameter via a persistent rclpy service
    client (see PoseWatcher docstring for why not `ros2 param set`), with an
    explicit timeout and a bounded retry instead of hanging indefinitely."""
    def _do():
        if not node.set_params_client.wait_for_service(timeout_sec=service_wait_sec):
            raise RuntimeError(f"/controller_server/set_parameters not available "
                                f"after {service_wait_sec}s")
        req = SetParameters.Request(parameters=[ParameterMsg(name=name, value=_param_value(value))])
        future = node.set_params_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done():
            raise RuntimeError(f"timed out after {timeout_sec}s waiting for response")
        result = future.result()
        if result is None:
            raise RuntimeError(f"service call raised {future.exception()}")
        if not result.results[0].successful:
            raise RuntimeError(f"rejected ({result.results[0].reason})")
    with_retries(_do, label=f"set parameter {name}")


def call_manage_nodes(node: PoseWatcher, command: int, label: str, timeout_sec=180.0,
                       service_wait_sec=15.0):
    """Calls lifecycle_manager_navigation's ManageLifecycleNodes service via a
    persistent rclpy client (see PoseWatcher docstring), with an explicit
    timeout and a bounded retry. timeout_sec is generous (a full stack
    RESET+STARTUP cycle has been observed to legitimately take ~60s, more if
    controller_server needs to recapture its CUDA graph) but still bounded,
    so a genuine hang is caught and retried instead of blocking forever."""
    def _do():
        if not node.manage_nodes_client.wait_for_service(timeout_sec=service_wait_sec):
            raise RuntimeError(f"/lifecycle_manager_navigation/manage_nodes not available "
                                f"after {service_wait_sec}s")
        req = ManageLifecycleNodes.Request(command=command)
        future = node.manage_nodes_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done():
            raise RuntimeError(f"timed out after {timeout_sec}s waiting for response")
        result = future.result()
        if result is None:
            raise RuntimeError(f"service call raised {future.exception()}")
        if not result.success:
            raise RuntimeError("returned success=False")
    with_retries(_do, label=label)


def push_linear_params_and_reload(node: PoseWatcher, weight, bias):
    log(f"pushing retrained linear weights into the live controller: "
        f"weight={weight} bias={bias}")
    set_controller_param(node, "FollowPath.linear_model.weight", list(weight))
    set_controller_param(node, "FollowPath.linear_model.bias", list(bias))
    reload_controller(node)


def reload_controller(node: PoseWatcher):
    """Cycles the WHOLE Nav2 stack via lifecycle_manager_navigation's own
    ManageLifecycleNodes service (RESET=3 then STARTUP=0), so it reconstructs
    NNDynamics fresh (re-reads the .pt file / just-pushed linear/dynamics_mode
    params) -- weights are otherwise only ever loaded once at startup.

    Does NOT use direct per-node `ros2 lifecycle set` calls on controller_server
    -- confirmed by trial and error that lifecycle_manager_navigation reacts to
    ANY externally-driven state change on a node it manages (not just a bond
    heartbeat timeout) and starts its own concurrent recovery, racing whatever
    sequence we're mid-way through (observed as "transition not registered"
    errors and the whole process crashing while Nav2 silently self-healed with
    nothing left running to notice). Going through the manager's own official
    control service instead means the manager is the one making the state
    changes, so it has no "unexpected" state to react to. This does mean all 6
    managed nodes get reconfigured, not just controller_server -- harmless
    (the other 5 just re-read their own unchanged params) but slower, and can
    take a while if the model needs CUDA graph capture on activate."""
    log("cycling the Nav2 stack via lifecycle_manager_navigation to reload the dynamics "
        "model (RESET then STARTUP -- can take a while)")
    call_manage_nodes(node, 3, "RESET")
    call_manage_nodes(node, 0, "STARTUP")


def record_outcome(bag_dir: Path, bag_name: str, outcome: str, goal_xy):
    """Append this trial's outcome to bag_dir/trial_outcomes.csv -- read by
    dynamics_retrain to always train on failed trials and (with
    failure_weighting) weight them up."""
    path = bag_dir / "trial_outcomes.csv"
    new = not path.exists()
    with open(path, "a") as f:
        if new:
            f.write("bag,outcome,goal_x,goal_y\n")
        f.write(f"{bag_name},{outcome},{goal_xy[0]:.4f},{goal_xy[1]:.4f}\n")


def maybe_retrain(node: PoseWatcher, retrain_cfg: dict, bag_dir: Path, new_bag_path: Path):
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
            fmean=retrain_cfg["fmean"], fstd=retrain_cfg["fstd"],
            failure_weighting=retrain_cfg["failure_weighting"])
    except Exception as e:
        log(f"WARNING: retrain failed ({e}) -- keeping the currently deployed weights")
        return

    if retrain_cfg["model_type"] == "linear":
        push_linear_params_and_reload(node, result["weight"], result["bias"])
    else:
        log(f"exported retrained mlp to {result['exported_path']}")
        reload_controller(node)


def push_dynamics_mode(node: PoseWatcher, mode: str):
    log(f"pushing dynamics_mode={mode} into the live controller")
    set_controller_param(node, "FollowPath.dynamics_mode", mode)


def push_model_path(node: PoseWatcher, model_path: Path):
    log(f"pushing nn_model_path={model_path} into the live controller")
    set_controller_param(node, "FollowPath.nn_model_path", str(model_path))


def maybe_train_from_scratch(node: PoseWatcher, fs_cfg: dict, bag_dir: Path, new_bag_path: Path,
                              trial_label: str):
    """Called after every post-bootstrap trial once train_from_scratch is
    active. Retrains (or, on the very first call, trains from a blank init)
    the from-scratch MLP, saving each iteration's weights to its own file
    under bag_dir/from_scratch_weights/ -- the shared deployed model under
    nav2_mppi_controller's share dir is never read from or written to."""
    from nav2_stack import dynamics_retrain  # deferred -- see import comment near top of file
    is_first = fs_cfg["current_weights_path"] is None
    save_path = fs_cfg["weights_dir"] / f"mlp{fs_cfg['width']}_{trial_label}.pt"
    log(f"train_from_scratch: {'training from a blank init' if is_first else 'continuing'} "
        f"mlp{fs_cfg['width']} on data in {bag_dir} -> {save_path}")
    try:
        result = dynamics_retrain.retrain(
            bag_dir=bag_dir, new_bag_path=new_bag_path,
            model_type="mlp", width=fs_cfg["width"],
            warm_start=not is_first, subset=fs_cfg["subset"],
            subset_fraction=fs_cfg["subset_fraction"],
            fmean=fs_cfg["fmean"], fstd=fs_cfg["fstd"],
            warm_start_path=fs_cfg["current_weights_path"], save_path=save_path,
            failure_weighting=fs_cfg["failure_weighting"])
    except Exception as e:
        log(f"WARNING: train_from_scratch retrain failed ({e}) -- keeping the "
            f"currently deployed from-scratch weights")
        return

    fs_cfg["current_weights_path"] = Path(result["exported_path"])
    push_model_path(node, fs_cfg["current_weights_path"])
    if is_first:
        push_dynamics_mode(node, "neural_network")
    reload_controller(node)


def run_gp_explorer(gp_explorer_path: Path, bag_dir: Path, name: str) -> bool:
    # gp_explorer_gpu.py's own --map default is Path(__file__).resolve().parent.parent
    # / 'maps' / 'map.pgm' -- only correct when run from source. From the
    # installed location that resolves to .../site-packages/maps/map.pgm (wrong).
    # Override explicitly with the real installed path, same fix already
    # applied for pose_csv.
    map_path = Path(get_package_share_directory("nav2_stack")) / "maps" / "map.pgm"
    for attempt in range(1, GP_EXPLORER_MAX_ATTEMPTS + 1):
        log(f"gp_explorer_gpu.py attempt {attempt}/{GP_EXPLORER_MAX_ATTEMPTS} "
            f"(name={name})")
        result = subprocess.run(
            [sys.executable, str(gp_explorer_path), "--bag-dir", str(bag_dir), "--name", name,
             "--map", str(map_path)])
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
    parser.add_argument("--train-from-scratch", default="false", type=parse_bool,
                        help="Run the first --from-scratch-n-bootstrap trials under pure "
                             "kinematics, then train a fresh MLP from a blank init on just "
                             "that data and keep updating it after every trial after that. "
                             "Weights are stored per-iteration under bag_dir/from_scratch_weights/ "
                             "and the shared deployed model is never read from or written to. "
                             "Takes over from --retrain-dynamics if both are set. Default: false")
    parser.add_argument("--failure-weighting", default="true", type=parse_bool,
                        help="When retraining (retrain_dynamics or train_from_scratch), weight "
                             "samples from failed trials x2 and no-progress stretches x3 (cap x5) "
                             "in the MLP loss. Default: true")
    parser.add_argument("--from-scratch-n-bootstrap", default=5, type=int,
                        help="Number of initial kinematics-only trials to collect before the "
                             "first from-scratch fit. Default: 5")
    # launch_ros.actions.Node always appends --ros-args (and would append any
    # remappings/params too) to the process's argv, since it assumes the
    # executable parses those via rclpy's standard handling. Strip them before
    # plain argparse sees argv, or it chokes on --ros-args as unrecognized.
    import rclpy.utilities
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    bag_dir = args.bag_dir
    bag_dir.mkdir(parents=True, exist_ok=True)

    yaml_path = Path(get_package_share_directory("nav2_stack")) / "config" / "nav2_param2.yaml"
    parsed = parse_nav2_param_yaml(yaml_path.read_text())

    if args.train_from_scratch and args.retrain_dynamics:
        log("WARNING: both --train-from-scratch and --retrain-dynamics are true -- "
            "train_from_scratch takes over, retrain_dynamics is ignored")

    fs_cfg = None
    if args.train_from_scratch:
        fs_cfg = {
            "width": parsed["nn_hidden_width"],
            "n_bootstrap": args.from_scratch_n_bootstrap,
            "weights_dir": bag_dir / "from_scratch_weights",
            "current_weights_path": None,
            "subset": args.retrain_subset,
            "subset_fraction": args.retrain_subset_fraction,
            "fmean": np.array(parsed["fmean"], dtype=np.float32),
            "fstd": np.array(parsed["fstd"], dtype=np.float32),
            "failure_weighting": args.failure_weighting,
        }
        fs_cfg["weights_dir"].mkdir(parents=True, exist_ok=True)
        log(f"train_from_scratch enabled: first {fs_cfg['n_bootstrap']} trials (including "
            f"initial_bag) run under kinematics, then mlp{fs_cfg['width']} trains from a "
            f"blank init on that data and updates every trial after -- weights isolated "
            f"under {fs_cfg['weights_dir']}, shared deployed model untouched")

    retrain_cfg = None
    if args.retrain_dynamics and not args.train_from_scratch:
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
                "failure_weighting": args.failure_weighting,
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
    pose_node = PoseWatcher(args.opti_topic)
    wait_for_nav2_active()

    if fs_cfg is not None and parsed["dynamics_mode"] != "kinematics":
        log("train_from_scratch: forcing dynamics_mode=kinematics for the bootstrap phase "
            "(regardless of what nav2_param2.yaml currently has deployed)")
        push_dynamics_mode(pose_node, "kinematics")
        reload_controller(pose_node)

    try:
        initial_bag = bag_dir / "initial_bag"
        if not initial_bag.exists():
            log(f"no initial_bag found in {bag_dir} -- collecting one at the origin")
            origin_csv = bag_dir / "initial_goal.csv"
            with open(origin_csv, "w") as f:
                f.write("x,y,z\n0.0,0.0,0.0\n")
            for attempt in range(1, NO_MOVEMENT_MAX_ATTEMPTS + 1):
                outcome = run_one_trial(bag_dir, "initial_bag", origin_csv, (0.0, 0.0), pose_node,
                                        "initial_bag")
                if outcome != "no_movement":
                    break
                log(f"initial_bag: no movement detected (attempt {attempt}/"
                    f"{NO_MOVEMENT_MAX_ATTEMPTS}) -- discarding {initial_bag} and retrying")
                shutil.rmtree(initial_bag, ignore_errors=True)
            else:
                log(f"initial_bag: no movement detected {NO_MOVEMENT_MAX_ATTEMPTS} times in a "
                    f"row -- giving up, treating as a normal stall")
                outcome = "stalled"

            record_outcome(bag_dir, "initial_bag", outcome, (0.0, 0.0))
            if outcome == "reached":
                log("initial_bag: waypoint reached")
            elif outcome == "safety_stop":
                sys.exit("FATAL: safety_watchdog triggered an emergency stop during "
                         "initial_bag -- aborting entire run. Check the waypoint.launch.py "
                         "log for the safety_watchdog diagnostic message.")
            else:
                log("initial_bag: FAILED to reach waypoint (stalled)")
            maybe_retrain(pose_node, retrain_cfg, bag_dir, initial_bag)
        else:
            log(f"found existing {initial_bag}, skipping bootstrap")

        start_i = next_trial_start_index(bag_dir)
        if start_i > 1:
            log(f"found existing bags up to bag_{start_i - 1:02d} in {bag_dir} -- resuming "
                f"at trial {start_i} (target is {args.n_trajectories} total) instead of "
                f"restarting from trial 1")
        if start_i > args.n_trajectories:
            log(f"{start_i - 1} bags already exist, meeting or exceeding the "
                f"{args.n_trajectories}-trial target -- nothing new to run")

        n_success = 0
        n_run = 0
        for i in range(start_i, args.n_trajectories + 1):
            n_run += 1
            traj_name = f"traj_{i:02d}"
            bag_name = f"bag_{i:02d}"
            log(f"=== trial {i}/{args.n_trajectories} ({traj_name}) ===")

            bag_path = bag_dir / bag_name
            for attempt in range(1, NO_MOVEMENT_MAX_ATTEMPTS + 1):
                if not run_gp_explorer(gp_explorer_path, bag_dir, traj_name):
                    sys.exit(f"FATAL: gp_explorer_gpu.py failed {GP_EXPLORER_MAX_ATTEMPTS} "
                             f"times for {traj_name} -- aborting entire run")

                goal_xy = read_goal_xy(pose_csv)
                outcome = run_one_trial(bag_dir, bag_name, pose_csv, goal_xy, pose_node, traj_name)
                if outcome != "no_movement":
                    break
                log(f"trial {i}/{args.n_trajectories} ({traj_name}): no movement detected "
                    f"(attempt {attempt}/{NO_MOVEMENT_MAX_ATTEMPTS}) -- discarding {bag_path} "
                    f"and re-planning this trial")
                shutil.rmtree(bag_path, ignore_errors=True)
            else:
                log(f"trial {i}/{args.n_trajectories} ({traj_name}): no movement detected "
                    f"{NO_MOVEMENT_MAX_ATTEMPTS} times in a row -- giving up on this trial, "
                    f"treating as a normal stall")
                outcome = "stalled"

            record_outcome(bag_dir, bag_name, outcome, goal_xy)
            if outcome == "reached":
                n_success += 1
                log(f"trial {i}/{args.n_trajectories} ({traj_name}): waypoint reached")
            elif outcome == "safety_stop":
                sys.exit(f"FATAL: safety_watchdog triggered an emergency stop during "
                         f"trial {i}/{args.n_trajectories} ({traj_name}) -- aborting entire "
                         f"run. Check the waypoint.launch.py log for the safety_watchdog "
                         f"diagnostic message.")
            else:
                log(f"trial {i}/{args.n_trajectories} ({traj_name}): FAILED to reach "
                    f"waypoint (stalled) -- planning a new trajectory")
            maybe_retrain(pose_node, retrain_cfg, bag_dir, bag_dir / bag_name)
            if fs_cfg is not None and i >= fs_cfg["n_bootstrap"]:
                maybe_train_from_scratch(pose_node, fs_cfg, bag_dir, bag_dir / bag_name,
                                          f"trial{i:02d}")

        if start_i > 1:
            log(f"all trials complete: reached the {args.n_trajectories}-trial target "
                f"(ran {n_run} new trial(s) this session, {n_success}/{n_run} successful; "
                f"{start_i - 1} already existed from before)")
        else:
            log(f"all {args.n_trajectories} trials complete: "
                f"{n_success}/{args.n_trajectories} successful")
    finally:
        pose_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
