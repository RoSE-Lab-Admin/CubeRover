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
import contextlib
import csv
import math
import os
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

# Every node the two Nav2 lifecycle managers bring up (lifecycle_manager_navigation
# + lifecycle_manager_controller, nav2.launch.py). No amcl
# in this stack (ground-truth pose comes from OptiTrack instead) -- deliberately
# NOT using BasicNavigator.waitUntilNav2Active(), whose default localizer='amcl'
# would hang forever waiting for a node that's never launched. Matches the
# narrower per-node wait path_follower.py already does internally for a subset
# of these (planner_server/controller_server/bt_navigator); this waits for all
# 6, once, before any trial starts.
NAV2_MANAGED_NODES = ["map_server", "planner_server", "controller_server",
                      "behavior_server", "bt_navigator", "waypoint_follower"]
# controller_server alone is managed by this one (see nav2.launch.py)
CONTROLLER_LIFECYCLE_MANAGER = "lifecycle_manager_controller"

# dynamics_retrain (and therefore torch) is imported lazily, inside
# maybe_retrain(), so that retrain_dynamics=false (the default) never requires
# torch to be importable at all -- only opting into retraining does.

XY_GOAL_TOLERANCE = 0.5     # matches nav2_param2.yaml's goal_checker xy_goal_tolerance
STALL_WINDOW_S = 60.0       # no-progress-for-this-long => treat the trial as stuck
MIN_IMPROVEMENT_M = 0.05    # smaller than this doesn't count as "progress" (noise floor)
GP_EXPLORER_MAX_ATTEMPTS = 3
GP_UNCERTAINTY_TIMEOUT = 300      # s, gp_uncertainty.py (normally ~5 s on the GPU)
NO_MOVEMENT_MAX_ATTEMPTS = 3  # retries for a trial that recorded a "no_movement" bag (see wait_for_goal)
BAG_START_DELAY_S = 2.0     # let `ros2 bag record` actually start before commands flow

BAG_TOPICS = [
    "/FitRosey_passive/pose", "/rover/pose", "/dynamic_joint_states", "/cmd_vel",
    "/roseybot_base_controller/cmd_vel_out", "/plan", "/optimal_trajectory",
    "/optimal_trajectory_model", "/imu/data", "/imu/rpy",
]
# --record-markers: also every OptiTrack marker of the rigid body (natnet pub_rigid_body_marker,
# ~120 Hz x 8 points = ~1000 msg/s, so off by default): lets the rigid body -> base_link calibration
# (scripts/rover_calibration) be checked / redone from a trial, and shows mislabelled or moved markers
MARKER_TOPIC_REGEX = r"/FitRosey_passive/marker[0-9]+/pose"
RECORD_MARKERS = False   # set from --record-markers in main()


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
        self.yaw = None
        self.safety_stop = False
        self.goal_result = None  # None | "succeeded" | "failed"
        self.trial_id = None     # results tagged with another trial's id are ignored
        self.create_subscription(PoseStamped, opti_topic, self._cb, 10)
        self.create_subscription(Bool, "/safety_stop", self._safety_stop_cb, 10)
        self.create_subscription(String, "/trial_goal_result", self._goal_result_cb, 10)
        self.set_params_client = self.create_client(
            SetParameters, "/controller_server/set_parameters")
        self.planner_params_client = self.create_client(
            SetParameters, "/planner_server/set_parameters")
        # controller_server has its own lifecycle manager (nav2.launch.py), so
        # reloading the dynamics model cycles only it, not the whole stack
        self.manage_nodes_client = self.create_client(
            ManageLifecycleNodes, f"/{CONTROLLER_LIFECYCLE_MANAGER}/manage_nodes")

    def _cb(self, msg: PoseStamped):
        self.xy = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def pose(self):
        return None if self.xy is None or self.yaw is None else (self.xy[0], self.xy[1], self.yaw)

    def _safety_stop_cb(self, msg: Bool):
        if msg.data:
            self.safety_stop = True

    def _goal_result_cb(self, msg: String):
        # "<result>:<trial_id>" (path_follower's trial_id); a result of another
        # trial (a late one from the previous trial's path_follower) is ignored
        result, _, trial_id = msg.data.partition(":")
        if trial_id and trial_id != self.trial_id:
            log(f"ignoring /trial_goal_result '{msg.data}' (current trial: {self.trial_id})")
            return
        self.goal_result = result

    def reset_trial_state(self, trial_id=None):
        self.goal_result = None
        self.trial_id = trial_id


def wait_for_node_active(nav: BasicNavigator, node_name: str, timeout_sec: float = 5.0,
                          max_attempts: int = 60, retry_sec: float = 1.0) -> bool:
    """Bounded-timeout, bounded-retry replacement for BasicNavigator's private
    _waitForNodeToActivate(), which loops forever with NO timeout at all
    (rclpy.spin_until_future_complete with no timeout_sec, inside a
    while-not-active loop with no exit condition) -- if a node's get_state
    call ever fails to resolve cleanly (observed live: an RMW
    response-delivery timeout on planner_server's side), the unbounded
    version gets stuck forever with no way to recover. Same fix as
    path_follower.py's _wait_for_node_active(), duplicated here since this
    runs in a separate process with no shared module. Returns True once
    node_name reports 'active', False if it never does within max_attempts.
    Waits retry_sec between attempts: without it, a node that answers "not
    active yet" (planner_server needs ~30 s to configure) used up all attempts
    in under a second, and the planner parameters set right after were lost
    (the plugin only applies them once active).
    """
    from lifecycle_msgs.srv import GetState
    for attempt in range(max_attempts):
        if attempt:
            time.sleep(retry_sec)
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
    """Blocks until every Nav2-managed node (both lifecycle managers) reports
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
        else:
            log(f"  {node_name}: active ({time.monotonic() - t0:.1f}s elapsed)")
    nav.destroy_node()
    log(f"Nav2 fully active after {time.monotonic() - t0:.1f}s")


def wait_for_goal(node: PoseWatcher, goal_xy, label: str, route=None, trial_id=None,
                  stop_on_distance=True) -> str:
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

    route (waypoint_mode "spiral"): the leg's points; progress is then the
    length still to go along the route (nearest point, searched only a few
    metres ahead -- see spiral.route_remaining) instead of the straight-line
    distance to goal_xy, which does not shrink steadily on a spiral, and the
    distance fallback only counts within the last metre of the route.

    Trusts path_follower's own /trial_goal_result over the distance poll:
    Nav2's internal goToPose result and this function's independent
    re-derivation of "reached" from a coarser, staler pose-topic poll can
    disagree right at the tolerance boundary (observed live: Nav2 reported
    TaskResult.SUCCEEDED while this poll still saw 0.542m > 0.5m and the
    trial was wrongly marked STALLED after burning the full stall window).
    The distance check is kept as a fallback "reached" trigger too -- it can
    only ever fire *correctly* (same 0.5m tolerance Nav2's own goal checker
    uses), so keeping it can't reintroduce that bug, only catch a reached
    goal if /trial_goal_result were ever dropped. stop_on_distance=False turns
    it off (path_follower follow_mode "pose": the rover may pass the goal
    mid-manoeuvre, before it faces the right way)."""
    node.reset_trial_state(trial_id)
    best_dist = math.inf
    first_dist = None
    last_improve_t = time.monotonic()
    if route is not None:
        from nav2_stack.spiral import route_remaining
        route = np.asarray(route, dtype=float)
        route_cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(route, axis=0).T))])
        route_idx = 0
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
            goal_dist = math.hypot(node.xy[0] - goal_xy[0], node.xy[1] - goal_xy[1])
            dist = goal_dist
            if route is not None:
                dist, route_idx = route_remaining(route, route_cum, node.xy, route_idx)
            if first_dist is None:
                first_dist = dist
                best_dist = dist
            if (stop_on_distance and goal_dist < XY_GOAL_TOLERANCE
                    and (route is None or dist < XY_GOAL_TOLERANCE + 1.0)):
                log(f"{label}: goal reached (dist={goal_dist:.3f}m)")
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


def run_one_trial(bag_dir: Path, bag_name: str, pose_csv: Path, goal_xy, pose_node, label: str,
                  follow_args=(), route=None, stop_on_distance=True) -> str:
    """Returns 'reached', 'stalled', 'no_movement', or 'safety_stop' (see wait_for_goal).
    follow_args: extra waypoint.launch.py arguments (follow_mode, advance_distance);
    route: see wait_for_goal."""
    bag_path = bag_dir / bag_name
    trial_id = f"{label}@{int(time.time())}"  # unique per attempt (see PoseWatcher._goal_result_cb)
    bag_proc = start_process(["ros2", "bag", "record", *BAG_TOPICS,
                              *(["-e", MARKER_TOPIC_REGEX] if RECORD_MARKERS else []),
                              "-o", str(bag_path)])
    time.sleep(BAG_START_DELAY_S)

    wp_proc = start_process([
        "ros2", "launch", "nav2_stack", "waypoint.launch.py",
        "include_nav2:=false", f"pose_csv:={pose_csv}", f"trial_id:={trial_id}", *follow_args,
    ])

    try:
        outcome = wait_for_goal(pose_node, goal_xy, label, route, trial_id, stop_on_distance)
    finally:
        stop_process(wp_proc, "waypoint.launch.py")
        stop_process(bag_proc, "ros2 bag record")
    return outcome


@contextlib.contextmanager
def initial_bag_setup(node: PoseWatcher, pattern_mode: bool, live_mode: str):
    """Spiral / lawnmower initial bag (pattern_mode): driven under kinematics, and
    the planner (GridBasedCustom) plans to the goal's heading -- ignore_goal_heading
    and arc_mode_enabled (one arc through the goal, any heading) off. Everything is
    put back afterwards, also if the initial bag raises. No-op otherwise."""
    if not pattern_mode:
        yield
        return
    from nav2_stack.escape import default_planner_id
    import yaml
    planner = default_planner_id()
    cfg = yaml.safe_load(open(Path(get_package_share_directory("nav2_stack")) / "config"
                              / "nav2_param2.yaml"))["planner_server"]["ros__parameters"].get(planner, {})
    restore = {}
    for name, value in (("ignore_goal_heading", False), ("arc_mode_enabled", False)):
        if name not in cfg:
            continue        # stock GridBased: no such param, it plans to the heading anyway
        try:
            set_controller_param(node, f"{planner}.{name}", value, server="planner_server")
            restore[name] = cfg[name]
        except Exception as e:
            log(f"WARNING: could not set {planner}.{name}={value} ({e}) -- the initial bag may "
                f"not arrive lined up with the first leg")
    if restore:
        log(f"initial_bag: {planner} plans to the goal heading "
            f"({', '.join(f'{k}=false' for k in restore)})")
    forced = live_mode != "kinematics"
    if forced:
        log(f"initial_bag: driving it under kinematics (deployed: {live_mode})")
        push_dynamics_mode(node, "kinematics")
        reload_controller(node)
    try:
        yield
    finally:
        for name, value in restore.items():
            try:
                set_controller_param(node, f"{planner}.{name}", bool(value), server="planner_server")
            except Exception as e:
                log(f"WARNING: could not restore {planner}.{name}={value} ({e}) -- restart the "
                    f"launch before relying on the planner")
        if restore:
            log(f"initial_bag: {planner} planner settings restored "
                f"({', '.join(f'{k}={str(v).lower()}' for k, v in restore.items())})")
        if forced:
            log(f"initial_bag: back to {live_mode}")
            push_dynamics_mode(node, live_mode)
            reload_controller(node)


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
        # past cmd_vel_out steps the MLP sees (training must match the controller)
        "nn_history_steps": int(grab_scalar("nn_history_steps", "0")),
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
                          service_wait_sec=10.0, server="controller_server"):
    """Sets a /controller_server parameter (or /planner_server with
    server="planner_server") via a persistent rclpy service client (see
    PoseWatcher docstring for why not `ros2 param set`), with an explicit
    timeout and a bounded retry instead of hanging indefinitely."""
    client = node.planner_params_client if server == "planner_server" else node.set_params_client
    def _do():
        if not client.wait_for_service(timeout_sec=service_wait_sec):
            raise RuntimeError(f"/{server}/set_parameters not available "
                                f"after {service_wait_sec}s")
        req = SetParameters.Request(parameters=[ParameterMsg(name=name, value=_param_value(value))])
        future = client.call_async(req)
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
    """Calls the controller's lifecycle manager (CONTROLLER_LIFECYCLE_MANAGER)
    ManageLifecycleNodes service via a persistent rclpy client (see
    PoseWatcher docstring), with an explicit timeout and a bounded retry.
    timeout_sec is generous (controller_server may need to recapture its CUDA
    graph; a full-stack cycle used to take 30-100 s) but still bounded, so a
    genuine hang is caught and retried instead of blocking forever."""
    def _do():
        if not node.manage_nodes_client.wait_for_service(timeout_sec=service_wait_sec):
            raise RuntimeError(f"/{CONTROLLER_LIFECYCLE_MANAGER}/manage_nodes not available "
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
    """Cycles controller_server via its own lifecycle manager's
    ManageLifecycleNodes service (CONTROLLER_LIFECYCLE_MANAGER, RESET=3 then
    STARTUP=0), so it reconstructs NNDynamics fresh (re-reads the .pt file /
    just-pushed linear/dynamics_mode params) -- weights are otherwise only
    ever loaded once at startup. controller_server is the only node that
    manager owns (nav2.launch.py), so the rest of the stack -- planner_server
    alone takes ~27 s to reconfigure -- is not touched.

    Does NOT use direct per-node `ros2 lifecycle set` calls on controller_server
    -- confirmed by trial and error that lifecycle_manager_navigation reacts to
    ANY externally-driven state change on a node it manages (not just a bond
    heartbeat timeout) and starts its own concurrent recovery, racing whatever
    sequence we're mid-way through (observed as "transition not registered"
    errors and the whole process crashing while Nav2 silently self-healed with
    nothing left running to notice). Going through the manager's own official
    control service instead means the manager is the one making the state
    changes, so it has no "unexpected" state to react to."""
    log(f"cycling controller_server via {CONTROLLER_LIFECYCLE_MANAGER} to reload the "
        f"dynamics model (RESET then STARTUP)")
    t0 = time.monotonic()
    call_manage_nodes(node, 3, "RESET")
    call_manage_nodes(node, 0, "STARTUP")
    log(f"controller_server reloaded in {time.monotonic() - t0:.1f}s")


def record_deployed_model(bag_dir: Path, bag_name: str, fs_cfg: dict):
    """Append which model drove this trial to bag_dir/deployed_models.csv
    (train_from_scratch only). Serially that is always the model trained after
    the previous trial, but with --async-training it is the one from two trials
    back, so the analysis scripts read this file instead of assuming."""
    path = bag_dir / "deployed_models.csv"
    new = not path.exists()
    model = fs_cfg["current_weights_path"]
    with open(path, "a") as f:
        if new:
            f.write("bag,dynamics_mode,model\n")
        f.write(f"{bag_name},{'kinematics' if model is None else 'neural_network'},"
                f"{'' if model is None else Path(model).name}\n")


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
        f"{retrain_cfg.get('width', '')}{' terrain' if retrain_cfg['terrain'] else ''} on data "
        f"in {bag_dir} (warm_start={retrain_cfg['warm_start']}, subset={retrain_cfg['subset']})")
    # terrain: never the shared deployed model (that one has no terrain inputs) -- each
    # iteration gets its own file under bag_dir/terrain_weights/, warm-started from
    # the previous one (a blank init before the first)
    terrain_paths = {}
    if retrain_cfg["terrain"]:
        current = retrain_cfg["current_weights_path"]
        terrain_paths = dict(
            warm_start_path=current,
            save_path=retrain_cfg["weights_dir"] /
            f"{model_stem(retrain_cfg['width'], True)}_{new_bag_path.name}.pt")
    try:
        result = dynamics_retrain.retrain(
            bag_dir=bag_dir, new_bag_path=new_bag_path,
            model_type=retrain_cfg["model_type"], width=retrain_cfg.get("width"),
            warm_start=retrain_cfg["warm_start"] and (
                not retrain_cfg["terrain"] or retrain_cfg["current_weights_path"] is not None),
            subset=retrain_cfg["subset"], subset_fraction=retrain_cfg["subset_fraction"],
            fmean=retrain_cfg["fmean"], fstd=retrain_cfg["fstd"],
            failure_weighting=retrain_cfg["failure_weighting"],
            symmetry_augmentation=retrain_cfg["symmetry_augmentation"],
            history_steps=retrain_cfg["history_steps"], terrain=retrain_cfg["terrain"],
            **terrain_paths)
    except Exception as e:
        log(f"WARNING: retrain failed ({e}) -- keeping the currently deployed weights")
        return

    if retrain_cfg["model_type"] == "linear":
        push_linear_params_and_reload(node, result["weight"], result["bias"])
    else:
        log(f"exported retrained mlp to {result['exported_path']}")
        if retrain_cfg["terrain"]:
            retrain_cfg["current_weights_path"] = Path(result["exported_path"])
            push_model_path(node, retrain_cfg["current_weights_path"])
        reload_controller(node)


def push_dynamics_mode(node: PoseWatcher, mode: str):
    log(f"pushing dynamics_mode={mode} into the live controller")
    set_controller_param(node, "FollowPath.dynamics_mode", mode)


def push_model_path(node: PoseWatcher, model_path: Path):
    log(f"pushing nn_model_path={model_path} into the live controller")
    set_controller_param(node, "FollowPath.nn_model_path", str(model_path))


def model_stem(width: int, terrain: bool) -> str:
    """Checkpoint file name prefix: terrain models get their own, so resuming never
    picks up a checkpoint of the other kind."""
    return f"mlp{width}_terrain" if terrain else f"mlp{width}"


def latest_from_scratch_checkpoint(weights_dir: Path, stem: str, label: str = "trial"):
    """Newest {stem}_{label}NN.pt in weights_dir (highest NN), or None. Lets a
    train_from_scratch session in an existing bag_dir resume from the last
    checkpoint instead of going back to kinematics and a blank init (label "bag_"
    for retrain_dynamics' terrain checkpoints)."""
    best, best_n = None, -1
    for p in weights_dir.glob(f"{stem}_{label}*.pt"):
        m = re.fullmatch(rf"{re.escape(stem)}_{label}(\d+)\.pt", p.name)
        if m and int(m.group(1)) > best_n:
            best, best_n = p, int(m.group(1))
    return best


def _from_scratch_retrain_kwargs(fs_cfg: dict, bag_dir: Path, new_bag_path: Path,
                                 save_path: Path, bags=None) -> dict:
    """dynamics_retrain.retrain() arguments for the next from-scratch iteration:
    warm-started from the currently deployed checkpoint, or a blank init if
    there is none yet."""
    return dict(
        bag_dir=bag_dir, new_bag_path=new_bag_path,
        model_type="slip_gp" if fs_cfg["terrain_gp"] else "mlp", width=fs_cfg["width"],
        warm_start=fs_cfg["current_weights_path"] is not None, subset=fs_cfg["subset"],
        subset_fraction=fs_cfg["subset_fraction"],
        fmean=fs_cfg["fmean"], fstd=fs_cfg["fstd"],
        warm_start_path=fs_cfg["current_weights_path"], save_path=save_path,
        failure_weighting=fs_cfg["failure_weighting"],
        symmetry_augmentation=fs_cfg["symmetry_augmentation"],
        history_steps=fs_cfg["history_steps"], bags=bags, terrain=fs_cfg["terrain"])


def deploy_from_scratch(node: PoseWatcher, fs_cfg: dict, model_path: Path):
    is_first = fs_cfg["current_weights_path"] is None
    fs_cfg["current_weights_path"] = Path(model_path)
    push_model_path(node, fs_cfg["current_weights_path"])
    if is_first:
        push_dynamics_mode(node, "neural_network")
    reload_controller(node)


def maybe_train_from_scratch(node: PoseWatcher, fs_cfg: dict, bag_dir: Path, new_bag_path: Path,
                              trial_label: str):
    """Called after every post-bootstrap trial once train_from_scratch is
    active. Retrains (or, on the very first call, trains from a blank init)
    the from-scratch MLP, saving each iteration's weights to its own file
    under bag_dir/from_scratch_weights/ -- the shared deployed model under
    nav2_mppi_controller's share dir is never read from or written to."""
    from nav2_stack import dynamics_retrain  # deferred -- see import comment near top of file
    is_first = fs_cfg["current_weights_path"] is None
    save_path = fs_cfg["weights_dir"] / f"{fs_cfg['stem']}_{trial_label}.pt"
    log(f"train_from_scratch: {'training from a blank init' if is_first else 'continuing'} "
        f"{fs_cfg['stem']} on data in {bag_dir} -> {save_path}")
    try:
        result = dynamics_retrain.retrain(
            **_from_scratch_retrain_kwargs(fs_cfg, bag_dir, new_bag_path, save_path))
    except Exception as e:
        log(f"WARNING: train_from_scratch retrain failed ({e}) -- keeping the "
            f"currently deployed from-scratch weights")
        return

    deploy_from_scratch(node, fs_cfg, Path(result["exported_path"]))


def finished_bags(bag_dir: Path):
    """Every bag directory under bag_dir right now, same selection and order as
    dynamics_retrain.get_bags(). Duplicated here so the async path never
    imports dynamics_retrain (torch) into this process: the training runs in
    its own process and this one should hold no CUDA context of its own."""
    def sort_key(p):
        nums = re.findall(r"\d+", p.name)
        return int(nums[0]) if nums else -1
    return sorted({p.parent for p in bag_dir.rglob("metadata.yaml")}, key=sort_key)


def start_background_from_scratch(fs_cfg: dict, bag_dir: Path, new_bag_path: Path,
                                  trial_label: str) -> dict:
    """--async-training: start the next from-scratch iteration in a background
    process and return right away, so the next trial can be planned and driven
    meanwhile (still on the previously deployed model). Trains on exactly the
    bags finished so far -- the next trial's bag, recorded into the same
    bag_dir while this runs, is never picked up half-written. Writes to a
    .partial file that finish_background_from_scratch() renames, so an
    aborted run never leaves a truncated mlp*_trialNN.pt for the resume logic
    to pick up."""
    from nav2_stack.async_retrain import BackgroundRetrain
    save_path = fs_cfg["weights_dir"] / f"{fs_cfg['stem']}_{trial_label}.pt"
    tmp_path = save_path.with_name(save_path.name + ".partial")
    log_path = fs_cfg["weights_dir"] / f"train_{trial_label}.log"
    bags = finished_bags(bag_dir)
    current = fs_cfg["current_weights_path"]
    job = BackgroundRetrain(
        _from_scratch_retrain_kwargs(fs_cfg, bag_dir, new_bag_path, tmp_path, bags), log_path)
    log(f"async training: {'training from a blank init' if current is None else 'continuing'} "
        f"{fs_cfg['stem']} on {len(bags)} bags -> {save_path.name} in the background "
        f"(log: {log_path}); the next trial drives on "
        f"{'kinematics' if current is None else Path(current).name}")
    return {"job": job, "label": trial_label, "save_path": save_path, "tmp_path": tmp_path}


def finish_background_from_scratch(node: PoseWatcher, fs_cfg: dict, pending):
    """--async-training: wait for the background iteration started after the
    previous trial (if it is still running) and deploy it, so the coming trial
    drives on the model trained through the trial before the one that just
    ended. A failed iteration keeps the currently deployed weights, as in the
    serial path."""
    if pending is None:
        return
    job, label = pending["job"], pending["label"]
    t0 = time.monotonic()
    if not job.done():
        log(f"async training: waiting for {label}'s model (training for {job.elapsed():.0f} s so far)")
    try:
        job.wait()
    except RuntimeError as e:
        pending["tmp_path"].unlink(missing_ok=True)
        log(f"WARNING: async training of {label} failed ({e}) -- keeping the currently "
            f"deployed from-scratch weights")
        return
    waited = time.monotonic() - t0
    os.replace(pending["tmp_path"], pending["save_path"])
    log(f"async training: {label} finished after {job.elapsed():.0f} s "
        f"({f'waited {waited:.0f} s for it' if waited >= 1.0 else 'no waiting'}) -- "
        f"deploying {pending['save_path'].name}")
    deploy_from_scratch(node, fs_cfg, pending["save_path"])


def abort_background_from_scratch(pending):
    if pending is None or pending["job"].done():
        return
    log(f"async training: stopping the unfinished background training of {pending['label']}")
    pending["job"].kill()
    pending["tmp_path"].unlink(missing_ok=True)


SPIRAL_ADVANCE_DISTANCE = 1.5  # m: option A keeps its goal this far ahead of the rover along the leg


def write_pattern_trial(bag_dir: Path, traj_name: str, leg, drive: str, name: str) -> Path:
    """The leg's points for waypoint.launch.py's pose_csv: every
    spiral.WAYPOINT_SPACING m for "waypoints" (spiral option A), the dense leg for
    "path". Kept in bag_dir as <traj_name>_<name>.csv (name: spiral / lawnmower)."""
    pts = (leg.waypoints() if drive == "waypoints"
           else leg.points[-1:] if drive in ZIGZAG_DRIVES else leg.points)
    path = bag_dir / f"{traj_name}_{name}.csv"
    with open(path, "w") as f:
        f.write("x,y,z\n")
        for x, y in pts:
            f.write(f"{x:.4f},{y:.4f},0.0\n")
    return path


ZIGZAG_DRIVES = ("zigzag", "zigzag_heading")


def pattern_follow_args(drive: str, leg, cusp_axis: bool = False):
    """waypoint.launch.py arguments for one spiral / lawnmower leg. cusp_axis: the
    goal yaw (E-W) is also the axis cusps are kept to (--lawnmower-cusp-tolerance)."""
    if drive == "waypoints":
        return ["follow_mode:=goals", f"advance_distance:={SPIRAL_ADVANCE_DISTANCE}"]
    if drive == "zigzag" and not cusp_axis:
        return ["follow_mode:=goals"]
    if drive in ZIGZAG_DRIVES:   # zigzag_heading: the planner arrives within the tolerance of E-W
        return ["follow_mode:=goals", f"goal_yaw:={leg.yaw}"]
    return ["follow_mode:=path"]


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


def run_gp_uncertainty(script: Path, bag_dir: Path, bag_name: str):
    """Terrain GP's global uncertainty after this trial (gp_uncertainty.py: trains the GP on
    every bag so far and appends a row to bag_dir/gp_uncertainty.csv). A subprocess, so it
    frees the GPU when done. Never fatal: a failure only logs a warning."""
    map_path = Path(get_package_share_directory("nav2_stack")) / "maps" / "map.pgm"
    try:
        result = subprocess.run(
            [sys.executable, str(script), "--bag-dir", str(bag_dir), "--bag", bag_name,
             "--map", str(map_path)], timeout=GP_UNCERTAINTY_TIMEOUT,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        line = next((l for l in result.stdout.splitlines() if l.startswith("[gp_uncertainty]")), None)
        if result.returncode == 0 and line:
            log(line)
        else:
            log(f"WARNING: gp_uncertainty.py failed for {bag_name} (exit {result.returncode}): "
                f"{result.stdout.strip().splitlines()[-1] if result.stdout.strip() else ''}")
    except Exception as e:
        log(f"WARNING: gp_uncertainty.py failed for {bag_name}: {e}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-dir", required=True, type=Path,
                        help="Directory bags are stored in and loaded from")
    parser.add_argument("--n-trajectories", default=25, type=int,
                        help="Number of trials to run (default: 25)")
    parser.add_argument("--opti-topic", default="/rover/pose",
                        help="Ground-truth rover pose topic (default: /rover/pose = base_link, the rover "
                             "centre, from the rover_pose node)")
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
    parser.add_argument("--symmetry-augmentation", default="false", type=parse_bool,
                        help="When retraining, also train on mirrored copies of the data "
                             "(left/right x0.5, forward/backward x0.25, both x0.25; see "
                             "dynamics_retrain.SYM_WEIGHTS). Default: false")
    parser.add_argument("--async-training", default="false", type=parse_bool,
                        help="train_from_scratch only: train in a background process while the "
                             "next trial is planned and driven, instead of between trials. "
                             "Trial i then drives on the model trained through trial i-2 "
                             "(one trajectory older than serially); if that model isn't done "
                             "when trial i-1 ends, the run waits for it. The first "
                             "--async-serial-trials trials still train serially. Default: false")
    parser.add_argument("--async-serial-trials", default=15, type=int,
                        help="With --async-training: trials 1..N still train serially (between "
                             "trials, so the next trial gets the newest model), because early on "
                             "training is quick and each new trial changes the model the most. "
                             "Async starts from trial N+1. Default: 15")
    parser.add_argument("--terrain-nn", default="false", type=parse_bool,
                        help="With train_from_scratch or retrain_dynamics (MLP): train the terrain "
                             "MLP instead -- OptiTrack pitch/roll as inputs, a lateral output, and "
                             "a GP height map (refit every retrain) for the attitude of MPPI's "
                             "future rollout steps. Checkpoints are kept apart from the plain "
                             "ones. Default: false")
    parser.add_argument("--terrain-gp", default="false", type=parse_bool,
                        help="train_from_scratch only: instead of an MLP, fit kinematics with slip "
                             "learned from the slope -- a second GP maps the GP height map's slope "
                             "(climb along the direction of travel, side slope) to linear and "
                             "angular slip ratios, refit every trial. Takes over from --terrain-nn "
                             "if both are set. Default: false")
    parser.add_argument("--waypoint-mode", default="gp", choices=["gp", "spiral", "lawnmower"],
                        help="How each trial's goal is chosen: gp = gp_explorer_gpu.py's "
                             "information-based goal (default); spiral = the next leg of a spiral "
                             "around --spiral-centre (see nav2_stack/spiral.py): one bag per leg, "
                             "from the inner circle to the outside or back in; lawnmower = lanes "
                             "along x joined by S-curves (see nav2_stack/lawnmower.py), one bag per "
                             "lane, up and back down, driven in path mode")
    parser.add_argument("--spiral-drive", default="path", choices=["path", "waypoints"],
                        help="waypoint-mode spiral: path = drive to the leg's start, then the leg "
                             "itself straight to the controller (FollowPath, no global planner; "
                             "default -- drove the spiral forward in the offline simulator); "
                             "waypoints = waypoints every 1 m along the leg, the goal kept ~1.5 m "
                             "ahead of the rover's progress along it, planned by the global planner "
                             "(often ended up driving the spiral in reverse offline)")
    parser.add_argument("--spiral-radius", default=1.8, type=float,
                        help="waypoint-mode spiral: radius of the inner circle = the tightest turn "
                             "(m). Default: 1.8")
    parser.add_argument("--lawnmower-spacing", default=0.8, type=float,
                        help="waypoint-mode lawnmower: distance between lanes (m). Default: 0.8")
    parser.add_argument("--lawnmower-drive", default="zigzag_heading",
                        choices=["path", "zigzag", "zigzag_heading"],
                        help="waypoint-mode lawnmower: path = S-curve lanes straight to the "
                             "controller; zigzag = one planned goal per trial, the next "
                             "lane's end on the other side (any arrival heading); zigzag_heading = "
                             "the same, but the planner has to arrive heading E-W (either way) "
                             "within --lawnmower-heading-tolerance (default since 11/21, V6)")
    parser.add_argument("--lawnmower-heading-tolerance", default=20.0, type=float,
                        help="lawnmower-drive zigzag_heading: allowed arrival heading off E-W "
                             "(deg). Default: 20")
    parser.add_argument("--lawnmower-cusp-tolerance", default=30.0, type=float,
                        help="lawnmower-drive zigzag / zigzag_heading: the planner only plans a "
                             "gear change (three-point turn) while the rover points within this "
                             "many deg of E-W, so turns shuffle along the lanes, not N-S across "
                             "them. Default: 30 (V6); < 0: off")
    parser.add_argument("--lawnmower-cusp-start-tolerance", default=60.0, type=float,
                        help="with --lawnmower-cusp-tolerance: reversing the rover right where it "
                             "is (while it moves) may be up to this many deg off E-W (the rover "
                             "reaches planned cusps further turned than planned). Default: 60")
    parser.add_argument("--lawnmower-approach-length", default=0.5, type=float,
                        help="lawnmower-drive zigzag_heading: the last this many m of every plan "
                             "stay within --lawnmower-heading-tolerance (the goal checker stops "
                             "the rover ~0.5 m short of the lane end). Default: 0.5; <= 0: off")
    parser.add_argument("--lawnmower-clearance", default=0.6, type=float,
                        help="waypoint-mode lawnmower: lanes keep at least this far (m) from walls and "
                             "craters in the map. 0.6 (11/22) plans the lane ends into the 0.75 m "
                             "inflation, so the lanes between the craters fit with the true crater "
                             "rims. Default: 0.6")
    parser.add_argument("--record-markers", default="false", type=parse_bool,
                        help="also record every OptiTrack marker of the rigid body in the trial bags "
                             "(calibration check, ~1000 msg/s). Default: false")
    parser.add_argument("--lawnmower-cusp-tail", default=-1.0, type=float,
                        help="lawnmower-drive zigzag / zigzag_heading: override GridBasedCustom "
                             "cusp_tail_length (m) for the run; < 0 (default): keep nav2_param2.yaml's "
                             "(0.25 m, after gear changes only, since 11/22)")
    parser.add_argument("--lawnmower-cusp-tail-before", default="false", type=parse_bool,
                        help="with --lawnmower-cusp-tail >= 0: cusp tail also BEFORE a gear change "
                             "(false: only after it -- before it the rover still has momentum). "
                             "Default: false")
    parser.add_argument("--lawnmower-momentum-zone", default=-1.0, type=float,
                        help="lawnmower-drive zigzag / zigzag_heading: override GridBasedCustom "
                             "momentum_zone_length (m) for the run; < 0 (default): keep "
                             "nav2_param2.yaml's (0.4 m since 11/22)")
    parser.add_argument("--lawnmower-free-start-reversal", default="false", type=parse_bool,
                        help="lawnmower-drive zigzag / zigzag_heading: GridBasedCustom "
                             "free_start_reversal -- stopping and reversing the moving rover at "
                             "the start of a replan does not count toward the 2nd-gear-change "
                             "ban (still costs motion_reversal_penalty), so a moving rover can "
                             "plan a 3-point turn instead of a long loop. Default: false")
    parser.add_argument("--path-align", default="auto", choices=["auto", "on", "off", "alternate"],
                        help="PathAlignCritic for every waypoint mode: auto = on for gp (at "
                             "--path-align-weight) and the spiral / lawnmower path drives (yaml "
                             "weight), --lawnmower-path-align for the zigzag drives; on / off = every "
                             "trial; alternate = odd trials on, even trials (and the initial bag) off, "
                             "for an A/B comparison within one run (on/off/alternate use "
                             "--path-align-weight). Every trial's setting is written to "
                             "<bag-dir>/path_align.csv. Default: auto")
    parser.add_argument("--path-align-weight", default=14.0, type=float,
                        help="PathAlignCritic cost_weight for gp trials (auto) and with --path-align "
                             "on/off/alternate. Default: 14 (= --lawnmower-path-align-weight)")
    parser.add_argument("--lawnmower-path-align", default="true", type=parse_bool,
                        help="lawnmower-drive zigzag / zigzag_heading: switch PathAlignCritic on "
                             "(pulls MPPI back onto the plan's line) with cost_weight "
                             "--lawnmower-path-align-weight. Default: true (V6)")
    parser.add_argument("--lawnmower-path-align-weight", default=14.0, type=float,
                        help="with --lawnmower-path-align: PathAlignCritic cost_weight for this run "
                             "(nav2_param2.yaml's 30 is tuned for path mode). Default: 14 (stock)")
    parser.add_argument("--spiral-centre-x", default=0.0, type=float)
    parser.add_argument("--spiral-centre-y", default=0.0, type=float)
    parser.add_argument("--from-scratch-n-bootstrap", default=5, type=int,
                        help="Number of initial kinematics-only trials to collect before the "
                             "first from-scratch fit. Default: 5")
    # launch_ros.actions.Node always appends --ros-args (and would append any
    # remappings/params too) to the process's argv, since it assumes the
    # executable parses those via rclpy's standard handling. Strip them before
    # plain argparse sees argv, or it chokes on --ros-args as unrecognized.
    import rclpy.utilities
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    global RECORD_MARKERS
    RECORD_MARKERS = args.record_markers

    bag_dir = args.bag_dir
    bag_dir.mkdir(parents=True, exist_ok=True)

    yaml_path = Path(get_package_share_directory("nav2_stack")) / "config" / "nav2_param2.yaml"
    parsed = parse_nav2_param_yaml(yaml_path.read_text())

    if args.train_from_scratch and args.retrain_dynamics:
        log("WARNING: both --train-from-scratch and --retrain-dynamics are true -- "
            "train_from_scratch takes over, retrain_dynamics is ignored")

    if args.terrain_gp and args.terrain_nn:
        log("WARNING: both --terrain-gp and --terrain-nn are true -- terrain_gp takes over")
        args.terrain_nn = False
    if args.terrain_gp and not args.train_from_scratch:
        log("WARNING: --terrain-gp only applies with --train-from-scratch -- ignored")
        args.terrain_gp = False

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
            "symmetry_augmentation": args.symmetry_augmentation,
            "history_steps": parsed["nn_history_steps"],
            "async": args.async_training,
            "async_serial_trials": args.async_serial_trials,
            "terrain": args.terrain_nn,
            "terrain_gp": args.terrain_gp,
            "stem": "slip_gp" if args.terrain_gp else model_stem(parsed["nn_hidden_width"],
                                                                 args.terrain_nn),
        }
        fs_cfg["weights_dir"].mkdir(parents=True, exist_ok=True)
        # a .partial left by an aborted async run is never a usable checkpoint
        for p in fs_cfg["weights_dir"].glob("*.pt.partial"):
            p.unlink()
        fs_cfg["current_weights_path"] = latest_from_scratch_checkpoint(
            fs_cfg["weights_dir"], fs_cfg["stem"])
        if fs_cfg["current_weights_path"] is not None:
            log(f"train_from_scratch enabled, resuming from checkpoint "
                f"{fs_cfg['current_weights_path']}: no kinematics trials, the next trial runs "
                f"on it and training continues from it after every trial")
        else:
            log(f"train_from_scratch enabled: first {fs_cfg['n_bootstrap']} trials (including "
                f"initial_bag) run under kinematics, then {fs_cfg['stem']} trains from a "
                f"blank init on that data and updates every trial after -- weights isolated "
                f"under {fs_cfg['weights_dir']}, shared deployed model untouched")

    if args.async_training:
        if fs_cfg is None:
            log("WARNING: --async-training only applies to --train-from-scratch -- ignored")
        else:
            log(f"async training: trials 1-{args.async_serial_trials} train serially; from trial "
                f"{args.async_serial_trials + 1} on, each from-scratch iteration trains in the "
                f"background during the next trial, so trial i drives on the model trained "
                f"through trial i-2")

    if args.terrain_nn and not (args.train_from_scratch or args.retrain_dynamics):
        log("WARNING: --terrain-nn only applies with --train-from-scratch or "
            "--retrain-dynamics -- ignored")

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
            "symmetry_augmentation": args.symmetry_augmentation,
            "history_steps": parsed["nn_history_steps"],
            "terrain": args.terrain_nn and model_type == "mlp",
            }
            if args.terrain_nn and model_type != "mlp":
                log("WARNING: --terrain-nn only applies to the MLP (dynamics_mode "
                    "neural_network) -- retraining the linear model without it")
            if retrain_cfg["terrain"]:
                retrain_cfg["weights_dir"] = bag_dir / "terrain_weights"
                retrain_cfg["weights_dir"].mkdir(parents=True, exist_ok=True)
                retrain_cfg["current_weights_path"] = latest_from_scratch_checkpoint(
                    retrain_cfg["weights_dir"], model_stem(retrain_cfg["width"], True), "bag_")
            log(f"online retraining enabled: model={model_type}"
                f"{retrain_cfg['width'] or ''}{' terrain' if retrain_cfg['terrain'] else ''}  "
                f"warm_start={args.warm_start}  "
                f"subset={args.retrain_subset} (fraction={args.retrain_subset_fraction})")

    # Same package directory as gp_explorer_gpu.py (source or installed -- this
    # script and gp_explorer_gpu.py are always siblings either way), so this is
    # robust regardless of how autonomous_trials.py itself was invoked.
    pattern = None   # waypoint_mode spiral / lawnmower: {"name", "legs", "drive"}
    if args.waypoint_mode == "spiral":
        from nav2_stack.spiral import Clearance, spiral_legs
        map_yaml = Path(get_package_share_directory("nav2_stack")) / "maps" / "map.yaml"
        legs = spiral_legs(args.spiral_radius, (args.spiral_centre_x, args.spiral_centre_y),
                           Clearance(map_yaml), args.n_trajectories)
        if not legs:
            sys.exit(f"FATAL: no spiral leg of radius {args.spiral_radius} m around "
                     f"({args.spiral_centre_x}, {args.spiral_centre_y}) fits the map")
        pattern = {"name": "spiral", "legs": legs, "drive": args.spiral_drive}
        log(f"waypoint mode spiral: r_min {args.spiral_radius} m around ({args.spiral_centre_x}, "
            f"{args.spiral_centre_y}), drive={args.spiral_drive}; {len(legs)} legs, "
            f"{min(L.length for L in legs):.1f}-{max(L.length for L in legs):.1f} m long")
    elif args.waypoint_mode == "lawnmower":
        from nav2_stack.spiral import Clearance
        from nav2_stack.lawnmower import lawnmower_legs, zigzag_legs, zigzag_start
        map_yaml = Path(get_package_share_directory("nav2_stack")) / "maps" / "map.yaml"
        clearance = Clearance(map_yaml)
        if args.lawnmower_drive == "path":
            legs = lawnmower_legs(clearance, args.n_trajectories, args.lawnmower_spacing,
                                  args.lawnmower_clearance)
            start = None
        else:
            legs = zigzag_legs(clearance, args.n_trajectories, args.lawnmower_spacing,
                               args.lawnmower_clearance)
            start = zigzag_start(clearance, args.lawnmower_spacing, args.lawnmower_clearance)
        if not legs:
            sys.exit(f"FATAL: fewer than 2 lawnmower lanes {args.lawnmower_spacing} m apart fit the map")
        pattern = {"name": "lawnmower", "legs": legs, "drive": args.lawnmower_drive, "start": start}
        per_pass = {}
        for L in legs:
            per_pass[L.pass_no] = L.n_lanes
        how = {"path": "S-curve lane changes, path mode",
               "zigzag": "zigzag: one planned goal per trial, the next lane's end on the other side",
               "zigzag_heading": f"zigzag: one planned goal per trial, the next lane's end on the "
                                 f"other side, arriving within {args.lawnmower_heading_tolerance:.0f} "
                                 f"deg of E-W"}[args.lawnmower_drive]
        log(f"waypoint mode lawnmower: lanes {args.lawnmower_spacing} m apart, shifted by a third "
            f"of that every pass; lanes per pass {list(per_pass.values())}; {how}; {len(legs)} legs, "
            f"{min(L.length for L in legs):.1f}-{max(L.length for L in legs):.1f} m long")

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
    gp_uncertainty_path = gp_explorer_path.parent / "gp_uncertainty.py"

    rclpy.init()
    pose_node = PoseWatcher(args.opti_topic)
    # drives the rover out of an obstacle before planning the next trial; spins
    # pose_node, which is never added to a persistent executor
    from nav2_stack.escape import Escaper, default_planner_id
    escaper = Escaper(pose_node, pose_node.pose, default_planner_id())
    wait_for_nav2_active()
    # PathAlignCritic keeps MPPI on the line of a path that is not replanned (spiral /
    # lawnmower in path mode, at nav2_param2.yaml's weight 30). --lawnmower-path-align: also on
    # for the zigzag drives (11/21 V5 test), at its own weight. GP trials: on at
    # --path-align-weight (14, = lawnmower) since 11/22 -- the 11/22 alternate A/B (45 NN trials)
    # found no clear tracking difference (5% slower), and on everywhere keeps GP and lawnmower
    # runs comparable (11/06: weight 30 did worse with GP goals). Live-reconfigurable; the
    # controller_server starts fresh from nav2_param2.yaml with every launch.
    zigzag_align = (pattern is not None and pattern["drive"] in ZIGZAG_DRIVES
                    and args.lawnmower_path_align)
    gp_align = pattern is None
    path_align = gp_align or (pattern is not None and (pattern["drive"] == "path" or zigzag_align))
    if args.path_align != "auto" or gp_align:
        align_weight = float(args.path_align_weight)
    elif zigzag_align:
        align_weight = float(args.lawnmower_path_align_weight)
    else:   # spiral / lawnmower path drive: the yaml's weight
        import yaml
        align_weight = float(yaml.safe_load(open(yaml_path))["controller_server"]["ros__parameters"]
                             ["FollowPath"]["PathAlignCritic"]["cost_weight"])
    try:
        if zigzag_align or gp_align or args.path_align != "auto":
            set_controller_param(pose_node, "FollowPath.PathAlignCritic.cost_weight", align_weight)
        set_controller_param(pose_node, "FollowPath.PathAlignCritic.enabled", path_align)
        log(f"PathAlignCritic {'enabled' if path_align else 'disabled'} for waypoint_mode "
            f"{args.waypoint_mode} (cost_weight {align_weight:g}"
            + (", lawnmower_path_align" if zigzag_align else "") + ")"
            + (f"; path_align {args.path_align} sets it per trial" if args.path_align != "auto" else ""))
    except Exception as e:
        log(f"WARNING: could not set FollowPath.PathAlignCritic ({e}) -- is it "
            f"in nav2_param2.yaml's critics list?")

    def set_path_align(trial: int, bag_name: str):
        """PathAlignCritic for this trial (trial 0 = initial bag), recorded in path_align.csv for
        every mode. auto: as set above for the whole run; on / off / alternate: set here."""
        if args.path_align == "auto":
            enabled = path_align
        else:
            enabled = args.path_align == "on" or (args.path_align == "alternate" and trial % 2 == 1)
            try:
                set_controller_param(pose_node, "FollowPath.PathAlignCritic.enabled", enabled)
                log(f"{bag_name}: PathAlignCritic {'on' if enabled else 'off'} (path_align {args.path_align})")
            except Exception as e:
                log(f"WARNING: {bag_name}: could not set PathAlignCritic.enabled={enabled} ({e})")
        path = bag_dir / "path_align.csv"
        new = not path.exists()
        with open(path, "a") as f:
            if new:
                f.write("bag,path_align,cost_weight,mode\n")
            f.write(f"{bag_name},{int(enabled)},{align_weight:g},{args.path_align}\n")
    # lawnmower zigzag_heading: GridBasedCustom only accepts final headings within the
    # tolerance of the goal yaw (either way); --lawnmower-cusp-tolerance: gear changes only
    # near E-W. Put back to -1 when the run ends.
    planner_tols = {}
    if pattern is not None and pattern["drive"] in ZIGZAG_DRIVES:
        if pattern["drive"] == "zigzag_heading":
            planner_tols["goal_heading_tolerance_deg"] = (args.lawnmower_heading_tolerance,
                                                          "arrive within it of E-W")
        if pattern["drive"] == "zigzag_heading" and args.lawnmower_approach_length > 0:
            planner_tols["goal_approach_length"] = (args.lawnmower_approach_length,
                                                    "m before the lane end already within the "
                                                    "arrival tolerance")
        if args.lawnmower_cusp_tolerance >= 0:
            planner_tols["cusp_heading_tolerance_deg"] = (args.lawnmower_cusp_tolerance,
                                                          "gear changes only within it of E-W")
            if args.lawnmower_cusp_start_tolerance >= 0:
                planner_tols["cusp_start_heading_tolerance_deg"] = (
                    args.lawnmower_cusp_start_tolerance,
                    "reversing where the rover is: within it of E-W")
        if args.lawnmower_free_start_reversal:
            planner_tols["free_start_reversal"] = (True, "stopping and reversing the moving rover "
                                                         "does not count toward the 2-gear-change ban")
        # 11/21: shorter turning limits around gear changes for the lawnmower, tail after only
        if args.lawnmower_cusp_tail >= 0:
            planner_tols["cusp_tail_length"] = (args.lawnmower_cusp_tail,
                                                "m without a tight turn at a gear change")
            planner_tols["cusp_tail_before"] = (bool(args.lawnmower_cusp_tail_before),
                                                "cusp tail also before the gear change")
        if args.lawnmower_momentum_zone >= 0:
            planner_tols["momentum_zone_length"] = (args.lawnmower_momentum_zone,
                                                    "m of gentle turning after a stop / gear change")
    # values these go back to when the run ends: nav2_param2.yaml's for params it sets, else off
    planner_restore = {}
    if planner_tols:
        import yaml
        _cfg = yaml.safe_load(open(Path(get_package_share_directory("nav2_stack")) / "config"
                                   / "nav2_param2.yaml"))["planner_server"]["ros__parameters"].get(
                                       default_planner_id(), {})
        for _p, (_v, _w) in planner_tols.items():
            _off = False if isinstance(_v, bool) else -1.0
            _y = _cfg.get(_p, _off)
            planner_restore[_p] = bool(_y) if isinstance(_v, bool) else float(_y)
    heading_tol_set = None
    for pname, (val, what) in planner_tols.items():
        planner = default_planner_id()
        try:
            set_controller_param(pose_node, f"{planner}.{pname}",
                                 val if isinstance(val, bool) else float(val),
                                 server="planner_server")
            heading_tol_set = planner
            log(f"{planner}: {pname}={val:g} ({what})" if not isinstance(val, bool) else
                f"{planner}: {pname}={str(val).lower()} ({what})")
        except Exception as e:
            log(f"WARNING: could not set {planner}.{pname} ({e}) -- the planner needs the "
                f"GridBasedCustom build with it; the zigzag runs without it")

    live_mode = parsed["dynamics_mode"]   # dynamics_mode the controller runs right now
    if fs_cfg is not None and fs_cfg["current_weights_path"] is not None:
        log(f"train_from_scratch: deploying checkpoint {fs_cfg['current_weights_path']} "
            f"(neural_network) for the first new trial")
        push_model_path(pose_node, fs_cfg["current_weights_path"])
        push_dynamics_mode(pose_node, "neural_network")
        reload_controller(pose_node)
        live_mode = "neural_network"
    elif fs_cfg is not None and parsed["dynamics_mode"] != "kinematics":
        log("train_from_scratch: forcing dynamics_mode=kinematics for the bootstrap phase "
            "(regardless of what nav2_param2.yaml currently has deployed)")
        push_dynamics_mode(pose_node, "kinematics")
        reload_controller(pose_node)
        live_mode = "kinematics"

    if retrain_cfg is not None and retrain_cfg["terrain"]:
        if retrain_cfg["current_weights_path"] is not None:
            log(f"terrain_nn: deploying checkpoint {retrain_cfg['current_weights_path']} for "
                f"the first new trial")
            push_model_path(pose_node, retrain_cfg["current_weights_path"])
            reload_controller(pose_node)
        else:
            log("terrain_nn: no terrain checkpoint in this bag_dir yet -- the deployed (plain) "
                "model drives until the first retrain, which starts from a blank init")

    pending = None  # --async-training: the background iteration started after the last trial
    try:
        initial_bag = bag_dir / "initial_bag"
        if not initial_bag.exists():
            origin_csv = bag_dir / "initial_goal.csv"
            if pattern is None:
                log(f"no initial_bag found in {bag_dir} -- collecting one at the origin")
                initial_xy, initial_args = (0.0, 0.0), ()
                with open(origin_csv, "w") as f:
                    f.write("x,y,z\n0.0,0.0,0.0\n")
            else:
                # spiral / lawnmower: drive to the first leg's start and arrive lined
                # up with it (either way round), so trial 1 starts on the leg
                (x0, y0), (x1, y1) = (pattern.get("start") or pattern["legs"][0].points)[:2]
                d = math.hypot(x1 - x0, y1 - y0)
                initial_xy, initial_args = (x0, y0), ("follow_mode:=pose",)
                log(f"no initial_bag found in {bag_dir} -- collecting one by driving to the "
                    f"first {pattern['name']} leg's start ({x0:.2f}, {y0:.2f}), lined up with it")
                with open(origin_csv, "w") as f:
                    f.write(f"x,y,z\n{x0:.4f},{y0:.4f},0.0\n"
                            f"{x0 + (x1 - x0) / d:.4f},{y0 + (y1 - y0) / d:.4f},0.0\n")
            set_path_align(0, "initial_bag")
            with initial_bag_setup(pose_node, pattern is not None, live_mode):
                for attempt in range(1, NO_MOVEMENT_MAX_ATTEMPTS + 1):
                    outcome = run_one_trial(bag_dir, "initial_bag", origin_csv, initial_xy, pose_node,
                                            "initial_bag", initial_args,
                                            stop_on_distance=pattern is None)
                    if outcome != "no_movement":
                        break
                    log(f"initial_bag: no movement detected (attempt {attempt}/"
                        f"{NO_MOVEMENT_MAX_ATTEMPTS}) -- discarding {initial_bag} and retrying")
                    shutil.rmtree(initial_bag, ignore_errors=True)
                else:
                    log(f"initial_bag: no movement detected {NO_MOVEMENT_MAX_ATTEMPTS} times in a "
                        f"row -- giving up, treating as a normal stall")
                    outcome = "stalled"

            record_outcome(bag_dir, "initial_bag", outcome, initial_xy)
            run_gp_uncertainty(gp_uncertainty_path, bag_dir, "initial_bag")
            if fs_cfg is not None:
                record_deployed_model(bag_dir, "initial_bag", fs_cfg)
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
            set_path_align(i, bag_name)

            bag_path = bag_dir / bag_name
            for attempt in range(1, NO_MOVEMENT_MAX_ATTEMPTS + 1):
                # a rover left in an obstacle by the last trial would make
                # gp_explorer find no path from it at all -- drive it out
                # first (toward the arena centre; see escape.py)
                ok, msg = escaper.run((0.0, 0.0))
                if msg != "not stuck":
                    log(f"pre-explorer escape: {'freed' if ok else 'not freed'} ({msg})")
                if pattern is not None:
                    leg = pattern["legs"][(i - 1) % len(pattern["legs"])]
                    trial_csv = write_pattern_trial(bag_dir, traj_name, leg, pattern["drive"],
                                                    pattern["name"])
                    goal_xy = leg.points[-1]
                    what = (f"spiral leg {i} {leg.direction}, r {leg.r0:.2f} -> {leg.r1:.2f} m"
                            if pattern["name"] == "spiral" else f"lawnmower {leg.label}")
                    log(f"{traj_name}: {what}, {leg.length:.1f} m, from ({leg.points[0][0]:.2f}, "
                        f"{leg.points[0][1]:.2f}) to ({goal_xy[0]:.2f}, {goal_xy[1]:.2f})")
                    outcome = run_one_trial(bag_dir, bag_name, trial_csv, goal_xy, pose_node,
                                            traj_name, pattern_follow_args(pattern["drive"], leg, args.lawnmower_cusp_tolerance >= 0),
                                            None if pattern["drive"] in ZIGZAG_DRIVES else leg.points)
                else:
                    if not run_gp_explorer(gp_explorer_path, bag_dir, traj_name):
                        sys.exit(f"FATAL: gp_explorer_gpu.py failed {GP_EXPLORER_MAX_ATTEMPTS} "
                                 f"times for {traj_name} -- aborting entire run")
                    goal_xy = read_goal_xy(pose_csv)
                    outcome = run_one_trial(bag_dir, bag_name, pose_csv, goal_xy, pose_node,
                                            traj_name)
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
            run_gp_uncertainty(gp_uncertainty_path, bag_dir, bag_name)
            if fs_cfg is not None:
                record_deployed_model(bag_dir, bag_name, fs_cfg)
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
                if fs_cfg["async"] and i > fs_cfg["async_serial_trials"]:
                    finish_background_from_scratch(pose_node, fs_cfg, pending)
                    pending = start_background_from_scratch(fs_cfg, bag_dir, bag_dir / bag_name,
                                                            f"trial{i:02d}")
                else:
                    maybe_train_from_scratch(pose_node, fs_cfg, bag_dir, bag_dir / bag_name,
                                              f"trial{i:02d}")

        # the last trial's iteration: wait for it so the run ends with its checkpoint saved
        finish_background_from_scratch(pose_node, fs_cfg, pending)
        pending = None

        if start_i > 1:
            log(f"all trials complete: reached the {args.n_trajectories}-trial target "
                f"(ran {n_run} new trial(s) this session, {n_success}/{n_run} successful; "
                f"{start_i - 1} already existed from before)")
        else:
            log(f"all {args.n_trajectories} trials complete: "
                f"{n_success}/{args.n_trajectories} successful")
    finally:
        abort_background_from_scratch(pending)
        for pname in (planner_tols if heading_tol_set is not None else ()):
            off = planner_restore[pname]
            try:
                set_controller_param(pose_node, f"{heading_tol_set}.{pname}", off,
                                     server="planner_server")
                log(f"{heading_tol_set}: {pname} back to {str(off).lower() if isinstance(off, bool) else f'{off:g}'}")
            except Exception as e:
                log(f"WARNING: could not reset {heading_tol_set}.{pname} ({e}) -- "
                    f"restart the launch before other runs")
        pose_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
