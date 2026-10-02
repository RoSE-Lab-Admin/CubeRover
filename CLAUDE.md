# CLAUDE.md

Durable, project-level context for any Claude session working in this repo. Keep this
file to facts and conventions that stay true regardless of which specific task or bug is
currently being worked on — investigation-specific state (what's broken right now, where
a debugging session left off) belongs in a one-time handoff doc instead, not here.

## Environment

- Dev-machine checkout and workstation checkout use **different directory names** for the
  same repo/branch — don't assume paths match across machines.
- Workstation: user `roselab@DEM-Workstation`. GPU is an NVIDIA GTX 1650 (**4GB VRAM —
  tight**; watch for CUDA memory pressure if multiple torch/libtorch processes are alive
  at once).
- CUDA/torch work needs the `cuda_end` Python venv active **before** `ros2 launch`, not
  inside it — launched child processes (e.g. `gp_explorer_gpu.py` spawned by
  `autonomous_trials.py`) inherit whatever Python env the parent was started under.
- ROS2 Jazzy. Installed OMPL version is **1.7**
  (`/opt/ros/jazzy/include/ompl-1.7/...`). OMPL's GitHub `main` branch has a newer,
  different API than what's actually installed (confirmed concretely for
  `ReedsSheppStateSpace`: installed has `ReedsSheppPath`/`reedsShepp()`, `main` has
  `PathType`/`getPath()`). **Always read the actually-installed header on the target
  machine before using any OMPL (or similarly version-sensitive vendored dependency) API
  — don't trust upstream docs/GitHub blindly.**
- `ament_python` packages (e.g. `nav2_stack`) need a real `colcon build` to pick up `.py`
  source changes, unless the workspace was built with `--symlink-install`. Don't assume a
  Python edit is live without confirming a rebuild happened.

## Vendored forks in `src/`

Two Nav2 plugin packages are vendored (not system-installed) because they needed
source-level changes. Both are kept as close to upstream as possible otherwise — prefer
minimal, additive changes over restructuring.

- **`nav2_mppi_controller`** — full fork. Contains a fixed, safety-critical bug worth
  protecting against regressing: `mppi::ParametersHandler::getParam(...)` defaults to
  `ParameterType::Dynamic`, which registers a live-reconfigure callback that captures the
  target **by reference**. If that target is a local/stack variable (not a persistent
  class member), the reference dangles the instant the enclosing function returns, and a
  later `ros2 param set` on that name writes through freed memory → segfault. **Any new
  `getParam` call bound to a local variable must pass `ParameterType::Static`
  explicitly**, or this exact crash class will recur. (Params backed by a persistent
  member variable are fine left as `Dynamic`.)

- **`nav2_smac_planner_custom`** — full fork of `nav2_smac_planner`, deliberately given
  its own package name / C++ namespace / pluginlib class name rather than overlaying the
  stock package (unlike the MPPI fork above). This is intentional: it lets stock
  `GridBased` keep being the safe, unmodified default planner, while `GridBasedCustom`
  (registered alongside it in `nav2_param2.yaml`'s `planner_plugins`) is the experimental
  fork, selectable via `planner_id` without needing to rebuild/remove anything. **Convention:
  new or experimental params only ever go on `GridBasedCustom`'s config block, never on
  `GridBased`.** `a_star.cpp` is templated/shared across three node types
  (`Node2D`/`NodeHybrid`/`NodeLattice`) even though only the Hybrid variant is actually
  configured for use — any change there needs to compile for all three.

## ROS2/rclpy pitfalls hit in this project (general, not task-specific)

- **rclpy executor reentrancy is a recurring source of subtle, hard-to-diagnose bugs.**
  If a node is already owned by a persistent `Executor` (e.g. `MultiThreadedExecutor` in
  a `main()`), do not call the bare global `rclpy.spin_until_future_complete(self, ...)`
  or `rclpy.spin_once(self, ...)` from within one of that node's own callbacks — the bare
  global function creates its *own* second executor for the same node when none is
  passed explicitly, which silently starves the node's other callbacks rather than
  raising an error. Calling `self.executor.spin_until_future_complete(...)` instead
  raises `RuntimeError: Executor is already spinning` (rclpy explicitly forbids
  re-entering an executor's own spin from inside its own callback). The safe pattern: spin
  a genuinely separate node that was never added to any persistent executor (e.g. an
  existing `BasicNavigator` instance already held for another purpose), not `self`.
- A `ReentrantCallbackGroup` timer callback can have multiple overlapping invocations
  running concurrently if a slow previous invocation hasn't finished when the timer fires
  again — guard with an explicit boolean/lock if the callback does anything
  non-reentrant-safe (e.g. spins another node).
- `nav2_simple_commander.BasicNavigator`'s private `_waitForNodeToActivate()` has **no
  internal timeout** — it loops forever if a `get_state` service exchange ever fails to
  resolve cleanly. Don't rely on it for anything that needs to recover from a transient
  failure.
- `lifecycle_manager_navigation` reacts to *any* externally-driven state change on a node
  it manages — direct `ros2 lifecycle set` calls on a manager-supervised node race the
  manager's own concurrent recovery. Go through the manager's own
  `nav2_msgs/srv/ManageLifecycleNodes` service (e.g. RESET then STARTUP) instead of
  driving individual node lifecycles directly.
