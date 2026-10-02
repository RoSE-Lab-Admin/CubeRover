# Nav2 session handoff

Condensed context from an extended Claude Code session (dev machine, no ROS2 toolchain)
working with a human operator who builds/tests everything on the real rover workstation.
Written so a *different* Claude session — one running directly on the workstation, with
real build/launch/topic access — can pick this up without re-deriving it. If you're that
agent: you have a capability this session didn't (you can actually run `colcon build`,
launch nodes, and inspect live topics yourself) — use it. Don't fall into the same
propose-then-wait-for-a-human-to-paste-logs-back loop unless something is genuinely
destructive or ambiguous.

Delete or move this file once its content has been absorbed — it's a transfer artifact,
not meant to become permanent repo documentation.

## Environment facts

- Branch: `feature/nav2/summer26`.
- Dev machine checkout: `/home/hansm/CubeRover`. Workstation checkout:
  `/home/roselab/CubeRover_waypoint` — **different directory name**, same repo/branch.
- Workstation: user `roselab@DEM-Workstation`, GPU is an NVIDIA GTX 1650 (4GB VRAM —
  tight; watch for CUDA memory pressure with multiple torch/libtorch processes alive at
  once). CUDA/torch work needs the `cuda_end` Python venv active (`source
  ~/cuda_end/bin/activate`) *before* `ros2 launch`, not inside it — `autonomous_trials.py`
  spawns `gp_explorer_gpu.py` under whatever Python env it itself was launched with.
- ROS2 Jazzy, OMPL version installed is **1.7**
  (`/opt/ros/jazzy/include/ompl-1.7/ompl/base/spaces/ReedsSheppStateSpace.h`) — this
  matters: OMPL's GitHub `main` branch has a *different*, newer API
  (`PathType`/`getPath()`) than what's actually installed (`ReedsSheppPath`/
  `reedsShepp()`). Always `find` and `cat` the real installed header before trusting any
  OMPL API reference — this bit us once already.
- `ament_python` packages (`nav2_stack`) need a real `colcon build` to pick up `.py`
  source changes unless the workspace was built with `--symlink-install`. Don't assume a
  `.py` edit is live without confirming a rebuild happened.
- This dev-machine session could never run/build anything — every fix here was static
  code review + the user manually testing and pasting logs/screenshots back. That
  constraint does **not** apply to you if you're running on the workstation.

## What's vendored, and why

Two Nav2 plugin packages are forked into `src/` (not system-installed) because they
needed source-level changes:

- **`src/nav2_mppi_controller/`** — full fork (upstream `1.3.12`). Changes made:
  - Fixed a real dangling-reference/segfault bug: `mppi::ParametersHandler::getParam(...)`
    defaults to `ParameterType::Dynamic`, which registers a live-reconfigure callback that
    captures the target **by reference**. Any `getParam` call whose target is a local/stack
    variable (not a persistent class member) was a dangling reference the instant the
    enclosing function returned — triggered by `ros2 param set` writing through freed
    memory → segfault. Fixed in `optimizer.cpp`'s `getParams()` by passing
    `ParameterType::Static` explicitly for every local-variable-bound `getParam` call
    (9 call sites). This was the most safety-critical fix of the whole session — if
    anyone ever adds a new `getParam` call bound to a local variable without
    `ParameterType::Static`, the same class of segfault will recur.
  - Added a new critic, `DirectionChangeCritic` (`src/critics/direction_change_critic.cpp`
    + header + registered in `critics.xml`/`CMakeLists.txt`/`nav2_param2.yaml`): penalizes
    the MPPI controller for commanding velocity opposite to the robot's **real, measured**
    current direction (`CriticData::state.speed`, not a candidate rollout), with a weight
    that decays exponentially over ~3 seconds since that direction was established (fresh
    start from a stop, or an actual reversal, both reset the clock). Params:
    `cost_weight: 8.0`, `time_constant: 1.0`, `motion_threshold: 0.02` — first guesses,
    **never actually live-tested** (the session moved on to the planner-side multi-reversal
    investigation before this got verified). Worth testing in isolation.

- **`src/nav2_smac_planner_custom/`** — full fork of `nav2_smac_planner` (upstream tag
  `1.3.11`, matched to the installed `ros-jazzy-navigation2` version — check this still
  matches if much time has passed). Deliberately a *separate* package/namespace/plugin
  name from stock `nav2_smac_planner`, not a package-name-collision overlay like the MPPI
  fork — the point was to let stock `GridBased` keep being the safe, unmodified default
  while `GridBasedCustom` (same Hybrid-A* algorithm, this fork) is available opt-in via
  `planner_id` or by reordering `planner_plugins` in `nav2_param2.yaml`. See
  `nav2_param2.yaml`'s `planner_server` section for the exact mechanism/comments.

  Changes made to this fork, in `node_hybrid.hpp`/`.cpp`, `types.hpp`,
  `smac_planner_hybrid.cpp`, `analytic_expansion.cpp`, `a_star.cpp` — all **additive**,
  default values make every new feature a no-op unless explicitly configured in
  `GridBasedCustom`'s YAML block:
  1. **`momentum_zone_length`/`momentum_zone_penalty`** — the rover can't execute a tight
     turn with no momentum (right after a path starts, or right after a direction
     reversal). Tracks `distance_since_momentum_reset` per search node (resets on every
     direction change, accumulates by a fixed per-primitive chord length otherwise);
     penalizes turning primitives that start inside that zone.
  2. **`change_penalty`** (upstream field, was silently `0.0`/unset in our config before
     this session — found by reading the source) — cost added when consecutive primitives
     change `TurnDirection` (curve flip or gear reversal). Now explicitly set.
  3. **`extra_direction_change_penalty`** — steep *escalating* multiplier for the 2nd+
     direction change on a single path (1st change unaffected — `^0`; 2nd gets
     `^1`; 3rd gets `^2`; ...). Tracks a monotonically-increasing
     `direction_change_count` per search node (never resets within a path, unlike the
     momentum-zone distance).
  4. **Analytic-expansion gating** (`analytic_expansion.cpp`) — analytic expansion
     computes a full Reeds-Shepp shortcut in one shot and hands it straight to
     `setAnalyticPath()` on success, **completely bypassing** `getTraversalCost()`'s
     per-primitive costs (all three items above). Fixed by using OMPL's real
     `reedsShepp()` API (see the OMPL-version gotcha above) to get the shortcut's exact
     segment/cusp decomposition, and rejecting shortcuts that would start turning with no
     momentum, or that would push the path's total reversal count above 1.
  5. **A real, confirmed search-correctness bug, now fixed**: `getNeighbors()` used to set
     `distance_since_momentum_reset`/`direction_change_count` on a neighbor node
     speculatively, every time *any* candidate parent reached it — but Hybrid-A* commonly
     reaches the same `(x, y, heading)` state via multiple different parent candidates,
     and only *one* (the cheapest) is accepted via `a_star.cpp`'s
     `if (g_cost < neighbor->getAccumulatedCost())` check. A later, losing candidate's
     `getNeighbors()` call was silently overwriting the fields a winning candidate had
     already committed, completely decoupling them from the actual accepted path. Fixed
     by moving the commit into a new `NodeHybrid::commitDirectionState(parent)`, called
     only from inside that accepted-block in `a_star.cpp` (via `if constexpr
     (std::is_same_v<NodeT, NodeHybrid>)`, since `a_star.cpp` is templated/shared across
     `Node2D`/`NodeHybrid`/`NodeLattice` — the fix must compile for all three even though
     it only does real work for Hybrid).
  6. **`smooth_path`** — enabled (`true`) for a while as a hypothesis for the multi-reversal
     symptom below, then **tested and ruled out** (disabling it live, exact same scenario,
     changed nothing). Don't re-chase the smoother as the explanation.

## UNSOLVED — the actual open problem

Despite all of the above being implemented *and confirmed loaded* (`ros2 param get
/planner_server GridBasedCustom.extra_direction_change_penalty` → `1000.0`) *and* the
real commit-order bug above being fixed and rebuilt, **`GridBasedCustom` still plans
`/plan` paths with 2+ direction changes** (an ugly multi-point-turn) in scenarios where
the human operator is confident, from looking at the costmap himself, that a clean
1-reversal path should be geometrically available. Screenshots were reviewed; this is
confirmed to be the actual `/plan` topic (not an overlapping display, not `/expansions`),
and confirmed to be one continuous path (not a rendering fork).

Ruled out so far: config not reaching runtime, the commit-order bug, the smoother,
overlapping RViz displays, insufficient penalty magnitude (tested up to ~14000x
effective multiplier on a 2nd reversal with zero behavioral change).

**Current leading hypothesis, not yet tested**: the search may never be *considering* a
1-reversal alternative as a candidate at all, in which case no cost tuning could ever
fix it — Hybrid-A* can only choose among paths it actually explores. Two config
parameters control what gets explored, not what it costs, and neither has been touched:
- `coarse_search_resolution: 4` on `GridBasedCustom` — literally skips heading bins during
  the main search phase. The exact heading bin a 1-reversal solution needs may be one of
  the skipped ones.
- `allow_primitive_interpolation` — not set (defaults `false`). When `true`, fills in
  additional intermediate turning radii/angle increments beyond the base 6 primitives.

**Next concrete step, agreed with the user but not yet executed**: set
`coarse_search_resolution: 1` and `allow_primitive_interpolation: true` on
`GridBasedCustom` (config-only, no rebuild) and retest the same scenario. If a clean
path appears, that confirms the discretization theory (and raises a new problem: slower
planning — watch `gp_explorer_gpu.py`'s `[3/5] Planning paths via Nav2 …` timing, and
`max_planning_time: 5.0` for timeouts).

**If that doesn't work**: the user has offered to record a bag of a repro scenario with
`/expansions` included (`debug_visualizations: true` already publishes it) — recording
every candidate the search actually explored would let you check directly whether a
1-reversal candidate was ever explored at all (→ discretization theory) or whether one
was explored and simply lost to the 2-reversal path anyway (→ there's still a cost-function
bug, go back into `getTraversalCost()`/`a_star.cpp` with that certainty instead of
guessing again). This is the most definitive diagnostic available if the config test
above doesn't resolve it — ask for it.

## Other working context (autonomous_trials.py / path_follower.py / pose_pub.py)

Heavy iteration this session on `src/nav2_stack/nav2_stack/`'s orchestration scripts —
not the open problem above, but worth knowing about if issues resurface here:

- **rclpy executor reentrancy is a recurring source of subtle bugs.** Specifically hit,
  in order: (1) `_waitForNodeToActivate` (private method on
  `nav2_simple_commander.BasicNavigator`) has **no internal timeout at all** — loops
  forever on a bad service exchange. (2) Replacing it with a bare
  `rclpy.spin_until_future_complete(self, ...)` from inside a callback on a node already
  owned by an external `MultiThreadedExecutor` silently starves that node's *other*
  callbacks (the bare global function creates its own second executor for the same
  node). (3) Using `self.executor.spin_until_future_complete(...)` instead raises
  `RuntimeError: Executor is already spinning` outright (can't re-enter an executor's own
  spin from within its own callback). (4) The actual fix: spin a genuinely *separate*
  node that was never added to any persistent executor (`path_follower.py` already has
  one: `self.nav`, a `BasicNavigator` instance) — safe because nothing else manages its
  executor. (5) Separately, a `ReentrantCallbackGroup` timer callback can have multiple
  overlapping invocations run concurrently if a previous invocation hasn't finished when
  the timer fires again — needed an explicit boolean guard
  (`self._checking_nav2`) to prevent two overlapping calls from both trying to spin the
  same node at once.
- `lifecycle_manager_navigation` reacts to *any* externally-driven state change on a node
  it manages — direct `ros2 lifecycle set` calls on a manager-supervised node race the
  manager's own concurrent recovery. Must go through the manager's own
  `nav2_msgs/srv/ManageLifecycleNodes` service (RESET then STARTUP) instead; see
  `reload_controller()`/`call_manage_nodes()` in `autonomous_trials.py`.
- `autonomous_trials.py` now: resumes bag numbering from the highest existing `bag_NN` in
  `--bag-dir` instead of restarting at `bag_01` (treats `--n-trajectories` as a *total
  target*, not "N more on top of what exists"); distinguishes a `"no_movement"` outcome
  (zero progress the *entire* 60s stall window, not just no recent improvement) from a
  normal `"stalled"`, and on `"no_movement"` discards the bag and retries the same trial
  number (capped retries) rather than polluting the training set with an empty bag.
- `path_follower.py`'s `/trial_goal_result` publisher (authoritative `TaskResult`) should
  be trusted over re-deriving "reached" from polling the pose topic — the two can
  disagree right at the goal-tolerance boundary.

## General working style this session landed on

- Nothing gets rebuilt/retested by this session directly — always state clearly what
  needs a `colcon build` vs. what's just a YAML change needing a relaunch.
- Default YAML scope discipline: `GridBased` (stock) stays the safe, untouched default;
  new/experimental params go on `GridBasedCustom` only, called out explicitly in comments
  when a param has "no equivalent in stock GridBased."
- When something doesn't work after a fix, get concrete evidence (a `ros2 param get`, a
  screenshot, a specific log line) before guessing again — this session burned real time
  on guesses that turned out wrong (OMPL API from the wrong branch; assuming the smoother
  without testing it in isolation) versus time well spent once asking for a specific
  before/after wasn't anymore.
