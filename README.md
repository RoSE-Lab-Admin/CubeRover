# CubeRover
ROS Source Build files for the RoSE Lab CubeRover

## Running the rover

---

### Session startup — do this every session

**Step 0 — start the rover Pi** (SSH in a separate terminal):
```bash
ssh rosey@192.168.2.50
ros2 launch roseybot_bringup hardware_startup.launch.py
```

Wait until you see `[roseybot_base_controller] Configured and activated`.

**or**
fastdds discovery --server-id 0
```

**Terminal 1** — start OptiTrack:
```bash
./optitrack.sh
```

**Terminal 2** — start the ROS2 domain bridge:
```bash
python3 ~/CubeRover/ros2_bridge.py
```

The bridge connects the rover Pi (ROS_DOMAIN_ID=1) to the lab machines (ROS_DOMAIN_ID=0). It must be running before Nav2 starts so that `/cmd_vel` is forwarded to the Pi immediately when the controller begins publishing. Once running you will see it auto-discover and print each Pi topic it bridges:

```
[bridge] Main→Pi: /cmd_vel [geometry_msgs/msg/TwistStamped]
[bridge] Pi→Main: /tf  [tf2_msgs/msg/TFMessage]
[bridge] Pi→Main: /joint_states  [sensor_msgs/msg/JointState]
[bridge] Pi→Main: /roseybot_base_controller/odom  [nav_msgs/msg/Odometry]
...
```

Leave this terminal running for the entire session.

---

### Before the first run — record an initial bag

Do a short manual drive while recording a bag so the GP has some training data:

**Terminal 3** — record a bag:
```bash
ros2 bag record /FitRosey_V1/pose /dynamic_joint_states /cmd_vel /roseybot_base_controller/cmd_vel_out /plan /optimal_trajectory /imu/data -o initial_bag
```

Stop recording (`Ctrl+C`) after the drive.

---

### Each exploration run

**Terminal 3** — build and launch Nav2:
```bash
cd CubeRover_waypoints/
source /opt/ros/jazzy/setup.bash
colcon build          # only needed if source files changed
source install/setup.bash
ros2 launch nav2_stack nav2.launch.py
```

Wait for: `[lifecycle_manager] All lifecycle nodes are active`

**Terminal 4** — run the GP explorer to pick the best goal (requires current rover position):
```bash
cd CubeRover_waypoints/
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source ~/gp_env/bin/activate
python3 src/nav2_stack/nav2_stack/gp_explorer.py \
    --bag-dir ~/bag_dir --name traj_01
```
**or**
```bash
cd CubeRover_waypoints/
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source ~/cude_end/bin/activate
python3 src/nav2_stack/nav2_stack/gp_explorer_gpu.py \
    --bag-dir ~/bag_dir --name traj_01
deactivate
```

The script saves the best `(x, y)` into `src/nav2_stack/pose.csv`.

**Terminal 4** (same terminal, after updating pose.csv) — record a new bag and drive:
```bash
ros2 bag record /FitRosey_V1/pose /dynamic_joint_states /cmd_vel /roseybot_base_controller/cmd_vel_out /plan /optimal_trajectory -o bag_01 &
ros2 launch nav2_stack waypoint.launch.py
```

The bag records in the background while the rover drives. Stop the bag after the run (`kill %1` or `fg` then `Ctrl+C`).

Repeat from **Terminal 3** for the next iteration.

---

### Each exploration run — automated (one command, N trials)

The manual per-trial cycle above (gp_explorer -> waypoint+bag -> repeat) is automated by
`autonomous_trials.launch.py`, which also launches Nav2 itself (so it's the only terminal
needed, replacing all 4 above).

**Activate the CUDA env first** (same one `gp_explorer_gpu.py` needs manually, above) —
`autonomous_trials.py` calls `gp_explorer_gpu.py` under whatever Python environment it was
itself launched with, so the env has to be active *before* the `ros2 launch` call, not inside
it. This also covers dynamics-model retraining (below), which needs the same torch/CUDA env:

```bash
cd CubeRover_waypoints/
source /opt/ros/jazzy/setup.bash
colcon build          # only needed if source files changed
source install/setup.bash
source ~/cude_end/bin/activate
ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:=/path/to/bag_dir
deactivate
```

**Flags:**

| Flag | Default | Meaning |
|---|---|---|
| `bag_dir` | *(required)* | Same directory `gp_explorer_gpu.py --bag-dir` and the manual bag-recording steps above use. |
| `n_trajectories` | `25` | Number of trials to run. |
| `retrain_dynamics` | `false` | Retrain the currently-deployed dynamics model (whichever `dynamics_mode`/`nn_hidden_width` is set in `nav2_param2.yaml` — linear or one of the MLP widths) after every trial, on the bags collected so far in `bag_dir`, and redeploy it live for the next trial. No-op (logs a warning, stays disabled) if `dynamics_mode: "kinematics"` — nothing to retrain. |
| `warm_start` | `true` | Continue training from the currently-deployed weights instead of a fresh init. Only affects the MLP case — linear fitting is a single closed-form solve, so there's nothing to warm-start from there. |
| `retrain_subset` | `false` | Train on a random subset of the previously-collected bags + the trajectory just collected (always included), instead of every bag collected so far. |
| `retrain_subset_fraction` | `0.3` | Fraction of the *previously existing* bags to sample when `retrain_subset:=true`. Unused otherwise. |

Example with retraining on:
```bash
ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:=/path/to/bag_dir \
    retrain_dynamics:=true warm_start:=true retrain_subset:=false
```

Other behavior:
- If `bag_dir` has no `initial_bag` yet, one is collected automatically by driving to the
  origin `(0,0)` first — no need to separately do the "Before the first run" step above.
- If `gp_explorer_gpu.py` fails (e.g. a Nav2 action server isn't up yet), it's retried up to
  3 times before the whole run aborts with a clear error.
- If a trial's rover makes no progress toward its goal for 30s, that trial is stopped and
  skipped (bag kept, possibly short) rather than hanging the whole run.
- With `retrain_dynamics:=true`, each trial takes noticeably longer (training time is added
  on top of gp_explorer + drive time), and redeploying briefly cycles `controller_server`'s
  lifecycle between trials — verify this doesn't upset anything on your setup with a short
  1-2 trial run before trusting it for a full unattended batch.
- See `src/nav2_stack/nav2_stack/autonomous_trials.py` and `dynamics_retrain.py` for exact
  behavior — they only sequence/retrain using the same existing scripts and training
  methodology above, no path-planning/following logic is reimplemented.
