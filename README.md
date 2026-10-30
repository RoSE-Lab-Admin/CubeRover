# CubeRover
ROS Source Build files for the RoSE Lab CubeRover

## Running the rover

---

### Session startup — do this every session

**Step 0 — start the rover Pi and OptiTrack.**

Rover Pi (SSH in a separate terminal):
```bash
ssh rosey@192.168.2.50
source CubeRover_URDF/install/setup.bash
ros2 launch rosey_bringup hardware_startup.launch.py
```

Wait until you see `[roseybot_base_controller] Configured and activated`.

OptiTrack:
```bash
./optitrack.sh
```

---

### Autonomous trials (one command, N trials)

`autonomous_trials.launch.py` runs the whole exploration loop: it launches Nav2 itself, then for
each trial picks a goal -- `gp_explorer_gpu.py`'s information-based goal by default, or the next
leg of a spiral with `waypoint_mode:=spiral` -- records a bag while the rover drives there, and
(optionally) retrains the dynamics model. It is the only terminal needed besides step 0.

**Activate the CUDA env first** — `colcon build` needs it (the MPPI controller builds against its
PyTorch), and `autonomous_trials.py` calls `gp_explorer_gpu.py` (and the dynamics-model
training) under whatever Python environment it was itself launched with, so the env has to be
active *before* `colcon build` and the `ros2 launch` call, not inside it:

```bash
cd CubeRover_waypoints/
source /opt/ros/jazzy/setup.bash
source ~/cuda_end/bin/activate
colcon build          # only needed if source files changed
source install/setup.bash
ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:=/path/to/bag_dir
deactivate
```

**Flags:**

| Flag | Default | Meaning |
|---|---|---|
| `bag_dir` | *(required)* | Where the bags are recorded and read from (also `gp_explorer_gpu.py --bag-dir`). |
| `n_trajectories` | `25` | Number of trials to run. |
| `retrain_dynamics` | `false` | Retrain the currently-deployed dynamics model (whichever `dynamics_mode`/`nn_hidden_width` is set in `nav2_param2.yaml` — linear or one of the MLP widths) after every trial, on the bags collected so far in `bag_dir`, and redeploy it live for the next trial. No-op (logs a warning, stays disabled) if `dynamics_mode: "kinematics"` — nothing to retrain. |
| `warm_start` | `true` | Continue training from the currently-deployed weights instead of a fresh init. Only affects the MLP case — linear fitting is a single closed-form solve, so there's nothing to warm-start from there. |
| `retrain_subset` | `false` | Train on a random subset of the previously-collected bags + the trajectory just collected (always included), instead of every bag collected so far. |
| `retrain_subset_fraction` | `0.3` | Fraction of the *previously existing* bags to sample when `retrain_subset:=true`. Unused otherwise. |
| `train_from_scratch` | `false` | Runs the first `from_scratch_n_bootstrap` trials under pure kinematics (regardless of what's deployed in `nav2_param2.yaml`), then trains a fresh MLP from a blank init on just that data (including `initial_bag`) and keeps updating it every trial after. Weights are saved per-iteration under `bag_dir/from_scratch_weights/` and pushed to the live controller via `nn_model_path` — the shared deployed model under `nav2_mppi_controller`'s share dir is never read from or written to. Takes over from `retrain_dynamics` if both are set (logs a warning). |
| `from_scratch_n_bootstrap` | `5` | Number of initial kinematics-only trials (not counting `initial_bag`, which is included in the dataset regardless) before the first from-scratch fit. |
| `failure_weighting` | `true` | When retraining, weight samples from failed trials ×2 and no-progress stretches ×3 (capped at ×5) in the MLP loss. |
| `symmetry_augmentation` | `false` | When retraining, also train on mirrored copies of the data (left/right ×0.5, forward/backward ×0.25, both ×0.25; validation stays on real data only). |
| `async_training` | `false` | `train_from_scratch` only (ignored with a warning otherwise): train each iteration in a background process while the next trial is planned and driven, instead of between trials. Trial *i* then drives on the model trained through trial *i−2* (one trial older than serially); if that model isn't finished when the previous trial ends, the run waits for it. Each background training writes its output to `bag_dir/from_scratch_weights/train_trialNN.log`. |
| `async_serial_trials` | `15` | With `async_training:=true`: trials 1 to N still train serially (between trials), because early on training is quick and each new trial changes the model the most. The background training starts at trial N+1. `0` = background from the first trained model on. |
| `terrain_nn` | `false` | With `train_from_scratch` or `retrain_dynamics` (MLP only): train the terrain MLP instead. It also takes the rover's pitch and roll (from OptiTrack) and predicts sideways motion. In MPPI's rollouts, step 0 uses the measured pitch/roll; later steps read them off a GP height map of the arena (refit at every retrain, same GP as `gp_explorer_gpu.py`) at the predicted pose, calibrated to match OptiTrack. Map and calibration are stored inside the model file; the controller recognises a terrain model by itself. Checkpoints are named `mlp<W>_terrain_*` (`from_scratch_weights/`, or `terrain_weights/` with `retrain_dynamics`, where the shared deployed model drives until the first retrain). |
| `terrain_gp` | `false` | With `train_from_scratch`: instead of an MLP, fit kinematics with slip learned from the slope. The GP height map (as for `terrain_nn`) gives the slope under the rover; a second GP maps it (climb along the direction of travel, side slope) to linear and angular slip ratios: v = v_cmd·(1 − s_x), ω = ω_cmd·(1 − s_ω). Refit every trial (~10 s on the CPU), checkpoints `from_scratch_weights/slip_gp_trialNN.pt`. Takes over from `terrain_nn` if both are set. |
| `waypoint_mode` | `gp` | How each trial's goal is chosen. `gp`: `gp_explorer_gpu.py`'s information-based goal. `spiral`: the next leg of a spiral (`nav2_stack/spiral.py`) instead -- it starts on a circle of `spiral_radius` around the spiral centre, grows by 0.5 m per loop until it would come within 0.75 m of a wall, then shrinks back, always counter-clockwise. One bag per leg: inner circle -> outside, or outside -> inner circle (or as far as the rover gets). The initial bag still drives to (0, 0) first. Each trial's points are kept as `bag_dir/traj_NN_spiral.csv`. |
| `spiral_drive` | `waypoints` | `waypoint_mode:=spiral` only. `waypoints`: waypoints every 1 m along the leg, and the goal is kept ~1.5 m ahead of the rover's progress along it, so it is planned by the global planner but the rover never stops in between. `path`: drive to the leg's start, then the leg itself goes straight to the controller (`FollowPath`, no global planner, no replanning). |
| `spiral_radius` | `1.8` | `waypoint_mode:=spiral` only: radius of the inner circle = the tightest turn (m). Larger than ~2.3 m does not fit the current arena with 0.75 m wall clearance. |
| `spiral_centre_x`, `spiral_centre_y` | `0.0` | `waypoint_mode:=spiral` only: the spiral's centre. (-1.0, 0.2) leaves the most room in the current arena. |
| `start_rviz` | `true` | Launches `rviz2` alongside everything else. Set to `false` to skip it. |

Example with retraining on:
```bash
ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:=/path/to/bag_dir \
    retrain_dynamics:=true warm_start:=true retrain_subset:=false
```

Example with train-from-scratch on:
```bash
ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:=/path/to/bag_dir \
    train_from_scratch:=true from_scratch_n_bootstrap:=5
```

Example with train-from-scratch and asynchronous training (serial for the first 15 trials):
```bash
ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:=/path/to/bag_dir \
    train_from_scratch:=true async_training:=true
```

Example with train-from-scratch and the terrain model (also works with `async_training:=true`):
```bash
ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:=/path/to/new_bag_dir \
    train_from_scratch:=true terrain_nn:=true
```
- Use a new `bag_dir`: terrain checkpoints (`from_scratch_weights/mlp<W>_terrain_trialNN.pt`) are
  never resumed from plain ones, so a directory with a plain from-scratch run would start over
  from kinematics. To train on bags from earlier sessions too (e.g. the hills bags), copy them
  into `bag_dir` before launching; they then also go into the GP height map, so only copy bags
  recorded on the same terrain.
- Every training prints `terrain: + pitch, roll inputs ...` and the GP -> OptiTrack calibration,
  e.g. `pitch = 1.06 * pitch_gp + 0.03 * roll_gp -4.55 deg (residual rms 2.0 deg)`. A residual
  well above ~3 deg means the GP map fits the terrain poorly.
- After each new model, `controller_server` logs `NNDynamics: ... terrain=yes`.

Example with spiral waypoints instead of the GP explorer (any training flags can be added, e.g.
`train_from_scratch:=true terrain_gp:=true`):
```bash
ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:=/path/to/new_bag_dir \
    waypoint_mode:=spiral spiral_radius:=1.8 spiral_drive:=waypoints
```
- Radii worth trying: 1.5, 1.8, 2.0 m. Legs are ~4-9 m around (0, 0) and up to ~19 m around
  `spiral_centre_x:=-1.0 spiral_centre_y:=0.2`.
- Offline (kinematic simulator) so far only: `spiral_drive:=waypoints` often ends up driving the
  spiral in reverse (the start heading is arbitrary and the planner keeps the gear it is moving
  in); `spiral_drive:=path` follows it forward, but cuts inside the spiral unless the controller
  has a `PathAlignCritic`, which `nav2_param2.yaml` does not have yet.

Example with train-from-scratch and the slope-slip GP model (same notes as above; each fit prints
the slip it learned, e.g. `slip GP linear: ... GP at level +0.21 / climb +10 deg +0.36 / ...`):
```bash
ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:=/path/to/new_bag_dir \
    train_from_scratch:=true terrain_gp:=true
```

Other behavior:
- If `bag_dir` has no `initial_bag` yet, one is collected automatically by driving to the
  origin `(0,0)` first — no need to separately do the "Before the first run" step below.
- Re-running with an existing `bag_dir` resumes: numbering continues after the last `bag_NN`,
  and `train_from_scratch` continues from the newest checkpoint in `from_scratch_weights/`.
- If `gp_explorer_gpu.py` fails (e.g. a Nav2 action server isn't up yet), it's retried up to
  3 times before the whole run aborts with a clear error (`waypoint_mode:=gp`).
- If a trial's rover makes no progress toward its goal for 60 s, that trial is stopped and
  recorded as stalled (bag kept, possibly short) rather than hanging the whole run. With
  `waypoint_mode:=spiral`, progress is the distance still to go *along the leg*, not the
  straight-line distance to its end (which does not shrink steadily on a spiral).
- A trial that recorded no movement at all is discarded and retried (up to 3 times).
- Each trial bag records `/FitRosey_passive/pose`, `/dynamic_joint_states`, `/cmd_vel`,
  `/roseybot_base_controller/cmd_vel_out`, `/plan`, `/optimal_trajectory`,
  `/optimal_trajectory_model`, `/imu/data` and `/imu/rpy`. Outcomes go to
  `bag_dir/trial_outcomes.csv`; with `train_from_scratch`, `bag_dir/deployed_models.csv`
  records which model drove each bag.
- With retraining on (and `async_training` off), each trial takes noticeably longer (training
  time is added on top of gp_explorer + drive time), and redeploying briefly cycles
  `controller_server`'s lifecycle between trials — verify this doesn't upset anything on your
  setup with a short 1-2 trial run before trusting it for a full unattended batch.
- See `src/nav2_stack/nav2_stack/autonomous_trials.py`, `dynamics_retrain.py` and `spiral.py` for
  exact behavior. Goals are driven by the same `waypoint.launch.py` / `path_follower.py` as the
  manual steps below; only `spiral_drive:=path` bypasses the global planner (the leg goes straight
  to the controller via `FollowPath`).

---

### Manual runs (separate terminals)

The same loop as above, one step at a time. Do step 0 first.

#### Before the first run — record an initial bag

Do a short manual drive while recording a bag so the GP has some training data:

**Terminal 1** — record a bag:
```bash
ros2 bag record /FitRosey_passive/pose /dynamic_joint_states /cmd_vel /roseybot_base_controller/cmd_vel_out /plan /optimal_trajectory /optimal_trajectory_model /imu/data /imu/rpy -o initial_bag
```

Stop recording (`Ctrl+C`) after the drive.

`/optimal_trajectory` is MPPI's chosen command sequence drawn as if the rover
followed the commands exactly; `/optimal_trajectory_model` is the same sequence
as predicted by the dynamics model selected by `dynamics_mode` (kinematics,
linear or neural network) -- i.e. what the controller believes will happen.
They are identical in kinematics mode.

#### Each exploration run

**Terminal 1** — build and launch Nav2 (the CUDA env has to be active for `colcon build`):
```bash
cd CubeRover_waypoints/
source /opt/ros/jazzy/setup.bash
source ~/cuda_end/bin/activate
colcon build          # only needed if source files changed
source install/setup.bash
ros2 launch nav2_stack nav2.launch.py
```

Wait for: `[lifecycle_manager] All lifecycle nodes are active`

**Terminal 2** — run the GP explorer to pick the best goal (requires current rover position):
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
source ~/cuda_end/bin/activate
python3 src/nav2_stack/nav2_stack/gp_explorer_gpu.py \
    --bag-dir ~/bag_dir --name traj_01
deactivate
```

The script saves the best `(x, y)` into `src/nav2_stack/pose.csv`.

**Terminal 2** (same terminal, after updating pose.csv) — record a new bag and drive:
```bash
ros2 bag record /FitRosey_passive/pose /dynamic_joint_states /cmd_vel /roseybot_base_controller/cmd_vel_out /plan /optimal_trajectory /optimal_trajectory_model /imu/data /imu/rpy -o bag_01 &
ros2 launch nav2_stack waypoint.launch.py
```

The bag records in the background while the rover drives. Stop the bag after the run (`kill %1` or `fg` then `Ctrl+C`).

Repeat from **Terminal 2** for the next iteration.
