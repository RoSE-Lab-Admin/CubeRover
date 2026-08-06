# CubeRover
ROS Source Build files for the RoSE Lab CubeRover

## Running the rover

---

### Before the first run — record an initial bag

Do a short manual drive while recording a bag so the GP has some training data:

**Terminal 1** — start OptiTrack:
```bash
./optitrack.sh
```

**Terminal 2** — record a bag:
```bash
ros2 bag record /FitRosey_V1/pose /dynamic_joint_states /cmd_vel /roseybot_base_controller/cmd_vel_out /plan /optimal_trajectory -o initial_bag
```

Stop recording (`Ctrl+C`) after the drive.

---

### Each exploration run

**Terminal 1** — start OptiTrack:
```bash
./optitrack.sh
```

**Terminal 2** — build and launch Nav2:
```bash
cd CubeRover_waypoints/
source /opt/ros/jazzy/setup.bash
colcon build          # only needed if source files changed
source install/setup.bash
ros2 launch nav2_stack nav2.launch.py
```

Wait for: `[lifecycle_manager] All lifecycle nodes are active`

**Terminal 3** — run the GP explorer to pick the best goal (requires current rover position):
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
python3 src/nav2_stack/nav2_stack/gp_explorer_gpu.py \
    --bag-dir ~/bag_dir --name traj_01
deactivate
```

The script prints the best `(x, y)` goal. Copy it into `src/nav2_stack/pose.csv`.

**Terminal 3** (same terminal, after updating pose.csv) — record a new bag and drive:
```bash
ros2 bag record /FitRosey_V1/pose /dynamic_joint_states /cmd_vel /roseybot_base_controller/cmd_vel_out /plan /optimal_trajectory -o bag_01 &
ros2 launch nav2_stack waypoint.launch.py
```

The bag records in the background while the rover drives. Stop the bag after the run (`kill %1` or `fg` then `Ctrl+C`).

Repeat from **Terminal 3** for the next iteration.
