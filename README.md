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
ros2 bag record /FitRosey_V1/pose /dynamic_joint_states /cmd_vel /roseybot_base_controller/cmd_vel_out /plan /optimal_trajectory -o initial_bag
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
python3 src/nav2_stack/nav2_stack/gp_explorer_gpu.py \
    --bag-dir ~/bag_dir --name traj_01
deactivate
```

The script prints the best `(x, y)` goal. Copy it into `src/nav2_stack/pose.csv`.

**Terminal 4** (same terminal, after updating pose.csv) — record a new bag and drive:
```bash
ros2 bag record /FitRosey_V1/pose /dynamic_joint_states /cmd_vel /roseybot_base_controller/cmd_vel_out /plan /optimal_trajectory -o bag_01 &
ros2 launch nav2_stack waypoint.launch.py
```

The bag records in the background while the rover drives. Stop the bag after the run (`kill %1` or `fg` then `Ctrl+C`).

Repeat from **Terminal 3** for the next iteration.
