# CubeRover Technical Documentation

Comprehensive documentation for the RoSE Lab CubeRover (RoSEy) — a ROS2-based differential-drive mobile robot with Gazebo simulation and Nav2 autonomous navigation.

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [System Data Flow](#system-data-flow)
3. [Package Reference](#package-reference)
   - [myrosey_description](#myrosey_description)
   - [rosey_sim](#rosey_sim)
   - [nav2_stack](#nav2_stack)
4. [Node Reference](#node-reference)
5. [ROS2 Topic Reference](#ros2-topic-reference)
6. [Transform Tree](#transform-tree)
7. [Launch File Reference](#launch-file-reference)
8. [Configuration Reference](#configuration-reference)
9. [Arduino Hardware Controller](#arduino-hardware-controller)
10. [Developer Guide](#developer-guide)

---

## Architecture Overview

The system is composed of three ROS2 packages and an Arduino firmware component:

```
┌──────────────────────────────────────────────────────────────────┐
│                        Gazebo Simulator                          │
│  ┌─────────────┐  ┌──────────┐  ┌──────────┐  ┌─────────────┐  │
│  │  DiffDrive   │  │   IMU    │  │  Pose    │  │ Joint State │  │
│  │  Plugin      │  │  Plugin  │  │ Publisher│  │  Publisher  │  │
│  └──────┬───────┘  └────┬─────┘  └────┬─────┘  └──────┬──────┘  │
└─────────┼───────────────┼─────────────┼────────────────┼─────────┘
          │               │             │                │
     ros_gz_bridge (topic bridging)
          │               │             │                │
          ▼               ▼             ▼                ▼
   /odom_raw           /imu       /opti_pose      /joint_states
          │               │             │
          ▼               │             │
  ┌───────────────┐       │             │
  │  covariance   │       │             │
  │    relay      │       │             │
  └───────┬───────┘       │             │
          ▼               ▼             │
   /odom (w/ cov)       /imu            │
          │               │             │
          ▼               ▼             ▼
   ┌──────────────┐  OR  ┌──────────────────┐
   │   EKF Node   │      │  path_follower   │
   │ (robot_loc.) │      │  (opti mode)     │
   └──────┬───────┘      └────────┬─────────┘
          │                       │
          ▼                       ▼
       /odometry/filtered      /odometry/filtered
                    │
                    ▼
           ┌────────────────┐
           │   Nav2 Stack   │
           │  (BT Nav +     │
           │   Controller)  │
           └───────┬────────┘
                   │
                   ▼
              /cmd_vel ──► Gazebo DiffDrive
```

Two localization modes are supported:
- **Opti mode**: Ground truth pose from Gazebo is used directly as localization, bypassing the EKF. This is useful for testing navigation in isolation from localization errors.
- **EKF mode**: The `robot_localization` EKF fuses wheel odometry and IMU data for a realistic localization pipeline.

---

## System Data Flow

### Velocity Command Pipeline

The velocity command follows this path from Nav2 to Gazebo:

1. Nav2 controller server publishes to `/cmd_vel_nav` (TwistStamped)
2. Velocity smoother reads `/cmd_vel_nav`, smooths it, publishes to `/cmd_vel`
3. `ros_gz_bridge` forwards `/cmd_vel` into Gazebo
4. Gazebo DiffDrive plugin applies the velocity to the simulated robot

This remapping is configured in `nav2.launch.py`:
- Controller server: `cmd_vel` remapped to `cmd_vel_nav`
- Velocity smoother: reads `cmd_vel_nav`, writes `cmd_vel_smoothed` remapped to `cmd_vel`

### Odometry Pipeline (EKF Mode)

1. Gazebo DiffDrive plugin publishes raw odometry to `/roseybot_base_controller/odom_raw`
2. `covariance_relay` node injects covariance values and republishes to `/roseybot_base_controller/odom`
3. Gazebo IMU plugin publishes to `/imu`
4. EKF node fuses `/roseybot_base_controller/odom` and `/imu`, publishes filtered state to `/odometry/filtered`
5. EKF broadcasts the `odom -> base_link` transform

### Odometry Pipeline (Opti Mode)

1. Gazebo pose plugin publishes ground truth to `/opti_pose`
2. `path_follower` node receives `/opti_pose` and:
   - Broadcasts the `odom -> base_link` transform from the ground truth pose
   - Publishes ground truth as `/odometry/filtered` (Odometry message)
3. Nav2 reads `/odometry/filtered` for localization

### Waypoint Execution Flow

1. `pose_pub` generates waypoints and publishes to `/sim_waypoints` (TRANSIENT_LOCAL QoS)
2. `path_follower` receives the waypoints via subscription
3. `path_follower` waits for the `bt_navigator` node to activate
4. `path_follower` calls `nav.followWaypoints()` via the Nav2 SimpleCommander API
5. Nav2 plans a global path (NavFn/Dijkstra), then the DWB controller tracks it locally
6. On completion, `path_follower` logs the result and resets for the next run

---

## Package Reference

### myrosey_description

**Type:** ament_cmake

Defines the robot's physical model. The URDF is structured as multiple xacro files composed together.

#### Robot Specifications

| Property | Value |
|---|---|
| Base dimensions | 0.703m x 0.422m x 0.15m |
| Base mass | 12 kg |
| Number of wheels | 4 |
| Wheel radius | 0.1515 m |
| Wheel width | 0.1 m |
| Wheel mass | 0.5 kg each |
| Wheel separation | 0.315 m |
| Friction (mu) | 0.5 |
| Friction (mu2) | 0.2 |

#### URDF File Structure

| File | Purpose |
|---|---|
| `urdf/roseybot.urdf.xacro` | Top-level entry point, includes all other files |
| `urdf/description/roseybot_description.urdf.xacro` | Links, joints, collision geometry, inertia tensors |
| `urdf/description/roseybot_gazebo.urdf.xacro` | Gazebo plugins: DiffDrive, joint state publisher, pose publisher |
| `urdf/description/roseybot_sensors.urdf.xacro` | IMU sensor plugin (100Hz, with noise models) |
| `urdf/description/roseybot_colors.urdf.xacro` | RVIZ material/color definitions |

#### Gazebo Plugins Configured

- **DiffDrive** (`gz::sim::systems::DiffDrive`): Subscribes to `/cmd_vel`, controls left/right wheel pairs
- **JointStatePublisher** (`gz::sim::systems::JointStatePublisher`): Publishes joint positions/velocities
- **PosePublisher** (`gz::sim::systems::PosePublisher`): Publishes ground truth pose at 60Hz
- **IMU** (`gz::sim::systems::Imu`): 100Hz IMU with configurable Gaussian noise

---

### rosey_sim

**Type:** ament_python

Manages the Gazebo simulation environment and the bridge between Gazebo and ROS2.

#### Nodes

**covariance_relay** (`rosey_sim/covariance_relay.py`)

Injects covariance values into raw odometry messages from Gazebo. This is necessary because the Gazebo DiffDrive plugin does not populate covariance fields, but the EKF requires them.

| Parameter | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use simulation clock |

Subscriptions:
- `/roseybot_base_controller/odom_raw` (Odometry) — Raw odometry from Gazebo

Publications:
- `/roseybot_base_controller/odom` (Odometry) — Odometry with covariance injected

Covariance values: 0.001 on all 6 diagonal elements for both pose and twist.

#### Bridged Topics

Configured in `config/ros_gz_bridge.yaml`:

| ROS2 Topic | Message Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | ROS2 -> Gazebo |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo -> ROS2 |
| `/roseybot_base_controller/odom_raw` | `nav_msgs/msg/Odometry` | Gazebo -> ROS2 |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo -> ROS2 |
| `/opti_pose` | `geometry_msgs/msg/PoseStamped` | Gazebo -> ROS2 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo -> ROS2 |

---

### nav2_stack

**Type:** ament_python

Provides autonomous waypoint-following navigation using the Nav2 framework.

#### Nav2 Components Launched

| Component | Plugin/Package | Purpose |
|---|---|---|
| Map Server | `nav2_map_server` | Serves the static occupancy grid map |
| Planner Server | `nav2_navfn_planner::NavfnPlanner` | Global path planning (Dijkstra) |
| Controller Server | `dwb_core::DWBLocalPlanner` | Local trajectory tracking (Dynamic Window Approach) |
| Smoother Server | `nav2_smoother::SimpleSmoother` | Path smoothing |
| Behavior Server | `nav2_behaviors` | Recovery behaviors (spin, backup, wait, drive on heading, assisted teleop) |
| BT Navigator | `nav2_bt_navigator` | Behavior tree-based navigation orchestration |
| Waypoint Follower | `nav2_waypoint_follower` | Sequential waypoint execution |
| Velocity Smoother | `nav2_velocity_smoother` | Smooths velocity commands before sending to robot |
| Lifecycle Manager | `nav2_lifecycle_manager` | Manages lifecycle transitions for all above nodes |

---

## Node Reference

### pose_pub

**Package:** `nav2_stack`
**Source:** `nav2_stack/pose_pub.py`

Generates a trajectory of waypoints and publishes them for the path follower to execute.

**Current behavior:** Generates a square-ish path of 5 waypoints, each 2m apart, turning -90 degrees at each step. Starts at (2, 0) heading east, then turns south, west, north, etc.

**Publications:**

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/sim_waypoints` | `nav_msgs/msg/Path` | TRANSIENT_LOCAL, depth 10 | Waypoint trajectory in `map` frame |

**Parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `csv_file` | string | (path to pose.csv) | Path to CSV file for trajectory data (infrastructure present but not currently used by gen_poses) |
| `num_waypoints` | float | 100.0 | Number of waypoints to use from CSV |
| `use_sim_time` | bool | true | Use simulation clock |

**Behavior:** Publishes the waypoint path once after a 2-second delay (to allow DDS discovery), then cancels the timer.

---

### path_follower

**Package:** `nav2_stack`
**Source:** `nav2_stack/path_follower.py`

Receives waypoints and executes them using the Nav2 SimpleCommander API. In opti mode, also handles ground truth transform broadcasting and odometry publishing.

**Subscriptions:**

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/sim_waypoints` | `nav_msgs/msg/Path` | TRANSIENT_LOCAL, depth 10 | Waypoint trajectory from pose_pub |
| `/opti_pose` | `geometry_msgs/msg/PoseStamped` | Default (depth 10) | Ground truth pose (opti mode only) |

**Publications (opti mode only):**

| Topic | Type | Description |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | Ground truth published as filtered odometry |

**Transforms broadcast (opti mode only):**

| Parent | Child | Description |
|---|---|---|
| `odom` | `base_link` | Robot pose from ground truth |

**Parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `use_opti` | bool | true | Enable optitrack/ground truth mode |
| `use_sim_time` | bool | true | Use simulation clock |

**Execution flow:**
1. Timer fires every 0.5 seconds
2. Waits until waypoints arrive on `/sim_waypoints`
3. Waits for `bt_navigator` to activate
4. Calls `nav.followWaypoints(waypoints)`
5. Polls `nav.isTaskComplete()` until done
6. Logs result (succeeded/canceled), resets state

Uses `MultiThreadedExecutor` with `ReentrantCallbackGroup` so that opti transform callbacks can execute concurrently while `followWaypoints` blocks.

---

### covariance_relay

**Package:** `rosey_sim`
**Source:** `rosey_sim/covariance_relay.py`

Relays odometry messages from Gazebo, injecting covariance values that the DiffDrive plugin does not provide. Required for proper EKF operation.

**Subscriptions:**

| Topic | Type | Description |
|---|---|---|
| `/roseybot_base_controller/odom_raw` | `nav_msgs/msg/Odometry` | Raw odometry from Gazebo |

**Publications:**

| Topic | Type | Description |
|---|---|---|
| `/roseybot_base_controller/odom` | `nav_msgs/msg/Odometry` | Odometry with covariance |

---

## ROS2 Topic Reference

### Input Topics

| Topic | Type | Source | Description |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Nav2 velocity smoother | Velocity command to robot |

### Odometry / Localization Topics

| Topic | Type | Source | Description |
|---|---|---|---|
| `/roseybot_base_controller/odom_raw` | `nav_msgs/msg/Odometry` | Gazebo (via bridge) | Raw wheel odometry without covariance |
| `/roseybot_base_controller/odom` | `nav_msgs/msg/Odometry` | covariance_relay | Wheel odometry with injected covariance |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF or path_follower | Filtered localization estimate used by Nav2 |

### Sensor Topics

| Topic | Type | Source | Description |
|---|---|---|---|
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo (via bridge) | IMU data (orientation, angular vel, linear accel) |
| `/opti_pose` | `geometry_msgs/msg/PoseStamped` | Gazebo (via bridge) | Ground truth pose from simulator |

### Joint State Topics

| Topic | Type | Source | Description |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo (via bridge) | Wheel joint positions and velocities |
| `/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | Gazebo (via bridge) | Dynamic joint state information |

### Navigation Topics

| Topic | Type | Source | Description |
|---|---|---|---|
| `/sim_waypoints` | `nav_msgs/msg/Path` | pose_pub | Waypoint trajectory for the path follower |
| `/cmd_vel_nav` | `geometry_msgs/msg/TwistStamped` | Nav2 controller | Raw nav velocity (before smoothing) |

### Infrastructure Topics

| Topic | Type | Description |
|---|---|---|
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transforms |
| `/robot_description` | `std_msgs/msg/String` | URDF robot description |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation clock from Gazebo |

---

## Transform Tree

```
map
 └── odom              (static identity transform, from rosey_gz.launch.py)
      └── base_link    (from EKF or path_follower opti mode)
           ├── front_left_wheel
           ├── front_right_wheel
           ├── back_left_wheel
           ├── back_right_wheel
           └── imu_link
```

| Transform | Source | Type |
|---|---|---|
| `map -> odom` | `static_transform_publisher` in rosey_gz.launch.py | Static (identity) |
| `odom -> base_link` | EKF node (EKF mode) or path_follower (opti mode) | Dynamic |
| `base_link -> *` | `robot_state_publisher` from URDF | Dynamic (from joint states) |

---

## Launch File Reference

### rosey_gz.launch.py

**Package:** `rosey_sim`

Starts the Gazebo simulation environment with the robot.

**What it launches:**
1. Static transform publisher: `map -> odom` (identity)
2. `covariance_relay` node
3. `robot_state_publisher` (with xacro-processed URDF)
4. Gazebo simulator with `worlds/world.world`
5. Robot spawner (spawns model at z=0.3)
6. `ros_gz_bridge` (configured via `config/ros_gz_bridge.yaml`)

**Usage:**
```bash
ros2 launch rosey_sim rosey_gz.launch.py
```

---

### waypoint.launch.py

**Package:** `nav2_stack`

Launches the waypoint following mission, including Nav2 and all supporting nodes.

**Launch Arguments:**

| Argument | Default | Description |
|---|---|---|
| `use_opti` | `true` | Use ground truth localization (`true`) or EKF (`false`) |

**What it launches:**
1. `pose_pub` node (waypoint generator)
2. `path_follower` node (waypoint executor)
3. `ekf_filter_node` (only when `use_opti:=false`)
4. Full Nav2 stack (via `nav2.launch.py`)

**Usage:**
```bash
# Opti mode (ground truth)
ros2 launch nav2_stack waypoint.launch.py use_opti:=true

# EKF mode
ros2 launch nav2_stack waypoint.launch.py use_opti:=false
```

---

### nav2.launch.py

**Package:** `nav2_stack`

Starts the Nav2 navigation stack. Typically included by `waypoint.launch.py` rather than launched directly.

**What it launches:**
1. Map server (loads from `maps/map.yaml`)
2. Lifecycle manager (auto-starts all managed nodes)
3. Planner server
4. Controller server (remaps `cmd_vel` -> `cmd_vel_nav`)
5. Smoother server
6. Behavior server
7. BT navigator
8. Waypoint follower
9. Velocity smoother (reads `cmd_vel_nav`, writes `cmd_vel`)

---

## Configuration Reference

### nav2_param.yaml

#### Controller (DWB)

| Parameter | Value | Description |
|---|---|---|
| `controller_frequency` | 15.0 Hz | Control loop rate |
| `max_vel_x` | 0.3 m/s | Maximum forward velocity |
| `max_vel_theta` | 0.635 rad/s | Maximum rotational velocity |
| `acc_lim_x` | 0.3 m/s^2 | Linear acceleration limit |
| `acc_lim_theta` | 3.2 rad/s^2 | Angular acceleration limit |
| `xy_goal_tolerance` | 0.1 m | Trajectory tracking tolerance |
| `vx_samples` | 20 | Velocity samples in x |
| `vtheta_samples` | 20 | Velocity samples in theta |

**DWB Critics:**
- `BaseObstacleCritic` (scale: 0.02) — Penalizes trajectories near obstacles
- `GoalDistCritic` (scale: 24.0) — Favors trajectories closer to the goal
- `GoalAlignCritic` (scale: 10.0) — Favors trajectories aligned with goal orientation
- `PathDistCritic` (scale: 32.0) — Favors trajectories close to the global path
- `PathAlignCritic` (scale: 24.0) — Favors trajectories aligned with the path
- `PreferForwardCritic` (scale: 5.0) — Penalizes backward motion

#### Goal Checker

| Parameter | Value | Description |
|---|---|---|
| `xy_goal_tolerance` | 0.25 m | Position tolerance for goal reached |
| `yaw_goal_tolerance` | 0.35 rad | Orientation tolerance for goal reached |

#### Planner (NavFn)

| Parameter | Value | Description |
|---|---|---|
| `tolerance` | 0.5 m | Goal tolerance for planning |
| `use_astar` | false | Uses Dijkstra (not A*) |
| `allow_unknown` | true | Can plan through unknown space |

#### Costmaps

**Local Costmap:**

| Parameter | Value |
|---|---|
| Size | 5m x 5m |
| Resolution | 0.15 m/cell |
| Frame | `odom` |
| Rolling window | true |
| Plugins | inflation_layer (radius 0.1m) |

**Global Costmap:**

| Parameter | Value |
|---|---|
| Resolution | 0.05 m/cell |
| Frame | `map` |
| Plugins | static_layer, inflation_layer (radius 1.75m) |

#### Velocity Smoother

| Parameter | Value |
|---|---|
| Smoothing frequency | 20 Hz |
| Max velocity | [0.3, 0.0, 0.635] (x, y, theta) |
| Max acceleration | [0.3, 0.3, 3.2] |
| Feedback | OPEN_LOOP |

#### Behaviors

Available recovery behaviors: `spin`, `backup`, `drive_on_heading`, `assisted_teleop`, `wait`

---

### ekf.yaml

Extended Kalman Filter configuration for sensor fusion using `robot_localization`.

| Parameter | Value | Description |
|---|---|---|
| `frequency` | 30 Hz | Filter update rate |
| `sensor_timeout` | 0.2 s | Sensor considered stale after this |
| `two_d_mode` | true | Ignores z-axis, operates in 2D |
| `world_frame` | `odom` | World-fixed reference frame |
| `publish_tf` | true | Broadcasts `odom -> base_link` transform |

**Sensor 0: Wheel Odometry** (`/roseybot_base_controller/odom`)

Fused states: x, y, z position; yaw; x, y, z velocity; yaw rate

| Parameter | Value |
|---|---|
| `odom0_differential` | false |
| `odom0_relative` | false |
| `odom0_pose_rejection_threshold` | 5.0 |
| `odom0_twist_rejection_threshold` | 1.0 |

**Sensor 1: IMU** (`/imu`)

Fused states: roll, pitch, yaw orientation; roll, pitch, yaw rate; x, y, z acceleration

| Parameter | Value |
|---|---|
| `imu0_differential` | false |
| `imu0_relative` | true |
| `imu0_remove_gravitational_acceleration` | true |
| `imu0_pose_rejection_threshold` | 0.8 |
| `imu0_twist_rejection_threshold` | 0.8 |

---

### maps/map.yaml

| Parameter | Value | Description |
|---|---|---|
| Image | `map.pgm` | Occupancy grid image |
| Resolution | 0.2 m/pixel | Map resolution |
| Origin | [-5.0, -5.0, 0.0] | Map corner in world frame |
| Occupied threshold | 0.65 | Pixels above this are obstacles |
| Free threshold | 0.25 | Pixels below this are free space |

---

## Arduino Hardware Controller

The `Arduino_ROS/` directory contains firmware for controlling the physical RoSEy rover. This runs on a Teensy/Arduino board connected to two RoboClaw motor controllers.

### Hardware Layout

- **ROBOCLAW_1** (Serial1): Controls front-left (M1) and back-left (M2) motors
- **ROBOCLAW_2** (Serial3): Controls front-right (M1) and back-right (M2) motors
- **RoboClaw baud rate:** 38400
- **Serial (USB) baud rate:** 115200
- **Motor QPPS:** 3400 (quadrature pulses per second at max RPM)

### Serial Command Protocol

Commands are sent over USB serial as ASCII strings terminated by carriage return (`\r`). Format: `<cmd_char> <arg1> <arg2> <arg3>\r`

| Command | Char | Arguments | Description |
|---|---|---|---|
| SET_MOTOR_SPEEDS | `m` | `lSpeed rSpeed` | Set left and right motor pair speeds (QPPS) |
| SET_MOTOR_SPEED | `d` | `motorIndex speed` | Set individual motor speed. Index: 1=FL, 2=BL, 3=FR, 4=BR |
| GET_TELEM | `t` | (none) | Returns telemetry string |
| RESET_ENCODERS | `r` | (none) | Zero all encoder counts |
| PID | `p` | `P I D` | Set velocity PID gains (values divided by 100 internally, stored in EEPROM) |

### Telemetry Response Format

The `GET_TELEM` response is a space-separated string prefixed with `e`:

```
e <FL_enc> <BL_enc> <FR_enc> <BR_enc> <FL_spd> <BL_spd> <FR_spd> <BR_spd> <FL_cur> <BL_cur> <FR_cur> <BR_cur> <RC1_volt> <RC2_volt>\r
```

Fields:
- Positions 0-3: Encoder counts (FL, BL, FR, BR)
- Positions 4-7: Encoder speeds (FL, BL, FR, BR)
- Positions 8-11: Motor currents (FL, BL, FR, BR)
- Positions 12-13: Battery voltages (RoboClaw 1, RoboClaw 2)

### Safety Features

- **Motor timeout:** If no `SET_MOTOR_SPEEDS` or `SET_MOTOR_SPEED` command is received within 2 seconds, all motors are automatically stopped.
- **Acceleration ramping:** The `Wheel::calcAccel()` method computes acceleration based on the velocity delta and time delta since last command, preventing instantaneous velocity jumps.
- **PID persistence:** PID gains are stored in EEPROM and survive power cycles.
- **Read retries:** Encoder, current, and voltage reads retry up to 3 times on failure.

---

## Developer Guide

### Adding New Waypoints

The waypoint trajectory is generated in `src/nav2_stack/nav2_stack/pose_pub.py` in the `gen_poses()` method.

**To modify the hardcoded square trajectory**, edit `gen_poses()`:

```python
def gen_poses(self):
    poses = []
    # Each waypoint is a PoseStamped in the 'map' frame
    pose = PoseStamped()
    pose.header.stamp = self.get_clock().now().to_msg()
    pose.header.frame_id = 'map'
    pose.pose.position.x = 3.0   # target x
    pose.pose.position.y = 1.0   # target y
    # Orientation as quaternion (yaw only for 2D):
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    poses.append(pose)
    return poses
```

**To load waypoints from a CSV file**, the launch file already passes a `csv_file` parameter. You would need to implement the CSV reading logic in `gen_poses()`:

```python
def gen_poses(self):
    self.declare_parameter('csv_file', '')
    csv_path = self.get_parameter('csv_file').value
    poses = []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(row[0])
            pose.pose.position.y = float(row[1])
            # set orientation as needed
            poses.append(pose)
    return poses
```

### Adding a New ROS2 Node

1. Create your node in the appropriate package's Python directory (e.g., `src/nav2_stack/nav2_stack/my_node.py`)

2. Add an entry point in the package's `setup.py`:
   ```python
   entry_points={
       'console_scripts': [
           'my_node = nav2_stack.my_node:main',
       ],
   },
   ```

3. Add the node to a launch file or create a new one:
   ```python
   my_node = Node(
       package='nav2_stack',
       executable='my_node',
       parameters=[{'use_sim_time': True}]
   )
   ```

4. Rebuild:
   ```bash
   colcon build --packages-select nav2_stack
   source install/setup.bash
   ```

### Adding a New Gazebo-ROS Bridge Topic

1. Edit `src/rosey_sim/config/ros_gz_bridge.yaml`

2. Add your topic entry:
   ```yaml
   - topic_name: "/my_topic"
     ros_type_name: "std_msgs/msg/Float64"
     gz_type_name: "gz.msgs.Double"
     direction: GZ_TO_ROS   # or ROS_TO_GZ
   ```

3. Rebuild `rosey_sim`.

### Modifying the Robot Model

Edit the appropriate xacro file under `src/myrosey_description/urdf/description/`:

- Physical dimensions/mass: `roseybot_description.urdf.xacro`
- Gazebo plugins/physics: `roseybot_gazebo.urdf.xacro`
- Sensors: `roseybot_sensors.urdf.xacro`
- Visual colors: `roseybot_colors.urdf.xacro`

After editing, rebuild and relaunch:
```bash
colcon build --packages-select myrosey_description
source install/setup.bash
```

### Tuning Nav2 Parameters

Key parameters to tune are in `src/nav2_stack/config/nav2_param.yaml`:

- **Velocity limits:** `max_vel_x`, `max_vel_theta` in the `FollowPath` section. These should match your robot's capabilities.
- **Goal tolerance:** `xy_goal_tolerance` and `yaw_goal_tolerance` in `general_goal_checker`. Increase if the robot oscillates near goals.
- **DWB critic scales:** Adjust the `scale` values of the critics to change how the local planner weighs path following vs. goal seeking vs. obstacle avoidance.
- **Costmap resolution:** Lower resolution (larger value) runs faster but loses detail. The local costmap at 0.15m is relatively coarse; decrease for tighter obstacle avoidance.
- **Inflation radius:** `inflation_radius` in the costmap layers controls how far from obstacles the robot is penalized. The global costmap uses 1.75m while the local uses 0.1m.

### Tuning EKF Parameters

Edit `src/nav2_stack/config/ekf.yaml`:

- **Process noise covariance:** If a state variable converges too slowly, increase its diagonal value in `process_noise_covariance`. This makes the filter trust measurements more.
- **Sensor config vectors:** The `odom0_config` and `imu0_config` arrays control which state variables each sensor contributes. Order: x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az.
- **Rejection thresholds:** `odom0_pose_rejection_threshold` and similar parameters set the Mahalanobis distance at which measurements are rejected as outliers.

### Building and Running

```bash
# Install dependencies
rosdep install --from-paths src -y --ignore-src

# Build all packages
colcon build

# Build a single package
colcon build --packages-select nav2_stack

# Source the workspace
source install/setup.bash

# Launch simulation
ros2 launch rosey_sim rosey_gz.launch.py

# Launch navigation (in a separate terminal, after sourcing)
ros2 launch nav2_stack waypoint.launch.py use_opti:=true
```

### Useful Debugging Commands

```bash
# List all active topics
ros2 topic list

# Monitor odometry output
ros2 topic echo /odometry/filtered

# Check transform tree
ros2 run tf2_tools view_frames

# Monitor Nav2 node states
ros2 lifecycle list /bt_navigator

# Check if bridge topics are flowing
ros2 topic hz /roseybot_base_controller/odom_raw
ros2 topic hz /imu

# View the robot model
ros2 topic echo /robot_description --once
```
