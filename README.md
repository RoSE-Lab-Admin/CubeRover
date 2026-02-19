# CubeRover
ROS2 Source Build files for the RoSE Lab CubeRover (RoSEy)

## Sim Branch:
Gazebo simulation of differential drive rover running NAV2 waypoint following

## Setup:
Before build, you must install ROS dependancies using rosdep. To do this, install rosdep using

```bash
apt-get install python3-rosdep
```
(this may need root privilage)

then install required ros packages:
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

To build, use colcon and source:
```bash
colcon build
source install/setup.bash
```

## Launch:
To launch the waypoint follower run the following:

```bash
ros2 launch rosey_sim rosey_gz.launch.py
ros2 launch nav2_stack waypoint.launch.py use_opti:=true
```
Or with EKF localization:
```bash
ros2 launch nav2_stack waypoint.launch.py use_opti:=false
```

## Packages

### myrosey_description
Defines the robot's physical model using URDF/Xacro files
- **Robot Dimensions:** 0.703m x 0.422m x 0.15m
- **Drivetrain:** 4-wheel differential drive, wheel radius 0.1515m, separation 0.315m
- **Sensors:** IMU at 100Hz with Gaussian noise models
- **Gazebo Plugins:** DiffDrive controller, joint state publisher, pose publisher at 60Hz

Key Files:
- `urdf/roseybot.urdf.xacro`: Main URDF entry point
- `urdf/description/roseybot_description.urdf.xacro`: Physical structure
- `urdf/description/roseybot_gazebo.urdf.xacro`: Gazebo plugins and physics properties
- `urdf/description/roseybot_sensors.urdf.xacro`: Sensor defintions

### rosey_sim
Manages Gazebo simulation environment and ROS-GZ topic bridging
**Launch File:** `rosey_gz.launch.py
- Spawns robot in Gazebo
- Starts `robot_state_publisher` for the TF tree
- creates a static `map -> odom` transform
- Runs `ros_gz_bridge` to bridge topics between Gazebo and ROS2
- Launches `coviarance_relay` node 

**Nodes**
- `covariance_relay`: Subscribes to raw odometry from Gazebo (`/roseybot_base_controller/odom_raw`), injects diagonal covariance (0.001), and republishes to `/roseybot_base_controller/odom`. Needed for proper function of EKF node.

**Bridged Topics**
Bridged topics are configured in `config/ros_gz_bridge.yaml`. All topics are bridging from Gazebo to ROS2, except for `/cmd_vel` which goes from ROS2 to Gazebo.
- `/cmd_vel` -> TwistStamped
- `/imu` -> Imu
- `/roseybot_base_controller/odom_raw` -> Odometry
- `/joint_states` -> JointState
- `/CubeRover_V1/pose` -> PoseStamped
- `/clock` -> Clock

### nav2_stack
Provides waypoint following functionality using NAV2

**Launch Files:**
- `nav2.launch.py`: Starts NAV2
- `waypoint.launch.py`: Launches the nav2 launch and the `pose_pub` and `path_follower` nodes. Supports two modes:
    - **Optitrack Mode** (`use_opti:=true`, default): Uses Gazebo ground truth to publish pose for localization. Broadcasts `odom->base_link` and publishes ground truth as `odometry/filtered`.
    - **EKF Mode** (`use-opti:=false`): Runs `robot_localization` EKF node that fuses wheel odometry and IMU data and publishes to `/odometry/filtered`

**Nodes:**
- `pose_pub`: Generates a path for the rover to follow. Publishes to `/sim_waypoints`
- `path_follower`: Subscribes to `/sim_waypoints` and runs NAV2 commands. In optitrack mode, it also broadcasts ground truth transforms and odometry.

**COnfiguration:**
- `config/nav2_param.yaml`: Nav2 paramters 
- `config/ekf.yaml`: EKF configuration fusing wheel odometry and IMU data
- `maps/map.yaml`: Static map configuration


## Interfaces:
Rosey uses typical twist messages to be controlled, and state interfaces are published over a few topics

For input interfaces, the primary method of movement is the ros topic:
```
/cmd_vel
Type: geometry_msgs/msg/TwistStamped
```

The following topics are output:
```
/imu
Type: sensor_msgs/msg/Imu
Description: Simulated IMU sensor data

/roseybot_base_controller/odom
Type: nav_msgs/msg/Odometry
Decsription: Wheel odometry data with covariance

/odometry/filtered
Type: nav_msgs/msg/Odometry
Description: Filtered from roseybot_base_controller/odom and /imu using EKF

/CubeRover_V1/pose
Type: geometry_msgs/msg/PoseStamped
Description: Ground truth pose from Gazebo

/sim_waypoints
Type: nav_msgs/msg/Path
Description: Published waypoint trajectory
```

Joint states are outputted to:
```
/dynamic_joint_states
Type: control_msgs/msg/DynamicJointState

/joint_states
Type: sensor_msgs/msg/JointState
```

Transforms and robot description info can be found in:
```
/tf
/tf_static
/robot_description
```

## Transforms
`map -> odom`
Description: Static identity transform

`odom -> base_link`
Description: Robot pose sourced from EKF or Gz ground truth

`base_link -> *`
Description: Wheel and sensor frames from URDF





