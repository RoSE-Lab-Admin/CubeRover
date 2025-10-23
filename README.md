# CubeRover
ROS2 Source Build files for the RoSE Lab CubeRover (RoSEy)

## Sim Branch:
used for simulations of Rosey using ros2 control mock hardware.

## Setup:
Before build, you must install ROS dependancies using rosdep. To do this, install rosdep using

```
apt-get install python3-rosdep
```
(this may need root privilage)

then install required ros packages:
```
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

To build, use colcon and source:
```
colcon build
source install/setup.bash
```

## Launch:
To launch RoSEy's hardware controllers, use ros2 launch

```
ros2 launch roseybot_control ros2_control.launch.py
```


## Interfaces:
Rosey uses typical twist messages to be controlled, and state interfaces are published over a few topics

For input interfaces, the primary method of movement is:
```
Topic:
/cmd_vel
Type: geometry_msgs/msg/TwistStamped
```

For output, odometry pose from the hardware controllers is outputted to:
```
/roseybot_base_controller/odom
Type: nav_msgs/msg/Odometry
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



