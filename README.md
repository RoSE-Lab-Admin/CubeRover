# CubeRover
ROS2 Source Build files for the RoSE Lab CubeRover (RoSEy)

## Sim Branch:
used for simulations of Rosey using ros2 control mock hardware.

## Setup:
Before build, you must install ROS dependancies using rosdep. To do this, install rosdep using

'''
apt-get install python3-rosdep
'''
(this may need root privilage)

then install required ros packages:
'''
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
'''

To build, use colcon and source:
'''
colcon build
source install/setup.bash
'''

## Launch:
To launch RoSEy's hardware controllers, use ros2 launch

'''
ros2 launch roseybot_control ros2_control.launch.py
'''


