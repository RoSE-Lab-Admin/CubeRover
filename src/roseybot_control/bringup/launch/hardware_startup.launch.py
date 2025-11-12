from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    #launch file for imu
    imu_launch = os.path.join(get_package_share_directory('roseybot_control'),'bringup', 'launch', 'imu_boot.launch.py')

    #launch file for control
    control_launch = os.path.join(get_package_share_directory('roseybot_control'),'bringup', 'launch', 'ros2_control.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch)
        )
    ])