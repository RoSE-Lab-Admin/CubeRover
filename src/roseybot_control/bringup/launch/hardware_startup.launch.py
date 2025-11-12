import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#writing-launch-files

def generate_launch_description():
    #launch file path for imu
    imu_launch = os.path.join(get_package_share_directory('roseybot_control'),'bringup', 'launch', 'imu_boot.launch.py')

    #launch file path for control
    control_launch = os.path.join(get_package_share_directory('roseybot_control'),'bringup', 'launch', 'ros2_control.launch.py')

    imu = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch)
        )

    control  = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch)
        )
    return LaunchDescription([
        imu,
        control
    ])