from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    imu_params = os.path.join(get_package_share_directory('rosey_bringup'),'bringup', 'config', 'um7.yaml')
    # ekf_params = os.path.join(get_package_share_directory('rosey_drivers'),'bringup', 'config', 'ekf.yaml')

    imu_node = Node(
        package="umx_driver",
        executable="um7_driver",
        parameters=[imu_params],
        output='log'
    )


    return LaunchDescription([
        imu_node,
        # robot_localization_node,
    ])
