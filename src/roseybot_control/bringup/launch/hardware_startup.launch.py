from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    imu_params = os.path.join(get_package_share_directory('roseybot_control'),'bringup', 'config', 'bno055.yaml')
    ekf_params = os.path.join(get_package_share_directory('roseybot_control'),'bringup', 'config', 'ekf.yaml')

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_params],
    )

    return LaunchDescription([
        robot_localization_node,
    ])
