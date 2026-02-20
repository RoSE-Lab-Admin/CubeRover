from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    imu_params = os.path.join(get_package_share_directory('roseybot_control'),'bringup', 'config', 'bno055.yaml')

    imu_node = Node(
        package="bno055",
        executable="bno055",
        parameters=[imu_params],
        output='log'
    )

    # Static transform: bno055 sensor frame relative to base_link.
    # Update x/y/z/yaw/pitch/roll to match actual IMU mounting position.
    bno055_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='bno055_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'bno055']
    )

    return LaunchDescription([
        imu_node,
        bno055_tf,
    ])
