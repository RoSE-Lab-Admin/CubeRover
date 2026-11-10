from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


# Online EKF fusing /roseybot_base_controller/odom and /imu/data.
# Output: /odometry/filtered (nav_msgs/Odometry) and the odom -> base_footprint TF.
def generate_launch_description():

    ekf_params = os.path.join(get_package_share_directory('rosey_bringup'), 'bringup', 'config', 'ekf.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params],
    )

    return LaunchDescription([
        ekf_node,
    ])
