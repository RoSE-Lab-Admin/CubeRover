from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():

    csv_file = os.path.expanduser('~/CubeRover/sim_data/extracted_data/11172025/11-21-10/pose_data/pose.csv')

    pose_pub_node = Node(
        package='nav2_stack',
        executable='pose_pub',
        parameters=[{
            'csv_file': csv_file,
            'rate': 10.0
        }]
    )

    path_follower_node = Node(
        package='nav2_stack',
        executable='path_follower',
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_stack'),
                'launch',
                'nav2.launch.py'
            ])
        )
    )

    nodes = [
        pose_pub_node,
        path_follower_node
    ]

    return LaunchDescription(nodes + [nav2_launch])

