from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context):
    use_opti = LaunchConfiguration('use_opti').perform(context)

    is_opti = use_opti.lower() == 'true'

    csv_file = os.path.expanduser('~/CubeRover/sim_data/extracted_data/11172025/11-21-10/pose_data/pose.csv')

    pose_pub_node = Node(
        package='nav2_stack',
        executable='pose_pub',
        parameters=[{
            'csv_file': csv_file,
            'num_waypoints': 100.0,
            'use_sim_time': True
        }]
    )

    path_follower_node = Node(
        package='nav2_stack',
        executable='path_follower',
        parameters=[{'use_sim_time': True},
                    {'use_opti': is_opti}]
    )

    nodes = [pose_pub_node, path_follower_node]

    # only run EKF when not using ground truth
    if not is_opti:
        ekf_params = os.path.join(
            get_package_share_directory('nav2_stack'), 'config', 'ekf.yaml'
        )
        robot_localization_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params, {'use_sim_time': True}],
        )
        nodes.append(robot_localization_node)

    return nodes


def generate_launch_description():

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_stack'),
                'launch',
                'nav2.launch.py'
            ])
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_opti', default_value='true'),
        OpaqueFunction(function=launch_setup),
        nav2_launch,
    ])

