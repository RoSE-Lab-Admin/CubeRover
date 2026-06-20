from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os

# ---------------------------------------------------------------------------
# rover_type selects which OptiTrack asset publishes the rover pose.
#
#   old_rosey  — original CubeRover hardware  (/CubeRover_V1/pose)
#   fit_rosey  — new FitRosey hardware        (/FitRosey_V1/pose)
#
# Usage:
#   ros2 launch nav2_stack waypoint.launch.py rover_type:=fit_rosey
# ---------------------------------------------------------------------------
ROVER_CONFIGS = {
    'old_rosey': {'opti_topic': '/CubeRover_V1/pose', 'robot_frame': 'CubeRover_V1'},
    'fit_rosey': {'opti_topic': '/FitRosey_V1/pose',  'robot_frame': 'FitRosey_V1'},
}

# csv use:
# ros2 launch nav2_stack waypoint.launch.py pose_csv:=/path/to/pose.csv
def launch_setup(context):
    use_opti   = LaunchConfiguration('use_opti').perform(context)
    pose_csv   = LaunchConfiguration('pose_csv').perform(context)
    rover_type = LaunchConfiguration('rover_type').perform(context)

    if rover_type not in ROVER_CONFIGS:
        raise ValueError(f"Unknown rover_type '{rover_type}'. Choose from: {list(ROVER_CONFIGS)}")

    opti_topic   = ROVER_CONFIGS[rover_type]['opti_topic']
    robot_frame  = ROVER_CONFIGS[rover_type]['robot_frame']
    is_opti      = use_opti.lower() == 'true'

    if not os.path.isabs(pose_csv):
        pkg_path = get_package_share_directory('nav2_stack')
        csv_file = os.path.join(pkg_path, 'pose.csv')
    else:
        csv_file = pose_csv

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
        parameters=[{
            'use_sim_time': True,
            'use_opti':     is_opti,
            'opti_topic':   opti_topic,
            'robot_frame':  robot_frame,
        }]
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_stack'),
                'launch',
                'nav2.launch.py'
            ])
        ),
        launch_arguments={'robot_frame': robot_frame}.items()
    )

    nodes = [pose_pub_node, path_follower_node, nav2_launch]

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
    return LaunchDescription([
        DeclareLaunchArgument('use_opti',    default_value='true'),
        DeclareLaunchArgument('pose_csv',    default_value='pose.csv'),
        DeclareLaunchArgument('rover_type',  default_value='fit_rosey'),
        OpaqueFunction(function=launch_setup),
    ])
