"""Offline planner test bench: map_server + planner_server only, no robot.

Uses the installed nav2_stack map and nav2_param2.yaml (so the planner config
under test is exactly what the real stack loads), plus identity static TFs
world -> map -> odom -> FitRosey_V1 so the global costmap can activate.

    ros2 launch scripts/planner_offline_test/planner_test.launch.py
    python3 scripts/planner_offline_test/plan_scenarios.py
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('nav2_stack')
    nav2_config = PathJoinSubstitution([share, 'config', 'nav2_param2.yaml'])
    map_config = PathJoinSubstitution([share, 'maps', 'map.yaml'])

    def static_tf(parent, child):
        return Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'{parent}_to_{child}_tf',
            arguments=['--frame-id', parent, '--child-frame-id', child])

    return LaunchDescription([
        static_tf('world', 'map'),
        static_tf('map', 'odom'),
        static_tf('odom', 'FitRosey_V1'),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_config, 'use_sim_time': False}]),
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen',
            parameters=[nav2_config, {'use_sim_time': False,
                                      'robot_base_frame': 'FitRosey_V1'}]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_planner_test', output='screen',
            parameters=[{'use_sim_time': False, 'autostart': True,
                         'node_names': ['map_server', 'planner_server']}]),
    ])
