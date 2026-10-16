"""Offline closed-loop MPPI test bench: map_server + planner_server +
controller_server from the installed nav2_param2.yaml, driven by the kinematic
rover_sim.py instead of the real rover and mocap. No bt_navigator: the driver
(mppi_trial.py) replans and sends FollowPath itself.

Run on an isolated DDS domain so nothing reaches the live stack:
    export ROS_DOMAIN_ID=42 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    ros2 launch scripts/mppi_offline_test/mppi_sim.launch.py \
        nn_model_path:=/path/to/mlp128_trialNN.pt dynamics_mode:=neural_network
"""
import os

from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT = 'FitRosey_V1'


def generate_launch_description():
    share = FindPackageShare('nav2_stack')
    nav2_config = LaunchConfiguration('params_file')
    map_config = PathJoinSubstitution([share, 'maps', 'map.yaml'])
    model = LaunchConfiguration('nn_model_path')
    mode = LaunchConfiguration('dynamics_mode')
    yaw_scale = LaunchConfiguration('yaw_scale')

    def static_tf(parent, child):
        return Node(package='tf2_ros', executable='static_transform_publisher',
                    name=f'{parent}_to_{child}_tf',
                    arguments=['--frame-id', parent, '--child-frame-id', child])

    common = {'use_sim_time': False, 'robot_base_frame': ROBOT}
    return LaunchDescription([
        DeclareLaunchArgument(
            'nn_model_path',
            default_value='/home/roselab/AL_scripts/rosbags_10_04_26_test/from_scratch_weights/'
                          'mlp128_trial13.pt'),
        DeclareLaunchArgument('dynamics_mode', default_value='neural_network'),
        DeclareLaunchArgument('yaw_scale', default_value='1.0'),
        DeclareLaunchArgument('rate_limiter', default_value='true'),  # sim base-controller limiter
        DeclareLaunchArgument('nn_use_cuda', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [share, 'config', 'nav2_param2.yaml'])),
        static_tf('world', 'map'),
        static_tf('map', 'odom'),
        ExecuteProcess(
            cmd=['python3', os.path.join(HERE, 'rover_sim.py'), '--ros-args',
                 '-p', ['yaw_scale:=', yaw_scale], '-p', ['rate_limiter:=', LaunchConfiguration('rate_limiter')], '-p', f'frame:={ROBOT}'],
            output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': map_config, 'use_sim_time': False}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[nav2_config, common]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen',
             parameters=[nav2_config, common, {'FollowPath.nn_model_path': model,
                                                'FollowPath.dynamics_mode': mode,
                                                'FollowPath.nn_use_cuda': ParameterValue(
                                                    LaunchConfiguration('nn_use_cuda'), value_type=bool)}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'bond_timeout': 0.0,
                          'node_names': ['map_server', 'planner_server', 'controller_server']}]),
    ])
