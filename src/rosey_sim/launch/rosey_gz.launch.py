import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    relay_node = Node(
        package='rosey_sim',
        executable='covariance_relay',
        parameters=[{'use_sim_time': True}]
    )

    # Use roseybot_control's sim URDF — includes ros2_control with GazeboSimSystem
    path_rosey_model = PathJoinSubstitution([
        FindPackageShare('roseybot_control'),
        'description',
        'urdf',
        'roseybot_sim.urdf.xacro'
    ])

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', path_rosey_model
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # robot state publisher (joint states come from joint_state_broadcaster now)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen',
    )

    # launch gazebo world
    world_path = PathJoinSubstitution([
        FindPackageShare('rosey_sim'),
        'worlds',
        'world.world'
    ])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path]
        }.items(),
    )

    # spawn robot (gz_ros2_control plugin inside the URDF starts the controller manager)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'roseybot',
            '-z', '0.3'
        ],
        output='screen'
    )

    # ros-gz bridge (cmd_vel / odom / joint_states are now owned by roseybot_control)
    bridge_config = PathJoinSubstitution([
        FindPackageShare('rosey_sim'),
        'config',
        'ros_gz_bridge.yaml'
    ])

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    # Spawn controllers after Gazebo has time to start the controller manager.
    # The gz_ros2_control plugin creates the controller manager inside Gazebo —
    # these spawners activate the controllers once it is available.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': True}],
    )

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('roseybot_control'),
        'bringup',
        'config',
        'roseybot_controllers.yaml',
    ])

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'roseybot_base_controller',
            '--param-file',
            robot_controllers,
            '--controller-ros-args',
            '-r /roseybot_base_controller/cmd_vel:=/cmd_vel',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # Delay spawners so Gazebo has time to start and the controller manager is ready
    delayed_spawners = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner, robot_controller_spawner]
    )

    return LaunchDescription([
        map_odom_tf,
        relay_node,

        robot_state_pub,
        gz,
        spawn_robot,
        ros_gz_bridge,

        delayed_spawners,
    ])
