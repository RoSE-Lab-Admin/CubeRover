import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
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

    ekf_params = os.path.join(get_package_share_directory('rosey_sim'), 'config', 'ekf.yaml')

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}],
    )

    relay_node = Node(
        package='rosey_sim',
        executable='covariance_relay',
        parameters=[{'use_sim_time':True}]
    )

    # turn rosey model xacro into urdf
    path_rosey_model = PathJoinSubstitution([
        FindPackageShare('myrosey_description'),
        'urdf',
        'roseybot.urdf.xacro'
    ])

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', path_rosey_model
    ])

    robot_description = {'robot_description': robot_description_content}

    # robot state publisher
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
        #launch_arguments={'gz_args':['-r',world_path]}.items(),
        launch_arguments={
            'gz_args': ['-r ', world_path]
        }.items(),
    )

    # spawn robot
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

    # ros-gz bridge
    bridge_config = PathJoinSubstitution([
        FindPackageShare('rosey_sim'),
        'config',
        'ros_gz_bridge.yaml'
    ])

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time':True}],
        output='screen',
    )

    return LaunchDescription([
        map_odom_tf,
        relay_node,
        robot_localization_node,
        robot_state_pub,
        gz,
        spawn_robot,
        ros_gz_bridge
    ])

