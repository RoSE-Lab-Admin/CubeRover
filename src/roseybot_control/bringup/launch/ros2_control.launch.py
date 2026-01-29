from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # static transform for simple open loop implementation, change later for closed loop
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', 
                   '--roll', '0', '--pitch', '0', '--yaw', '0', 
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': False}]
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('roseybot_control'),
                'bringup',
                'launch',
                'rviz.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'robot_name': 'roseybot'
        }.items()
    )

    path_to_urdf = PathJoinSubstitution([
            FindPackageShare('roseybot_control'),
            'description',
            'urdf',
            'roseybot.urdf.xacro'
    ])

    gen_urdf_path = "/tmp/roseybot.urdf"

    gen_urdf = ExecuteProcess(
        cmd=[
            FindExecutable(name="xacro"),
            path_to_urdf,
            "-o",
            gen_urdf_path
        ],
        output="screen",
    )

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        path_to_urdf
    ])

    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("roseybot_control"),
            "bringup",
            "config",
            "roseybot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "roseybot_base_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            "-r /roseybot_base_controller/cmd_vel:=/cmd_vel",
        ],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        map_odom_tf
    ]

    return LaunchDescription(nodes + [gen_urdf, rviz_launch])