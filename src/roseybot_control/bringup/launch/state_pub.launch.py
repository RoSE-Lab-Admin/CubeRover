from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    path_to_urdf = PathJoinSubstitution([
            FindPackageShare('roseybot_control'),
            'description',
            'urdf',
            'roseybot.urdf.xacro'
    ])

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        path_to_urdf
    ])

    robot_description = {'robot_description': robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    nodes = [
        robot_state_pub_node,
    ]

    return LaunchDescription(nodes)