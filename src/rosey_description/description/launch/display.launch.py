from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# View the robot model in RViz without ros2_control or hardware.
def generate_launch_description():
    path_to_urdf = PathJoinSubstitution([
            FindPackageShare('rosey_description'),
            'description',
            'urdf',
            'roseybot.urdf.xacro'
    ])

    rviz_config = PathJoinSubstitution([
            FindPackageShare('rosey_description'),
            'description',
            'rviz',
            'roseybot.rviz'
    ])

    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', path_to_urdf]), value_type=str)

    use_gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Use joint_state_publisher_gui (sliders) to move the wheels'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(use_gui),
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
