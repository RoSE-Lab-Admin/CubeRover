from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('roseybot_control'),'bringup', 'config')

    bno055_params = os.path.join(config_dir, 'bno055.yaml')
    um7_params = os.path.join(config_dir, 'um7.yaml')

    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='rosey',
        description='Which Robot to Launch: rosey or flat_rosey'
    )

    robot = LaunchConfiguration('robot')

    bno055_node = Node(
        package="bno055",
        executable="bno055",
        parameters=[bno055_params],
        output='log',
        condition=IfCondition(PythonExpression(["'", robot, "' == 'rosey'"]))
    )
    um7_node = Node(
        package="umx_driver", #umx_driver
        executable="um7_driver", #?? umx_driver_node verify with ros2 package executables umx driver
        parameters=[um7_params],
        output='log',
        condition=IfCondition(PythonExpression(["'", robot, "' == 'flat_rosey'"]))

    )

    # Static transform: bno055 sensor frame relative to base_link.
    # Update x/y/z/yaw/pitch/roll to match actual IMU mounting position.
    bno055_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'bno055'],
        condition=IfCondition(PythonExpression(["'", robot, "' == 'rosey'"]))
    )
    um7_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'um7'],
        condition=IfCondition(PythonExpression(["'", robot, "' == 'flat_rosey'"]))
    )

    return LaunchDescription([
        robot_arg,
        bno055_node,
        bno055_tf,
        um7_node,
        um7_tf,
    ])
