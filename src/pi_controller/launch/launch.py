from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pi_controller',
            executable='Serial',
            name='serial_bridge',
            parameters=[{
                'COMPORT': '/dev/ttyAMA0',
                'BAUD': 38400
            }]
        ),
        Node(
            package='pi_controller',
            executable='Command',
            name='motor_command'
        )
    ])