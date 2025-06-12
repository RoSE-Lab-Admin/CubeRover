from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    roverheadlaunchfile = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
        os.path.join(get_package_share_directory('rover_bringup'),
                     'launch/roverhead.launch.xml')
        )
    )
    ld.add_action(roverheadlaunchfile)

    sshprocess = ExecuteProcess(
        cmd=[
            'ssh', 'cubecam@192.168.1.51',
            'bash', '-c',
            'source /opt/ros/jazzy/setup.bash',
            'source ~/CubeRover/install/setup.bash',
            'ros2 launch rover_bringup rovercam.launch.xml',
        ],
        shell = True
    )
    ld.add_action(sshprocess)
    
    return ld