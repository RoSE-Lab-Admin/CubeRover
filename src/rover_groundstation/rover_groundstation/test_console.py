import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rover_interfaces.action import TestCommand

import rosbag2_py



class TestConsole(Node):
    def __init__(self):
        super().__init__("testing_console")
        self.action_cli = ActionClient(
            self,
            TestCommand,
            'TestingActionServer',
            self.action_callback
        )

        #set up service client for lidar

        #set up service client for bag nodes
        
        
        lin_vel = float(input("Trial linear speed (cm/s): "))
        turn_rad = float(input("Trial turning radius (cm): (enter anything bigger than 1E6 for straight)"))
        input("Press enter to start trial")

        #start bag

        #send lidar service command

        #wait 10s

        #send rover action command

        #rover stops

        #send lidar service command

        #wait 10s
        
        #stop bag
        




def main(args=None):
    rclpy.init(args=args)
    node = TestConsole()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()