import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rover_interfaces.action import TestCommand
from rover_interfaces.srv import BagStart
from std_srvs.srv import Trigger
import rosbag2_py
from pathlib import Path
import time



class TestConsole(Node):
    def __init__(self):
        super().__init__("testing_console")
        self.action_cli = ActionClient(
            self,
            TestCommand,
            'TestingActionServer',
        )

        #set up service client for lidar
        self.capture = self.create_client()

        #set up service client for bag nodes
        self.start_bag_serv = []
        self.stop_bag_serv = []
        self.discover_baggers()


        bag_name = str(input("Bag folder name: "))
        self.path = str(Path.home() / "rosbags" / bag_name)
        lin_vel = float(input("Trial linear speed (cm/s): "))
        turn_rad = float(input("Trial turning radius (cm): (enter anything bigger than 1E6 for straight)"))
        input("Press enter to start trial...")

        #start bag
        self.start_bags()

        #send lidar service command [todo]


        #wait 10s
        time.sleep(10)

        #send rover action command
        goal = TestCommand.Goal()
        goal.linear_speed = lin_vel
        goal.turning_radius = turn_rad
        goal.run_duration = 40000
        goal.accel_deaccel_duration = 5000
        future = self.action_cli.send_goal_async(goal)

        #rover stops
        rclpy.spin_until_future_complete(self, future=future)
        
        #send lidar service command [todo]

        #wait 10s
        time.sleep(10)

        #stop bag
        self.stop_bags()
        self.get_logger().info("trial complete.")
        self.get_logger().info(f"bag saved to {self.path}")
        self.destroy_node()

    def discover_baggers(self):
        
        all_nodes = self.get_node_names()
        print(all_nodes)
        for name in all_nodes:
            if "bag" in name:
                start_bag = self.create_client(BagStart, f"{name}/start")
                self.get_logger().info(f"service name:{name}/start")
                stop_bag = self.create_client(Trigger, f"{name}/stop")
                self.get_logger().info(f"service name:{name}/stop")
                if not start_bag.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error(f"{name} start service not connected")
                    continue

                if not stop_bag.wait_for_service(timeout_sec=2.0):
                    self.get_logger().error(f"{name} stop service not connected")
                    continue

                self.start_bag_serv.append(start_bag)
                self.stop_bag_serv.append(stop_bag)

                self.get_logger().info(f"connected to bag services on {name}")
    
    def start_bags(self):
        request = BagStart.Request()
        request.uri = self.path
        for client in self.start_bag_serv:
            client.call_async(request)
            self.get_logger().info(f"started bagger")
        return
    
    def stop_bags(self):
        for client in self.stop_bag_serv:
            client.call_async(Trigger.Request())
            self.get_logger().info(f"stopped bagger")
        




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