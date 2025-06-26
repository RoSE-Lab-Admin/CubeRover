import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rover_interfaces.action import TestCommand
from rover_interfaces.srv import BagStart
from gantry_lidar_interfaces.srv import Capture, DownloadName, DownloadTimeRange, DeleteName, DeleteTimeRange
from std_srvs.srv import Trigger
import rosbag2_py
from pathlib import Path
import time
import json
import subprocess




class TestConsole(Node):
    def __init__(self):
        super().__init__("testing_console")

        self.action_cli = ActionClient(
            self,
            TestCommand,
            'TestingActionServer',
        )

        #set up service client for lidar
        self.gant_capture = self.create_client(Capture, "gantry_capture_service/capture")
        self.gant_download = self.create_client(DownloadName, "gantry_capture_service/download/name")
        self.gant_delete = self.create_client(DeleteName, "gantry_capture_service/delete/name")

        #set up service client for bag nodes
        self.start_bag_serv = []
        self.stop_bag_serv = []
        self.discover_baggers()


        self.test_name = str(input("Bag folder name: "))
        self.path = str(Path.home() / "rosbags" / self.test_name)
        lin_vel = float(input("Trial linear speed (cm/s): "))
        turn_rad = float(input("Trial turning radius (cm): (enter anything bigger than 1E6 for straight)"))
        input("Press enter to start trial...")

        #send rover action command
        self.goal = TestCommand.Goal()
        self.goal.linear_speed = lin_vel
        self.goal.turning_radius = turn_rad
        self.goal.run_duration = 40000
        self.goal.accel_deaccel_duration = 5000

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

    def start_lidar(self):
        capture_request = Capture.Request()
        capture_request.outname = self.test_name + "_lidar"
        capture_request.sensors = ["l515_east"]
        capture_request.duration = 10.0
        future_cap = self.gant_capture.call_async(capture_request)
        return future_cap
    
    
    def download_lidar(self, lidar_response):
        downloadname_request = DownloadName.Request()
        downloadname_request.name = json.loads(lidar_response.outdata)["outname"]
        future_down = self.gant_download.call_async(downloadname_request)
        rclpy.spin_until_future_complete(self, future=future_down)
        downloadname_response = future_down.result()
        cap_1_url = json.loads(downloadname_response.outdata)["url"]
        subprocess.Popen(["wget", "-r", "-P", f"{self.path}", f"{cap_1_url}"])
        return
    
    def start_rover(self):
        future = self.action_cli.send_goal_async(self.goal)
        return future
    


def main(args=None):
    rclpy.init(args=args)
 
    #start node and init
    node = TestConsole()

    #start bag
    node.start_bags()

    #init lidar scan
    lidar_capture = node.start_lidar()

    #wait for lidar scan
    rclpy.spin_until_future_complete(node, future=lidar_capture)
    lidar_data = lidar_capture.result()

    #download lidar scan
    node.download_lidar(lidar_data)

    #run rover
    rover_command_future = node.start_rover()

    #wait

    rclpy.spin_until_future_complete(node, future=rover_command_future)

    #init 2nd lidar scan
    #lidar_capture = node.start_lidar()
    time.sleep(10)
    #stop bag

    #download lidar scan

    #close ros node
    node.stop_bags()
    node.get_logger().info("trial complete.")
    node.get_logger().info(f"bag saved to {node.path}")

    node.destroy_node()
    rclpy.shutdown()