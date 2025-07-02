import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import json
import subprocess
from pathlib import Path

#interfaces
from rover_interfaces.action import TestCommand
from rover_interfaces.srv import BagStart
from gantry_lidar_interfaces.srv import Capture, DownloadName
from std_srvs.srv import Trigger


PAD = 1
DURATION = 10

class TestConsole(Node):
    def __init__(self):
        super().__init__("testing_console")

        # clients
        self.action_cli = ActionClient(self, TestCommand, 'TestingActionServer')
        self.gant_capture = self.create_client(Capture, "gantry_capture_service/capture")
        self.gant_download = self.create_client(DownloadName, "gantry_capture_service/download/name")

        # discover bag services
        self.start_bag_serv = []
        self.stop_bag_serv = []
        self.discover_baggers()

        # user inputs
        # self.test_name = input("Bag folder name: ")
        # self.path = str(Path.home() / "rosbags" / self.test_name)
        # lin_vel = float(input("Trial linear speed (cm/s): "))
        # turn_rad = float(input("Trial turning radius (cm) (enter >1E6 for straight, 0 for turn in place): "))
        # input("Press ENTER to begin initial padding and first LiDAR scan...")

        # build goal
        self.goal = TestCommand.Goal()
        

    def set_params(self, linvel, turningrad, testname):
        self.goal.linear_speed = linvel
        self.goal.turning_radius = turningrad
        self.goal.accel_deaccel_duration = 5000   # ms
        self.goal.run_duration = 30000            # ms
        self.path = str(Path.home() / "test_outputs" / testname)

    def discover_baggers(self):
        for name in ["imu_bag", "cam_bag", "motor_bag", "mocap_bag"]:
            start_cli = self.create_client(BagStart, f"{name}/start")
            stop_cli  = self.create_client(Trigger,  f"{name}/stop")
            if not start_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f"{name}/start unavailable")
                continue
            if not stop_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f"{name}/stop unavailable")
                continue
            self.start_bag_serv.append(start_cli)
            self.stop_bag_serv.append(stop_cli)
            self.get_logger().info(f"Connected to {name} bag services")

    def start_bags(self):
        req = BagStart.Request()
        req.uri = self.path
        for cli in self.start_bag_serv:
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            try:
                fut.result()  # will raise on error
            except BaseException as e:
                raise e
            self.get_logger().info("Bag started successfully.")

    def stop_bags(self):
        for cli in self.stop_bag_serv:
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut)
            fut.result()
            self.get_logger().info(f"Bag stopped successfully. {fut.message} messages.")

    def start_lidar(self):
        req = Capture.Request()
        req.outname = self.test_name + "_lidar"
        req.sensors = ["l515_center"] #, "l515_east", "l515_west"]
        req.duration = DURATION
        self.get_logger().info("Starting LiDAR capture (10s)!")
        return self.gant_capture.call_async(req)

    def download_lidar(self, resp):
        out = json.loads(resp.outdata)
        name = out["outname"]
        req = DownloadName.Request()
        req.name = name
        self.get_logger().info(f"Downloading LiDAR data '{name}'!")
        fut = self.gant_download.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        url = json.loads(fut.result().outdata)["url"]
        subprocess.Popen(["wget", "-r", "-P", self.path, url])
        self.get_logger().info("LiDAR download launched.")

    def start_rover(self):
        self.get_logger().info("Sending rover drive command!")
        return self.action_cli.send_goal_async(self.goal)
    
    def start_test(self):
        try:
            # â”€â”€â”€ Stage 1: Initial LiDAR â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            t0 = time.time()
            self.get_logger().info("=== Stage 1: Initial Padding & First LiDAR Scan ===")
            lidf1 = self.start_lidar()
            time.sleep(DURATION - PAD)
            rclpy.spin_until_future_complete(self, lidf1)
            self.download_lidar(lidf1.result())
            t1 = time.time()
            self.get_logger().info(f"--- Stage 1 duration: {(t1 - t0):.2f}s ---")


            # â”€â”€â”€ Stage 2: Pre-travel pad + start bags â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            t0 = time.time()
            self.get_logger().info(f"=== Stage 2: Waiting {PAD:.1f}s pre-travel & starting bags ===")
            self.start_time = time.time()
            self.start_bags()
            t1 = time.time()
            self.get_logger().info(f"--- Stage 2 duration: {(t1 - t0):.2f}s ---")

            # â”€â”€â”€ Stage 3: Drive action â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            t0 = time.time()
            self.get_logger().info("=== Stage 3: Wind-Up & Steady Driving ===")
            drive_fut = self.start_rover()
            rclpy.spin_until_future_complete(self, drive_fut)
            goal_handle = drive_fut.result()
            res_fut = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_fut)
            self.get_logger().info("Rover action complete.")
            t1 = time.time()
            self.get_logger().info(f"--- Stage 3 duration: {(t1 - t0):.2f}s ---")

            # â”€â”€â”€ Stage 5: Final LiDAR â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            t0 = time.time()
            self.get_logger().info("=== Stage 5: Final Padding & Second LiDAR Scan ===")
            lidf2 = self.start_lidar()
            time.sleep(PAD)
            self.stop_bags()
            rclpy.spin_until_future_complete(self, lidf2)
            self.download_lidar(lidf2.result())
            t1 = time.time()
            self.get_logger().info(f"--- Stage 5 duration: {t1 - t0:.2f}s ---")

        except Exception as e:
            self.get_logger().error(f"Trial aborted due to error: {e}")

