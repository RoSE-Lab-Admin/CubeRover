import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
import time
import json
import subprocess
from pathlib import Path
import datetime

from rover_interfaces.action import TestCommand
from rover_interfaces.srv import BagStart
from gantry_lidar_interfaces.srv import Capture, DownloadName, DeleteName
from std_srvs.srv import Trigger

# message types for your debug subscriptions
from rover_interfaces.msg import MotorData
from sensor_msgs.msg import Imu, CompressedImage
from geometry_msgs.msg import PoseStamped

from rover_groundstation.Data_Processor import main as data_process

PAD = 1.0
DURATION = 20.0



class TestConsole(Node):
    def __init__(self, test_num, lin_vel, turn_rad, slope):
        super().__init__("testing_console")

        self.start_time = time.time()
        self.bags_running = False

        # clients
        self.action_cli = ActionClient(self, TestCommand, 'vel_action_server')
        self.gant_capture = self.create_client(Capture, "gantry_capture_service/capture")
        self.gant_download = self.create_client(DownloadName, "gantry_capture_service/download/name")
        self.gant_delete = self.create_client(DeleteName, "gantry_capture_service/delete/name")

        # discover bag services
        self.start_bag_serv = []
        self.stop_bag_serv = []
        self.discover_baggers()

        # debug subscriptions
        self.topics = {
            '/RoverTelem/roboclaw/enc_telem': (MotorData, self.on_motor_msg),
            '/RoverTelem/bno055/imu': (Imu, self.on_imu_msg),
            '/CubeRover_V1/pose': (PoseStamped, self.on_pose_msg),
            '/RoverTelem/camera/image_raw/compressed': (CompressedImage, self.on_cam_msg)
        }

        self.first_msg_received = {t: False for t in self.topics}
        qos = QoSProfile(depth=1)
        for topic, (msg_type, cb) in self.topics.items():
            self.create_subscription(msg_type, topic, cb, qos)

        # user inputs
        turn_rad_name = turn_rad
        if (turn_rad >= 1E6):
            turn_rad_name = 'inf'
    

        self.test_name =  f"Trial_{int(lin_vel)}cm_{turn_rad_name}radius_{slope}slope_Trial{int(test_num)}_{datetime.datetime.now().strftime("%m%d%Y_%H_%M_%S")}"
        self.path = str(Path("/mnt") / "d" / "rosbags" / self.test_name)
        # build goal
        self.goal = TestCommand.Goal()
        self.goal.linear_speed = lin_vel
        self.goal.turning_radius = turn_rad
        self.goal.accel_deaccel_duration = 5000   # ms

        if (slope != 0):
            self.goal.run_duration = 25000            # ms
        else:
            self.goal.run_duration = 40000            # ms

    def discover_baggers(self):
        for name in ["imu_bag", "cam_bag", "motor_bag", "mocap_bag", "overhead_cam_bag"]:
            start_cli = self.create_client(BagStart, f"{name}/start")
            stop_cli  = self.create_client(Trigger,  f"{name}/stop")
            if not start_cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().error(f"{name}/start unavailable")
                continue
            if not stop_cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().error(f"{name}/stop unavailable")
                continue
            self.start_bag_serv.append(start_cli)
            self.stop_bag_serv.append(stop_cli)
            self.get_logger().info(f"Connected to {name} bag services")

    def _log_first(self, topic, label):
        if not self.first_msg_received[topic]:
            delta = time.time() - self.start_time
            self.get_logger().info(f"[{label}] first message at +{delta:.2f}s")
            self.first_msg_received[topic] = True

    def on_motor_msg(self, msg):    self._log_first('/RoverTelem/roboclaw/enc_telem', 'Motor')
    def on_imu_msg(self, msg):      self._log_first('/RoverTelem/bno055/imu', 'IMU')
    def on_pose_msg(self, msg):     self._log_first('/CubeRover_V1/pose',    'Mocap')
    def on_cam_msg(self, msg):      self._log_first('/RoverTelem/camera/image_raw/compressed', 'Camera')

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
        self.bags_running = True

    def stop_bags(self):
        for cli in self.stop_bag_serv:
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut)
            res = fut.result()
            self.get_logger().info(f"Bag stopped successfully. {res.message} messages.")
        self.bags_running = False

    def start_lidar(self):
        req = Capture.Request()
        req.outname = self.test_name + "_lidar"
        req.sensors = ["l515_center"] #, "l515_east", "l515_west"]
        req.duration = DURATION
        self.get_logger().info(f"Starting LiDAR capture ({DURATION})!")
        return self.gant_capture.call_async(req)

    def download_lidar(self, resp):
        out = json.loads(resp.outdata)
        name = out["outname"]
        self.scanname = name
        req = DownloadName.Request()
        req.name = name
        self.get_logger().info(f"Downloading LiDAR data '{name}'!")
        fut = self.gant_download.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        url = json.loads(fut.result().outdata)["url"]
        subprocess.Popen(["wget", "-r", "-P", self.path, url])
        self.get_logger().info("LiDAR download launched.")

    def delete_lidar(self):
        req = DeleteName.Request()
        req.name = self.scanname
        self.gant_delete.call_async(req)
        self.get_logger().info("LiDAR deleted from latte panda")

    def start_rover(self):
        self.get_logger().info("Sending rover drive command!")
        fut = self.action_cli.send_goal_async(self.goal)

        rclpy.spin_until_future_complete(self,fut)
        goal_handle = fut.result()
        self.get_logger().info("Rover goal accepted by server." if goal_handle.accepted else "Rover goal rejected by server.")
        return goal_handle


    def cancel_goal(self, goal):
        self.get_logger().info("Cancelling rover drive command!")
        cancel_fut = goal.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_fut)
        try:
            cancel_result = cancel_fut.result()
        except Exception as e:
            self.get_logger().error(f"Cancel failed: {e}")
            return

        self.get_logger().info(f"Cancel request finished: {cancel_result}")
        return

        


def main(args=None):

    test_num = int(input("Test number to start on: "))
    num_tests = int(input("How many tests to be run: ")) + test_num
    lin_vel = float(input("Trial linear speed (cm/s): "))
    turn_rad = float(input("Trial turning radius (cm) (enter >1E6 for straight, 0 for turn in place): "))
    slope = float(input("input slope value: "))


    for i in range(test_num,num_tests):
        rclpy.init(args=args)
        node = TestConsole(i, lin_vel, turn_rad, slope)        
        input("Press ENTER to begin initial padding and first LiDAR scan...")
        try:
            # â”€â”€â”€ Stage 1: Initial LiDAR â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            node.get_logger().info("=== Stage 1: Initial Padding & First LiDAR Scan ===")
            node.start_bags()
            node.start_time = time.time()
            lidf1 = node.start_lidar()
            time.sleep(DURATION)
            rclpy.spin_until_future_complete(node, lidf1)
            node.download_lidar(lidf1.result())

            # â”€â”€â”€ Stage 3: Drive action â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            t0 = time.time()
            node.get_logger().info("=== Stage 2: Wind-Up & Steady Driving ===")
            drive_fut = node.start_rover()

            input("======================================\n\n\n\n\n\n\n Press ENTER to cancel rover driving \n\n\n\n\n\n\n========================================")
            node.cancel_goal(drive_fut)

            node.get_logger().info("Rover action complete.")
            t1 = time.time()
            node.get_logger().info(f"--- Rover Movement duration: {(t1 - t0):.2f}s ---")
            node.delete_lidar()

            # â”€â”€â”€ Stage 5: Final LiDAR â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            node.get_logger().info("=== Stage 3: Final Padding & Second LiDAR Scan ===")
            lidf2 = node.start_lidar()
            time.sleep(2)
            rclpy.spin_until_future_complete(node, lidf2)
            node.stop_bags()
            data_process(node.path)
            node.download_lidar(lidf2.result())
            t1 = time.time()
            time.sleep(10)
            node.delete_lidar()

            node.get_logger().info(f"ROVER MOVEMENT DURATION: {(t1 - t0):.2f}s")
            node.get_logger().info(f"ROVER MOVEMENT DURATION: {(t1 - t0):.2f}s")
            node.get_logger().info(f"ROVER MOVEMENT DURATION: {(t1 - t0):.2f}s")
            node.get_logger().info(f"ROVER MOVEMENT DURATION: {(t1 - t0):.2f}s")
            node.get_logger().info(f"ROVER MOVEMENT DURATION: {(t1 - t0):.2f}s")




        except Exception as e:
            node.get_logger().error(f"Trial aborted due to error: {e}")

        finally:
            if node.bags_running:
                node.get_logger().info("Cleaning up: stopping residual bag recordingâ€¦")
                try:
                    node.stop_bags()
                except Exception:
                    pass

            node.get_logger().info(f"Trial complete. Bags saved to {node.path}")
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
