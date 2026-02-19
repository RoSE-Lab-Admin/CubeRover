import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster


import math
import csv
from datetime import datetime

class PosePub(Node):
    def __init__(self):
        super().__init__('pose_pub')

        # csv file parameter
        self.declare_parameter('csv_file', 'pose.csv')
        self.csv_file = self.get_parameter('csv_file').value

        # transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.current_odom = None

        # publisher, sends when a subscriber becomes available
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.waypoint_pub = self.create_publisher(Path, '/sim_waypoints', qos)

        # create list of waypoints
        self.poses = self.read_pose()

        # publish after a delay to allow DDS discovery to complete
        self.pub_timer = self.create_timer(2.0, self.publish_waypoints)

    def read_pose(self):
        poses = []
        try:
            with open(self.csv_file, 'r') as f:
                reader = csv.reader(f)
                next(reader)

                for row in reader:
                    pose = PoseStamped()

                    pose.pose.position.x = float(row[0]) 
                    pose.pose.position.y = float(row[1])

                    poses.append(pose)

        except Exception as e:
            self.get_logger().info(f'CSV reading failed: {e}')

        return poses

    def gen_poses(self):
        # initialize list of poses
        poses = []

        # generate a few different pose messages
        corners = [(2.0, 0.0), (2.0, -2.0), (0.0, -2.0), (0.0, 0.0), (2.0, 0.0)]
        for x, y in corners:

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        return poses


    def publish_waypoints(self):
        self.pub_timer.cancel()

        # create path message
        waypoints = Path()
        waypoints.header.stamp = self.get_clock().now().to_msg()
        waypoints.header.frame_id = 'map'

        for idx in range(len(self.poses)):
            waypoints.poses.append(self.poses[idx])

        # publish waypoints as a path message
        self.waypoint_pub.publish(waypoints)


def main(args=None):
    rclpy.init(args=args)
    pose_pub = PosePub()
    rclpy.spin(pose_pub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()