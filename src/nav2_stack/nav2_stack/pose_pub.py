import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster


import os
import csv
from datetime import datetime

class PosePub(Node):
    def __init__(self):
        super().__init__('pose_pub')

        # parameters
        self.declare_parameter('csv_file', 'sim_data/extracted_data/11172025/11-21-10/pose_data/pose.csv')
        self.declare_parameter('rate', 10.0)

        self.csv_file =self.get_parameter('csv_file').value
        self.rate = self.get_parameter('rate').value

        # publisher
        self.pose_publisher = self.create_publisher(PoseStamped, 'opti_pose', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.waypoint_pub = self.create_publisher(Path, 'sim_waypoints', 10)

        # read in poses, find waypoints
        self.poses = self.read_pose()
        self.publish_waypoints()

        # publish poses
        self.timer = self.create_timer(1.0 / self.rate, self.publish_pose)
        self.pose_idx = 0


    def read_pose(self):
        poses = []
        try:
            with open(self.csv_file, 'r') as f:
                reader = csv.reader(f)
                next(reader)

                for row in reader:

                    pose = PoseStamped()

                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = float(row[2])
                    pose.pose.position.y = float(row[3])
                    pose.pose.position.z = float(row[4])
                    pose.pose.orientation.x = float(row[5])
                    pose.pose.orientation.y = float(row[6])
                    pose.pose.orientation.z = float(row[7])
                    pose.pose.orientation.w = float(row[8])

                    poses.append(pose)

        except Exception as e:
            print(e)


        return poses
    
    def publish_pose(self):
        if self.pose_idx < len(self.poses):
            pose = self.poses[self.pose_idx]
            self.pose_idx += 1

            # update timestamp
            pose.header.stamp = self.get_clock().now().to_msg()
            # publish pose and pose transform
            self.pose_publisher.publish(pose)
            self.map_transform(pose)

    def publish_waypoints(self):
        # create 10 waypoints
        waypoint_num = 10
        idx_skip = int(len(self.poses) / waypoint_num)

        # create path message
        waypoints = Path()
        waypoints.header.stamp = self.get_clock().now().to_msg()
        waypoints.header.frame_id = 'map'

        waypoints.poses.append(self.poses[0])
        prev_idx = 0
        for idx in range(len(self.poses)):
            if idx - prev_idx == idx_skip:
                waypoints.poses.append(self.poses[idx])
                prev_idx = idx

        # publish waypoints as a path message
        self.waypoint_pub.publish(waypoints)

    # publish odom -> map transform
    def map_transform(self, pose):
        trans = TransformStamped()
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = 'map'
        trans.child_frame_id = 'odom'

        trans.transform.translation.x = pose.pose.position.x
        trans.transform.translation.y = pose.pose.position.y
        trans.transform.translation.z = pose.pose.position.z
        trans.transform.rotation = pose.pose.orientation

        self.tf_pub.sendTransform(trans)


def main(args=None):
    rclpy.init(args=args)
    pose_pub = PosePub()
    rclpy.spin(pose_pub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()