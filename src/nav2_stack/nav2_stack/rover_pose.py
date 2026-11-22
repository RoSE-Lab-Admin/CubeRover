#!/usr/bin/env python3
"""Rover-centre pose from the OptiTrack rigid body.

The OptiTrack pose (/FitRosey_passive/pose, TF world -> FitRosey_passive from natnet) is the
Motive rigid body: its origin is the rear-right marker and its axes are rotated a few degrees
against the rover. This node makes the rover frame itself available, from
config/rover_geometry.yaml (scripts/rover_calibration/marker_calibration.py):

  - static TF  FitRosey_passive -> base_link   (Nav2's robot_base_frame is base_link)
  - /rover/pose  PoseStamped in world: the base_link pose (rover centre, true heading), for every
    node that reads the robot pose from a topic (path_follower, safety_watchdog, autonomous_trials,
    the planner's motion seeding, gp_explorer)

Parameters: input_topic (/FitRosey_passive/pose), output_topic (/rover/pose), geometry_file
(default: the package's config/rover_geometry.yaml).
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from scipy.spatial.transform import Rotation as Rot
from tf2_ros import StaticTransformBroadcaster

from nav2_stack import rover_geometry as rg


class RoverPose(Node):
    def __init__(self):
        super().__init__("rover_pose")
        self.declare_parameter("input_topic", "/FitRosey_passive/pose")
        self.declare_parameter("output_topic", "/rover/pose")
        self.declare_parameter("geometry_file", "")
        path = self.get_parameter("geometry_file").value or None
        self.geo = rg.load(path)
        body = self.geo["rigid_body"] or "FitRosey_passive"
        q = Rot.from_matrix(self.geo["R"]).as_quat()

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = body
        tf.child_frame_id = "base_link"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = \
            (float(v) for v in self.geo["t"])
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = (float(v) for v in q)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(tf)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PoseStamped, self.get_parameter("output_topic").value, qos)
        self.create_subscription(PoseStamped, self.get_parameter("input_topic").value, self.on_pose, qos)
        ypr = np.degrees(Rot.from_matrix(self.geo["R"]).as_euler("ZYX"))
        self.get_logger().info(
            f"{body} -> base_link: t {np.round(self.geo['t'], 4).tolist()} m, yaw/pitch/roll "
            f"{np.round(ypr, 2).tolist()} deg; {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}")

    def on_pose(self, msg: PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
        pos, rot = rg.base_link_poses(self.geo, np.array([[p.x, p.y, p.z]]), np.array([[q.x, q.y, q.z, q.w]]))
        qb = Rot.from_matrix(rot[0]).as_quat()
        out = PoseStamped()
        out.header = msg.header
        out.pose.position.x, out.pose.position.y, out.pose.position.z = (float(v) for v in pos[0])
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = \
            (float(v) for v in qb)
        self.pub.publish(out)


def main():
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = RoverPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                    # Ctrl-C / launch shutdown: exit quietly
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
