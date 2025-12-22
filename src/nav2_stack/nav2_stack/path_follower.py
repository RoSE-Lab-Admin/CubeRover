import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import math


class PathFollower(Node):
    def __init__(self):

        super().__init__('path_follower')

        # create subscribors
        self.pose_sub = self.create_subscription(PoseStamped, '/opti_pose', self.pose_callback, 10)
        self.waypoint_sub = self.create_subscription(Path, '/sim_waypoints', self.path_callback, 10)

        # initialize nav2
        self.nav = BasicNavigator()

        # list of waypoints to follow
        self.waypoints = []

        self.started = False
        self.timer = self.create_timer(0.5, self.follow_waypoints)

    def pose_callback(self, pose):
        print(pose.pose)

    def path_callback(self, path):
        if len(self.waypoints) == 0:
            self.waypoints = path.poses

    def follow_waypoints(self):
        if self.started or len(self.waypoints) == 0:
            return
        
        self.started = True
        self.timer.destroy()

        # wait for nav2 to initialize
        self.nav.waitUntilNav2Active()
        self.get_logger().info('Nav2 is ready')

        self.nav.followWaypoints(self.waypoints)

        while not self.nav.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.nav.getResult()




def main(args=None):
    rclpy.init()
    path_follower = PathFollower()
    rclpy.spin(path_follower)
    rclpy.shutdown()

if __name__ == '__main__':
    main()