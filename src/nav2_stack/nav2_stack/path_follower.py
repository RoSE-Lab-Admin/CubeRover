import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, DurabilityPolicy

class PathFollower(Node):
    def __init__(self):

        super().__init__('path_follower')

        # create subscription
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_sub = self.create_subscription(Path, '/sim_waypoints', self.waypoint_callback, qos)

        # initialize nav2
        self.nav = BasicNavigator() 

        self.waypoints = []

        self.started = False
        self.timer = self.create_timer(0.5, self.follow_waypoints)

    def waypoint_callback(self, trajectory):
        # path message with list of posestamped waypoints
        self.waypoints = trajectory.poses


    def follow_waypoints(self):
        if self.started or len(self.waypoints) == 0:
            return
        
        self.started = True
        
        # wait for nav2 to initialize
        self.nav._waitForNodeToActivate('bt_navigator')
        self.get_logger().info('Nav2 is ready')

        self.nav.followWaypoints(self.waypoints)

        while not self.nav.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.nav.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("trajectory completed")

        elif result == TaskResult.CANCELED:
            self.get_logger().info("trajectory cancelled")
            



def main(args=None):
    rclpy.init()
    path_follower = PathFollower()
    rclpy.spin(path_follower)
    rclpy.shutdown()

if __name__ == '__main__':
    main()