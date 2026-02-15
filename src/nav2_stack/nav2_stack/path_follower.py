import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster

class PathFollower(Node):
    def __init__(self):

        super().__init__('path_follower')

        # parameters
        self.declare_parameter('use_opti', True)
        use_opti = self.get_parameter('use_opti').value

        # create callback group so it can execute while nav2 blocks
        self.opti_group = ReentrantCallbackGroup()

        # create subscriptions
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_sub = self.create_subscription(Path, '/sim_waypoints', self.waypoint_callback, qos)
        # subscribe to ground truth
        if use_opti:
            self.opti_sub = self.create_subscription(PoseStamped, '/opti_pose', self.opti_callback, 10, callback_group=self.opti_group)
            self.odom_trans = TransformBroadcaster(self)
            self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)

        # initialize nav2
        self.nav = BasicNavigator() 

        self.waypoints = []

        # state trackers
        self.nav2_ready = False
        self.started = False

        self.timer = self.create_timer(0.5, self.follow_waypoints)

    def waypoint_callback(self, trajectory):
        # path message with list of posestamped waypoints
        self.waypoints = trajectory.poses

    # callback for if opti mode is being used
    def opti_callback(self, msg):
        self.rec_pose = True

        stamp = self.get_clock().now().to_msg()

        # broadcast ground truth odom -> base_link transform
        trans = TransformStamped()
        trans.header.stamp = stamp
        trans.header.frame_id = 'odom'
        trans.child_frame_id = 'base_link'
        trans.transform.translation.x = msg.pose.position.x
        trans.transform.translation.y = msg.pose.position.y
        trans.transform.translation.z = msg.pose.position.z
        trans.transform.rotation = msg.pose.orientation
        self.odom_trans.sendTransform(trans)

        # publish ground truth as odometry for nav2
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = msg.pose
        self.odom_pub.publish(odom)

    def follow_waypoints(self):
        
        # if using opti mode, dont run until transforms are being published
        # if no path received yet
        if len(self.waypoints) == 0:
            return
        
        # wait for nav2 to initialize
        if not self.nav2_ready:
            self.nav._waitForNodeToActivate('bt_navigator')
            self.nav2_ready = True

        # start navigating
        if not self.started:
            self.nav.followWaypoints(self.waypoints)
            self.started = True
            return

        # check to see if task has completed
        if not self.nav.isTaskComplete():
            return

        result = self.nav.getResult()

        # reset 
        self.started = False

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("trajectory completed")

        elif result == TaskResult.CANCELED:
            self.get_logger().info("trajectory cancelled")
            



def main(args=None):
    rclpy.init()
    path_follower = PathFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(path_follower)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()