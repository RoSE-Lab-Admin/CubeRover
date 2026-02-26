import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Vector3
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster

from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

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
            self.opti_sub = self.create_subscription(PoseStamped, '/CubeRover_V1/pose', self.opti_callback, 10, callback_group=self.opti_group)
            self.odom_trans = TransformBroadcaster(self)
            self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
            # create previous poses list
            self.prev_poses = deque()
            self.pose_idx = 0

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

        # calculate a rough linear and angular velocity
        if len(self.prev_poses) < 5:
            self.prev_poses.append(msg)
            self.odom_pub.publish(odom)
            return
        
        # if enough points to calc, pop first and add to end
        self.prev_poses.popleft()
        self.prev_poses.append(msg)

        velx, vely, omega = self.vel_interp()

        twist_vel = Twist()
        twist_vel.linear.x = velx
        twist_vel.linear.y = vely
        twist_vel.angular.x = omega[0]
        twist_vel.angular.y = omega[1]
        twist_vel.angular.z = omega[2]

        odom.twist.twist = twist_vel
        self.odom_pub.publish(odom)



    # calculate
    def vel_interp(self):
        # time dif in seconds
        first_time = Time.from_msg(self.prev_poses[0].header.stamp)
        last_time = Time.from_msg(self.prev_poses[-1].header.stamp)
        delta_t = (last_time - first_time).nanoseconds / 1e9

        # SHOULD I ADD COVIARIANCE FOR THIS?
        # linear velocity interp, in xy plane only
        first_posx = self.prev_poses[0].pose.position.x
        first_posy = self.prev_poses[0].pose.position.y
        last_posx = self.prev_poses[-1].pose.position.x
        last_posy = self.prev_poses[-1].pose.position.y
        
        velx = (last_posx - first_posx) / delta_t
        vely = (last_posy - first_posy) / delta_t

        # angular velocity interp 
        first_rot = self.prev_poses[0].pose.orientation
        last_rot = self.prev_poses[-1].pose.orientation
        q0 = np.array([
            first_rot.x,
            first_rot.y,
            first_rot.z,
            first_rot.w
        ])
        q1 = np.array([
            last_rot.x,
            last_rot.y,
            last_rot.z,
            last_rot.w
        ])

        # ensure shortest path taken
        if np.dot(q0,q1) < 0.0:
            q1 = -q1

        R0 = R.from_quat(q0)
        R1 = R.from_quat(q1)

        # calc relative rotation
        Rrel = R0.inv() * R1
        # convert to angle axis
        rotvec = Rrel.as_rotvec()
        # calc angular vel
        omega = rotvec / delta_t

        return velx, vely, omega


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