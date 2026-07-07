import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped, Vector3
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from collections import deque
import copy
import numpy as np
from scipy.spatial.transform import Rotation as R
import time, signal

class PathFollower(Node):
    def __init__(self):

        super().__init__('path_follower')

        # parameters
        self.declare_parameter('use_opti', True)
        self.declare_parameter('opti_topic', '/CubeRover_V1/pose')
        self.declare_parameter('robot_frame', 'CubeRover_V1')
        self.use_opti    = self.get_parameter('use_opti').value
        self.opti_topic  = self.get_parameter('opti_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value

        # create callback group so it can execute while nav2 blocks
        self.opti_group = ReentrantCallbackGroup()

        # create subscriptions
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_sub = self.create_subscription(Path, '/sim_waypoints', self.waypoint_callback, qos)
        # subscribe to ground truth
        if self.use_opti:
            self.opti_sub = self.create_subscription(PoseStamped, self.opti_topic, self.opti_callback, 10, callback_group=self.opti_group)
            self.odom_trans = TransformBroadcaster(self)
            self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
            # create previous poses list
            self.prev_poses = deque()
            self.pose_idx = 0

        # publisher to explicitly stop motors on shutdown
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # initialize nav2
        self.nav = BasicNavigator()

        self.waypoints = []
        self.point_path = []

        # state trackers
        self.nav2_ready = False
        self.started = False
        self.finished = False
        self.rec_pose = False
        self.current_wp_idx = 0
        self.last_goal_time = None
        self.last_issued_heading = None
        self.goal_replan_interval = 2.0      # seconds between orientation checks
        self.heading_update_threshold = 0.26  # ~15 degrees: only re-issue if heading changed more than this

        # poll for nav2 readiness separately so it doesn't block the control loop
        self.nav2_check_timer = self.create_timer(1.0, self.check_nav2_ready, callback_group=self.opti_group)
        self.timer = self.create_timer(0.1, self.follow_waypoints)

    def waypoint_callback(self, trajectory):
        # path message with list of posestamped waypoints
        self.point_path = trajectory.poses
        self.waypoints = trajectory.poses

    def arc_arrival_heading(self, x0, y0, theta0, x1, y1):
        # find the unique circular arc from (x0,y0,theta0) through (x1,y1)
        # and return the tangent heading at the goal
        dx, dy = x1 - x0, y1 - y0
        denom = dx * np.sin(theta0) - dy * np.cos(theta0)

        if abs(denom) < 1e-6:  # already aligned: straight line
            return np.arctan2(dy, dx)

        r  = -(dx**2 + dy**2) / (2.0 * denom)
        Cx = x0 - r * np.sin(theta0)
        Cy = y0 + r * np.cos(theta0)

        rx, ry = x1 - Cx, y1 - Cy
        if r > 0:  # CCW: tangent = radial rotated +90°
            return np.arctan2(rx, -ry)
        else:      # CW:  tangent = radial rotated -90°
            return np.arctan2(-rx, ry)

    def orientation_calc(self):
        poses = self.point_path
        n = len(poses)

        # chain of positions: rover start followed by each waypoint
        pos_x = [self.first_pos.x] + [p.pose.position.x for p in poses]
        pos_y = [self.first_pos.y] + [p.pose.position.y for p in poses]

        # initial heading from stored quaternion
        q0 = self.first_orien
        theta = R.from_quat([q0.x, q0.y, q0.z, q0.w]).as_euler('xyz')[2]

        for i in range(n):
            # arc from pos[i] (heading theta) to pos[i+1]
            theta = self.arc_arrival_heading(
                pos_x[i], pos_y[i], theta,
                pos_x[i + 1], pos_y[i + 1]
            )
            q = R.from_euler('z', theta).as_quat()  # [x, y, z, w]
            poses[i].pose.orientation.x = q[0]
            poses[i].pose.orientation.y = q[1]
            poses[i].pose.orientation.z = q[2]
            poses[i].pose.orientation.w = q[3]

        self.waypoints = poses

    # callback for if opti mode is being used
    def opti_callback(self, msg):

        self.rec_pose = True

        stamp = self.get_clock().now().to_msg()

        # broadcast ground truth odom -> base_link transform
        # trans = TransformStamped()
        # trans.header.stamp = stamp
        # trans.header.frame_id = 'odom'
        # trans.child_frame_id = 'base_link'
        # trans.transform.translation.x = msg.pose.position.x
        # trans.transform.translation.y = msg.pose.position.y
        # trans.transform.translation.z = msg.pose.position.z
        # trans.transform.rotation = msg.pose.orientation
        # self.odom_trans.sendTransform(trans)

        # publish ground truth as odometry for nav2
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = self.robot_frame
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
        if delta_t == 0.0:
            return 0.0, 0.0, np.zeros(3)

        # SHOULD I ADD COVIARIANCE FOR THIS?
        # linear velocity interp, in xy plane only
        first_posx = self.prev_poses[0].pose.position.x
        first_posy = self.prev_poses[0].pose.position.y
        last_posx = self.prev_poses[-1].pose.position.x
        last_posy = self.prev_poses[-1].pose.position.y
        
        velx_world = (last_posx - first_posx) / delta_t
        vely_world = (last_posy - first_posy) / delta_t

        # rotate world-frame velocity into robot body frame
        last_quat = self.prev_poses[-1].pose.orientation
        q_robot = np.array([last_quat.x, last_quat.y, last_quat.z, last_quat.w])
        v_body = R.from_quat(q_robot).inv().apply(np.array([velx_world, vely_world, 0.0]))
        velx = v_body[0]
        vely = v_body[1]

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


    def check_nav2_ready(self):
        self.nav2_check_timer.cancel()
        self.nav._waitForNodeToActivate('planner_server')
        self.nav._waitForNodeToActivate('controller_server')
        self.nav._waitForNodeToActivate('bt_navigator')
        self.nav2_ready = True

    def _arc_heading(self):
        if not self.use_opti or len(self.prev_poses) == 0:
            return None
        wp = self.point_path[self.current_wp_idx]
        cur = self.prev_poses[-1]
        q = cur.pose.orientation
        theta = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
        return self.arc_arrival_heading(
            cur.pose.position.x, cur.pose.position.y, theta,
            wp.pose.position.x, wp.pose.position.y
        )

    def _issue_goal(self):
        wp = copy.deepcopy(self.point_path[self.current_wp_idx])
        heading = self._arc_heading()
        if heading is not None:
            q_new = R.from_euler('z', heading).as_quat()
            wp.pose.orientation.x = float(q_new[0])
            wp.pose.orientation.y = float(q_new[1])
            wp.pose.orientation.z = float(q_new[2])
            wp.pose.orientation.w = float(q_new[3])
        self.last_issued_heading = heading
        self.last_goal_time = self.get_clock().now()
        self.nav.goToPose(wp)

    def follow_waypoints(self):

        # if no path received yet
        if len(self.point_path) == 0:
            return

        # wait for nav2 to initialize
        if not self.nav2_ready:
            return

        if self.finished:
            return

        # start navigating
        if not self.started:
            if self.use_opti and not self.rec_pose:
                self.get_logger().info("waiting to receive OptiTrack pose before beginning trajectory")
                return
            self.current_wp_idx = 0
            self._issue_goal()
            self.started = True
            return

        if not self.nav.isTaskComplete():
            now = self.get_clock().now()
            elapsed = (now - self.last_goal_time).nanoseconds / 1e9
            if elapsed > self.goal_replan_interval:
                # only re-issue if the arc heading has shifted significantly
                new_heading = self._arc_heading()
                if new_heading is not None and self.last_issued_heading is not None:
                    diff = abs(np.arctan2(np.sin(new_heading - self.last_issued_heading),
                                         np.cos(new_heading - self.last_issued_heading)))
                    if diff > self.heading_update_threshold:
                        self._issue_goal()
                    else:
                        self.last_goal_time = self.get_clock().now()  # reset timer, skip re-issue
                else:
                    self._issue_goal()
            return

        result = self.nav.getResult()

        if result == TaskResult.CANCELED:
            self.get_logger().info("trajectory cancelled")
            self.started = False
            self.finished = True
            self.stop_nav()
            return

        # advance to next waypoint
        self.current_wp_idx += 1
        if self.current_wp_idx >= len(self.point_path):
            self.get_logger().info("trajectory completed")
            self.started = False
            self.finished = True
            self.stop_nav()
            return

        self._issue_goal()


    def stop_nav(self):
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_pub.publish(stop_msg)

    def shutdown_handler(self, signum, frame):
        self.get_logger().info("Attempting shutdown")

        try:
            self.nav.cancelTask()
        except Exception:
            pass

        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_pub.publish(stop_msg)
        time.sleep(0.2)
        rclpy.shutdown()

            

def main(args=None):
    rclpy.init()
    path_follower = PathFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(path_follower)

    signal.signal(signal.SIGINT, path_follower.shutdown_handler)

    executor.spin()

    # try:
    #     executor.spin()
    # finally:
    #     path_follower.stop_nav()
    #     rclpy.shutdown()

if __name__ == '__main__':
    main()