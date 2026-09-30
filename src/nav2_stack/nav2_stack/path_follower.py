import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped, Vector3
from std_msgs.msg import String
from lifecycle_msgs.srv import GetState
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
        self.declare_parameter('opti_topic', '/FitRosey_V1/pose')
        self.declare_parameter('robot_frame', 'FitRosey_V1')
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
        # authoritative trajectory outcome -- autonomous_trials.py's own
        # distance-based polling of the pose topic is a coarser, staler proxy
        # for the same thing and can disagree with Nav2's own goToPose result
        # right at the goal-tolerance boundary (observed: Nav2 reported
        # TaskResult.SUCCEEDED while the external distance poll still saw
        # 0.542m > 0.5m tolerance and eventually gave up as "stalled"), so
        # this publishes the real result for it to trust instead.
        self.goal_result_pub = self.create_publisher(String, '/trial_goal_result', 10)
        self.trajectory_had_failure = False

        # initialize nav2
        self.nav = BasicNavigator()

        self.waypoints = []
        self.point_path = []

        # state trackers
        self.nav2_ready = False
        self._checking_nav2 = False  # guards against overlapping check_nav2_ready() calls
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
        if trajectory.poses:
            last = trajectory.poses[-1].pose.position
            self.get_logger().info(
                f"received path with {len(trajectory.poses)} waypoint(s), "
                f"final target=({last.x:.3f}, {last.y:.3f})")
        else:
            self.get_logger().warn("received EMPTY path (0 waypoints)")

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


    def _wait_for_node_active(self, node_name: str, timeout_sec: float = 5.0) -> bool:
        """Bounded-timeout replacement for BasicNavigator's private
        _waitForNodeToActivate(), which loops forever with NO timeout at all
        (rclpy.spin_until_future_complete with no timeout_sec, inside a
        while-not-active loop with no exit condition) -- if the node's
        get_state call ever fails to resolve cleanly (observed live: an RMW
        response-delivery timeout on planner_server's side), that left
        check_nav2_ready() -- and therefore the whole trial -- permanently
        stuck with no way to retry. Returns True if node_name reports
        'active' within timeout_sec, False otherwise (caller should retry).

        Spins self.nav (the existing BasicNavigator instance), NOT self
        (PathFollower) -- self is already owned by main()'s
        MultiThreadedExecutor, and calling a blocking spin from within one of
        its OWN callbacks doesn't work either way: the bare global
        rclpy.spin_until_future_complete(self, ...) creates its own SEPARATE
        temporary executor for the same node, which silently starves this
        node's OTHER callbacks (confirmed live: waypoint_callback stopped
        firing entirely, so a path was never received); and
        self.executor.spin_until_future_complete(...) raises `RuntimeError:
        Executor is already spinning` outright, since rclpy explicitly
        forbids re-entering an executor's own spin from inside one of its own
        callbacks (also confirmed live). self.nav is a genuinely separate
        node that's never added to any persistent executor (matching how the
        original _waitForNodeToActivate calls it used self.nav too), so
        spinning it independently via the bare global function is safe."""
        client = self.nav.create_client(GetState, f'{node_name}/get_state')
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                return False
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.nav, future, timeout_sec=timeout_sec)
            result = future.result()
            return result is not None and result.current_state.label == 'active'
        finally:
            self.nav.destroy_client(client)

    def check_nav2_ready(self):
        # opti_group is a ReentrantCallbackGroup, so the executor is free to
        # start a SECOND overlapping invocation of this same timer callback
        # if one is already in progress (each _wait_for_node_active call can
        # take several real seconds, easily longer than this timer's 1s
        # period) -- two concurrent calls both spinning self.nav collide on
        # self.nav's own executor (RuntimeError: Executor is already
        # spinning, confirmed live). This guard ensures only one invocation
        # of check_nav2_ready itself is ever actually running at a time.
        if self._checking_nav2:
            return
        self._checking_nav2 = True
        try:
            for node_name in ('planner_server', 'controller_server', 'bt_navigator'):
                if not self._wait_for_node_active(node_name):
                    self.get_logger().warn(
                        f"check_nav2_ready: {node_name} not active yet (or its get_state "
                        f"call timed out) -- retrying in 1s")
                    return
            self.nav2_check_timer.cancel()
            self.nav2_ready = True
            self.get_logger().info("nav2_ready=True (planner_server/controller_server/"
                                   "bt_navigator all active)")
        finally:
            self._checking_nav2 = False

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
        self.get_logger().info(
            f"issuing goal {self.current_wp_idx + 1}/{len(self.point_path)}: "
            f"({wp.pose.position.x:.3f}, {wp.pose.position.y:.3f})  heading={heading}")
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

        if result == TaskResult.FAILED:
            # NOTE: previously fell through and was silently treated the same
            # as success (current_wp_idx still advanced) -- now at least logged
            # so a failed goToPose is visible instead of indistinguishable from
            # a real success.
            self.get_logger().warn(
                f"goToPose FAILED for waypoint {self.current_wp_idx + 1}/{len(self.point_path)} "
                f"-- advancing anyway (existing behavior, unchanged)")
            self.trajectory_had_failure = True
        else:
            self.get_logger().info(
                f"goToPose result={result} for waypoint {self.current_wp_idx + 1}/{len(self.point_path)}")

        # advance to next waypoint
        self.current_wp_idx += 1
        if self.current_wp_idx >= len(self.point_path):
            self.get_logger().info("trajectory completed")
            self.started = False
            self.finished = True
            self.goal_result_pub.publish(
                String(data="failed" if self.trajectory_had_failure else "succeeded"))
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
        # Stop the EXECUTOR (lets executor.spin() in main() return cleanly),
        # not rclpy itself -- calling rclpy.shutdown() here, while spin() is
        # still actively running, left spin()'s internal loop trying to build
        # a new wait-set against an already-shutdown context on its next
        # iteration (RCLError: "failed to initialize wait set ... the given
        # context is not valid"), which crashed the process instead of exiting
        # cleanly and made launch wait out the full SIGINT timeout before
        # escalating to SIGTERM on every single trial.
        self.executor.shutdown()

def main(args=None):
    rclpy.init()
    path_follower = PathFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(path_follower)
    path_follower.executor = executor

    signal.signal(signal.SIGINT, path_follower.shutdown_handler)

    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()