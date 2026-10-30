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

from nav2_stack.escape import Escaper, default_planner_id

STUCK_DWELL_S = 2.0          # stuck this long while navigating -> escape
MAX_ESCAPES_PER_WAYPOINT = 2

class PathFollower(Node):
    def __init__(self):

        super().__init__('path_follower')

        # parameters
        self.declare_parameter('use_opti', True)
        self.declare_parameter('opti_topic', '/FitRosey_passive/pose')
        self.declare_parameter('robot_frame', 'FitRosey_passive')
        # Re-send the current goal mid-drive when the arc heading drifts. Off by
        # default: every re-send forces a fresh replan from the current pose,
        # which made the rover flip between path shapes instead of committing
        # to one, and GridBasedCustom (ignore_goal_heading) ignores the goal
        # yaw anyway.
        self.declare_parameter('reissue_on_heading_change', False)
        # Drive out of an obstacle when the rover is stuck in it (see
        # nav2_stack/escape.py): the planner and controller cannot recover
        # from a footprint already in collision on their own.
        self.declare_parameter('escape_when_stuck', True)
        # How the waypoints are driven (autonomous_trials' waypoint_mode "spiral"):
        #  - "goals" (default): goToPose to each waypoint in turn. With
        #    advance_distance > 0 the goal is instead always the waypoint about
        #    advance_distance ahead of the rover's progress along the waypoints, so
        #    it never stops in between (0 = wait until each goal is reached, the
        #    original behaviour).
        #  - "path": goToPose to the first waypoint, then the waypoints themselves
        #    (dense, e.g. a spiral leg) go straight to the controller via
        #    FollowPath -- no global planner, no replanning.
        self.declare_parameter('follow_mode', 'goals')
        self.declare_parameter('advance_distance', 0.0)
        # tag for /trial_goal_result ("<result>:<trial_id>"), so autonomous_trials
        # can tell this trial's result from a late one of the previous trial's
        # path_follower (which it may already have stopped by distance)
        self.declare_parameter('trial_id', '')
        self.use_opti    = self.get_parameter('use_opti').value
        self.opti_topic  = self.get_parameter('opti_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.reissue_on_heading_change = self.get_parameter('reissue_on_heading_change').value
        self.escape_when_stuck = self.get_parameter('escape_when_stuck').value
        self.follow_mode = self.get_parameter('follow_mode').value
        self.advance_distance = float(self.get_parameter('advance_distance').value)
        self.trial_id = self.get_parameter('trial_id').value
        if self.follow_mode not in ('goals', 'path'):
            raise ValueError(f"follow_mode must be 'goals' or 'path', got {self.follow_mode!r}")
        self.path_phase = None   # follow_mode "path": "approach" (goToPose) then "follow"
        self.progress_idx = 0    # advance_distance > 0: waypoint the rover has got to

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
        # escape helper: spins self.nav (never added to an executor), like
        # _wait_for_node_active; pose comes from opti_callback
        self.escaper = (Escaper(self.nav, self._current_pose, default_planner_id(),
                                robot_frame=self.robot_frame, cmd_pub=self.cmd_vel_pub)
                        if self.escape_when_stuck and self.use_opti and self.follow_mode == 'goals'
                        else None)
        self.stuck_since = None
        self.escapes_this_wp = 0

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

    def _current_pose(self):
        if not self.use_opti or len(self.prev_poses) == 0:
            return None
        cur = self.prev_poses[-1]
        q = cur.pose.orientation
        theta = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
        return (cur.pose.position.x, cur.pose.position.y, theta)

    def _cancel_nav(self, timeout_sec=3.0):
        # BasicNavigator.cancelTask() spins without a timeout; bound it here
        handle = getattr(self.nav, 'goal_handle', None)
        if handle is None:
            return
        fut = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.nav, fut, timeout_sec=timeout_sec)

    def _try_escape(self, reason):
        """Stuck in an obstacle: cancel the goal, drive out until the planner
        can plan again (escape.py), then re-send the same waypoint. Returns
        True if an escape was attempted (the goal has been re-sent)."""
        if self.escaper is None or self.escapes_this_wp >= MAX_ESCAPES_PER_WAYPOINT:
            return False
        pose = self._current_pose()
        if pose is None or not self.escaper._load_costmap() or not self.escaper.is_stuck(pose):
            return False
        self.escapes_this_wp += 1
        wp = self.point_path[self.current_wp_idx].pose.position
        self.get_logger().warn(
            f"rover stuck ({reason}) -- escape attempt {self.escapes_this_wp}/"
            f"{MAX_ESCAPES_PER_WAYPOINT} toward waypoint {self.current_wp_idx + 1}")
        self._cancel_nav()
        ok, msg = self.escaper.run((wp.x, wp.y))
        self.get_logger().warn(f"escape {'succeeded' if ok else 'did not free the rover'}: {msg}")
        self.stuck_since = None
        self._issue_goal()
        return True

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
            self.started = True
            if self.follow_mode == 'path':
                self._start_path_mode()
                return
            self._issue_goal()
            return

        if self.follow_mode == 'path':
            self._step_path_mode()
            return

        if not self.nav.isTaskComplete():
            if self._advance_early():
                return
            # stuck in an obstacle while navigating -> escape (see escape.py)
            if self.escaper is not None and self.escapes_this_wp < MAX_ESCAPES_PER_WAYPOINT:
                pose = self._current_pose()
                if pose is not None and self.escaper._load_costmap() and self.escaper.is_stuck(pose):
                    now_s = time.monotonic()
                    if self.stuck_since is None:
                        self.stuck_since = now_s
                    elif now_s - self.stuck_since >= STUCK_DWELL_S:
                        self._try_escape(f'in an obstacle for {STUCK_DWELL_S:.0f} s')
                        return
                else:
                    self.stuck_since = None
            if not self.reissue_on_heading_change:
                return
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

        if result == TaskResult.FAILED and self._try_escape('goToPose failed'):
            return  # same waypoint re-sent after the escape

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
        self.escapes_this_wp = 0
        self.stuck_since = None
        if self.current_wp_idx >= len(self.point_path):
            self.get_logger().info("trajectory completed")
            self.started = False
            self.finished = True
            self._publish_result("failed" if self.trajectory_had_failure else "succeeded")
            self.stop_nav()
            return

        self._issue_goal()


    def _advance_early(self):
        """advance_distance > 0: keep the goal about advance_distance ahead of the
        rover's progress along the waypoints (preempting the current goal), so it
        keeps driving instead of stopping at every waypoint. Progress = the nearest
        waypoint within 3 m ahead of the last one reached, counted only while the
        rover is within 1 m of it -- far from the waypoints (e.g. starting from the
        centre of a spiral) it just drives to the current goal first, rather than
        skipping every waypoint that happens to be close."""
        if self.advance_distance <= 0 or self.current_wp_idx >= len(self.point_path) - 1:
            return False
        pose = self._current_pose()
        if pose is None:
            return False
        xy = np.array([[p.pose.position.x, p.pose.position.y] for p in self.point_path])
        cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
        hi = int(np.searchsorted(cum, cum[self.progress_idx] + 3.0, side='right'))
        d = np.hypot(xy[self.progress_idx:hi, 0] - pose[0], xy[self.progress_idx:hi, 1] - pose[1])
        if len(d) and d.min() < 1.0:
            self.progress_idx += int(np.argmin(d))
        target = int(np.searchsorted(cum, cum[self.progress_idx] + self.advance_distance))
        target = min(max(target, self.progress_idx + 1), len(xy) - 1)
        if target <= self.current_wp_idx:
            return False
        self.current_wp_idx = target
        self.escapes_this_wp = 0
        self.stuck_since = None
        self._issue_goal()
        return True

    def _dense_path(self, reverse=False):
        """self.point_path as a nav_msgs/Path with each pose facing the next one --
        or, with reverse, facing away from it: the rover drives the path backwards,
        encoded like the planner's reverse segments (pose heading = rover heading),
        which the controller's critics (PathAngleCritic mode 2) follow as such."""
        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()
        pts = [(p.pose.position.x, p.pose.position.y) for p in self.point_path]
        for i, (x, y) in enumerate(pts):
            j = min(i + 1, len(pts) - 1)
            k = j - 1 if j == i else i
            yaw = np.arctan2(pts[j][1] - pts[k][1], pts[j][0] - pts[k][0]) + (np.pi if reverse else 0.0)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z, ps.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
            path.poses.append(ps)
        return path

    def _start_path_mode(self):
        """follow_mode "path": drive to the path's start with the planner unless
        the rover is already there, then hand the path to the controller."""
        pose = self._current_pose()
        first = self.point_path[0].pose.position
        if pose is not None and np.hypot(first.x - pose[0], first.y - pose[1]) < 0.5:
            self._follow_dense_path()
            return
        self.path_phase = 'approach'
        self._issue_goal()

    def _path_gear_is_reverse(self):
        """Gear for the whole path, from how the rover faces right now: reverse if
        it faces more than 90 deg away from the path's direction of travel at the
        path point nearest to it (the path has no cusps, so one gear throughout)."""
        pose = self._current_pose()
        if pose is None:
            return False
        pts = np.array([[p.pose.position.x, p.pose.position.y] for p in self.point_path])
        i = int(np.argmin(np.hypot(pts[:, 0] - pose[0], pts[:, 1] - pose[1])))
        j = min(i + 1, len(pts) - 1)
        k = j - 1 if j == i else i
        travel = np.arctan2(pts[j, 1] - pts[k, 1], pts[j, 0] - pts[k, 0])
        return abs(np.arctan2(np.sin(pose[2] - travel), np.cos(pose[2] - travel))) > np.pi / 2

    def _follow_dense_path(self):
        self.path_phase = 'follow'
        reverse = self._path_gear_is_reverse()
        path = self._dense_path(reverse)
        self.get_logger().info(
            f"following the {len(path.poses)}-pose path directly with the controller (FollowPath), "
            f"{'in REVERSE (the rover faces against the path)' if reverse else 'forward'}")
        self.nav.followPath(path)

    def _step_path_mode(self):
        if not self.nav.isTaskComplete():
            return
        result = self.nav.getResult()
        if result == TaskResult.CANCELED:
            # stopped from outside (autonomous_trials ends the trial): no
            # result to report, as in goals mode
            self.get_logger().info("trajectory cancelled")
            self.started = False
            self.finished = True
            self.stop_nav()
            return
        if self.path_phase == 'approach':
            if result != TaskResult.SUCCEEDED:
                self.get_logger().warn(f"approach to the path's start: result={result} -- "
                                       f"following the path from here anyway")
            self._follow_dense_path()
            return
        self.get_logger().info(f"FollowPath result={result}")
        self.started = False
        self.finished = True
        self._publish_result("succeeded" if result == TaskResult.SUCCEEDED else "failed")
        self.stop_nav()

    def _publish_result(self, result):
        self.goal_result_pub.publish(
            String(data=f"{result}:{self.trial_id}" if self.trial_id else result))

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