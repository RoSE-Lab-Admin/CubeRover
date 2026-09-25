#!/usr/bin/env python3
"""
Safety watchdog: detects a stale/frozen OptiTrack pose feed while the rover is
actually moving (wheel encoders + commanded velocity both show real motion),
and immediately stops the rover + aborts the current run.

Why this can't live in the Nav2 behavior tree: every BT recovery behavior
(BackUp, costmap clears, etc.) implicitly trusts the pose feed -- their whole
premise is "the robot seems stuck, try to unstick it." Here the pose feed
itself is the thing lying, so a BT recovery would run through the exact same
compromised pose-consuming machinery. This is an independent node that
cross-checks pose against wheel encoders and commanded velocity, then acts
*outside* the planning/control stack entirely.

Launched alongside pose_pub/path_follower in waypoint.launch.py (always
included) -- protects both the manual workflow and autonomous_trials.py with
no extra wiring needed on either side.
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool
from control_msgs.msg import DynamicJointState
from nav2_simple_commander.robot_navigator import BasicNavigator

LEFT_WHEELS = ["back_left_wheel_joint", "front_left_wheel_joint"]
RIGHT_WHEELS = ["front_right_wheel_joint", "back_right_wheel_joint"]
WHEEL_RADIUS = 0.158       # m -- matches plotting_data training pipeline
WHEEL_SEPARATION = 0.3365  # m

STOP_BURST_HZ = 20.0
STOP_BURST_S = 2.0
EVAL_HZ = 10.0


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class SafetyWatchdog(Node):
    def __init__(self):
        super().__init__("safety_watchdog")

        self.declare_parameter("opti_topic", "/FitRosey_V1/pose")
        self.declare_parameter("window_s", 2.0)
        self.declare_parameter("pose_static_thresh_m", 0.03)
        self.declare_parameter("yaw_static_thresh_rad", 0.05)
        self.declare_parameter("wheel_moving_thresh_mps", 0.03)
        self.declare_parameter("wheel_moving_thresh_radps", 0.05)
        self.declare_parameter("cmd_moving_thresh_mps", 0.05)
        self.declare_parameter("cmd_moving_thresh_radps", 0.1)

        self.window_s = self.get_parameter("window_s").value
        self.pose_static_thresh_m = self.get_parameter("pose_static_thresh_m").value
        self.yaw_static_thresh_rad = self.get_parameter("yaw_static_thresh_rad").value
        self.wheel_moving_thresh_mps = self.get_parameter("wheel_moving_thresh_mps").value
        self.wheel_moving_thresh_radps = self.get_parameter("wheel_moving_thresh_radps").value
        self.cmd_moving_thresh_mps = self.get_parameter("cmd_moving_thresh_mps").value
        self.cmd_moving_thresh_radps = self.get_parameter("cmd_moving_thresh_radps").value
        opti_topic = self.get_parameter("opti_topic").value

        # (t, x, y, yaw) / (t, vx_wheel, wz_wheel) / (t, vx_cmd, wz_cmd), each
        # trimmed to the last window_s seconds on arrival.
        self.pose_hist = deque()
        self.wheel_hist = deque()
        self.cmd_hist = deque()

        self.triggered = False
        self.stop_burst_until = None

        self.create_subscription(PoseStamped, opti_topic, self._pose_cb, 10)
        self.create_subscription(DynamicJointState, "/dynamic_joint_states", self._joint_cb, 10)
        self.create_subscription(TwistStamped, "/roseybot_base_controller/cmd_vel_out",
                                 self._cmd_cb, 10)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.safety_stop_pub = self.create_publisher(Bool, "/safety_stop", 10)

        self.nav = BasicNavigator()

        self.create_timer(1.0 / EVAL_HZ, self._evaluate)
        self.create_timer(1.0 / STOP_BURST_HZ, self._stop_burst_tick)

        self.get_logger().info(
            f"safety_watchdog active: window={self.window_s}s  "
            f"pose_static<{self.pose_static_thresh_m}m/{self.yaw_static_thresh_rad}rad  "
            f"wheel_moving>{self.wheel_moving_thresh_mps}mps/{self.wheel_moving_thresh_radps}radps  "
            f"cmd_moving>{self.cmd_moving_thresh_mps}mps/{self.cmd_moving_thresh_radps}radps")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _trim(self, hist: deque, t: float):
        while hist and t - hist[0][0] > self.window_s:
            hist.popleft()

    def _pose_cb(self, msg: PoseStamped):
        t = self._now()
        o = msg.pose.orientation
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        self.pose_hist.append((t, msg.pose.position.x, msg.pose.position.y, yaw))
        self._trim(self.pose_hist, t)

    def _joint_cb(self, msg: DynamicJointState):
        t = self._now()
        vel = {}
        for jn, iv in zip(msg.joint_names, msg.interface_values):
            d = dict(zip(iv.interface_names, iv.values))
            if "velocity" in d:
                vel[jn] = d["velocity"]
        # Require every configured wheel joint present -- safety-critical, so a
        # partial/incomplete reading (e.g. one wheel not reporting) is treated
        # as "no usable sample" rather than silently averaging over less data.
        if not all(j in vel for j in LEFT_WHEELS + RIGHT_WHEELS):
            return
        wl = np.mean([vel[j] for j in LEFT_WHEELS])
        wr = np.mean([vel[j] for j in RIGHT_WHEELS])
        vx_wheel = WHEEL_RADIUS * (wl + wr) / 2.0
        wz_wheel = WHEEL_RADIUS * (wr - wl) / WHEEL_SEPARATION
        self.wheel_hist.append((t, vx_wheel, wz_wheel))
        self._trim(self.wheel_hist, t)

    def _cmd_cb(self, msg: TwistStamped):
        t = self._now()
        self.cmd_hist.append((t, msg.twist.linear.x, msg.twist.angular.z))
        self._trim(self.cmd_hist, t)

    def _evaluate(self):
        if self.triggered:
            return
        t = self._now()
        self._trim(self.pose_hist, t)
        self._trim(self.wheel_hist, t)
        self._trim(self.cmd_hist, t)

        # Need a genuinely full window of pose history to judge staleness --
        # covers both "value frozen but still publishing" and "topic went
        # silent" (oldest sample ages out, window never fills back up).
        if not self.pose_hist or (t - self.pose_hist[0][0]) < self.window_s:
            return
        if not self.wheel_hist or not self.cmd_hist:
            return

        xs = [p[1] for p in self.pose_hist]
        ys = [p[2] for p in self.pose_hist]
        yaws = [p[3] for p in self.pose_hist]
        pos_range = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        yaw_range = max(yaws) - min(yaws)
        pose_stale = pos_range < self.pose_static_thresh_m and yaw_range < self.yaw_static_thresh_rad

        wheel_moved = all(
            abs(w[1]) > self.wheel_moving_thresh_mps or abs(w[2]) > self.wheel_moving_thresh_radps
            for w in self.wheel_hist)
        cmd_significant = all(
            abs(c[1]) > self.cmd_moving_thresh_mps or abs(c[2]) > self.cmd_moving_thresh_radps
            for c in self.cmd_hist)

        if pose_stale and wheel_moved and cmd_significant:
            self._trigger(pos_range, yaw_range)

    def _trigger(self, pos_range: float, yaw_range: float):
        self.triggered = True
        self.stop_burst_until = self._now() + STOP_BURST_S

        wheel_speeds = [math.hypot(w[1], w[2]) for w in self.wheel_hist]
        cmd_speeds = [math.hypot(c[1], c[2]) for c in self.cmd_hist]
        self.get_logger().fatal(
            f"SAFETY WATCHDOG TRIGGERED: pose frozen for {self.window_s}s "
            f"(pos_range={pos_range:.4f}m, yaw_range={yaw_range:.4f}rad) while wheel encoders "
            f"(min/max speed={min(wheel_speeds):.3f}/{max(wheel_speeds):.3f}) and commanded "
            f"velocity (min/max speed={min(cmd_speeds):.3f}/{max(cmd_speeds):.3f}) both show "
            f"sustained motion -- OptiTrack pose feed is very likely stale/frozen. "
            f"Cancelling navigation and stopping the rover NOW.")

        try:
            self.nav.cancelTask()
        except Exception as e:
            self.get_logger().error(f"cancelTask() failed during safety stop: {e}")

        for _ in range(3):
            stop = Bool()
            stop.data = True
            self.safety_stop_pub.publish(stop)

    def _stop_burst_tick(self):
        if self.stop_burst_until is None or self._now() > self.stop_burst_until:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_pub.publish(msg)  # zero velocity by default construction


def main(args=None):
    rclpy.init(args=args)
    node = SafetyWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
