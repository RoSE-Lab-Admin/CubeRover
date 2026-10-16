#!/usr/bin/env python3
"""Kinematic stand-in for the rover + mocap, for offline MPPI tests.

Integrates the pose from /cmd_vel (TwistStamped, as the base controller gets
it) and publishes what the live stack sees from mocap and path_follower:
  - TF world -> FitRosey_V1
  - /FitRosey_V1/pose                       (planner gear seeding)
  - /odometry/filtered                      (body-frame twist from the last 5
                                             poses, like path_follower.vel_interp)
  - /roseybot_base_controller/cmd_vel_out   (the command actually applied)

Base controller rate limiter (on by default, like the real diff_drive_controller
with the Pi's live config): every 1/limit_rate s the applied command moves
toward the latest /cmd_vel by at most accel_vx / accel_wz per second, and is
held in between; /roseybot_base_controller/cmd_vel_out is that applied command.

yaw_scale (default 1.0 = pure kinematics) scales the rover's yaw response to
the applied wz (~0.24 measured over all bags, 10/09), so a rover that turns
less than commanded can be tried without code changes.
Reset the pose by publishing a PoseStamped on /sim/reset_pose.
"""
import collections
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class RoverSim(Node):
    def __init__(self):
        super().__init__('rover_sim')
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('yaw_scale', 1.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('start', [0.0, 0.0, 0.0])
        self.declare_parameter('frame', 'FitRosey_V1')
        self.declare_parameter('rate_limiter', True)
        self.declare_parameter('limit_rate', 10.0)       # Hz, controller_manager update_rate
        self.declare_parameter('accel_vx', 0.5)          # m/s^2, linear.x.max_acceleration (Pi)
        self.declare_parameter('accel_wz', 0.211666666667)  # rad/s^2, angular.z.max_acceleration
        self.applied = (0.0, 0.0)
        self.next_limit = None
        x, y, yaw = self.get_parameter('start').value
        self.state = [x, y, yaw]
        self.cmd = (0.0, 0.0)
        self.cmd_time = None
        self.frame = self.get_parameter('frame').value
        self.hist = collections.deque(maxlen=5)  # (t, x, y, yaw), like path_follower
        self.tf = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, f'/{self.frame}/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.out_pub = self.create_publisher(TwistStamped, '/roseybot_base_controller/cmd_vel_out', 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(PoseStamped, '/sim/reset_pose', self.on_reset, 10)
        self.dt = 1.0 / self.get_parameter('rate').value
        self.last = self.get_clock().now()
        self.create_timer(self.dt, self.step)

    def on_cmd(self, msg):
        self.cmd = (msg.twist.linear.x, msg.twist.angular.z)
        self.cmd_time = self.get_clock().now()

    def on_reset(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.state = [msg.pose.position.x, msg.pose.position.y, yaw]
        self.cmd, self.cmd_time = (0.0, 0.0), None
        self.applied = (0.0, 0.0)
        self.hist.clear()
        self.get_logger().info(f'reset to ({self.state[0]:.2f}, {self.state[1]:.2f}, '
                               f'{math.degrees(yaw):.1f} deg)')

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds * 1e-9
        self.last = now
        vx, wz = self.cmd
        timeout = self.get_parameter('cmd_timeout').value
        if self.cmd_time is None or (now - self.cmd_time).nanoseconds * 1e-9 > timeout:
            vx, wz = 0.0, 0.0
        if self.get_parameter('rate_limiter').value:
            # base controller update: step the applied command toward the target
            t = now.nanoseconds * 1e-9
            period = 1.0 / self.get_parameter('limit_rate').value
            if self.next_limit is None or t >= self.next_limit:
                self.next_limit = t + period
                ax = self.get_parameter('accel_vx').value * period
                az = self.get_parameter('accel_wz').value * period
                avx, awz = self.applied
                self.applied = (avx + max(-ax, min(ax, vx - avx)), awz + max(-az, min(az, wz - awz)))
            vx, wz = self.applied
        else:
            self.applied = (vx, wz)
        applied_vx, applied_wz = vx, wz
        wz *= self.get_parameter('yaw_scale').value
        x, y, yaw = self.state
        # exact arc integration
        if abs(wz) > 1e-9:
            x += vx / wz * (math.sin(yaw + wz * dt) - math.sin(yaw))
            y -= vx / wz * (math.cos(yaw + wz * dt) - math.cos(yaw))
        else:
            x += vx * dt * math.cos(yaw)
            y += vx * dt * math.sin(yaw)
        yaw = math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt))
        self.state = [x, y, yaw]
        self.publish(now, applied_vx, applied_wz)   # cmd_vel_out = applied command, before the rover's response

    def publish(self, now, vx_applied, wz_applied):
        x, y, yaw = self.state
        stamp = now.to_msg()
        qx, qy, qz, qw = yaw_quat(yaw)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'world'
        t.child_frame_id = self.frame
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf.sendTransform(t)

        p = PoseStamped()
        p.header.stamp = stamp
        p.header.frame_id = 'world'
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = qz, qw
        self.pose_pub.publish(p)

        # odometry like path_follower.opti_callback: pose + twist from the
        # last 5 poses, rotated into the body frame
        self.hist.append((now.nanoseconds * 1e-9, x, y, yaw))
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = self.frame
        o.pose.pose = p.pose
        if len(self.hist) == self.hist.maxlen:
            t0, x0, y0, h0 = self.hist[0]
            t1, x1, y1, h1 = self.hist[-1]
            T = t1 - t0
            if T > 0:
                vxw, vyw = (x1 - x0) / T, (y1 - y0) / T
                o.twist.twist.linear.x = math.cos(h1) * vxw + math.sin(h1) * vyw
                o.twist.twist.linear.y = -math.sin(h1) * vxw + math.cos(h1) * vyw
                o.twist.twist.angular.z = math.remainder(h1 - h0, 2 * math.pi) / T
        self.odom_pub.publish(o)

        out = TwistStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.frame
        out.twist.linear.x, out.twist.angular.z = vx_applied, wz_applied
        self.out_pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(RoverSim())


if __name__ == '__main__':
    main()
