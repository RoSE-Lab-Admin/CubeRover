#!/usr/bin/env python3
"""Drive the rover through a list of constant (vx, wz) commands to measure the
turning radius it actually achieves (analysed later from the bag with
turn_radius_analyze.py).

Each step: press Enter (reposition the rover first if needed), the command is
published at 10 Hz on /cmd_vel until the heading has changed by --max-turn
degrees or --duration seconds have passed, then zero is sent. The commanded
step is also published on /turn_test/step so the bag carries the labels.
Ctrl-C at any time sends zero and exits.

Nav2 / joystick must not be publishing /cmd_vel at the same time. Record with:

    ros2 bag record -o turn_radius_test /FitRosey_V1/pose /cmd_vel /dynamic_joint_states /turn_test/step

    python3 scripts/turn_radius_test.py [--only 3 4] [--duration 15] [--max-turn 120]
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

# (vx m/s, wz rad/s); positive wz = left
STEPS = [
    (0.20, 0.30), (0.20, 0.63),
    (0.12, 0.30), (0.12, 0.63),
    (0.08, 0.30), (0.08, 0.63),     # 0.08 = VelocityDeadbandCritic deadband; nothing slower
    (0.12, -0.63), (0.08, -0.63),   # right turns (symmetry check)
    (-0.08, 0.63),                  # reverse
]


class TurnTest(Node):
    def __init__(self):
        super().__init__('turn_radius_test')
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.step_pub = self.create_publisher(String, '/turn_test/step', 10)
        self.yaw = None
        self.create_subscription(PoseStamped, '/FitRosey_V1/pose', self._pose, 20)

    def _pose(self, m):
        q = m.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def send(self, vx, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'FitRosey_V1'
        m.twist.linear.x, m.twist.angular.z = float(vx), float(wz)
        self.cmd_pub.publish(m)

    def label(self, text):
        self.step_pub.publish(String(data=text))

    def stop(self):
        for _ in range(5):
            self.send(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', nargs='*', type=int, help='step numbers (1-based) to run')
    ap.add_argument('--duration', type=float, default=15.0)
    ap.add_argument('--max-turn', type=float, default=120.0, help='stop a step after this many degrees')
    a = ap.parse_args()
    rclpy.init()
    n = TurnTest()
    try:
        t = time.monotonic()
        while n.yaw is None and time.monotonic() - t < 5.0:
            rclpy.spin_once(n, timeout_sec=0.1)
        if n.yaw is None:
            print('WARNING: no /FitRosey_V1/pose -- steps will only stop on --duration')
        for i, (vx, wz) in enumerate(STEPS, 1):
            if a.only and i not in a.only:
                continue
            input(f'\nstep {i}/{len(STEPS)}: vx={vx:+.2f} wz={wz:+.2f}  -- Enter to start (Ctrl-C aborts)')
            n.label(f'start {i} {vx} {wz}')
            yaw0, prev, turned = n.yaw, n.yaw, 0.0
            t0 = time.monotonic()
            while time.monotonic() - t0 < a.duration and abs(turned) < math.radians(a.max_turn):
                n.send(vx, wz)
                end = time.monotonic() + 0.1
                while time.monotonic() < end:
                    rclpy.spin_once(n, timeout_sec=0.02)
                if yaw0 is not None and n.yaw is not None:
                    turned += math.atan2(math.sin(n.yaw - prev), math.cos(n.yaw - prev))
                    prev = n.yaw
            n.stop()
            n.label(f'end {i} {vx} {wz}')
            print(f'   done: {time.monotonic() - t0:.1f} s, heading change {math.degrees(turned):+.0f} deg')
    except KeyboardInterrupt:
        print('\naborted')
    finally:
        n.stop()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
