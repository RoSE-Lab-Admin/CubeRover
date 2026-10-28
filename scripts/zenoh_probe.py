#!/usr/bin/env python3
"""Times what gp_explorer does with ROS each trial: start, subscribe to the pose, get one
message, destroy the node, rclpy.shutdown(). Slow or failing shutdowns = the rmw_zenoh
"close operation timed out" problem. Run with and without the cameras streaming.

    python3 scripts/zenoh_probe.py            # 20 rounds
    python3 scripts/zenoh_probe.py -n 50 --topic /FitRosey_passive/pose
"""
import argparse
import time

import rclpy
from geometry_msgs.msg import PoseStamped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", type=int, default=20)
    ap.add_argument("--topic", default="/FitRosey_passive/pose")
    a = ap.parse_args()
    slow = failed = 0
    for i in range(1, a.n + 1):
        t0 = time.monotonic()
        rclpy.init()
        node = rclpy.create_node("zenoh_probe")
        got = []
        node.create_subscription(PoseStamped, a.topic, lambda m: got.append(1), 10)
        while not got and time.monotonic() - t0 < 10.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        t_msg = time.monotonic() - t0
        t1 = time.monotonic()
        err = ""
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            err, failed = f"  SHUTDOWN FAILED: {e}", failed + 1
        t_close = time.monotonic() - t1
        slow += t_close > 2.0
        print(f"[{i:3d}] first pose {t_msg:5.2f} s{'' if got else ' (none)'}   "
              f"shutdown {t_close:5.2f} s{err}", flush=True)
    print(f"{a.n} rounds: {slow} shutdowns > 2 s, {failed} failed")


if __name__ == "__main__":
    main()
