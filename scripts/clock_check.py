#!/usr/bin/env python3
"""Clock sync check, run on Slade (the Pi and NUC sync to Slade's chrony).

  1. Slade's NTP server listening on UDP 123? (after a WSL restart it can fail
     to open the port -- then nobody can sync; fix: sudo systemctl restart chrony)
  2. Pi clock offset, from /dynamic_joint_states header stamps (stamped on the Pi)
  3. NUC clock offset, from /cmd_vel stamps -- only while teleop is publishing

    python3 scripts/clock_check.py [--seconds 4]
Exit code 0 = all fine, 1 = something is off.
"""
import argparse
import os
import statistics
import subprocess
import sys
import time

OK_S = 0.1   # offsets above this are reported as off (the base controller drops commands > 0.5 s old)


def ntp_listening():
    out = subprocess.run(['ss', '-uln'], capture_output=True, text=True).stdout
    return any(line.split()[3].endswith(':123') for line in out.splitlines()[1:] if len(line.split()) > 3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=4.0)
    a = ap.parse_args()
    bad = False
    if ntp_listening():
        print('Slade NTP server : listening on UDP 123  OK')
    else:
        bad = True
        print('Slade NTP server : NOT listening on UDP 123 -> run: sudo systemctl restart chrony')

    try:
        import rclpy
        from control_msgs.msg import DynamicJointState
        from geometry_msgs.msg import TwistStamped
    except ImportError:
        print('(ROS not sourced -- skipping the Pi/NUC offsets)')
        return 1 if bad else 0
    rclpy.init()
    node = rclpy.create_node('clock_check')
    got = {'Pi  (/dynamic_joint_states)': [], 'NUC (/cmd_vel, teleop only)': []}

    def cb(key):
        def f(m):
            s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if s > 0:
                got[key].append(time.time() - s)
        return f
    node.create_subscription(DynamicJointState, '/dynamic_joint_states', cb('Pi  (/dynamic_joint_states)'), 10)
    node.create_subscription(TwistStamped, '/cmd_vel', cb('NUC (/cmd_vel, teleop only)'), 10)
    t_end = time.time() + a.seconds
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    for key, v in got.items():
        if not v:
            print(f'{key}: no messages ({"rover/bridge not running?" if key.startswith("Pi") else "not driving with the joystick"})')
            continue
        off = statistics.median(v)
        state = 'OK' if abs(off) < OK_S else ('BEHIND' if off > 0 else 'AHEAD')
        if state != 'OK':
            bad = True
        human = f'{off / 3600:+.1f} h' if abs(off) > 3600 else f'{off:+.3f} s'
        print(f'{key}: clock {state:6s} (Slade - stamp = {human}; includes ~ms of network delay)')
    if bad:
        print('\nTo fix: on Slade `sudo systemctl restart chrony`; then on the Pi/NUC '
              '`sudo chronyc burst 4/4; sleep 10; sudo chronyc makestep`')
    node.destroy_node()
    rclpy.shutdown()
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
