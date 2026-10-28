#!/usr/bin/env python3
"""Compare active-learning (train_from_scratch) runs, e.g. rate limiter on/off.

Per trial bag: outcome (trial_outcomes.csv), drive time (first to last motion
command), driven length / first plan length, gear changes, share of replans
that contain a reverse segment (3-point turns), turn-command jitter, MPPI
prediction error (/optimal_trajectory_model vs. where the rover was 2.5 s
later), battery voltage and speed tracking (real speed / applied command).
Trials with a low battery (slow rover) are flagged and left out of the means.
Trials 1-5 drive on kinematics (bootstrap) and are reported separately.

    source install/setup.bash
    python3 scripts/arc_test/al_compare.py NAME=BAG_DIR [NAME=BAG_DIR ...]
"""
import csv
import math
import re
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# bags before 2026-10-27 recorded the rigid body under its old Motive name, /FitRosey_V1
POSE_TOPICS = ('/FitRosey_passive/pose', '/FitRosey_V1/pose')
TOPICS = ['/FitRosey_passive/pose', '/cmd_vel', '/roseybot_base_controller/cmd_vel_out', '/plan',
          '/optimal_trajectory_model', '/dynamic_joint_states']
SLOW_RATIO, LOW_VOLT = 0.88, 13.5   # speed tracking / battery limits for a usable trial
N_BOOT = 5


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS + list(POSE_TOPICS[1:]) if t in types]))
    D = {k: [] for k in ('P', 'C', 'O', 'plans', 'opt', 'V')}
    while r.has_next():
        tp, raw, t = r.read_next()
        t *= 1e-9
        m = deserialize_message(raw, get_message(types[tp]))
        if tp in POSE_TOPICS:
            D['P'].append((t, m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation)))
        elif tp == TOPICS[1]:
            D['C'].append((t, m.twist.linear.x, m.twist.angular.z))
        elif tp == TOPICS[2]:
            D['O'].append((t, m.twist.linear.x, m.twist.angular.z))
        elif tp == TOPICS[3] and m.poses:
            D['plans'].append(np.array([(p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation))
                                        for p in m.poses]))
        elif tp == TOPICS[4] and m.poses:
            D['opt'].append((t, np.array([(p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation))
                                          for p in m.poses])))
        elif tp == TOPICS[5]:
            v = [iv.values[list(iv.interface_names).index('voltage')] for iv in m.interface_values
                 if 'voltage' in iv.interface_names]
            if v:
                D['V'].append(np.mean(v))
    for k in ('P', 'C', 'O'):
        D[k] = np.array(D[k])
    return D


def has_reverse(plan):
    if len(plan) < 3:
        return False
    d = np.diff(plan[:, :2], axis=0)
    fwd = d[:, 0] * np.cos(plan[:-1, 2]) + d[:, 1] * np.sin(plan[:-1, 2])
    return bool((fwd < -1e-4).any() and (fwd > 1e-4).any()) or bool((fwd < -1e-4).all())


def trial(bag, outcome):
    D = read(bag)
    P, C, O = D['P'], D['C'], D['O']
    r = dict(bag=bag.name, outcome=outcome)
    if len(P) < 50 or len(C) < 10:
        r['invalid'] = True
        return r
    mv = np.abs(C[:, 1]) > 0.02
    tm = C[mv, 0]
    r['time'] = float(tm[-1] - tm[0]) if len(tm) > 1 else np.nan
    g = np.sign(C[mv, 1])
    r['gears'] = int((np.diff(g) != 0).sum()) if len(g) > 1 else 0
    r['jitter'] = float(np.abs(np.diff(C[mv, 2])).mean()) if mv.sum() > 2 else np.nan
    sel = (P[:, 0] >= tm[0]) & (P[:, 0] <= tm[-1]) if len(tm) > 1 else slice(None)
    drv = np.hypot(*np.diff(P[sel, 1:3], axis=0).T).sum()
    plan0 = D['plans'][0] if D['plans'] else None
    r['path_ratio'] = float(drv / max(np.hypot(*np.diff(plan0[:, :2], axis=0).T).sum(), 0.3)) \
        if plan0 is not None else np.nan
    r['rev_plans'] = float(np.mean([has_reverse(p) for p in D['plans']])) if D['plans'] else np.nan
    eh, ep = [], []
    for t, traj in D['opt']:
        b = np.searchsorted(P[:, 0], t + 0.1 * len(traj))
        if b >= len(P) or len(traj) < 2:
            continue
        eh.append(abs(math.degrees(math.remainder(traj[-1, 2] - P[b, 3], 2 * math.pi))))
        ep.append(math.hypot(traj[-1, 0] - P[b, 1], traj[-1, 1] - P[b, 2]))
    r['pred_head'] = float(np.mean(eh)) if eh else np.nan
    r['pred_pos'] = float(np.mean(ep)) if ep else np.nan
    # battery / speed tracking: real forward speed vs applied command on straight stretches
    ratios = []
    src = O if len(O) else C
    yw = np.unwrap(P[:, 3])
    for t, vx, wz in src:
        if abs(vx) < 0.15 or abs(wz) > 0.2:
            continue
        a, b = np.searchsorted(P[:, 0], [t - 0.25, t + 0.25])
        b = min(b, len(P) - 1)
        if b - a < 5:
            continue
        h = 0.5 * (yw[a] + yw[b])
        ratios.append(((P[b, 1] - P[a, 1]) * math.cos(h) + (P[b, 2] - P[a, 2]) * math.sin(h))
                      / (P[b, 0] - P[a, 0]) / vx)
    r['speed_ratio'] = float(np.median(ratios)) if len(ratios) >= 5 else np.nan
    r['volt'] = float(np.median(D['V'])) if D['V'] else np.nan
    r['low_batt'] = bool((r['speed_ratio'] < SLOW_RATIO) or (r['volt'] < LOW_VOLT))
    return r


def summarize(rs, label):
    keys = ['time', 'gears', 'path_ratio', 'rev_plans', 'jitter', 'pred_head', 'pred_pos']
    n = len(rs)
    if not n:
        return
    m = {k: np.nanmean([r[k] for r in rs]) for k in keys}
    reached = sum(r['outcome'] == 'reached' for r in rs)
    print(f"  {label:18s} n={n:2d}  reached {reached:2d}/{n:<2d} ({100 * reached / n:3.0f}%)  time {m['time']:5.1f} s  "
          f"gears {m['gears']:4.1f}  path/plan {m['path_ratio']:4.2f}  reverse plans {100 * m['rev_plans']:3.0f}%  "
          f"jitter {m['jitter']:.3f}  pred err {m['pred_head']:4.1f} deg / {m['pred_pos']:.2f} m")


def main():
    runs = [a.split('=', 1) for a in sys.argv[1:]]
    allres = {}
    for name, d in runs:
        d = Path(d).expanduser()
        outcomes = {row['bag']: row['outcome'] for row in csv.DictReader(open(d / 'trial_outcomes.csv'))}
        bags = sorted((b for b in d.glob('bag_*') if (b / 'metadata.yaml').exists()),
                      key=lambda b: int(re.findall(r'\d+', b.name)[0]))
        res = []
        for b in bags:
            r = trial(b, outcomes.get(b.name, '?'))
            r['i'] = int(re.findall(r'\d+', b.name)[0])
            if not r.get('invalid'):
                res.append(r)
        allres[name] = res
        low = [r for r in res if r['low_batt']]
        txt = ', '.join(f"{r['bag']} ({r['speed_ratio']:.2f}, {r['volt']:.1f} V)" for r in low) or 'none'
        print(f'\n== {name} ({d.name}): {len(res)} trials; low battery / slow (excluded below): {txt}')
    for name, res in allres.items():
        ok = [r for r in res if not r['low_batt']]
        print(f'\n== {name}')
        summarize([r for r in ok if r['i'] <= N_BOOT], 'bootstrap 1-5')
        summarize([r for r in ok if r['i'] > N_BOOT], 'NN trials 6-50')
        for lo in range(6, 51, 15):
            summarize([r for r in ok if lo <= r['i'] < lo + 15], f'  trials {lo}-{min(lo + 14, 50)}')


if __name__ == '__main__':
    main()
