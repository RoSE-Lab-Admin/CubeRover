#!/usr/bin/env python3
"""Scores model_eval_test.py runs (<out>/runs.csv + bags) per configuration.

Per run: reached, time, gear changes, planner "arc rejected" count (switches to
Hybrid-A*), driven path length / straight-line distance, distance from the
first plan (mean / max), and MPPI prediction error: the predicted trajectory
(/optimal_trajectory_model) vs where the rover actually was 2.5 s later.
Arc tests also get the arc_follow_analyze "held" verdict.

    source install/setup.bash
    python3 scripts/arc_test/model_eval_analyze.py ~/AL_scripts/model_eval_1013
"""
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, str(Path(__file__).resolve().parent))
import al_compare  # noqa: E402  (battery / speed-tracking check)
import arc_follow_analyze as A  # noqa: E402

# bags before 2026-10-27 recorded the rigid body under its old Motive name, /FitRosey_V1
POSE_TOPICS = ('/FitRosey_passive/pose', '/FitRosey_V1/pose')
TOPICS = POSE_TOPICS + ( '/cmd_vel', '/plan', '/rosout', '/optimal_trajectory_model')


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    P, C, plans, logs, opt = [], [], [], [], []
    while r.has_next():
        topic, raw, t = r.read_next()
        m = deserialize_message(raw, get_message(types[topic]))
        t *= 1e-9
        if topic in POSE_TOPICS:
            P.append((t, m.pose.position.x, m.pose.position.y, A.yaw(m.pose.orientation)))
        elif topic == '/cmd_vel':
            C.append((t, m.twist.linear.x, m.twist.angular.z))
        elif topic == '/plan' and m.poses:
            plans.append((t, np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses])))
        elif topic == '/optimal_trajectory_model' and m.poses:
            opt.append((t, np.array([(p.pose.position.x, p.pose.position.y, A.yaw(p.pose.orientation))
                                     for p in m.poses])))
        elif topic == '/rosout' and m.name == 'planner_server' and 'GridBasedCustom: arc' in m.msg:
            logs.append((m.stamp.sec + m.stamp.nanosec * 1e-9, m.msg))
    return np.array(P), np.array(C), plans, logs, opt


def score(row, bag):
    P, C, plans, logs, opt = read(bag)
    t0 = float(row['t_start'])
    plans = [p for p in plans if p[0] >= t0]
    logs = [l for l in logs if l[0] >= t0]
    r = dict(run=row['run'], config=row['config'], test=row['test'], kind=row['kind'],
             reached=row['outcome'] == 'reached', time=float(row['t_end']) - t0)
    if len(P) < 10 or not plans or r['time'] < 5:
        r['invalid'] = True
        return r
    mv = np.r_[False, np.hypot(*np.diff(P[:, 1:3], axis=0).T) > 1e-4]
    Pm = P[mv] if mv.sum() > 10 else P
    goal = np.array([float(row['goal_x']), float(row['goal_y'])])
    start = np.array([float(row['start_x']), float(row['start_y'])])
    r['path_ratio'] = float(np.hypot(*np.diff(Pm[:, 1:3], axis=0).T).sum() / max(np.hypot(*(goal - start)), 0.1))
    dev = A.dist_to_poly(Pm[::5, 1:3], plans[0][1])
    r['dev_mean'], r['dev_max'] = float(dev.mean()), float(dev.max())
    r['rejects'] = sum('rejected' in m for _, m in logs)
    if len(C):
        g = np.sign(C[:, 1][np.abs(C[:, 1]) > 0.02])
        r['gears'] = int((np.diff(g) != 0).sum()) if len(g) > 1 else 0
        mvc = np.abs(C[:, 1]) > 0.02
        r['wz_jitter'] = float(np.abs(np.diff(C[mvc, 2])).mean()) if mvc.sum() > 2 else np.nan
    # prediction error: predicted end pose vs real pose after the same time (bag clock for both)
    eh, ep = [], []
    for t, traj in opt:
        if t < t0 or len(traj) < 2:
            continue
        b = np.searchsorted(P[:, 0], t + 0.1 * len(traj))
        if b >= len(P):
            continue
        eh.append(abs(math.degrees(math.remainder(traj[-1, 2] - P[b, 3], 2 * math.pi))))
        ep.append(math.hypot(traj[-1, 0] - P[b, 1], traj[-1, 1] - P[b, 2]))
    r['pred_head'] = float(np.mean(eh)) if eh else np.nan
    r['pred_pos'] = float(np.mean(ep)) if ep else np.nan
    if row['kind'] == 'arc':
        t_fail = next((t for t, m in logs if 'rejected' in m and
                       math.hypot(*(P[min(np.searchsorted(P[:, 0], t), len(P) - 1), 1:3] - goal)) > A.END_ZONE),
                      None)
        upto = Pm[Pm[:, 0] <= t_fail] if t_fail is not None else Pm
        d2 = A.dist_to_poly(upto[::5, 1:3], plans[0][1]) if len(upto) > 5 else np.array([np.inf])
        r['held'] = bool(t_fail is None and d2.max() <= A.MAX_DEV)
    return r


def main():
    out = Path(sys.argv[1]).expanduser()
    res = []
    for row in csv.DictReader(open(out / 'runs.csv')):
        bag = out / row['run']
        if not bag.exists():
            continue
        r = score(row, bag)
        if r.get('invalid'):
            print(f"invalid (nothing driven): {r['run']}")
            continue
        b = al_compare.trial(bag, row['outcome'])
        if b.get('low_batt'):
            print(f"excluded, rover too slow / battery low: {r['run']} (speed {b['speed_ratio']:.2f} of "
                  f"command, {b['volt']:.1f} V)")
            continue
        res.append(r)
    keys = ('time', 'gears', 'rejects', 'path_ratio', 'dev_max', 'wz_jitter', 'pred_head', 'pred_pos')
    print(f"{'run':42s} reach  time gears rej  path  devX  jit  predH predP held")
    for r in res:
        print(f"{r['run']:42s} {('yes' if r['reached'] else 'NO'):5s} {r['time']:5.0f} {r.get('gears', 0):5d} "
              f"{r['rejects']:3d} {r['path_ratio']:5.2f} {r['dev_max']:5.2f} {r.get('wz_jitter', np.nan):5.3f} "
              f"{r['pred_head']:5.1f} {r['pred_pos']:5.2f} {'' if 'held' not in r else ('YES' if r['held'] else 'no')}")
    configs = sorted({r['config'] for r in res})
    for group in ('all', 'arc', 'rel:gentle', 'rel:sharp', 'rel:behind'):
        sel = (lambda r: True) if group == 'all' else (lambda r, g=group: r['kind'] == 'arc') if group == 'arc' \
            else (lambda r, g=group.split(':')[1]: r['test'].startswith(g))
        print(f'\n== {group}')
        print(f"{'config':12s} {'n':>3s} reached held   time  gears rejects path  devX  jitter predH  predP")
        for c in configs:
            rs = [r for r in res if r['config'] == c and sel(r)]
            if not rs:
                continue
            m = {k: np.nanmean([r.get(k, np.nan) for r in rs]) for k in keys}
            held = [r['held'] for r in rs if 'held' in r]
            print(f"{c:12s} {len(rs):3d} {sum(r['reached'] for r in rs):3d}/{len(rs):<3d} "
                  f"{(str(sum(held)) + '/' + str(len(held))) if held else '-':6s} {m['time']:5.0f} {m['gears']:6.1f} "
                  f"{m['rejects']:6.1f} {m['path_ratio']:5.2f} {m['dev_max']:5.2f} {m['wz_jitter']:6.3f} "
                  f"{m['pred_head']:5.1f} {m['pred_pos']:6.2f}")


if __name__ == '__main__':
    main()
