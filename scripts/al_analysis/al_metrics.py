#!/usr/bin/env python3
"""Per-trial metrics for active-learning / kinematics trial folders (autonomous_trials
bags), cached for the comparison notebook (al_comparison.ipynb).

Metrics per trial (bag):
  1. one-step error   : the model that drove the trial (kinematics, or the NN checkpoint
                        trained after the previous trial) predicting the motion over 0.1 s
                        from the applied command (cmd_vel_out) -- the same samples
                        dynamics_retrain trains on -- vs mocap. Yaw-rate and speed RMSE.
  2. 2.5 s error      : /optimal_trajectory_model end pose (MPPI's own belief of its
                        chosen trajectory) vs where the rover was 2.5 s later.
  3. 10 s plan error  : rolling, every /plan (1 Hz): mean distance of the next 10 s of
                        driving from that plan, and the end-point error -- real position
                        after 10 s vs the plan point at the same driven distance.
  4. distance         : driven length / first plan length, and / straight start-goal line.
  5. arcs and gears   : gear changes; share of replans with a reverse leg (3-point turns);
                        for trials whose first plan is one arc: max deviation from it while
                        > 1 m from the goal, and whether it was held (<= 0.25 m, no reverse).
  plus outcome, drive time, battery voltage and speed tracking (low-battery flag).

Run with the cuda_end python (torch + rosbags), ROS sourced:
    PYTHONPATH=~/CubeRover_waypoint/src/nav2_stack ~/cuda_end/bin/python \
        scripts/al_analysis/al_metrics.py NAME=FOLDER:kinematics|nn [...]
Writes ~/AL_scripts/al_analysis_cache/<NAME>.json (one entry per trial).
"""
import csv
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
import torch
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from nav2_stack import dynamics_retrain as dr

dr.DEVICE = torch.device('cpu')
CACHE = Path.home() / 'AL_scripts' / 'al_analysis_cache'
TOPICS = ['/FitRosey_V1/pose', '/cmd_vel', '/roseybot_base_controller/cmd_vel_out', '/plan',
          '/optimal_trajectory_model', '/dynamic_joint_states']
FMEAN = np.array([0.00372449, -0.03188930], np.float32)   # nav2_param2.yaml linear_model.fmean/fstd
FSTD = np.array([0.19342424, 0.31374899], np.float32)
N_BOOT = 5
PLAN_WINDOW = 10.0       # s
END_ZONE = 1.0           # m from the goal: arc deviation not counted (goal tolerance / end of arc)
MAX_DEV = 0.25           # m: arc "held"
SLOW_RATIO, LOW_VOLT = 0.88, 13.5


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    D = {'P': [], 'C': [], 'O': [], 'plans': [], 'opt': [], 'V': []}
    while r.has_next():
        tp, raw, t = r.read_next()
        t *= 1e-9
        m = deserialize_message(raw, get_message(types[tp]))
        if tp == TOPICS[0]:
            D['P'].append((t, m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)))
        elif tp == TOPICS[1]:
            D['C'].append((t, m.twist.linear.x, m.twist.angular.z))
        elif tp == TOPICS[2]:
            D['O'].append((t, m.twist.linear.x, m.twist.angular.z))
        elif tp == TOPICS[3] and m.poses:
            D['plans'].append((t, np.array([(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation))
                                            for p in m.poses])))
        elif tp == TOPICS[4] and m.poses:
            D['opt'].append((t, np.array([(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation))
                                          for p in m.poses])))
        elif tp == TOPICS[5]:
            v = [iv.values[list(iv.interface_names).index('voltage')] for iv in m.interface_values
                 if 'voltage' in iv.interface_names]
            if v:
                D['V'].append(float(np.mean(v)))
    for k in ('P', 'C', 'O'):
        D[k] = np.array(D[k]) if D[k] else np.zeros((0, 4 if k == 'P' else 3))
    return D


def dist_to_poly(pts, poly):
    a, b = poly[:-1], poly[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    out = np.empty(len(pts))
    for i, p in enumerate(pts):
        u = np.clip(((p - a) * ab).sum(1) / L2, 0, 1)
        out[i] = np.hypot(*(a + u[:, None] * ab - p).T).min()
    return out


def point_at_length(poly, s):
    seg = np.hypot(*np.diff(poly, axis=0).T)
    cs = np.r_[0, np.cumsum(seg)]
    if s >= cs[-1]:
        return poly[-1]
    i = int(np.searchsorted(cs, s)) - 1
    u = (s - cs[i]) / max(seg[i], 1e-9)
    return poly[i] + u * (poly[i + 1] - poly[i])


def has_reverse(plan):
    if len(plan) < 3:
        return False
    d = np.diff(plan[:, :2], axis=0)
    fwd = d[:, 0] * np.cos(plan[:-1, 2]) + d[:, 1] * np.sin(plan[:-1, 2])
    return bool((fwd < -1e-4).any())


def is_single_arc(plan):
    if len(plan) < 4 or has_reverse(plan):
        return False
    h = np.unwrap(plan[:, 2])
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(plan[:, :2], axis=0).T))]
    if s[-1] < 0.3:
        return False
    k = np.diff(h) / np.maximum(np.diff(s), 1e-6)
    return float(np.std(k)) < 0.05 + 0.1 * abs(float(np.mean(k)))


class ModelBank:
    """The dynamics model that drove each trial (None = kinematics)."""

    def __init__(self, folder, mode):
        self.folder, self.mode, self.cache = folder, mode, {}

    def for_trial(self, i):
        if self.mode == 'kinematics' or i <= N_BOOT:
            return None, 'kinematics'
        # newest checkpoint from before this trial: if a retrain failed, autonomous_trials
        # keeps driving on the previous model (e.g. no trial28.pt -> trial 29 used trial27.pt)
        p = None
        for k in range(i - 1, N_BOOT - 1, -1):
            q = self.folder / 'from_scratch_weights' / f'mlp128_trial{k:02d}.pt'
            if q.exists():
                p = q
                break
        if p is None:
            return None, 'kinematics (no checkpoint yet)'
        if p not in self.cache:
            self.cache[p] = torch.jit.load(str(p), map_location='cpu').eval()
        return self.cache[p], p.name


@torch.no_grad()
def one_step(model, segs):
    """yaw-rate / speed error (prediction - real) for every 0.1 s step, from cmd_vel_out."""
    rows = [sg['data'] for sg in segs or []]
    if not rows:
        return None
    d = np.concatenate(rows)
    d = d[1:] if len(d) > 1 else d
    cmd = d[:, [dr.COL_VX_CMD, dr.COL_WZ_CMD]].astype(np.float32)
    moving = (np.abs(cmd[:, 0]) > dr.STAT_VX) | (np.abs(cmd[:, 1]) > dr.STAT_WZ)
    if moving.sum() < 5:
        return None
    cmd, d = cmd[moving], d[moving]
    pf, py = cmd[:, 0] * dr.DT, cmd[:, 1] * dr.DT
    if model is not None:
        x = torch.from_numpy((cmd - FMEAN) / FSTD)
        corr = model(x).numpy()
        pf, py = pf + corr[:, 0], py + corr[:, 1]
    e_yaw = (py - d[:, dr.COL_DYAW]) / dr.DT
    e_v = (pf - d[:, dr.COL_DFWD]) / dr.DT
    return dict(n=int(len(e_yaw)), yaw_rmse=float(np.sqrt(np.mean(e_yaw ** 2))),
                speed_rmse=float(np.sqrt(np.mean(e_v ** 2))),
                yaw_bias=float(np.mean(e_yaw)))


def trial_metrics(bag, outcome, goal, model, model_name, typestore):
    D = read_bag(bag)
    P, C, O = D['P'], D['C'], D['O']
    r = dict(bag=bag.name, outcome=outcome, model=model_name)
    if len(P) < 50 or len(C) < 10:
        r['invalid'] = 'too little data'
        return r
    mv = np.abs(C[:, 1]) > 0.02
    tm = C[mv, 0]
    if len(tm) < 5:
        r['invalid'] = 'no motion commands'
        return r
    t0, t1 = tm[0], tm[-1]
    r['time'] = float(t1 - t0)
    g = np.sign(C[mv, 1])
    r['gears'] = int((np.diff(g) != 0).sum())
    sel = (P[:, 0] >= t0) & (P[:, 0] <= t1)
    track = P[sel]
    driven = float(np.hypot(*np.diff(track[:, 1:3], axis=0).T).sum()) if len(track) > 1 else 0.0
    r['driven'] = driven
    plans = [(t, p) for t, p in D['plans'] if len(p) >= 2]
    if plans:
        p0 = plans[0][1]
        plen = float(np.hypot(*np.diff(p0[:, :2], axis=0).T).sum())
        r['plan0_len'] = plen
        r['path_ratio_plan'] = driven / max(plen, 0.3)
    start = track[0, 1:3] if len(track) else P[0, 1:3]
    if goal is not None:
        r['straight'] = float(np.hypot(*(np.array(goal) - start)))
        r['path_ratio_straight'] = driven / max(r['straight'], 0.3)
    r['rev_plan_share'] = float(np.mean([has_reverse(p) for _, p in plans])) if plans else np.nan

    # 2. MPPI's 2.5 s belief vs reality
    eh, ep = [], []
    for t, traj in D['opt']:
        if t < t0 or t > t1 or len(traj) < 2:
            continue
        b = np.searchsorted(P[:, 0], t + 0.1 * len(traj))
        if b >= len(P):
            continue
        eh.append(abs(math.degrees(math.remainder(traj[-1, 2] - P[b, 3], 2 * math.pi))))
        ep.append(math.hypot(traj[-1, 0] - P[b, 1], traj[-1, 1] - P[b, 2]))
    r['pred25_head_deg'] = float(np.mean(eh)) if eh else np.nan
    r['pred25_pos_m'] = float(np.mean(ep)) if ep else np.nan

    # 3. rolling 10 s vs /plan
    xt_mean, end_err = [], []
    for t, plan in plans:
        if t < t0 or t + PLAN_WINDOW > t1:
            continue
        w = (P[:, 0] >= t) & (P[:, 0] <= t + PLAN_WINDOW)
        seg = P[w]
        if len(seg) < 20:
            continue
        xt_mean.append(float(dist_to_poly(seg[::10, 1:3], plan[:, :2]).mean()))
        s_drv = float(np.hypot(*np.diff(seg[:, 1:3], axis=0).T).sum())
        end_err.append(float(np.hypot(*(point_at_length(plan[:, :2], s_drv) - seg[-1, 1:3]))))
    r['plan10_xtrack_m'] = float(np.mean(xt_mean)) if xt_mean else np.nan
    r['plan10_end_m'] = float(np.mean(end_err)) if end_err else np.nan
    r['plan10_n'] = len(xt_mean)

    # 5. arcs
    if plans and is_single_arc(plans[0][1]):
        arc = plans[0][1][:, :2]
        gpt = arc[-1] if goal is None else np.array(goal)
        far = np.hypot(track[:, 1] - gpt[0], track[:, 2] - gpt[1]) > END_ZONE
        t_rev = next((t for t, p in plans if has_reverse(p) and t <= t1), None)
        upto = track[far & ((track[:, 0] <= t_rev) if t_rev is not None else True)]
        dev = dist_to_poly(upto[::10, 1:3], arc) if len(upto) > 5 else np.array([0.0])
        r['arc_trial'] = True
        r['arc_dev_max'] = float(dev.max())
        r['arc_held'] = bool(dev.max() <= MAX_DEV and t_rev is None)
    else:
        r['arc_trial'] = False

    # 1. one-step error of the model that drove this trial
    segs, _ = dr.extract_trajectory(bag, typestore, None, False)
    os_ = one_step(model, segs)
    if os_:
        r.update({f'step1_{k}': v for k, v in os_.items()})

    # battery / speed tracking
    ratios = []
    yw = np.unwrap(P[:, 3])
    for t, vx, wz in (O if len(O) else C):
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
    r['low_batt'] = bool(r['speed_ratio'] < SLOW_RATIO or r['volt'] < LOW_VOLT)
    return r


@torch.no_grad()
def final_model_on(model, folder, typestore):
    """one-step error of a fixed model on every trial of another folder (cross-check)."""
    out = []
    for b in sorted(folder.glob('bag_*')):
        if (b / 'metadata.yaml').exists():
            segs, _ = dr.extract_trajectory(b, typestore, None, False)
            o = one_step(model, segs)
            if o:
                out.append(o)
    if not out:
        return None
    n = np.array([o['n'] for o in out])
    return dict(n=int(n.sum()),
                yaw_rmse=float(np.sqrt(np.sum(n * np.array([o['yaw_rmse'] for o in out]) ** 2) / n.sum())),
                speed_rmse=float(np.sqrt(np.sum(n * np.array([o['speed_rmse'] for o in out]) ** 2) / n.sum())))


def main():
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_JAZZY)
    CACHE.mkdir(parents=True, exist_ok=True)
    specs = []
    for a in sys.argv[1:]:
        name, rest = a.split('=', 1)
        folder, mode = rest.rsplit(':', 1)
        specs.append((name, Path(folder).expanduser(), mode))
    for name, folder, mode in specs:
        bank = ModelBank(folder, mode)
        outcomes, goals = {}, {}
        for row in csv.DictReader(open(folder / 'trial_outcomes.csv')):
            outcomes[row['bag']] = row['outcome']
            goals[row['bag']] = (float(row['goal_x']), float(row['goal_y']))
        bags = sorted((b for b in folder.glob('bag_*') if (b / 'metadata.yaml').exists()),
                      key=lambda b: int(re.findall(r'\d+', b.name)[0]))
        res = []
        for b in bags:
            i = int(re.findall(r'\d+', b.name)[0])
            model, mname = bank.for_trial(i)
            r = trial_metrics(b, outcomes.get(b.name, '?'), goals.get(b.name), model, mname, ts)
            r['i'] = i
            res.append(r)
            print(f"{name:10s} {b.name}: {r.get('invalid') or 'ok'}  model {mname}", flush=True)
        meta = dict(name=name, folder=str(folder), mode=mode, trials=res)
        # the run's final model on every folder (NN runs only): is the learned model itself better?
        if mode == 'nn':
            fm = sorted((folder / 'from_scratch_weights').glob('mlp128_trial*.pt'),
                        key=lambda p: int(re.findall(r'\d+', p.name)[-1]))
            if fm:
                model = torch.jit.load(str(fm[-1]), map_location='cpu').eval()
                meta['final_model'] = fm[-1].name
                meta['final_model_on'] = {n2: final_model_on(model, f2, ts) for n2, f2, _ in specs}
        json.dump(meta, open(CACHE / f'{name}.json', 'w'), indent=1, default=float)
        print(f'wrote {CACHE / (name + ".json")}', flush=True)


if __name__ == '__main__':
    main()
