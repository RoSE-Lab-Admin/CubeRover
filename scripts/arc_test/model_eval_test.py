#!/usr/bin/env python3
"""Fixed test drives to compare trained configurations (e.g. rate limiter on/off)
with their final dynamics models. One bag per run; score with model_eval_analyze.py.

Nav2 must already be running (ros2 launch nav2_stack nav2.launch.py). Each
configuration is NAME=BAG_DIR:on|off -- the newest from_scratch_weights
checkpoint in BAG_DIR is loaded (neural_network), "on" = rate limiter on (the
Pi's limits 0.5 m/s^2 / 0.2117 rad/s^2 on the predicted velocities), "off" = off.

Every test is defined relative to the rover's pose when it starts (the rover
cannot be parked at a chosen heading), so each configuration drives exactly
the same maneuver:
  arc_*    : planned single arcs, radius R, 90 deg (75 if 90 does not fit)
  gentle_* : goal 3.0 m away at +-25 deg off the nose (one wide arc)
  sharp_*  : goal 2.5 m away at +-75 deg (tighter than arc mode -> Hybrid-A*)
  behind_* : goal 2.0 m away at +-150 deg (needs reversing / a 3-point turn)
Tests are interleaved: each test runs once per configuration back to back
(alternating which goes first), so battery/floor drift hits all equally. When
a test does not fit from the current pose, the rover first drives (unrecorded)
to one of the repositioning spots.

    python3 scripts/arc_test/model_eval_test.py --out ~/AL_scripts/model_eval_1013 \
        --config baseline=~/AL_scripts/<baseline_dir>:off \
        --config limiter=~/AL_scripts/<limiter_dir>:on --repeats 2
"""
import argparse
import csv
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arc_follow_test as aft  # noqa: E402  (helpers: Map, pick_arc, drive, ...)
from nav2_stack import autonomous_trials as at  # noqa: E402

# the limiter only limits the predicted velocities (rate_limiter_ax/az); the
# sampling limits ax/az stay loose -- clamping them too (first version, 10/13)
# kept MPPI from ever trying a hard turn
LIMITER = {
    'on': {'rate_limiter_enabled': True, 'ax_max': 2.0, 'ax_min': -2.0, 'az_max': 7.0,
           'rate_limiter_ax': 0.5, 'rate_limiter_az': 0.211666666667},
    'off': {'rate_limiter_enabled': False, 'ax_max': 2.0, 'ax_min': -2.0, 'az_max': 7.0},
}
# (name, kind, params)
TESTS = [
    ('arc_R1.55_left', 'arc', dict(R=1.55, side='left')),
    ('arc_R1.55_right', 'arc', dict(R=1.55, side='right')),
    ('arc_R1.70_left', 'arc', dict(R=1.70, side='left')),
    ('arc_R1.70_right', 'arc', dict(R=1.70, side='right')),
    ('gentle_left', 'rel', dict(d=3.0, bearing=25.0)),
    ('gentle_right', 'rel', dict(d=3.0, bearing=-25.0)),
    ('sharp_left', 'rel', dict(d=2.5, bearing=75.0)),
    ('sharp_right', 'rel', dict(d=2.5, bearing=-75.0)),
    ('behind_left', 'rel', dict(d=2.0, bearing=150.0)),
    ('behind_right', 'rel', dict(d=2.0, bearing=-150.0)),
]
ARC_SWEEPS = (90, 75)
GOAL_CLEAR = 1.0      # m from walls for a relative goal (like gp_explorer's min clearance)
LINE_CLEAR = 0.6      # m: the straight start->goal line must stay this far from walls


def rel_goal(pose, d, bearing_deg):
    a = pose[2] + math.radians(bearing_deg)
    return pose[0] + d * math.cos(a), pose[1] + d * math.sin(a)


def fits(pose, kind, p, amap):
    """Goal for this test from pose, or None if it does not fit the arena."""
    if kind == 'arc':
        c = aft.pick_arc(pose, p['R'], [p['side']], amap, 0.9, ARC_SWEEPS)
        return (c[2], c[1]) if c else None          # goal, sweep
    g = rel_goal(pose, p['d'], p['bearing'])
    line = [np.array(pose[:2]) + t * (np.array(g) - np.array(pose[:2])) for t in np.linspace(0, 1, 25)]
    if amap.clearance(*g) < GOAL_CLEAR or min(amap.clearance(*q) for q in line) < LINE_CLEAR:
        return None
    return g, None


def predicted_arrival(pose, target, min_radius=1.55, max_sweep=115.0):
    """Pose the rover arrives at if the planner drives the single arc (arc mode)
    from pose to target, or None when arc mode would not plan it (too tight,
    too much sweep): then the arrival heading is unpredictable."""
    x0, y0, h = pose
    gx, gy = target[0] - x0, target[1] - y0
    dx = math.cos(h) * gx + math.sin(h) * gy
    dy = -math.sin(h) * gx + math.cos(h) * gy
    d2 = dx * dx + dy * dy
    if d2 < 0.25 or abs(dx) < 1e-6:
        return None
    k = 2.0 * dy / d2
    sweep = abs(2.0 * math.atan(dy / dx))
    if abs(k) * min_radius > 1.0 or math.degrees(sweep) > max_sweep:
        return None
    return target[0], target[1], h + 2.0 * math.atan(dy / dx)


def arc_clear(pose, target, amap, n=25):
    """Smallest wall clearance along that arc."""
    x0, y0, h = pose
    gx, gy = target[0] - x0, target[1] - y0
    dx = math.cos(h) * gx + math.sin(h) * gy
    dy = -math.sin(h) * gx + math.cos(h) * gy
    k = 2.0 * dy / (dx * dx + dy * dy)
    s_goal = dx if abs(k) < 1e-9 else 2.0 * math.atan(dy / dx) / k
    out = []
    for s in np.linspace(0.0, s_goal, n):
        lx, ly = (s, 0.0) if abs(k) < 1e-9 else (math.sin(k * s) / k, (1 - math.cos(k * s)) / k)
        out.append(amap.clearance(x0 + math.cos(h) * lx - math.sin(h) * ly,
                                  y0 + math.sin(h) * lx + math.cos(h) * ly))
    return min(out)


def smart_reposition(pose, kind, p, amap):
    """Nearest free spot reachable by one arc from which this test fits on arrival."""
    best = None
    for x in np.arange(-2.6, 2.0, 0.2):
        for y in np.arange(-2.2, 1.8, 0.2):
            if amap.clearance(x, y) < 1.0:
                continue
            arr = predicted_arrival(pose, (x, y))
            if arr is None or arc_clear(pose, (x, y), amap) < 0.7 or fits(arr, kind, p, amap) is None:
                continue
            d = math.hypot(x - pose[0], y - pose[1])
            if best is None or d < best[0]:
                best = (d, (round(float(x), 2), round(float(y), 2)))
    return best[1] if best else None


def reload_robust(node, attempts=3):
    """RESET + STARTUP, the whole cycle retried. at.reload_controller() only
    retries STARTUP, which cannot succeed once the controller did come up but
    the manager's get_state call failed (seen live 10/13: "async_send_request
    failed", then "Transition is not registered ... from current state active")."""
    for i in range(1, attempts + 1):
        try:
            at.reload_controller(node)
            return
        except Exception as e:  # noqa: BLE001
            at.log(f'WARNING: controller reload {i}/{attempts} failed ({e}) -- full RESET+STARTUP again')
            time.sleep(3.0)
    raise RuntimeError(f'controller reload failed {attempts} times')


def parse_config(s):
    name, rest = s.split('=', 1)
    path, lim = rest.rsplit(':', 1)
    if lim not in LIMITER:
        raise SystemExit(f'--config {s}: limiter must be on or off')
    return name, Path(path).expanduser(), lim


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--config', action='append', required=True, help='NAME=BAG_DIR:on|off (repeat)')
    ap.add_argument('--repeats', type=int, default=2)
    ap.add_argument('--tests', nargs='*', help='subset of test names (default all)')
    ap.add_argument('--goal-tolerance', type=float, default=0.3)
    a = ap.parse_args()
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    configs = [parse_config(c) for c in a.config]
    tests = [t for t in TESTS if not a.tests or t[0] in a.tests]
    amap = aft.Map()

    rclpy.init()
    node = at.PoseWatcher('/FitRosey_V1/pose')
    ctrl_get = node.create_client(GetParameters, '/controller_server/get_parameters')
    at.wait_for_nav2_active()
    width = aft.get_param(node, ctrl_get, 'FollowPath.nn_hidden_width') or 128
    models = {}
    for name, d, lim in configs:
        ck = at.latest_from_scratch_checkpoint(d / 'from_scratch_weights', width)
        if ck is None:
            raise SystemExit(f'no mlp{width}_trialNN.pt in {d / "from_scratch_weights"}')
        models[name] = ck
        at.log(f'config {name}: model {ck}, rate limiter {lim}')
    keys = ['FollowPath.nn_model_path', 'FollowPath.dynamics_mode', 'goal_checker.xy_goal_tolerance'] + \
        [f'FollowPath.{k}' for k in LIMITER['on']]
    launched = {k: aft.get_param(node, ctrl_get, k) for k in keys}
    at.log(f'launched controller settings (restored at the end): {launched}')
    at.set_controller_param(node, 'goal_checker.xy_goal_tolerance', float(a.goal_tolerance))
    at.XY_GOAL_TOLERANCE = a.goal_tolerance

    runs_csv = out / 'runs.csv'
    new = not runs_csv.exists()
    done = {}   # (config, test) -> runs already recorded (resume after an abort)
    if not new:
        import collections
        done = collections.Counter((r['config'], r['test']) for r in csv.DictReader(open(runs_csv)))
        if done:
            at.log(f'resuming: {sum(done.values())} runs already in {runs_csv} are skipped')
    fcsv = open(runs_csv, 'a', newline='')
    w = csv.writer(fcsv)
    if new:
        w.writerow(['run', 'config', 'limiter', 'model', 'test', 'kind', 'radius', 'side', 'sweep_deg',
                    'rel_d', 'rel_bearing', 'start_x', 'start_y', 'start_yaw_deg', 'goal_x', 'goal_y',
                    'outcome', 't_start', 't_end'])
    active = None
    try:
        k = 0
        for rep in range(a.repeats):
            for tname, kind, p in tests:
                order = configs if k % 2 == 0 else configs[::-1]
                k += 1
                for cname, _, lim in order:
                    if done.get((cname, tname), 0) > rep:
                        continue
                    if active != cname:
                        at.log(f'--- controller -> {cname} (rate limiter {lim})')
                        at.set_controller_param(node, 'FollowPath.nn_model_path', str(models[cname]))
                        at.set_controller_param(node, 'FollowPath.dynamics_mode', 'neural_network')
                        for pk, pv in LIMITER[lim].items():
                            at.set_controller_param(node, f'FollowPath.{pk}', pv)
                        reload_robust(node)
                        active = cname
                    pose = aft.current_pose(node)
                    fit = fits(pose, kind, p, amap)
                    for i in range(aft.MAX_REPOSITIONS):
                        if fit is not None:
                            break
                        # a spot one arc away from which the test fits on arrival, else the fixed spots
                        tgt = smart_reposition(pose, kind, p, amap) or next(
                            q for q in aft.REPOSITION[i:] + aft.REPOSITION
                            if math.hypot(q[0] - pose[0], q[1] - pose[1]) > 0.8)
                        at.log(f'{tname}: does not fit from ({pose[0]:.2f}, {pose[1]:.2f}, '
                               f'{math.degrees(pose[2]):.0f} deg) -- repositioning to {tgt}')
                        aft.drive(node, out, 'reposition', tgt, record=False)
                        pose = aft.current_pose(node)
                        fit = fits(pose, kind, p, amap)
                    if fit is None:
                        at.log(f'{tname} / {cname}: still does not fit -- skipped')
                        continue
                    goal, sweep = fit
                    run = f'{datetime.now():%H%M%S}_{cname}_{tname}'
                    at.log(f'=== {run}: from ({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.0f} deg) '
                           f'to ({goal[0]:.2f}, {goal[1]:.2f})')
                    t0 = time.time()
                    outcome = aft.drive(node, out, run, goal, record=True)
                    w.writerow([run, cname, lim, models[cname].name, tname, kind, p.get('R', ''),
                                p.get('side', ''), sweep or '', p.get('d', ''), p.get('bearing', ''),
                                f'{pose[0]:.4f}', f'{pose[1]:.4f}', f'{math.degrees(pose[2]):.1f}',
                                f'{goal[0]:.4f}', f'{goal[1]:.4f}', outcome, f'{t0:.3f}', f'{time.time():.3f}'])
                    fcsv.flush()
                    at.log(f'=== {run}: {outcome}')
                    if outcome == 'safety_stop':
                        raise SystemExit('safety_watchdog stop -- aborting')
    finally:
        fcsv.close()
        at.log('restoring the launched controller settings')
        try:
            for key, v in launched.items():
                if v is not None:
                    at.set_controller_param(node, key, v)
            reload_robust(node)
        except Exception as e:  # noqa: BLE001
            at.log(f'WARNING: could not restore the controller settings ({e})')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
