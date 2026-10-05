"""Get the rover unstuck when its footprint is in or against an obstacle.

The planner (smac GridBasedCustom) and the MPPI controller both give up when
the rover's centre is in INSCRIBED/LETHAL cost or its footprint touches a
LETHAL cell (StartOccupied / "Optimizer fail to compute path"), and Nav2's own
BackUp/DriveOnHeading refuse to move a footprint that is already in collision.
This helper detects exactly that condition and drives the rover out:

  1. stuck check: the planner's rule, on the global costmap at the mocap pose
  2. pick an escape motion: forward/backward x straight/arc-left/arc-right
     (arcs R = ARC_RADIUS). A candidate is valid if its swept footprint never
     reaches an obstacle other than the one(s) it already touches, never
     overlaps them more than it does now, and becomes un-stuck within
     MAX_DIST. The valid one with the most clearance at the point where it
     becomes un-stuck wins -- at a wall that is the arc turning away from it.
     No valid candidate -> no escape.
  3. drive it at SPEED, asking the planner every STEP whether it can plan to
     the goal again; stop as soon as it can (or on MAX_DIST / TIMEOUT / a new
     obstacle ahead), so Nav2 takes over with as little extra driving as
     possible.

The caller supplies a node that is NOT in a persistent executor (this spins
it; see CLAUDE.md) and a get_pose() -> (x, y, yaw) or None, kept current by
something else (or by spinning that same node).
"""
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rclpy.action import ActionClient
from scipy.ndimage import distance_transform_edt, label

LETHAL = 254
INSCRIBED = 253
FOOTPRINT_HALF_L = 0.37     # m, nav2_param2.yaml footprint [[0.37, 0.23], ...]
FOOTPRINT_HALF_W = 0.23
SPEED = 0.10                # m/s
ARC_RADIUS = 2.0            # m, for the arc candidates
MAX_DIST = 1.0              # m driven at most
STEP = 0.05                 # m driven between "can Nav2 plan?" checks
TIMEOUT = 15.0              # s
SWEEP_DS = 0.02             # m, candidate sweep resolution
OVERLAP_TOL = 2             # footprint samples allowed above the starting overlap (discretisation)
CLEARANCE_LOOKAHEAD = 0.3   # m past the free point where a candidate's clearance is judged
CANDIDATES = [(g, k, f'{"forward" if g > 0 else "backward"} {name}')
              for g in (1, -1)
              for k, name in ((0.0, 'straight'), (1.0 / ARC_RADIUS, 'arc left'),
                              (-1.0 / ARC_RADIUS, 'arc right'))]


def default_planner_id():
    """The planner navigation uses: planner_plugins[0] of the installed nav2
    yaml (same rule as nav2.launch.py)."""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    path = f"{get_package_share_directory('nav2_stack')}/config/nav2_param2.yaml"
    with open(path) as f:
        return yaml.safe_load(f)['planner_server']['ros__parameters']['planner_plugins'][0]


class Escaper:
    def __init__(self, node, get_pose, planner_id, frame='world', robot_frame='FitRosey_V1',
                 cmd_pub=None):
        self.node = node
        self.get_pose = get_pose
        self.planner_id = planner_id
        self.frame = frame
        self.robot_frame = robot_frame
        self.cmd_pub = cmd_pub or node.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.costmap_cli = node.create_client(GetCostmap, '/global_costmap/get_costmap')
        self.plan_ac = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
        self.grid = None
        self.log = node.get_logger()

    # ── costmap ──────────────────────────────────────────────────────────────
    def _load_costmap(self, timeout=10.0):
        if self.grid is not None:
            return True
        if not self.costmap_cli.wait_for_service(timeout_sec=timeout):
            self.log.warn('escape: /global_costmap/get_costmap unavailable')
            return False
        fut = self.costmap_cli.call_async(GetCostmap.Request())
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            self.log.warn('escape: get_costmap timed out')
            return False
        m = fut.result().map
        md = m.metadata
        self.res = md.resolution
        self.ox, self.oy = md.origin.position.x, md.origin.position.y
        self.grid = np.array(m.data, dtype=np.uint8).reshape(md.size_y, md.size_x)
        lethal = self.grid == LETHAL
        self.lethal_dist = distance_transform_edt(~lethal) * self.res  # m to nearest lethal cell
        self.labels, _ = label(lethal)
        # footprint sample points (robot frame): outline + interior at ~half a cell
        step = self.res / 2.0
        xs = np.arange(-FOOTPRINT_HALF_L, FOOTPRINT_HALF_L + 1e-9, step)
        ys = np.arange(-FOOTPRINT_HALF_W, FOOTPRINT_HALF_W + 1e-9, step)
        gx, gy = np.meshgrid(xs, ys)
        self.fp_area = np.stack([gx.ravel(), gy.ravel()], 1)
        edge = ((np.isclose(np.abs(gx), xs.max()) | np.isclose(np.abs(gy), ys.max()))).ravel()
        self.fp_edge = self.fp_area[edge]
        return True

    def _cells(self, x, y, yaw, pts):
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + c * pts[:, 0] - s * pts[:, 1]
        wy = y + s * pts[:, 0] + c * pts[:, 1]
        i = np.floor((wx - self.ox) / self.res).astype(int)
        j = np.floor((wy - self.oy) / self.res).astype(int)
        inside = (i >= 0) & (j >= 0) & (i < self.grid.shape[1]) & (j < self.grid.shape[0])
        return i.clip(0, self.grid.shape[1] - 1), j.clip(0, self.grid.shape[0] - 1), inside

    def is_stuck(self, pose):
        """The planner's start check (smac collision_checker): centre cost
        INSCRIBED/LETHAL, or a footprint-edge cell LETHAL (or off the map)."""
        x, y, yaw = pose
        i, j, inside = self._cells(x, y, yaw, np.zeros((1, 2)))
        if not inside[0] or self.grid[j[0], i[0]] >= INSCRIBED:
            return True
        i, j, inside = self._cells(x, y, yaw, self.fp_edge)
        return bool((~inside).any() or (self.grid[j, i] == LETHAL).any())

    def _overlap(self, pose):
        """(number of footprint samples on LETHAL cells, set of obstacle labels touched)."""
        i, j, inside = self._cells(*pose, self.fp_area)
        on = inside & (self.grid[j, i] == LETHAL)
        return int(on.sum()) + int((~inside).sum()), set(np.unique(self.labels[j[on], i[on]]).tolist())

    def _clearance(self, pose):
        i, j, inside = self._cells(*pose, self.fp_area)
        return float(self.lethal_dist[j, i].min()) if inside.all() else 0.0

    @staticmethod
    def _along(pose, gear, k, s):
        x, y, yaw = pose
        s = gear * s
        if abs(k) < 1e-9:
            return x + s * math.cos(yaw), y + s * math.sin(yaw), yaw
        return (x + (math.sin(yaw + k * s) - math.sin(yaw)) / k,
                y - (math.cos(yaw + k * s) - math.cos(yaw)) / k, yaw + k * s)

    def _lookahead_clearance(self, pose, gear, k, s_free, n0, labels0):
        """Clearance a little past the free point (right at it every candidate
        still hugs the obstacle), as far as the candidate stays safe: this is
        what prefers turning away from a wall over sliding along it."""
        best_s = s_free
        s = s_free + SWEEP_DS
        while s <= s_free + CLEARANCE_LOOKAHEAD + 1e-9:
            n, labels = self._overlap(self._along(pose, gear, k, s))
            if labels - labels0 or n > n0 + OVERLAP_TOL:
                break
            best_s = s
            s += SWEEP_DS
        return self._clearance(self._along(pose, gear, k, best_s))

    def choose(self, pose):
        """Evaluate the escape candidates. Returns (best or None, all results);
        a result is dict(name, gear, k, valid, dist, clearance, reason)."""
        n0, labels0 = self._overlap(pose)
        results = []
        for gear, k, name in CANDIDATES:
            r = dict(name=name, gear=gear, k=k, valid=False, dist=None, clearance=None, reason='')
            s = SWEEP_DS
            while s <= MAX_DIST + 1e-9:
                p = self._along(pose, gear, k, s)
                n, labels = self._overlap(p)
                if labels - labels0:
                    r['reason'] = f'reaches a different obstacle at {s:.2f} m'
                    break
                if n > n0 + OVERLAP_TOL:
                    r['reason'] = f'drives further into the obstacle at {s:.2f} m'
                    break
                if not self.is_stuck(p):
                    r.update(valid=True, dist=s, clearance=self._lookahead_clearance(
                        pose, gear, k, s, n0, labels0))
                    break
                s += SWEEP_DS
            else:
                r['reason'] = f'still stuck after {MAX_DIST:.1f} m'
            results.append(r)
        valid = [r for r in results if r['valid']]
        if not valid:
            return None, results
        # most clearance; ties (within 2 cm) -> shorter, then straight
        best = max(valid, key=lambda r: (round(r['clearance'] / 0.02), -r['dist'], r['k'] == 0.0))
        return best, results

    # ── driving ──────────────────────────────────────────────────────────────
    def _cmd(self, vx, wz):
        m = TwistStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = self.robot_frame
        m.twist.linear.x, m.twist.angular.z = float(vx), float(wz)
        self.cmd_pub.publish(m)

    def _plan_goal(self, goal_xy):
        g = ComputePathToPose.Goal()
        g.goal = PoseStamped()
        g.goal.header.frame_id = self.frame
        g.goal.header.stamp = self.node.get_clock().now().to_msg()
        g.goal.pose.position.x, g.goal.pose.position.y = float(goal_xy[0]), float(goal_xy[1])
        g.goal.pose.orientation.w = 1.0
        g.planner_id = self.planner_id
        g.use_start = False  # from the robot's current pose, like the BT
        return g

    def run(self, goal_xy):
        """Escape if stuck. Returns (escaped, message). escaped=False with a
        message when not stuck, no valid motion, or the drive gave up."""
        if not self._load_costmap():
            return False, 'no costmap'
        pose = self.get_pose()
        if pose is None:
            return False, 'no pose'
        if not self.is_stuck(pose):
            return False, 'not stuck'
        best, results = self.choose(pose)
        summary = '; '.join(
            f"{r['name']}: " + (f"free after {r['dist']:.2f} m, clearance {r['clearance']:.2f} m"
                                if r['valid'] else r['reason']) for r in results)
        if best is None:
            self.log.warn(f'escape: stuck at ({pose[0]:.2f}, {pose[1]:.2f}) but no safe way out '
                          f'-- not escaping. [{summary}]')
            return False, 'no valid escape motion'
        self.log.warn(f"escape: stuck at ({pose[0]:.2f}, {pose[1]:.2f}), driving {best['name']} "
                      f"(free after ~{best['dist']:.2f} m). [{summary}]")
        if not self.plan_ac.wait_for_server(timeout_sec=5.0):
            return False, 'planner action unavailable'

        vx = SPEED * best['gear']
        wz = vx * best['k']
        n0, labels0 = self._overlap(pose)
        start, last = pose, pose
        driven, next_check = 0.0, STEP
        t0 = time.monotonic()
        plan_fut = result_fut = None
        outcome = None
        while outcome is None:
            self._cmd(vx, wz)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            p = self.get_pose()
            if p is not None:
                driven += math.hypot(p[0] - last[0], p[1] - last[1])
                last = p
                n, labels = self._overlap(p)
                if labels - labels0 or n > n0 + OVERLAP_TOL:
                    outcome = (False, 'stopped: would touch another obstacle / drive further in')
                    break
            # ask the planner whether Nav2 can take over again
            if plan_fut is None and result_fut is None and driven >= next_check:
                next_check = driven + STEP
                plan_fut = self.plan_ac.send_goal_async(self._plan_goal(goal_xy))
            if plan_fut is not None and plan_fut.done():
                handle = plan_fut.result()
                plan_fut = None
                if handle is not None and handle.accepted:
                    result_fut = handle.get_result_async()
            if result_fut is not None and result_fut.done():
                res = result_fut.result()
                result_fut = None
                if res is not None and res.status == GoalStatus.STATUS_SUCCEEDED and \
                        res.result.path.poses and p is not None and not self.is_stuck(p):
                    # the planner can plan again AND the footprint is clear of the
                    # obstacle, so the controller can start too
                    outcome = (True, f'Nav2 can plan again after {driven:.2f} m')
            if driven >= MAX_DIST:
                outcome = (False, f'gave up after {driven:.2f} m')
            elif time.monotonic() - t0 > TIMEOUT:
                outcome = (False, f'gave up after {TIMEOUT:.0f} s ({driven:.2f} m)')
        for _ in range(3):
            self._cmd(0.0, 0.0)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.log.warn(f"escape ({best['name']}): {outcome[1]}")
        return outcome
