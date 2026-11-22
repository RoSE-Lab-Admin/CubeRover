#!/usr/bin/env python3
"""Rigid body (OptiTrack) -> rover base_link transform from a marker recording.

Reads a bag with the rigid-body pose /<body>/pose and its markers /<body>/marker<N>/pose
(natnet_ros2, pub_rigid_body_marker), expresses every marker in the rigid-body frame (they
must be rigid: the script reports their spread), matches the URDF's mocap_1..4 markers to them
by pairwise distances (so it does not rely on Motive's marker numbering), and solves the
rigid-body -> base_link rotation and translation (Kabsch). Writes the result, the URDF wheel
centres and the wheel radius to rover_geometry.yaml (read by nav2_stack/rover_geometry.py).

    python3 scripts/rover_calibration/marker_calibration.py BAG \\
        --urdf /home/roselab/CubeRover/src/rosey_description/description/robot/roseybot_description.urdf.xacro \\
        --out src/nav2_stack/config/rover_geometry.yaml

Needs ROS sourced (rosbag2_py). The URDF is the SolidWorks export on CubeRover's
dev-fitRoseyRefactored branch (rosey_description), which has the mocap_1..4 marker links.
"""
import argparse
import itertools
import subprocess
import sys
import xml.etree.ElementTree as ET
from datetime import date
from pathlib import Path

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation as Rot

WHEELS = ("front_left", "front_right", "back_left", "back_right")
MATCH_TOL = 0.005   # m, max pairwise-distance error for a URDF <-> marker match


def urdf_points(path: Path):
    """mocap_1..4 marker centres and the four wheel centres in base_link (joint origins)."""
    joints = {j.get("name"): j for j in ET.parse(path).getroot().iter("joint")}
    def origin(name):
        return [float(v) for v in joints[name].find("origin").get("xyz").split()]
    markers = {f"mocap_{i}": origin(f"mocap_{i}_joint") for i in range(1, 5)}
    wheels = {w: origin(f"{w}_wheel_joint") for w in WHEELS}
    return markers, wheels


def read_frames(bag: str, body: str):
    """(pose (N,7) x y z qx qy qz qw, markers (N,K,3)) for frames with every marker present.
    natnet publishes the pose, then the body's markers, each stamped separately, so a frame is
    a pose message plus the marker messages that follow it."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    frames, cur, ids = [], None, set()
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == f"/{body}/pose":
            if cur is not None:
                frames.append(cur)
            m = deserialize_message(data, PoseStamped)
            p, q = m.pose.position, m.pose.orientation
            cur = ((p.x, p.y, p.z, q.x, q.y, q.z, q.w), {})
        elif topic.startswith(f"/{body}/marker") and cur is not None:
            i = int(topic[len(f"/{body}/marker"):].split("/")[0])
            m = deserialize_message(data, PointStamped)
            cur[1][i] = (m.point.x, m.point.y, m.point.z)
            ids.add(i)
    if cur is not None:
        frames.append(cur)
    ids = sorted(ids)
    full = [f for f in frames if len(f[1]) == len(ids)]
    if not full:
        sys.exit(f"no frame with the pose and all {len(ids)} markers of {body} in {bag}")
    return (np.array([f[0] for f in full]), np.array([[f[1][i] for i in ids] for f in full]), ids,
            len(frames))


def kabsch(a: np.ndarray, b: np.ndarray):
    """R, t with b ~ R a + t (rows are points)."""
    ca, cb = a.mean(0), b.mean(0)
    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rot = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return rot, cb - rot @ ca


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--urdf", required=True, type=Path)
    ap.add_argument("--body", default="FitRosey_passive")
    ap.add_argument("--wheel-radius", type=float, default=0.158,
                    help="m; 0.158 = between the wheel and the grouser tips (roseybot_controllers.yaml)")
    ap.add_argument("--out", type=Path, default=None, help="rover_geometry.yaml to write (default: print only)")
    a = ap.parse_args()

    pose, mk, ids, n_frames = read_frames(a.bag, a.body)
    rot_rb = Rot.from_quat(pose[:, 3:7]).as_matrix()                          # rb -> world
    local = np.einsum("nji,nkj->nki", rot_rb, mk - pose[:, None, :3])          # markers in the rb frame
    mean = local.mean(0)
    spread = np.linalg.norm(local - mean[None], axis=2)
    print(f"{len(pose)} of {n_frames} frames with all {len(ids)} markers of {a.body}")
    for k, i in enumerate(ids):
        print(f"  marker{i}: rb frame {np.round(mean[k], 4)} m, spread rms {1000 * np.sqrt((spread[:, k] ** 2).mean()):.2f} "
              f"/ max {1000 * spread[:, k].max():.1f} mm")
    if spread.max() > 0.02:
        print("WARNING: a marker moved > 2 cm in the rigid-body frame -- mislabel or loose marker?")

    markers, wheels = urdf_points(a.urdf)
    names = list(markers)
    u = np.array([markers[n] for n in names])
    du = np.linalg.norm(u[:, None] - u[None], axis=2)
    dm = np.linalg.norm(mean[:, None] - mean[None], axis=2)
    iu = np.triu_indices(len(names), 1)
    ranked = sorted((np.abs(dm[np.ix_(p, p)] - du)[iu].max(), p)
                    for p in itertools.permutations(range(len(ids)), len(names)))
    err, perm = ranked[0]
    if err > MATCH_TOL:
        sys.exit(f"no URDF <-> marker match within {1000 * MATCH_TOL:.0f} mm (best {1000 * err:.1f} mm)")
    print(f"URDF <-> marker match (pairwise distances within {1000 * err:.1f} mm; next best "
          f"{1000 * ranked[1][0]:.1f} mm): " + ", ".join(f"{n} = marker{ids[p]}" for n, p in zip(names, perm)))

    rc, tc = kabsch(mean[list(perm)], u)         # p_bl = rc p_rb + tc
    resid = np.linalg.norm(mean[list(perm)] @ rc.T + tc - u, axis=1)
    # base_link frame expressed in the rigid-body frame (TF FitRosey_passive -> base_link)
    rot_bl = rc.T
    t_bl = -rc.T @ tc
    ypr = Rot.from_matrix(rot_bl).as_euler("ZYX", degrees=True)
    print(f"base_link in the {a.body} frame: translation {np.round(t_bl, 4)} m, yaw/pitch/roll "
          f"{np.round(ypr, 2)} deg; marker fit residuals {np.round(1000 * resid, 1)} mm")

    commit = ""
    try:
        commit = subprocess.run(["git", "-C", str(a.urdf.parent), "log", "-1", "--format=%h %ad", "--date=short",
                                 "--", a.urdf.name], capture_output=True, text=True).stdout.strip()
    except Exception:
        pass
    q = Rot.from_matrix(rot_bl).as_quat()
    lines = [
        f"# Rover geometry for {a.body}: generated by scripts/rover_calibration/marker_calibration.py",
        f"# on {date.today().isoformat()} from {a.bag}",
        f"# URDF: {a.urdf} ({commit})",
        "# Frames: rigid body = the OptiTrack pose (/FitRosey_passive/pose, TF FitRosey_passive);",
        "# base_link = the rover frame of the URDF (x forward, z up, centred between the wheels).",
        f"rigid_body: {a.body}",
        "base_link_in_rigid_body:      # pose of base_link in the rigid-body frame (= TF rigid body -> base_link)",
        f"  translation: [{t_bl[0]:.5f}, {t_bl[1]:.5f}, {t_bl[2]:.5f}]   # m",
        f"  rotation_xyzw: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]",
        f"  yaw_pitch_roll_deg: [{ypr[0]:.3f}, {ypr[1]:.3f}, {ypr[2]:.3f}]   # ZYX, informational",
        "calibration:",
        "  marker_map: {" + ", ".join(f"{n}: {ids[p]}" for n, p in zip(names, perm)) + "}   # URDF link: natnet marker index",
        f"  fit_residual_mm: [{', '.join(f'{1000 * v:.2f}' for v in resid)}]",
        f"  frames: {len(pose)}",
        f"wheel_radius: {a.wheel_radius}   # m, between the wheel and the grouser tips",
        "wheels:                       # wheel centres in base_link (URDF wheel joints), m",
    ] + [f"  {w}: [{p[0]:.5f}, {p[1]:.5f}, {p[2]:.5f}]" for w, p in wheels.items()]
    text = "\n".join(lines) + "\n"
    if a.out:
        a.out.write_text(text)
        print(f"wrote {a.out}")
    else:
        print(text)


if __name__ == "__main__":
    main()
