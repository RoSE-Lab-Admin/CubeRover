"""Rover geometry: OptiTrack rigid-body pose -> rover base_link pose -> wheel contact points.

The OptiTrack pose (/FitRosey_passive/pose) is the pose of the Motive rigid body, whose origin is
the rear-right marker (URDF mocap_1) and whose axes are rotated a few degrees against the rover.
config/rover_geometry.yaml (scripts/rover_calibration/marker_calibration.py) holds base_link's
pose in the rigid-body frame, the URDF wheel centres and the wheel radius. Valid for every bag
recorded with the FitRosey_passive asset (10/27/26 on).

    from nav2_stack import rover_geometry as rg
    geo = rg.load()
    pos_bl, rot_bl = rg.base_link_poses(geo, pos_rb, quat_rb)      # (N,3), (N,3,3)
    yaw, pitch, roll = rg.yaw_pitch_roll(rot_bl)                     # rover attitude (ZYX, rad)
    contacts = rg.wheel_contacts(geo, pos_bl, rot_bl)                # (N,4,3) world
"""
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as Rot

WHEEL_ORDER = ("front_left", "front_right", "back_left", "back_right")


def default_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory("nav2_stack")) / "config" / "rover_geometry.yaml"
        if p.exists():
            return p
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent / "config" / "rover_geometry.yaml"


def load(path: Optional[Path] = None) -> Dict:
    """{"t": base_link origin in the rigid-body frame (3,), "R": base_link axes in the rigid-body
    frame (3,3), "wheels": wheel centres in base_link (4,3) in WHEEL_ORDER, "radius": m}."""
    cfg = yaml.safe_load(Path(path or default_path()).read_text())
    b = cfg["base_link_in_rigid_body"]
    return {"t": np.asarray(b["translation"], dtype=float),
            "R": Rot.from_quat(b["rotation_xyzw"]).as_matrix(),
            "wheels": np.array([cfg["wheels"][w] for w in WHEEL_ORDER], dtype=float),
            "radius": float(cfg["wheel_radius"]),
            "rigid_body": cfg.get("rigid_body", "")}


def base_link_poses(geo: Dict, pos_rb: np.ndarray, quat_rb: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """World base_link position (N,3) and rotation matrices (N,3,3) from rigid-body positions (N,3)
    and quaternions (N,4, x y z w)."""
    rot_rb = Rot.from_quat(np.asarray(quat_rb, dtype=float)).as_matrix()
    rot_bl = rot_rb @ geo["R"]
    pos_bl = np.asarray(pos_rb, dtype=float) + rot_rb @ geo["t"]
    return pos_bl, rot_bl


def yaw_pitch_roll(rot: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """ZYX yaw, pitch, roll (rad) of rotation matrices (N,3,3) -- ROS convention (pitch + = nose down)."""
    e = Rot.from_matrix(rot).as_euler("ZYX")
    return e[..., 0], e[..., 1], e[..., 2]


def wheel_contacts(geo: Dict, pos_bl: np.ndarray, rot_bl: np.ndarray) -> np.ndarray:
    """(N,4,3) world points where the wheels touch the ground (WHEEL_ORDER): the wheel centre minus
    the radius along the world-down direction projected into the wheel plane (the plane normal to
    the axle, base_link y)."""
    centres = pos_bl[:, None, :] + np.einsum("nij,kj->nki", rot_bl, geo["wheels"])
    axle = rot_bl[:, :, 1]                                    # base_link y in world
    down = np.array([0.0, 0.0, -1.0])
    d = down[None, :] - (axle @ down)[:, None] * axle         # down projected into the wheel plane
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    return centres + geo["radius"] * d[:, None, :]
