#!/usr/bin/env python3
"""
Pipeline for ROS bag processing:
  1) Extract raw CSVs under results/raw_data
  2) Extract wheel‑cam video at its true FPS
  3) Plot raw mocap, IMU and motor data
  4) Merge, clean and save output_filled.csv under results/edited_and_rotated
  5) Rotate, re‑zero and save output_rotated.csv
  6) Plot rotated data (using 'timestamp')
"""

from pathlib import Path
from dataclasses import asdict
from typing import List, Tuple

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import cv2
from tqdm import tqdm
from rosbags.highlevel import AnyReader

# ==== USER PARAMETERS =======================================================

# parser = argparse.ArgumentParser(
#                     prog='Data_processor',
#                     description='processes bag data',
#                     epilog='')

# parser.add_argument('filename')

# args = parser.parse_args()

BASE_DIR     = Path(Path.home())

# Path('/content/drive/MyDrive/Field Testing/Test ROS Bags/7 15 2025/'
#                     'Trial_1cm_10000000.0radius_0.0slope_Trial6_07152025_17_34_59')

OUTPUT_DIR   = BASE_DIR / 'results'
RAW_DIR      = OUTPUT_DIR / 'raw_data'
EDITED_DIR   = OUTPUT_DIR / 'edited_and_rotated'

WANTED_BAGS  = {'cam_bag', 'imu_bag', 'mocap_bag', 'motor_bag'}
IMAGE_TOPIC  = '/Rover/camera/image_raw/compressed'
CODEC        = 'mp4v'

STATIC_END   = 10.0
DRIVE_WINDOW = 5.0

DROP_COLS = [
    'stamp_ns','__msgtype__','header.stamp.sec','header.stamp.nanosec',
    'header.stamp.__msgtype__','header.frame_id','header.__msgtype__',
    'pose.position.__msgtype__','pose.orientation.__msgtype__','pose.__msgtype__',
    'orientation.__msgtype__','angular_velocity.__msgtype__','linear_acceleration.__msgtype__',
]
COV_COLS = [
    'orientation_covariance',
    'angular_velocity_covariance',
    'linear_acceleration_covariance',
]

# ==== FUNCTIONS =============================================================
def extract_csv(bag_path: Path, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    with AnyReader([bag_path]) as reader:
        for conn in reader.connections:
            rows = []
            desc = f"{bag_path.name}:{conn.topic}"
            for _, ts, raw in tqdm(reader.messages(connections=[conn]),
                                   total=conn.msgcount, desc=desc):
                msg = reader.deserialize(raw, conn.msgtype)
                row = {'stamp_ns': ts}
                row.update(pd.json_normalize(asdict(msg)).iloc[0].to_dict())
                rows.append(row)
            csv_file = out_dir / f"{conn.topic.strip('/').replace('/', '_')}.csv"
            pd.DataFrame(rows).to_csv(csv_file, index=False)
            print(f"Wrote {len(rows)} rows to {csv_file.name}")

def extract_video(bag_path: Path, topic: str, out_file: Path, codec: str = CODEC):
    out_file.parent.mkdir(parents=True, exist_ok=True)
    buffer: List[Tuple] = []
    with AnyReader([bag_path]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise ValueError(f"Topic {topic!r} not found in {bag_path}")
        total = sum(c.msgcount for c in conns)
        for conn, ts, raw in tqdm(reader.messages(connections=conns),
                                   total=total, desc=f"Buffering {topic}"):
            msg = reader.deserialize(raw, conn.msgtype)
            buffer.append((conn, ts, msg))

    if len(buffer) < 2:
        raise RuntimeError("Not enough frames to compute FPS")

    timestamps = [ts for _, ts, _ in buffer]
    duration_s = (timestamps[-1] - timestamps[0]) / 1e9
    fps = len(buffer) / duration_s
    print(f"Computed dynamic FPS: {fps:.2f}")

    writer = None
    fourcc = cv2.VideoWriter_fourcc(*codec)
    for conn, _, msg in tqdm(buffer, desc="Writing frames"):
        if 'CompressedImage' in conn.msgtype:
            arr   = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        else:
            h, w   = msg.height, msg.width
            flat   = np.frombuffer(msg.data, np.uint8)
            enc    = msg.encoding.lower()
            if enc == 'rgb8':
                frame = cv2.cvtColor(flat.reshape((h, w, 3)), cv2.COLOR_RGB2BGR)
            elif enc == 'bgr8':
                frame = flat.reshape((h, w, 3))
            elif enc == 'mono8':
                frame = cv2.cvtColor(flat.reshape((h, w)), cv2.COLOR_GRAY2BGR)
            else:
                raise RuntimeError(f"Unsupported encoding: {msg.encoding}")
        if writer is None:
            h, w = frame.shape[:2]
            writer = cv2.VideoWriter(str(out_file), fourcc, fps, (w, h))
        writer.write(frame)

    if writer:
        writer.release()
        print(f"Saved video → {out_file}")

def plot_csv(csv_path: Path,
             y_cols: List[str],
             out_dir: Path,
             tag: str,
             time_col: str = 'stamp_ns',
             title: str = None,
             dpi: int = 110):
    """
    Plot one or more columns vs time and save PNG in out_dir.
    time_col defaults to 'stamp_ns' but can be overridden.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    df = pd.read_csv(csv_path)
    if time_col not in df:
        raise KeyError(f"{time_col} missing in {csv_path.name}")
    t = (df[time_col] - df[time_col].iloc[0])

    fig, ax = plt.subplots(dpi=dpi)
    for col in y_cols:
        if col not in df:
            raise KeyError(f"{col} missing in {csv_path.name}")
        ax.plot(t, df[col], label=col)

    ax.set(xlabel='Time [s]',
           ylabel='Value',
           title=title or f"{csv_path.stem}: {tag}")
    ax.legend()
    plt.tight_layout()
    fig.savefig(out_dir / f"{csv_path.stem}_{tag}.png")
    plt.close(fig)

def merge_and_clean(csv_files: List[Path], output_file: Path) -> pd.DataFrame:
    dfs = []
    for p in csv_files:
        df = pd.read_csv(p)
        if 'stamp_ns' not in df.columns:
            raise KeyError(f"stamp_ns missing in {p.name}")
        df['timestamp'] = df['stamp_ns'] * 1e-9
        df = df.drop(columns=[c for c in DROP_COLS if c in df.columns])
        dfs.append(df)
    combined = pd.concat(dfs, ignore_index=True).sort_values('timestamp')
    combined['timestamp'] -= combined['timestamp'].iloc[0]
    combined[COV_COLS] = combined[COV_COLS].fillna(method='bfill').fillna(method='ffill')
    others = [c for c in combined.columns if c not in COV_COLS + ['timestamp']]
    combined[others] = combined[others].interpolate(method='linear', limit_direction='both')
    output_file.parent.mkdir(parents=True, exist_ok=True)
    combined.to_csv(output_file, index=False)
    print(f"Saved merged data to {output_file}")
    return combined

def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1,y1,z1,w1 = q1; x2,y2,z2,w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])

def align_dataframe(df: pd.DataFrame, output_file: Path):
    pos_cols = ['pose.position.x','pose.position.y','pose.position.z']
    mask = (df['timestamp'] >= STATIC_END) & (df['timestamp'] <= STATIC_END + DRIVE_WINDOW)
    seg = df.loc[mask]
    if len(seg) < 2:
        raise ValueError("Not enough data to compute direction")
    start = seg.iloc[0][pos_cols].to_numpy()
    end   = seg.iloc[-1][pos_cols].to_numpy()
    delta = end - start
    phi   = np.arctan2(delta[1], delta[0])
    c, s = np.cos(-phi), np.sin(-phi)
    Rz   = np.array([[c, -s, 0],[s, c, 0],[0,0,1]])
    half = -phi/2
    q_align = np.array([0.0, 0.0, np.sin(half), np.cos(half)])

    for group in [
        pos_cols,
        ['linear_acceleration.x','linear_acceleration.y','linear_acceleration.z'],
        ['angular_velocity.x','angular_velocity.y','angular_velocity.z'],
    ]:
        arr = (Rz @ df[group].to_numpy().T).T
        df[group] = arr

    ori_cols = ['pose.orientation.x','pose.orientation.y','pose.orientation.z','pose.orientation.w']
    Q = df[ori_cols].to_numpy()
    df[ori_cols] = np.vstack([quat_mul(q_align, q) for q in Q])

    origin = df.loc[df['timestamp'].idxmin(), pos_cols].to_numpy()
    df[pos_cols] = df[pos_cols] - origin

    output_file.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(output_file, index=False)
    print(f"Saved aligned data to {output_file}")

# ==== MAIN ==================================================================
def main(base_dir: Path):
    global BASE_DIR, OUTPUT_DIR, RAW_DIR, EDITED_DIR
    BASE_DIR =  Path(base_dir)

    OUTPUT_DIR   = BASE_DIR / 'results'
    RAW_DIR      = OUTPUT_DIR / 'raw_data'
    EDITED_DIR   = OUTPUT_DIR / 'edited_and_rotated'

    RAW_DIR.mkdir(parents=True, exist_ok=True)
    EDITED_DIR.mkdir(parents=True, exist_ok=True)

    # 1) Raw CSVs
    for stem in WANTED_BAGS:
        folder = BASE_DIR / stem
        if not folder.exists():
            raise FileNotFoundError(f"Expected folder {folder}")
        extract_csv(folder, RAW_DIR)

    # 2) Wheel‑cam video
    extract_video(BASE_DIR/'cam_bag', IMAGE_TOPIC, RAW_DIR/'wheel_cams.mp4')

    # 3) Raw plots (using 'stamp_ns')
    plot_csv(RAW_DIR/'CubeRover_V1_pose.csv',
             ['pose.position.x','pose.position.y','pose.position.z'],
             RAW_DIR, tag='pos', title='Raw Mocap Position')
    plot_csv(RAW_DIR/'CubeRover_V1_pose.csv',
             ['pose.orientation.w','pose.orientation.x','pose.orientation.y','pose.orientation.z'],
             RAW_DIR, tag='quat', title='Raw Mocap Orientation')
    plot_csv(RAW_DIR/'Rover_bno055_imu.csv',
             ['orientation.w','orientation.x','orientation.y','orientation.z'],
             RAW_DIR, tag='imu_quat', title='Raw IMU Orientation')
    plot_csv(RAW_DIR/'Rover_bno055_imu.csv',
             ['angular_velocity.x','angular_velocity.y','angular_velocity.z'],
             RAW_DIR, tag='imu_angvel', title='Raw IMU Angular Velocity')
    plot_csv(RAW_DIR/'Rover_bno055_imu.csv',
             ['linear_acceleration.x','linear_acceleration.y','linear_acceleration.z'],
             RAW_DIR, tag='imu_linacc', title='Raw IMU Linear Acceleration')
    plot_csv(RAW_DIR/'Rover_roboclaw_enc_telem.csv',
             ['enc1','enc2','enc3','enc4'],
             RAW_DIR, tag='enc', title='Raw Encoder Counts')
    plot_csv(RAW_DIR/'Rover_roboclaw_enc_telem.csv',
             ['vel1','vel2','vel3','vel4'],
             RAW_DIR, tag='vel', title='Raw Wheel Velocities')
    plot_csv(RAW_DIR/'Rover_roboclaw_enc_telem.csv',
             ['m1current','m2current','m3current','m4current'],
             RAW_DIR, tag='curr', title='Raw Motor Currents')

    # 4) Merge & clean
    merged = merge_and_clean([
        RAW_DIR/'CubeRover_V1_pose.csv',
        RAW_DIR/'Rover_bno055_imu.csv',
        RAW_DIR/'Rover_roboclaw_enc_telem.csv',
    ], EDITED_DIR/'output_filled.csv')

    # 5) Align & rotate
    align_dataframe(merged, EDITED_DIR/'output_rotated.csv')

    # 6) Rotated plots (using 'timestamp')
    pr = EDITED_DIR/'output_rotated.csv'
    plot_csv(pr,
             ['pose.position.x','pose.position.y','pose.position.z'],
             EDITED_DIR, tag='rot_pos', title='Rotated Position',
             time_col='timestamp')
    plot_csv(pr,
             ['pose.orientation.w','pose.orientation.x','pose.orientation.y','pose.orientation.z'],
             EDITED_DIR, tag='rot_mocap_quat', title='Rotated Mocap Orientation',
             time_col='timestamp')
    plot_csv(pr,
             ['orientation.w','orientation.x','orientation.y','orientation.z'],
             EDITED_DIR, tag='rot_imu_quat', title='Rotated IMU Orientation',
             time_col='timestamp')
    plot_csv(pr,
             ['angular_velocity.x','angular_velocity.y','angular_velocity.z'],
             EDITED_DIR, tag='rot_imu_angvel', title='Rotated IMU Angular Velocity',
             time_col='timestamp')
    plot_csv(pr,
             ['linear_acceleration.x','linear_acceleration.y','linear_acceleration.z'],
             EDITED_DIR, tag='rot_imu_linacc', title='Rotated IMU Linear Acceleration',
             time_col='timestamp')
    plot_csv(pr,
             ['enc1','enc2','enc3','enc4'],
             EDITED_DIR, tag='rot_enc', title='Rotated Encoder Counts',
             time_col='timestamp')
    plot_csv(pr,
             ['vel1','vel2','vel3','vel4'],
             EDITED_DIR, tag='rot_vel', title='Rotated Wheel Velocities',
             time_col='timestamp')
    plot_csv(pr,
             ['m1current','m2current','m3current','m4current'],
             EDITED_DIR, tag='rot_curr', title='Rotated Motor Currents',
             time_col='timestamp')
