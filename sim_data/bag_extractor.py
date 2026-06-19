from rosbags.highlevel import AnyReader
import os
from pathlib import Path
import pandas as pd
import numpy as np

class ExtractBag():
    def __init__(self, trial_folder, rosey_file):
        self.odom_bag = Path(f'sim_data/{trial_folder}/{rosey_file}')

        # folders to store bag data
        self.pose_dir = f'sim_data/extracted_data/{trial_folder}/pose_data'
        os.makedirs(self.pose_dir, exist_ok=True)

    def pose_extract(self):
        with AnyReader([self.odom_bag]) as reader:
            # topic
            pose_topic = '/CubeRover_V1/pose'
            # list of connections
            pose_list = [c for c in reader.connections if c.topic==pose_topic][0]

            # tracking variables
            last_time = 0
            sec = 1000
            i = 0

            # data output list
            pose_data = []

            for conn, t, raw_msg in reader.messages([pose_list]):
                msg = reader.deserialize(raw_msg, conn.msgtype)
                pose_data.append({
                    't': t,
                    'frame_id': msg.header.frame_id,
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': msg.pose.position.z,
                    'qx': msg.pose.orientation.x,
                    'qy': msg.pose.orientation.y,
                    'qz': msg.pose.orientation.z,
                    'qw': msg.pose.orientation.w,
                })
            
            # create dataframe
            extracted_pose = pd.DataFrame(pose_data)
            # add time in seconds
            extracted_pose['t_sec'] = extracted_pose['t'] * 1e-9
            # save as parquet
            extracted_pose.to_csv(
                f'{self.pose_dir}/pose.csv',
                index=False
            )

if __name__ == '__main__':
    trial_folder = '11172025/11-21-10'
    rosey_file = 'RoseyBag_0.mcap'
    extractor = ExtractBag(trial_folder=trial_folder, rosey_file=rosey_file)
    extractor.pose_extract()






