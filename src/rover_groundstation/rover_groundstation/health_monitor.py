# A la Geppetto
# Need to adapt to nested bag structure of our captures, and apply protection for case where there isn't anything

import os
import yaml
from pathlib import Path
from collections import defaultdict
import numpy as np

# N = 5  # number of most recent trials to consider

class BagSummary:
    def __init__(self):
        self.topics = defaultdict(list)
        self.sizes = []

    def stats(self):
        # Use this to generate our topic and size summary
        return np.mean(self.sizes), {k:np.mean(v) for k,v in self.topics.items()}

    def __repr__(self):
        return f"{self.topics} | {self.sizes}"

def get_latest_trial_dirs(parent_dir, pattern="Trial_", n=5):
    """Return the latest n subdirectories by modification time that start with 'pattern'."""
    subdirs = [d for d in Path(parent_dir).iterdir() if d.is_dir() and str(d.name).startswith(pattern)]
    sorted_dirs = sorted(subdirs, key=lambda x: x.stat().st_mtime, reverse=True)
    return sorted_dirs[:n]

def get_all_bags(parent_dir, bagtype="mcap"):
    """Get all directories which contain mcap files"""
#    subdirs = [d for d in Path(parent_dir).iterdir() if d.is_dir()]
    bagfiles = Path(parent_dir).rglob(f'*.{bagtype}')
    bagdirs = sorted([f.parent for f in bagfiles], key=lambda x: x.stat().st_mtime)
    return bagdirs

def get_directory_size(directory):
    """Get total size of directory in bytes."""
    return sum(f.stat().st_size for f in Path(directory).rglob('*') if f.is_file())

def parse_metadata_yaml(bag_dir):
    """Extract topic message counts from metadata.yaml."""
    metadata_path = Path(bag_dir) / "metadata.yaml"
    if not metadata_path.exists():
        return {}

    with open(metadata_path, 'r') as f:
        metadata = yaml.safe_load(f)

    topic_counts = {}
    for topic in metadata.get('topics_with_message_count', []):
        topic_name = topic['topic_metadata']['name']
        count = topic['message_count']
        topic_counts[topic_name] = count
    return topic_counts

def analyze_trials(parent_dir, N):
    latest_dirs = get_latest_trial_dirs(parent_dir, n=N)
    if not latest_dirs:
        print("[INFO] No trial directories found.")
        return
    if len(latest_dirs) < N:
        print("[INFO] Not enough trial directories to compute history yet! Skipping threshold alerting...")

    # Bags always written in same order!!!
    BAG_NAMES = None # TODO

#    total_sizes = defaultdict(list)
#    topic_history = defaultdict(list)

    HISTORY = defaultdict(BagSummary)

    print("\n=== Trial Stats ===")
    for trial_dir in latest_dirs:

        for i,bag in enumerate(get_all_bags(trial_dir)):

            bag_size = get_directory_size(bag)
            HISTORY[i].sizes.append(bag_size)

            topic_counts = parse_metadata_yaml(bag)
            print(topic_counts)
            for topic, count in topic_counts.items():
                HISTORY[i].topics[topic].append(count)

    print([v.stats() for v in HISTORY.values()])

#        print(f"- {trial_dir.name}: {bag_size / 1e6:.2f} MB, {len(topic_counts)} topics")

#    avg_size = sum(total_sizes) / len(total_sizes)
#    print(f"\n📦 Average Bag Size: {avg_size / 1e6:.2f} MB")
#
#    print("\n📊 Average Message Counts per Topic:")
#    for topic, counts in topic_history.items():
#        avg = sum(counts) / len(counts)
#        print(f"  {topic}: {avg:.2f} msgs (across {len(counts)} bags)")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Analyze ROS 2 bags over recent trials.")
    parser.add_argument("parent_dir", help="Path to parent directory containing trial subdirectories.")
    parser.add_argument("num_dir", type=int, help="Number of directories to include in running statistics and threshold alerting.", default=5)
    args = parser.parse_args()

    analyze_trials(args.parent_dir, args.num_dir)
