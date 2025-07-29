# A la Geppetto
# Need to adapt to nested bag structure of our captures, and apply protection for case where there isn't anything

import os
import yaml
from pathlib import Path
from collections import defaultdict

N = 5  # number of most recent trials to consider

def get_latest_trial_dirs(parent_dir, n=N):
    """Return the latest n subdirectories by modification time."""
    subdirs = [d for d in Path(parent_dir).iterdir() if d.is_dir()]
    sorted_dirs = sorted(subdirs, key=lambda x: x.stat().st_mtime, reverse=True)
    return sorted_dirs[:n]

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

def analyze_trials(parent_dir):
    latest_dirs = get_latest_trial_dirs(parent_dir, N)
    if not latest_dirs:
        print("No trial directories found.")
        return

    total_sizes = []
    topic_history = defaultdict(list)

    print("\n=== Trial Stats ===")
    for trial_dir in latest_dirs:
        bag_size = get_directory_size(trial_dir)
        total_sizes.append(bag_size)

        topic_counts = parse_metadata_yaml(trial_dir)
        for topic, count in topic_counts.items():
            topic_history[topic].append(count)

        print(f"- {trial_dir.name}: {bag_size / 1e6:.2f} MB, {len(topic_counts)} topics")

    avg_size = sum(total_sizes) / len(total_sizes)
    print(f"\n📦 Average Bag Size: {avg_size / 1e6:.2f} MB")

    print("\n📊 Average Message Counts per Topic:")
    for topic, counts in topic_history.items():
        avg = sum(counts) / len(counts)
        print(f"  {topic}: {avg:.2f} msgs (across {len(counts)} bags)")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Analyze ROS 2 bags over recent trials.")
    parser.add_argument("parent_dir", help="Path to parent directory containing trial subdirectories.")
    args = parser.parse_args()

    analyze_trials(args.parent_dir)
