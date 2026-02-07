#!/usr/bin/env python3
import subprocess
import time
import signal
import os
import sys

# --- CONFIGURATION ---
BAG_DIR = os.environ.get("BAG_DIR", "ros_bags") # Folder name
DURATION = int(os.environ.get("DURATION", 15)) # Seconds to record
TOPICS = "-a" # Record all topics
STORAGE_ID = "mcap" # Format

def main():
    # 1. SETUP PATHS
    # Get the directory where this script file lives
    script_location = os.path.dirname(os.path.abspath(__file__))
    
    # Create the full path to the bag folder relative to the script
    base_bag_dir = os.path.join(script_location, BAG_DIR)
    
    # Create the specific bag name and path
    bag_name = time.strftime("recording_%Y-%m-%d_%H-%M-%S")
    full_bag_path = os.path.join(base_bag_dir, bag_name)
    
    # Create directory if it doesn't exist
    os.makedirs(base_bag_dir, exist_ok=True)

    print(f"BAG_PATH={full_bag_path}")
    print(f"==========================================")
    print(f"   ROS 2 Python Recorder")
    print(f"==========================================")
    print(f" Script Location: {script_location}")
    print(f" Output Bag:      {full_bag_path}")
    print(f" Duration:        {DURATION}s")
    print(f"------------------------------------------")

    # 2. BUILD COMMAND
    cmd = [
        "ros2", "bag", "record",
        "-s", STORAGE_ID,
        "-a", 
        "-o", full_bag_path
    ]

    # 3. START RECORDING
    print(f"🚀 Launching recorder...")
    # We pass env=os.environ.copy() so it inherits your ROS setup
    process = subprocess.Popen(cmd, env=os.environ.copy())

    print(f"✅ Recorder started. PID: {process.pid}")

    # 4. WAIT LOOP
    try:
        for i in range(DURATION, 0, -1):
            if process.poll() is not None:
                print("\n❌ Process died early!")
                return
            sys.stdout.write(f"\r⏳ Recording... {i:02d}s remaining")
            sys.stdout.flush()
            time.sleep(1)
        print("\n🛑 Time's up.")

    except KeyboardInterrupt:
        print("\n⚠️  Manual Interrupt detected.")

    # 5. SHUTDOWN SEQUENCE
    if process.poll() is None:
        print("Sending SIGINT (Ctrl+C)...")
        # SIGINT is the specific signal ROS 2 needs to write metadata
        process.send_signal(signal.SIGINT)

        try:
            # Wait up to 5 seconds for graceful exit
            process.wait(timeout=5)
            print("✅ Process exited gracefully.")
        except subprocess.TimeoutExpired:
            print("⚠️  Process stuck! Forcing kill...")
            process.kill()

    # 6. VERIFY METADATA
    metadata_path = os.path.join(full_bag_path, "metadata.yaml")
    
    # Check if the folder exists first
    if not os.path.exists(full_bag_path):
         print(f"❌ FAILURE: Bag folder not created: {full_bag_path}")
         return

    if os.path.exists(metadata_path):
        print(f"🎉 SUCCESS: Metadata found at:\n   {metadata_path}")
    else:
        print("❌ FAILURE: Metadata missing.")
        # Check for MCAP file (sometimes metadata is embedded/skipped if crashed)
        mcap_path = os.path.join(full_bag_path, f"{bag_name}_0.mcap")
        if os.path.exists(mcap_path):
            print(f"   (But .mcap file exists at: {mcap_path})")


# Main Entry Point
if __name__ == "__main__":
    main()