#!/usr/bin/env python3
import subprocess
import time
import signal
import os
import sys
from pathlib import Path


# --- CONFIGURATION ---
# 1. Cleaner path resolution
SCRIPT_LOCATION = Path(__file__).parent.resolve()
DEFAULT_BAG_DIR = SCRIPT_LOCATION / "ros_bags"

# 2. Wrap environment variables in Path objects
# We use str() on DEFAULT_BAG_DIR because get() expects the default to be the same type
ENV_BAG_DIR = os.environ.get("BAG_DIR")
BAG_DIR = Path(ENV_BAG_DIR) if ENV_BAG_DIR else DEFAULT_BAG_DIR

DURATION = int(os.environ.get("DURATION", 15))
STORAGE_ID = "mcap"


# --- HELPER FUNCTIONS ---
def get_valid_bag_dir(requested: Path, default: Path) -> Path:
    """
    Tries to create the requested directory. 
    Returns the requested Path if successful/writable, otherwise returns default Path.
    """
    try:
        # 1. Try to create requested directory (parents=True is like mkdir -p)
        requested.mkdir(parents=True, exist_ok=True)

        # 2. Check permissions (Pathlib objects work directly with os.access)
        if not os.access(requested, os.W_OK):
            raise PermissionError(f"Directory {requested} is not writable.")

        return requested

    except (OSError, PermissionError) as e:
        print(f"⚠️  WARNING: Could not use requested directory '{requested}'")
        print(f"   Reason: {e}")
        print(f"   -> Falling back to default: {default}")

        # 3. Create default (We assume local script dir is always writable)
        default.mkdir(parents=True, exist_ok=True)
        return default


# --- MAIN ---
def main():
    # 1. Resolve Path
    final_bag_dir = get_valid_bag_dir(BAG_DIR, DEFAULT_BAG_DIR)
    
    # 2. Build Filename (Using the / operator)
    bag_name = time.strftime("recording_%Y-%m-%d_%H-%M-%S")
    full_bag_path = final_bag_dir / bag_name

    # Used by test_bag.py to get the path
    print(f"BAG_PATH={full_bag_path}")

    print(f"==========================================")
    print(f"   ROS 2 Python Recorder (Pathlib)")
    print(f"==========================================")
    print(f" Script Location: {SCRIPT_LOCATION}")
    print(f" Output Bag:      {full_bag_path}")
    print(f" Duration:        {DURATION}s")
    print(f"------------------------------------------")

    # 3. BUILD COMMAND
    # Note: subprocess requires string paths, so we use str(full_bag_path)
    cmd = [
        "ros2", "bag", "record",
        "-s", STORAGE_ID,
        "-a", 
        "-o", str(full_bag_path)
    ]

    # 4. START RECORDING
    print(f"🚀 Launching recorder...")
    process = subprocess.Popen(cmd, env=os.environ.copy())
    print(f"✅ Recorder started. PID: {process.pid}")

    # 5. WAIT LOOP
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

    # 6. SHUTDOWN SEQUENCE
    if process.poll() is None:
        print("Sending SIGINT (Ctrl+C)...")
        process.send_signal(signal.SIGINT)

        try:
            process.wait(timeout=5)
            print("✅ Process exited gracefully.")
        except subprocess.TimeoutExpired:
            print("⚠️  Process stuck! Forcing kill...")
            process.kill()

    # 7. VERIFY METADATA
    metadata_path = full_bag_path / "metadata.yaml"
    mcap_path = full_bag_path / f"{bag_name}_0.mcap"

    if not full_bag_path.exists():
         print(f"❌ FAILURE: Bag folder not created: {full_bag_path}")
         return

    if metadata_path.exists():
        print(f"🎉 SUCCESS: Metadata found at:\n   {metadata_path}")
    elif mcap_path.exists():
        print(f"❌ FAILURE: Metadata missing.")
        print(f"   (But .mcap file exists at: {mcap_path})")
    else:
        print("❌ FAILURE: No data found.")


# --- ENTRY POINT ---
if __name__ == "__main__":
    main()