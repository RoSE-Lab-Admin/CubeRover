#!/bin/bash

# Load variables
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "${SCRIPT_DIR}/config.sh"

echo "========================================"
echo " Stopping ROS2 nodes on Raspberry Pi..."
echo "========================================"

# SSH in and kill ROS2 processes
# -2 sends SIGINT (like pressing Ctrl+C), giving nodes a chance to shut down cleanly
# || true prevents the script from throwing an error if no nodes are currently running
SCRIPT="
echo 'Sending shut down signal to ROS2 processes...';
pkill -2 -f 'ros2 launch' || true;
pkill -2 -f 'hardware_startup.launch.py' || true;
"

ssh -t -l ${PI_USERNAME} ${PI_HOST} "${SCRIPT}"