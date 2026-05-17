#!/bin/bash

# Load variables
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "${SCRIPT_DIR}/config.sh"

echo "========================================"
echo " Building and launching on Raspberry Pi..."
echo "========================================"

SCRIPT="
cd ${REMOTE_WS_DIR};
colcon build --symlink-install;
source install/setup.bash;
ros2 launch roseybot_control hardware_startup.launch.py
"

ssh -t -l ${PI_USERNAME} ${PI_HOST} "${SCRIPT}"