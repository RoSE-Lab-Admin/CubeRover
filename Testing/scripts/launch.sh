#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "${SCRIPT_DIR}/config.sh"

echo " Launching hardware on Raspberry Pi..."

SCRIPT="
cd ${REMOTE_WS_DIR};
source /opt/ros/jazzy/setup.bash;
source install/setup.bash; 
ros2 launch roseybot_control hardware_startup.launch.py
"

${SSH_CMD} -t ${PI_ALIAS} "${SCRIPT}"