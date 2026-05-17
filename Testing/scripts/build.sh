#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "${SCRIPT_DIR}/config.sh"

echo " Building workspace on Raspberry Pi..."

SCRIPT="
cd ${REMOTE_WS_DIR};
source /opt/ros/jazzy/setup.bash;
colcon build --symlink-install;
"

${SSH_CMD} -t ${PI_ALIAS} "${SCRIPT}"