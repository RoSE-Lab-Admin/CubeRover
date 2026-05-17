#!/bin/bash

# Load variables
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "${SCRIPT_DIR}/config.sh"

echo "========================================"
echo " Scrubbing build/ and install/ on Pi..."
echo "========================================"

# SSH in and recursively remove the generated folders
SCRIPT="
cd ${REMOTE_WS_DIR};
rm -rf build/ install/ log/;
echo 'Workspace cleaned!';
"

${SSH_CMD} -t -l ${PI_USERNAME} ${PI_HOST} "${SCRIPT}"