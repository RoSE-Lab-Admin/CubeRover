#!/bin/bash

# Get the script's directory so it can call the other scripts reliably
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo "========================================"
echo " Starting Full Deployment Pipeline..."
echo "========================================"

# Run sync.sh, then build.sh, then launch.sh
bash "${SCRIPT_DIR}/sync.sh" && bash "${SCRIPT_DIR}/build.sh" && bash "${SCRIPT_DIR}/launch.sh"