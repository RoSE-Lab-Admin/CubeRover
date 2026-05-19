#!/bin/bash

# Get the script's directory so it can call the other scripts reliably
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Run the sync, build, and launch script
bash "${SCRIPT_DIR}/scripts/sync_build_launch.sh"