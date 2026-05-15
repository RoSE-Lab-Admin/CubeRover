#!/bin/bash

# Get the script's directory so it can call the other scripts reliably
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Run sync.sh, and if it succeeds (&&), run launch.sh
"${SCRIPT_DIR}/sync.sh" && "${SCRIPT_DIR}/launch.sh"