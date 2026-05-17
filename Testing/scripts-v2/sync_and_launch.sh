#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Just run them sequence. The first one asks for a password; the second uses the persistent tunnel automatically.
bash "${SCRIPT_DIR}/sync.sh" && bash "${SCRIPT_DIR}/launch.sh"