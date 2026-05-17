#!/bin/bash
# We use an SSH alias defined in our local config file
export PI_ALIAS="entry_pi"
export REMOTE_WS_DIR="~/CubeRoverSync/CubeRover"

# Tell rsync and ssh to use our custom config file
export RSYNC_RSH="ssh -F ${SCRIPT_DIR}/ssh_config"
export SSH_CMD="ssh -F ${SCRIPT_DIR}/ssh_config"
