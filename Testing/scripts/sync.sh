#!/bin/bash

# Load variables
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "${SCRIPT_DIR}/config.sh"

LOCAL_WS_DIR="${SCRIPT_DIR}/../"

echo "========================================"
echo " Syncing workspace to Raspberry Pi..."
echo "========================================"

rsync -avz --delete \
    --exclude 'build/' \
    --exclude 'install/' \
    --exclude 'log/' \
    --exclude '.git/' \
    --exclude 'Testing/' \
    "${LOCAL_WS_DIR}" "${PI_USERNAME}@${PI_HOST}:${REMOTE_WS_DIR}/"