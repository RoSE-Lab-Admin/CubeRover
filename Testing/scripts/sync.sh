#!/bin/bash

# Load variables
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "${SCRIPT_DIR}/config.sh"

# Use realpath to resolve the "../" into the actual, clean folder path without a trailing slash
LOCAL_WS_DIR="${SCRIPT_DIR}/../../"

echo "========================================"
echo " Preparing remote directory..."
echo "========================================"

# Tell the Pi to create the nested folders via the SSH alias
ssh ${PI_ALIAS} "mkdir -p ${REMOTE_WS_DIR}"

echo "========================================"
echo " Syncing workspace to Raspberry Pi..."
echo "========================================"

# Note: Remove the trailing slash from ${LOCAL_WS_DIR}
rsync -avz --delete \
    --exclude '.*/' \
    --exclude 'build/' \
    --exclude 'install/' \
    --exclude 'log/' \
    --exclude 'Testing/' \
    "${LOCAL_WS_DIR}" "${PI_USERNAME}@${PI_HOST}:${REMOTE_WS_DIR}"