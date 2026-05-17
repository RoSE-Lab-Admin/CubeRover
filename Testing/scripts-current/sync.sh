#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "${SCRIPT_DIR}/config.sh"

# Copy the CONTENTS of the local folder
LOCAL_WS_DIR="${SCRIPT_DIR}/../../"

echo "========================================"
echo " Preparing remote directory..."
echo "========================================"

# Tell the Pi to create the nested folders via the SSH alias
${SSH_CMD} ${PI_ALIAS} "mkdir -p ${REMOTE_WS_DIR}"

echo "========================================"
echo " Syncing workspace to Raspberry Pi..."
echo "========================================"

# 3. Add the trailing slash to the destination as well
rsync -avz --delete \
    --exclude '.*/' \
    --exclude 'build/' \
    --exclude 'install/' \
    --exclude 'log/' \
    --exclude 'Testing/' \
    "${LOCAL_WS_DIR}" "${PI_ALIAS}:${REMOTE_WS_DIR}/"