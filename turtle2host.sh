#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <script_name.py>"
    exit 1
fi

SCRIPT_NAME="$1"
REMOTE_USER="ubuntu"
REMOTE_HOST="ubuntu"
REMOTE_PATH="~/ros2_ws/src/predprey/predprey/"
LOCAL_DEST="~/predprey"

scp "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}/${SCRIPT_NAME}" "${LOCAL_DEST}/"