#!/usr/bin/env bash
# Usage:
#   ./scripts/build.sh            # rebuild the workspace
#   ./scripts/build.sh --rosdep   # also (re)install missing ROS deps first

set -e

cd "$(dirname "$0")/.."

source /opt/ros/jazzy/setup.bash

if [[ "$1" == "--rosdep" ]]; then
    rosdep install --from-paths src -y --ignore-src
fi

colcon build

echo ""
echo "Build complete. To use it in this shell, run:"
echo "    source install/setup.bash"
